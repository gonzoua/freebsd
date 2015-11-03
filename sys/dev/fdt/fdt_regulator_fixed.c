/*-
 * Copyright (c) 2015 Michal Meloun
 * All rights reserved.
 *
 * Redistribution and use in source and binullry forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binullry form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/gpio.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/malloc.h>
#include <sys/rman.h>

#include <machine/bus.h>
#include <machine/fdt.h>

#include <dev/fdt/fdt_regulator.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>


struct regulator_fixed_softc {
	device_t		dev;
	char			*name;
	int 			min_uvolt;
	int 			max_uvolt;
	int 			min_uamp;
	int 			max_uamp;
	int 			boot_on;
	int 			always_on;

	int			gpio_open_drain;
	int			enable_active_high;
	phandle_t		supply_pnode;
	struct gpiobus_pin	*gpio_en;
};

static struct ofw_compat_data compat_data[] = {
	{"regulator-fixed",		1},
	{NULL,				0},
};


static int
regulator_fixed_set(device_t provider, intptr_t id, int val)
{
	struct regulator_fixed_softc *sc;
	int rv;

	sc = device_get_softc(provider);
	if (sc->always_on && (val == 0))
		return (0);
	if (sc->gpio_en == NULL)
		return (0);

	if (sc->enable_active_high == 0)
		val = (val != 0) ? 0 : 1;

	rv = GPIO_PIN_SET(sc->gpio_en->dev, sc->gpio_en->pin, val);

	return (rv);
}

static int
regulator_fixed_parse(struct regulator_fixed_softc *sc, phandle_t node)
{
	int rv;

	rv = OF_getprop_alloc(node, "regulator-name", 1,
	    (void **)&sc->name);
	if (rv <= 0)
		sc->name = "Unnamed";

	rv = OF_getencprop(node, "regulator-min-microvolt", &sc->min_uvolt,
	    sizeof(sc->min_uvolt));
	if (rv <= 0)
		sc->min_uvolt = 0;

	rv = OF_getencprop(node, "regulator-max-microvolt", &sc->max_uvolt,
	    sizeof(sc->max_uvolt));
	if (rv <= 0)
		sc->max_uvolt = 0;

	rv = OF_getencprop(node, "regulator-min-microamp", &sc->min_uamp,
	    sizeof(sc->min_uamp));
	if (rv <= 0)
		sc->min_uamp = 0;

	rv = OF_getencprop(node, "regulator-max-microamp", &sc->max_uamp,
	    sizeof(sc->max_uamp));
	if (rv <= 0)
		sc->max_uamp = 0;

	if (OF_hasprop(node, "regulator-boot-on"))
		sc->boot_on = 1;
	if (OF_hasprop(node, "regulator-always-on"))
		sc->always_on = 1;

	if (OF_hasprop(node, "gpio-open-drain"))
		sc->gpio_open_drain = 1;

	if (OF_hasprop(node, "enable-active-high"))
		sc->enable_active_high = 1;
	rv = OF_getencprop(node, "vin-supply", &sc->supply_pnode,
	    sizeof(sc->supply_pnode));
	if (rv <= 0)
		sc->supply_pnode = 0;
	rv = ofw_gpiobus_parse_gpios(sc->dev, "gpio", &sc->gpio_en);
	if (rv != 1)
		sc->gpio_en = NULL;

	return (0);
}

static int
regulator_fixed_configure_gpio(struct regulator_fixed_softc *sc)
{
	int rv;
	uint32_t flags;
	uint32_t init_val;

	flags = GPIO_PIN_OUTPUT;
/* XXX Not now
	if (!sc->enable_active_high) {
		flags |= GPIO_PIN_INVIN;
		flags |= GPIO_PIN_INVOUT;
	}
*/
	if (sc->gpio_open_drain)
		flags |= GPIO_PIN_OPENDRAIN;
	init_val = sc->boot_on ? 1 : 0;

	rv = GPIO_PIN_SET(sc->gpio_en->dev, sc->gpio_en->pin, init_val);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot configure GPIO pin: %d\n",
		    sc->gpio_en->pin);
		return (rv);
	}

	rv = GPIO_PIN_SETFLAGS(sc->gpio_en->dev, sc->gpio_en->pin, flags);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot configure GPIO pin: %d\n",
		    sc->gpio_en->pin);
		return (rv);
	}

	return (0);
}

static int
regulator_fixed_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
		return (ENXIO);

	device_set_desc(dev, "fixed regulator");
	return (BUS_PROBE_DEFAULT);
}

static int
regulator_fixed_detach(device_t dev)
{

	/* This device is always present. */
	return (EBUSY);
}

static int
regulator_fixed_attach(device_t dev)
{
	struct regulator_fixed_softc * sc;

	sc = device_get_softc(dev);
	sc->dev = dev;
	fdt_regulator_register_provider(dev, 0);

	return (0);
}

static device_method_t regulator_fixed_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		regulator_fixed_probe),
	DEVMETHOD(device_attach,	regulator_fixed_attach),
	DEVMETHOD(device_detach,	regulator_fixed_detach),

	DEVMETHOD(fdt_regulator_set,	regulator_fixed_set),

	DEVMETHOD_END
};

static driver_t regulator_fixed_driver = {
	"regulator_fixed",
	regulator_fixed_methods,
	sizeof(struct regulator_fixed_softc),
};

static devclass_t regulator_fixed_devclass;
EARLY_DRIVER_MODULE(regulator_fixed, simplebus, regulator_fixed_driver,
   regulator_fixed_devclass, 0, 0, 70);
