/*-
 * Copyright (c) 2015 Oleksandr Tymoshenko <gonzo@freebsd.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include "opt_platform.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/gpio.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/proc.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>

#include <dev/gpio/gpiobusvar.h>
#include <dev/gpio/gpiokeys.h>

#include "gpiobus_if.h"

#define	AUTOREPEAT_DELAY	250
#define	AUTOREPEAT_REPEAT	34

struct gpiokey_softc 
{
	device_t	sc_dev;
	device_t	sc_busdev;
	int		sc_irq_rid;
	struct resource	*sc_irq_res;
	void		*sc_intr_hl;
	struct mtx	sc_mtx;
	uint32_t	sc_keycode;
	int		sc_autorepeat;
	struct callout	sc_debounce_callout;
	struct callout	sc_repeat_callout;
	int		sc_repeat_delay;
	int		sc_repeat;
};

#define GPIOKEY_LOCK(_sc)		mtx_lock(&(_sc)->sc_mtx)
#define	GPIOKEY_UNLOCK(_sc)		mtx_unlock(&(_sc)->sc_mtx)
#define GPIOKEY_LOCK_INIT(_sc) \
	mtx_init(&_sc->sc_mtx, device_get_nameunit(_sc->sc_dev), \
	    "gpiokey", MTX_DEF)
#define GPIOKEY_LOCK_DESTROY(_sc)	mtx_destroy(&_sc->sc_mtx);

/* No key code */
#define GPIOKEY_NONE	0

#define	GPIOKEY_E0(k)	(SCAN_PREFIX_E0 | k)

/* Single key device */
static int gpiokey_probe(device_t);
static int gpiokey_attach(device_t);
static int gpiokey_detach(device_t);

static uint32_t
gpiokey_map_linux_code(uint32_t linux_code)
{
	switch (linux_code) {
		case 28: /* ENTER */
			return 28;
			break;
		case 105: /* LEFT */
			return GPIOKEY_E0(0x4b);
			break;
		case 106: /* RIGHT */
			return GPIOKEY_E0(0x4d);
			break;
		case 103: /* UP */
			return GPIOKEY_E0(0x48);
			break;
		case 108: /* DOWN */
			return GPIOKEY_E0(0x50);
			break;

		default:
			return GPIOKEY_NONE;
	}
}

static void
gpiokey_autorepeat(void *arg)
{
	struct gpiokey_softc *sc;

	sc = arg;

	if (sc->sc_keycode == GPIOKEY_NONE)
		return;

	gpiokeys_key_event(sc->sc_keycode, 1);

	callout_reset(&sc->sc_repeat_callout, sc->sc_repeat,
		    gpiokey_autorepeat, sc);
}

static void
gpiokey_debounced_intr(void *arg)
{
	struct gpiokey_softc *sc;
	int error;
	int val;

	sc = arg;

	if (sc->sc_keycode == GPIOKEY_NONE)
		return;

	error = GPIOBUS_ACQUIRE_BUS(sc->sc_busdev, sc->sc_dev,
	    GPIOBUS_DONTWAIT);
	if (error != 0)
		return;
	GPIOBUS_PIN_GET(sc->sc_busdev, sc->sc_dev, 0, &val);
	GPIOBUS_RELEASE_BUS(sc->sc_busdev, sc->sc_dev);
	if (val == 0) {
		gpiokeys_key_event(sc->sc_keycode, 1);
		if (sc->sc_autorepeat) {
			callout_reset(&sc->sc_repeat_callout, sc->sc_repeat_delay,
			    gpiokey_autorepeat, sc);
		}
	}
	else {
		if (sc->sc_autorepeat &&
		    callout_pending(&sc->sc_repeat_callout))
			callout_stop(&sc->sc_repeat_callout);
		gpiokeys_key_event(sc->sc_keycode, 0);
	}
}

static void
gpiokey_intr(void *arg)
{
	struct gpiokey_softc *sc;
	int debounce_ticks;

	sc = arg;

	GPIOKEY_LOCK(sc);
	debounce_ticks = hz*5/1000;
	if (debounce_ticks == 0)
		debounce_ticks = 1;
	if (!callout_pending(&sc->sc_debounce_callout))
		callout_reset(&sc->sc_debounce_callout, debounce_ticks,
		    gpiokey_debounced_intr, sc);
	GPIOKEY_UNLOCK(sc);
}


static void
gpiokey_identify(driver_t *driver, device_t bus)
{
	phandle_t child, leds, root;

	root = OF_finddevice("/");
	if (root == 0)
		return;
	for (leds = OF_child(root); leds != 0; leds = OF_peer(leds)) {
		if (!fdt_is_compatible_strict(leds, "gpio-keys"))
			continue;
		/* Traverse the 'gpio-leds' node and add its children. */
		for (child = OF_child(leds); child != 0; child = OF_peer(child)) {
			if (!OF_hasprop(child, "gpios"))
				continue;
			if (ofw_gpiobus_add_fdt_child(bus, driver->name, child) == NULL)
				continue;
		}
	}
}

static int
gpiokey_probe(device_t dev)
{
	struct gpiokey_softc *sc;
	phandle_t node;
	pcell_t prop;
	char *name;
	uint32_t code;

	sc = device_get_softc(dev);
	if ((node = ofw_bus_get_node(dev)) == -1)
		return (ENXIO);

	name = NULL;
	if (OF_getprop_alloc(node, "label", 1, (void **)&name) == -1)
		OF_getprop_alloc(node, "name", 1, (void **)&name);

	if (name != NULL) {
		device_set_desc_copy(dev, name);
		free(name, M_OFWPROP);
	}

	sc->sc_autorepeat = OF_hasprop(node, "autorepeat");
	if ((OF_getprop(node, "freebsd,code", &prop, sizeof(prop))) > 0)
		sc->sc_keycode = fdt32_to_cpu(prop);
	else if ((OF_getprop(node, "linux,code", &prop, sizeof(prop))) > 0) {
		code = fdt32_to_cpu(prop);
		sc->sc_keycode = gpiokey_map_linux_code(code);
		if (sc->sc_keycode == GPIOKEY_NONE)
			device_printf(dev, "failed to map linux,code value %d\n", code);
	}
	else
		device_printf(dev, "no key code property\n");

	return (0);
}

static int
gpiokey_attach(device_t dev)
{
	struct gpiokey_softc *sc;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;
	sc->sc_busdev = device_get_parent(dev);

	sc->sc_irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &sc->sc_irq_rid,
	    RF_ACTIVE);
	if (!sc->sc_irq_res) {
		device_printf(dev, "cannot allocate interrupt\n");
		return (ENXIO);
	}

	if (bus_setup_intr(dev, sc->sc_irq_res, INTR_TYPE_MISC | INTR_MPSAFE,
			NULL, gpiokey_intr, sc,
			&sc->sc_intr_hl) != 0) {
		bus_release_resource(dev, SYS_RES_IRQ, sc->sc_irq_rid,
		    sc->sc_irq_res);
		device_printf(dev, "Unable to setup the irq handler.\n");
		return (ENXIO);
	}

	GPIOKEY_LOCK_INIT(sc);

	sc->sc_repeat_delay = hz*AUTOREPEAT_DELAY/1000;
	if (sc->sc_repeat_delay == 0)
		sc->sc_repeat_delay = 1;

	sc->sc_repeat = hz*AUTOREPEAT_REPEAT/1000;
	if (sc->sc_repeat == 0)
		sc->sc_repeat = 1;

	callout_init_mtx(&sc->sc_debounce_callout, &sc->sc_mtx, 0);
	callout_init_mtx(&sc->sc_repeat_callout, &sc->sc_mtx, 0);

	return (0);
}

static int
gpiokey_detach(device_t dev)
{
	struct gpiokey_softc *sc;

	sc = device_get_softc(dev);

	if (sc->sc_intr_hl) {
		bus_teardown_intr(dev, sc->sc_irq_res, sc->sc_intr_hl);
		bus_release_resource(dev, SYS_RES_IRQ, sc->sc_irq_rid,
		    sc->sc_irq_res);
	}

	if (callout_pending(&sc->sc_repeat_callout))
		callout_stop(&sc->sc_repeat_callout);
	if (callout_pending(&sc->sc_debounce_callout))
		callout_stop(&sc->sc_debounce_callout);

	GPIOKEY_LOCK_DESTROY(sc);

	return (0);
}

static devclass_t gpiokey_devclass;

static device_method_t gpiokey_methods[] = {
	/* Device interface */
	DEVMETHOD(device_identify,	gpiokey_identify),

	DEVMETHOD(device_probe,		gpiokey_probe),
	DEVMETHOD(device_attach,	gpiokey_attach),
	DEVMETHOD(device_detach,	gpiokey_detach),

	{ 0, 0 }
};

static driver_t gpiokey_driver = {
	"gpiokey",
	gpiokey_methods,
	sizeof(struct gpiokey_softc),
};

DRIVER_MODULE(gpiokey, gpiobus, gpiokey_driver, gpiokey_devclass, 0, 0);
