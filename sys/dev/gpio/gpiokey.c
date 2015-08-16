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
__FBSDID("$FreeBSD: head/sys/dev/gpio/gpiokey.c 283360 2015-05-24 07:45:42Z ganbold $");

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

#include "gpiobus_if.h"

struct gpiokey_softc 
{
	device_t	sc_dev;
	device_t	sc_busdev;
	int		sc_irq_rid;
	struct resource	*sc_irq_res;
	void		*sc_intr_hl;
	struct mtx	sc_mtx;
};

#define GPIOKEY_LOCK(_sc)		mtx_lock(&(_sc)->sc_mtx)
#define	GPIOKEY_UNLOCK(_sc)		mtx_unlock(&(_sc)->sc_mtx)
#define GPIOKEY_LOCK_INIT(_sc) \
	mtx_init(&_sc->sc_mtx, device_get_nameunit(_sc->sc_dev), \
	    "gpiokey", MTX_DEF)
#define GPIOKEY_LOCK_DESTROY(_sc)	mtx_destroy(&_sc->sc_mtx);

/* Single key device */
static int gpiokey_probe(device_t);
static int gpiokey_attach(device_t);
static int gpiokey_detach(device_t);

static void
gpiokey_intr(void *arg)
{
	struct gpiokey_softc *sc;
	int error;
	int val;

	sc = arg;

	GPIOKEY_LOCK(sc);
	error = GPIOBUS_ACQUIRE_BUS(sc->sc_busdev, sc->sc_dev,
	    GPIOBUS_DONTWAIT);
	if (error != 0) {
		GPIOKEY_UNLOCK(sc);
		return;
	}
	GPIOBUS_PIN_GET(sc->sc_busdev, sc->sc_dev, 0, &val);
	GPIOBUS_RELEASE_BUS(sc->sc_busdev, sc->sc_dev);
	GPIOKEY_UNLOCK(sc);

	device_printf(sc->sc_dev, ": %d\n", val);
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
	phandle_t node;
	char *name;

	if ((node = ofw_bus_get_node(dev)) == -1)
		return (ENXIO);

	name = NULL;
	if (OF_getprop_alloc(node, "label", 1, (void **)&name) == -1)
		OF_getprop_alloc(node, "name", 1, (void **)&name);

	if (name != NULL) {
		device_set_desc_copy(dev, name);
		free(name, M_OFWPROP);
	}

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

	return (0);
}

static int
gpiokey_detach(device_t dev)
{
	struct gpiokey_softc *sc;

	sc = device_get_softc(dev);
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


