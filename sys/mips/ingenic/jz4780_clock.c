/*-
 * Copyright 2015 Alexander Kabaev <kan@FreeBSD.org>
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

/*
 * Ingenic JZ4780 CGU driver.
 *
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/conf.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/resource.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <dev/fdt/fdt_common.h>
#include <dev/fdt/fdt_clock.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>


struct jz4780_clock_softc {
	device_t				dev;
	struct {
		struct jz4780_clock_function	*func;
		struct jz4780_clock_pkg_pin	*ppin;
		boolean_t			pud_ctrl;
	}					soc;
	struct resource				*res[6];
	struct mtx				mtx;
};

static struct resource_spec jz4780_clock_spec[] = {
	{ SYS_RES_MEMORY, 0, RF_ACTIVE },
	{ -1, 0 }
};

#define	CGU_LOCK(sc)		mtx_lock(&(sc)->mtx)
#define	CGU_UNLOCK(sc)		mtx_unlock(&(sc)->mtx)
#define	CGU_LOCK_INIT(sc)	\
    mtx_init(&(sc)->mtx, device_get_nameunit((sc)->dev),	\
    "jz4780-cgu", MTX_DEF)
#define	CGU_LOCK_DESTROY(sc)	mtx_destroy(&(sc)->mtx);

static int
jz4780_clock_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "ingenic,jz4780-cgu"))
		return (ENXIO);

	device_set_desc(dev, "Ingenic jz4780 CGU");

	return (BUS_PROBE_DEFAULT);
}

static int
jz4780_clock_attach(device_t dev)
{
	struct jz4780_clock_softc *sc = device_get_softc(dev);

	sc->dev = dev;

	if (bus_alloc_resources(dev, jz4780_clock_spec, sc->res)) {
		device_printf(dev, "could not allocate resources for device\n");
		return (ENXIO);
	}

	CGU_LOCK_INIT(sc);

	fdt_clock_register_provider(dev);

	return (0);
}

static int
jz4780_clock_detach(device_t dev)
{
	struct jz4780_clock_softc *sc = device_get_softc(dev);

	fdt_clock_unregister_provider(dev);

	bus_release_resources(dev, jz4780_clock_spec, sc->res);
	CGU_LOCK_DESTROY(sc);

	return (0);
}

static int
jz4780_clock_enable(device_t dev, int index)
{

	return (0);
}

static int
jz4780_clock_disable(device_t dev, int index)
{

	return (0);
}

static int
jz4780_clock_get_info(device_t dev, int index, struct fdt_clock_info *info)
{

	return (0);
}

static device_method_t jz4780_clock_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		jz4780_clock_probe),
	DEVMETHOD(device_attach,	jz4780_clock_attach),
	DEVMETHOD(device_detach,	jz4780_clock_detach),

	/* fdt_clock interface */
	DEVMETHOD(fdt_clock_enable,	jz4780_clock_enable),
	DEVMETHOD(fdt_clock_disable,	jz4780_clock_disable),
	DEVMETHOD(fdt_clock_get_info,	jz4780_clock_get_info),

	DEVMETHOD_END
};

static driver_t jz4780_clock_driver = {
	"cgu",
	jz4780_clock_methods,
	sizeof(struct jz4780_clock_softc),
};

static devclass_t jz4780_clock_devclass;

EARLY_DRIVER_MODULE(jz4780_clock, simplebus, jz4780_clock_driver,
    jz4780_clock_devclass, 0, 0,  BUS_PASS_CPU + BUS_PASS_ORDER_LATE);
