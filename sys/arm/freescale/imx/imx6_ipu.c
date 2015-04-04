/*-
 * Copyright 2015 Oleksandr Tymoshenko <gonzo@freebsd.org>
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

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/clock.h>
#include <sys/time.h>
#include <sys/bus.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/resource.h>
#include <sys/rman.h>
#include <sys/sysctl.h>

#include <machine/bus.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#define	IPU_LOCK(_sc)		mtx_lock(&(_sc)->sc_mtx)
#define	IPU_UNLOCK(_sc)		mtx_unlock(&(_sc)->sc_mtx)
#define	IPU_LOCK_INIT(_sc)	mtx_init(&(_sc)->sc_mtx, \
    device_get_nameunit(_sc->sc_dev), "ipu", MTX_DEF)
#define	IPU_LOCK_DESTROY(_sc)	mtx_destroy(&(_sc)->sc_mtx);

#define	IPU_READ4(_sc, reg)	bus_read_4((_sc)->sc_mem_res, reg);
#define	IPU_WRITE4(_sc, reg, value)	\
    bus_write_4((_sc)->sc_mem_res, reg, value);



struct ipu_softc {
	device_t		sc_dev;
	struct resource		*sc_mem_res;
	int			sc_mem_rid;
	struct resource		*sc_irq_res;
	int			sc_irq_rid;
	void			*sc_intr_hl;
	struct mtx		sc_mtx;
};

static int
ipu_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "fsl,imx6q-ipu"))
		return (ENXIO);

	device_set_desc(dev, "Freescale IPU");

	return (BUS_PROBE_DEFAULT);
}

static int
ipu_attach(device_t dev)
{
	struct ipu_softc *sc;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;

	sc->sc_mem_rid = 0;
	sc->sc_mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &sc->sc_mem_rid, RF_ACTIVE);
	if (!sc->sc_mem_res) {
		device_printf(dev, "cannot allocate memory window\n");
		return (ENXIO);
	}

	sc->sc_irq_rid = 0;
	sc->sc_irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ,
	    &sc->sc_irq_rid, RF_ACTIVE);
	if (!sc->sc_irq_res) {
		bus_release_resource(dev, SYS_RES_MEMORY,
		    sc->sc_mem_rid, sc->sc_mem_res);
		device_printf(dev, "cannot allocate interrupt\n");
		return (ENXIO);
	}

#if 0
	if (bus_setup_intr(dev, sc->sc_irq_res, INTR_TYPE_MISC | INTR_MPSAFE,
			NULL, ipu_intr, sc,
			&sc->sc_intr_hl) != 0) {
		bus_release_resource(dev, SYS_RES_IRQ, rid,
		    sc->sc_irq_res);
		bus_release_resource(dev, SYS_RES_MEMORY, rid,
		    sc->sc_mem_res);
		device_printf(dev, "Unable to setup the irq handler.\n");
		return (ENXIO);
	}
#endif

	IPU_LOCK_INIT(sc);

	return (0);
}

static int
ipu_detach(device_t dev)
{
	/* Do not let unload driver */
	return (EBUSY);
}

static device_method_t ipu_methods[] = {
	DEVMETHOD(device_probe,		ipu_probe),
	DEVMETHOD(device_attach,	ipu_attach),
	DEVMETHOD(device_detach,	ipu_detach),

	DEVMETHOD_END
};

static driver_t ipu_driver = {
	"ipu",
	ipu_methods,
	sizeof(struct ipu_softc),
};

static devclass_t ipu_devclass;

DRIVER_MODULE(ipu, simplebus, ipu_driver, ipu_devclass, 0, 0);
MODULE_VERSION(ipu, 1);
MODULE_DEPEND(ipu, simplebus, 1, 1, 1);
