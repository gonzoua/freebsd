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

/*
 * HDMI core module
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/rman.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <machine/bus.h>

#include <arm/freescale/imx/imx6_hdmi.h>
#include <arm/freescale/imx/imx6_hdmi_regs.h>

#include <arm/freescale/imx/imx_iomuxvar.h>

struct hdmi_softc {
	device_t	dev;
	struct resource	*mem_res;
	int		mem_rid;
};

static struct hdmi_softc *hdmi_sc;

static inline uint8_t
RD1(struct hdmi_softc *sc, bus_size_t off)
{

	return (bus_read_1(sc->mem_res, off));
}

static inline void
WR1(struct hdmi_softc *sc, bus_size_t off, uint8_t val)
{

	bus_write_1(sc->mem_res, off, val);
}

uint8_t
hdmi_core_read_1(bus_size_t off)
{

	return RD1(hdmi_sc, off);
}

void
hdmi_core_write_1(bus_size_t off, uint8_t val)
{

	WR1(hdmi_sc, off, val);
}

static int
hdmi_detach(device_t dev)
{
	struct hdmi_softc *sc;

	sc = device_get_softc(dev);

	if (sc->mem_res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem_res);

	return (0);
}

static int
hdmi_attach(device_t dev)
{
	struct hdmi_softc *sc;
	int err;
	uint32_t gpr3;
	int ipu_id, disp_id;
	pcell_t prop;
	phandle_t node;

	sc = device_get_softc(dev);
	err = 0;

	/* Allocate memory resources. */
	sc->mem_rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->mem_rid,
	    RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "Cannot allocate memory resources\n");
		err = ENXIO;
		goto out;
	}

	sc->dev = dev;
	hdmi_sc = sc;

	err = 0;

	device_printf(sc->dev, "HDMI controller %02x:%02x:%02x:%02x\n", 
	    RD1(sc, HDMI_DESIGN_ID), RD1(sc, HDMI_REVISION_ID),
	    RD1(sc, HDMI_PRODUCT_ID0), RD1(sc, HDMI_PRODUCT_ID1));

	ipu_id = 0;
	disp_id = 0;
	node = ofw_bus_get_node(dev);

	if (OF_getencprop(node, "ipu_id", &prop, sizeof(prop)) != -1)
		ipu_id = prop;

	if (OF_getencprop(node, "disp_id", &prop, sizeof(prop)) != -1)
		disp_id = prop;

	gpr3 = imx_iomux_gpr_get(12);
	printf("GPR3 %08x -> ", gpr3);
	gpr3 &= ~0x0d;
	gpr3 |= ((ipu_id << 1) | disp_id) << 2;
	printf("%08x\n", gpr3);
	imx_iomux_gpr_set(12, gpr3);

	WR1(sc, HDMI_PHY_POL0, HDMI_PHY_HPD);
	WR1(sc, HDMI_IH_PHY_STAT0, HDMI_IH_PHY_STAT0_HPD);

out:

	if (err != 0)
		hdmi_detach(dev);

	return (err);
}

static int
hdmi_probe(device_t dev)
{

        if (ofw_bus_is_compatible(dev, "fsl,imx6dl-hdmi-core") == 0)
		return (ENXIO);

	device_set_desc(dev, "Freescale i.MX6 HDMI core");

	return (BUS_PROBE_DEFAULT);
}


static device_method_t hdmi_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,  hdmi_probe),
	DEVMETHOD(device_attach, hdmi_attach),
	DEVMETHOD(device_detach, hdmi_detach),

	DEVMETHOD_END
};

static driver_t hdmi_driver = {
	"hdmi",
	hdmi_methods,
	sizeof(struct hdmi_softc)
};

static devclass_t hdmi_devclass;

DRIVER_MODULE(hdmi, simplebus, hdmi_driver, hdmi_devclass, 0, 0);
