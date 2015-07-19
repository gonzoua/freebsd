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

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/clock.h>
#include <sys/time.h>
#include <sys/bus.h>
#include <sys/resource.h>
#include <sys/rman.h>
#include <sys/types.h>
#include <sys/systm.h>

#include <dev/iicbus/iicbus.h>
#include <dev/iicbus/iiconf.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/videomode/videomode.h>
#include <dev/videomode/edidvar.h>

#include <arm/freescale/imx/imx6_hdmi.h>

#include "iicbus_if.h"

#define	EDID_LENGTH		0x80

struct hdmi_edid_softc {
	device_t		sc_dev;
	uint32_t		sc_addr;
	int			sc_current_page;
	uint8_t			*sc_edid;
	uint32_t		sc_edid_len;
};

struct hdmi_edid_softc *hdmi_edid_sc = NULL;

int
hdmi_edid_read(void)
{
	struct hdmi_edid_softc *sc;
	uint8_t edid[1024];
	int result, i;
	uint8_t addr = 0;
	struct iic_msg msg[] = {
		{ 0, IIC_M_WR, 1, &addr },
		{ 0, IIC_M_RD, EDID_LENGTH, edid },
	};

	if (!hdmi_edid_sc)
		return (ENXIO);
	sc = hdmi_edid_sc;
	msg[0].slave = sc->sc_addr;
	msg[1].slave = sc->sc_addr;

	result =  iicbus_transfer(sc->sc_dev, msg, 2);
	if (result)
		printf("hdmi_edid_read failed: %d\n", result);
#if 0
	for (i = 0; i < EDID_LENGTH; i++) {
		printf("%02x ", edid[i]);
		if ((i % 0x10) == 0xf) {
			printf("\n");
		}
	}
	printf("\n");
#endif

	struct edid_info ei;
	edid_parse(edid, &ei);
	edid_print(&ei);
	return (result);
}

static int
hdmi_edid_probe(device_t dev)
{

	if (!ofw_bus_is_compatible(dev, "fsl,imx6-hdmi-i2c"))
		return (ENXIO);

	return (BUS_PROBE_DEFAULT);
}

static int
hdmi_edid_attach(device_t dev)
{
	struct hdmi_edid_softc *sc;

	sc = device_get_softc(dev);

	sc->sc_dev = dev;
	sc->sc_addr = iicbus_get_addr(dev) << 1;
	sc->sc_edid = malloc(EDID_LENGTH, M_DEVBUF, M_WAITOK | M_ZERO);
	sc->sc_edid_len = EDID_LENGTH;

	device_set_desc(dev, "iMX6 HDMI I2C interface");
	hdmi_edid_sc = sc;

	return (0);
}

static int
hdmi_edid_detach(device_t dev)
{

	/* XXX: Do not let unload drive */
	return (EBUSY);
}

static device_method_t hdmi_edid_methods[] = {
	DEVMETHOD(device_probe,		hdmi_edid_probe),
	DEVMETHOD(device_attach,	hdmi_edid_attach),
	DEVMETHOD(device_detach,	hdmi_edid_detach),
	{0, 0},
};

static driver_t hdmi_edid_driver = {
	"edid",
	hdmi_edid_methods,
	sizeof(struct hdmi_edid_softc),
};

static devclass_t hdmi_edid_devclass;

DRIVER_MODULE(hdmi_edid, iicbus, hdmi_edid_driver, hdmi_edid_devclass, 0, 0);
MODULE_VERSION(hdmi_edid, 1);
MODULE_DEPEND(hdmi_edid, iicbus, 1, 1, 1);
