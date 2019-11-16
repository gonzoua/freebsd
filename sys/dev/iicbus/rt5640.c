/*-
 * Copyright (c) 2019 Oleksandr Tymoshenko <gonzo@FreeBSD.org>
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * This code base on isl12xx.c
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

/*
 * Driver for Realtek rt5640 audio codec
 */

#include "opt_platform.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/clock.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>

#ifdef FDT
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#endif

#include <dev/iicbus/iiconf.h>
#include <dev/iicbus/iicbus.h>

#include "iicbus_if.h"

#define	MAX_BUFFER	16

struct rt5640_softc {
	device_t	dev;
	device_t	busdev;
	struct intr_config_hook 
			init_hook;
};

#ifdef FDT
static struct ofw_compat_data compat_data[] = {
	{"realtek,rt5640", 1},
	{NULL,           0},
};
#endif

static void	rt5640_init(void *arg);
static int	rt5640_probe(device_t dev);
static int	rt5640_attach(device_t dev);
static int	rt5640_detach(device_t dev);

static int	rt5640_writeto(device_t slavedev, uint8_t regaddr, 
		    void *buffer, uint16_t buflen, int waithow);

static int 
rt5640_writeto(device_t slavedev, uint8_t regaddr, void *buffer,
    uint16_t buflen, int waithow)
{
	struct iic_msg	msgs;
	uint8_t		slaveaddr;
	uint8_t		newbuf[MAX_BUFFER];

	if (buflen > MAX_BUFFER - 1)
		return (EINVAL);

	slaveaddr = iicbus_get_addr(slavedev);

	newbuf[0] = regaddr;
	memcpy(newbuf + 1, buffer, buflen);
	msgs.slave = slaveaddr;
	msgs.flags = IIC_M_WR;
	msgs.len   = 1 + buflen;
	msgs.buf   = newbuf;

	return (iicbus_transfer_excl(slavedev, &msgs, 1, waithow));
}

static inline int
rt5640_read2(struct rt5640_softc *sc, uint8_t reg, uint16_t *data) 
{

	return (iicdev_readfrom(sc->dev, reg, data, 1, IIC_WAIT));
}

static inline int
rt5640_write2(struct rt5640_softc *sc, uint8_t reg, uint16_t val) 
{

	return (rt5640_writeto(sc->dev, reg, &val, 1, IIC_WAIT));
}

static void
rt5640_init(void *arg)
{
	struct rt5640_softc	*sc;
	
	sc = (struct rt5640_softc*)arg;
	config_intrhook_disestablish(&sc->init_hook);

	return;
}

static int
rt5640_probe(device_t dev)
{

#ifdef FDT
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data != 0) {
		device_set_desc(dev, "Realtek RT5640");
		return (BUS_PROBE_DEFAULT);
	}
#endif
	return (ENXIO);
}

static int
rt5640_attach(device_t dev)
{
	struct rt5640_softc	*sc;
	
	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->busdev = device_get_parent(sc->dev);

	/*
	 * Wait until i2c is ready to set up the chip
	 */
	sc->init_hook.ich_func = rt5640_init;
	sc->init_hook.ich_arg = sc;
	if (config_intrhook_establish(&sc->init_hook) != 0)
		return (ENOMEM);

	return (0);
}

static int
rt5640_detach(device_t dev)
{

	return (0);
}

static device_method_t rt5640_methods[] = {
        /* device_if methods */
	DEVMETHOD(device_probe,		rt5640_probe),
	DEVMETHOD(device_attach,	rt5640_attach),
	DEVMETHOD(device_detach,	rt5640_detach),

	DEVMETHOD_END,
};

static driver_t rt5640_driver = {
	"rt5640",
	rt5640_methods,
	sizeof(struct rt5640_softc),
};
static devclass_t rt5640_devclass;

DRIVER_MODULE(rt5640, iicbus, rt5640_driver, rt5640_devclass, NULL, NULL);
MODULE_VERSION(rt5640, 1);
MODULE_DEPEND(rt5640, iicbus, IICBUS_MINVER, IICBUS_PREFVER, IICBUS_MAXVER);
IICBUS_FDT_PNP_INFO(compat_data);
