/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/resource.h>
#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/extres/clk/clk.h>
#include <dev/extres/hwreset/hwreset.h>
#include <dev/extres/syscon/syscon.h>

#include "syscon_if.h"

#include "opt_snd.h"
#include <dev/sound/pcm/sound.h>

#define	AUDIO_BUFFER_SIZE	48000 * 4

#define	I2S_TXCR	0x0000
#define		I2S_CSR_2		(0 << 15)
#define		I2S_CSR_4		(1 << 15)
#define		I2S_CSR_6		(2 << 15)
#define		I2S_CSR_8		(3 << 15)
#define		I2S_TXCR_IBM_NORMAL	(0 << 9)
#define		I2S_TXCR_VDW_16		(0xf << 0)
#define	I2S_RXCR	0x0004
#define		I2S_RXCR_IBM_NORMAL	(0 << 9)
#define		I2S_RXCR_VDW_16		(0xf << 0)
#define	I2S_CKR		0x0008
#define		I2S_CKR_MSS_MASK	(1 << 27)
#define		I2S_CKR_MSS_MASTER	(0 << 27)
#define		I2S_CKR_MSS_SLAVE	(1 << 27)
#define		I2S_CKR_CKP		(1 << 26)
#define		I2S_CKR_MDIV(n)		((n) << 16)
#define		I2S_CKR_MDIV_MASK	(0xff << 16)
#define		I2S_CKR_RSD(n)		((n) << 8)
#define		I2S_CKR_RSD_MASK	(0xff << 8)
#define		I2S_CKR_TSD(n)		((n) << 0)
#define		I2S_CKR_TSD_MASK	(0xff << 0)
#define	I2S_TXFIFOLR	0x000c
#define	I2S_DMACR	0x0010
#define		I2S_DMACR_RDE_ENABLE	(1 << 24)
#define		I2S_DMACR_RDL(n)	((n) << 16)
#define		I2S_DMACR_TDE_ENABLE	(1 << 8)
#define		I2S_DMACR_TDL(n)	((n) << 0)
#define	I2S_INTCR	0x0014
#define		I2S_INTCR_TXUIC		(1 << 2)
#define	I2S_INTSR	0x0018
#define		I2S_INTSR_TXUI		(1 << 1)
#define		I2S_INTSR_TXEI		(1 << 0)
#define	I2S_XFER	0x001c
#define		I2S_XFER_RXS_START	(1 << 1)
#define		I2S_XFER_TXS_START	(1 << 0)
#define	I2S_CLR		0x0020
#define	I2S_TXDR	0x0024
#define	I2S_RXDR	0x0028
#define	I2S_RXFIFOLR	0x002c

/* syscon */
#define	I2S_IO_DIRECTION_MASK		(7)
#define	I2S_IO_8CH_OUT_2CH_IN		(0)
#define	I2S_IO_6CH_OUT_4CH_IN		(4)
#define	I2S_IO_4CH_OUT_6CH_IN		(6)
#define	I2S_IO_2CH_OUT_8CH_IN		(7)

#define DIV_ROUND_CLOSEST(n,d)  (((n) + (d) / 2) / (d))

#define	AUDIO_RATE	48000

static struct ofw_compat_data compat_data[] = {
	{ "rockchip,rk3399-i2s",		1 },
	{ NULL,					0 }
};

static struct resource_spec rk_i2s_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE | RF_SHAREABLE },
	{ -1, 0 }
};

struct rk_i2s_info {
	device_t	dev;
	struct resource	*res[2];
	struct mtx	mtx;
	clk_t		clk;
	clk_t		hclk;
	void *		intrhand;
	uint32_t	bclk_fs;
	struct syscon	*grf;
	struct pcm_channel 	*pcm;		/* PCM channel */
	struct snd_dbuf		*buf; 		/* PCM buffer */
	/* Pointer to first unsubmitted sample */
	uint32_t		unsubmittedptr;
};

#define	RK_I2S_LOCK(sc)			mtx_lock(&(sc)->mtx)
#define	RK_I2S_UNLOCK(sc)		mtx_unlock(&(sc)->mtx)
#define	RK_I2S_READ_4(sc, reg)		bus_read_4((sc)->res[0], (reg))
#define	RK_I2S_WRITE_4(sc, reg, val)	bus_write_4((sc)->res[0], (reg), (val))

static int rk_i2s_probe(device_t dev);
static int rk_i2s_attach(device_t dev);
static int rk_i2s_detach(device_t dev);
static void rk_i2s_intr(void *arg);

static uint32_t
rk_i2s_chan_setblocksize(kobj_t obj, void *data, uint32_t blocksz)
{

	return (blocksz);
}

static int
rk_i2s_chan_setformat(kobj_t obj, void *data, uint32_t format)
{

	/* TODO: implement */
	return (0);
}

static uint32_t
rk_i2s_chan_setspeed(kobj_t obj, void *data, uint32_t speed)
{

	/* TODO: implement */
	return (AUDIO_RATE);
}

static uint32_t
rk_i2s_chan_getptr(kobj_t obj, void *data)
{
	struct rk_i2s_info 	*i2s = data;
	uint32_t r;

	RK_I2S_LOCK(i2s);
	r = i2s->unsubmittedptr;
	RK_I2S_UNLOCK(i2s);

	return (r);

}

static void *
rk_i2s_chan_init(kobj_t obj, void *devinfo, struct snd_dbuf *b,
	struct pcm_channel *c, int dir)
{
	struct rk_i2s_info 	*i2s = devinfo;
	device_t		dev;
	void			*buffer;

	i2s->pcm = c;
	i2s->buf = b;
	dev = i2s->dev;

	buffer = malloc(AUDIO_BUFFER_SIZE, M_DEVBUF, M_WAITOK | M_ZERO);

	if (sndbuf_setup(b, buffer, AUDIO_BUFFER_SIZE) != 0) {
		device_printf(i2s->dev, "sndbuf_setup failed\n");
		free(buffer, M_DEVBUF);
		return NULL;
	}

	return (i2s);
}

static int
rk_i2s_chan_trigger(kobj_t obj, void *data, int go)
{
	struct rk_i2s_info 	*i2s = data;
	uint32_t val;

	switch (go) {
	case PCMTRIG_START:
		RK_I2S_WRITE_4(i2s, I2S_INTCR, 0x01f00f1);
		val = I2S_XFER_TXS_START | I2S_XFER_RXS_START;
		RK_I2S_WRITE_4(i2s, I2S_XFER, val);
		return (0);

	case PCMTRIG_STOP:
	case PCMTRIG_ABORT:
			RK_I2S_WRITE_4(i2s, I2S_XFER, 0);
			RK_I2S_WRITE_4(i2s, I2S_INTCR, 0x01f000f0);

		RK_I2S_LOCK(i2s);
		i2s->unsubmittedptr = 0;
		sndbuf_reset(i2s->buf);
		RK_I2S_UNLOCK(i2s);
		return (0);
	}

	return (0);
}

static int
rk_i2s_chan_free(kobj_t obj, void *data)
{

	/* TODO: implement */
	return (0);
}

static uint32_t sc_fmt[] = {
	SND_FORMAT(AFMT_S16_LE, 2, 0),
	0
};
static struct pcmchan_caps rk_i2s_caps = {48000, 48000, sc_fmt, 0};

static struct pcmchan_caps *
rk_i2s_chan_getcaps(kobj_t obj, void *data)
{
	return (&rk_i2s_caps);
}

static kobj_method_t rk_i2s_chan_methods[] = {
	KOBJMETHOD(channel_init, 	rk_i2s_chan_init),
	KOBJMETHOD(channel_free, 	rk_i2s_chan_free),
	KOBJMETHOD(channel_setformat, 	rk_i2s_chan_setformat),
	KOBJMETHOD(channel_setspeed, 	rk_i2s_chan_setspeed),
	KOBJMETHOD(channel_setblocksize,rk_i2s_chan_setblocksize),
	KOBJMETHOD(channel_trigger,	rk_i2s_chan_trigger),
	KOBJMETHOD(channel_getptr,	rk_i2s_chan_getptr),
	KOBJMETHOD(channel_getcaps,	rk_i2s_chan_getcaps),
	KOBJMETHOD_END
};
CHANNEL_DECLARE(rk_i2s_chan);

static int
rk_i2s_init(struct rk_i2s_info *sc)
{
	uint32_t val;
	uint64_t clk_freq;
	uint64_t bus_clk_freq;
	int error;
	uint32_t bus_clock_div, lr_clock_div;
	clk_t parent;

	// to trigger re-parenting
	clk_set_freq(sc->clk, 256 * AUDIO_RATE, 0);

	if (clk_get_parent(sc->clk, &parent) == 0) {
		error = clk_set_freq(parent, 256 * AUDIO_RATE, CLK_SET_ROUND_DOWN);

		if (error != 0) {
			device_printf(sc->dev, "cannot set freq for i2s_clk clock\n");
		}

	}
	else {
		device_printf(sc->dev, "failed to get parent!!!!\n");
	}

	error = clk_enable(sc->clk);
	if (error != 0) {
		device_printf(sc->dev, "cannot enable i2s_clk clock\n");
		return (ENXIO);
	}

	RK_I2S_WRITE_4(sc, I2S_TXCR, 0x00071f1f);
#if 0
	RK_I2S_WRITE_4(sc, I2S_DMACR, 0x001f0000);
#endif
	RK_I2S_WRITE_4(sc, I2S_INTCR, 0x01f000f0);

	/* Set format */
	val = RK_I2S_READ_4(sc, I2S_CKR);

	val &= ~(I2S_CKR_MSS_MASK);
	val |= I2S_CKR_MSS_MASTER;

	// data on negedge
	// val &= ~I2S_CKR_CKP;
	// val |= (1 << 24);
	// use only TX lrclk
	val |= (1 << 28);

	error = clk_get_freq(sc->clk, &clk_freq);
	if (error != 0) {
		device_printf(sc->dev, "failed to get clk frequency: err=%d\n", error);
		return (error);
	}
	bus_clk_freq = sc->bclk_fs * AUDIO_RATE;
	bus_clock_div = DIV_ROUND_CLOSEST(clk_freq, bus_clk_freq);
	lr_clock_div = sc->bclk_fs;
	bus_clock_div = bus_clock_div - 1;
	lr_clock_div -= 1;
	device_printf(sc->dev, "clk_freq = %jd\n", clk_freq);
	device_printf(sc->dev, "bus_clock_div = %d, lr_clock_div = %d\n", bus_clock_div, lr_clock_div);

	val &= ~(I2S_CKR_MDIV_MASK | I2S_CKR_RSD_MASK | I2S_CKR_TSD_MASK);
	val |= I2S_CKR_MDIV(bus_clock_div);
	val |= I2S_CKR_RSD(lr_clock_div);
	val |= I2S_CKR_TSD(lr_clock_div);

	RK_I2S_WRITE_4(sc, I2S_CKR, val);

	val = I2S_TXCR_IBM_NORMAL | I2S_TXCR_VDW_16 | I2S_CSR_2;
	RK_I2S_WRITE_4(sc, I2S_TXCR, val);

	val = I2S_RXCR_IBM_NORMAL | I2S_RXCR_VDW_16 | I2S_CSR_2;
	RK_I2S_WRITE_4(sc, I2S_RXCR, val);

	if (sc->grf) {
		val = (0 << 11);
		val |= (I2S_IO_DIRECTION_MASK << 11) << 16;
		SYSCON_WRITE_4(sc->grf, 0xe220, val);

		// HACK: enable IO domain
		val = (1 << 1);
		val |= (1 << 1) << 16;
		SYSCON_WRITE_4(sc->grf, 0xe640, val);
	}

	val = I2S_DMACR_TDL(16);
	RK_I2S_WRITE_4(sc, I2S_DMACR, val);
	val |= I2S_DMACR_RDL(16);
	RK_I2S_WRITE_4(sc, I2S_DMACR, val);

#if 0
	val |= I2S_DMACR_TDE_ENABLE;
	RK_I2S_WRITE_4(sc, I2S_DMACR, val);
#endif
	RK_I2S_WRITE_4(sc, I2S_DMACR, 0x001f0000);

	RK_I2S_WRITE_4(sc, I2S_XFER, 0);

	return (0);
}

static int
rk_i2s_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
		return (ENXIO);

	device_set_desc(dev, "Rockchip I2S");
	return (BUS_PROBE_DEFAULT);
}

static int
rk_i2s_attach(device_t dev)
{
	struct rk_i2s_info *i2s;
	int error;
	phandle_t node;

	i2s = malloc(sizeof *i2s, M_DEVBUF, M_WAITOK | M_ZERO);
	if (i2s == NULL)
		return (ENOMEM);

	i2s->dev = dev;

	mtx_init(&i2s->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	if (bus_alloc_resources(dev, rk_i2s_spec, i2s->res) != 0) {
		device_printf(dev, "cannot allocate resources for device\n");
		error = ENXIO;
		goto fail;
	}

	if (bus_setup_intr(dev, i2s->res[1],
	    INTR_TYPE_MISC | INTR_MPSAFE, NULL, rk_i2s_intr, i2s,
	    &i2s->intrhand)) {
		bus_release_resources(dev, rk_i2s_spec, i2s->res);
		device_printf(dev, "cannot setup interrupt handler\n");
		return (ENXIO);
	}

	/* Activate the module clock. */
	error = clk_get_by_ofw_name(dev, 0, "i2s_hclk", &i2s->hclk);
	if (error != 0) {
		device_printf(dev, "cannot get i2s_hclk clock\n");
		goto fail;
	}
	error = clk_get_by_ofw_name(dev, 0, "i2s_clk", &i2s->clk);
	if (error != 0) {
		device_printf(dev, "cannot get i2s_clk clock\n");
		goto fail;
	}

	error = clk_enable(i2s->hclk);
	if (error != 0) {
		device_printf(dev, "cannot enable i2s_hclk clock\n");
		goto fail;
	}

	#if 0
	error = clk_enable(i2s->clk);
	if (error != 0) {
		device_printf(dev, "cannot enable i2s_clk clock\n");
		goto fail;
	}
	#endif

	node = ofw_bus_get_node(dev);
	if (OF_hasprop(node, "rockchip,grf") &&
	    syscon_get_by_ofw_property(dev, node,
	    "rockchip,grf", &i2s->grf) != 0) {
		device_printf(dev, "cannot get grf driver handle\n");
		return (ENXIO);
	}

	/* TODO: read from "rockchip,bclk-fs" */
	i2s->bclk_fs = 64;

	if (pcm_register(dev, i2s, 1, 0))
		goto fail;

	pcm_getbuffersize(dev, AUDIO_BUFFER_SIZE, AUDIO_BUFFER_SIZE,
	    AUDIO_BUFFER_SIZE);
	pcm_addchan(dev, PCMDIR_PLAY, &rk_i2s_chan_class, i2s);

	pcm_setstatus(dev, "at EXPERIMENT");

		rk_i2s_init(i2s);

	return (0);

fail:
	rk_i2s_detach(dev);
	return (error);
}

static int
rk_i2s_detach(device_t dev)
{
	struct rk_i2s_info *i2s;

	i2s = device_get_softc(dev);

	if (i2s->hclk != NULL)
		clk_release(i2s->hclk);
	if (i2s->clk)
		clk_release(i2s->clk);

	if (i2s->intrhand != NULL)
		bus_teardown_intr(i2s->dev, i2s->res[1], i2s->intrhand);

	bus_release_resources(dev, rk_i2s_spec, i2s->res);
	mtx_destroy(&i2s->mtx);

	return (0);
}

static void
rk_i2s_intr(void *arg)
{
	struct rk_i2s_info *i2s;
	uint32_t status, ctrl;
	uint32_t level;

	uint32_t val = 0x00;

	i2s = arg;

	RK_I2S_LOCK(i2s);
	status = RK_I2S_READ_4(i2s, I2S_INTSR);
	if (status & I2S_INTSR_TXUI) {
		ctrl = RK_I2S_READ_4(i2s, I2S_INTCR);
		ctrl |= I2S_INTCR_TXUIC;
		RK_I2S_WRITE_4(i2s, I2S_INTCR, ctrl);
	}

	if (status & (I2S_INTSR_TXUI | I2S_INTSR_TXEI)) {
		level = RK_I2S_READ_4(i2s, I2S_TXFIFOLR) & 0x1f;
		uint8_t *buf;
		uint32_t count, size, readyptr, written;
		count = sndbuf_getready(i2s->buf);
		size = sndbuf_getsize(i2s->buf);
		readyptr = sndbuf_getreadyptr(i2s->buf);

		buf = (uint8_t*)sndbuf_getbuf(i2s->buf);
		written = 0;
		for (; level < 31; level++) {
			val  = (buf[readyptr++ % size] << 0);
			val |= (buf[readyptr++ % size] << 8);
			val |= (buf[readyptr++ % size] << 16);
			val |= (buf[readyptr++ % size] << 24);
			written += 4;
			RK_I2S_WRITE_4(i2s, I2S_TXDR, val);
		}
		i2s->unsubmittedptr += written;
		i2s->unsubmittedptr %= size;
		RK_I2S_UNLOCK(i2s);
		chn_intr(i2s->pcm);
		RK_I2S_LOCK(i2s);
	}

	RK_I2S_UNLOCK(i2s);
}

static device_method_t rk_i2s_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		rk_i2s_probe),
	DEVMETHOD(device_attach,	rk_i2s_attach),
	DEVMETHOD(device_detach,	rk_i2s_detach),

	DEVMETHOD_END
};

static driver_t rk_i2s_driver = {
	"pcm",
	rk_i2s_methods,
	PCM_SOFTC_SIZE,
};

static devclass_t rk_i2s_devclass;

DRIVER_MODULE(rk_i2s, simplebus, rk_i2s_driver, rk_i2s_devclass, 0, 0);
SIMPLEBUS_PNP_INFO(compat_data);
