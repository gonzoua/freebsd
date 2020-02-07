/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2020 Oleksandr Tymoshenko <gonzo@FreeBSD.org>
 * Copyright (c) 2018 Jared McNeill <jmcneill@invisible.ca>
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

#include "syscon_if.h"

#include "opt_snd.h"
#include <dev/sound/pcm/sound.h>
#include <dev/sound/fdt/audio_dai.h>
#include "audio_dai_if.h"

static struct ofw_compat_data compat_data[] = {
	{ "allwinner,sun8i-a33-codec",	1},
	{ NULL,				0 }
};

static struct resource_spec sun8i_codec_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE | RF_SHAREABLE },
	{ -1, 0 }
};

struct sun8i_codec_softc {
	device_t	dev;
	struct resource	*res[2];
	struct mtx	mtx;
	clk_t		clk_gate;
	clk_t		clk_mod;
	struct sunxi_i2s_config	*cfg;
	void *		intrhand;
	/* pointers to playback/capture buffers */
	uint32_t	play_ptr;
	uint32_t	rec_ptr;
};

#define	I2S_LOCK(sc)		mtx_lock(&(sc)->mtx)
#define	I2S_UNLOCK(sc)		mtx_unlock(&(sc)->mtx)
#define	I2S_READ(sc, reg)	bus_read_4((sc)->res, (reg))
#define	I2S_WRITE(sc, reg, val)	bus_write_4((sc)->res, (reg), (val))
#define	I2S_TYPE(sc)		((sc)->cfg->type)

static int sun8i_codec_probe(device_t dev);
static int sun8i_codec_attach(device_t dev);
static int sun8i_codec_detach(device_t dev);

static int
sun8i_codec_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
		return (ENXIO);

	device_set_desc(dev, "Allwinner Codec");
	return (BUS_PROBE_DEFAULT);
}

static int
sun8i_codec_attach(device_t dev)
{
	struct sun8i_codec_softc *sc;
	int error;
	phandle_t node;

	sc = device_get_softc(dev);
	sc->dev = dev;

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	if (bus_alloc_resources(dev, sun8i_codec_spec, sc->res) != 0) {
		device_printf(dev, "cannot allocate resources for device\n");
		error = ENXIO;
		goto fail;
	}

	error = clk_get_by_ofw_name(dev, 0, "mod", &sc->clk_mod);
	if (error != 0) {
		device_printf(dev, "cannot get \"mod\" clock\n");
		goto fail;
	}

	error = clk_get_by_ofw_name(dev, 0, "bus", &sc->clk_gate);
	if (error != 0) {
		device_printf(dev, "cannot get \"bus\" clock\n");
		goto fail;
	}

	error = clk_enable(sc->clk_gate);
	if (error != 0) {
		device_printf(dev, "cannot enable \"bus\" clock\n");
		goto fail;
	}

	node = ofw_bus_get_node(dev);
	OF_device_register_xref(OF_xref_from_node(node), dev);

	return (0);

fail:
	sun8i_codec_detach(dev);
	return (error);
}

static int
sun8i_codec_detach(device_t dev)
{
	struct sun8i_codec_softc *i2s;

	i2s = device_get_softc(dev);

	if (i2s->clk_gate)
		clk_release(i2s->clk_gate);

	if (i2s->clk_mod)
		clk_release(i2s->clk_mod);

	if (i2s->intrhand != NULL)
		bus_teardown_intr(i2s->dev, i2s->res[1], i2s->intrhand);

	bus_release_resources(dev, sun8i_codec_spec, i2s->res);
	mtx_destroy(&i2s->mtx);

	return (0);
}

static int
sun8i_codec_dai_init(device_t dev, uint32_t format)
{
	struct sun8i_codec_softc *sc;
	int fmt, pol, clk;

	sc = device_get_softc(dev);

	fmt = AUDIO_DAI_FORMAT_FORMAT(format);
	pol = AUDIO_DAI_FORMAT_POLARITY(format);
	clk = AUDIO_DAI_FORMAT_CLOCK(format);

	return (0);
}

static int
sun8i_codec_dai_trigger(device_t dev, int go, int pcm_dir)
{
	struct sun8i_codec_softc 	*sc = device_get_softc(dev);

	if ((pcm_dir != PCMDIR_PLAY) && (pcm_dir != PCMDIR_REC))
		return (EINVAL);

	switch (go) {
	case PCMTRIG_START:
		device_printf(sc->dev, "TODO: implement me %s\n", __func__);
		break;

	case PCMTRIG_STOP:
	case PCMTRIG_ABORT:
		device_printf(sc->dev, "TODO: implement me %s\n", __func__);
		break;
	}

	return (0);
}

static int
sun8i_codec_dai_setup_mixer(device_t dev, device_t pcmdev)
{
	struct sun8i_codec_softc *sc;

	sc = device_get_softc(dev);
	device_printf(sc->dev, "TODO: implement me %s\n", __func__);

	return (0);
}


static device_method_t sun8i_codec_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		sun8i_codec_probe),
	DEVMETHOD(device_attach,	sun8i_codec_attach),
	DEVMETHOD(device_detach,	sun8i_codec_detach),

	DEVMETHOD(audio_dai_init,	sun8i_codec_dai_init),
	DEVMETHOD(audio_dai_setup_mixer,	sun8i_codec_dai_setup_mixer),
	DEVMETHOD(audio_dai_trigger,	sun8i_codec_dai_trigger),

	DEVMETHOD_END
};

static driver_t sun8i_codec_driver = {
	"sun8icodec",
	sun8i_codec_methods,
	sizeof(struct sun8i_codec_softc),
};

static devclass_t sun8i_codec_devclass;

DRIVER_MODULE(sun8i_codec, simplebus, sun8i_codec_driver, sun8i_codec_devclass, 0, 0);
SIMPLEBUS_PNP_INFO(compat_data);
