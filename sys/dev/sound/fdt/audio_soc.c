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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include "opt_platform.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/clock.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/endian.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/sound/fdt/audio_dai.h>

struct audio_soc_link {
	device_t	cpu_dev;
	device_t	codec_dev;
	unsigned int	mclk_fs;
};

struct audio_soc_softc {
	device_t	dev;
	char		*name;
};

static struct ofw_compat_data compat_data[] = {
	{"simple-audio-card",	1},
	{NULL,			0},
};

static struct {
	const char *name;
	unsigned int fmt;
} ausoc_dai_formats[] = {
	{ "i2s",	AUDIO_DAI_FORMAT_I2S },
	{ "right_j",	AUDIO_DAI_FORMAT_RJ },
	{ "left_j",	AUDIO_DAI_FORMAT_LJ },
	{ "dsp_a",	AUDIO_DAI_FORMAT_DSPA },
	{ "dsp_b",	AUDIO_DAI_FORMAT_DSPB },
	{ "ac97",	AUDIO_DAI_FORMAT_AC97 },
	{ "pdm",	AUDIO_DAI_FORMAT_PDM },
};


static int	audio_soc_probe(device_t dev);
static int	audio_soc_attach(device_t dev);
static int	audio_soc_detach(device_t dev);

static int
audio_soc_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data != 0) {
		device_set_desc(dev, "simple-audio-card");
		return (BUS_PROBE_DEFAULT);
	}

	return (ENXIO);
}

static int
audio_soc_attach(device_t dev)
{
	struct audio_soc_softc *sc;
	char *name;
	phandle_t node;
	int i, ret;
	char tmp[32];
	unsigned int fmt, pol, clk;
	
	sc = device_get_softc(dev);
	sc->dev = dev;
	node = ofw_bus_get_node(dev);

	ret = OF_getprop_alloc(node, "name", (void **)&name);
	if (ret == -1)
		name = "SoC audio";

	sc->name = strdup(name, M_DEVBUF);

	if (ret != -1)
		OF_prop_free(name);

	ret = OF_getprop(node, "simple-audio-card,format", tmp, sizeof(tmp));
	if (ret == 0) {
		for (i = 0; i < nitems(ausoc_dai_formats); i++) {
			if (strcmp(tmp, ausoc_dai_formats[i].name) == 0) {
				fmt = ausoc_dai_formats[i].fmt;
				break;
			}
		}
		if (i == nitems(ausoc_dai_formats))
			return (EINVAL);
	} else
		fmt = AUDIO_DAI_FORMAT_I2S;

	bool frame_master = true;
	bool bitclock_master = true;

	if (frame_master) {
		clk = bitclock_master ?
		    AUDIO_DAI_CLOCK_CBM_CFM : AUDIO_DAI_CLOCK_CBS_CFM;
	} else {
		clk = bitclock_master ?
		    AUDIO_DAI_CLOCK_CBM_CFS : AUDIO_DAI_CLOCK_CBS_CFS;
	}

	bool bitclock_inversion = OF_hasprop(node, "simple-audio-card,bitclock-inversion");
	bool frame_inversion = OF_hasprop(node, "simple-audio-card,frame-inversion");
	if (bitclock_inversion) {
		pol = frame_inversion ?
		    AUDIO_DAI_POLARITY_IB_IF : AUDIO_DAI_POLARITY_IB_NF;
	} else {
		pol = frame_inversion ?
		    AUDIO_DAI_POLARITY_NB_IF : AUDIO_DAI_POLARITY_NB_NF;
	}

	return (0);
}

static int
audio_soc_detach(device_t dev)
{
	struct audio_soc_softc *sc;
	
	sc = device_get_softc(dev);
	if (sc->name)
		free(sc->name, M_DEVBUF);

	return (0);
}

static device_method_t audio_soc_methods[] = {
        /* device_if methods */
	DEVMETHOD(device_probe,		audio_soc_probe),
	DEVMETHOD(device_attach,	audio_soc_attach),
	DEVMETHOD(device_detach,	audio_soc_detach),

	DEVMETHOD_END,
};

static driver_t audio_soc_driver = {
	"ausoc",
	audio_soc_methods,
	sizeof(struct audio_soc_softc),
};
static devclass_t audio_soc_devclass;

DRIVER_MODULE(audio_soc, simplebus, audio_soc_driver, audio_soc_devclass, NULL, NULL);
MODULE_VERSION(audio_soc, 1);
