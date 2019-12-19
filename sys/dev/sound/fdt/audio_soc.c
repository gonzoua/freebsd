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
#include <dev/sound/pcm/sound.h>
#include "audio_dai_if.h"

#define	AUDIO_BUFFER_SIZE	48000 * 4

struct audio_soc_softc {
	/*
	 * pcm_register assumes that sc is snddev_info,
	 * so this has to be first structure member for "compatiblity"
	 */
	struct snddev_info	info;
	device_t		dev;
	char			*name;
	struct intr_config_hook init_hook;
	device_t		cpu_dev;
	device_t		codec_dev;
	unsigned int		mclk_fs;
	struct pcm_channel 	*pcm;		/* PCM channel */
	struct snd_dbuf		*buf; 		/* PCM buffer */
	/*
	 * The format is from the CPU node, for CODEC node clock roles
	 * need to be reversed.
	 */
	uint32_t		format;
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

/*
 * Invert master/slave roles for CODEC side of the node
 */
static uint32_t
audio_soc_reverse_clocks(uint32_t format)
{
	int fmt, pol, clk;

	fmt = AUDIO_DAI_FORMAT_FORMAT(format);
	pol = AUDIO_DAI_FORMAT_POLARITY(format);
	clk = AUDIO_DAI_FORMAT_CLOCK(format);

	switch (clk) {
	case AUDIO_DAI_CLOCK_CBM_CFM:
		clk = AUDIO_DAI_CLOCK_CBS_CFS;
		break;
	case AUDIO_DAI_CLOCK_CBS_CFM:
		clk = AUDIO_DAI_CLOCK_CBM_CFS;
		break;
	case AUDIO_DAI_CLOCK_CBM_CFS:
		clk = AUDIO_DAI_CLOCK_CBS_CFM;
		break;
	case AUDIO_DAI_CLOCK_CBS_CFS:
		clk = AUDIO_DAI_CLOCK_CBM_CFM;
		break;
	}

	return AUDIO_DAI_FORMAT(fmt, pol, clk);
}

static uint32_t
audio_soc_chan_setblocksize(kobj_t obj, void *data, uint32_t blocksz)
{

	return (blocksz);
}

static int
audio_soc_chan_setformat(kobj_t obj, void *data, uint32_t format)
{

	struct audio_soc_softc *sc;

	sc = data;

	return AUDIO_DAI_SET_CHANFORMAT(sc->cpu_dev, format);
}

static uint32_t
audio_soc_chan_setspeed(kobj_t obj, void *data, uint32_t speed)
{

	struct audio_soc_softc *sc;

	sc = data;

	return AUDIO_DAI_SET_CHANSPEED(sc->cpu_dev, speed);
}

static uint32_t
audio_soc_chan_getptr(kobj_t obj, void *data)
{
	struct audio_soc_softc *sc;

	sc = data;

	return AUDIO_DAI_GET_PTR(sc->cpu_dev);
}

static void *
audio_soc_chan_init(kobj_t obj, void *devinfo, struct snd_dbuf *b,
	struct pcm_channel *c, int dir)
{
	struct audio_soc_softc *sc;
	void *buffer;

	sc = devinfo;
	buffer = malloc(AUDIO_BUFFER_SIZE, M_DEVBUF, M_WAITOK | M_ZERO);

	if (sndbuf_setup(b, buffer, AUDIO_BUFFER_SIZE) != 0) {
		free(buffer, M_DEVBUF);
		return NULL;
	}
	sc->buf = b;
	sc->pcm = c;

	return (devinfo);
}

static int
audio_soc_chan_trigger(kobj_t obj, void *data, int go)
{
	struct audio_soc_softc *sc;

	sc = (struct audio_soc_softc *)data;
	return AUDIO_DAI_TRIGGER(sc->cpu_dev, go);
}

static int
audio_soc_chan_free(kobj_t obj, void *data)
{

	printf("%s:%s:%d\n", __FILE__, __func__, __LINE__);
	return (0);
}

static struct pcmchan_caps *
audio_soc_chan_getcaps(kobj_t obj, void *data)
{
	struct audio_soc_softc *sc = data;

	return AUDIO_DAI_GET_CAPS(sc->cpu_dev);
}

static kobj_method_t audio_soc_chan_methods[] = {
	KOBJMETHOD(channel_init, 	audio_soc_chan_init),
	KOBJMETHOD(channel_free, 	audio_soc_chan_free),
	KOBJMETHOD(channel_setformat, 	audio_soc_chan_setformat),
	KOBJMETHOD(channel_setspeed, 	audio_soc_chan_setspeed),
	KOBJMETHOD(channel_setblocksize,audio_soc_chan_setblocksize),
	KOBJMETHOD(channel_trigger,	audio_soc_chan_trigger),
	KOBJMETHOD(channel_getptr,	audio_soc_chan_getptr),
	KOBJMETHOD(channel_getcaps,	audio_soc_chan_getcaps),
	KOBJMETHOD_END
};
CHANNEL_DECLARE(audio_soc_chan);

static void
audio_soc_intr(void *arg)
{
	struct audio_soc_softc *sc;
	int channel_intr_required;

	sc = (struct audio_soc_softc *)arg;
	channel_intr_required = AUDIO_DAI_INTR(sc->cpu_dev, sc->buf);
	if (channel_intr_required)
		chn_intr(sc->pcm);
}

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

static void
audio_soc_init(void *arg)
{
	struct audio_soc_softc *sc;
	phandle_t node, child;
	device_t daidev;
	uint32_t xref;

	sc = (struct audio_soc_softc *)arg;
	config_intrhook_disestablish(&sc->init_hook);

	node = ofw_bus_get_node(sc->dev);
	/* TODO: handle multi-link nodes */
	child = ofw_bus_find_child(node, "simple-audio-card,cpu");
	if (child == 0) {
		device_printf(sc->dev, "cpu node is missing\n");
		return;
	}
	if ((OF_getencprop(child, "sound-dai", &xref, sizeof(xref))) <= 0) {
		device_printf(sc->dev, "missing sound-dai property in cpu node\n");
		return;
	}
	daidev = OF_device_from_xref(xref);
	if (daidev == NULL) {
		device_printf(sc->dev, "no driver attached to cpu node\n");
		return;
	}
	sc->cpu_dev = daidev;

	child = ofw_bus_find_child(node, "simple-audio-card,codec");
	if (child == 0) {
		device_printf(sc->dev, "codec node is missing\n");
		return;
	}
	if ((OF_getencprop(child, "sound-dai", &xref, sizeof(xref))) <= 0) {
		device_printf(sc->dev, "missing sound-dai property in codec node\n");
		return;
	}
	daidev = OF_device_from_xref(xref);
	if (daidev == NULL) {
		device_printf(sc->dev, "no driver attached to codec node\n");
		return;
	}
	sc->codec_dev = daidev;

	if (AUDIO_DAI_INIT(sc->cpu_dev, sc->format)) {
		device_printf(sc->dev, "failed to initalize cpu node\n");
		return;
	}

	/* Reverse clock roles for CODEC */
	if (AUDIO_DAI_INIT(sc->codec_dev, audio_soc_reverse_clocks(sc->format))) {
		device_printf(sc->dev, "failed to initalize codec node\n");
		return;
	}

	if (pcm_register(sc->dev, sc, 1, 0)) {
		device_printf(sc->dev, "failed to register PCM\n");
		return;
	}

	pcm_getbuffersize(sc->dev, AUDIO_BUFFER_SIZE, AUDIO_BUFFER_SIZE,
	    AUDIO_BUFFER_SIZE);
	pcm_addchan(sc->dev, PCMDIR_PLAY, &audio_soc_chan_class, sc);

	pcm_setstatus(sc->dev, "at EXPERIMENT");

	AUDIO_DAI_SETUP_INTR(sc->cpu_dev, audio_soc_intr, sc);
	AUDIO_DAI_SETUP_MIXER(sc->codec_dev, sc->dev);
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
	device_set_desc(dev, sc->name);

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

	sc->format = AUDIO_DAI_FORMAT(fmt, pol, clk);

	sc->init_hook.ich_func = audio_soc_init;
	sc->init_hook.ich_arg = sc;
	if (config_intrhook_establish(&sc->init_hook) != 0)
		return (ENOMEM);

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
	"pcm",
	audio_soc_methods,
	sizeof(struct audio_soc_softc),
};
static devclass_t audio_soc_devclass;

DRIVER_MODULE(audio_soc, simplebus, audio_soc_driver, audio_soc_devclass, NULL, NULL);
MODULE_VERSION(audio_soc, 1);
