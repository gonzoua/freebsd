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

/*
 * Driver for Realtek es8316 audio codec
 */

#include "opt_platform.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/clock.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/endian.h>

#ifdef FDT
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#endif

#include <dev/extres/clk/clk.h>

#include <dev/iicbus/iiconf.h>
#include <dev/iicbus/iicbus.h>
#include <sys/sysctl.h>

#include <dev/sound/fdt/audio_dai.h>
#include <dev/sound/pcm/sound.h>

#include "iicbus_if.h"
#include "mixer_if.h"
#include "audio_dai_if.h"

#define	MAX_BUFFER	16

#define	ES8316_MIXER_DEVS (1 << SOUND_MIXER_VOLUME)

struct es8316_softc {
	device_t	dev;
	device_t	busdev;
	struct intr_config_hook
			init_hook;
	clk_t		clk;
};

#ifdef FDT
static struct ofw_compat_data compat_data[] = {
	{"everest,es8316", 1},
	{NULL,           0},
};
#endif

static void	es8316_init(void *arg);
static int	es8316_probe(device_t dev);
static int	es8316_attach(device_t dev);
static int	es8316_detach(device_t dev);

static inline int
es8316_read(struct es8316_softc *sc, uint8_t reg, uint8_t *data)
{
	int res;
	res = iicdev_readfrom(sc->dev, reg, data, 1, IIC_WAIT);
	if (res != 0)
		*data = 0xff;
	return (res);
}

static inline int
es8316_write(struct es8316_softc *sc, uint8_t reg, uint8_t val)
{

	return (iicdev_writeto(sc->dev, reg, &val, 1, IIC_WAIT));
}

static void
es8316_init(void *arg)
{
	struct es8316_softc *sc;

	sc = (struct es8316_softc*)arg;
	config_intrhook_disestablish(&sc->init_hook);
}

static int
es8316_mixer_init(struct snd_mixer *m)
{
	mix_setdevs(m, ES8316_MIXER_DEVS);

	return (0);
}

static int
es8316_mixer_uninit(struct snd_mixer *m)
{

	return (0);
}

static int
es8316_mixer_reinit(struct snd_mixer *m)
{

	return (0);
}

static int
es8316_mixer_set(struct snd_mixer *m, unsigned dev, unsigned left, unsigned right)
{
	struct es8316_softc *sc;
	struct mtx *mixer_lock;
	int locked;

	sc = device_get_softc(mix_getdevinfo(m));
	mixer_lock = mixer_get_lock(m);
	locked = mtx_owned(mixer_lock);

	/*
	 * We need to unlock the mixer lock because iicbus_transfer()
	 * may sleep. The mixer lock itself is unnecessary here
	 * because it is meant to serialize hardware access, which
	 * is taken care of by the I2C layer, so this is safe.
	 */
	if (locked)
		mtx_unlock(mixer_lock);
	switch (dev) {
	case SOUND_MIXER_VOLUME:
		printf("[%s] %s:%d\n", __func__, __FILE__, __LINE__);
		return (left | (right << 8));
	default:
		break;
	}

	if (locked)
		mtx_lock(mixer_lock);

	return (0);
}

static u_int32_t
es8316_mixer_setrecsrc(struct snd_mixer *m, u_int32_t src)
{

	return (0);
}


static kobj_method_t es8316_mixer_methods[] = {
	KOBJMETHOD(mixer_init, 		es8316_mixer_init),
	KOBJMETHOD(mixer_uninit, 	es8316_mixer_uninit),
	KOBJMETHOD(mixer_reinit, 	es8316_mixer_reinit),
	KOBJMETHOD(mixer_set, 		es8316_mixer_set),
	KOBJMETHOD(mixer_setrecsrc, 	es8316_mixer_setrecsrc),
	KOBJMETHOD_END
};

MIXER_DECLARE(es8316_mixer);

static int
es8316_probe(device_t dev)
{

#ifdef FDT
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data != 0) {
		device_set_desc(dev, "Everest Semi ES8316 CODEC");
		return (BUS_PROBE_DEFAULT);
	}
#endif
	return (ENXIO);
}

static int
es8316_attach(device_t dev)
{
	struct es8316_softc	*sc;
	int error;
	phandle_t node;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->busdev = device_get_parent(sc->dev);

	error = clk_get_by_ofw_name(dev, 0, "mclk", &sc->clk);
	if (error != 0) {
		device_printf(dev, "cannot get mclk clock\n");
		return (ENXIO);
	}
	clk_set_freq(sc->clk, 12288000, 0);

	clk_enable(sc->clk);

	/*
	 * Wait until i2c is ready to set up the chip
	 */
	sc->init_hook.ich_func = es8316_init;
	sc->init_hook.ich_arg = sc;
	if (config_intrhook_establish(&sc->init_hook) != 0)
		return (ENOMEM);

	node = ofw_bus_get_node(dev);
	OF_device_register_xref(OF_xref_from_node(node), dev);

	return (0);
}

static int
es8316_detach(device_t dev)
{

	return (0);
}

static int
es8316_dai_init(device_t dev, uint32_t format)
{
	struct es8316_softc* sc;
	int fmt, pol, clk;

	sc = (struct es8316_softc*)device_get_softc(dev);

	fmt = AUDIO_DAI_FORMAT_FORMAT(format);
	pol = AUDIO_DAI_FORMAT_POLARITY(format);
	clk = AUDIO_DAI_FORMAT_CLOCK(format);

	return (0);
}

static int
es8316_dai_setup_mixer(device_t dev, device_t pcmdev)
{

	mixer_init(pcmdev, &es8316_mixer_class, dev);

	return (0);
}

static int
es8316_dai_trigger(device_t dev, int go, int pcm_dir)
{

	switch (go) {
	case PCMTRIG_START:
		/* TODO: power up amps */
		break;

	case PCMTRIG_STOP:
	case PCMTRIG_ABORT:
		/* TODO: power down amps */
		break;
	}

	return (0);
}

static device_method_t es8316_methods[] = {
        /* device_if methods */
	DEVMETHOD(device_probe,		es8316_probe),
	DEVMETHOD(device_attach,	es8316_attach),
	DEVMETHOD(device_detach,	es8316_detach),

	DEVMETHOD(audio_dai_init,	es8316_dai_init),
	DEVMETHOD(audio_dai_setup_mixer,	es8316_dai_setup_mixer),
	DEVMETHOD(audio_dai_trigger,	es8316_dai_trigger),

	DEVMETHOD_END,
};

static driver_t es8316_driver = {
	"es8316codec",
	es8316_methods,
	sizeof(struct es8316_softc),
};
static devclass_t es8316_devclass;

DRIVER_MODULE(es8316codec, iicbus, es8316_driver, es8316_devclass, NULL, NULL);
MODULE_VERSION(es8316codec, 1);
MODULE_DEPEND(es8316codec, iicbus, IICBUS_MINVER, IICBUS_PREFVER, IICBUS_MAXVER);
IICBUS_FDT_PNP_INFO(compat_data);
