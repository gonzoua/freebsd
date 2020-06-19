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
#include <dev/extres/regulator/regulator.h>

#include "syscon_if.h"

#include "opt_snd.h"
#include <dev/sound/pcm/sound.h>
#include <dev/sound/fdt/audio_dai.h>
#include "audio_dai_if.h"
#include "mixer_if.h"

#define	RKCODEC_MIXER_DEVS	((1 << SOUND_MIXER_VOLUME) | \
	(1 << SOUND_MIXER_MIC))

static struct ofw_compat_data compat_data[] = {
	{ "rockchip,rk3328-codec",	1},
	{ NULL,				0 }
};

struct rkcodec_softc {
	device_t	dev;
	struct resource	*res;
	struct mtx	mtx;
	u_int	regaddr;	/* address for the sysctl */
};

#define	RKCODEC_LOCK(sc)		mtx_lock(&(sc)->mtx)
#define	RKCODEC_UNLOCK(sc)		mtx_unlock(&(sc)->mtx)
#define	RKCODEC_READ(sc, reg)		bus_read_4((sc)->res, (reg))
#define	RKCODEC_WRITE(sc, reg, val)	bus_write_4((sc)->res, (reg), (val))

static int rkcodec_probe(device_t dev);
static int rkcodec_attach(device_t dev);
static int rkcodec_detach(device_t dev);

static int
rkcodec_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
		return (ENXIO);

	device_set_desc(dev, "Rockchip RK3328 CODEC");
	return (BUS_PROBE_DEFAULT);
}

static int
rkcodec_attach(device_t dev)
{
	struct rkcodec_softc *sc;
	int error, rid;
	phandle_t node;

	sc = device_get_softc(dev);
	sc->dev = dev;

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	rid = 0;
	sc->res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (!sc->res) {
		device_printf(dev, "cannot allocate resource for device\n");
		error = ENXIO;
		goto fail;
	}

	node = ofw_bus_get_node(dev);
	OF_device_register_xref(OF_xref_from_node(node), dev);

	return (0);

fail:
	rkcodec_detach(dev);
	return (error);
}

static int
rkcodec_detach(device_t dev)
{
	struct rkcodec_softc *sc;

	sc = device_get_softc(dev);

	if (sc->res)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->res);
	mtx_destroy(&sc->mtx);

	return (0);
}

static int
rkcodec_mixer_init(struct snd_mixer *m)
{

	mix_setdevs(m, RKCODEC_MIXER_DEVS);

	return (0);
}

static int
rkcodec_mixer_uninit(struct snd_mixer *m)
{

	return (0);
}

static int
rkcodec_mixer_reinit(struct snd_mixer *m)
{

	return (0);
}

static int
rkcodec_mixer_set(struct snd_mixer *m, unsigned dev, unsigned left, unsigned right)
{
	struct rkcodec_softc *sc;
	struct mtx *mixer_lock;
	uint8_t do_unlock;

	sc = device_get_softc(mix_getdevinfo(m));
	mixer_lock = mixer_get_lock(m);

	if (mtx_owned(mixer_lock)) {
		do_unlock = 0;
	} else {
		do_unlock = 1;
		mtx_lock(mixer_lock);
	}

	right = left;

	RKCODEC_LOCK(sc);
	switch(dev) {
	case SOUND_MIXER_VOLUME:
		printf("[%s] %s:%d\n", __func__, __FILE__, __LINE__);
		break;

	case SOUND_MIXER_MIC:
		printf("[%s] %s:%d\n", __func__, __FILE__, __LINE__);
		break;
	default:
		break;
	}
	RKCODEC_UNLOCK(sc);

	if (do_unlock) {
		mtx_unlock(mixer_lock);
	}

	return (left | (right << 8));
}

static unsigned
rkcodec_mixer_setrecsrc(struct snd_mixer *m, unsigned src)
{

	return (0);
}

static kobj_method_t rkcodec_mixer_methods[] = {
	KOBJMETHOD(mixer_init,		rkcodec_mixer_init),
	KOBJMETHOD(mixer_uninit,	rkcodec_mixer_uninit),
	KOBJMETHOD(mixer_reinit,	rkcodec_mixer_reinit),
	KOBJMETHOD(mixer_set,		rkcodec_mixer_set),
	KOBJMETHOD(mixer_setrecsrc,	rkcodec_mixer_setrecsrc),
	KOBJMETHOD_END
};

MIXER_DECLARE(rkcodec_mixer);

static int
rkcodec_dai_init(device_t dev, uint32_t format)
{

	return (0);
}

static int
rkcodec_dai_trigger(device_t dev, int go, int pcm_dir)
{
	// struct rkcodec_softc 	*sc = device_get_softc(dev);

	if ((pcm_dir != PCMDIR_PLAY) && (pcm_dir != PCMDIR_REC))
		return (EINVAL);

	switch (go) {
	case PCMTRIG_START:
		if (pcm_dir == PCMDIR_PLAY) {
			printf("[%s] %s:%d\n", __func__, __FILE__, __LINE__);
		}
		else if (pcm_dir == PCMDIR_REC) {
			printf("[%s] %s:%d\n", __func__, __FILE__, __LINE__);
		}
		break;

	case PCMTRIG_STOP:
	case PCMTRIG_ABORT:
		if (pcm_dir == PCMDIR_PLAY) {
			printf("[%s] %s:%d\n", __func__, __FILE__, __LINE__);
		}
		else if (pcm_dir == PCMDIR_REC) {
			printf("[%s] %s:%d\n", __func__, __FILE__, __LINE__);
		}
		break;
	}

	return (0);
}

static int
rkcodec_dai_setup_mixer(device_t dev, device_t pcmdev)
{

	mixer_init(pcmdev, &rkcodec_mixer_class, dev);

	return (0);
}

static device_method_t rkcodec_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		rkcodec_probe),
	DEVMETHOD(device_attach,	rkcodec_attach),
	DEVMETHOD(device_detach,	rkcodec_detach),

	DEVMETHOD(audio_dai_init,	rkcodec_dai_init),
	DEVMETHOD(audio_dai_setup_mixer,	rkcodec_dai_setup_mixer),
	DEVMETHOD(audio_dai_trigger,	rkcodec_dai_trigger),

	DEVMETHOD_END
};

static driver_t rkcodec_driver = {
	"rk3328codec",
	rkcodec_methods,
	sizeof(struct rkcodec_softc),
};

static devclass_t rkcodec_devclass;

DRIVER_MODULE(rkcodec, simplebus, rkcodec_driver, rkcodec_devclass, 0, 0);
SIMPLEBUS_PNP_INFO(compat_data);
