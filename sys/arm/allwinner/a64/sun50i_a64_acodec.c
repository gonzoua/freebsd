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

struct a64codec_softc {
	device_t	dev;
	struct resource	*res;
	struct mtx	mtx;
};

#define	A64CODEC_LOCK(sc)		mtx_lock(&(sc)->mtx)
#define	A64CODEC_UNLOCK(sc)		mtx_unlock(&(sc)->mtx)
#define	A64CODEC_READ(sc, reg)	bus_read_4((sc)->res, (reg))
#define	A64CODEC_WRITE(sc, reg, val)	bus_write_4((sc)->res, (reg), (val))

static int a64codec_probe(device_t dev);
static int a64codec_attach(device_t dev);
static int a64codec_detach(device_t dev);

static int
a64codec_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
		return (ENXIO);

	device_set_desc(dev, "Allwinner Codec");
	return (BUS_PROBE_DEFAULT);
}

static int
a64codec_attach(device_t dev)
{
	struct a64codec_softc *sc;
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
	a64codec_detach(dev);
	return (error);
}

static int
a64codec_detach(device_t dev)
{
	struct a64codec_softc *sc;

	sc = device_get_softc(dev);

	if (sc->res)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->res);
	mtx_destroy(&sc->mtx);

	return (0);
}

static int
a64codec_dai_init(device_t dev, uint32_t format)
{

	return (0);
}

static int
a64codec_dai_trigger(device_t dev, int go, int pcm_dir)
{
	struct a64codec_softc 	*sc = device_get_softc(dev);

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
a64codec_dai_setup_mixer(device_t dev, device_t pcmdev)
{
	struct a64codec_softc *sc;

	sc = device_get_softc(dev);
	device_printf(sc->dev, "TODO: implement me %s\n", __func__);

	return (0);
}


static device_method_t a64codec_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		a64codec_probe),
	DEVMETHOD(device_attach,	a64codec_attach),
	DEVMETHOD(device_detach,	a64codec_detach),

	DEVMETHOD(audio_dai_init,	a64codec_dai_init),
	DEVMETHOD(audio_dai_setup_mixer,	a64codec_dai_setup_mixer),
	DEVMETHOD(audio_dai_trigger,	a64codec_dai_trigger),

	DEVMETHOD_END
};

static driver_t a64codec_driver = {
	"a64codec",
	a64codec_methods,
	sizeof(struct a64codec_softc),
};

static devclass_t a64codec_devclass;

DRIVER_MODULE(a64codec, simplebus, a64codec_driver, a64codec_devclass, 0, 0);
SIMPLEBUS_PNP_INFO(compat_data);
