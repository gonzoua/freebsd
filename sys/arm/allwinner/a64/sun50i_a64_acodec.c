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

#define	A64_PR_CFG		0x00
#define	 A64_AC_PR_RST		(1 << 28)
#define	 A64_AC_PR_RW		(1 << 24)
#define	 A64_AC_PR_ADDR_MASK	(0x1f << 16)
#define	 A64_AC_PR_ADDR(n)	(((n) & 0x1f) << 16)
#define	 A64_ACDA_PR_WDAT_MASK	(0xff << 8)
#define	 A64_ACDA_PR_WDAT(n)	(((n) & 0xff) << 8)
#define	 A64_ACDA_PR_RDAT(n)	((n) & 0xff)

#define	A64_HP_CTRL		0x00
#define	 A64_HPPA_EN		(1 << 6)
#define	 A64_HPVOL		__BITS(5,0)
#define	A64_OL_MIX_CTRL		0x01
#define	 A64_LMIXMUTE_LDAC	(1 << 1)
#define	A64_OR_MIX_CTRL		0x02
#define	 A64_RMIXMUTE_RDAC	(1 << 1)
#define	A64_LINEOUT_CTRL0	0x05
#define	 A64_LINEOUT_LEFT_EN	(1 << 7)
#define	 A64_LINEOUT_RIGHT_EN	(1 << 6)
#define	 A64_LINEOUT_EN		(A64_LINEOUT_LEFT_EN|A64_LINEOUT_RIGHT_EN)
#define	A64_LINEOUT_CTRL1	0x06
#define	 A64_LINEOUT_VOL	__BITS(4,0)
#define	A64_MIC1_CTRL		0x07
#define	 A64_MIC1G		__BITS(6,4)
#define	 A64_MIC1AMPEN		(1 << 3)
#define	 A64_MIC1BOOST		__BITS(2,0)
#define	A64_MIC2_CTRL		0x08
#define	 A64_MIC2_SEL		(1 << 7)
#define	 A64_MIC2G		__BITS(6,4)
#define	 A64_MIC2AMPEN		(1 << 3)
#define	 A64_MIC2BOOST		__BITS(2,0)
#define	A64_LINEIN_CTRL		0x09
#define	 A64_LINEING		__BITS(6,4)
#define	A64_MIX_DAC_CTRL	0x0a
#define	 A64_DACAREN		(1 << 7)
#define	 A64_DACALEN		(1 << 6)
#define	 A64_RMIXEN		(1 << 5)
#define	 A64_LMIXEN		(1 << 4)
#define	 A64_RHPPAMUTE		(1 << 3)
#define	 A64_LHPPAMUTE		(1 << 2)
#define	 A64_RHPIS		(1 << 1)
#define	 A64_LHPIS		(1 << 0)
#define	A64_L_ADCMIX_SRC	0x0b
#define	A64_R_ADCMIX_SRC	0x0c
#define	 A64_ADCMIX_SRC_MIC1	(1 << 6)
#define	 A64_ADCMIX_SRC_MIC2	(1 << 5)
#define	 A64_ADCMIX_SRC_LINEIN	(1 << 2)
#define	 A64_ADCMIX_SRC_OMIXER	(1 << 1)
#define	A64_ADC_CTRL		0x0d
#define	 A64_ADCREN		(1 << 7)
#define	 A64_ADCLEN		(1 << 6)
#define	 A64_ADCG		__BITS(2,0)
#define	A64_JACK_MIC_CTRL	0x1d
#define	 A64_JACKDETEN		(1 << 7)
#define	 A64_INNERRESEN		(1 << 6)
#define	 A64_AUTOPLEN		(1 << 1)

static struct ofw_compat_data compat_data[] = {
	{ "allwinner,sun50i-a64-codec-analog",	1},
	{ NULL,					0 }
};

struct a64codec_softc {
	device_t	dev;
	struct resource	*res;
	struct mtx	mtx;
};

#define	A64CODEC_LOCK(sc)		mtx_lock(&(sc)->mtx)
#define	A64CODEC_UNLOCK(sc)		mtx_unlock(&(sc)->mtx)
#define	A64CODEC_READ(sc, reg)		bus_read_4((sc)->res, (reg))
#define	A64CODEC_WRITE(sc, reg, val)	bus_write_4((sc)->res, (reg), (val))

static int a64codec_probe(device_t dev);
static int a64codec_attach(device_t dev);
static int a64codec_detach(device_t dev);

static u_int
a64_acodec_pr_read(struct a64codec_softc *sc, u_int addr)
{
	uint32_t val;

	/* Read current value */
	val = A64CODEC_READ(sc, A64_PR_CFG);

	/* De-assert reset */
	val |= A64_AC_PR_RST;
	A64CODEC_WRITE(sc, A64_PR_CFG, val);

	/* Read mode */
	val &= ~A64_AC_PR_RW;
	A64CODEC_WRITE(sc, A64_PR_CFG, val);

	/* Set address */
	val &= ~A64_AC_PR_ADDR_MASK;
	val |= A64_AC_PR_ADDR(addr);
	A64CODEC_WRITE(sc, A64_PR_CFG, val);

	/* Read data */
	val = A64CODEC_READ(sc, A64_PR_CFG);
	return A64_ACDA_PR_RDAT(val);
}

static void
a64_acodec_pr_write(struct a64codec_softc *sc, u_int addr, u_int data)
{
	uint32_t val;

	/* Read current value */
	val = A64CODEC_READ(sc, A64_PR_CFG);

	/* De-assert reset */
	val |= A64_AC_PR_RST;
	A64CODEC_WRITE(sc, A64_PR_CFG, val);

	/* Set address */
	val &= ~A64_AC_PR_ADDR_MASK;
	val |= A64_AC_PR_ADDR(addr);
	A64CODEC_WRITE(sc, A64_PR_CFG, val);

	/* Write data */
	val &= ~A64_ACDA_PR_WDAT_MASK;
	val |= A64_ACDA_PR_WDAT(data);
	A64CODEC_WRITE(sc, A64_PR_CFG, val);

	/* Write mode */
	val |= A64_AC_PR_RW;
	A64CODEC_WRITE(sc, A64_PR_CFG, val);

	/* Clear write mode */
	val &= ~A64_AC_PR_RW;
	A64CODEC_WRITE(sc, A64_PR_CFG, val);
}

static void
a64_acodec_pr_set_clear(struct a64codec_softc *sc, u_int addr, u_int set, u_int clr)
{
	u_int old, new;

	old = a64_acodec_pr_read(sc, addr);
	new = set | (old & ~clr);
	a64_acodec_pr_write(sc, addr, new);
}

static int
a64codec_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
		return (ENXIO);

	device_set_desc(dev, "Allwinner A64 Analog Codec");
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

	/* Right & Left Headphone PA enable */
	a64_acodec_pr_set_clear(sc, A64_HP_CTRL,
	    A64_HPPA_EN, 0);

	/* Jack detect enable */
	a64_acodec_pr_set_clear(sc, A64_JACK_MIC_CTRL,
	    A64_JACKDETEN | A64_INNERRESEN | A64_AUTOPLEN, 0);

	/* Unmute DAC to output mixer */
	a64_acodec_pr_set_clear(sc, A64_OL_MIX_CTRL,
	    A64_LMIXMUTE_LDAC, 0);
	a64_acodec_pr_set_clear(sc, A64_OR_MIX_CTRL,
	    A64_RMIXMUTE_RDAC, 0);

	/* For now we work only with headphones */
	a64_acodec_pr_set_clear(sc, A64_LINEOUT_CTRL0,
	    0, A64_LINEOUT_EN);
	a64_acodec_pr_set_clear(sc, A64_HP_CTRL,
	    A64_HPPA_EN, 0);

	/* TODO: implement mixer control, for now just hardcode volume */
	u_int val = a64_acodec_pr_read(sc, A64_HP_CTRL);
	val &= ~(0x3f);
	val |= 0x25;
	a64_acodec_pr_write(sc, A64_HP_CTRL, val);

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
		if (pcm_dir == PCMDIR_PLAY) {
			/* Enable DAC analog l/r channels, HP PA, and output mixer */
			a64_acodec_pr_set_clear(sc, A64_MIX_DAC_CTRL,
			    A64_DACAREN | A64_DACALEN | A64_RMIXEN | A64_LMIXEN |
			    A64_RHPPAMUTE | A64_LHPPAMUTE, 0);
		}
		else if (pcm_dir == PCMDIR_REC) {
			/* Enable ADC analog l/r channels */
			a64_acodec_pr_set_clear(sc, A64_ADC_CTRL,
			    A64_ADCREN | A64_ADCLEN, 0);
		}
		break;

	case PCMTRIG_STOP:
	case PCMTRIG_ABORT:
		if (pcm_dir == PCMDIR_PLAY) {
			/* Disable DAC analog l/r channels, HP PA, and output mixer */
			a64_acodec_pr_set_clear(sc, A64_MIX_DAC_CTRL,
			    0, A64_DACAREN | A64_DACALEN | A64_RMIXEN | A64_LMIXEN |
			    A64_RHPPAMUTE | A64_LHPPAMUTE);
		}
		else if (pcm_dir == PCMDIR_REC) {
			/* Disable ADC analog l/r channels */
			a64_acodec_pr_set_clear(sc, A64_ADC_CTRL,
			    0, A64_ADCREN | A64_ADCLEN);
		}
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
