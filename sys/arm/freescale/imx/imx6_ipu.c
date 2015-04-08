/*-
 * Copyright 2015 Oleksandr Tymoshenko <gonzo@freebsd.org>
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
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/resource.h>
#include <sys/rman.h>
#include <sys/sysctl.h>
#include <sys/fbio.h>
#include <sys/consio.h>

#include <machine/bus.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/fb/fbreg.h>
#include <dev/vt/vt.h>

#include "fb_if.h"

#define	WIDTH	1024
#define	HEIGHT	768
#define	BPP	16

#define	DMA_CHANNEL	23

#define	IPU_LOCK(_sc)		mtx_lock(&(_sc)->sc_mtx)
#define	IPU_UNLOCK(_sc)		mtx_unlock(&(_sc)->sc_mtx)
#define	IPU_LOCK_INIT(_sc)	mtx_init(&(_sc)->sc_mtx, \
    device_get_nameunit(_sc->sc_dev), "ipu", MTX_DEF)
#define	IPU_LOCK_DESTROY(_sc)	mtx_destroy(&(_sc)->sc_mtx);

#define	IPU_READ4(_sc, reg)	bus_read_4((_sc)->sc_mem_res, reg);
#define	IPU_WRITE4(_sc, reg, value)	\
    bus_write_4((_sc)->sc_mem_res, reg, value);

#define	CPMEM_BASE	0x300000

#define	IPU_CONF		0x200000
#define	IPU_CH_DB_MODE_SEL_0	0x200150
#define	IPU_CH_DB_MODE_SEL_1	0x200154
#define	IPU_CUR_BUF_0		0x20023C
#define	IPU_CUR_BUF_1		0x200240

#define	DMFC_RD_CHAN		0x260000
#define	DMFC_WR_CHAN		0x260004
#define	DMFC_WR_CHAN_DEF	0x260008
#define	DMFC_DP_CHAN		0x26000C
#define	DMFC_DP_CHAN_DEF	0x260010
#define	DMFC_GENERAL_1		0x260014
#define	DMFC_IC_CTRL		0x26001C

struct ipu_cpmem_word {
	uint32_t	data[5];
	uint32_t	padding[3];
};

struct ipu_cpmem_ch_param {
	struct ipu_cpmem_word	word[2];
};

#define	CH_PARAM_RESET(param) memset(param, 0, sizeof(*param))
#define	IPU_READ_CH_PARAM(_sc, ch, param) bus_read_region_4( \
	(_sc)->sc_mem_res, CPMEM_BASE + ch*(sizeof(*param)),\
	(uint32_t*)param, sizeof(*param)/4)
#define	IPU_WRITE_CH_PARAM(_sc, ch, param) bus_write_region_4( \
	(_sc)->sc_mem_res, CPMEM_BASE + ch*(sizeof(*param)),\
	(uint32_t*)param, sizeof(*param)/4)

#define	CH_PARAM_SET_FW(param, v) ipu_ch_param_set_value((param), \
	0, 125, 13, (v))
#define	CH_PARAM_SET_FH(param, v) ipu_ch_param_set_value((param), \
	0, 138, 12, (v))
#define	CH_PARAM_SET_SLY(param, v) ipu_ch_param_set_value((param), \
	1, 102, 14, (v))
#define	CH_PARAM_SET_EBA0(param, v) ipu_ch_param_set_value((param), \
	1, 0, 29, (v))
#define	CH_PARAM_SET_EBA1(param, v) ipu_ch_param_set_value((param), \
	1, 29, 29, (v))
#define	CH_PARAM_SET_BPP(param, v) ipu_ch_param_set_value((param), \
	0, 107, 3, (v))
#define	CH_PARAM_SET_PFS(param, v) ipu_ch_param_set_value((param), \
	1, 85, 4, (v))
#define	CH_PARAM_SET_NPB(param, v) ipu_ch_param_set_value((param), \
	1, 78, 7, (v))
#define	CH_PARAM_SET_UBO(param, v) ipu_ch_param_set_value((param), \
	0, 46, 22, (v))
#define	CH_PARAM_SET_VBO(param, v) ipu_ch_param_set_value((param), \
	0, 68, 22, (v))

#define	CH_PARAM_GET_FW(param) ipu_ch_param_get_value((param), \
	0, 125, 13)
#define	CH_PARAM_GET_FH(param) ipu_ch_param_get_value((param), \
	0, 138, 12)
#define	CH_PARAM_GET_SLY(param) ipu_ch_param_get_value((param), \
	1, 102, 14)
#define	CH_PARAM_GET_EBA0(param) ipu_ch_param_get_value((param), \
	1, 0, 29)
#define	CH_PARAM_GET_EBA1(param) ipu_ch_param_get_value((param), \
	1, 29, 29)
#define	CH_PARAM_GET_BPP(param) ipu_ch_param_get_value((param), \
	0, 107, 3)
#define	CH_PARAM_GET_PFS(param) ipu_ch_param_get_value((param), \
	1, 85, 4)
#define	CH_PARAM_GET_NPB(param) ipu_ch_param_get_value((param), \
	1, 78, 7)
#define	CH_PARAM_GET_UBO(param) ipu_ch_param_get_value((param), \
	0, 46, 22)
#define	CH_PARAM_GET_VBO(param) ipu_ch_param_get_value((param), \
	0, 68, 22)

#define	IPU_PIX_FORMAT_RGB	7

struct ipu_softc {
	device_t		sc_dev;
	struct resource		*sc_mem_res;
	int			sc_mem_rid;
	struct resource		*sc_irq_res;
	int			sc_irq_rid;
	void			*sc_intr_hl;
	struct mtx		sc_mtx;
	struct fb_info		sc_fb_info;

	/* Framebuffer */
	bus_dma_tag_t		sc_dma_tag;
	bus_dmamap_t		sc_dma_map;
	size_t			sc_fb_size;
	bus_addr_t		sc_fb_phys;
	uint8_t			*sc_fb_base;
};

static void
ipu_dmamap_cb(void *arg, bus_dma_segment_t *segs, int nseg, int err)
{
	bus_addr_t *addr;

	if (err)
		return;

	addr = (bus_addr_t*)arg;
	*addr = segs[0].ds_addr;
}

static void
ipu_ch_param_set_value(struct ipu_cpmem_ch_param *param,
    int word, int offset, int len, uint32_t value)
{
	uint32_t datapos, bitpos, mask;
	uint32_t data, data2;

	datapos = offset / 32;
	bitpos = offset % 32;
	mask = (1 << len) - 1;

	if (len > 32)
		panic("%s: field len is more than 32",
		    __func__);

	data = param->word[word].data[datapos];
	data &= ~(mask << bitpos);
	data |= (value << bitpos);
	param->word[word].data[datapos] = data;

	if ((bitpos + len) > 32) {
		len = bitpos + len - 32;
		mask = (1UL << len) - 1;
		data2 = param->word[word].data[datapos+1];
		data2 &= mask;
		data2 |= (value >> (32 - bitpos));
		param->word[word].data[datapos+1] = data2;
	}
}

static uint32_t
ipu_ch_param_get_value(struct ipu_cpmem_ch_param *param,
    int word, int offset, int len)
{
	uint32_t datapos, bitpos, mask;
	uint32_t data, data2;

	datapos = offset / 32;
	bitpos = offset % 32;
	mask = (1UL << len) - 1;

	if (len > 32)
		panic("%s: field len is more than 32",
		    __func__);
	data = param->word[word].data[datapos];
	data = data >> bitpos;
	data &= mask;
	if ((bitpos + len) > 32) {
		len = bitpos + len - 32;
		mask = (1UL << len) - 1;
		data2 = param->word[word].data[datapos+1];
		data2 &= mask;
		data |= (data2 << (32 - bitpos));
	}

	return (data);
}

static void
ipu_print_channel(struct ipu_cpmem_ch_param *param)
{
	printf("WORD0: %08x %08x %08x %08x %08x\n",
		param->word[0].data[0], param->word[0].data[1],
		param->word[0].data[2], param->word[0].data[3],
		param->word[0].data[4]);
	printf("WORD1: %08x %08x %08x %08x %08x\n",
		param->word[1].data[0], param->word[1].data[1],
		param->word[1].data[2], param->word[1].data[3],
		param->word[1].data[4]);
	printf("FW:   %d\n", CH_PARAM_GET_FW(param));
	printf("FH:   %d\n", CH_PARAM_GET_FH(param));
	printf("SLY:  %d\n", CH_PARAM_GET_SLY(param));
	printf("EBA0: 0x%08x\n", CH_PARAM_GET_EBA0(param));
	printf("EBA1: 0x%08x\n", CH_PARAM_GET_EBA1(param));
	printf("BPP:  %d\n", CH_PARAM_GET_BPP(param));
	printf("PFS:  %d\n", CH_PARAM_GET_PFS(param));
	printf("NPB:  %d\n", CH_PARAM_GET_NPB(param));
	printf("UBO:  %d\n", CH_PARAM_GET_UBO(param));
	printf("VBO:  %d\n", CH_PARAM_GET_VBO(param));
}

static void
ipu_init_buffer(struct ipu_softc *sc)
{
	struct ipu_cpmem_ch_param param;
	uint32_t stride;
	uint32_t reg, db_mode_sel, cur_buf;

	stride = WIDTH*BPP/8;

	/* init channel paramters */
	CH_PARAM_RESET(&param);
	/* XXX: interlaced modes are not supported yet */
	CH_PARAM_SET_FW(&param, WIDTH - 1);
	CH_PARAM_SET_FH(&param, HEIGHT - 1);
	CH_PARAM_SET_SLY(&param, stride - 1);

	CH_PARAM_SET_EBA0(&param, (sc->sc_fb_phys >> 3));

	CH_PARAM_SET_BPP(&param, BPP/8);
	CH_PARAM_SET_PFS(&param, IPU_PIX_FORMAT_RGB);
	CH_PARAM_SET_NPB(&param, 15);

	CH_PARAM_SET_UBO(&param, 0);
	CH_PARAM_SET_VBO(&param, 0);

	IPU_WRITE_CH_PARAM(sc, DMA_CHANNEL, &param);

	/* init DMFC */
	IPU_WRITE4(sc, DMFC_IC_CTRL, 0x2);
	/* High resolution DP */
	IPU_WRITE4(sc, DMFC_WR_CHAN, 0x00000090);
	IPU_WRITE4(sc, DMFC_WR_CHAN_DEF, 0x202020F6);
	IPU_WRITE4(sc, DMFC_DP_CHAN, 0x0000968a);
	IPU_WRITE4(sc, DMFC_DP_CHAN_DEF, 0x2020F6F6);

	reg = IPU_READ4(sc, DMFC_GENERAL_1);
	reg &= ~(1UL << 20);
	IPU_WRITE4(sc, DMFC_GENERAL_1, reg);

	/* XXX: set priority? */

	/* Set single buffer mode */
	if (DMA_CHANNEL < 32) {
		db_mode_sel = IPU_CH_DB_MODE_SEL_0;
		cur_buf = IPU_CUR_BUF_0;
	} else {
		db_mode_sel = IPU_CH_DB_MODE_SEL_1;
		cur_buf = IPU_CUR_BUF_1;
	}

	reg = IPU_READ4(sc, db_mode_sel);
	reg &= ~(1UL << (DMA_CHANNEL & 0x1f));
	IPU_WRITE4(sc, db_mode_sel, reg);
	IPU_WRITE4(sc, cur_buf, (1UL << (DMA_CHANNEL & 0x1f)));
}

static int
ipu_init(struct ipu_softc *sc)
{
	uint32_t reg;
	int err;
	size_t dma_size;

	reg = IPU_READ4(sc, IPU_CONF);
	printf("IPU_CONF == %08x\n", reg);

	dma_size = round_page(1026*768*4);

#if 0
	struct ipu_cpmem_ch_param param;
	CH_PARAM_RESET(&param);
	IPU_READ_CH_PARAM(sc, 23, &param);
	printf("Channel 23\n");
	ipu_print_channel(&param);

	CH_PARAM_RESET(&param);
	IPU_READ_CH_PARAM(sc, 27, &param);
	printf("Channel 27\n");
	ipu_print_channel(&param);

	CH_PARAM_RESET(&param);
	IPU_READ_CH_PARAM(sc, 28, &param);
	printf("Channel 28\n");
	ipu_print_channel(&param);
	CH_PARAM_SET_FW(&param, 1024);
	CH_PARAM_SET_FH(&param, 768);
	printf("Channel 28\n");
	ipu_print_channel(&param);
#endif

	/*
	 * Now allocate framebuffer memory
	 */
	err = bus_dma_tag_create(
	    bus_get_dma_tag(sc->sc_dev),
	    4, 0,		/* alignment, boundary */
	    BUS_SPACE_MAXADDR_32BIT,	/* lowaddr */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    dma_size, 1,			/* maxsize, nsegments */
	    dma_size, 0,			/* maxsegsize, flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->sc_dma_tag);
	if (err)
		goto fail;

	err = bus_dmamem_alloc(sc->sc_dma_tag, (void **)&sc->sc_fb_base,
	    BUS_DMA_COHERENT, &sc->sc_dma_map);

	if (err) {
		device_printf(sc->sc_dev, "cannot allocate framebuffer\n");
		goto fail;
	}

	err = bus_dmamap_load(sc->sc_dma_tag, sc->sc_dma_map, sc->sc_fb_base,
	    dma_size, ipu_dmamap_cb, &sc->sc_fb_phys, BUS_DMA_NOWAIT);

	if (err) {
		device_printf(sc->sc_dev, "cannot load DMA map\n");
		goto fail;
	}

	/* Make sure it's blank */
	memset(sc->sc_fb_base, 0xAA, dma_size);

	/* Calculate actual FB Size */
	sc->sc_fb_size = WIDTH*HEIGHT*BPP/8;

	ipu_init_buffer(sc);

	sc->sc_fb_info.fb_name = device_get_nameunit(sc->sc_dev);
	sc->sc_fb_info.fb_vbase = (intptr_t)sc->sc_fb_base;
	sc->sc_fb_info.fb_pbase = sc->sc_fb_phys;
	sc->sc_fb_info.fb_size = sc->sc_fb_size;
	sc->sc_fb_info.fb_bpp = sc->sc_fb_info.fb_depth = BPP;
	sc->sc_fb_info.fb_stride = WIDTH*BPP / 8;
	sc->sc_fb_info.fb_width = WIDTH;
	sc->sc_fb_info.fb_height = HEIGHT;

	device_t fbd = device_add_child(sc->sc_dev, "fbd",
	    device_get_unit(sc->sc_dev));
	if (fbd == NULL) {
		device_printf(sc->sc_dev, "Failed to add fbd child\n");
		goto fail;
	}
	if (device_probe_and_attach(fbd) != 0) {
		device_printf(sc->sc_dev, "Failed to attach fbd device\n");
		goto fail;
	}

	return (0);
fail:

	return (err);
}

static int
ipu_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "fsl,imx6q-ipu"))
		return (ENXIO);

	device_set_desc(dev, "Freescale IPU");

	return (BUS_PROBE_DEFAULT);
}

static int
ipu_attach(device_t dev)
{
	struct ipu_softc *sc;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;

	sc->sc_mem_rid = 0;
	sc->sc_mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &sc->sc_mem_rid, RF_ACTIVE);
	if (!sc->sc_mem_res) {
		device_printf(dev, "cannot allocate memory window\n");
		return (ENXIO);
	}

	sc->sc_irq_rid = 0;
	sc->sc_irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ,
	    &sc->sc_irq_rid, RF_ACTIVE);
	if (!sc->sc_irq_res) {
		bus_release_resource(dev, SYS_RES_MEMORY,
		    sc->sc_mem_rid, sc->sc_mem_res);
		device_printf(dev, "cannot allocate interrupt\n");
		return (ENXIO);
	}

	ipu_init(sc);

#if 0
	if (bus_setup_intr(dev, sc->sc_irq_res, INTR_TYPE_MISC | INTR_MPSAFE,
			NULL, ipu_intr, sc,
			&sc->sc_intr_hl) != 0) {
		bus_release_resource(dev, SYS_RES_IRQ, rid,
		    sc->sc_irq_res);
		bus_release_resource(dev, SYS_RES_MEMORY, rid,
		    sc->sc_mem_res);
		device_printf(dev, "Unable to setup the irq handler.\n");
		return (ENXIO);
	}
#endif

	IPU_LOCK_INIT(sc);

	return (0);
}

static int
ipu_detach(device_t dev)
{
	/* Do not let unload driver */
	return (EBUSY);
}

static struct fb_info *
ipu_fb_getinfo(device_t dev)
{
	struct ipu_softc *sc;

	sc = device_get_softc(dev);

	return (&sc->sc_fb_info);
}

static device_method_t ipu_methods[] = {
	DEVMETHOD(device_probe,		ipu_probe),
	DEVMETHOD(device_attach,	ipu_attach),
	DEVMETHOD(device_detach,	ipu_detach),

	/* Framebuffer service methods */
	DEVMETHOD(fb_getinfo,		ipu_fb_getinfo),

	DEVMETHOD_END
};

static driver_t ipu_driver = {
	"fb",
	ipu_methods,
	sizeof(struct ipu_softc),
};

static devclass_t ipu_devclass;

DRIVER_MODULE(ipu, simplebus, ipu_driver, ipu_devclass, 0, 0);
MODULE_VERSION(ipu, 1);
MODULE_DEPEND(ipu, simplebus, 1, 1, 1);
