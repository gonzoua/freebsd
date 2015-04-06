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

#include <machine/bus.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#define WIDTH	1024
#define	HEIGHT	768
#define	BPP	24

#define	DMA_CHANNEL	28

#define	IPU_LOCK(_sc)		mtx_lock(&(_sc)->sc_mtx)
#define	IPU_UNLOCK(_sc)		mtx_unlock(&(_sc)->sc_mtx)
#define	IPU_LOCK_INIT(_sc)	mtx_init(&(_sc)->sc_mtx, \
    device_get_nameunit(_sc->sc_dev), "ipu", MTX_DEF)
#define	IPU_LOCK_DESTROY(_sc)	mtx_destroy(&(_sc)->sc_mtx);

#define	IPU_READ4(_sc, reg)	bus_read_4((_sc)->sc_mem_res, reg);
#define	IPU_WRITE4(_sc, reg, value)	\
    bus_write_4((_sc)->sc_mem_res, reg, value);

#define	IPU1_BASE	0x200000
#define	IPU1_CONF	(IPU1_BASE + 0x00)

struct ipu_cpmem_word {
	uint32_t	data[5];
	uint32_t	padding[3];
};

struct ipu_cpmem_ch_param {
	struct ipu_cpmem_word	word[2];
};

struct ipu_softc {
	device_t		sc_dev;
	struct resource		*sc_mem_res;
	int			sc_mem_rid;
	struct resource		*sc_irq_res;
	int			sc_irq_rid;
	void			*sc_intr_hl;
	struct mtx		sc_mtx;

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
	uint32_t data;

	datapos = offset / 32;
	bitpos = offset % 32;
	mask = (1 << len) - 1;

	if (bitpos + len > 32)
		panic("%s: data boundary violation <w:%d, o:%d, l:%d>",
		    __func__, word, offset, len);
	if (value > mask)
		panic("%s: value is out of bound length:%d, value:%d\n",
		    __func__, len, value);
	data = param->word[word].data[datapos];
	data &= ~(mask << offset);
	data |= (value << offset);
	param->word[word].data[datapos] = data;
}

static uint32_t
ipu_ch_param_get_value(struct ipu_cpmem_ch_param *param,
    int word, int offset, int len)
{
	uint32_t datapos, bitpos, mask;
	uint32_t data;

	datapos = offset / 32;
	bitpos = offset % 32;
	mask = (1 << len) - 1;

	if (bitpos + len > 32)
		panic("%s: data boundary violation <w:%d, o:%d, l:%d>",
		    __func__, word, offset, len);
	data = param->word[word].data[datapos];
	data = data >> offset;
	data &= mask;

	return (data);
}

static void
ipu_init_buffer()
{
	/* init channel paramters */
	/* init DMFC */
	/* set high priority? */
}

static int
ipu_init(struct ipu_softc *sc)
{
	uint32_t reg;
	int err;
	size_t dma_size;

	reg = IPU_READ4(sc, IPU1_CONF);
	printf("IPU1_CONF == %08x\n", reg);

	dma_size = round_page(1026*768*4);

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
	memset(sc->sc_fb_base, 0x00, dma_size);

	/* Calculate actual FB Size */
	sc->sc_fb_size = 1026*768*4;

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

static device_method_t ipu_methods[] = {
	DEVMETHOD(device_probe,		ipu_probe),
	DEVMETHOD(device_attach,	ipu_attach),
	DEVMETHOD(device_detach,	ipu_detach),

	DEVMETHOD_END
};

static driver_t ipu_driver = {
	"ipu",
	ipu_methods,
	sizeof(struct ipu_softc),
};

static devclass_t ipu_devclass;

DRIVER_MODULE(ipu, simplebus, ipu_driver, ipu_devclass, 0, 0);
MODULE_VERSION(ipu, 1);
MODULE_DEPEND(ipu, simplebus, 1, 1, 1);
