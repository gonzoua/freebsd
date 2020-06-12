/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2013 Ian Lepore <ian@freebsd.org>
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

/*
 * Clocks and power control driver for Freescale i.MX6 family of SoCs.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/bus.h>
#include <sys/rman.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <machine/bus.h>

#include <arm64/freescale/imx/imx_ccm_clk.h>
#include <arm64/freescale/imx/clk/imx_clk_gate.h>
#include <arm64/freescale/imx/clk/imx_clk_mux.h>
#include <arm64/freescale/imx/clk/imx_clk_composite.h>
#include <arm64/freescale/imx/clk/imx_clk_sscg_pll.h>
#include <arm64/freescale/imx/clk/imx_clk_frac_pll.h>

#include <gnu/dts/include/dt-bindings/clock/imx8mq-clock.h>

#include "clkdev_if.h"

static const char *pll_ref_p[] = {
	"osc_25m", "osc_27m", "dummy", "dummy"
};
static const char *sys3_pll_out_p[] = {
	"sys3_pll1_ref_sel"
};
static const char *uart_p[] = {
	"osc_25m", "sys1_pll_80m", "sys2_pll_200m", "sys2_pll_100m", "sys3_pll_out", "clk_ext2", "clk_ext4", "audio_pll2_out"
};
static const char *usdhc_p[] = {
	"osc_25m", "sys1_pll_400m", "sys1_pll_800m", "sys2_pll_500m", "audio_pll2_out", "sys1_pll_266m", "sys3_pll_out", "sys1_pll_100m"
};
static const char *enet_axi_p[] = {
	"osc_25m", "sys1_pll_266m", "sys1_pll_800m", "sys2_pll_250m", "sys2_pll_200m", "audio_pll1_out", "video_pll1_out", "sys3_pll_out"
};
static const char *enet_ref_p[] = {
	"osc_25m", "sys2_pll_125m", "sys2_pll_500m", "sys2_pll_100m", "sys1_pll_160m", "audio_pll1_out", "video_pll1_out", "clk_ext4"
};
static const char *enet_timer_p[] = {
	"osc_25m", "sys2_pll_100m", "audio_pll1_out", "clk_ext1", "clk_ext2", "clk_ext3", "clk_ext4", "video_pll1_out"
};
static const char *enet_phy_ref_p[] = {
	"osc_25m", "sys2_pll_50m", "sys2_pll_125m", "sys2_pll_500m", "audio_pll1_out", "video_pll1_out", "audio_pll2_out"
};
static const char *usb_bus_p[] = {
	"osc_25m", "sys2_pll_500m", "sys1_pll_800m", "sys2_pll_100m", "sys2_pll_200m", "clk_ext2", "clk_ext4", "audio_pll2_out"
};
static const char *usb_core_phy_p[] = {
	"osc_25m", "sys1_pll_100m", "sys1_pll_40m", "sys2_pll_100m", "sys2_pll_200m", "clk_ext2", "clk_ext3", "audio_pll2_out"
};
static const char *i2c_p[] = {
	"osc_25m", "sys1_pll_160m", "sys2_pll_50m", "sys3_pll_out", "audio_pll1_out", "video_pll1_out", "audio_pll2_out", "sys1_pll_133m"
};


static struct imx_clk imx_clks[] = {
	FIXED(IMX8MQ_CLK_DUMMY, "dummy", 0),

	LINK(IMX8MQ_CLK_32K, "ckil"),
        LINK(IMX8MQ_CLK_25M, "osc_25m"),
        LINK(IMX8MQ_CLK_27M, "osc_27m"),
        LINK(IMX8MQ_CLK_EXT1, "clk_ext1"),
        LINK(IMX8MQ_CLK_EXT2, "clk_ext2"),
        LINK(IMX8MQ_CLK_EXT3, "clk_ext3"),
        LINK(IMX8MQ_CLK_EXT4, "clk_ext4"),

	FIXED(IMX8MQ_SYS1_PLL_OUT, "sys1_pll_out", 800000000),
	FIXED(IMX8MQ_SYS2_PLL_OUT, "sys2_pll_out", 1000000000),
	SSCG_PLL(IMX8MQ_SYS3_PLL_OUT, "sys3_pll_out", sys3_pll_out_p, 0x48),
	
        MUX(IMX8MQ_ARM_PLL_REF_SEL, "arm_pll_ref_sel", pll_ref_p, 0, 0x28, 16, 2),
        MUX(IMX8MQ_GPU_PLL_REF_SEL, "gpu_pll_ref_sel", pll_ref_p, 0, 0x18, 16, 2),
        MUX(IMX8MQ_VPU_PLL_REF_SEL, "vpu_pll_ref_sel", pll_ref_p, 0, 0x20, 16, 2),
        MUX(IMX8MQ_AUDIO_PLL1_REF_SEL, "audio_pll1_ref_sel", pll_ref_p, 0, 0x0, 16, 2),
        MUX(IMX8MQ_AUDIO_PLL2_REF_SEL, "audio_pll2_ref_sel", pll_ref_p, 0, 0x8, 16, 2),
        MUX(IMX8MQ_VIDEO_PLL1_REF_SEL, "video_pll1_ref_sel", pll_ref_p, 0, 0x10, 16, 2),
        MUX(IMX8MQ_SYS3_PLL1_REF_SEL, "sys3_pll1_ref_sel", pll_ref_p, 0, 0x48, 0, 2),
        MUX(IMX8MQ_DRAM_PLL1_REF_SEL, "dram_pll1_ref_sel", pll_ref_p, 0, 0x60, 0, 2),
        MUX(IMX8MQ_VIDEO2_PLL1_REF_SEL, "video2_pll1_ref_sel", pll_ref_p, 0, 0x54, 0, 2),

	GATE(IMX8MQ_SYS1_PLL_40M_CG, "sys1_pll_40m_cg", "sys1_pll_out", 0x30, 9),
	GATE(IMX8MQ_SYS1_PLL_80M_CG, "sys1_pll_80m_cg", "sys1_pll_out", 0x30, 11),
	GATE(IMX8MQ_SYS1_PLL_100M_CG, "sys1_pll_100m_cg", "sys1_pll_out", 0x30, 13),
	GATE(IMX8MQ_SYS1_PLL_133M_CG, "sys1_pll_133m_cg", "sys1_pll_out", 0x30, 15),
	GATE(IMX8MQ_SYS1_PLL_160M_CG, "sys1_pll_160m_cg", "sys1_pll_out", 0x30, 17),
	GATE(IMX8MQ_SYS1_PLL_200M_CG, "sys1_pll_200m_cg", "sys1_pll_out", 0x30, 19),
	GATE(IMX8MQ_SYS1_PLL_266M_CG, "sys1_pll_266m_cg", "sys1_pll_out", 0x30, 21),
	GATE(IMX8MQ_SYS1_PLL_400M_CG, "sys1_pll_400m_cg", "sys1_pll_out", 0x30, 23),
	GATE(IMX8MQ_SYS1_PLL_800M_CG, "sys1_pll_800m_cg", "sys1_pll_out", 0x30, 25),

	FFACT(IMX8MQ_SYS1_PLL_40M, "sys1_pll_40m", "sys1_pll_40m_cg", 1, 20),
	FFACT(IMX8MQ_SYS1_PLL_80M, "sys1_pll_80m", "sys1_pll_80m_cg", 1, 10),
	FFACT(IMX8MQ_SYS1_PLL_100M, "sys1_pll_100m", "sys1_pll_100m_cg", 1, 8),
	FFACT(IMX8MQ_SYS1_PLL_133M, "sys1_pll_133m", "sys1_pll_133m_cg", 1, 6),
	FFACT(IMX8MQ_SYS1_PLL_160M, "sys1_pll_160m", "sys1_pll_160m_cg", 1, 5),
	FFACT(IMX8MQ_SYS1_PLL_200M, "sys1_pll_200m", "sys1_pll_200m_cg", 1, 4),
	FFACT(IMX8MQ_SYS1_PLL_266M, "sys1_pll_266m", "sys1_pll_266m_cg", 1, 3),
	FFACT(IMX8MQ_SYS1_PLL_400M, "sys1_pll_400m", "sys1_pll_400m_cg", 1, 2),
	FFACT(IMX8MQ_SYS1_PLL_800M, "sys1_pll_800m", "sys1_pll_800m_cg", 1, 1),

	GATE(IMX8MQ_SYS2_PLL_50M_CG, "sys2_pll_50m_cg", "sys2_pll_out", 0x3c, 9),
	GATE(IMX8MQ_SYS2_PLL_100M_CG, "sys2_pll_100m_cg", "sys2_pll_out", 0x3c, 11),
	GATE(IMX8MQ_SYS2_PLL_125M_CG, "sys2_pll_125m_cg", "sys2_pll_out", 0x3c, 13),
	GATE(IMX8MQ_SYS2_PLL_166M_CG, "sys2_pll_166m_cg", "sys2_pll_out", 0x3c, 15),
	GATE(IMX8MQ_SYS2_PLL_200M_CG, "sys2_pll_200m_cg", "sys2_pll_out", 0x3c, 17),
	GATE(IMX8MQ_SYS2_PLL_250M_CG, "sys2_pll_250m_cg", "sys2_pll_out", 0x3c, 19),
	GATE(IMX8MQ_SYS2_PLL_333M_CG, "sys2_pll_333m_cg", "sys2_pll_out", 0x3c, 21),
	GATE(IMX8MQ_SYS2_PLL_500M_CG, "sys2_pll_500m_cg", "sys2_pll_out", 0x3c, 23),
	GATE(IMX8MQ_SYS2_PLL_1000M_CG, "sys2_pll_1000m_cg", "sys2_pll_out", 0x3c, 25),

	FFACT(IMX8MQ_SYS2_PLL_50M, "sys2_pll_50m", "sys2_pll_50m_cg", 1, 20),
	FFACT(IMX8MQ_SYS2_PLL_100M, "sys2_pll_100m", "sys2_pll_100m_cg", 1, 10),
	FFACT(IMX8MQ_SYS2_PLL_125M, "sys2_pll_125m", "sys2_pll_125m_cg", 1, 8),
	FFACT(IMX8MQ_SYS2_PLL_166M, "sys2_pll_166m", "sys2_pll_166m_cg", 1, 6),
	FFACT(IMX8MQ_SYS2_PLL_200M, "sys2_pll_200m", "sys2_pll_200m_cg", 1, 5),
	FFACT(IMX8MQ_SYS2_PLL_250M, "sys2_pll_250m", "sys2_pll_250m_cg", 1, 4),
	FFACT(IMX8MQ_SYS2_PLL_333M, "sys2_pll_333m", "sys2_pll_333m_cg", 1, 3),
	FFACT(IMX8MQ_SYS2_PLL_500M, "sys2_pll_500m", "sys2_pll_500m_cg", 1, 2),
	FFACT(IMX8MQ_SYS2_PLL_1000M, "sys2_pll_1000m", "sys2_pll_1000m_cg", 1, 1),

	COMPOSITE(IMX8MQ_CLK_UART1, "uart1", uart_p, 0xaf00, 0),
	COMPOSITE(IMX8MQ_CLK_UART2, "uart2", uart_p, 0xaf80, 0),
	COMPOSITE(IMX8MQ_CLK_UART3, "uart3", uart_p, 0xb000, 0),
	COMPOSITE(IMX8MQ_CLK_UART4, "uart4", uart_p, 0xb080, 0),

	ROOT_GATE(IMX8MQ_CLK_UART1_ROOT, "uart1_root_clk", "uart1", 0x4490),
	ROOT_GATE(IMX8MQ_CLK_UART2_ROOT, "uart2_root_clk", "uart2", 0x44a0),
	ROOT_GATE(IMX8MQ_CLK_UART3_ROOT, "uart3_root_clk", "uart3", 0x44b0),
	ROOT_GATE(IMX8MQ_CLK_UART4_ROOT, "uart4_root_clk", "uart4", 0x44c0),

	COMPOSITE(IMX8MQ_CLK_USDHC1, "usdhc1", usdhc_p, 0xac00, CLK_SET_ROUND_DOWN),
	COMPOSITE(IMX8MQ_CLK_USDHC2, "usdhc2", usdhc_p, 0xac80, CLK_SET_ROUND_DOWN),

	ROOT_GATE(IMX8MQ_CLK_USDHC1_ROOT, "usdhc1_root_clk", "usdhc1", 0x4510),
	ROOT_GATE(IMX8MQ_CLK_USDHC2_ROOT, "usdhc2_root_clk", "usdhc2", 0x4520),

	COMPOSITE(IMX8MQ_CLK_ENET_AXI, "enet_axi", enet_axi_p, 0x8800, 0),
	COMPOSITE(IMX8MQ_CLK_ENET_REF, "enet_ref", enet_ref_p, 0xa980, 0),
	COMPOSITE(IMX8MQ_CLK_ENET_TIMER, "enet_timer", enet_timer_p, 0xaa00, 0),
	COMPOSITE(IMX8MQ_CLK_ENET_PHY_REF, "enet_phy_ref", enet_phy_ref_p, 0xaa80, 0),

	ROOT_GATE(IMX8MQ_CLK_ENET1_ROOT, "enet1_root_clk", "enet_axi", 0x40a0),

	COMPOSITE(IMX8MQ_CLK_USB_BUS, "usb_bus", usb_bus_p, 0x8b80, 0),
	COMPOSITE(IMX8MQ_CLK_USB_CORE_REF, "usb_core_ref", usb_core_phy_p, 0xb100, 0),
	COMPOSITE(IMX8MQ_CLK_USB_PHY_REF, "usb_phy_ref", usb_core_phy_p, 0xb180, 0),

	ROOT_GATE(IMX8MQ_CLK_USB1_CTRL_ROOT, "usb1_ctrl_root_clk", "usb_bus", 0x44d0),
	ROOT_GATE(IMX8MQ_CLK_USB2_CTRL_ROOT, "usb2_ctrl_root_clk", "usb_bus", 0x44e0),
	ROOT_GATE(IMX8MQ_CLK_USB1_PHY_ROOT, "usb1_phy_root_clk", "usb_phy_ref", 0x44f0),
	ROOT_GATE(IMX8MQ_CLK_USB2_PHY_ROOT, "usb2_phy_root_clk", "usb_phy_ref", 0x4500),

	COMPOSITE(IMX8MQ_CLK_I2C1, "i2c1", i2c_p, 0xad00, 0),
	COMPOSITE(IMX8MQ_CLK_I2C2, "i2c2", i2c_p, 0xad80, 0),
	COMPOSITE(IMX8MQ_CLK_I2C3, "i2c3", i2c_p, 0xae00, 0),
	COMPOSITE(IMX8MQ_CLK_I2C4, "i2c4", i2c_p, 0xae80, 0),

	ROOT_GATE(IMX8MQ_CLK_I2C1_ROOT, "i2c1_root_clk", "i2c1", 0x4170),
	ROOT_GATE(IMX8MQ_CLK_I2C2_ROOT, "i2c2_root_clk", "i2c2", 0x4180),
	ROOT_GATE(IMX8MQ_CLK_I2C3_ROOT, "i2c3_root_clk", "i2c3", 0x4190),
	ROOT_GATE(IMX8MQ_CLK_I2C4_ROOT, "i2c4_root_clk", "i2c4", 0x41a0),
};

struct ccm_softc {
	device_t	dev;
	struct resource	*mem_res;
	struct clkdom		*clkdom;
	struct mtx		mtx;
	// struct imx_ccm_gate	*gates;
	// int			ngates;
	struct imx_clk		*clks;
	int			nclks;
};

static inline uint32_t
CCU_READ4(struct ccm_softc *sc, bus_size_t off)
{

	return (bus_read_4(sc->mem_res, off));
}

static inline void
CCU_WRITE4(struct ccm_softc *sc, bus_size_t off, uint32_t val)
{

	bus_write_4(sc->mem_res, off, val);
}

static int
ccm_detach(device_t dev)
{
	struct ccm_softc *sc;

	sc = device_get_softc(dev);

	if (sc->mem_res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem_res);

	return (0);
}

static int
ccm_attach(device_t dev)
{
	struct ccm_softc *sc;
	int err, rid;
	phandle_t node;
	int i;

	sc = device_get_softc(dev);
	err = 0;

	/* Allocate bus_space resources. */
	rid = 0;
	sc->clks = imx_clks;
	sc->nclks = nitems(imx_clks);
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "Cannot allocate memory resources\n");
		err = ENXIO;
		goto out;
	}

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	sc->clkdom = clkdom_create(dev);
	if (sc->clkdom == NULL)
		panic("Cannot create clkdom\n");

	for (i = 0; i < sc->nclks; i++) {
		switch (sc->clks[i].type) {
		case IMX_CLK_UNDEFINED:
			break;
		case IMX_CLK_LINK:
			clknode_link_register(sc->clkdom,
			    sc->clks[i].clk.link);
			break;
		case IMX_CLK_FIXED:
			clknode_fixed_register(sc->clkdom,
			    sc->clks[i].clk.fixed);
			break;
		case IMX_CLK_MUX:
			imx_clk_mux_register(sc->clkdom, sc->clks[i].clk.mux);
			break;
		case IMX_CLK_GATE:
			imx_clk_gate_register(sc->clkdom, sc->clks[i].clk.gate);
			break;
		case IMX_CLK_COMPOSITE:
			imx_clk_composite_register(sc->clkdom, sc->clks[i].clk.composite);
			break;
		case IMX_CLK_SSCG_PLL:
			imx_clk_sscg_pll_register(sc->clkdom, sc->clks[i].clk.sscg_pll);
			break;
		case IMX_CLK_FRAC_PLL:
			imx_clk_frac_pll_register(sc->clkdom, sc->clks[i].clk.frac_pll);
			break;
		default:
			device_printf(dev, "Unknown clock type %d\n", sc->clks[i].type);
			return (ENXIO);
		}
	}

#if 0
	if (sc->gates)
		imx_ccm_register_gates(sc);
#endif

	if (clkdom_finit(sc->clkdom) != 0)
		panic("cannot finalize clkdom initialization\n");

	if (bootverbose)
		clkdom_dump(sc->clkdom);

	node = ofw_bus_get_node(dev);
	clk_set_assigned(dev, node);

	err = 0;

out:

	if (err != 0)
		ccm_detach(dev);

	return (err);
}

static int
ccm_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

        if (ofw_bus_is_compatible(dev, "fsl,imx8mq-ccm") == 0)
		return (ENXIO);

	device_set_desc(dev, "Freescale i.MX8 Clock Control Module");

	return (BUS_PROBE_DEFAULT);
}

static int
imx_ccm_write_4(device_t dev, bus_addr_t addr, uint32_t val)
{
	struct ccm_softc *sc;

	sc = device_get_softc(dev);
	CCU_WRITE4(sc, addr, val);
	return (0);
}

static int
imx_ccm_read_4(device_t dev, bus_addr_t addr, uint32_t *val)
{
	struct ccm_softc *sc;

	sc = device_get_softc(dev);

	*val = CCU_READ4(sc, addr);
	return (0);
}

static int
imx_ccm_modify_4(device_t dev, bus_addr_t addr, uint32_t clr, uint32_t set)
{
	struct ccm_softc *sc;
	uint32_t reg;

	sc = device_get_softc(dev);

	reg = CCU_READ4(sc, addr);
	reg &= ~clr;
	reg |= set;
	CCU_WRITE4(sc, addr, reg);

	return (0);
}

static void
imx_ccm_device_lock(device_t dev)
{
	struct ccm_softc *sc;

	sc = device_get_softc(dev);
	mtx_lock(&sc->mtx);
}

static void
imx_ccm_device_unlock(device_t dev)
{
	struct ccm_softc *sc;

	sc = device_get_softc(dev);
	mtx_unlock(&sc->mtx);
}


static device_method_t ccm_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,  ccm_probe),
	DEVMETHOD(device_attach, ccm_attach),
	DEVMETHOD(device_detach, ccm_detach),

	/* clkdev interface */
	DEVMETHOD(clkdev_write_4,	imx_ccm_write_4),
	DEVMETHOD(clkdev_read_4,	imx_ccm_read_4),
	DEVMETHOD(clkdev_modify_4,	imx_ccm_modify_4),
	DEVMETHOD(clkdev_device_lock,	imx_ccm_device_lock),
	DEVMETHOD(clkdev_device_unlock,	imx_ccm_device_unlock),

	DEVMETHOD_END
};

static driver_t ccm_driver = {
	"ccm",
	ccm_methods,
	sizeof(struct ccm_softc)
};

static devclass_t ccm_devclass;

EARLY_DRIVER_MODULE(ccm, simplebus, ccm_driver, ccm_devclass, 0, 0, 
    BUS_PASS_CPU + BUS_PASS_ORDER_EARLY);
