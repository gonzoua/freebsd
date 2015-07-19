/*-
 * Copyright (c) 2015 Oleksandr Tymoshenko <gonzo@freebsd.org>
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
 * HDMI core module
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/rman.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <machine/bus.h>

#include <dev/videomode/videomode.h>
#include <dev/videomode/edidvar.h>

#include <dev/iicbus/iicbus.h>
#include <dev/iicbus/iiconf.h>

#include <arm/freescale/imx/imx6_hdmi.h>
#include <arm/freescale/imx/imx6_hdmi_regs.h>

#include <arm/freescale/imx/imx_iomuxvar.h>

#include <arm/ti/am335x/hdmi.h>
#include "hdmi_if.h"

#define	I2C_DDC_ADDR	(0x50 << 1)
#define	EDID_LENGTH	0x80

struct imx_hdmi_softc {
	device_t		sc_dev;
	struct resource		*sc_mem_res;
	int			sc_mem_rid;
	struct resource		*sc_irq_res;
	int			sc_irq_rid;
	void			*sc_intr_hl;
	struct intr_config_hook	sc_mode_hook;
	struct videomode	sc_mode;
	uint8_t			*sc_edid;
	uint8_t			sc_edid_len;
	phandle_t		sc_i2c_xref;
};

static struct ofw_compat_data compat_data[] = {
	{"fsl,imx6dl-hdmi", 1},
	{"fsl,imx6q-hdmi",  1},
	{NULL,	            0}
};

static struct imx_hdmi_softc *imx_hdmi_sc;

static inline uint8_t
RD1(struct imx_hdmi_softc *sc, bus_size_t off)
{

	return (bus_read_1(sc->sc_mem_res, off));
}

static inline void
WR1(struct imx_hdmi_softc *sc, bus_size_t off, uint8_t val)
{

	bus_write_1(sc->sc_mem_res, off, val);
}

static uint8_t
imx_hdmi_read_1(bus_size_t off)
{

	return RD1(imx_hdmi_sc, off);
}

static void
imx_hdmi_write_1(bus_size_t off, uint8_t val)
{

	WR1(imx_hdmi_sc, off, val);
}

static void
imx_hdmi_phy_wait_i2c_done(struct imx_hdmi_softc *sc, int msec)
{
	unsigned char val = 0;
	val = imx_hdmi_read_1(HDMI_IH_I2CMPHY_STAT0) & 0x3;
	while (val == 0) {
		DELAY(1000);
		if (msec-- == 0)
			return;
		val = imx_hdmi_read_1(HDMI_IH_I2CMPHY_STAT0) & 0x3;
	}
}

static void imx_hdmi_phy_i2c_write(struct imx_hdmi_softc *sc, unsigned short data,
			      unsigned char addr)
{
	imx_hdmi_write_1(HDMI_IH_I2CMPHY_STAT0, 0xFF);
	imx_hdmi_write_1(HDMI_PHY_I2CM_ADDRESS_ADDR, addr);
	imx_hdmi_write_1(HDMI_PHY_I2CM_DATAO_1_ADDR, (unsigned char)(data >> 8));
	imx_hdmi_write_1(HDMI_PHY_I2CM_DATAO_0_ADDR, (unsigned char)(data >> 0));
	imx_hdmi_write_1(HDMI_PHY_I2CM_OPERATION_ADDR, HDMI_PHY_I2CM_OPERATION_ADDR_WRITE);
	imx_hdmi_phy_wait_i2c_done(sc, 1000);
}

static void
imx_hdmi_disable_overflow_interrupts(struct imx_hdmi_softc *sc)
{
	imx_hdmi_write_1(HDMI_IH_MUTE_FC_STAT2, HDMI_IH_MUTE_FC_STAT2_OVERFLOW_MASK);
	imx_hdmi_write_1(HDMI_FC_MASK2, 0xff);
}

static void
imx_hdmi_av_composer(struct imx_hdmi_softc *sc)
{
	uint8_t inv_val;
	int hblank, vblank, hsync_len, hbp, vbp;

	/* Set up HDMI_FC_INVIDCONF */
	inv_val = ((sc->sc_mode.flags & VID_NVSYNC) ?
		HDMI_FC_INVIDCONF_VSYNC_IN_POLARITY_ACTIVE_LOW :
		HDMI_FC_INVIDCONF_VSYNC_IN_POLARITY_ACTIVE_HIGH);

	inv_val |= ((sc->sc_mode.flags & VID_NHSYNC) ?
		HDMI_FC_INVIDCONF_HSYNC_IN_POLARITY_ACTIVE_LOW :
		HDMI_FC_INVIDCONF_HSYNC_IN_POLARITY_ACTIVE_HIGH);

	inv_val |= HDMI_FC_INVIDCONF_DE_IN_POLARITY_ACTIVE_HIGH;

	inv_val |= ((sc->sc_mode.flags & VID_INTERLACE) ?
			HDMI_FC_INVIDCONF_R_V_BLANK_IN_OSC_ACTIVE_HIGH :
			HDMI_FC_INVIDCONF_R_V_BLANK_IN_OSC_ACTIVE_LOW);

	inv_val |= ((sc->sc_mode.flags & VID_INTERLACE) ?
		HDMI_FC_INVIDCONF_IN_I_P_INTERLACED :
		HDMI_FC_INVIDCONF_IN_I_P_PROGRESSIVE);

	inv_val |= (1 /*DVI*/ ?
		HDMI_FC_INVIDCONF_DVI_MODEZ_DVI_MODE :
		HDMI_FC_INVIDCONF_DVI_MODEZ_HDMI_MODE);

	imx_hdmi_write_1(HDMI_FC_INVIDCONF, inv_val);

	/* Set up horizontal active pixel region width */
	imx_hdmi_write_1(HDMI_FC_INHACTV1, sc->sc_mode.hdisplay >> 8);
	imx_hdmi_write_1(HDMI_FC_INHACTV0, sc->sc_mode.hdisplay);

	/* Set up vertical blanking pixel region width */
	imx_hdmi_write_1(HDMI_FC_INVACTV1, sc->sc_mode.vdisplay >> 8);
	imx_hdmi_write_1(HDMI_FC_INVACTV0, sc->sc_mode.vdisplay);

	/* Set up horizontal blanking pixel region width */
	hblank = sc->sc_mode.htotal - sc->sc_mode.hdisplay;
	imx_hdmi_write_1(HDMI_FC_INHBLANK1, hblank >> 8);
	imx_hdmi_write_1(HDMI_FC_INHBLANK0, hblank);

	/* Set up vertical blanking pixel region width */
	vblank = sc->sc_mode.vtotal - sc->sc_mode.vdisplay;
	imx_hdmi_write_1(HDMI_FC_INVBLANK, vblank);

	/* Set up HSYNC active edge delay width (in pixel clks) */
	hbp = sc->sc_mode.htotal - sc->sc_mode.hsync_end;
	imx_hdmi_write_1(HDMI_FC_HSYNCINDELAY1, hbp >> 8);
	imx_hdmi_write_1(HDMI_FC_HSYNCINDELAY0, hbp);

	/* Set up VSYNC active edge delay (in pixel clks) */
	vbp = sc->sc_mode.vtotal - sc->sc_mode.vsync_end;
	imx_hdmi_write_1(HDMI_FC_VSYNCINDELAY, vbp);

	hsync_len = (sc->sc_mode.hsync_end - sc->sc_mode.hsync_start);
	/* Set up HSYNC active pulse width (in pixel clks) */
	imx_hdmi_write_1(HDMI_FC_HSYNCINWIDTH1, hsync_len >> 8);
	imx_hdmi_write_1(HDMI_FC_HSYNCINWIDTH0, hsync_len);

	/* Set up VSYNC active edge delay (in pixel clks) */
	imx_hdmi_write_1(HDMI_FC_VSYNCINWIDTH, (sc->sc_mode.vsync_end - sc->sc_mode.vsync_start));
}

static void
imx_hdmi_phy_enable_power(struct imx_hdmi_softc *sc, uint8_t enable)
{
	uint8_t reg;

	reg = imx_hdmi_read_1(HDMI_PHY_CONF0);
	reg &= ~HDMI_PHY_CONF0_PDZ_MASK;
	reg |= (enable << HDMI_PHY_CONF0_PDZ_OFFSET);
	imx_hdmi_write_1(HDMI_PHY_CONF0, reg);
}

static void
imx_hdmi_phy_enable_tmds(struct imx_hdmi_softc *sc, uint8_t enable)
{
	uint8_t reg;

	reg = imx_hdmi_read_1(HDMI_PHY_CONF0);
	reg &= ~HDMI_PHY_CONF0_ENTMDS_MASK;
	reg |= (enable << HDMI_PHY_CONF0_ENTMDS_OFFSET);
	imx_hdmi_write_1(HDMI_PHY_CONF0, reg);
}

static void
imx_hdmi_phy_gen2_pddq(struct imx_hdmi_softc *sc, uint8_t enable)
{
	uint8_t reg;

	reg = imx_hdmi_read_1(HDMI_PHY_CONF0);
	reg &= ~HDMI_PHY_CONF0_GEN2_PDDQ_MASK;
	reg |= (enable << HDMI_PHY_CONF0_GEN2_PDDQ_OFFSET);
	imx_hdmi_write_1(HDMI_PHY_CONF0, reg);
}

static void
imx_hdmi_phy_gen2_txpwron(struct imx_hdmi_softc *sc, uint8_t enable)
{
	uint8_t reg;

	reg = imx_hdmi_read_1(HDMI_PHY_CONF0);
	reg &= ~HDMI_PHY_CONF0_GEN2_TXPWRON_MASK;
	reg |= (enable << HDMI_PHY_CONF0_GEN2_TXPWRON_OFFSET);
	imx_hdmi_write_1(HDMI_PHY_CONF0, reg);
}

static void
imx_hdmi_phy_sel_data_en_pol(struct imx_hdmi_softc *sc, uint8_t enable)
{
	uint8_t reg;

	reg = imx_hdmi_read_1(HDMI_PHY_CONF0);
	reg &= ~HDMI_PHY_CONF0_SELDATAENPOL_MASK;
	reg |= (enable << HDMI_PHY_CONF0_SELDATAENPOL_OFFSET);
	imx_hdmi_write_1(HDMI_PHY_CONF0, reg);
}

static void
imx_hdmi_phy_sel_interface_control(struct imx_hdmi_softc *sc, uint8_t enable)
{
	uint8_t reg;

	reg = imx_hdmi_read_1(HDMI_PHY_CONF0);
	reg &= ~HDMI_PHY_CONF0_SELDIPIF_MASK;
	reg |= (enable << HDMI_PHY_CONF0_SELDIPIF_OFFSET);
	imx_hdmi_write_1(HDMI_PHY_CONF0, reg);
}

static inline void 
imx_hdmi_phy_test_clear(struct imx_hdmi_softc *sc, unsigned char bit)
{
	uint8_t val;

	val = imx_hdmi_read_1(HDMI_PHY_TST0);
	val &= ~HDMI_PHY_TST0_TSTCLR_MASK;
	val |= (bit << HDMI_PHY_TST0_TSTCLR_OFFSET) &
		HDMI_PHY_TST0_TSTCLR_MASK;
	imx_hdmi_write_1(HDMI_PHY_TST0, val);
}


static void imx_hdmi_clear_overflow(struct imx_hdmi_softc *sc)
{
	int count;
	uint8_t val;

	/* TMDS software reset */
	imx_hdmi_write_1(HDMI_MC_SWRSTZ, (uint8_t)~HDMI_MC_SWRSTZ_TMDSSWRST_REQ);

	val = imx_hdmi_read_1(HDMI_FC_INVIDCONF);

	for (count = 0 ; count < 5 ; count++)
		imx_hdmi_write_1(HDMI_FC_INVIDCONF, val);
}


static int imx_hdmi_phy_configure(struct imx_hdmi_softc *sc)
{
	uint8_t val;
	uint8_t msec;

	imx_hdmi_write_1(HDMI_MC_FLOWCTRL, HDMI_MC_FLOWCTRL_FEED_THROUGH_OFF_CSC_BYPASS);

	/* gen2 tx power off */
	imx_hdmi_phy_gen2_txpwron(sc, 0);

	/* gen2 pddq */
	imx_hdmi_phy_gen2_pddq(sc, 1);

	/* PHY reset */
	imx_hdmi_write_1(HDMI_MC_PHYRSTZ, HDMI_MC_PHYRSTZ_DEASSERT);
	imx_hdmi_write_1(HDMI_MC_PHYRSTZ, HDMI_MC_PHYRSTZ_ASSERT);

	imx_hdmi_write_1(HDMI_MC_HEACPHY_RST, HDMI_MC_HEACPHY_RST_ASSERT);

	imx_hdmi_phy_test_clear(sc, 1);
	imx_hdmi_write_1(HDMI_PHY_I2CM_SLAVE_ADDR, HDMI_PHY_I2CM_SLAVE_ADDR_PHY_GEN2);
	imx_hdmi_phy_test_clear(sc, 0);

	if (sc->sc_mode.dot_clock*1000 <= 45250000) {
		/* PLL/MPLL Cfg */
		imx_hdmi_phy_i2c_write(sc, 0x01e0, 0x06);
		imx_hdmi_phy_i2c_write(sc, 0x0000, 0x15);  /* GMPCTRL */
	} else if (sc->sc_mode.dot_clock*1000 <= 92500000) {
		imx_hdmi_phy_i2c_write(sc, 0x0140, 0x06);
		imx_hdmi_phy_i2c_write(sc, 0x0005, 0x15);
	} else if (sc->sc_mode.dot_clock*1000 <= 148500000) {
		imx_hdmi_phy_i2c_write(sc, 0x00a0, 0x06);
		imx_hdmi_phy_i2c_write(sc, 0x000a, 0x15);
	} else {
		imx_hdmi_phy_i2c_write(sc, 0x00a0, 0x06);
		imx_hdmi_phy_i2c_write(sc, 0x000a, 0x15);
	}

	if (sc->sc_mode.dot_clock*1000 <= 54000000) {
		imx_hdmi_phy_i2c_write(sc, 0x091c, 0x10);  /* CURRCTRL */
	} else if (sc->sc_mode.dot_clock*1000 <= 58400000) {
		imx_hdmi_phy_i2c_write(sc, 0x091c, 0x10);
	} else if (sc->sc_mode.dot_clock*1000 <= 72000000) {
		imx_hdmi_phy_i2c_write(sc, 0x06dc, 0x10);
	} else if (sc->sc_mode.dot_clock*1000 <= 74250000) {
		imx_hdmi_phy_i2c_write(sc, 0x06dc, 0x10);
	} else if (sc->sc_mode.dot_clock*1000 <= 118800000) {
		imx_hdmi_phy_i2c_write(sc, 0x091c, 0x10);
	} else if (sc->sc_mode.dot_clock*1000 <= 216000000) {
		imx_hdmi_phy_i2c_write(sc, 0x06dc, 0x10);
	} else {
		panic("Unsupported mode\n");
	}

	imx_hdmi_phy_i2c_write(sc, 0x0000, 0x13);  /* PLLPHBYCTRL */
	imx_hdmi_phy_i2c_write(sc, 0x0006, 0x17);
	/* RESISTANCE TERM 133Ohm Cfg */
	imx_hdmi_phy_i2c_write(sc, 0x0005, 0x19);  /* TXTERM */
	/* PREEMP Cgf 0.00 */
	imx_hdmi_phy_i2c_write(sc, 0x800d, 0x09);  /* CKSYMTXCTRL */
	/* TX/CK LVL 10 */
	imx_hdmi_phy_i2c_write(sc, 0x01ad, 0x0E);  /* VLEVCTRL */

	/* REMOVE CLK TERM */
	imx_hdmi_phy_i2c_write(sc, 0x8000, 0x05);  /* CKCALCTRL */

	if (sc->sc_mode.dot_clock*1000 > 148500000) {
		imx_hdmi_phy_i2c_write(sc, 0x800b, 0x09);
		imx_hdmi_phy_i2c_write(sc, 0x0129, 0x0E);
	}

	imx_hdmi_phy_enable_power(sc, 1);

	/* toggle TMDS enable */
	imx_hdmi_phy_enable_tmds(sc, 0);
	imx_hdmi_phy_enable_tmds(sc, 1);

	/* gen2 tx power on */
	imx_hdmi_phy_gen2_txpwron(sc, 1);
	imx_hdmi_phy_gen2_pddq(sc, 0);

	/*Wait for PHY PLL lock */
	msec = 4;
	val = imx_hdmi_read_1(HDMI_PHY_STAT0) & HDMI_PHY_TX_PHY_LOCK;
	while (val == 0) {
		DELAY(1000);
		if (msec-- == 0) {
			device_printf(sc->sc_dev, "PHY PLL not locked\n");
			return (-1);
		}
		val = imx_hdmi_read_1(HDMI_PHY_STAT0) & HDMI_PHY_TX_PHY_LOCK;
	}

	return true;
}

static void imx_hdmi_phy_init(struct imx_hdmi_softc *sc)
{
	int i;

	/* HDMI Phy spec says to do the phy initialization sequence twice */
	for (i = 0 ; i < 2 ; i++) {
		imx_hdmi_phy_sel_data_en_pol(sc, 1);
		imx_hdmi_phy_sel_interface_control(sc, 0);
		imx_hdmi_phy_enable_tmds(sc, 0);
		imx_hdmi_phy_enable_power(sc, 0);

		/* Enable CSC */
		imx_hdmi_phy_configure(sc);
	}
}

static void imx_hdmi_enable_video_path(struct imx_hdmi_softc *sc)
{
	uint8_t clkdis;

	/* control period minimum duration */
	imx_hdmi_write_1(HDMI_FC_CTRLDUR, 12);
	imx_hdmi_write_1(HDMI_FC_EXCTRLDUR, 32);
	imx_hdmi_write_1(HDMI_FC_EXCTRLSPAC, 1);

	/* Set to fill TMDS data channels */
	imx_hdmi_write_1(HDMI_FC_CH0PREAM, 0x0B);
	imx_hdmi_write_1(HDMI_FC_CH1PREAM, 0x16);
	imx_hdmi_write_1(HDMI_FC_CH2PREAM, 0x21);

	/* Save CEC clock */
	clkdis = imx_hdmi_read_1(HDMI_MC_CLKDIS) & HDMI_MC_CLKDIS_CECCLK_DISABLE;
	clkdis |= ~HDMI_MC_CLKDIS_CECCLK_DISABLE;

	/* Enable pixel clock and tmds data path */
	clkdis = 0x7F & clkdis;
	clkdis &= ~HDMI_MC_CLKDIS_PIXELCLK_DISABLE;
	imx_hdmi_write_1(HDMI_MC_CLKDIS, clkdis);

	clkdis &= ~HDMI_MC_CLKDIS_TMDSCLK_DISABLE;
	imx_hdmi_write_1(HDMI_MC_CLKDIS, clkdis);
}

static void imx_hdmi_video_packetize(struct imx_hdmi_softc *sc)
{
	unsigned int color_depth = 0;
	unsigned int remap_size = HDMI_VP_REMAP_YCC422_16BIT;
	unsigned int output_select = HDMI_VP_CONF_OUTPUT_SELECTOR_PP;
	uint8_t val;

	output_select = HDMI_VP_CONF_OUTPUT_SELECTOR_BYPASS;
	color_depth = 0;

	/* set the packetizer registers */
	val = ((color_depth << HDMI_VP_PR_CD_COLOR_DEPTH_OFFSET) &
		HDMI_VP_PR_CD_COLOR_DEPTH_MASK);
	imx_hdmi_write_1(HDMI_VP_PR_CD, val);

	val = imx_hdmi_read_1(HDMI_VP_STUFF);
	val &= ~HDMI_VP_STUFF_PR_STUFFING_MASK;
	val |= HDMI_VP_STUFF_PR_STUFFING_STUFFING_MODE;
	imx_hdmi_write_1(HDMI_VP_STUFF, val);

	val = imx_hdmi_read_1(HDMI_VP_CONF);
	val &= ~(HDMI_VP_CONF_PR_EN_MASK |
		HDMI_VP_CONF_BYPASS_SELECT_MASK);
	val |= HDMI_VP_CONF_PR_EN_DISABLE |
		HDMI_VP_CONF_BYPASS_SELECT_VID_PACKETIZER;
	imx_hdmi_write_1(HDMI_VP_CONF, val);

	val = imx_hdmi_read_1(HDMI_VP_STUFF);
	val &= ~HDMI_VP_STUFF_IDEFAULT_PHASE_MASK;
	val |= 1 << HDMI_VP_STUFF_IDEFAULT_PHASE_OFFSET;
	imx_hdmi_write_1(HDMI_VP_STUFF, val);

	imx_hdmi_write_1(HDMI_VP_REMAP, remap_size);

	if (output_select == HDMI_VP_CONF_OUTPUT_SELECTOR_PP) {
		val = imx_hdmi_read_1(HDMI_VP_CONF);
		val &= ~(HDMI_VP_CONF_BYPASS_EN_MASK |
			HDMI_VP_CONF_PP_EN_ENMASK |
			HDMI_VP_CONF_YCC422_EN_MASK);
		val |= HDMI_VP_CONF_BYPASS_EN_DISABLE |
			HDMI_VP_CONF_PP_EN_ENABLE |
			HDMI_VP_CONF_YCC422_EN_DISABLE;
		imx_hdmi_write_1(HDMI_VP_CONF, val);
	} else if (output_select == HDMI_VP_CONF_OUTPUT_SELECTOR_YCC422) {
		val = imx_hdmi_read_1(HDMI_VP_CONF);
		val &= ~(HDMI_VP_CONF_BYPASS_EN_MASK |
			HDMI_VP_CONF_PP_EN_ENMASK |
			HDMI_VP_CONF_YCC422_EN_MASK);
		val |= HDMI_VP_CONF_BYPASS_EN_DISABLE |
			HDMI_VP_CONF_PP_EN_DISABLE |
			HDMI_VP_CONF_YCC422_EN_ENABLE;
		imx_hdmi_write_1(HDMI_VP_CONF, val);
	} else if (output_select == HDMI_VP_CONF_OUTPUT_SELECTOR_BYPASS) {
		val = imx_hdmi_read_1(HDMI_VP_CONF);
		val &= ~(HDMI_VP_CONF_BYPASS_EN_MASK |
			HDMI_VP_CONF_PP_EN_ENMASK |
			HDMI_VP_CONF_YCC422_EN_MASK);
		val |= HDMI_VP_CONF_BYPASS_EN_ENABLE |
			HDMI_VP_CONF_PP_EN_DISABLE |
			HDMI_VP_CONF_YCC422_EN_DISABLE;
		imx_hdmi_write_1(HDMI_VP_CONF, val);
	} else {
		return;
	}

	val = imx_hdmi_read_1(HDMI_VP_STUFF);
	val &= ~(HDMI_VP_STUFF_PP_STUFFING_MASK |
		HDMI_VP_STUFF_YCC422_STUFFING_MASK);
	val |= HDMI_VP_STUFF_PP_STUFFING_STUFFING_MODE |
		HDMI_VP_STUFF_YCC422_STUFFING_STUFFING_MODE;
	imx_hdmi_write_1(HDMI_VP_STUFF, val);

	val = imx_hdmi_read_1(HDMI_VP_CONF);
	val &= ~HDMI_VP_CONF_OUTPUT_SELECTOR_MASK;
	val |= output_select;
	imx_hdmi_write_1(HDMI_VP_CONF, val);
}

static void imx_hdmi_video_sample(struct imx_hdmi_softc *sc)
{
	int color_format;
	uint8_t val;

	color_format = 0x01;
	val = HDMI_TX_INVID0_INTERNAL_DE_GENERATOR_DISABLE |
		((color_format << HDMI_TX_INVID0_VIDEO_MAPPING_OFFSET) &
		HDMI_TX_INVID0_VIDEO_MAPPING_MASK);
	imx_hdmi_write_1(HDMI_TX_INVID0, val);

	/* Enable TX stuffing: When DE is inactive, fix the output data to 0 */
	val = HDMI_TX_INSTUFFING_BDBDATA_STUFFING_ENABLE |
		HDMI_TX_INSTUFFING_RCRDATA_STUFFING_ENABLE |
		HDMI_TX_INSTUFFING_GYDATA_STUFFING_ENABLE;
	imx_hdmi_write_1(HDMI_TX_INSTUFFING, val);
	imx_hdmi_write_1(HDMI_TX_GYDATA0, 0x0);
	imx_hdmi_write_1(HDMI_TX_GYDATA1, 0x0);
	imx_hdmi_write_1(HDMI_TX_RCRDATA0, 0x0);
	imx_hdmi_write_1(HDMI_TX_RCRDATA1, 0x0);
	imx_hdmi_write_1(HDMI_TX_BCBDATA0, 0x0);
	imx_hdmi_write_1(HDMI_TX_BCBDATA1, 0x0);
}

static int
imx_hdmi_set_mode(struct imx_hdmi_softc *sc)
{
	imx_hdmi_disable_overflow_interrupts(sc);
	imx_hdmi_av_composer(sc);
	imx_hdmi_phy_init(sc);
	imx_hdmi_enable_video_path(sc);
	// TODO: AVI infoframes 
	imx_hdmi_video_packetize(sc);
	// imx_hdmi_video_csc(sc);
	imx_hdmi_video_sample(sc);
	imx_hdmi_clear_overflow(sc);

	return (0);
}

static int
hdmi_edid_read(struct imx_hdmi_softc *sc, uint8_t **edid, uint32_t *edid_len)
{
	device_t i2c_hc;
	device_t i2c_bus;
	device_t i2c_dev;
	int result;
	uint8_t addr = 0;
	struct iic_msg msg[] = {
		{ 0, IIC_M_WR, 1, &addr },
		{ 0, IIC_M_RD, EDID_LENGTH, NULL}
	};

	*edid = NULL;
	*edid_len = 0;

	if (sc->sc_i2c_xref == 0)
		return (ENXIO);

	/* XXX: HACK! HACK! HACK! */
	i2c_dev = NULL;
	i2c_bus = NULL;
	i2c_hc = OF_device_from_xref(sc->sc_i2c_xref);
	if (i2c_hc)
		i2c_bus = device_find_child(i2c_hc, "iicbus", 0);
	if (i2c_bus)
		i2c_dev = device_find_child(i2c_bus, "iic", 0);

	if (!i2c_dev) {
		device_printf(sc->sc_dev,
		    "no actual device for \"ddc-i2c-bus\" property (handle=%x)\n", sc->sc_i2c_xref);
		return (ENXIO);
	}



	msg[0].slave = I2C_DDC_ADDR;
	msg[1].slave = I2C_DDC_ADDR;
	msg[1].buf = sc->sc_edid;

	result =  iicbus_transfer(i2c_dev, msg, 2);
	if (result) {
		device_printf(sc->sc_dev, "i2c transfer failed: %d\n", result);
		return (result);
	}
	else {
		*edid_len = sc->sc_edid_len;
		*edid = sc->sc_edid;
	}
#if 0
	for (i = 0; i < EDID_LENGTH; i++) {
		printf("%02x ", edid[i]);
		if ((i % 0x10) == 0xf) {
			printf("\n");
		}
	}
	printf("\n");
#endif

	return (result);
}

static void
imx_hdmi_detect_cable(void *arg)
{
	struct imx_hdmi_softc *sc;

	sc = arg;
	EVENTHANDLER_INVOKE(hdmi_event, sc->sc_dev);
	/* Finished with the interrupt hook */
	config_intrhook_disestablish(&sc->sc_mode_hook);
}

static void
imx_hdmi_intr(void *arg)
{
	panic("INTERRUPT");
}



static int
imx_hdmi_detach(device_t dev)
{
	struct imx_hdmi_softc *sc;

	sc = device_get_softc(dev);

	if (sc->sc_mem_res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->sc_mem_res);

	if (sc->sc_intr_hl)
		bus_teardown_intr(dev, sc->sc_irq_res, sc->sc_intr_hl);

	if (sc->sc_irq_res != NULL)
		bus_release_resource(dev, SYS_RES_IRQ, sc->sc_irq_rid, sc->sc_irq_res);

	return (0);
}

static int
imx_hdmi_attach(device_t dev)
{
	struct imx_hdmi_softc *sc;
	int err;
	uint32_t gpr3;
	int ipu_id, disp_id;
	phandle_t node, i2c_xref;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;
	err = 0;

	/* Allocate memory resources. */
	sc->sc_mem_rid = 0;
	sc->sc_mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->sc_mem_rid,
	    RF_ACTIVE);
	if (sc->sc_mem_res == NULL) {
		device_printf(dev, "Cannot allocate memory resources\n");
		err = ENXIO;
		goto out;
	}

	/* Allocate bus_space resources. */
	sc->sc_irq_rid = 0;
	sc->sc_irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &sc->sc_irq_rid,
	    RF_ACTIVE);
	if (sc->sc_irq_res == NULL) {
		device_printf(dev, "No IRQ\n");
		err = ENXIO;
		goto out;
	}

	if (bus_setup_intr(dev, sc->sc_irq_res, INTR_TYPE_MISC | INTR_MPSAFE,
			NULL, imx_hdmi_intr, sc,
			&sc->sc_intr_hl) != 0) {
		device_printf(dev, "Unable to setup the irq handler.\n");
		err = ENXIO;
		goto out;
	}

	sc->sc_mode_hook.ich_func = imx_hdmi_detect_cable;
	sc->sc_mode_hook.ich_arg = sc;

	if (config_intrhook_establish(&sc->sc_mode_hook) != 0) {
		err = ENOMEM;
		goto out;
	}

	node = ofw_bus_get_node(dev);
	if (OF_getencprop(node, "ddc-i2c-bus", &i2c_xref, sizeof(i2c_xref)) == -1)
		sc->sc_i2c_xref = 0;
	else
		sc->sc_i2c_xref = i2c_xref; 

	err = 0;

	sc->sc_edid = malloc(EDID_LENGTH, M_DEVBUF, M_WAITOK | M_ZERO);
	sc->sc_edid_len = EDID_LENGTH;

	device_printf(sc->sc_dev, "HDMI controller %02x:%02x:%02x:%02x\n", 
	    RD1(sc, HDMI_DESIGN_ID), RD1(sc, HDMI_REVISION_ID),
	    RD1(sc, HDMI_PRODUCT_ID0), RD1(sc, HDMI_PRODUCT_ID1));

	ipu_id = 0;
	disp_id = 0;

	gpr3 = imx_iomux_gpr_get(12);
	printf("GPR3 %08x -> ", gpr3);
	gpr3 &= ~0x0d;
	gpr3 |= ((ipu_id << 1) | disp_id) << 2;
	printf("%08x\n", gpr3);
	imx_iomux_gpr_set(12, gpr3);

	WR1(sc, HDMI_PHY_POL0, HDMI_PHY_HPD);
	WR1(sc, HDMI_IH_PHY_STAT0, HDMI_IH_PHY_STAT0_HPD);

	imx_hdmi_sc = sc;
out:

	if (err != 0)
		imx_hdmi_detach(dev);

	return (err);
}

static int
imx_hdmi_probe(device_t dev)
{

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Freescale i.MX6 HDMI core");

	return (BUS_PROBE_DEFAULT);
}

static int
imx_hdmi_get_edid(device_t dev, uint8_t **edid, uint32_t *edid_len)
{
	struct imx_hdmi_softc *sc;

	sc = device_get_softc(dev);

	return hdmi_edid_read(sc, edid, edid_len);
}

static int
imx_hdmi_set_videomode(device_t dev, const struct videomode *mode)
{
	struct imx_hdmi_softc *sc;

	sc = device_get_softc(dev);
	memcpy(&sc->sc_mode, mode, sizeof(*mode));
	imx_hdmi_set_mode(sc);

	return (0);
}

static device_method_t imx_hdmi_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,  imx_hdmi_probe),
	DEVMETHOD(device_attach, imx_hdmi_attach),
	DEVMETHOD(device_detach, imx_hdmi_detach),

	/* HDMI methods */
	DEVMETHOD(hdmi_get_edid,	imx_hdmi_get_edid),
	DEVMETHOD(hdmi_set_videomode,	imx_hdmi_set_videomode),

	DEVMETHOD_END
};

static driver_t imx_hdmi_driver = {
	"hdmi",
	imx_hdmi_methods,
	sizeof(struct imx_hdmi_softc)
};

static devclass_t imx_hdmi_devclass;

DRIVER_MODULE(hdmi, simplebus, imx_hdmi_driver, imx_hdmi_devclass, 0, 0);
