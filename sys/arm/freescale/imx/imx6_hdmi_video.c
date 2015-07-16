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

#include <dev/videomode/videomode.h>
#include <dev/videomode/edidvar.h>

#include <machine/bus.h>

#include <arm/freescale/imx/imx_iomuxvar.h>

#include <arm/freescale/imx/imx6_hdmi.h>
#include <arm/freescale/imx/imx6_hdmi_regs.h>

struct hdmi_video_softc {
	device_t	dev;
	struct resource	*irq_res;
	int		irq_rid;
	struct intr_config_hook	mode_hook;
	const struct videomode *mode;

	uint16_t	phy_reg_vlev;
	uint16_t	phy_reg_cksymtx;
};

/*
 * These macros help the modelines below fit on one line.
 */
#define HP VID_PHSYNC
#define HN VID_NHSYNC
#define VP VID_PVSYNC
#define VN VID_NVSYNC
#define I VID_INTERLACE
#define DS VID_DBLSCAN

#define M(nm,hr,vr,clk,hs,he,ht,vs,ve,vt,f) \
	{ clk, hr, hs, he, ht, vr, vs, ve, vt, f, nm } 

static struct hdmi_video_softc *hdmi_video_sc;
struct videomode mode1024x768 = M("1024x768x60",1024,768,65000,1048,1184,1344,771,777,806,HN|VN);
struct videomode mode640x480 = M("640x480x60",640,480,25175,656,752,800,490,492,525,HN|VN);

static void
hdmi_video_phy_wait_i2c_done(struct hdmi_video_softc *sc, int msec)
{
	unsigned char val = 0;
	val = hdmi_core_read_1(HDMI_IH_I2CMPHY_STAT0) & 0x3;
	while (val == 0) {
		DELAY(1000);
		if (msec-- == 0)
			return;
		val = hdmi_core_read_1(HDMI_IH_I2CMPHY_STAT0) & 0x3;
	}
}

static void hdmi_video_phy_i2c_write(struct hdmi_video_softc *sc, unsigned short data,
			      unsigned char addr)
{
	hdmi_core_write_1(HDMI_IH_I2CMPHY_STAT0, 0xFF);
	hdmi_core_write_1(HDMI_PHY_I2CM_ADDRESS_ADDR, addr);
	hdmi_core_write_1(HDMI_PHY_I2CM_DATAO_1_ADDR, (unsigned char)(data >> 8));
	hdmi_core_write_1(HDMI_PHY_I2CM_DATAO_0_ADDR, (unsigned char)(data >> 0));
	hdmi_core_write_1(HDMI_PHY_I2CM_OPERATION_ADDR, HDMI_PHY_I2CM_OPERATION_ADDR_WRITE);
	hdmi_video_phy_wait_i2c_done(sc, 1000);
}


static void
hdmi_video_init_ih_mutes(struct hdmi_video_softc *sc)
{
	uint8_t r;

	r = hdmi_core_read_1(HDMI_IH_MUTE);
	r |= (HDMI_IH_MUTE_MUTE_WAKEUP_INTERRUPT | HDMI_IH_MUTE_MUTE_ALL_INTERRUPT);
	hdmi_core_write_1(HDMI_IH_MUTE, r);

	/* by default mask all interrupts */
	hdmi_core_write_1(HDMI_VP_MASK, 0xff);
	hdmi_core_write_1(HDMI_FC_MASK0, 0xff);
	hdmi_core_write_1(HDMI_FC_MASK1, 0xff);
	hdmi_core_write_1(HDMI_FC_MASK2, 0xff);
	hdmi_core_write_1(HDMI_PHY_MASK0, 0xff);
	hdmi_core_write_1(HDMI_PHY_I2CM_INT_ADDR, 0xff);
	hdmi_core_write_1(HDMI_PHY_I2CM_CTLINT_ADDR, 0xff);
	hdmi_core_write_1(HDMI_AUD_INT, 0xff);
	hdmi_core_write_1(HDMI_AUD_SPDIFINT, 0xff);
	hdmi_core_write_1(HDMI_AUD_HBR_MASK, 0xff);
	hdmi_core_write_1(HDMI_GP_MASK, 0xff);
	hdmi_core_write_1(HDMI_A_APIINTMSK, 0xff);
	hdmi_core_write_1(HDMI_CEC_MASK, 0xff);
	hdmi_core_write_1(HDMI_I2CM_INT, 0xff);
	hdmi_core_write_1(HDMI_I2CM_CTLINT, 0xff);

	/* Disable interrupts in the IH_MUTE_* registers */
	hdmi_core_write_1(HDMI_IH_MUTE_FC_STAT0, 0xff);
	hdmi_core_write_1(HDMI_IH_MUTE_FC_STAT1, 0xff);
	hdmi_core_write_1(HDMI_IH_MUTE_FC_STAT2, 0xff);
	hdmi_core_write_1(HDMI_IH_MUTE_AS_STAT0, 0xff);
	hdmi_core_write_1(HDMI_IH_MUTE_PHY_STAT0, 0xff);
	hdmi_core_write_1(HDMI_IH_MUTE_I2CM_STAT0, 0xff);
	hdmi_core_write_1(HDMI_IH_MUTE_CEC_STAT0, 0xff);
	hdmi_core_write_1(HDMI_IH_MUTE_VP_STAT0, 0xff);
	hdmi_core_write_1(HDMI_IH_MUTE_I2CMPHY_STAT0, 0xff);
	hdmi_core_write_1(HDMI_IH_MUTE_AHBDMAAUD_STAT0, 0xff);

	r &= ~(HDMI_IH_MUTE_MUTE_WAKEUP_INTERRUPT | HDMI_IH_MUTE_MUTE_ALL_INTERRUPT);
	hdmi_core_write_1(HDMI_IH_MUTE, r);
}

static void
hdmi_video_disable_overflow_interrupts(struct hdmi_video_softc *sc)
{
	hdmi_core_write_1(HDMI_IH_MUTE_FC_STAT2, HDMI_IH_MUTE_FC_STAT2_OVERFLOW_MASK);
	hdmi_core_write_1(HDMI_FC_MASK2, 0xff);
}

static void
hdmi_video_av_composer(struct hdmi_video_softc *sc)
{
	uint8_t inv_val;
	int hblank, vblank, hsync_len, hbp, vbp;

	/* Set up HDMI_FC_INVIDCONF */
	inv_val = ((sc->mode->flags & VID_NVSYNC) ?
		HDMI_FC_INVIDCONF_VSYNC_IN_POLARITY_ACTIVE_LOW :
		HDMI_FC_INVIDCONF_VSYNC_IN_POLARITY_ACTIVE_HIGH);

	inv_val |= ((sc->mode->flags & VID_NHSYNC) ?
		HDMI_FC_INVIDCONF_HSYNC_IN_POLARITY_ACTIVE_LOW :
		HDMI_FC_INVIDCONF_HSYNC_IN_POLARITY_ACTIVE_HIGH);

	inv_val |= HDMI_FC_INVIDCONF_DE_IN_POLARITY_ACTIVE_HIGH;

	inv_val |= ((sc->mode->flags & VID_INTERLACE) ?
			HDMI_FC_INVIDCONF_R_V_BLANK_IN_OSC_ACTIVE_HIGH :
			HDMI_FC_INVIDCONF_R_V_BLANK_IN_OSC_ACTIVE_LOW);

	inv_val |= ((sc->mode->flags & VID_INTERLACE) ?
		HDMI_FC_INVIDCONF_IN_I_P_INTERLACED :
		HDMI_FC_INVIDCONF_IN_I_P_PROGRESSIVE);

	inv_val |= (1 /*DVI*/ ?
		HDMI_FC_INVIDCONF_DVI_MODEZ_DVI_MODE :
		HDMI_FC_INVIDCONF_DVI_MODEZ_HDMI_MODE);

	hdmi_core_write_1(HDMI_FC_INVIDCONF, inv_val);

	/* Set up horizontal active pixel region width */
	hdmi_core_write_1(HDMI_FC_INHACTV1, sc->mode->hdisplay >> 8);
	hdmi_core_write_1(HDMI_FC_INHACTV0, sc->mode->hdisplay);

	/* Set up vertical blanking pixel region width */
	hdmi_core_write_1(HDMI_FC_INVACTV1, sc->mode->vdisplay >> 8);
	hdmi_core_write_1(HDMI_FC_INVACTV0, sc->mode->vdisplay);

	/* Set up horizontal blanking pixel region width */
	hblank = sc->mode->htotal - sc->mode->hdisplay;
	hdmi_core_write_1(HDMI_FC_INHBLANK1, hblank >> 8);
	hdmi_core_write_1(HDMI_FC_INHBLANK0, hblank);

	/* Set up vertical blanking pixel region width */
	vblank = sc->mode->vtotal - sc->mode->vdisplay;
	hdmi_core_write_1(HDMI_FC_INVBLANK, vblank);

	/* Set up HSYNC active edge delay width (in pixel clks) */
	hbp = sc->mode->htotal - sc->mode->hsync_end;
	hdmi_core_write_1(HDMI_FC_HSYNCINDELAY1, hbp >> 8);
	hdmi_core_write_1(HDMI_FC_HSYNCINDELAY0, hbp);

	/* Set up VSYNC active edge delay (in pixel clks) */
	vbp = sc->mode->vtotal - sc->mode->vsync_end;
	hdmi_core_write_1(HDMI_FC_VSYNCINDELAY, vbp);

	hsync_len = (sc->mode->hsync_end - sc->mode->hsync_start);
	/* Set up HSYNC active pulse width (in pixel clks) */
	hdmi_core_write_1(HDMI_FC_HSYNCINWIDTH1, hsync_len >> 8);
	hdmi_core_write_1(HDMI_FC_HSYNCINWIDTH0, hsync_len);

	/* Set up VSYNC active edge delay (in pixel clks) */
	hdmi_core_write_1(HDMI_FC_VSYNCINWIDTH, (sc->mode->vsync_end - sc->mode->vsync_start));
}

static void
hdmi_video_phy_enable_power(struct hdmi_video_softc *sc, uint8_t enable)
{
	uint8_t reg;

	reg = hdmi_core_read_1(HDMI_PHY_CONF0);
	reg &= ~HDMI_PHY_CONF0_PDZ_MASK;
	reg |= (enable << HDMI_PHY_CONF0_PDZ_OFFSET);
	hdmi_core_write_1(HDMI_PHY_CONF0, reg);
}

static void
hdmi_video_phy_enable_tmds(struct hdmi_video_softc *sc, uint8_t enable)
{
	uint8_t reg;

	reg = hdmi_core_read_1(HDMI_PHY_CONF0);
	reg &= ~HDMI_PHY_CONF0_ENTMDS_MASK;
	reg |= (enable << HDMI_PHY_CONF0_ENTMDS_OFFSET);
	hdmi_core_write_1(HDMI_PHY_CONF0, reg);
}

static void
hdmi_video_phy_gen2_pddq(struct hdmi_video_softc *sc, uint8_t enable)
{
	uint8_t reg;

	reg = hdmi_core_read_1(HDMI_PHY_CONF0);
	reg &= ~HDMI_PHY_CONF0_GEN2_PDDQ_MASK;
	reg |= (enable << HDMI_PHY_CONF0_GEN2_PDDQ_OFFSET);
	hdmi_core_write_1(HDMI_PHY_CONF0, reg);
}

static void
hdmi_video_phy_gen2_txpwron(struct hdmi_video_softc *sc, uint8_t enable)
{
	uint8_t reg;

	reg = hdmi_core_read_1(HDMI_PHY_CONF0);
	reg &= ~HDMI_PHY_CONF0_GEN2_TXPWRON_MASK;
	reg |= (enable << HDMI_PHY_CONF0_GEN2_TXPWRON_OFFSET);
	hdmi_core_write_1(HDMI_PHY_CONF0, reg);
}

static void
hdmi_video_phy_sel_data_en_pol(struct hdmi_video_softc *sc, uint8_t enable)
{
	uint8_t reg;

	reg = hdmi_core_read_1(HDMI_PHY_CONF0);
	reg &= ~HDMI_PHY_CONF0_SELDATAENPOL_MASK;
	reg |= (enable << HDMI_PHY_CONF0_SELDATAENPOL_OFFSET);
	hdmi_core_write_1(HDMI_PHY_CONF0, reg);
}

static void
hdmi_video_phy_sel_interface_control(struct hdmi_video_softc *sc, uint8_t enable)
{
	uint8_t reg;

	reg = hdmi_core_read_1(HDMI_PHY_CONF0);
	reg &= ~HDMI_PHY_CONF0_SELDIPIF_MASK;
	reg |= (enable << HDMI_PHY_CONF0_SELDIPIF_OFFSET);
	hdmi_core_write_1(HDMI_PHY_CONF0, reg);
}

static inline void 
hdmi_video_phy_test_clear(struct hdmi_video_softc *sc, unsigned char bit)
{
	uint8_t val;

	val = hdmi_core_read_1(HDMI_PHY_TST0);
	val &= ~HDMI_PHY_TST0_TSTCLR_MASK;
	val |= (bit << HDMI_PHY_TST0_TSTCLR_OFFSET) &
		HDMI_PHY_TST0_TSTCLR_MASK;
	hdmi_core_write_1(HDMI_PHY_TST0, val);
}


static void hdmi_video_clear_overflow(struct hdmi_video_softc *sc)
{
	int count;
	uint8_t val;

	/* TMDS software reset */
	hdmi_core_write_1(HDMI_MC_SWRSTZ, (uint8_t)~HDMI_MC_SWRSTZ_TMDSSWRST_REQ);

	val = hdmi_core_read_1(HDMI_FC_INVIDCONF);

	for (count = 0 ; count < 5 ; count++)
		hdmi_core_write_1(HDMI_FC_INVIDCONF, val);
}


static int hdmi_video_phy_configure(struct hdmi_video_softc *sc)
{
	uint8_t val;
	uint8_t msec;

	hdmi_core_write_1(HDMI_MC_FLOWCTRL, HDMI_MC_FLOWCTRL_FEED_THROUGH_OFF_CSC_BYPASS);

	/* gen2 tx power off */
	hdmi_video_phy_gen2_txpwron(sc, 0);

	/* gen2 pddq */
	hdmi_video_phy_gen2_pddq(sc, 1);

	/* PHY reset */
	hdmi_core_write_1(HDMI_MC_PHYRSTZ, HDMI_MC_PHYRSTZ_DEASSERT);
	hdmi_core_write_1(HDMI_MC_PHYRSTZ, HDMI_MC_PHYRSTZ_ASSERT);

	hdmi_core_write_1(HDMI_MC_HEACPHY_RST, HDMI_MC_HEACPHY_RST_ASSERT);

	hdmi_video_phy_test_clear(sc, 1);
	hdmi_core_write_1(HDMI_PHY_I2CM_SLAVE_ADDR, HDMI_PHY_I2CM_SLAVE_ADDR_PHY_GEN2);
	hdmi_video_phy_test_clear(sc, 0);

	if (sc->mode->dot_clock*1000 <= 45250000) {
		/* PLL/MPLL Cfg */
		hdmi_video_phy_i2c_write(sc, 0x01e0, 0x06);
		hdmi_video_phy_i2c_write(sc, 0x0000, 0x15);  /* GMPCTRL */
	} else if (sc->mode->dot_clock*1000 <= 92500000) {
		hdmi_video_phy_i2c_write(sc, 0x0140, 0x06);
		hdmi_video_phy_i2c_write(sc, 0x0005, 0x15);
	} else if (sc->mode->dot_clock*1000 <= 148500000) {
		hdmi_video_phy_i2c_write(sc, 0x00a0, 0x06);
		hdmi_video_phy_i2c_write(sc, 0x000a, 0x15);
	} else {
		hdmi_video_phy_i2c_write(sc, 0x00a0, 0x06);
		hdmi_video_phy_i2c_write(sc, 0x000a, 0x15);
	}

	if (sc->mode->dot_clock*1000 <= 54000000) {
		hdmi_video_phy_i2c_write(sc, 0x091c, 0x10);  /* CURRCTRL */
	} else if (sc->mode->dot_clock*1000 <= 58400000) {
		hdmi_video_phy_i2c_write(sc, 0x091c, 0x10);
	} else if (sc->mode->dot_clock*1000 <= 72000000) {
		hdmi_video_phy_i2c_write(sc, 0x06dc, 0x10);
	} else if (sc->mode->dot_clock*1000 <= 74250000) {
		hdmi_video_phy_i2c_write(sc, 0x06dc, 0x10);
	} else if (sc->mode->dot_clock*1000 <= 118800000) {
		hdmi_video_phy_i2c_write(sc, 0x091c, 0x10);
	} else if (sc->mode->dot_clock*1000 <= 216000000) {
		hdmi_video_phy_i2c_write(sc, 0x06dc, 0x10);
	} else {
		panic("Unsupported mode\n");
	}

	hdmi_video_phy_i2c_write(sc, 0x0000, 0x13);  /* PLLPHBYCTRL */
	hdmi_video_phy_i2c_write(sc, 0x0006, 0x17);
	/* RESISTANCE TERM 133Ohm Cfg */
	hdmi_video_phy_i2c_write(sc, 0x0005, 0x19);  /* TXTERM */
	/* PREEMP Cgf 0.00 */
	hdmi_video_phy_i2c_write(sc, 0x800d, 0x09);  /* CKSYMTXCTRL */
	/* TX/CK LVL 10 */
	hdmi_video_phy_i2c_write(sc, 0x01ad, 0x0E);  /* VLEVCTRL */

	/* Board specific setting for PHY register 0x09, 0x0e to pass HCT */
	if (sc->phy_reg_cksymtx != 0)
		hdmi_video_phy_i2c_write(sc, sc->phy_reg_cksymtx, 0x09);

	if (sc->phy_reg_vlev != 0)
		hdmi_video_phy_i2c_write(sc, sc->phy_reg_vlev, 0x0E);

	/* REMOVE CLK TERM */
	hdmi_video_phy_i2c_write(sc, 0x8000, 0x05);  /* CKCALCTRL */

	if (sc->mode->dot_clock*1000 > 148500000) {
		hdmi_video_phy_i2c_write(sc, 0x800b, 0x09);
		hdmi_video_phy_i2c_write(sc, 0x0129, 0x0E);
	}

	hdmi_video_phy_enable_power(sc, 1);

	/* toggle TMDS enable */
	hdmi_video_phy_enable_tmds(sc, 0);
	hdmi_video_phy_enable_tmds(sc, 1);

	/* gen2 tx power on */
	hdmi_video_phy_gen2_txpwron(sc, 1);
	hdmi_video_phy_gen2_pddq(sc, 0);

	/*Wait for PHY PLL lock */
	msec = 4;
	val = hdmi_core_read_1(HDMI_PHY_STAT0) & HDMI_PHY_TX_PHY_LOCK;
	while (val == 0) {
		DELAY(1000);
		if (msec-- == 0) {
			device_printf(sc->dev, "PHY PLL not locked\n");
			return (-1);
		}
		val = hdmi_core_read_1(HDMI_PHY_STAT0) & HDMI_PHY_TX_PHY_LOCK;
	}

	return true;
}

static void hdmi_video_phy_init(struct hdmi_video_softc *sc)
{
	int i;

	/* HDMI Phy spec says to do the phy initialization sequence twice */
	for (i = 0 ; i < 2 ; i++) {
		hdmi_video_phy_sel_data_en_pol(sc, 1);
		hdmi_video_phy_sel_interface_control(sc, 0);
		hdmi_video_phy_enable_tmds(sc, 0);
		hdmi_video_phy_enable_power(sc, 0);

		/* Enable CSC */
		hdmi_video_phy_configure(sc);
	}
}

static void hdmi_video_enable_video_path(struct hdmi_video_softc *sc)
{
	uint8_t clkdis;

	/* control period minimum duration */
	hdmi_core_write_1(HDMI_FC_CTRLDUR, 12);
	hdmi_core_write_1(HDMI_FC_EXCTRLDUR, 32);
	hdmi_core_write_1(HDMI_FC_EXCTRLSPAC, 1);

	/* Set to fill TMDS data channels */
	hdmi_core_write_1(HDMI_FC_CH0PREAM, 0x0B);
	hdmi_core_write_1(HDMI_FC_CH1PREAM, 0x16);
	hdmi_core_write_1(HDMI_FC_CH2PREAM, 0x21);

	/* Save CEC clock */
	clkdis = hdmi_core_read_1(HDMI_MC_CLKDIS) & HDMI_MC_CLKDIS_CECCLK_DISABLE;
	clkdis |= ~HDMI_MC_CLKDIS_CECCLK_DISABLE;

	/* Enable pixel clock and tmds data path */
	clkdis = 0x7F & clkdis;
	clkdis &= ~HDMI_MC_CLKDIS_PIXELCLK_DISABLE;
	hdmi_core_write_1(HDMI_MC_CLKDIS, clkdis);

	clkdis &= ~HDMI_MC_CLKDIS_TMDSCLK_DISABLE;
	hdmi_core_write_1(HDMI_MC_CLKDIS, clkdis);
}

static void hdmi_video_video_packetize(struct hdmi_video_softc *sc)
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
	hdmi_core_write_1(HDMI_VP_PR_CD, val);

	val = hdmi_core_read_1(HDMI_VP_STUFF);
	val &= ~HDMI_VP_STUFF_PR_STUFFING_MASK;
	val |= HDMI_VP_STUFF_PR_STUFFING_STUFFING_MODE;
	hdmi_core_write_1(HDMI_VP_STUFF, val);

	val = hdmi_core_read_1(HDMI_VP_CONF);
	val &= ~(HDMI_VP_CONF_PR_EN_MASK |
		HDMI_VP_CONF_BYPASS_SELECT_MASK);
	val |= HDMI_VP_CONF_PR_EN_DISABLE |
		HDMI_VP_CONF_BYPASS_SELECT_VID_PACKETIZER;
	hdmi_core_write_1(HDMI_VP_CONF, val);

	val = hdmi_core_read_1(HDMI_VP_STUFF);
	val &= ~HDMI_VP_STUFF_IDEFAULT_PHASE_MASK;
	val |= 1 << HDMI_VP_STUFF_IDEFAULT_PHASE_OFFSET;
	hdmi_core_write_1(HDMI_VP_STUFF, val);

	hdmi_core_write_1(HDMI_VP_REMAP, remap_size);

	if (output_select == HDMI_VP_CONF_OUTPUT_SELECTOR_PP) {
		val = hdmi_core_read_1(HDMI_VP_CONF);
		val &= ~(HDMI_VP_CONF_BYPASS_EN_MASK |
			HDMI_VP_CONF_PP_EN_ENMASK |
			HDMI_VP_CONF_YCC422_EN_MASK);
		val |= HDMI_VP_CONF_BYPASS_EN_DISABLE |
			HDMI_VP_CONF_PP_EN_ENABLE |
			HDMI_VP_CONF_YCC422_EN_DISABLE;
		hdmi_core_write_1(HDMI_VP_CONF, val);
	} else if (output_select == HDMI_VP_CONF_OUTPUT_SELECTOR_YCC422) {
		val = hdmi_core_read_1(HDMI_VP_CONF);
		val &= ~(HDMI_VP_CONF_BYPASS_EN_MASK |
			HDMI_VP_CONF_PP_EN_ENMASK |
			HDMI_VP_CONF_YCC422_EN_MASK);
		val |= HDMI_VP_CONF_BYPASS_EN_DISABLE |
			HDMI_VP_CONF_PP_EN_DISABLE |
			HDMI_VP_CONF_YCC422_EN_ENABLE;
		hdmi_core_write_1(HDMI_VP_CONF, val);
	} else if (output_select == HDMI_VP_CONF_OUTPUT_SELECTOR_BYPASS) {
		val = hdmi_core_read_1(HDMI_VP_CONF);
		val &= ~(HDMI_VP_CONF_BYPASS_EN_MASK |
			HDMI_VP_CONF_PP_EN_ENMASK |
			HDMI_VP_CONF_YCC422_EN_MASK);
		val |= HDMI_VP_CONF_BYPASS_EN_ENABLE |
			HDMI_VP_CONF_PP_EN_DISABLE |
			HDMI_VP_CONF_YCC422_EN_DISABLE;
		hdmi_core_write_1(HDMI_VP_CONF, val);
	} else {
		return;
	}

	val = hdmi_core_read_1(HDMI_VP_STUFF);
	val &= ~(HDMI_VP_STUFF_PP_STUFFING_MASK |
		HDMI_VP_STUFF_YCC422_STUFFING_MASK);
	val |= HDMI_VP_STUFF_PP_STUFFING_STUFFING_MODE |
		HDMI_VP_STUFF_YCC422_STUFFING_STUFFING_MODE;
	hdmi_core_write_1(HDMI_VP_STUFF, val);

	val = hdmi_core_read_1(HDMI_VP_CONF);
	val &= ~HDMI_VP_CONF_OUTPUT_SELECTOR_MASK;
	val |= output_select;
	hdmi_core_write_1(HDMI_VP_CONF, val);
}

#if 0
static void hdmi_video_video_csc(struct hdmi *hdmi)
{
	int color_depth = 0;
	int interpolation = HDMI_CSC_CFG_INTMODE_DISABLE;
	int decimation = HDMI_CSC_CFG_DECMODE_DISABLE;
	uint8_t val;

	/* YCC422 interpolation to 444 mode */
	if (isColorSpaceInterpolation(hdmi))
		interpolation = HDMI_CSC_CFG_INTMODE_CHROMA_INT_FORMULA1;
	else if (isColorSpaceDecimation(hdmi))
		decimation = HDMI_CSC_CFG_DECMODE_CHROMA_INT_FORMULA3;

	if (hdmi->hdmi_video_data.enc_color_depth == 8)
		color_depth = HDMI_CSC_SCALE_CSC_COLORDE_PTH_24BPP;
	else if (hdmi->hdmi_video_data.enc_color_depth == 10)
		color_depth = HDMI_CSC_SCALE_CSC_COLORDE_PTH_30BPP;
	else if (hdmi->hdmi_video_data.enc_color_depth == 12)
		color_depth = HDMI_CSC_SCALE_CSC_COLORDE_PTH_36BPP;
	else if (hdmi->hdmi_video_data.enc_color_depth == 16)
		color_depth = HDMI_CSC_SCALE_CSC_COLORDE_PTH_48BPP;
	else
		return;

	/*configure the CSC registers */
	hdmi_core_write_1(HDMI_CSC_CFG, interpolation | decimation);
	val = hdmi_core_read_1(HDMI_CSC_SCALE);
	val &= ~HDMI_CSC_SCALE_CSC_COLORDE_PTH_MASK;
	val |= color_depth;
	hdmi_core_write_1(HDMI_CSC_SCALE, val);

	update_csc_coeffs(hdmi);
}
#endif 

static void hdmi_video_video_sample(struct hdmi_video_softc *sc)
{
	int color_format;
	uint8_t val;

	color_format = 0x01;
	val = HDMI_TX_INVID0_INTERNAL_DE_GENERATOR_DISABLE |
		((color_format << HDMI_TX_INVID0_VIDEO_MAPPING_OFFSET) &
		HDMI_TX_INVID0_VIDEO_MAPPING_MASK);
	hdmi_core_write_1(HDMI_TX_INVID0, val);

	/* Enable TX stuffing: When DE is inactive, fix the output data to 0 */
	val = HDMI_TX_INSTUFFING_BDBDATA_STUFFING_ENABLE |
		HDMI_TX_INSTUFFING_RCRDATA_STUFFING_ENABLE |
		HDMI_TX_INSTUFFING_GYDATA_STUFFING_ENABLE;
	hdmi_core_write_1(HDMI_TX_INSTUFFING, val);
	hdmi_core_write_1(HDMI_TX_GYDATA0, 0x0);
	hdmi_core_write_1(HDMI_TX_GYDATA1, 0x0);
	hdmi_core_write_1(HDMI_TX_RCRDATA0, 0x0);
	hdmi_core_write_1(HDMI_TX_RCRDATA1, 0x0);
	hdmi_core_write_1(HDMI_TX_BCBDATA0, 0x0);
	hdmi_core_write_1(HDMI_TX_BCBDATA1, 0x0);
}

static int
hdmi_video_setup(struct hdmi_video_softc *sc)
{
	hdmi_video_disable_overflow_interrupts(sc);
	hdmi_video_av_composer(sc);
	hdmi_video_phy_init(sc);
	hdmi_video_enable_video_path(sc);
	// TODO: AVI infoframes 
	hdmi_video_video_packetize(sc);
	// hdmi_video_video_csc(sc);
	hdmi_video_video_sample(sc);
	hdmi_video_clear_overflow(sc);

	return (0);
}

static void
hdmi_video_detect_mode(void *arg)
{
	struct hdmi_video_softc *sc = arg;
	uint32_t gpr3;
	int ipu_id, disp_id;

	ipu_id = 0;
	disp_id = 0;

	gpr3 = imx_iomux_gpr_get(12);
	printf("GPR3 %08x -> ", gpr3);
	gpr3 &= ~0x0d;
	gpr3 |= ((ipu_id << 1) | disp_id) << 2;
	printf("%08x\n", gpr3);
	imx_iomux_gpr_set(12, gpr3);

	hdmi_edid_read();
	sc->mode = &mode1024x768;
	sc->phy_reg_vlev = 0x294;
	sc->phy_reg_cksymtx = 0x800d;

	printf("HDMI setup\n");
	hdmi_video_setup(sc);
	printf("HDMI setup done\n");

}

int hdmi_video_enable()
{
	return (0);
}

static int
hdmi_video_detach(device_t dev)
{
	struct hdmi_video_softc *sc;

	sc = device_get_softc(dev);

	if (sc->irq_res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, sc->irq_rid, sc->irq_res);

	return (0);
}

static int
hdmi_video_attach(device_t dev)
{
	struct hdmi_video_softc *sc;
	int err;

	sc = device_get_softc(dev);
	err = 0;

	/* Allocate bus_space resources. */
	sc->irq_rid = 0;
	sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &sc->irq_rid,
	    RF_ACTIVE);
	if (sc->irq_res == NULL) {
		device_printf(dev, "No IRQ\n");
		// err = ENXIO;
		// goto out;
	}

	sc->dev = dev;
	hdmi_video_sc = sc;

	err = 0;

	sc->mode_hook.ich_func = hdmi_video_detect_mode;
	sc->mode_hook.ich_arg = sc;

	if (config_intrhook_establish(&sc->mode_hook) != 0)
		return (ENOMEM);

	if (err != 0)
		hdmi_video_detach(dev);

	return (err);
}

static int
hdmi_video_probe(device_t dev)
{

        if (ofw_bus_is_compatible(dev, "fsl,imx6dl-hdmi-video") == 0)
		return (ENXIO);

	device_set_desc(dev, "Freescale i.MX6 HDMI video");

	return (BUS_PROBE_DEFAULT);
}


static device_method_t hdmi_video_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,  hdmi_video_probe),
	DEVMETHOD(device_attach, hdmi_video_attach),
	DEVMETHOD(device_detach, hdmi_video_detach),

	DEVMETHOD_END
};

static driver_t hdmi_video_driver = {
	"hdmi_video",
	hdmi_video_methods,
	sizeof(struct hdmi_video_softc)
};

static devclass_t hdmi_video_devclass;

DRIVER_MODULE(hdmi_video, simplebus, hdmi_video_driver, hdmi_video_devclass, 0, 0);
