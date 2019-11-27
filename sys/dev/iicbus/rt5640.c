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
 * This code base on isl12xx.c
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

/*
 * Driver for Realtek rt5640 audio codec
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

#include "iicbus_if.h"


#define	RT5640_RESET		0x00
#define	RT5640_HP_VOL		0x02
#define	RT5640_AD_DA_MIXER	0x29
#define	RT5640_STO_DAC_MIXER	0x2a
#define	RT5640_DIG_MIXER	0x2c
#define	RT5640_DSP_PATH2	0x2e
#define	RT5640_REC_L2_MIXER	0x3c
#define	RT5640_REC_R2_MIXER	0x3e
#define	RT5640_HPO_MIXER	0x45
#define	RT5640_OUT_L3_MIXER	0x4f
#define	RT5640_OUT_R3_MIXER	0x52
#define	RT5640_LOUT_MIXER	0x53
#define	RT5640_PWR_DIG1		0x61
#define	RT5640_PWR_DIG2		0x62
#define	RT5640_PWR_ANLG1	0x63
#define	RT5640_PWR_ANLG2	0x64
#define	RT5640_PWR_MIXER	0x65
#define	RT5640_PWR_VOL		0x66
#define	RT5640_PR_INDEX		0x6a
#define	RT5640_PR_DATA		0x6c
#define	RT5640_I2S1_SDP		0x70
#define	RT5640_ADDA_CLK1	0x73
#define	RT5640_GLB_CLK		0x80
#define	RT5640_DEPOP_M1		0x8e
#define	RT5640_DEPOP_M2		0x8f
#define	RT5640_CHARGE_PUMP	0x91
#define	RT5640_GCTRL1		0xfa
#define	RT5640_VENDOR_ID2	0xff

#define	MAX_BUFFER	16

struct rt5640_softc {
	device_t	dev;
	device_t	busdev;
	struct intr_config_hook 
			init_hook;
	clk_t		clk;
};

#ifdef FDT
static struct ofw_compat_data compat_data[] = {
	{"realtek,rt5640", 1},
	{NULL,           0},
};
#endif

static void	rt5640_init(void *arg);
static int	rt5640_probe(device_t dev);
static int	rt5640_attach(device_t dev);
static int	rt5640_detach(device_t dev);

static int	rt5640_writeto(device_t slavedev, uint8_t regaddr, 
		    void *buffer, uint16_t buflen, int waithow);

static int 
rt5640_writeto(device_t slavedev, uint8_t regaddr, void *buffer,
    uint16_t buflen, int waithow)
{
	struct iic_msg	msgs;
	uint8_t		slaveaddr;
	uint8_t		newbuf[MAX_BUFFER];

	if (buflen > MAX_BUFFER - 1)
		return (EINVAL);

	slaveaddr = iicbus_get_addr(slavedev);

	newbuf[0] = regaddr;
	memcpy(newbuf + 1, buffer, buflen);
	msgs.slave = slaveaddr;
	msgs.flags = IIC_M_WR;
	msgs.len   = 1 + buflen;
	msgs.buf   = newbuf;

	return (iicbus_transfer_excl(slavedev, &msgs, 1, waithow));
}

static inline int
rt5640_read2(struct rt5640_softc *sc, uint8_t reg, uint16_t *data) 
{
	int res;
	res = iicdev_readfrom(sc->dev, reg, data, 2, IIC_WAIT);
	if (res == 0)
		*data = be16toh(*data);
	return (res);
}

static inline int
rt5640_write2(struct rt5640_softc *sc, uint8_t reg, uint16_t val) 
{
	val = htobe16(val);

	return (rt5640_writeto(sc->dev, reg, &val, 2, IIC_WAIT));
}


static inline int
rt5640_pr_read2(struct rt5640_softc *sc, uint8_t reg, uint16_t *data) 
{
	int res;
	res = rt5640_write2(sc, RT5640_PR_INDEX, reg);
	if (res != 0)
		return (res);
	res = rt5640_read2(sc, RT5640_PR_DATA, data);
	return res;
}

static inline int
rt5640_pr_write2(struct rt5640_softc *sc, uint8_t reg, uint16_t val) 
{
	int res;
	res = rt5640_write2(sc, RT5640_PR_INDEX, reg);
	if (res != 0)
		return (res);
	res = rt5640_write2(sc, RT5640_PR_DATA, val);
	return res;
}

static void
rt5640_powerup(struct rt5640_softc *sc)
{
	uint16_t reg;

	// "Improve HP Amp Drv"
	// power on

        rt5640_pr_read2(sc, 0x24, &reg);
	reg &= ~0x0700;
	reg |= 0x0200;
        rt5640_pr_write2(sc, 0x24, reg);

	rt5640_read2(sc, RT5640_DEPOP_M2, &reg);
	reg |= (1 << 13);
	rt5640_write2(sc, RT5640_DEPOP_M2, reg);

	rt5640_read2(sc, RT5640_DEPOP_M1, &reg);
	reg &= ~0xf;
	reg |= 9;
	rt5640_write2(sc, RT5640_DEPOP_M1, reg);

        rt5640_pr_write2(sc, 0x77, 0x9f00);

	// VREF1/VREF2 are slow
	rt5640_read2(sc, RT5640_PWR_ANLG1, &reg);
	reg &= ~((1 << 14) | (1 << 3));
	rt5640_write2(sc, RT5640_PWR_ANLG1, reg);

	// Improve HP Amp driving
	reg |= (1 << 5);
	rt5640_write2(sc, RT5640_PWR_ANLG1, reg);

	pause("pwrup", hz);

	reg |= ((1 << 14) | (1 << 3));
	rt5640_write2(sc, RT5640_PWR_ANLG1, reg);

	// depop
	rt5640_read2(sc, RT5640_DEPOP_M2, &reg);
	reg &= ~(1 << 13);
	reg |= (1 << 6);
	rt5640_write2(sc, RT5640_DEPOP_M2, reg);

	rt5640_read2(sc, RT5640_CHARGE_PUMP, &reg);
	reg &= ~(3 << 8);
	reg |= (2 << 8);
	rt5640_write2(sc, RT5640_CHARGE_PUMP, reg);

        rt5640_pr_write2(sc, 0x37, 0x1c00);

	rt5640_read2(sc, RT5640_DEPOP_M1, &reg);
	reg &= ~(3 << 2);
	reg |= (1 << 2);
	rt5640_write2(sc, RT5640_DEPOP_M1, reg);

        rt5640_pr_read2(sc, 0x24, &reg);
	reg &= ~0x0700;
	reg |= 0x0400;
        rt5640_pr_write2(sc, 0x24, reg);
}

static void
rt5640_init(void *arg)
{
	struct rt5640_softc *sc;
	uint16_t reg;
	
	sc = (struct rt5640_softc*)arg;
	config_intrhook_disestablish(&sc->init_hook);

	rt5640_read2(sc, RT5640_VENDOR_ID2, &reg);
	device_printf(sc->dev, "RT5640_VENDOR_ID2 == %04x\n", reg);

	rt5640_write2(sc, RT5640_RESET, 0);

        rt5640_pr_write2(sc, 0x3d, 0x3600);
        rt5640_pr_write2(sc, 0x12, 0x0aa8);
        rt5640_pr_write2(sc, 0x14, 0x0aaa);
        rt5640_pr_write2(sc, 0x20, 0x6110);
        rt5640_pr_write2(sc, 0x21, 0xe0e0);
        rt5640_pr_write2(sc, 0x23, 0x1804);

	rt5640_read2(sc, RT5640_DSP_PATH2, &reg);
	reg &= ~0xfc00;
	reg |= 0x0c00;
	rt5640_write2(sc, RT5640_DSP_PATH2, reg);

	rt5640_read2(sc, RT5640_GCTRL1, &reg);
	reg |= 0x0301;
	rt5640_write2(sc, RT5640_GCTRL1, reg);

	// HP L/R amp
	reg |= ((1 << 7) | (1 << 6));
	rt5640_write2(sc, RT5640_PWR_ANLG1, reg);

	// "HP L/R Playback"
	// unmute HPO
	rt5640_read2(sc, RT5640_HP_VOL, &reg);
	reg &= ~((1 << 15) | (1 << 7));
	// unmute HPOVOL
	reg &= ~((1 << 14) | (1 << 6));
	// max volume
	reg &= ~(0x3f << 8);
	reg &= ~(0x3f << 0);
	rt5640_write2(sc, RT5640_HP_VOL, reg);

	// "HPO MIX DAC1 Switch"
	// "HPO MIX HPVOL Switch"
	rt5640_read2(sc, RT5640_HPO_MIXER, &reg);
	// unmute DAC1 and HPOVOL
	// reg &= ~((1 << 14) | (1 << 13));
	// unmute HPOVOL
	reg &= ~((1 << 13));
	rt5640_write2(sc, RT5640_HPO_MIXER, reg);
	rt5640_write2(sc, RT5640_HPO_MIXER, 0); // LINUX

	// "DAC L1/R1"
	rt5640_read2(sc, RT5640_PWR_DIG1, &reg);
	reg |= ((1 << 12) | (1 << 11));
	rt5640_write2(sc, RT5640_PWR_DIG1, reg);

	// TODO: "DAC L1/R1 Switch"
#if 0
	rt5640_read2(sc, RT5640_LOUT_MIXER, &reg);
	reg &= ~((1 << 15) | (1 << 14));
	rt5640_write2(sc, RT5640_LOUT_MIXER, reg);
#endif

	// DAC MIXL/MIXR
	//   Stereo ADC Switch
	//   INF1 Switch
	rt5640_read2(sc, RT5640_AD_DA_MIXER, &reg);
	// reg &= ~((1 << 15) | (1 << 14) | (1 << 7) | (1 << 6));
	reg &= ~((1 << 14) | (1 << 6));
	rt5640_write2(sc, RT5640_AD_DA_MIXER, reg);

	// HPOR Switch
	rt5640_read2(sc, RT5640_REC_R2_MIXER, &reg);
	reg &= ~(1 << 6);
	rt5640_write2(sc, RT5640_REC_R2_MIXER, reg);

	// HPOL Switch
	rt5640_read2(sc, RT5640_REC_L2_MIXER, &reg);
	reg &= ~(1 << 6);
	rt5640_write2(sc, RT5640_REC_L2_MIXER, reg);

	// I2S1 power
	rt5640_read2(sc, RT5640_PWR_DIG1, &reg);
	reg |= (1 << 15);
	rt5640_write2(sc, RT5640_PWR_DIG1, reg);

	// I2S1 setup
	rt5640_read2(sc, RT5640_I2S1_SDP, &reg);
	// channel mapping
	reg &= ~(0x7 << 12);
	// format
	reg &= ~(0xf);
	// slave
	reg |= (1 << 15);
	rt5640_write2(sc, RT5640_I2S1_SDP, reg);

	// DAC L1/R1 Switch
	rt5640_read2(sc, RT5640_DIG_MIXER, &reg);
	reg &= ~((1 << 15) | (1 << 13));
	rt5640_write2(sc, RT5640_DIG_MIXER, reg);

	// HPOVOL L/R
	rt5640_read2(sc, RT5640_PWR_VOL, &reg);
	reg |= ((1 << 11) | (1 << 10));
	rt5640_write2(sc, RT5640_PWR_VOL, reg);

	// OUT MIX power
	rt5640_read2(sc, RT5640_PWR_MIXER, &reg);
	reg |= ((1 << 15) | (1 << 14));
	rt5640_write2(sc, RT5640_PWR_MIXER, reg);

	// OUT MIXL
	rt5640_read2(sc, RT5640_OUT_R3_MIXER, &reg);
	reg &= ~(1 << 0);
	rt5640_write2(sc, RT5640_OUT_R3_MIXER, reg);

	// OUT MIXR
	rt5640_read2(sc, RT5640_OUT_L3_MIXER, &reg);
	reg &= ~(1 << 0);
	rt5640_write2(sc, RT5640_OUT_L3_MIXER, reg);

	// Stereo DAC MIXL/MIXR
	rt5640_read2(sc, RT5640_STO_DAC_MIXER, &reg);
	reg &= ~((1 << 14) | (1 << 6));
	rt5640_write2(sc, RT5640_STO_DAC_MIXER, reg);
	rt5640_write2(sc, RT5640_STO_DAC_MIXER, 0x00000414); // LINUX
	rt5640_write2(sc, 4, 0); // LINUX

	// PLL1
	rt5640_read2(sc, RT5640_PWR_ANLG2, &reg);
	reg |= (1 << 9);
	rt5640_write2(sc, RT5640_PWR_ANLG2, reg);

	rt5640_write2(sc, RT5640_ADDA_CLK1, 0x114);

	rt5640_powerup(sc);

	int i;
	for (i = 0; i < 0x100; i++) {
		rt5640_read2(sc, i, &reg);
		printf("gonzo [%02x] = %08x\n", i, reg);
	}

	return;
}

static int
rt5640_probe(device_t dev)
{

#ifdef FDT
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data != 0) {
		device_set_desc(dev, "Realtek RT5640");
		return (BUS_PROBE_DEFAULT);
	}
#endif
	return (ENXIO);
}

static int
rt5640_attach(device_t dev)
{
	struct rt5640_softc	*sc;
	int error;
	
	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->busdev = device_get_parent(sc->dev);

	error = clk_get_by_ofw_name(dev, 0, "mclk", &sc->clk);
	if (error != 0) {
		device_printf(dev, "cannot get mclk clock\n");
		return (ENXIO);
	}
	clk_set_freq(sc->clk, 12288000, 0);

	/*
	 * Wait until i2c is ready to set up the chip
	 */
	sc->init_hook.ich_func = rt5640_init;
	sc->init_hook.ich_arg = sc;
	if (config_intrhook_establish(&sc->init_hook) != 0)
		return (ENOMEM);

	return (0);
}

static int
rt5640_detach(device_t dev)
{

	return (0);
}

static device_method_t rt5640_methods[] = {
        /* device_if methods */
	DEVMETHOD(device_probe,		rt5640_probe),
	DEVMETHOD(device_attach,	rt5640_attach),
	DEVMETHOD(device_detach,	rt5640_detach),

	DEVMETHOD_END,
};

static driver_t rt5640_driver = {
	"rt5640codec",
	rt5640_methods,
	sizeof(struct rt5640_softc),
};
static devclass_t rt5640_devclass;

DRIVER_MODULE(rt5640codec, iicbus, rt5640_driver, rt5640_devclass, NULL, NULL);
MODULE_VERSION(rt5640codec, 1);
MODULE_DEPEND(rt5640codec, iicbus, IICBUS_MINVER, IICBUS_PREFVER, IICBUS_MAXVER);
IICBUS_FDT_PNP_INFO(compat_data);
