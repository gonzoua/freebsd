/*-
 * Copyright 2015 Alexander Kabaev <kan@FreeBSD.org>
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

/*
 * Ingenic JZ4780 CGU driver.
 *
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/conf.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/resource.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <dev/fdt/fdt_common.h>
#include <dev/fdt/fdt_clock.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <mips/ingenic/jz4780_regs.h>

#include <gnu/dts/include/dt-bindings/clock/jz4780-cgu.h>

struct jz4780_clock_softc;

typedef int (jz4780_clock_control_t)(struct jz4780_clock_softc *sc, int enable, void *info);
typedef int (jz4780_clock_inquiry_t)(struct jz4780_clock_softc *sc, void *info, void *clkinfo);

struct jz4780_clock_desc {
	const char             *clock_name;
	uint32_t                clock_type;
	jz4780_clock_control_t *clock_ctl;
	jz4780_clock_inquiry_t *clock_inq;
	void                   *clock_data;
};

struct jz4780_clock_softc {
	device_t	dev;
	struct resource	*res[1];
	struct mtx	mtx;
};

#define CGU_PLL_M_SHIFT		19
#define CGU_PLL_M_WIDTH		13

#define CGU_PLL_N_SHIFT		13
#define CGU_PLL_N_WIDTH		6

#define CGU_PLL_OD_SHIFT	9
#define CGU_PLL_OD_WIDTH	3

#define CGU_PLL_ON_SHIFT	4
#define CGU_PLL_ON_WIDTH	1

#define CGU_PLL_MODE_SHIFT	3
#define CGU_PLL_MODE_WIDTH	1

#define CGU_PLL_BP_SHIFT	1
#define CGU_PLL_BP_WIDTH	1

#define CGU_PLL_EN_SHIFT	0
#define CGU_PLL_EN_WIDTH	1

#define REG_MSK(field)			(((1u << field ## _WIDTH) - 1) << field ##_SHIFT)
#define REG_VAL(field, val)		((val) << field ##_SHIFT)
#define REG_CLR(reg, field)		((reg) & ~REG_MSK(field))
#define REG_GET(reg, field)		(((reg) & REG_MSK(field)) >> field ##_SHIFT)
#define REG_SET(reg, field, val)	(REG_CLR(reg) | REG_VAL(val))

static struct resource_spec jz4780_clock_spec[] = {
	{ SYS_RES_MEMORY, 0, RF_ACTIVE },
	{ -1, 0 }
};

#define	CGU_LOCK(sc)		mtx_lock(&(sc)->mtx)
#define	CGU_UNLOCK(sc)		mtx_unlock(&(sc)->mtx)
#define	CGU_LOCK_INIT(sc)	\
    mtx_init(&(sc)->mtx, device_get_nameunit((sc)->dev),	\
    "jz4780-cgu", MTX_DEF)
#define	CGU_LOCK_DESTROY(sc)	mtx_destroy(&(sc)->mtx);

#define CSR_WRITE_4(sc, reg, val)	bus_write_4((sc)->res[0], (reg), (val))
#define CSR_READ_4(sc, reg)		bus_read_4((sc)->res[0], (reg))

static int
jz4780_clock_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "ingenic,jz4780-cgu"))
		return (ENXIO);

	device_set_desc(dev, "Ingenic jz4780 CGU");

	return (BUS_PROBE_DEFAULT);
}

/* Remove soon */
static void apbus_attach(device_t dev);

static int
jz4780_clock_attach(device_t dev)
{
	struct jz4780_clock_softc *sc = device_get_softc(dev);

	sc->dev = dev;

	if (bus_alloc_resources(dev, jz4780_clock_spec, sc->res)) {
		device_printf(dev, "could not allocate resources for device\n");
		return (ENXIO);
	}

	/* HACK */
	apbus_attach(dev);

	CGU_LOCK_INIT(sc);

	fdt_clock_register_provider(dev);

	return (0);
}

static int
jz4780_clock_detach(device_t dev)
{
	struct jz4780_clock_softc *sc = device_get_softc(dev);

	fdt_clock_unregister_provider(dev);

	bus_release_resources(dev, jz4780_clock_spec, sc->res);
	CGU_LOCK_DESTROY(sc);

	return (0);
}

static int
jz4780_clock_enable(device_t dev, int index)
{

	return (0);
}

static int
jz4780_clock_disable(device_t dev, int index)
{

	return (0);
}

static int
jz4780_clock_get_info(device_t dev, int index, struct fdt_clock_info *info)
{

	return (ENXIO);
}

static device_method_t jz4780_clock_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		jz4780_clock_probe),
	DEVMETHOD(device_attach,	jz4780_clock_attach),
	DEVMETHOD(device_detach,	jz4780_clock_detach),

	/* fdt_clock interface */
	DEVMETHOD(fdt_clock_enable,	jz4780_clock_enable),
	DEVMETHOD(fdt_clock_disable,	jz4780_clock_disable),
	DEVMETHOD(fdt_clock_get_info,	jz4780_clock_get_info),

	DEVMETHOD_END
};

static driver_t jz4780_clock_driver = {
	"cgu",
	jz4780_clock_methods,
	sizeof(struct jz4780_clock_softc),
};

static devclass_t jz4780_clock_devclass;

EARLY_DRIVER_MODULE(jz4780_clock, simplebus, jz4780_clock_driver,
    jz4780_clock_devclass, 0, 0,  BUS_PASS_CPU + BUS_PASS_ORDER_LATE);

typedef struct apbus_dev {
	const char *name;	/* driver name */
	bus_addr_t addr;	/* base address */
	uint32_t irq;		/* interrupt */
	uint32_t clk0;		/* bit(s) in CLKGR0 */
	uint32_t clk1;		/* bit(s) in CLKGR1 */
	uint32_t clkreg;	/* CGU register */
} apbus_dev_t;

/*
 * Hack: steal code from NetBSD until fine-grained CGU is
 * functional.
 */
static const apbus_dev_t apbus_devs[] = {
	{ "efuse",	JZ_EFUSE,	-1, 0, 0, 0},
	{ "com",	JZ_UART0,	51, CLK_UART0, 0, 0},
	{ "com",	JZ_UART1,	50, CLK_UART1, 0, 0},
	{ "com",	JZ_UART2,	49, CLK_UART2, 0, 0},
	{ "com",	JZ_UART3,	48, CLK_UART3, 0, 0},
	{ "com",	JZ_UART4,	34, 0, CLK_UART4, 0},
	{ "dwctwo",	JZ_DWC2_BASE,   21, CLK_OTG0 | CLK_UHC, CLK_OTG1, 0},
	{ "ohci",	JZ_OHCI_BASE,    5, CLK_UHC, 0, 0},
	{ "ehci",	JZ_EHCI_BASE,   20, CLK_UHC, 0, 0},
	{ "jziic",	JZ_SMB0_BASE,   60, CLK_SMB0, 0, 0},
	{ "jziic",	JZ_SMB1_BASE,   59, CLK_SMB1, 0, 0},
	{ "jziic",	JZ_SMB2_BASE,   58, CLK_SMB2, 0, 0},
	{ "jziic",	JZ_SMB3_BASE,   57, 0, CLK_SMB3, 0},
	{ "jziic",	JZ_SMB4_BASE,   56, 0, CLK_SMB4, 0},
	{ "jzmmc",	JZ_MSC0_BASE,   37, CLK_MSC0, 0, JZ_MSC0CDR},
	{ "jzmmc",	JZ_MSC1_BASE,   36, CLK_MSC1, 0, JZ_MSC1CDR},
	{ "jzmmc",	JZ_MSC2_BASE,   35, CLK_MSC2, 0, JZ_MSC2CDR},
	{ "jzfb",	JZ_LCDC0_BASE,  31, CLK_LCD, CLK_HDMI, 0},
	{ "dme",	0,		-1, CLK_MAC, 0, 0},
	{ "core1",	0,		-1, 0, CLK_P1, 0},
	{ NULL,		-1,             -1, 0, 0, 0}
};

static void
apbus_attach(device_t dev)
{
	struct jz4780_clock_softc *sc;

	uint32_t reg, mpll, m, n, p, mclk, pclk, pdiv, cclk, cdiv;

	sc = device_get_softc(dev);

	/* assuming we're using MPLL */
	mpll = CSR_READ_4(sc, JZ_CPMPCR);
	m = (mpll & JZ_PLLM_M) >> JZ_PLLM_S;
	n = (mpll & JZ_PLLN_M) >> JZ_PLLN_S;
	p = (mpll & JZ_PLLP_M) >> JZ_PLLP_S;
	/* assuming 48MHz EXTCLK */
	mclk = (48000 * (m + 1) / (n + 1)) / (p + 1);

	reg = CSR_READ_4(sc, JZ_CPCCR);
	pdiv = ((reg & JZ_PDIV_M) >> JZ_PDIV_S) + 1;
	pclk = mclk / pdiv;
	cdiv = (reg & JZ_CDIV_M) + 1;
	cclk = mclk / cdiv;

	device_printf(dev, "mclk %d kHz\n", mclk);
	device_printf(dev, "pclk %d kHz\n", pclk);
	device_printf(dev, "CPU clock %d kHz\n", cclk);

	/* enable clocks */
	reg = CSR_READ_4(sc, JZ_CLKGR1);
	reg &= ~CLK_AHB_MON;	/* AHB_MON clock */
	CSR_WRITE_4(sc, JZ_CLKGR1, reg);

	/* wake up the USB part */
	reg = CSR_READ_4(sc, JZ_OPCR);
	reg |= OPCR_SPENDN0 | OPCR_SPENDN1;
	CSR_WRITE_4(sc, JZ_OPCR, reg);

	for (const apbus_dev_t *adv = apbus_devs; adv->name != NULL; adv++) {
		/* enable clocks as needed */
		if (adv->clk0 != 0) {
			reg = CSR_READ_4(sc, JZ_CLKGR0);
			reg &= ~adv->clk0;
			CSR_WRITE_4(sc, JZ_CLKGR0, reg);
		}

		if (adv->clk1 != 0) {
			reg = CSR_READ_4(sc, JZ_CLKGR1);
			reg &= ~adv->clk1;
			CSR_WRITE_4(sc, JZ_CLKGR1, reg);
		}
	}

	device_printf(dev, "JZ_CPCCR  %08x\n", CSR_READ_4(sc, JZ_CPCCR));
	device_printf(dev, "JZ_CLKGR0 %08x\n", CSR_READ_4(sc, JZ_CLKGR0));
	device_printf(dev, "JZ_CLKGR1 %08x\n", CSR_READ_4(sc, JZ_CLKGR1));
	device_printf(dev, "JZ_SPCR0  %08x\n", CSR_READ_4(sc, JZ_SPCR0));
	device_printf(dev, "JZ_SPCR1  %08x\n", CSR_READ_4(sc, JZ_SPCR1));
	device_printf(dev, "JZ_SRBC   %08x\n", CSR_READ_4(sc, JZ_SRBC));
	device_printf(dev, "JZ_OPCR   %08x\n", CSR_READ_4(sc, JZ_OPCR));
	device_printf(dev, "JZ_UHCCDR %08x\n", CSR_READ_4(sc, JZ_UHCCDR));
	device_printf(dev, "JZ_ERNG   %08x\n", CSR_READ_4(sc, JZ_ERNG));
	device_printf(dev, "JZ_RNG    %08x\n", CSR_READ_4(sc, JZ_RNG));
}

