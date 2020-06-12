/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2018 Emmanuel Vadot <manu@freebsd.org>
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

#include <dev/extres/clk/clk.h>

#include <arm64/freescale/imx/clk/imx_clk_frac_pll.h>

#include "clkdev_if.h"

struct imx_clk_frac_pll_sc {
	uint32_t	offset;
};

#define	WRITE4(_clk, off, val)						\
	CLKDEV_WRITE_4(clknode_get_device(_clk), off, val)
#define	READ4(_clk, off, val)						\
	CLKDEV_READ_4(clknode_get_device(_clk), off, val)
#define	DEVICE_LOCK(_clk)						\
	CLKDEV_DEVICE_LOCK(clknode_get_device(_clk))
#define	DEVICE_UNLOCK(_clk)						\
	CLKDEV_DEVICE_UNLOCK(clknode_get_device(_clk))

#define	CFG0	0

#if 0
#define	dprintf(format, arg...)						\
	printf("%s:(%s)" format, __func__, clknode_get_name(clk), arg)
#else
#define	dprintf(format, arg...)
#endif

static int
imx_clk_frac_pll_init(struct clknode *clk, device_t dev)
{
	struct imx_clk_frac_pll_sc *sc;
	uint32_t val, idx;

	sc = clknode_get_softc(clk);

	DEVICE_LOCK(clk);
	READ4(clk, sc->offset + CFG0, &val);
	DEVICE_UNLOCK(clk);
	idx = 0; /* TODO */

	clknode_init_parent_idx(clk, idx);

	return (0);
}

static int
imx_clk_frac_pll_set_gate(struct clknode *clk, bool enable)
{
	panic("[%s] %s:%d\n", __func__, __FILE__, __LINE__);
	return (0);
}

static int
imx_clk_frac_pll_set_mux(struct clknode *clk, int index)
{
	panic("[%s] %s:%d\n", __func__, __FILE__, __LINE__);
	return (0);
}

static int
imx_clk_frac_pll_recalc(struct clknode *clk, uint64_t *freq)
{
	panic("[%s] %s:%d\n", __func__, __FILE__, __LINE__);
	return (0);
}

static int
imx_clk_frac_pll_set_freq(struct clknode *clk, uint64_t fparent, uint64_t *fout,
    int flags, int *stop)
{
	panic("[%s] %s:%d\n", __func__, __FILE__, __LINE__);
	return (0);
}

static clknode_method_t imx_clk_frac_pll_clknode_methods[] = {
	/* Device interface */
	CLKNODEMETHOD(clknode_init,		imx_clk_frac_pll_init),
	CLKNODEMETHOD(clknode_set_gate,		imx_clk_frac_pll_set_gate),
	CLKNODEMETHOD(clknode_set_mux,		imx_clk_frac_pll_set_mux),
	CLKNODEMETHOD(clknode_recalc_freq,	imx_clk_frac_pll_recalc),
	CLKNODEMETHOD(clknode_set_freq,		imx_clk_frac_pll_set_freq),
	CLKNODEMETHOD_END
};

DEFINE_CLASS_1(imx_clk_frac_pll_clknode, imx_clk_frac_pll_clknode_class,
    imx_clk_frac_pll_clknode_methods, sizeof(struct imx_clk_frac_pll_sc),
    clknode_class);

int
imx_clk_frac_pll_register(struct clkdom *clkdom,
    struct imx_clk_frac_pll_def *clkdef)
{
	struct clknode *clk;
	struct imx_clk_frac_pll_sc *sc;

	clk = clknode_create(clkdom, &imx_clk_frac_pll_clknode_class,
	    &clkdef->clkdef);
	if (clk == NULL)
		return (1);

	sc = clknode_get_softc(clk);

	sc->offset = clkdef->offset;

	clknode_register(clkdom, clk);

	return (0);
}
