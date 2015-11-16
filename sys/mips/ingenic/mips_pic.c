/*-
 * Copyright (c) 2015 Alexander Kabaev
 * Copyright (c) 2006 Oleksandr Tymoshenko
 * Copyright (c) 2002-2004 Juli Mallett <jmallett@FreeBSD.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification, immediately at the beginning of the file.
 * 2. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include "opt_platform.h"
#include "opt_hwpmc_hooks.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/ktr.h>
#include <sys/module.h>
#include <sys/malloc.h>
#include <sys/rman.h>
#include <sys/pcpu.h>
#include <sys/proc.h>
#include <sys/cpuset.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/smp.h>
#include <sys/sched.h>
#include <machine/bus.h>
#include <machine/intr.h>
#include <machine/smp.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "pic_if.h"

#define NHARD_IRQS	6
#define NSOFT_IRQS	2
#define NREAL_IRQS	(NHARD_IRQS + NSOFT_IRQS)

#ifdef SMP
static u_int pic_irq_cpu;
static int mips_pic_bind(device_t dev, struct arm_irqsrc *isrc);
#endif
static int mips_pic_intr(void *);

struct mips_pic_softc {
	device_t		pic_dev;
	struct arm_irqsrc *	pic_irqs[NREAL_IRQS + MIPS_IPI_COUNT];
	struct mtx		mutex;
	uint32_t		nirqs;
};

static struct mips_pic_softc *pic_sc;

static struct ofw_compat_data compat_data[] = {
	{"mti,cpu-interrupt-controller",	true},
	{NULL,					false}
};

static int
mips_pic_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
		return (ENXIO);
	device_set_desc(dev, "MIPS32 Interrupt Controller");
	return (BUS_PROBE_DEFAULT);
}

static inline void
pic_irq_unmask(struct mips_pic_softc *sc, u_int irq)
{

	mips_wr_status(mips_rd_status() | ((1 << irq) << 8));
}

static inline void
pic_irq_mask(struct mips_pic_softc *sc, u_int irq)
{

	mips_wr_status(mips_rd_status() & ~((1 << irq) << 8));
}

#ifdef SMP
static void
mips_pic_init_secondary(device_t dev)
{
}
#endif /* SMP */

static inline intptr_t
pic_xref(device_t dev)
{
#ifdef FDT
	return (OF_xref_from_node(ofw_bus_get_node(dev)));
#else
	return (0);
#endif
}

static int
mips_pic_attach(device_t dev)
{
	struct		mips_pic_softc *sc;
	intptr_t	xref = pic_xref(dev);

	if (pic_sc)
		return (ENXIO);

	sc = device_get_softc(dev);

	sc->pic_dev = dev;
	pic_sc = sc;

	/* Initialize mutex */
	mtx_init(&sc->mutex, "PIC lock", "", MTX_SPIN);

	/* Set the number of interrupts */
	sc->nirqs = nitems(sc->pic_irqs);

	/*
	 * Now, when everything is initialized, it's right time to
	 * register interrupt controller to interrupt framefork.
	 */
	if (arm_pic_register(dev, xref) != 0) {
		device_printf(dev, "could not register PIC\n");
		goto cleanup;
	}

	/* Claim our root controller role */
	if (arm_pic_claim_root(dev, xref, mips_pic_intr, sc, sc->nirqs) != 0) {
		device_printf(dev, "could not set PIC as a root\n");
		arm_pic_unregister(dev, xref);
		goto cleanup;
	}

	return (0);

cleanup:
	return(ENXIO);
}

int
mips_pic_intr(void *arg)
{
	struct mips_pic_softc *sc = arg;
	register_t cause, status;
	struct arm_irqsrc *isrc;
	int i, intr;

	cause = mips_rd_cause();
	status = mips_rd_status();
	intr = (cause & MIPS_INT_MASK) >> 8;
	/*
	 * Do not handle masked interrupts. They were masked by
	 * pre_ithread function (mips_mask_XXX_intr) and will be
	 * unmasked once ithread is through with handler
	 */
	intr &= (status & MIPS_INT_MASK) >> 8;
	while ((i = fls(intr)) != 0) {
		i--; /* Get a 0-offset interrupt. */
		intr &= ~(1 << i);

		isrc = sc->pic_irqs[i];
		if (isrc == NULL) {
			device_printf(sc->pic_dev, "Stray interrupt %u detected\n", i);
			pic_irq_mask(sc, i);
			continue;
		}

		arm_irq_dispatch(isrc, curthread->td_intr_frame);
	}

	KASSERT(i == 0, ("all interrupts handled"));

#ifdef HWPMC_HOOKS
	if (pmc_hook && (PCPU_GET(curthread)->td_pflags & TDP_CALLCHAIN))
		pmc_hook(PCPU_GET(curthread), PMC_FN_USER_CALLCHAIN, tf);
#endif
	return (FILTER_HANDLED);
}

static int
pic_attach_isrc(struct mips_pic_softc *sc, struct arm_irqsrc *isrc, u_int irq)
{

	/*
	 * 1. The link between ISRC and controller must be set atomically.
	 * 2. Just do things only once in rare case when consumers
	 *    of shared interrupt came here at the same moment.
	 */
	mtx_lock_spin(&sc->mutex);
	if (sc->pic_irqs[irq] != NULL) {
		mtx_unlock_spin(&sc->mutex);
		return (sc->pic_irqs[irq] == isrc ? 0 : EEXIST);
	}
	sc->pic_irqs[irq] = isrc;
	isrc->isrc_data = irq;
	mtx_unlock_spin(&sc->mutex);

	if (irq < NSOFT_IRQS)
		arm_irq_set_name(isrc, "sint%u", irq);
	else if (irq < NREAL_IRQS)
		arm_irq_set_name(isrc, "int%u", irq - NSOFT_IRQS);
	else
		arm_irq_set_name(isrc, "ipi%u", irq - NREAL_IRQS);
	return (0);
}

static int
pic_detach_isrc(struct mips_pic_softc *sc, struct arm_irqsrc *isrc, u_int irq)
{

	mtx_lock_spin(&sc->mutex);
	if (sc->pic_irqs[irq] != isrc) {
		mtx_unlock_spin(&sc->mutex);
		return (sc->pic_irqs[irq] == NULL ? 0 : EINVAL);
	}
	sc->pic_irqs[irq] = NULL;
	isrc->isrc_data = 0;
	mtx_unlock_spin(&sc->mutex);

	arm_irq_set_name(isrc, "%s", "");
	return (0);
}

static int
pic_irq_from_nspc(struct mips_pic_softc *sc, u_int type, u_int num, u_int *irqp)
{

	switch (type) {
	case ARM_IRQ_NSPC_PLAIN:
		*irqp = num;
		return (*irqp < sc->nirqs ? 0 : EINVAL);

	case ARM_IRQ_NSPC_SWI:
		*irqp = num;
		return (num < NSOFT_IRQS ? 0 : EINVAL);

	case ARM_IRQ_NSPC_IRQ:
		*irqp = num + NSOFT_IRQS;
		return (num < NHARD_IRQS ? 0 : EINVAL);

	case ARM_IRQ_NSPC_IPI:
		*irqp = num + NHARD_IRQS + NSOFT_IRQS;
		return (num < MIPS_IPI_COUNT ? 0 : EINVAL);

	default:
		return (EINVAL);
	}
}

static int
pic_map_nspc(struct mips_pic_softc *sc, struct arm_irqsrc *isrc, u_int *irqp)
{
	int error;

	error = pic_irq_from_nspc(sc, isrc->isrc_nspc_type, isrc->isrc_nspc_num,
	    irqp);
	if (error != 0)
		return (error);
	return (pic_attach_isrc(sc, isrc, *irqp));
}

#ifdef FDT
static int
pic_map_fdt(struct mips_pic_softc *sc, struct arm_irqsrc *isrc, u_int *irqp)
{
	u_int irq;
	int error;

	irq = isrc->isrc_cells[0];

	if (irq >= sc->nirqs)
		return (EINVAL);

	error = pic_attach_isrc(sc, isrc, irq);
	if (error != 0)
		return (error);

	isrc->isrc_nspc_type = ARM_IRQ_NSPC_PLAIN;
	isrc->isrc_nspc_num = irq;
	isrc->isrc_trig = INTR_TRIGGER_CONFORM;
	isrc->isrc_pol = INTR_POLARITY_CONFORM;

	*irqp = irq;
	return (0);
}
#endif

static int
mips_pic_register(device_t dev, struct arm_irqsrc *isrc, boolean_t *is_percpu)
{
	struct mips_pic_softc *sc = device_get_softc(dev);
	u_int irq;
	int error;

	if (isrc->isrc_type == ARM_ISRCT_NAMESPACE)
		error = pic_map_nspc(sc, isrc, &irq);
#ifdef FDT
	else if (isrc->isrc_type == ARM_ISRCT_FDT)
		error = pic_map_fdt(sc, isrc, &irq);
#endif
	else
		return (EINVAL);

	if (error == 0)
		*is_percpu = TRUE;
	return (error);
}

static void
mips_pic_enable_intr(device_t dev, struct arm_irqsrc *isrc)
{

	if (isrc->isrc_trig == INTR_TRIGGER_CONFORM)
		isrc->isrc_trig = INTR_TRIGGER_LEVEL;

	/*
	 * XXX - In case that per CPU interrupt is going to be enabled in time
	 *       when SMP is already started, we need some IPI call which
	 *       enables it on others CPUs. Further, it's more complicated as
	 *       pic_enable_source() and pic_disable_source() should act on
	 *       per CPU basis only. Thus, it should be solved here somehow.
	 */
	if (isrc->isrc_flags & ARM_ISRCF_PERCPU)
		CPU_SET(PCPU_GET(cpuid), &isrc->isrc_cpu);

#ifdef SMP
	mips_pic_bind(dev, isrc);
#endif
}

static void
mips_pic_enable_source(device_t dev, struct arm_irqsrc *isrc)
{
	struct mips_pic_softc *sc = device_get_softc(dev);
	u_int irq = isrc->isrc_data;

	pic_irq_unmask(sc, irq);
}

static void
mips_pic_disable_source(device_t dev, struct arm_irqsrc *isrc)
{
	struct mips_pic_softc *sc = device_get_softc(dev);
	u_int irq = isrc->isrc_data;

	pic_irq_mask(sc, irq);
}

static int
mips_pic_unregister(device_t dev, struct arm_irqsrc *isrc)
{
	struct mips_pic_softc *sc = device_get_softc(dev);
	u_int irq = isrc->isrc_data;

	return (pic_detach_isrc(sc, isrc, irq));
}

static void
mips_pic_pre_ithread(device_t dev, struct arm_irqsrc *isrc)
{

	mips_pic_disable_source(dev, isrc);
}

static void
mips_pic_post_ithread(device_t dev, struct arm_irqsrc *isrc)
{

	mips_pic_enable_source(dev, isrc);
}

static void
mips_pic_post_filter(device_t dev, struct arm_irqsrc *isrc)
{
}

#ifdef SMP
static int
mips_pic_bind(device_t dev, struct arm_irqsrc *isrc)
{
	struct mips_pic_softc *sc = device_get_softc(dev);
	uint32_t irq = isrc->isrc_data;

	if (CPU_EMPTY(&isrc->isrc_cpu)) {
		pic_irq_cpu = arm_irq_next_cpu(gic_irq_cpu, &all_cpus);
		CPU_SETOF(pic_irq_cpu, &isrc->isrc_cpu);
	}
	return (pic_bind(sc, irq, &isrc->isrc_cpu));
}

static void
mips_pic_ipi_send(device_t dev, struct arm_irqsrc *isrc, cpuset_t cpus)
{
	struct mips_pic_softc *sc = device_get_softc(dev);
	uint32_t irq, val = 0, i;

	irq = isrc->isrc_data;

	for (i = 0; i < MAXCPU; i++)
		if (CPU_ISSET(i, &cpus))
			val |= 1 << (16 + i);
}
#endif

static device_method_t mips_pic_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		mips_pic_probe),
	DEVMETHOD(device_attach,	mips_pic_attach),
	/* Interrupt controller interface */
	DEVMETHOD(pic_disable_source,	mips_pic_disable_source),
	DEVMETHOD(pic_enable_intr,	mips_pic_enable_intr),
	DEVMETHOD(pic_enable_source,	mips_pic_enable_source),
	DEVMETHOD(pic_post_filter,	mips_pic_post_filter),
	DEVMETHOD(pic_post_ithread,	mips_pic_post_ithread),
	DEVMETHOD(pic_pre_ithread,	mips_pic_pre_ithread),
	DEVMETHOD(pic_register,		mips_pic_register),
	DEVMETHOD(pic_unregister,	mips_pic_unregister),
#ifdef SMP
	DEVMETHOD(pic_bind,		mips_pic_bind),
	DEVMETHOD(pic_init_secondary,	mips_pic_init_secondary),
	DEVMETHOD(pic_ipi_send,		mips_pic_ipi_send),
#endif
	{ 0, 0 }
};

static driver_t mips_pic_driver = {
	"mips_pic",
	mips_pic_methods,
	sizeof(struct mips_pic_softc),
};

static devclass_t mips_pic_devclass;

EARLY_DRIVER_MODULE(cpupic, ofwbus, mips_pic_driver, mips_pic_devclass, 0, 0,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_MIDDLE);

void
cpu_init_interrupts(void)
{
}

void
cpu_establish_hardintr(const char *name, driver_filter_t *filt,
    void (*handler)(void*), void *arg, int irq, int flags, void **cookiep)
{
	/*
	 * We have 6 levels, but thats 0 - 5 (not including 6)
	 */
	if (irq < 0 || irq >= NHARD_IRQS)
		panic("%s called for unknown hard intr %d", __func__, irq);
}

void
cpu_establish_softintr(const char *name, driver_filter_t *filt,
    void (*handler)(void*), void *arg, int irq, int flags,
    void **cookiep)
{

	if (irq < 0 || irq > NSOFT_IRQS)
		panic("%s called for unknown hard intr %d", __func__, irq);

	KASSERT(pic_sc != NULL, ("%s: no pic", __func__));
}

