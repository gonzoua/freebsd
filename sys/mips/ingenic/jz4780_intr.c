/*-
 * Copyright (c) 2015 Alexander Kabaev
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

#include "opt_platform.h"

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

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

#include <mips/ingenic/jz4780_regs.h>

#include "pic_if.h"

#define	JZ4780_NIRQS	64

static int jz4780_pic_intr(void *);

struct jz4780_pic_softc {
	device_t		pic_dev;
	void *                  pic_intrhand;
	struct resource *       pic_res[2];
	struct arm_irqsrc *	pic_irqs[JZ4780_NIRQS];
	struct mtx		mutex;
	uint32_t		nirqs;
};

static struct resource_spec jz4780_pic_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },	/* Registers */
	{ SYS_RES_IRQ,		0,	RF_ACTIVE },	/* Parent interrupt */
	{ -1, 0 }
};

static struct ofw_compat_data compat_data[] = {
	{"ingenic,jz4780-intc",	true},
	{NULL,			false}
};

#define	READ4(_sc, _reg)	bus_read_4((_sc)->pic_res[0], _reg)
#define	WRITE4(_sc, _reg, _val) bus_write_4((_sc)->pic_res[0], _reg, _val)

static int
jz4780_pic_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
		return (ENXIO);
	device_set_desc(dev, "JZ4780 Interrupt Controller");
	return (BUS_PROBE_DEFAULT);
}

static inline void
pic_irq_unmask(struct jz4780_pic_softc *sc, u_int irq)
{
	if (irq < 32)
		WRITE4(sc, JZ_ICMCR0, (1u << irq));
	else
		WRITE4(sc, JZ_ICMSR1, (1u << (irq - 32)));
}

static inline void
pic_irq_mask(struct jz4780_pic_softc *sc, u_int irq)
{
	if (irq < 32)
		WRITE4(sc, JZ_ICMSR1, (1u << irq));
	else
		WRITE4(sc, JZ_ICMSR1, (1u << (irq - 32)));
}

static inline intptr_t
pic_xref(device_t dev)
{
	return (OF_xref_from_node(ofw_bus_get_node(dev)));
}

static int
jz4780_pic_attach(device_t dev)
{
	struct jz4780_pic_softc *sc;
	intptr_t xref = pic_xref(dev);

	sc = device_get_softc(dev);

	if (bus_alloc_resources(dev, jz4780_pic_spec, sc->pic_res)) {
		device_printf(dev, "could not allocate resources\n");
		return (ENXIO);
	}

	sc->pic_dev = dev;

	/* Initialize mutex */
	mtx_init(&sc->mutex, "PIC lock", "", MTX_SPIN);

	/* Set the number of interrupts */
	sc->nirqs = nitems(sc->pic_irqs);

	/* Mask all interrupts */
	WRITE4(sc, JZ_ICMR0, 0xFFFFFFFF);
	WRITE4(sc, JZ_ICMR1, 0xFFFFFFFF);

	/*
	 * Now, when everything is initialized, it's right time to
	 * register interrupt controller to interrupt framefork.
	 */
	if (arm_pic_register(dev, xref) != 0) {
		device_printf(dev, "could not register PIC\n");
		goto cleanup;
	}

	if (bus_setup_intr(dev, sc->pic_res[1], INTR_TYPE_CLK,
	    jz4780_pic_intr, NULL, sc, &sc->pic_intrhand)) {
		device_printf(dev, "could not setup irq handler\n");
		arm_pic_unregister(dev, xref);
		goto cleanup;
	}
	return (0);

cleanup:
	bus_release_resources(dev, jz4780_pic_spec, sc->pic_res);
	return(ENXIO);
}

static int
jz4780_pic_intr(void *arg)
{
	struct jz4780_pic_softc *sc = arg;
	struct arm_irqsrc *isrc;
	uint32_t i, intr;

	intr = READ4(sc, JZ_ICPR0);

	while ((i = fls(intr)) != 0) {
		i--;
		intr &= ~(1u << i);

		isrc = sc->pic_irqs[i];
		if (isrc == NULL) {
			device_printf(sc->pic_dev, "Stray interrupt %u detected\n", i);
			pic_irq_mask(sc, i);
			continue;
		}
		arm_irq_dispatch(isrc, curthread->td_intr_frame);
	}

	KASSERT(i == 0, ("all interrupts handled"));

	intr = READ4(sc, JZ_ICPR1);

	while ((i = fls(intr)) != 0) {
		i--;
		intr &= ~(1u << i);
		i += 32;

		isrc = sc->pic_irqs[i];
		if (isrc == NULL) {
			device_printf(sc->pic_dev, "Stray interrupt %u detected\n", i);
			pic_irq_mask(sc, i);
			continue;
		}
		arm_irq_dispatch(isrc, curthread->td_intr_frame);
	}

	KASSERT(i == 0, ("all interrupts handled"));

	return (FILTER_HANDLED);
}

static int
pic_attach_isrc(struct jz4780_pic_softc *sc, struct arm_irqsrc *isrc, u_int irq)
{
	const char *name;

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

	name = device_get_nameunit(sc->pic_dev);
	arm_irq_set_name(isrc, "%s,i%u", name, irq);
	return (0);
}

static int
pic_detach_isrc(struct jz4780_pic_softc *sc, struct arm_irqsrc *isrc, u_int irq)
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
pic_map_fdt(struct jz4780_pic_softc *sc, struct arm_irqsrc *isrc, u_int *irqp)
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

static int
jz4780_pic_register(device_t dev, struct arm_irqsrc *isrc, boolean_t *is_percpu)
{
	struct jz4780_pic_softc *sc = device_get_softc(dev);
	u_int irq;
	int error;

	if (isrc->isrc_type == ARM_ISRCT_FDT)
		error = pic_map_fdt(sc, isrc, &irq);
	else
		return (EINVAL);

	if (error == 0)
		*is_percpu = TRUE;
	return (error);
}

static void
jz4780_pic_enable_intr(device_t dev, struct arm_irqsrc *isrc)
{
	if (isrc->isrc_trig == INTR_TRIGGER_CONFORM)
		isrc->isrc_trig = INTR_TRIGGER_LEVEL;
}

static void
jz4780_pic_enable_source(device_t dev, struct arm_irqsrc *isrc)
{
	struct jz4780_pic_softc *sc = device_get_softc(dev);
	u_int irq = isrc->isrc_data;

	pic_irq_unmask(sc, irq);
}

static void
jz4780_pic_disable_source(device_t dev, struct arm_irqsrc *isrc)
{
	struct jz4780_pic_softc *sc = device_get_softc(dev);
	u_int irq = isrc->isrc_data;

	pic_irq_mask(sc, irq);
}

static int
jz4780_pic_unregister(device_t dev, struct arm_irqsrc *isrc)
{
	struct jz4780_pic_softc *sc = device_get_softc(dev);
	u_int irq = isrc->isrc_data;

	return (pic_detach_isrc(sc, isrc, irq));
}

static void
jz4780_pic_pre_ithread(device_t dev, struct arm_irqsrc *isrc)
{

	jz4780_pic_disable_source(dev, isrc);
}

static void
jz4780_pic_post_ithread(device_t dev, struct arm_irqsrc *isrc)
{

	jz4780_pic_enable_source(dev, isrc);
}

static void
jz4780_pic_post_filter(device_t dev, struct arm_irqsrc *isrc)
{
}

static device_method_t jz4780_pic_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		jz4780_pic_probe),
	DEVMETHOD(device_attach,	jz4780_pic_attach),
	/* Interrupt controller interface */
	DEVMETHOD(pic_disable_source,	jz4780_pic_disable_source),
	DEVMETHOD(pic_enable_intr,	jz4780_pic_enable_intr),
	DEVMETHOD(pic_enable_source,	jz4780_pic_enable_source),
	DEVMETHOD(pic_post_filter,	jz4780_pic_post_filter),
	DEVMETHOD(pic_post_ithread,	jz4780_pic_post_ithread),
	DEVMETHOD(pic_pre_ithread,	jz4780_pic_pre_ithread),
	DEVMETHOD(pic_register,		jz4780_pic_register),
	DEVMETHOD(pic_unregister,	jz4780_pic_unregister),
	{ 0, 0 }
};

static driver_t jz4780_pic_driver = {
	"intc",
	jz4780_pic_methods,
	sizeof(struct jz4780_pic_softc),
};

static devclass_t jz4780_pic_devclass;

EARLY_DRIVER_MODULE(intc, ofwbus, jz4780_pic_driver, jz4780_pic_devclass, 0, 0,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_MIDDLE);
