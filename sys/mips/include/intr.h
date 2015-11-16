/* 	$NetBSD: intr.h,v 1.7 2003/06/16 20:01:00 thorpej Exp $	*/

/*-
 * Copyright (c) 1997 Mark Brinicombe.
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
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed by Mark Brinicombe
 *	for the NetBSD Project.
 * 4. The name of the company nor the name of the author may be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 *
 */

#ifndef _MACHINE_INTR_H_
#define _MACHINE_INTR_H_

#ifdef FDT
#include <dev/ofw/openfirm.h>
#endif

#ifndef NIRQ
#define	NIRQ		128	/* XXX - It should be an option and match size used in exception.S */
#endif

#ifdef notyet
#define	INTR_SOLO	INTR_MD1
typedef int arm_irq_filter_t(void *arg, struct trapframe *tf);
#else
typedef int arm_irq_filter_t(void *arg);
#endif

#define ARM_ISRC_NAMELEN	(MAXCOMLEN + 1)

typedef void arm_ipi_filter_t(void *arg);

enum arm_isrc_type {
	ARM_ISRCT_NAMESPACE,
	ARM_ISRCT_FDT
};

#define ARM_ISRCF_REGISTERED	0x01	/* registered in a controller */
#define ARM_ISRCF_PERCPU	0x02	/* per CPU interrupt */
#define ARM_ISRCF_BOUND		0x04	/* bound to a CPU */

/* Interrupt source definition. */
struct arm_irqsrc {
	device_t		isrc_dev;	/* where isrc is mapped */
	intptr_t		isrc_xref;	/* device reference key */
	uintptr_t		isrc_data;	/* device data for isrc */
	u_int			isrc_irq;	/* unique identificator */
	enum arm_isrc_type	isrc_type;	/* how is isrc decribed */
	u_int			isrc_flags;
	char			isrc_name[ARM_ISRC_NAMELEN];
	uint16_t		isrc_nspc_type;
	uint16_t		isrc_nspc_num;
	enum intr_trigger	isrc_trig;
	enum intr_polarity	isrc_pol;
	cpuset_t		isrc_cpu;	/* on which CPUs is enabled */
	u_int			isrc_index;
	u_long *		isrc_count;
	u_int			isrc_handlers;
	struct intr_event *	isrc_event;
	arm_irq_filter_t *	isrc_filter;
	arm_ipi_filter_t *	isrc_ipifilter;
	void *			isrc_arg;
#ifdef FDT
	u_int			isrc_ncells;
	pcell_t			isrc_cells[];	/* leave it last */
#endif
};

void arm_irq_set_name(struct arm_irqsrc *isrc, const char *fmt, ...)
    __printflike(2, 3);

void arm_irq_dispatch(struct arm_irqsrc *isrc, struct trapframe *tf);

#define ARM_IRQ_NSPC_NONE	0
#define ARM_IRQ_NSPC_PLAIN	1
#define ARM_IRQ_NSPC_IRQ	2
#define ARM_IRQ_NSPC_IPI	3
#define ARM_IRQ_NSPC_SWI	4

u_int arm_namespace_map_irq(device_t dev, uint16_t type, uint16_t num);
#ifdef FDT
u_int arm_fdt_map_irq(phandle_t, pcell_t *, u_int);
#endif

int arm_pic_register(device_t dev, intptr_t xref);
int arm_pic_unregister(device_t dev, intptr_t xref);
int arm_pic_claim_root(device_t dev, intptr_t xref, arm_irq_filter_t *filter,
    void *arg, u_int ipicount);

int arm_irq_add_handler(device_t dev, driver_filter_t, driver_intr_t, void *,
    u_int, int, void **);
int arm_irq_remove_handler(device_t dev, u_int, void *);
int arm_irq_config(u_int, enum intr_trigger, enum intr_polarity);
int arm_irq_describe(u_int, void *, const char *);

u_int arm_irq_next_cpu(u_int current_cpu, cpuset_t *cpumask);

#ifdef SMP
int arm_irq_bind(u_int, int);

void arm_ipi_dispatch(struct arm_irqsrc *isrc, struct trapframe *tf);

#define AISHF_NOALLOC	0x0001

int arm_ipi_set_handler(u_int ipi, const char *name, arm_ipi_filter_t *filter,
    void *arg, u_int flags);

void arm_pic_init_secondary(void);
#endif

/* MIPS compatibility for legacy mips code */
void cpu_init_interrupts(void);
void cpu_establish_hardintr(const char *, driver_filter_t *, driver_intr_t *,
    void *, int, int, void **);
void cpu_establish_softintr(const char *, driver_filter_t *, void (*)(void*),
    void *, int, int, void **);
/* MIPS interrupt C entry point */
void cpu_intr(struct trapframe *);


#endif	/* _MACHINE_INTR_H */
