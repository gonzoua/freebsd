/*-
 * Copyright (c) 2009 Oleksandr Tymoshenko
 * Copyright (c) 2015 Alexander Kabaev
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

#include "opt_ddb.h"

#include <sys/param.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/cons.h>
#include <sys/kdb.h>
#include <sys/reboot.h>

#include <vm/vm.h>
#include <vm/vm_page.h>

#include <net/ethernet.h>

#include <machine/clock.h>
#include <machine/cpu.h>
#include <machine/cpuregs.h>
#include <machine/hwfunc.h>
#include <machine/md_var.h>
#include <machine/trap.h>
#include <machine/vmparam.h>

#include <mips/ingenic/jz47xx_regs.h>

uint32_t * const led = (uint32_t *)0xb0010548;

extern char edata[], end[];

void
platform_cpu_init()
{
	/* Nothing special */
}

void
platform_reset(void)
{
	/*
	 * For now, provoke a watchdog reset in about a second, so UART buffers
	 * have a fighting chance to flush before we pull the plug
	 */
	writereg(JZ_WDOG_TCER, 0);	/* disable watchdog */
	writereg(JZ_WDOG_TCNT, 0);	/* reset counter */
	writereg(JZ_WDOG_TDR, 128);	/* wait for ~1s */
	writereg(JZ_WDOG_TCSR, TCSR_RTC_EN | TCSR_DIV_256);
	writereg(JZ_WDOG_TCER, TCER_ENABLE);	/* fire! */

	/* Wait for reset */
	while (1)
		;
}

extern char cpu_model[];

void
platform_start(__register_t a0 __unused, __register_t a1 __unused,
    __register_t a2 __unused, __register_t a3 __unused)
{
	uint64_t platform_counter_freq;
	vm_offset_t kernend;

	/*
	 * clear the BSS and SBSS segments, this should be first call in
	 * the function
	 */
	kernend = (vm_offset_t)&end;
	memset(&edata, 0, kernend - (vm_offset_t)(&edata));

	mips_postboot_fixup();

	/* Initialize pcpu stuff */
	mips_pcpu0_init();

	/* Initialize OST */
	platform_counter_freq = 12000000;
	mips_timer_early_init(platform_counter_freq);

	bootverbose = 1;

	/* phys_avail regions are in bytes */
	phys_avail[0] = MIPS_KSEG0_TO_PHYS(kernel_kseg0_end);
	phys_avail[1] = 256 * 1024 * 1024;

	phys_avail[2] = 0x30000000;
	phys_avail[3] = phys_avail[2] + 768 * 1024 * 1024;

	dump_avail[0] = phys_avail[0];
	dump_avail[1] = phys_avail[1] - phys_avail[0];

	dump_avail[2] = phys_avail[2];
	dump_avail[3] = phys_avail[3] - phys_avail[2];

	physmem = realmem = btoc(1 * 1024 * 1024 * 1024);

	/*
	 * ns8250 uart code uses DELAY so ticker should be inititalized
	 * before cninit. And mips_timer_init_params refers to hz, so * init_param1
	 * should be called first.
	 */
	init_param1();

	mips_timer_init_params(platform_counter_freq, 1);

	cninit();

	strcpy(cpu_model, "Blah");

	/* Platform setup */
	init_param2(physmem);
	mips_cpu_init();
	pmap_bootstrap();
	mips_proc0_init();
	mutex_init();

	kdb_init();
#ifdef KDB
	if (boothowto & RB_KDB)
 		kdb_enter(KDB_WHY_BOOTFLAGS, "Boot flags requested debugger");
#endif
}
