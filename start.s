/*
 * armboot - Startup Code for OMAP3530/ARM Cortex CPU-core
 *
 * Copyright (c) 2004	Texas Instruments <r-woodruff2@ti.com>
 *
 * Copyright (c) 2001	Marius Gröger <mag@sysgo.de>
 * Copyright (c) 2002	Alex Züpke <azu@sysgo.de>
 * Copyright (c) 2002	Gary Jennejohn <garyj@denx.de>
 * Copyright (c) 2003	Richard Woodruff <r-woodruff2@ti.com>
 * Copyright (c) 2003	Kshitij <kshitij@ti.com>
 * Copyright (c) 2006-2008 Syed Mohammed Khasim <x0khasim@ti.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

/*
#include <asm-offsets.h>
#include <config.h>
#include <asm/armv7.h>
*/

//#include "system.h"
//#include "linkage.h"

/*************************************************************************
 *
 * Startup Code (reset vector)
 *
 * Do important init only if we don't start from memory!
 * Setup memory and board specific bits prior to relocation.
 * Relocate armboot to ram. Setup stack.
 *
 *************************************************************************/

	.globl	reset
	.globl	save_boot_params_ret
	.globl	cpu_init_cp15
	.globl	cpu_init_crit
	.globl _start
	.globl lowlevel_init


/*
 *************************************************************************
 *
 * Vectors have their own section so linker script can map them easily
 *
 *************************************************************************
 */

	.section ".vectors", "ax"

/*
 *************************************************************************
 *
 * Exception vectors as described in ARM reference manuals
 *
 * Uses indirect branch to allow reaching handlers anywhere in memory.
 *
 *************************************************************************
 */

	.globl	_undefined_instruction
	.globl	_software_interrupt
	.globl	_prefetch_abort
	.globl	_data_abort
	.globl	_not_used
	.globl	_irq
	.globl	_fiq
	
_start:

	b	reset
	ldr	pc, _undefined_instruction
	ldr	pc, _software_interrupt
	ldr	pc, _prefetch_abort
	ldr	pc, _data_abort
	ldr	pc, _not_used
	ldr	pc, _irq
	ldr	pc, _fiq


/*
 *************************************************************************
 *
 * Indirect vectors table
 *
 * Symbols referenced here must be defined somewhere else
 *
 *************************************************************************
 */


_undefined_instruction:	.word undefined_instruction
_software_interrupt:	.word software_interrupt
_prefetch_abort:		.word prefetch_abort
_data_abort:			.word data_abort
_not_used:				.word not_used
_irq:					.word irq
_fiq:					.word fiq

//	.balignl 16,0xdeadbeef
	.word 0xdeadbeef


///*
// *************************************************************************
// *
// * Interrupt handling
// *
// *************************************************************************
// */
//
///* SPL interrupt handling: just hang */
//
//

//	.align	5
undefined_instruction:
software_interrupt:
prefetch_abort:
data_abort:
not_used:
irq:
fiq:
1:
	bl	1b			/* hang and never return */

	
//#ifdef CONFIG_ARMV7_LPAE
//	.global	switch_to_hypervisor_ret
//#endif

reset:
//	/* Allow the board to save important registers */
//	b	save_boot_params
save_boot_params_ret:

	/*
	 * disable interrupts (FIQ and IRQ), also set the cpu to SVC32 mode,
	 * except if in HYP mode already
	 */
	mrs		r0, cpsr
	and		r1, r0, #0x1f		@ mask mode bits
	teq		r1, #0x1a			@ test for HYP mode
	bicne	r0, r0, #0x1f		@ clear all mode bits
	orrne	r0, r0, #0x13		@ set SVC mode
	orr	r0, r0, #0xc0			@ disable FIQ and IRQ
	msr		cpsr,r0

/*
 * Setup vector:
 * (OMAP4 spl TEXT_BASE is not 32 byte aligned.
 * Continue to use ROM code vector only in OMAP4 spl)
 */

	/* Set V=0 in CP15 SCTLR register - for VBAR to point to vector */
	mrc	p15, 0, r0, c1, c0, 0	@ Read CP15 SCTLR Register
	bic	r0, #0x2000				@ C_RV = 0x2000 - V = 0
	mcr	p15, 0, r0, c1, c0, 0	@ Write CP15 SCTLR Register

	/* Set vector address in CP15 VBAR register */
	ldr	r0, =_start
	mcr	p15, 0, r0, c12, c0, 0	@Set VBAR

	/* the mask ROM code should have PLL and others stable */
	bl	cpu_init_cp15
	bl	cpu_init_crit

	/* FDy : call board_init_f from \arch\arm\mach-socfpga\spl.c instead of main for crt0.lib */ 
	//bl		_main
	bl			board_init_f	// doesn't work directly without a call to _main

/*------------------------------------------------------------------------------*/

//ENTRY(c_runtime_cpu_setup)
.globl c_runtime_cpu_setup;
/*
 * If I-cache is enabled invalidate it
 */

	mcr		p15, 0, r0, c7, c5, 0	@ invalidate icache
	mcr     p15, 0, r0, c7, c10, 4	@ DSB
	mcr     p15, 0, r0, c7, c5, 4	@ ISB

	bx	lr

//ENDPROC(c_runtime_cpu_setup)

/*************************************************************************
 *
 * void save_boot_params(u32 r0, u32 r1, u32 r2, u32 r3)
 *	__attribute__((weak));
 *
 * Stack pointer is not yet initialized at this moment
 * Don't save anything to stack even if compiled with -O0
 *
 *************************************************************************/
//ENTRY(save_boot_params)
.globl save_boot_params;
	b	save_boot_params_ret		@ back to my caller
//ENDPROC(save_boot_params)
	.weak	save_boot_params


/*************************************************************************
 *
 * cpu_init_cp15
 *
 * Setup CP15 registers (cache, MMU, TLBs). The I-cache is turned on unless
 * CONFIG_SYS_ICACHE_OFF is defined.
 *
 *************************************************************************/
//ENTRY(cpu_init_cp15)
cpu_init_cp15:
.align 4

	/*
	 * Invalidate L1 I/D
	 */
	mov		r0, #0					@ set up for MCR
	mcr		p15, 0, r0, c8, c7, 0	@ invalidate TLBs
	mcr		p15, 0, r0, c7, c5, 0	@ invalidate icache
	mcr		p15, 0, r0, c7, c5, 6	@ invalidate BP array
	mcr     p15, 0, r0, c7, c10, 4	@ DSB
	mcr     p15, 0, r0, c7, c5, 4	@ ISB

	/*
	 * disable MMU stuff and caches
	 */
	mrc	p15, 0, r0, c1, c0, 0
	bic	r0, r0, #0x00002000		@ clear bits 13 (--V-)
	bic	r0, r0, #0x00000007		@ clear bits 2:0 (-CAM)
	orr	r0, r0, #0x00000002		@ set bit 1 (--A-) Align
	orr	r0, r0, #0x00000800		@ set bit 11 (Z---) BTB
	orr	r0, r0, #0x00001000		@ set bit 12 (I) I-cache
	mcr	p15, 0, r0, c1, c0, 0

	mov	r5, lr					@ Store my Caller
	mrc	p15, 0, r1, c0, c0, 0	@ r1 has Read Main ID Register (MIDR)
	mov	r3, r1, lsr #20			@ get variant field
	and	r3, r3, #0xf			@ r3 has CPU variant
	and	r4, r1, #0xf			@ r4 has CPU revision
	mov	r2, r3, lsl #4			@ shift variant field for combined value
	orr	r2, r4, r2				@ r2 has combined CPU variant + revision


	mov	pc, r5					@ back to my caller
//ENDPROC(cpu_init_cp15)
//.size cpu_init_cp15, .-cpu_init_cp15
//.type cpu_init_cp15;

/*************************************************************************
 *
 * CPU_init_critical registers
 *
 * setup important registers
 * setup memory timing
 *
 *************************************************************************/
//ENTRY(cpu_init_crit)
cpu_init_crit:
.align 4
	
	/*
	 * Jump to board specific initialization...
	 * The Mask ROM will have already initialized
	 * basic memory. Go here to bump up clock rate and handle
	 * wake up conditions.
	 */
	b	lowlevel_init		@ go setup pll,mux,memory*/
	bx 	lr
//ENDPROC(cpu_init_crit)
//.size cpu_init_crit, .-cpu_init_crit
//.-cpu_init_crit

