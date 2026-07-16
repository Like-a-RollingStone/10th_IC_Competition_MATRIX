#include "regdef.h"
#include "uart_print.h"
#include "handler.h"

	.section .init
	.globl _start
	.type _start,@function

_start:

    ## init regs
    addi.w $r1,zero,0x0;  addi.w $r2,zero,0x0;  addi.w $r3,zero,0x0;  addi.w $r4,zero,0x0;
    addi.w $r5,zero,0x0;  addi.w $r6,zero,0x0;  addi.w $r7,zero,0x0;  addi.w $r8,zero,0x0;
    addi.w $r9,zero,0x0;  addi.w $r10,zero,0x0; addi.w $r11,zero,0x0; addi.w $r12,zero,0x0;
    addi.w $r13,zero,0x0; addi.w $r14,zero,0x0; addi.w $r15,zero,0x0; addi.w $r16,zero,0x0;
    addi.w $r17,zero,0x0; addi.w $r18,zero,0x0; addi.w $r19,zero,0x0; addi.w $r20,zero,0x0;
    addi.w $r21,zero,0x0; addi.w $r22,zero,0x0; addi.w $r23,zero,0x0; addi.w $r24,zero,0x0;
    addi.w $r25,zero,0x0; addi.w $r26,zero,0x0; addi.w $r27,zero,0x0; addi.w $r28,zero,0x0;
    addi.w $r29,zero,0x0; addi.w $r30,zero,0x0; addi.w $r31,zero,0x0;

    /* Establish the normal CPU address mapping before touching MMIO. */
	/* open da mode */
    li.w	    $r12,0x8
    li.w	    $r13,0x18
    csrxchg	$r12,$r13,0x0

    /* init dmw */
    csrwr	$r0,0x180
    csrwr	$r0,0x181
    li.w    $r12,0x09
    csrwr	$r12,0x180
    li.w    $r12,0xa0000009
    csrwr	$r12,0x181

    /* open pg mode */
    li.w	    $r12,0x10
    li.w	    $r13,0x18
    csrxchg	$r12,$r13,0x0

    /* CPU MMIO routine: initialize UART, emit MATMUL_START, then configure
     * and start the AXI-attached accelerator.  The full cache invalidation
     * below remains intact, but now overlaps accelerator execution. */
    bl matmul_early_start

#if has_cache==1
    # invalidate old instructions in icache and old data in dcache by index
    li.w t0,0x0
    li.w t2, cache_index_depth
1:
    slli.w    t1, t0, cache_offset_width
#if cache_way==1
    cacop     0x00, t1, 0x0
    cacop     0x01, t1, 0x0
#elif cache_way==2
    cacop     0x00, t1, 0x0
    cacop     0x00, t1, 0x1
    cacop     0x01, t1, 0x0
    cacop     0x01, t1, 0x1
#elif cache_way==4
    cacop     0x00, t1, 0x0
    cacop     0x00, t1, 0x1
    cacop     0x00, t1, 0x2
    cacop     0x00, t1, 0x3
    cacop     0x01, t1, 0x0
    cacop     0x01, t1, 0x1
    cacop     0x01, t1, 0x2
    cacop     0x01, t1, 0x3
#endif
    addi.w    t0, t0, 0x1
    bne       t0, t2, 1b
#else
    /* disable cache */
    li.w    $r12,0x1
    csrwr  $r12,0x101
#endif

    /* load data section */
	la.local t0, _data_lma
	la.local t1, _data
	la.local t2, _edata
	bgeu t1, t2, 2f
1:
	ld.w t3, t0, 0
	st.w t3, t1, 0
	addi.w t0, t0, 4
	addi.w t1, t1, 4
	bltu t1, t2, 1b
2:

	/* clear bss section */
	la.local t0, __bss_start
	la.local t1, _end
	bgeu t0, t1, 2f
1:
	st.w $r0, t0, 0
	addi.w t0, t0, 4
	bltu t0, t1, 1b
2:

    /* enable cache */
#if has_cache==1
    li.w    $r12,0x19
    csrwr  $r12,0x180
#endif

    /* init exception */
#ifdef RTThread
    la.local t0, rtthread_irq_entry
#else
    la.local t0, trap_handler
#endif
    csrwr    t0, csr_eentry

    # clear int
    li.w        t1, 0x1
    csrwr       t1, csr_ticlr

    # enable int
    li.w        t1, 0x4
#ifdef RTThread
    csrxchg     zero, t1, csr_crmd
#else
    csrxchg     t1, t1, csr_crmd
#endif
    csrwr       zero, csr_prmd
    li.w        t1, 0x1fff
    csrwr       zero, csr_ecfg
    csrwr       t1, csr_ecfg

	la.local sp, _stack

	/* argc = argv = 0 */
	li.w a0, 0
	li.w a1, 0

#ifdef RTThread
    bl entry
#else
	bl main
#endif

	/* tail exit */
	bl _myexit
1:
	b 1b

.globl _myexit
.org 0x200
_myexit:
1:
	b 1b

    /* Kept outside .init so the fixed 0x200 startup layout is preserved. */
    .section .text.matmul_early_start,"ax"
    .type matmul_early_start,@function
matmul_early_start:
    /* UART at 0xbf000000, 115200 baud.  No initialized C data is used here. */
    li.w        t0, 0xbf000000
    li.w        t2, 0x07
    st.b        t2, t0, 2
    li.w        t2, 0x80
    st.b        t2, t0, 3
    li.w        t2, 0x00
    st.b        t2, t0, 1
    li.w        t3, 0xbf20f500
    ld.w        t3, t3, 0
    li.w        t2, 0x1b
    beq         zero, t3, 1f
    li.w        t2, 0x1
1:
    st.b        t2, t0, 0
    li.w        t2, 0x03
    st.b        t2, t0, 3
    st.b        zero, t0, 4

    /* "MATMUL_START\n" fits in the 16-byte UART TX FIFO. */
    li.w t2, 'M'; st.b t2, t0, 0
    li.w t2, 'A'; st.b t2, t0, 0
    li.w t2, 'T'; st.b t2, t0, 0
    li.w t2, 'M'; st.b t2, t0, 0
    li.w t2, 'U'; st.b t2, t0, 0
    li.w t2, 'L'; st.b t2, t0, 0
    li.w t2, '_'; st.b t2, t0, 0
    li.w t2, 'S'; st.b t2, t0, 0
    li.w t2, 'T'; st.b t2, t0, 0
    li.w t2, 'A'; st.b t2, t0, 0
    li.w t2, 'R'; st.b t2, t0, 0
    li.w t2, 'T'; st.b t2, t0, 0
    li.w t2, 10;  st.b t2, t0, 0

    /* Matrix accelerator AXI slave at 0xbf500000. */
    li.w        t0, 0xbf500000
    li.w        t1, 0x1c400000
    st.w        t1, t0, 0x150
    li.w        t2, MATMUL_GROUP_NUM
    st.w        t2, t0, 0x158
    slli.w      t3, t2, 7
    add.w       t3, t3, t1
    st.w        t3, t0, 0x154
    li.w        t2, 1
    st.w        t2, t0, 0

    jirl        zero, ra, 0
    .size matmul_early_start, .-matmul_early_start
