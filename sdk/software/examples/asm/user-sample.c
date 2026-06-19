#include "led.h"
#include "matmul.h"

#define EXTRAM_CACHED_BASE_ADDR   0x1c400000u
#define EXTRAM_UNCACHED_BASE_ADDR 0xbc400000u
#define AB_WORDS_PER_GROUP        32u
#define C_WORDS_PER_GROUP         48u
#define UART_REG_RBR_THR          0x0u
#define UART_REG_LSR              0x5u
#define UART_LSR_THRE             0x20u

#ifndef MATMUL_GROUP_NUM
#define MATMUL_GROUP_NUM 50
#endif

#if MATMUL_GROUP_NUM <= 0
#error "MATMUL_GROUP_NUM must be positive"
#endif

#define GROUP_COUNT ((U32)MATMUL_GROUP_NUM)

unsigned long UART_BASE = 0xbf000000UL;

static volatile U32 *const extram_src_words = (volatile U32 *)EXTRAM_CACHED_BASE_ADDR;
static volatile U32 *const extram_dst_words = (volatile U32 *)EXTRAM_UNCACHED_BASE_ADDR;

static volatile unsigned char *uart_reg(U32 offset)
{
    return (volatile unsigned char *)(UART_BASE + (unsigned long)offset);
}

static int uart_tx_ready(void)
{
    return ((*uart_reg(UART_REG_LSR)) & UART_LSR_THRE) ? 1 : 0;
}

static void uart_putchar_blocking(char ch)
{
    while (!uart_tx_ready()) {
    }
    *uart_reg(UART_REG_RBR_THR) = (unsigned char)ch;
}

static void uart_puts_blocking(const char *str)
{
    if (str == 0) {
        return;
    }

    while (*str != '\0') {
        if (*str == '\n') {
            uart_putchar_blocking('\r');
        }
        uart_putchar_blocking(*str);
        str++;
    }
}

static void uart_put_hex8(U32 value)
{
    int shift;

    for (shift = 28; shift >= 0; shift -= 4) {
        U32 digit = (value >> (U32)shift) & 0xfu;
        uart_putchar_blocking((char)(digit < 10u ? ('0' + digit) : ('a' + (digit - 10u))));
    }
}

static void uart_put_u32(U32 value)
{
    char buf[10];
    int idx = 0;

    if (value == 0u) {
        uart_putchar_blocking('0');
        return;
    }

    while ((value != 0u) && (idx < (int)sizeof(buf))) {
        buf[idx++] = (char)('0' + (value % 10u));
        value /= 10u;
    }

    while (idx > 0) {
        uart_putchar_blocking(buf[--idx]);
    }
}

static U32 crc32_update_byte(U32 crc, U8 byte)
{
    U32 bit;

    crc ^= (U32)byte;
    for (bit = 0u; bit < 8u; ++bit) {
        if ((crc & 1u) != 0u) {
            crc = (crc >> 1) ^ 0xedb88320u;
        } else {
            crc >>= 1;
        }
    }

    return crc;
}

static U32 compute_result_crc32(U32 dst_base_word)
{
    U32 crc = 0xffffffffu;
    U32 word_index;

    for (word_index = 0u; word_index < GROUP_COUNT * C_WORDS_PER_GROUP; ++word_index) {
        U32 value = extram_dst_words[dst_base_word + word_index];
        crc = crc32_update_byte(crc, (U8)(value & 0xffu));
        crc = crc32_update_byte(crc, (U8)((value >> 8) & 0xffu));
        crc = crc32_update_byte(crc, (U8)((value >> 16) & 0xffu));
        crc = crc32_update_byte(crc, (U8)((value >> 24) & 0xffu));
    }

    return crc ^ 0xffffffffu;
}

static void commit_result_region(U32 dst_base_word)
{
    volatile U32 tail_word;
    U32 last_word_index = dst_base_word + GROUP_COUNT * C_WORDS_PER_GROUP - 1u;

    /* Force all prior uncached stores toward ExtRAM before emitting the
     * completion UART message that the testbench watches for. The readback
     * gives us a concrete completion point on the same slave path. */
    __asm__ volatile("" : : : "memory");
    __dbar(0);
    tail_word = extram_dst_words[last_word_index];
    __dbar(0);
    __asm__ volatile("" : : : "memory");
    (void)tail_word;
}

static void fail(U32 code, U32 detail)
{
    setLedPin((code & 0xffffu) | 0x8000u);
    uart_puts_blocking("MATMUL_FAIL code=");
    uart_put_u32(code);
    uart_puts_blocking(" detail=0x");
    uart_put_hex8(detail);
    uart_puts_blocking("\r\n");
    while (1) {
    }
}

int main(void)
{
    U32 group;
    U32 word;
    U32 src_base_word = 0u;
    U32 dst_base_word = GROUP_COUNT * AB_WORDS_PER_GROUP;
    U32 crc32;
    U32 status;

    setLedPin(0x0001u);
    uart_puts_blocking("MATMUL_START\r\n");

    matmul_soft_reset();

    for (group = 0; group < GROUP_COUNT; ++group) {
        U32 src_group_base = src_base_word + group * AB_WORDS_PER_GROUP;
        U32 dst_group_base = dst_base_word + group * C_WORDS_PER_GROUP;

        for (word = 0; word < 16u; ++word) {
            matmul_load_a_word(word, extram_src_words[src_group_base + word]);
            matmul_load_b_word(word, extram_src_words[src_group_base + 16u + word]);
        }

        matmul_start();
        status = matmul_wait_done();
        if ((status & MATMUL_STATUS_ERROR) != 0u) {
            fail(1u, status);
        }

        /* Keep source reads cached, but write results through the uncached
         * DMW alias so each store reaches ExtRAM directly instead of relying
         * on a write-back cache line flush. */
        for (word = 0; word < C_WORDS_PER_GROUP; ++word) {
            extram_dst_words[dst_group_base + word] = matmul_read_c_word(word);
        }

        setLedPin((U32)(1u << (group & 0xf)));
    }

    commit_result_region(dst_base_word);
    crc32 = compute_result_crc32(dst_base_word);
    uart_puts_blocking("MATMUL_CRC32=");
    uart_put_hex8(crc32);
    uart_puts_blocking("\r\n");
    uart_puts_blocking("MATMUL_DONE\r\n");
    setLedPin(0x00ffu);

    while (1) {
    }
}
