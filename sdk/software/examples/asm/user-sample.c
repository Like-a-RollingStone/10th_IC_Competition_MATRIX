#include <stdio.h>

#include "led.h"
#include "matmul.h"

#define EXTRAM_CACHED_BASE_ADDR   0x1c400000u
#define EXTRAM_UNCACHED_BASE_ADDR 0xbc400000u
#define AB_WORDS_PER_GROUP        32u
#define C_WORDS_PER_GROUP         48u

#ifndef MATMUL_GROUP_NUM
#define MATMUL_GROUP_NUM 50
#endif

#if MATMUL_GROUP_NUM <= 0
#error "MATMUL_GROUP_NUM must be positive"
#endif

#define GROUP_COUNT ((U32)MATMUL_GROUP_NUM)

static volatile U32 *const extram_src_words = (volatile U32 *)EXTRAM_CACHED_BASE_ADDR;
static volatile U32 *const extram_dst_words = (volatile U32 *)EXTRAM_UNCACHED_BASE_ADDR;

static void fail(U32 code, U32 detail)
{
    setLedPin((code & 0xffffu) | 0x8000u);
    printf("MATMUL_FAIL code=%u detail=0x%08x\r\n", code, detail);
    while (1) {
    }
}

int main(void)
{
    U32 group;
    U32 word;
    U32 src_base_word = 0u;
    U32 dst_base_word = GROUP_COUNT * AB_WORDS_PER_GROUP;
    U32 status;

    setvbuf(stdout, 0, _IONBF, 0);
    setLedPin(0x0001u);
    printf("MATMUL_START\r\n");

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

    printf("MATMUL_DONE\r\n");
    setLedPin(0x00ffu);

    while (1) {
    }
}
