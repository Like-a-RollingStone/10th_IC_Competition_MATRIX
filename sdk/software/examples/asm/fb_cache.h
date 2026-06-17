#ifndef FB_CACHE_H
#define FB_CACHE_H

#include "common_func.h"

void fb_cache_flush_range(U32 base, U32 bytes);
void fb_cache_flush_frame(U32 fb_base, U16 width, U16 height, U32 stride_bytes);

#endif /* FB_CACHE_H */
