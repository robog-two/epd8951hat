 


#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

#define GFP_KERNEL 0

/* Allocation instrumentation for the userspace benchmark/tests. Weak globals so
 * every translation unit that includes this header shares one counter without a
 * multiple-definition error; they stay zero unless the benchmark inspects them. */
unsigned long epd_alloc_calls __attribute__((weak));
unsigned long epd_alloc_bytes __attribute__((weak));

static inline void *epd_counted_calloc(size_t n, size_t sz)
{
	epd_alloc_calls += 1;
	epd_alloc_bytes += (unsigned long)n * sz;
	return calloc(n, sz);
}

#define kcalloc(n, sz, flags) epd_counted_calloc((n), (sz))
#define kfree(p)              free(p)

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define clamp(v, lo, hi) (min(max((v), (lo)), (hi)))

static inline u8 bitrev8(u8 byte)
{
	byte = (u8)(((byte & 0xF0u) >> 4) | ((byte & 0x0Fu) << 4));
	byte = (u8)(((byte & 0xCCu) >> 2) | ((byte & 0x33u) << 2));
	byte = (u8)(((byte & 0xAAu) >> 1) | ((byte & 0x55u) << 1));
	return byte;
}



#ifndef __APPLE__
static inline char *strnstr(const char *haystack, const char *needle, size_t n)
{
	size_t needle_len;

	if (!needle || !*needle)
		return (char *)haystack;

	needle_len = strlen(needle);
	if (n < needle_len)
		return NULL;

	for (size_t i = 0; i <= n - needle_len; i++) {
		if (memcmp(haystack + i, needle, needle_len) == 0)
			return (char *)(haystack + i);
	}
	return NULL;
}
#endif
