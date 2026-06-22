/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Kernel-to-userspace compatibility shim for unit-testing epd8951hat pipeline
 * functions without the kernel build environment.
 *
 * Included by epd8951hat_pipeline.h when TESTING_BUILD is defined.
 */
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

#define kcalloc(n, sz, flags) calloc((n), (sz))
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

/*
 * strnstr: find @needle in the first @n bytes of @haystack.
 * Returns pointer to first occurrence, or NULL.
 */
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
