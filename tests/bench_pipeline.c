/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Userspace micro-benchmark for the epd8951hat display pipeline hot path.
 *
 * Measures the per-frame cost of the dither + dirty-rect transforms on a
 * full IT8951 panel (1872x1404), reporting wall-clock time and the number /
 * volume of heap allocations made through the kernel kcalloc shim.
 */
#define _POSIX_C_SOURCE 199309L

#include "../epd8951hat_pipeline.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

extern unsigned long epd_alloc_calls;
extern unsigned long epd_alloc_bytes;

#define W       1872
#define H       1404
#define STRIDE  (W / 8)
#define ITERS   60

static double now_sec(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

int main(void)
{
	size_t src_bytes = (size_t)W * H * 4;
	u8 *src   = malloc(src_bytes);
	u8 *mono  = malloc((size_t)STRIDE * H);
	u8 *flip  = calloc((size_t)STRIDE * H, 1);
	int y0, y1, b0, b1;
	double t0, t1;
	int it;

	if (!src || !mono || !flip) {
		fprintf(stderr, "alloc failed\n");
		return 2;
	}

	/* A non-trivial gradient+noise pattern exercises the error diffusion and
	 * makes most bytes differ between frames so the dirty scan does real work. */
	for (size_t i = 0; i < (size_t)W * H; i++) {
		u8 g = (u8)((i * 37u + (i / W) * 13u) & 0xff);
		src[i * 4 + 0] = g;
		src[i * 4 + 1] = (u8)(g ^ 0x55);
		src[i * 4 + 2] = (u8)(255 - g);
		src[i * 4 + 3] = 0;
	}

	unsigned long a0 = epd_alloc_calls, b0_bytes = epd_alloc_bytes;
	t0 = now_sec();
	for (it = 0; it < ITERS; it++) {
		/* Perturb a column each frame so the dither output actually changes. */
		src[(it % W) * 4] ^= 0xff;
		epd_dither_xrgb8888_fn(W, H, STRIDE, src, W * 4, mono, 0, H - 1);
		epd_compute_dirty_rect(H, STRIDE, false, mono, flip,
				       0, STRIDE - 1, 0, H - 1,
				       &y0, &y1, &b0, &b1);
	}
	t1 = now_sec();

	double ms = (t1 - t0) * 1000.0 / ITERS;
	printf("frames=%d  per-frame=%.3f ms  total=%.1f ms\n",
	       ITERS, ms, (t1 - t0) * 1000.0);
	printf("kcalloc: calls=%lu  bytes=%lu  (per frame: %.1f calls, %lu bytes)\n",
	       epd_alloc_calls - a0, epd_alloc_bytes - b0_bytes,
	       (double)(epd_alloc_calls - a0) / ITERS,
	       (epd_alloc_bytes - b0_bytes) / ITERS);

	free(src);
	free(mono);
	free(flip);
	return 0;
}
