/* SPDX-License-Identifier: GPL-2.0 */
/*
 * epd8951hat_pipeline.h – Pure data-transformation functions for the
 * e-paper display pipeline.
 *
 * These functions have no side-effects beyond their output arguments: no SPI,
 * no GPIO, no kernel allocations in the hot path.  They are compiled into
 * the kernel module normally, and can also be compiled in a userspace test
 * binary by defining TESTING_BUILD and providing tests/compat.h.
 */

#ifndef EPD8951HAT_PIPELINE_H
#define EPD8951HAT_PIPELINE_H

#ifndef TESTING_BUILD
#  include <linux/types.h>   /* u8, u16, u32, bool, size_t */
#  include <linux/bitrev.h>  /* bitrev8 */
#else
#  include <compat.h>
#  include <stddef.h>        /* size_t */
#endif

/* -------------------------------------------------------------------------
 * Display refresh mode codes (A2 variants depend on LUT)
 * ------------------------------------------------------------------------- */

#define EPD_MODE_A2_M641    4u
#define EPD_MODE_A2_M841    6u

/* -------------------------------------------------------------------------
 * LUT variant (detected from dev_info.lut_version at probe time)
 * ------------------------------------------------------------------------- */

enum epd_lut_variant {
	EPD_LUT_UNKNOWN,
	EPD_LUT_M641,
	EPD_LUT_M841,
	EPD_LUT_M841_TFA2812,
	EPD_LUT_M841_TFA5210,
};

/* =========================================================================
 * Pure pipeline functions
 * ========================================================================= */

/*
 * epd_dither_xrgb8888_fn – Floyd-Steinberg XRGB8888 → 1bpp dither.
 *
 * Converts an @w × @h XRGB8888 source image (@src, @src_pitch bytes/row)
 * into a packed 1bpp @mono_buf (@stride bytes/row).  @mono_buf is zeroed
 * on entry.  Bit convention: bit=1 means black (ink), matching MONO01 /
 * IT8951 BGVR 0x00F0.  Bit 7 of byte 0 is the leftmost pixel of row 0.
 *
 * DRM_FORMAT_XRGB8888 byte layout: byte[0]=B, [1]=G, [2]=R, [3]=X.
 */
void epd_dither_xrgb8888_fn(u16 w, u16 h, u32 stride,
			      const u8 *src, u32 src_pitch,
			      u8 *mono_buf);

/*
 * epd_compute_dirty_rect – Mirror mono_buf into flip_buf, find changed bytes.
 *
 * For each byte position (y, b):
 *   - if @mirror_x: read mono_buf[y][stride-1-b] and bitrev8() it
 *   - else: read mono_buf[y][b] directly
 *   Compare to flip_buf[y][b]; if different, update flip_buf and expand the
 *   dirty bounding box.
 *
 * Outputs the dirty bounding box in byte/row coordinates (inclusive).
 * If no bytes differ: *y0_out > *y1_out (caller checks this sentinel).
 */
void epd_compute_dirty_rect(u16 h, u32 stride, bool mirror_x,
			      const u8 *mono_buf, u8 *flip_buf,
			      int *y0_out, int *y1_out,
			      int *b0_out, int *b1_out);

/*
 * epd_align_dirty_bytes – Expand [*b0, *b1] to 4-byte boundaries.
 *
 * Some LUT variants (M641) require x and w to be multiples of 32 pixels
 * (4 bytes).  *b1 is clamped to @stride - 1 to avoid overrun.
 * No-op when @needs_4byte_align is false.
 */
void epd_align_dirty_bytes(u32 stride, bool needs_4byte_align,
			    int *b0, int *b1);

/*
 * epd_lut_classify – Identify the LUT variant from the device-info string.
 *
 * @lut_str / @lut_bytes: the raw lut_version field from struct it8951_dev_info
 *   cast to (const char *) with its byte length.
 * Fills @a2_mode and @needs_4byte_align, returns the variant enum.
 * Substrings are checked in priority order so "M841_TFA2812" never falls
 * through to the plain "M841" branch.
 */
enum epd_lut_variant epd_lut_classify(const char *lut_str, size_t lut_bytes,
				       u8 *a2_mode, bool *needs_4byte_align);

#endif /* EPD8951HAT_PIPELINE_H */
