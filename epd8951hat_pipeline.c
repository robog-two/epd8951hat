



#ifndef TESTING_BUILD
#  include <linux/bitrev.h>
#  include <linux/kernel.h>
#  include <linux/slab.h>
#  include <linux/string.h>
#endif

#include "epd8951hat_pipeline.h"

/* Helper to convert xRGB8888 pixel to grayscale using perceptual weights */
static inline int rgb8888_to_grayscale(const u8 *px)
{
	return (77 * px[2] + 150 * px[1] + 29 * px[0]) >> 8;
}

/* Fallback dithering using simple thresholding */
static void apply_threshold_dithering(u16 w, u32 stride,
				      const u8 *src, u32 src_pitch,
				      u8 *mono_buf, int y_start, int y_end)
{
	for (int y = y_start; y <= y_end; y++) {
		const u8 *row = src + (size_t)y * src_pitch;
		for (int x = 0; x < (int)w; x++) {
			int g = rgb8888_to_grayscale(&row[x * 4]);
			/* bit set = white, bit clear = black (IT8951 1bpp convention) */
			if (g >= 128)
				mono_buf[(size_t)y * stride + x / 8] |= (u8)(1u << (x & 7));
		}
	}
}

/* Floyd-Steinberg dithering.
 *
 * Instead of the classic two-row ping-pong (err_cur/err_nxt) this keeps a
 * single error buffer of w elements that is rewritten in place as each row is
 * consumed. err[x] holds, in unnormalised 1/16-pixel units, the diffused error
 * destined for column x of the row currently being processed. The rightward
 * (7/16) error stays within the row in the scalar `carry`, and the down-right
 * (1/16) error is deferred one column in the scalar `dr`; both are off the
 * panel at the row edges, exactly matching the old buffer's dropped indices.
 *
 * Because err[x] is reassigned (not accumulated) before it is reused, the per-
 * row memset of the original is unnecessary, and the output is bit-for-bit
 * identical to the two-buffer version. Pixels are packed into an 8-bit
 * accumulator and flushed once per byte rather than read-modify-written
 * per pixel. `err` must hold at least w ints, zeroed by the caller. */
static void apply_floyd_steinberg(u16 w, u32 stride,
				  const u8 *src, u32 src_pitch,
				  u8 *mono_buf, int *err,
				  int y_start, int y_end)
{
	for (int y = y_start; y <= y_end; y++) {
		const u8 *row = src + (size_t)y * src_pitch;
		u8 *mrow = mono_buf + (size_t)y * stride;
		int carry = 0;   /* rightward 7/16 error within the row */
		int dr    = 0;   /* down-right 1/16 error deferred one column */
		u8 bits   = 0;   /* LSB-first packed output, flushed every 8 px */

		for (int x = 0; x < (int)w; x++) {
			int gray = rgb8888_to_grayscale(&row[x * 4]);
			int val  = clamp(gray + (err[x] + carry) / 16, 0, 255);
			int new_val, e;

			/* bit set = white, bit clear = black (IT8951 1bpp convention) */
			if (val >= 128) {
				new_val = 255;
				bits |= (u8)(1u << (x & 7));
			} else {
				new_val = 0;
			}

			e = val - new_val;

			/* Rebuild err[] for the next row in place: column x is now
			 * consumed, so store its down (5/16) error plus the down-right
			 * (1/16) carried from column x-1; the down-left (3/16) goes to
			 * the already-consumed column x-1. */
			err[x] = e * 5 + dr;
			if (x > 0)
				err[x - 1] += e * 3;
			dr    = e;
			carry = e * 7;

			if ((x & 7) == 7) {
				mrow[x >> 3] = bits;
				bits = 0;
			}
		}

		/* Flush the trailing partial byte when w is not a multiple of 8. */
		if (w & 7)
			mrow[(w - 1) >> 3] = bits;
	}
}

void epd_dither_xrgb8888_fn(u16 w, u16 h, u32 stride,
			      const u8 *src, u32 src_pitch,
			      u8 *mono_buf,
			      int clip_y0, int clip_y1)
{
	int y_start = max(clip_y0, 0);
	int y_end   = min(clip_y1, (int)h - 1);

	if (y_start > y_end)
		return;

	/* One zeroed error row replaces the old two-buffer ping-pong. The fast
	 * path writes every output byte itself, so the band no longer needs to be
	 * pre-zeroed; only the threshold fallback (which ORs bits in) does. */
	int *err = kcalloc(w, sizeof(int), GFP_KERNEL);

	if (!err) {
		memset(mono_buf + (size_t)y_start * stride, 0,
		       (size_t)(y_end - y_start + 1) * stride);
		apply_threshold_dithering(w, stride, src, src_pitch, mono_buf,
					  y_start, y_end);
		return;
	}

	apply_floyd_steinberg(w, stride, src, src_pitch, mono_buf,
			      err, y_start, y_end);

	kfree(err);
}

void epd_compute_dirty_rect(u16 h, u32 stride, bool mirror_x,
			      const u8 *mono_buf, u8 *flip_buf,
			      int b_clip0, int b_clip1,
			      int clip_y0, int clip_y1,
			      int *y0_out, int *y1_out,
			      int *b0_out, int *b1_out)
{
	int y0 = (int)h, y1 = -1;
	int b0 = (int)stride, b1 = -1;
	int y_start = max(clip_y0, 0);
	int y_end   = min(clip_y1, (int)h - 1);
	int b_start = max(b_clip0, 0);
	int b_end   = min(b_clip1, (int)stride - 1);

	/* Hoist the mirror_x decision out of the per-byte inner loop: it is
	 * constant for the whole scan, so branching on it once per row keeps the
	 * hot loop straight-line. Row base pointers are computed once per row
	 * rather than re-deriving y * stride for every byte. */
	for (int y = y_start; y <= y_end; y++) {
		const u8 *mrow = mono_buf + (size_t)y * stride;
		u8 *frow       = flip_buf + (size_t)y * stride;

		if (mirror_x) {
			for (int b = b_start; b <= b_end; b++) {
				u8 nb = bitrev8(mrow[stride - 1 - b]);

				if (nb == frow[b])
					continue;

				frow[b] = nb;

				if (y < y0) y0 = y;
				if (y > y1) y1 = y;
				if (b < b0) b0 = b;
				if (b > b1) b1 = b;
			}
		} else {
			for (int b = b_start; b <= b_end; b++) {
				u8 nb = mrow[b];

				if (nb == frow[b])
					continue;

				frow[b] = nb;

				if (y < y0) y0 = y;
				if (y > y1) y1 = y;
				if (b < b0) b0 = b;
				if (b > b1) b1 = b;
			}
		}
	}

	*y0_out = y0; *y1_out = y1;
	*b0_out = b0; *b1_out = b1;
}

void epd_align_dirty_bytes(u32 stride, bool needs_4byte_align,
			    int *b0, int *b1)
{
	/* The IT8951 always requires x and width to be 4-byte (32-pixel) aligned
	 * in 1bpp packed mode, regardless of LUT variant. needs_4byte_align is
	 * kept for API compatibility but is no longer used as a branch. */
	(void)needs_4byte_align;
	*b0 &= ~3;
	*b1  = min(*b1 | 3, (int)stride - 1);
}

enum epd_lut_variant epd_lut_classify(const char *lut_str, size_t lut_bytes,
				       u8 *a2_mode, bool *needs_4byte_align)
{
	if (strnstr(lut_str, "M641", lut_bytes)) {
		*a2_mode           = EPD_MODE_A2_M641;
		*needs_4byte_align = true;
		return EPD_LUT_M641;
	}
	if (strnstr(lut_str, "M841_TFA2812", lut_bytes)) {
		*a2_mode           = EPD_MODE_A2_M841;
		*needs_4byte_align = true;
		return EPD_LUT_M841_TFA2812;
	}
	if (strnstr(lut_str, "M841_TFA5210", lut_bytes)) {
		*a2_mode           = EPD_MODE_A2_M841;
		*needs_4byte_align = true;
		return EPD_LUT_M841_TFA5210;
	}
	if (strnstr(lut_str, "M841", lut_bytes)) {
		*a2_mode           = EPD_MODE_A2_M841;
		*needs_4byte_align = true;
		return EPD_LUT_M841;
	}

	*a2_mode           = EPD_MODE_A2_M841;
	*needs_4byte_align = true;
	return EPD_LUT_UNKNOWN;
}
