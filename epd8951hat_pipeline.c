



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
static void apply_threshold_dithering(u16 w, u16 h, u32 stride,
				      const u8 *src, u32 src_pitch,
				      u8 *mono_buf)
{
	for (int y = 0; y < (int)h; y++) {
		const u8 *row = src + (size_t)y * src_pitch;
		for (int x = 0; x < (int)w; x++) {
			int g = rgb8888_to_grayscale(&row[x * 4]);
			if (g < 128)
				mono_buf[(size_t)y * stride + x / 8] |= (u8)(0x80u >> (x & 7));
		}
	}
}

/* Floyd-Steinberg dithering implementation */
static void apply_floyd_steinberg(u16 w, u16 h, u32 stride,
				  const u8 *src, u32 src_pitch,
				  u8 *mono_buf, int *err_cur, int *err_nxt)
{
	for (int y = 0; y < (int)h; y++) {
		const u8 *row = src + (size_t)y * src_pitch;
		memset(err_nxt, 0, (w + 2) * sizeof(int));

		for (int x = 0; x < (int)w; x++) {
			int gray = rgb8888_to_grayscale(&row[x * 4]);
			int val  = clamp(gray + err_cur[x + 1] / 16, 0, 255);
			int new_val, err;

			if (val < 128) {
				new_val = 0;
				mono_buf[(size_t)y * stride + x / 8] |= (u8)(0x80u >> (x & 7));
			} else {
				new_val = 255;
			}

			err = val - new_val;
			
			err_cur[x + 2]        += err * 7;
			if (x > 0) err_nxt[x] += err * 3;
			err_nxt[x + 1]        += err * 5;
			err_nxt[x + 2]        += err;
		}

		int *tmp = err_cur;
		err_cur = err_nxt;
		err_nxt = tmp;
	}
}

void epd_dither_xrgb8888_fn(u16 w, u16 h, u32 stride,
			      const u8 *src, u32 src_pitch,
			      u8 *mono_buf)
{
	memset(mono_buf, 0, (size_t)stride * h);

	int *err_cur = kcalloc(w + 2, sizeof(int), GFP_KERNEL);
	int *err_nxt = kcalloc(w + 2, sizeof(int), GFP_KERNEL);

	if (!err_cur || !err_nxt) {
		apply_threshold_dithering(w, h, stride, src, src_pitch, mono_buf);
	} else {
		apply_floyd_steinberg(w, h, stride, src, src_pitch, mono_buf, err_cur, err_nxt);
	}

	kfree(err_cur);
	kfree(err_nxt);
}

/* Helper to safely retrieve a monochromatic byte with optional mirroring */
static inline u8 get_mono_byte(bool mirror_x, const u8 *mono_buf, u32 stride, int y, int b)
{
	if (mirror_x)
		return bitrev8(mono_buf[(size_t)y * stride + (stride - 1 - b)]);
	return mono_buf[(size_t)y * stride + b];
}

void epd_compute_dirty_rect(u16 h, u32 stride, bool mirror_x,
			      const u8 *mono_buf, u8 *flip_buf,
			      int *y0_out, int *y1_out,
			      int *b0_out, int *b1_out)
{
	int y0 = (int)h, y1 = -1;
	int b0 = (int)stride, b1 = -1;

	for (int y = 0; y < (int)h; y++) {
		for (int b = 0; b < (int)stride; b++) {
			u8 nb = get_mono_byte(mirror_x, mono_buf, stride, y, b);

			if (nb == flip_buf[(size_t)y * stride + b])
				continue;

			flip_buf[(size_t)y * stride + b] = nb;

			if (y < y0) y0 = y;
			if (y > y1) y1 = y;
			if (b < b0) b0 = b;
			if (b > b1) b1 = b;
		}
	}

	*y0_out = y0; *y1_out = y1;
	*b0_out = b0; *b1_out = b1;
}

void epd_align_dirty_bytes(u32 stride, bool needs_4byte_align,
			    int *b0, int *b1)
{
	if (needs_4byte_align) {
		*b0 &= ~3;
		*b1  = min(*b1 | 3, (int)stride - 1);
	} else {
		*b0 &= ~1;
		*b1  = min(*b1 | 1, (int)stride - 1);
	}
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
		*needs_4byte_align = false;
		return EPD_LUT_M841_TFA2812;
	}
	if (strnstr(lut_str, "M841_TFA5210", lut_bytes)) {
		*a2_mode           = EPD_MODE_A2_M841;
		*needs_4byte_align = false;
		return EPD_LUT_M841_TFA5210;
	}
	if (strnstr(lut_str, "M841", lut_bytes)) {
		*a2_mode           = EPD_MODE_A2_M841;
		*needs_4byte_align = false;
		return EPD_LUT_M841;
	}

	*a2_mode           = EPD_MODE_A2_M841;
	*needs_4byte_align = false;
	return EPD_LUT_UNKNOWN;
}
