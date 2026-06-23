 



#ifndef EPD8951HAT_PIPELINE_H
#define EPD8951HAT_PIPELINE_H

#ifndef TESTING_BUILD
#  include <linux/types.h>    
#  include <linux/bitrev.h>   
#else
#  include <compat.h>
#  include <stddef.h>         
#endif




#define EPD_MODE_A2_M641    4u
#define EPD_MODE_A2_M841    6u




enum epd_lut_variant {
	EPD_LUT_UNKNOWN,
	EPD_LUT_M641,
	EPD_LUT_M841,
	EPD_LUT_M841_TFA2812,
	EPD_LUT_M841_TFA5210,
};






/* clip_y0/clip_y1 limit processing to a horizontal band of rows (inclusive).
 * Pass 0 / h-1 for a full-frame dither. Only the clipped rows are zeroed and
 * re-dithered; rows outside the band keep their previous mono_buf content. */
void epd_dither_xrgb8888_fn(u16 w, u16 h, u32 stride,
			      const u8 *src, u32 src_pitch,
			      u8 *mono_buf,
			      int clip_y0, int clip_y1);



/* clip_y0/clip_y1 restrict the diff scan to the row band used for dithering.
 * b_clip0/b_clip1 restrict it to a byte-column range (inclusive) derived from
 * the DRM damage x bounds, so only the columns that actually changed are
 * considered. Pass 0 / stride-1 to scan the full width. */
void epd_compute_dirty_rect(u16 h, u32 stride, bool mirror_x,
			      const u8 *mono_buf, u8 *flip_buf,
			      int b_clip0, int b_clip1,
			      int clip_y0, int clip_y1,
			      int *y0_out, int *y1_out,
			      int *b0_out, int *b1_out);



void epd_align_dirty_bytes(u32 stride, bool needs_4byte_align,
			    int *b0, int *b1);



enum epd_lut_variant epd_lut_classify(const char *lut_str, size_t lut_bytes,
				       u8 *a2_mode, bool *needs_4byte_align);

#endif  
