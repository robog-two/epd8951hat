 



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






void epd_dither_xrgb8888_fn(u16 w, u16 h, u32 stride,
			      const u8 *src, u32 src_pitch,
			      u8 *mono_buf);



void epd_compute_dirty_rect(u16 h, u32 stride, bool mirror_x,
			      const u8 *mono_buf, u8 *flip_buf,
			      int *y0_out, int *y1_out,
			      int *b0_out, int *b1_out);



void epd_align_dirty_bytes(u32 stride, bool needs_4byte_align,
			    int *b0, int *b1);



enum epd_lut_variant epd_lut_classify(const char *lut_str, size_t lut_bytes,
				       u8 *a2_mode, bool *needs_4byte_align);

#endif  
