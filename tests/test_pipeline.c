



#include "../epd8951hat_pipeline.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>




static int g_passed;
static int g_failed;
static int g_test_ok;   

#define CHECK(cond) do { \
	if (!(cond)) { \
		printf("    FAIL %s:%d: " #cond "\n", __FILE__, __LINE__); \
		g_test_ok = 0; \
	} \
} while (0)

#define CHECK_EQ(a, b) do { \
	if ((a) != (b)) { \
		printf("    FAIL %s:%d: " #a " == " #b " (%lld != %lld)\n", \
		       __FILE__, __LINE__, (long long)(a), (long long)(b)); \
		g_test_ok = 0; \
	} \
} while (0)

static void run_test(const char *name, void (*fn)(void))
{
	printf("  %-55s", name);
	fflush(stdout);
	g_test_ok = 1;
	fn();
	if (g_test_ok) {
		printf("OK\n");
		g_passed++;
	} else {
		printf("FAIL\n");
		g_failed++;
	}
}

#define RUN(fn) run_test(#fn, fn)




static void px(u8 *buf, u8 r, u8 g, u8 b)
{
	buf[0] = b;
	buf[1] = g;
	buf[2] = r;
	buf[3] = 0;
}




static void test_dither_white_image(void)
{
	 
	const u16 w = 4, h = 4;
	const u32 stride = 1;               
	u8 src[4 * 4 * 4];                 
	u8 mono[4 * 1];                    
	int i;

	for (i = 0; i < 4 * 4; i++)
		px(src + i * 4, 255, 255, 255);

	memset(mono, 0xAA, sizeof(mono));   
	epd_dither_xrgb8888_fn(w, h, stride, src, w * 4, mono);

	for (i = 0; i < (int)(stride * h); i++)
		CHECK_EQ(mono[i], 0x00);
}

static void test_dither_black_image(void)
{
	 
	const u16 w = 8, h = 2;
	const u32 stride = 1;
	u8 src[8 * 2 * 4];
	u8 mono[1 * 2];
	int i;

	for (i = 0; i < 8 * 2; i++)
		px(src + i * 4, 0, 0, 0);

	epd_dither_xrgb8888_fn(w, h, stride, src, w * 4, mono);

	for (i = 0; i < (int)(stride * h); i++)
		CHECK_EQ(mono[i], 0xFF);
}

static void test_dither_bit_order(void)
{
	

	const u16 w = 16, h = 1;
	const u32 stride = 2;               
	u8 src[16 * 4];
	u8 mono[2];
	int i;

	for (i = 0; i < 16; i++)
		px(src + i * 4, 255, 255, 255);   
	px(src + 0 * 4, 0, 0, 0);             

	epd_dither_xrgb8888_fn(w, h, stride, src, w * 4, mono);

	CHECK(mono[0] & 0x80);              
	CHECK_EQ(mono[1], 0x00);            
}

static void test_dither_bit_order_last_col(void)
{
	

	const u16 w = 8, h = 1;
	const u32 stride = 1;
	u8 src[8 * 4];
	u8 mono[1];
	int i;

	for (i = 0; i < 8; i++)
		px(src + i * 4, 255, 255, 255);
	px(src + 7 * 4, 0, 0, 0);

	epd_dither_xrgb8888_fn(w, h, stride, src, w * 4, mono);

	CHECK(mono[0] & 0x01);              
}

static void test_dither_50pct_gray_two_pixels(void)
{
	

	const u16 w = 2, h = 1;
	const u32 stride = 1;
	u8 src[2 * 4];
	u8 mono[1];

	px(src + 0, 127, 127, 127);
	px(src + 4, 127, 127, 127);

	epd_dither_xrgb8888_fn(w, h, stride, src, w * 4, mono);

	CHECK(mono[0] & 0x80);              
	CHECK(!(mono[0] & 0x40));           
}

static void test_dither_luma_weights(void)
{
	

	const u16 w = 3, h = 1;
	const u32 stride = 1;
	u8 src[3 * 4];
	u8 mono[1];

	px(src + 0 * 4, 255, 0, 0);    
	px(src + 1 * 4, 0, 255, 0);    
	px(src + 2 * 4, 0, 0, 255);    

	epd_dither_xrgb8888_fn(w, h, stride, src, w * 4, mono);

	CHECK(mono[0] & 0x80);              
	CHECK(!(mono[0] & 0x40));           
	CHECK(mono[0] & 0x20);              
}

static void test_dither_second_row(void)
{
	

	const u16 w = 8, h = 2;
	const u32 stride = 1;
	u8 src[8 * 2 * 4];
	u8 mono[2];
	int i;

	for (i = 0; i < 8 * 2; i++)
		px(src + i * 4, 255, 255, 255);    
	px(src + (8 + 0) * 4, 0, 0, 0);        

	epd_dither_xrgb8888_fn(w, h, stride, src, w * 4, mono);

	CHECK_EQ(mono[0], 0x00);    
	CHECK(mono[1] & 0x80);      
}

static void test_dither_no_spillover_into_next_row(void)
{
	

	const u16 w = 3, h = 1;
	const u32 stride = 1;
	u8 src[3 * 4];
	u8 mono[1];

	px(src + 0 * 4, 0, 0, 0);
	px(src + 1 * 4, 0, 0, 0);
	px(src + 2 * 4, 0, 0, 0);

	epd_dither_xrgb8888_fn(w, h, stride, src, w * 4, mono);

	 
	CHECK_EQ(mono[0] & 0x1F, 0x00);
}




static void test_dirty_identical_frames(void)
{
	 
	const u16 h = 4;
	const u32 stride = 2;
	u8 mono[4 * 2];
	u8 flip[4 * 2];
	int y0, y1, b0, b1;
	int i;

	for (i = 0; i < (int)(h * stride); i++)
		mono[i] = flip[i] = (u8)(i + 1);   

	epd_compute_dirty_rect(h, stride, false, mono, flip,
			       &y0, &y1, &b0, &b1);

	CHECK(y0 > y1);   
}

static void test_dirty_single_byte_top_left(void)
{
	 
	const u16 h = 4;
	const u32 stride = 2;
	u8 mono[4 * 2];
	u8 flip[4 * 2];
	int y0, y1, b0, b1;

	memset(mono, 0x00, sizeof(mono));
	memset(flip, 0x00, sizeof(flip));
	mono[0] = 0xFF;   

	epd_compute_dirty_rect(h, stride, false, mono, flip,
			       &y0, &y1, &b0, &b1);

	CHECK_EQ(y0, 0); CHECK_EQ(y1, 0);
	CHECK_EQ(b0, 0); CHECK_EQ(b1, 0);
}

static void test_dirty_single_byte_interior(void)
{
	 
	const u16 h = 8;
	const u32 stride = 5;
	u8 mono[8 * 5];
	u8 flip[8 * 5];
	int y0, y1, b0, b1;

	memset(mono, 0x00, sizeof(mono));
	memset(flip, 0x00, sizeof(flip));
	mono[5 * 5 + 3] = 0xAB;   

	epd_compute_dirty_rect(h, stride, false, mono, flip,
			       &y0, &y1, &b0, &b1);

	CHECK_EQ(y0, 5); CHECK_EQ(y1, 5);
	CHECK_EQ(b0, 3); CHECK_EQ(b1, 3);
}

static void test_dirty_multiple_rows(void)
{
	 
	const u16 h = 5;
	const u32 stride = 4;
	u8 mono[5 * 4];
	u8 flip[5 * 4];
	int y0, y1, b0, b1;

	memset(mono, 0x00, sizeof(mono));
	memset(flip, 0x00, sizeof(flip));
	mono[1 * 4 + 0] = 0x01;   
	mono[3 * 4 + 2] = 0x02;   

	epd_compute_dirty_rect(h, stride, false, mono, flip,
			       &y0, &y1, &b0, &b1);

	CHECK_EQ(y0, 1); CHECK_EQ(y1, 3);
	CHECK_EQ(b0, 0); CHECK_EQ(b1, 2);
}

static void test_dirty_flip_buf_updated(void)
{
	 
	const u16 h = 1;
	const u32 stride = 2;
	u8 mono[2] = { 0xAB, 0xCD };
	u8 flip[2] = { 0x00, 0x00 };
	int y0, y1, b0, b1;

	epd_compute_dirty_rect(h, stride, false, mono, flip,
			       &y0, &y1, &b0, &b1);

	CHECK_EQ(flip[0], 0xAB);
	CHECK_EQ(flip[1], 0xCD);
}

static void test_dirty_no_mirror(void)
{
	 
	const u16 h = 1;
	const u32 stride = 2;
	u8 mono[2] = { 0x12, 0x34 };
	u8 flip[2] = { 0x00, 0x00 };
	int y0, y1, b0, b1;

	epd_compute_dirty_rect(h, stride, false, mono, flip,
			       &y0, &y1, &b0, &b1);

	CHECK_EQ(flip[0], 0x12);
	CHECK_EQ(flip[1], 0x34);
}

static void test_dirty_mirror_x_single_byte(void)
{
	

	const u16 h = 1;
	const u32 stride = 1;
	u8 mono[1] = { 0xAB };
	u8 flip[1] = { 0x00 };
	int y0, y1, b0, b1;

	epd_compute_dirty_rect(h, stride, true, mono, flip,
			       &y0, &y1, &b0, &b1);

	CHECK_EQ(flip[0], bitrev8(0xAB));
	CHECK_EQ(y0, 0); CHECK_EQ(y1, 0);
	CHECK_EQ(b0, 0); CHECK_EQ(b1, 0);
}

static void test_dirty_mirror_x_two_bytes(void)
{
	

	const u16 h = 1;
	const u32 stride = 2;
	u8 mono[2] = { 0xAB, 0xCD };
	u8 flip[2] = { 0x00, 0x00 };
	int y0, y1, b0, b1;

	epd_compute_dirty_rect(h, stride, true, mono, flip,
			       &y0, &y1, &b0, &b1);

	CHECK_EQ(flip[0], bitrev8(0xCD));    
	CHECK_EQ(flip[1], bitrev8(0xAB));    
}

static void test_dirty_mirror_identical_after_reverse(void)
{
	

	const u16 h = 1;
	const u32 stride = 1;
	u8 mono[1] = { 0xAB };
	u8 flip[1] = { bitrev8(0xAB) };    
	int y0, y1, b0, b1;

	epd_compute_dirty_rect(h, stride, true, mono, flip,
			       &y0, &y1, &b0, &b1);

	CHECK(y0 > y1);   
}

static void test_dirty_only_changed_cols_in_bbox(void)
{
	

	const u16 h = 1;
	const u32 stride = 4;
	u8 mono[4] = { 0x00, 0x00, 0xFF, 0x00 };
	u8 flip[4] = { 0x00, 0x00, 0x00, 0x00 };
	int y0, y1, b0, b1;

	epd_compute_dirty_rect(h, stride, false, mono, flip,
			       &y0, &y1, &b0, &b1);

	CHECK_EQ(b0, 2); CHECK_EQ(b1, 2);
}




static void test_align_no_align_needed(void)
{
	int b0 = 5, b1 = 6;
	epd_align_dirty_bytes(8, false, &b0, &b1);
	CHECK_EQ(b0, 5);
	CHECK_EQ(b1, 6);
}

static void test_align_b0_rounds_down(void)
{
	 
	int b0 = 5, b1 = 6;
	epd_align_dirty_bytes(8, true, &b0, &b1);
	CHECK_EQ(b0, 4);
}

static void test_align_b0_already_aligned(void)
{
	int b0 = 4, b1 = 6;
	epd_align_dirty_bytes(8, true, &b0, &b1);
	CHECK_EQ(b0, 4);
}

static void test_align_b1_rounds_up(void)
{
	 
	int b0 = 4, b1 = 6;
	epd_align_dirty_bytes(8, true, &b0, &b1);
	CHECK_EQ(b1, 7);
}

static void test_align_b1_already_aligned(void)
{
	int b0 = 4, b1 = 7;
	epd_align_dirty_bytes(8, true, &b0, &b1);
	CHECK_EQ(b1, 7);
}

static void test_align_b1_clamped_to_stride_minus_1(void)
{
	 
	int b0 = 4, b1 = 5;
	epd_align_dirty_bytes(6, true, &b0, &b1);
	CHECK_EQ(b1, 5);
}

static void test_align_full_range_unchanged(void)
{
	 
	int b0 = 0, b1 = 7;
	epd_align_dirty_bytes(8, true, &b0, &b1);
	CHECK_EQ(b0, 0);
	CHECK_EQ(b1, 7);
}




static void test_lut_m641(void)
{
	u8 a2; bool align;
	enum epd_lut_variant v = epd_lut_classify("M641", 4, &a2, &align);
	CHECK_EQ(v, EPD_LUT_M641);
	CHECK_EQ(a2, EPD_MODE_A2_M641);
	CHECK(align);
}

static void test_lut_m841_plain(void)
{
	u8 a2; bool align;
	enum epd_lut_variant v = epd_lut_classify("M841", 4, &a2, &align);
	CHECK_EQ(v, EPD_LUT_M841);
	CHECK_EQ(a2, EPD_MODE_A2_M841);
	CHECK(!align);
}

static void test_lut_m841_tfa2812(void)
{
	u8 a2; bool align;
	enum epd_lut_variant v = epd_lut_classify("M841_TFA2812", 12, &a2, &align);
	CHECK_EQ(v, EPD_LUT_M841_TFA2812);
	CHECK_EQ(a2, EPD_MODE_A2_M841);
	CHECK(!align);
}

static void test_lut_m841_tfa5210(void)
{
	u8 a2; bool align;
	enum epd_lut_variant v = epd_lut_classify("M841_TFA5210", 12, &a2, &align);
	CHECK_EQ(v, EPD_LUT_M841_TFA5210);
	CHECK_EQ(a2, EPD_MODE_A2_M841);
	CHECK(!align);
}

static void test_lut_tfa2812_not_matched_as_plain_m841(void)
{
	

	u8 a2; bool align;
	enum epd_lut_variant v = epd_lut_classify("M841_TFA2812", 12, &a2, &align);
	CHECK(v != EPD_LUT_M841);
	CHECK_EQ(v, EPD_LUT_M841_TFA2812);
}

static void test_lut_unknown(void)
{
	u8 a2; bool align;
	enum epd_lut_variant v = epd_lut_classify("GD_TFA2612", 10, &a2, &align);
	CHECK_EQ(v, EPD_LUT_UNKNOWN);
	CHECK_EQ(a2, EPD_MODE_A2_M841);   
	CHECK(!align);
}

static void test_lut_empty_string(void)
{
	u8 a2; bool align;
	enum epd_lut_variant v = epd_lut_classify("", 0, &a2, &align);
	CHECK_EQ(v, EPD_LUT_UNKNOWN);
}

static void test_lut_embedded_in_padding(void)
{
	

	char lut[16];
	u8 a2; bool align;
	enum epd_lut_variant v;

	memset(lut, 0, sizeof(lut));
	memcpy(lut, "M841", 4);
	v = epd_lut_classify(lut, sizeof(lut), &a2, &align);
	CHECK_EQ(v, EPD_LUT_M841);
}

static void test_lut_m641_embedded_in_padding(void)
{
	char lut[16];
	u8 a2; bool align;
	enum epd_lut_variant v;

	memset(lut, 0, sizeof(lut));
	memcpy(lut, "M641", 4);
	v = epd_lut_classify(lut, sizeof(lut), &a2, &align);
	CHECK_EQ(v, EPD_LUT_M641);
	CHECK(align);
}




static void test_bitrev8_known_values(void)
{
	 
	CHECK_EQ(bitrev8(0xAB), 0xD5u);
	 
	CHECK_EQ(bitrev8(0x80), 0x01u);
	 
	CHECK_EQ(bitrev8(0x01), 0x80u);
	 
	CHECK_EQ(bitrev8(0xFF), 0xFFu);
	CHECK_EQ(bitrev8(0x00), 0x00u);
	 
	CHECK_EQ(bitrev8(0xF0), 0x0Fu);
}

static void test_bitrev8_involution(void)
{
	 
	for (int i = 0; i < 256; i++)
		CHECK_EQ(bitrev8(bitrev8((u8)i)), (u8)i);
}




int main(void)
{
	printf("epd8951hat pipeline unit tests\n");
	printf("================================\n");

	printf("\nbitrev8\n");
	RUN(test_bitrev8_known_values);
	RUN(test_bitrev8_involution);

	printf("\nDithering (epd_dither_xrgb8888_fn)\n");
	RUN(test_dither_white_image);
	RUN(test_dither_black_image);
	RUN(test_dither_bit_order);
	RUN(test_dither_bit_order_last_col);
	RUN(test_dither_50pct_gray_two_pixels);
	RUN(test_dither_luma_weights);
	RUN(test_dither_second_row);
	RUN(test_dither_no_spillover_into_next_row);

	printf("\nDirty-rect computation (epd_compute_dirty_rect)\n");
	RUN(test_dirty_identical_frames);
	RUN(test_dirty_single_byte_top_left);
	RUN(test_dirty_single_byte_interior);
	RUN(test_dirty_multiple_rows);
	RUN(test_dirty_flip_buf_updated);
	RUN(test_dirty_no_mirror);
	RUN(test_dirty_mirror_x_single_byte);
	RUN(test_dirty_mirror_x_two_bytes);
	RUN(test_dirty_mirror_identical_after_reverse);
	RUN(test_dirty_only_changed_cols_in_bbox);

	printf("\nDirty-byte alignment (epd_align_dirty_bytes)\n");
	RUN(test_align_no_align_needed);
	RUN(test_align_b0_rounds_down);
	RUN(test_align_b0_already_aligned);
	RUN(test_align_b1_rounds_up);
	RUN(test_align_b1_already_aligned);
	RUN(test_align_b1_clamped_to_stride_minus_1);
	RUN(test_align_full_range_unchanged);

	printf("\nLUT classification (epd_lut_classify)\n");
	RUN(test_lut_m641);
	RUN(test_lut_m841_plain);
	RUN(test_lut_m841_tfa2812);
	RUN(test_lut_m841_tfa5210);
	RUN(test_lut_tfa2812_not_matched_as_plain_m841);
	RUN(test_lut_unknown);
	RUN(test_lut_empty_string);
	RUN(test_lut_embedded_in_padding);
	RUN(test_lut_m641_embedded_in_padding);

	printf("\n================================\n");
	printf("Results: %d passed, %d failed\n", g_passed, g_failed);
	return g_failed ? 1 : 0;
}
