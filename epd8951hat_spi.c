// SPDX-License-Identifier: GPL-2.0
/*
 * epd8951hat_spi.c  –  IT8951 SPI/GPIO protocol layer
 *
 * Implements all hardware communication for the Waveshare IT8951 e-Paper HAT.
 * The IT8951 uses a 16-bit SPI protocol with manual CS GPIO control and a
 * BUSY GPIO that must be polled before each SPI transfer.
 */

#include <asm/byteorder.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/spi/spi.h>
#include <linux/string.h>
#include <linux/types.h>

#include "epd8951hat.h"

/* =========================================================================
 * Low-level SPI helpers (static)
 * ========================================================================= */

/*
 * epd_wait_busy - Poll BUSY GPIO until HIGH (ready) or timeout.
 *
 * IT8951 signals BUSY HIGH when ready to accept another word.
 * Poll with a short sleep between attempts; fail after EPD_BUSY_TIMEOUT_MS.
 *
 * Returns 0 when ready, -ETIMEDOUT on timeout.
 */
static int epd_wait_busy(struct epd_device *epd)
{
	u64 deadline = ktime_get_ns() +
		       (u64)EPD_BUSY_TIMEOUT_MS * NSEC_PER_MSEC;

	while (!gpiod_get_value_cansleep(epd->gpio_busy)) {
		if (ktime_get_ns() >= deadline) {
			dev_err(&epd->spi->dev,
				"BUSY timeout after %u ms\n",
				EPD_BUSY_TIMEOUT_MS);
			return -ETIMEDOUT;
		}
		usleep_range(100, 200);
	}
	return 0;
}

/*
 * epd_spi_send_word - Write a single 16-bit word over SPI (MSB-first).
 *
 * Converts @word to big-endian and issues a 2-byte spi_write().
 * CS must already be asserted by the caller.
 */
static int epd_spi_send_word(struct epd_device *epd, u16 word)
{
	__be16 buf = cpu_to_be16(word);

	return spi_write(epd->spi, &buf, sizeof(buf));
}

/*
 * epd_write_cmd - Full IT8951 command write sequence.
 *
 * Sequence:
 *   wait_busy → CS LOW → send preamble 0x6000 → wait_busy → send cmd → CS HIGH
 */
static int epd_write_cmd(struct epd_device *epd, u16 cmd)
{
	int ret;

	ret = epd_wait_busy(epd);
	if (ret)
		return ret;

	gpiod_set_value_cansleep(epd->gpio_cs, 0);

	ret = epd_spi_send_word(epd, IT8951_PREAMBLE_CMD);
	if (ret) {
		dev_err(&epd->spi->dev, "SPI error sending CMD preamble: %d\n", ret);
		goto out_cs_high;
	}

	ret = epd_wait_busy(epd);
	if (ret)
		goto out_cs_high;

	ret = epd_spi_send_word(epd, cmd);
	if (ret)
		dev_err(&epd->spi->dev, "SPI error sending CMD 0x%04x: %d\n", cmd, ret);

out_cs_high:
	gpiod_set_value_cansleep(epd->gpio_cs, 1);
	return ret;
}

/*
 * epd_write_data - Write a single 16-bit data word to IT8951.
 *
 * Sequence:
 *   wait_busy → CS LOW → send preamble 0x0000 → wait_busy → send data → CS HIGH
 */
static int epd_write_data(struct epd_device *epd, u16 data)
{
	int ret;

	ret = epd_wait_busy(epd);
	if (ret)
		return ret;

	gpiod_set_value_cansleep(epd->gpio_cs, 0);

	ret = epd_spi_send_word(epd, IT8951_PREAMBLE_WRITE);
	if (ret) {
		dev_err(&epd->spi->dev, "SPI error sending DATA preamble: %d\n", ret);
		goto out_cs_high;
	}

	ret = epd_wait_busy(epd);
	if (ret)
		goto out_cs_high;

	ret = epd_spi_send_word(epd, data);
	if (ret)
		dev_err(&epd->spi->dev, "SPI error sending DATA 0x%04x: %d\n", data, ret);

out_cs_high:
	gpiod_set_value_cansleep(epd->gpio_cs, 1);
	return ret;
}

/*
 * epd_read_data - Read a single 16-bit word from IT8951.
 *
 * Sequence:
 *   wait_busy → CS LOW → send preamble 0x1000 → wait_busy →
 *   read 2 dummy bytes → wait_busy → read 2 actual bytes → CS HIGH
 *
 * Returns 0 on success with result in *out, negative errno on error.
 */
static int epd_read_data(struct epd_device *epd, u16 *out)
{
	u8 dummy[2];
	__be16 result;
	int ret;

	ret = epd_wait_busy(epd);
	if (ret)
		return ret;

	gpiod_set_value_cansleep(epd->gpio_cs, 0);

	ret = epd_spi_send_word(epd, IT8951_PREAMBLE_READ);
	if (ret) {
		dev_err(&epd->spi->dev,
			"SPI error sending READ preamble: %d\n", ret);
		goto out_cs_high;
	}

	ret = epd_wait_busy(epd);
	if (ret)
		goto out_cs_high;

	/* Discard 2 dummy bytes */
	ret = spi_read(epd->spi, dummy, sizeof(dummy));
	if (ret) {
		dev_err(&epd->spi->dev,
			"SPI error reading dummy bytes: %d\n", ret);
		goto out_cs_high;
	}

	ret = epd_wait_busy(epd);
	if (ret)
		goto out_cs_high;

	/* Read actual 16-bit result */
	ret = spi_read(epd->spi, &result, sizeof(result));
	if (ret) {
		dev_err(&epd->spi->dev,
			"SPI error reading data word: %d\n", ret);
		goto out_cs_high;
	}

	*out = be16_to_cpu(result);

out_cs_high:
	gpiod_set_value_cansleep(epd->gpio_cs, 1);
	return ret;
}

/* =========================================================================
 * Register access (static)
 * ========================================================================= */

/*
 * epd_write_reg - Write a 16-bit value to an IT8951 register.
 *
 * Sends: CMD_REG_WR → data(reg addr) → data(value)
 */
static int epd_write_reg(struct epd_device *epd, u16 reg, u16 val)
{
	int ret;

	ret = epd_write_cmd(epd, IT8951_CMD_REG_WR);
	if (ret)
		return ret;

	ret = epd_write_data(epd, reg);
	if (ret)
		return ret;

	return epd_write_data(epd, val);
}

/*
 * epd_read_reg - Read a 16-bit value from an IT8951 register.
 *
 * Sends: CMD_REG_RD → data(reg addr) → read result into *val
 */
static int epd_read_reg(struct epd_device *epd, u16 reg, u16 *val)
{
	int ret;

	ret = epd_write_cmd(epd, IT8951_CMD_REG_RD);
	if (ret)
		return ret;

	ret = epd_write_data(epd, reg);
	if (ret)
		return ret;

	return epd_read_data(epd, val);
}

/* =========================================================================
 * LUT variant detection (static helper)
 * ========================================================================= */

/*
 * epd_detect_lut - Parse dev_info.lut_version and populate variant fields.
 *
 * lut_version is a u16[8] array that holds a null-terminated ASCII string
 * in big-endian 16-bit encoding (each char in the high byte of its word).
 * We cast directly to char* for strncmp since the high bytes of each u16
 * correspond to the ASCII characters in the string on big-endian on-wire data
 * after the SPI read.  In practice Waveshare stores these as packed ASCII
 * byte pairs so a raw strncmp over the byte array works correctly.
 *
 * Priority: M841_TFA2812 and M841_TFA5210 must be checked before plain M841.
 */
static void epd_detect_lut(struct epd_device *epd)
{
	const char *lut = (const char *)epd->dev_info.lut_version;
	size_t lut_bytes = sizeof(epd->dev_info.lut_version);

	/*
	 * Search for substrings.  Use a helper lambda pattern via a local
	 * function-like macro since C99 lacks closures.
	 */
#define LUT_CONTAINS(s) (strnstr(lut, (s), lut_bytes) != NULL)

	if (LUT_CONTAINS("M641")) {
		epd->lut_variant      = EPD_LUT_M641;
		epd->a2_mode          = EPD_MODE_A2_M641;
		epd->needs_4byte_align = true;
		dev_dbg(&epd->spi->dev, "LUT variant: M641 (A2 mode %u, 4-byte align)\n",
			epd->a2_mode);
	} else if (LUT_CONTAINS("M841_TFA2812")) {
		epd->lut_variant      = EPD_LUT_M841_TFA2812;
		epd->a2_mode          = EPD_MODE_A2_M841;
		epd->needs_4byte_align = false;
		dev_dbg(&epd->spi->dev, "LUT variant: M841_TFA2812 (A2 mode %u)\n",
			epd->a2_mode);
	} else if (LUT_CONTAINS("M841_TFA5210")) {
		epd->lut_variant      = EPD_LUT_M841_TFA5210;
		epd->a2_mode          = EPD_MODE_A2_M841;
		epd->needs_4byte_align = false;
		dev_dbg(&epd->spi->dev, "LUT variant: M841_TFA5210 (A2 mode %u)\n",
			epd->a2_mode);
	} else if (LUT_CONTAINS("M841")) {
		epd->lut_variant      = EPD_LUT_M841;
		epd->a2_mode          = EPD_MODE_A2_M841;
		epd->needs_4byte_align = false;
		dev_dbg(&epd->spi->dev, "LUT variant: M841 (A2 mode %u)\n",
			epd->a2_mode);
	} else {
		epd->lut_variant      = EPD_LUT_UNKNOWN;
		epd->a2_mode          = EPD_MODE_A2_M841;  /* safe default */
		epd->needs_4byte_align = false;
		dev_dbg(&epd->spi->dev,
			"LUT variant: UNKNOWN, defaulting to A2 mode %u\n",
			epd->a2_mode);
	}

#undef LUT_CONTAINS
}

/* =========================================================================
 * Exported functions – hardware lifecycle
 * ========================================================================= */

/*
 * epd_hw_reset - Hard-reset the IT8951 via RST GPIO.
 *
 * Drives RST low for 10 ms, then high, then waits 200 ms for the
 * controller's internal boot sequence, then polls BUSY until ready.
 */
void epd_hw_reset(struct epd_device *epd)
{
	gpiod_set_value_cansleep(epd->gpio_rst, 0);
	msleep(10);
	gpiod_set_value_cansleep(epd->gpio_rst, 1);
	msleep(200);

	/* Best-effort poll; ignore timeout here — init will catch it */
	epd_wait_busy(epd);
}

/*
 * epd_hw_init - Full hardware initialisation sequence.
 *
 * Steps:
 *   1. Hard reset
 *   2. SYS_RUN (wake the controller)
 *   3. GET_DEV_INFO → populate dev_info, img_ram_addr, panel_w/h
 *   4. Enable packed pixel writes (I80CPCR = 1)
 *   5. Set VCOM if different from stored value
 *   6. Detect LUT variant → fill lut_variant, a2_mode, needs_4byte_align
 *
 * Returns 0 on success, negative errno on failure.
 */
int epd_hw_init(struct epd_device *epd)
{
	struct it8951_dev_info *di = &epd->dev_info;
	u16 cur_vcom = 0;
	size_t i;
	int ret;

	/* 1. Reset */
	epd_hw_reset(epd);

	/* 2. SYS_RUN */
	ret = epd_write_cmd(epd, IT8951_CMD_SYS_RUN);
	if (ret) {
		dev_err(&epd->spi->dev, "SYS_RUN failed: %d\n", ret);
		return ret;
	}

	/* 3. GET_DEV_INFO — reads back a struct it8951_dev_info worth of words */
	ret = epd_write_cmd(epd, IT8951_CMD_GET_DEV_INFO);
	if (ret) {
		dev_err(&epd->spi->dev, "GET_DEV_INFO cmd failed: %d\n", ret);
		return ret;
	}

	/*
	 * The device returns sizeof(struct it8951_dev_info) / 2 consecutive
	 * 16-bit words.  Read them one by one into the struct.
	 */
	{
		u16 *raw = (u16 *)di;
		size_t nwords = sizeof(*di) / sizeof(u16);

		for (i = 0; i < nwords; i++) {
			ret = epd_read_data(epd, &raw[i]);
			if (ret) {
				dev_err(&epd->spi->dev,
					"GET_DEV_INFO read[%zu] failed: %d\n",
					i, ret);
				return ret;
			}
		}
	}

	epd->panel_w      = di->panel_w;
	epd->panel_h      = di->panel_h;
	epd->img_ram_addr = ((u32)di->mem_addr_h << 16) | di->mem_addr_l;

	dev_dbg(&epd->spi->dev,
		"IT8951 panel %u×%u img_ram=0x%08x\n",
		epd->panel_w, epd->panel_h, epd->img_ram_addr);

	/* Sanity-check panel dimensions */
	if (!epd->panel_w || !epd->panel_h ||
	    epd->panel_w > EPD_MAX_WIDTH || epd->panel_h > EPD_MAX_HEIGHT) {
		dev_err(&epd->spi->dev,
			"Implausible panel size %u×%u (max %u×%u)\n",
			epd->panel_w, epd->panel_h,
			EPD_MAX_WIDTH, EPD_MAX_HEIGHT);
		return -ENODEV;
	}

	/* 4. Enable packed pixel writes (I80CPCR bit 0) */
	ret = epd_write_reg(epd, IT8951_REG_I80CPCR, 0x0001u);
	if (ret) {
		dev_err(&epd->spi->dev,
			"Failed to enable packed write mode: %d\n", ret);
		return ret;
	}

	/* 5. Set VCOM only if it differs from what the hardware already has */
	ret = epd_write_cmd(epd, IT8951_CMD_VCOM);
	if (ret) {
		dev_err(&epd->spi->dev, "VCOM read cmd failed: %d\n", ret);
		return ret;
	}
	ret = epd_write_data(epd, 0x0000u);  /* arg: 0 = read VCOM */
	if (ret)
		return ret;
	ret = epd_read_data(epd, &cur_vcom);
	if (ret) {
		dev_err(&epd->spi->dev, "VCOM read data failed: %d\n", ret);
		return ret;
	}

	dev_dbg(&epd->spi->dev,
		"Current VCOM: %u mV, configured: %u mV\n",
		cur_vcom, epd->vcom_mv);

	if (cur_vcom != epd->vcom_mv) {
		ret = epd_set_vcom(epd, epd->vcom_mv);
		if (ret)
			return ret;
	}

	/* 6. LUT variant detection */
	epd_detect_lut(epd);

	epd->initialized = true;
	dev_dbg(&epd->spi->dev, "IT8951 hardware init complete\n");

	return 0;
}

/*
 * epd_hw_sleep - Send SLEEP command to power down the IT8951.
 *
 * The panel draws ~0 µA in sleep mode. Call epd_hw_wakeup() to resume.
 */
void epd_hw_sleep(struct epd_device *epd)
{
	int ret;

	ret = epd_write_cmd(epd, IT8951_CMD_SLEEP);
	if (ret)
		dev_err(&epd->spi->dev, "SLEEP command failed: %d\n", ret);
}

/*
 * epd_hw_wakeup - Wake the IT8951 from sleep.
 *
 * Performs a full hardware reset followed by SYS_RUN so the controller
 * re-initialises its internal state machine.  The caller is responsible
 * for re-sending any display content if needed.
 */
void epd_hw_wakeup(struct epd_device *epd)
{
	int ret;

	epd_hw_reset(epd);

	ret = epd_write_cmd(epd, IT8951_CMD_SYS_RUN);
	if (ret) {
		dev_err(&epd->spi->dev, "SYS_RUN after wakeup failed: %d\n", ret);
		return;
	}

	/* Re-enable packed pixel writes; the register is lost across reset. */
	ret = epd_write_reg(epd, IT8951_REG_I80CPCR, 0x0001u);
	if (ret)
		dev_err(&epd->spi->dev, "I80CPCR restore after wakeup failed: %d\n", ret);
}

/* =========================================================================
 * Exported functions – display configuration
 * ========================================================================= */

/*
 * epd_set_vcom - Program the VCOM voltage into the IT8951.
 *
 * IT8951 VCOM command sequence: CMD_VCOM → write arg 0x0001 (write mode)
 * → write VCOM value in millivolts (positive integer, e.g. 2300 for -2.30 V).
 *
 * Returns 0 on success, negative errno on error.
 */
int epd_set_vcom(struct epd_device *epd, u16 vcom_mv)
{
	int ret;

	ret = epd_write_cmd(epd, IT8951_CMD_VCOM);
	if (ret) {
		dev_err(&epd->spi->dev, "VCOM write cmd failed: %d\n", ret);
		return ret;
	}

	ret = epd_write_data(epd, 0x0001u);  /* arg: 1 = write VCOM */
	if (ret)
		return ret;

	ret = epd_write_data(epd, vcom_mv);
	if (ret) {
		dev_err(&epd->spi->dev,
			"VCOM write data failed (val=%u): %d\n", vcom_mv, ret);
		return ret;
	}

	dev_dbg(&epd->spi->dev, "VCOM set to %u mV\n", vcom_mv);
	return 0;
}

/*
 * epd_enhance_driving - Enable enhanced driving capability.
 *
 * Writes IT8951_ENHANCE_DRV_VAL to IT8951_REG_ENHANCE_DRV.
 * Required for some panels at high refresh rates to stabilise waveforms.
 *
 * Returns 0 on success, negative errno on error.
 */
int epd_enhance_driving(struct epd_device *epd)
{
	int ret;

	ret = epd_write_reg(epd, IT8951_REG_ENHANCE_DRV, IT8951_ENHANCE_DRV_VAL);
	if (ret)
		dev_err(&epd->spi->dev,
			"enhance_driving register write failed: %d\n", ret);
	return ret;
}

/*
 * epd_wait_display_ready - Poll LUTAFSR until the display engine is idle.
 *
 * LUTAFSR is non-zero while a display update waveform is in progress.
 * We wait up to 10 seconds (display updates can be slow in INIT/GC16 mode).
 *
 * Returns 0 when idle, -ETIMEDOUT if still busy after 10 s.
 */
int epd_wait_display_ready(struct epd_device *epd)
{
	u64 deadline = ktime_get_ns() + (u64)10 * NSEC_PER_SEC;
	u16 lut_busy;
	int ret;

	do {
		ret = epd_read_reg(epd, IT8951_REG_LUTAFSR, &lut_busy);
		if (ret) {
			dev_err(&epd->spi->dev,
				"LUTAFSR read failed: %d\n", ret);
			return ret;
		}

		if (lut_busy == 0)
			return 0;

		if (ktime_get_ns() >= deadline) {
			dev_err(&epd->spi->dev,
				"Display engine busy timeout (LUTAFSR=0x%04x)\n",
				lut_busy);
			return -ETIMEDOUT;
		}

		usleep_range(100, 200);
	} while (true);
}

/* =========================================================================
 * Exported functions – image loading and display
 * ========================================================================= */

/*
 * epd_load_image_1bpp - Load a 1-bpp pixel region into IT8951 DRAM.
 *
 * Extracts the sub-region [x, x+w) × [y, y+h) from a 1-bpp Linux framebuffer
 * (MSB-first, row-major, stride = fb_stride bytes/row), converts each bit to a
 * 4-bpp nibble (bit=0 → 0x0 white, bit=1 → 0xF black), packs two nibbles per
 * byte, and streams the result to IT8951 using LD_IMG_AREA in 4BPP host format.
 *
 * This avoids the UP1SR 1-bp display mode entirely: the A2/GC16 waveforms
 * handle binary 0x0/0xF pixel values natively.
 *
 * w must be even (4-byte-aligned for M641).  Enforced by the caller in
 * epd_do_refresh() via epd_align_region().
 *
 * Steps:
 *   1. Set LISAR (IT8951 image RAM base address)
 *   2. Send LD_IMG_AREA command with 5 args
 *   3. Extract region from fb_base, convert 1bpp→4bpp, pack into spi_buf
 *   4. Bulk-send spi_buf (single DMA-friendly spi_write)
 *   5. Send LD_IMG_END
 *
 * Returns 0 on success, negative errno on error.
 */
int epd_load_image_1bpp(struct epd_device *epd,
			 const u8 *fb_base, u32 fb_stride,
			 u16 x, u16 y, u16 w, u16 h)
{
	u16 args[5];
	u32 base_addr = epd->img_ram_addr;
	/*
	 * 4bpp output: 2 pixels per byte → (w * h / 2) bytes.
	 * w is guaranteed even by caller alignment, but (w/2)*h can still be
	 * odd (e.g. w=2, h=3 → 3 bytes).  The IT8951 SPI interface is 16-bit;
	 * an odd-byte transfer splits the last word across SPI transactions and
	 * corrupts the protocol.  Round up to the next even byte and pad with
	 * 0x00 (white nibbles).
	 */
	size_t out_bytes     = (size_t)(w / 2) * h;
	size_t out_bytes_dma = ALIGN(out_bytes, 2);
	int ret;

	if (out_bytes_dma > epd->spi_buf_size) {
		dev_err(&epd->spi->dev,
			"load_image_1bpp: region %ux%u needs %zu bytes, spi_buf only %zu\n",
			w, h, out_bytes_dma, epd->spi_buf_size);
		return -EINVAL;
	}

	/* 1. Set IT8951 image RAM target address via LISAR registers. */
	ret = epd_write_reg(epd, IT8951_REG_LISAR + 2, (u16)(base_addr >> 16));
	if (ret)
		return ret;
	ret = epd_write_reg(epd, IT8951_REG_LISAR, (u16)(base_addr & 0xFFFFu));
	if (ret)
		return ret;

	/* 2. LD_IMG_AREA command with 5 arguments. */
	ret = epd_write_cmd(epd, IT8951_CMD_LD_IMG_AREA);
	if (ret)
		return ret;

	/*
	 * args[0] format: bits[15:8]=endian, bits[7:4]=pixel_fmt, bits[3:0]=rotate
	 * Using 4BPP because we convert 1bpp bits to 0x0/0xF nibbles below.
	 */
	args[0] = (u16)((IT8951_ENDIAN_LITTLE << 8) |
			(IT8951_PIX_FMT_4BPP  << 4) |
			IT8951_ROTATE_0);
	args[1] = x;
	args[2] = y;
	args[3] = w;
	args[4] = h;

	{
		size_t i;

		for (i = 0; i < ARRAY_SIZE(args); i++) {
			ret = epd_write_data(epd, args[i]);
			if (ret)
				return ret;
		}
	}

	/*
	 * 3. Extract [x, x+w) × [y, y+h) from the framebuffer, convert each
	 *    bit to a 4bpp nibble, and pack two nibbles per byte.
	 *
	 *    Linux 1bpp: in byte B at column group c*8..c*8+7, bit 7 is the
	 *    leftmost pixel.  For pixel at column p:
	 *      byte   = fb_base[row * fb_stride + p / 8]
	 *      bit    = (byte >> (7 - p % 8)) & 1
	 *      nibble = bit ? 0xF : 0x0
	 *
	 *    IT8951 4bpp: upper nibble = left pixel of pair, lower = right.
	 *    Two pairs per 16-bit word; we build in epd->spi_buf directly.
	 */
	{
		u8 *dst = epd->spi_buf;
		size_t dst_pos = 0;
		int row, col;

		for (row = 0; row < (int)h; row++) {
			int src_row = (int)y + row;

			for (col = 0; col < (int)w; col += 2) {
				int sc0 = (int)x + col;
				int sc1 = sc0 + 1;
				u8 src_byte;
				u8 n0, n1;

				src_byte = fb_base[(size_t)src_row * fb_stride + sc0 / 8];
				n0 = ((src_byte >> (7 - sc0 % 8)) & 1u) ? 0xFu : 0x0u;

				if (sc1 < (int)x + (int)w) {
					src_byte = fb_base[(size_t)src_row * fb_stride + sc1 / 8];
					n1 = ((src_byte >> (7 - sc1 % 8)) & 1u) ? 0xFu : 0x0u;
				} else {
					n1 = 0x0u;  /* white padding for odd width */
				}

				/* little-endian 4bpp: first pixel in low nibble */
				dst[dst_pos++] = (n1 << 4) | n0;
			}
		}

		WARN_ON(dst_pos != out_bytes);

		/* Pad to even byte count if necessary (white nibble = 0x00). */
		if (out_bytes_dma > out_bytes)
			dst[dst_pos] = 0x00;
	}

	/* 4. Bulk-send spi_buf: preamble + all 4bpp pixel bytes with CS held low. */
	ret = epd_wait_busy(epd);
	if (ret)
		return ret;

	gpiod_set_value_cansleep(epd->gpio_cs, 0);

	ret = epd_spi_send_word(epd, IT8951_PREAMBLE_WRITE);
	if (ret) {
		dev_err(&epd->spi->dev,
			"load_image_1bpp: preamble write failed: %d\n", ret);
		goto out_cs_high;
	}

	ret = epd_wait_busy(epd);
	if (ret)
		goto out_cs_high;

	ret = spi_write(epd->spi, epd->spi_buf, out_bytes_dma);
	if (ret)
		dev_err(&epd->spi->dev,
			"load_image_1bpp: bulk SPI write failed (%zu bytes): %d\n",
			out_bytes, ret);

out_cs_high:
	gpiod_set_value_cansleep(epd->gpio_cs, 1);
	if (ret)
		return ret;

	/* 5. LD_IMG_END */
	ret = epd_write_cmd(epd, IT8951_CMD_LD_IMG_END);
	if (ret)
		dev_err(&epd->spi->dev, "LD_IMG_END failed: %d\n", ret);

	return ret;
}

/*
 * epd_display_area - Trigger a display update for a pixel region.
 *
 * Waits for the display engine to be idle, then sends CMD_DPY_AREA with
 * the rectangle coordinates and the requested refresh mode.
 *
 * @mode: one of EPD_MODE_INIT, EPD_MODE_DU, EPD_MODE_GC16, EPD_MODE_A2_*, etc.
 *
 * Returns 0 on success, negative errno on error.
 */
int epd_display_area(struct epd_device *epd, u16 x, u16 y, u16 w, u16 h,
		     u8 mode)
{
	u16 args[5];
	size_t i;
	int ret;

	ret = epd_wait_display_ready(epd);
	if (ret)
		return ret;

	ret = epd_write_cmd(epd, IT8951_CMD_DPY_AREA);
	if (ret)
		return ret;

	args[0] = x;
	args[1] = y;
	args[2] = w;
	args[3] = h;
	args[4] = (u16)mode;

	for (i = 0; i < ARRAY_SIZE(args); i++) {
		ret = epd_write_data(epd, args[i]);
		if (ret) {
			dev_err(&epd->spi->dev,
				"display_area: arg[%zu] write failed: %d\n",
				i, ret);
			return ret;
		}
	}

	dev_dbg(&epd->spi->dev,
		"display_area: x=%u y=%u w=%u h=%u mode=%u\n",
		x, y, w, h, mode);

	return 0;
}

/*
 * epd_full_clear - Clear the entire panel to white using INIT mode.
 *
 * Sends an all-zero 4bpp image (0x00 = white) covering the full panel,
 * then triggers a full-screen INIT refresh to erase any A2-mode ghosting.
 *
 * This function bypasses epd_load_image_1bpp() and drives the SPI protocol
 * directly to avoid spi_buf aliasing: epd_load_image_1bpp() would write its
 * 4bpp output into spi_buf, so we cannot use spi_buf as its 1bpp source.
 *
 * Returns 0 on success, negative errno on error.
 */
int epd_full_clear(struct epd_device *epd)
{
	u16 args[5];
	u32 base_addr = epd->img_ram_addr;
	/*
	 * 4bpp: 2 pixels per byte.  Full panel with even width (already ensured
	 * by the panel detection which sanity-checks against EPD_MAX_WIDTH).
	 */
	size_t out_bytes = (size_t)(epd->panel_w / 2) * epd->panel_h;
	size_t i;
	int ret;

	/*
	 * Wait for any in-progress waveform before writing to image DRAM.
	 * epd_full_clear is called both from probe (no prior waveform) and from
	 * epd_do_refresh ghosting-recovery (a waveform may be active).
	 */
	ret = epd_wait_display_ready(epd);
	if (ret) {
		dev_err(&epd->spi->dev,
			"full_clear: display not ready before DRAM write: %d\n", ret);
		return ret;
	}

	if (out_bytes > epd->spi_buf_size) {
		dev_err(&epd->spi->dev,
			"full_clear: %zu bytes needed, spi_buf is %zu\n",
			out_bytes, epd->spi_buf_size);
		return -EINVAL;
	}

	/* 0x00 = 4bpp white (nibble 0x0 = no charge = white pixel) */
	memset(epd->spi_buf, 0x00, out_bytes);

	/* Set LISAR */
	ret = epd_write_reg(epd, IT8951_REG_LISAR + 2, (u16)(base_addr >> 16));
	if (ret)
		return ret;
	ret = epd_write_reg(epd, IT8951_REG_LISAR, (u16)(base_addr & 0xFFFFu));
	if (ret)
		return ret;

	/* LD_IMG_AREA for full panel */
	ret = epd_write_cmd(epd, IT8951_CMD_LD_IMG_AREA);
	if (ret)
		return ret;

	args[0] = (u16)((IT8951_ENDIAN_LITTLE << 8) |
			(IT8951_PIX_FMT_4BPP  << 4) |
			IT8951_ROTATE_0);
	args[1] = 0;
	args[2] = 0;
	args[3] = epd->panel_w;
	args[4] = epd->panel_h;

	for (i = 0; i < ARRAY_SIZE(args); i++) {
		ret = epd_write_data(epd, args[i]);
		if (ret)
			return ret;
	}

	/* Bulk-send the all-white 4bpp buffer */
	ret = epd_wait_busy(epd);
	if (ret)
		return ret;

	gpiod_set_value_cansleep(epd->gpio_cs, 0);

	ret = epd_spi_send_word(epd, IT8951_PREAMBLE_WRITE);
	if (ret) {
		dev_err(&epd->spi->dev, "full_clear: preamble failed: %d\n", ret);
		goto out_cs_high;
	}

	ret = epd_wait_busy(epd);
	if (ret)
		goto out_cs_high;

	ret = spi_write(epd->spi, epd->spi_buf, out_bytes);
	if (ret)
		dev_err(&epd->spi->dev,
			"full_clear: bulk write failed (%zu bytes): %d\n",
			out_bytes, ret);

out_cs_high:
	gpiod_set_value_cansleep(epd->gpio_cs, 1);
	if (ret)
		return ret;

	ret = epd_write_cmd(epd, IT8951_CMD_LD_IMG_END);
	if (ret)
		return ret;

	/* INIT waveform erases all pixel memory (removes ghosting) */
	ret = epd_display_area(epd, 0, 0, epd->panel_w, epd->panel_h,
			       EPD_MODE_INIT);
	if (ret)
		dev_err(&epd->spi->dev, "full_clear: display_area failed: %d\n", ret);

	return ret;
}
