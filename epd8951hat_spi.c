



#include <asm/byteorder.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/string.h>
#include <linux/types.h>

#include "epd8951hat.h"
#include "epd8951hat_pipeline.h"






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






static int epd_write_cmd(struct epd_device *epd, u16 cmd)
{
	__be16 buf[2] = {
		cpu_to_be16(IT8951_PREAMBLE_CMD),
		cpu_to_be16(cmd),
	};
	int ret;

	ret = epd_wait_busy(epd);
	if (ret)
		return ret;

	ret = spi_write(epd->spi, buf, sizeof(buf));
	if (ret)
		dev_err(&epd->spi->dev, "SPI error sending CMD 0x%04x: %d\n",
			cmd, ret);
	return ret;
}



static int epd_write_data(struct epd_device *epd, u16 data)
{
	__be16 buf[2] = {
		cpu_to_be16(IT8951_PREAMBLE_WRITE),
		cpu_to_be16(data),
	};
	int ret;

	ret = epd_wait_busy(epd);
	if (ret)
		return ret;

	ret = spi_write(epd->spi, buf, sizeof(buf));
	if (ret)
		dev_err(&epd->spi->dev, "SPI error sending DATA 0x%04x: %d\n",
			data, ret);
	return ret;
}



static int epd_write_data_bulk(struct epd_device *epd,
			       const void *payload, size_t len)
{
	__be16 pre = cpu_to_be16(IT8951_PREAMBLE_WRITE);
	struct spi_transfer xfers[2] = {
		{ .tx_buf = &pre,    .len = sizeof(pre) },
		{ .tx_buf = payload, .len = len         },
	};
	int ret;

	ret = epd_wait_busy(epd);
	if (ret)
		return ret;

	ret = spi_sync_transfer(epd->spi, xfers, ARRAY_SIZE(xfers));
	if (ret)
		dev_err(&epd->spi->dev,
			"SPI bulk write failed (%zu bytes): %d\n", len, ret);
	return ret;
}



static int epd_read_words(struct epd_device *epd, u16 *out, size_t n)
{
	size_t total = 4 + 2 * n;    
	struct spi_transfer xfer;
	u8 *tx, *rx;
	size_t i;
	int ret;

	ret = epd_wait_busy(epd);
	if (ret)
		return ret;

	tx = kzalloc(total, GFP_KERNEL);
	rx = kzalloc(total, GFP_KERNEL);
	if (!tx || !rx) {
		ret = -ENOMEM;
		goto out;
	}

	 
	tx[0] = (IT8951_PREAMBLE_READ >> 8) & 0xff;
	tx[1] = IT8951_PREAMBLE_READ & 0xff;

	memset(&xfer, 0, sizeof(xfer));
	xfer.tx_buf = tx;
	xfer.rx_buf = rx;
	xfer.len    = total;

	ret = spi_sync_transfer(epd->spi, &xfer, 1);
	if (ret) {
		dev_err(&epd->spi->dev, "SPI read of %zu words failed: %d\n",
			n, ret);
		goto out;
	}

	 
	for (i = 0; i < n; i++)
		out[i] = ((u16)rx[4 + 2 * i] << 8) | rx[4 + 2 * i + 1];

out:
	kfree(tx);
	kfree(rx);
	return ret;
}



static int epd_read_data(struct epd_device *epd, u16 *out)
{
	return epd_read_words(epd, out, 1);
}






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






static void epd_detect_lut(struct epd_device *epd)
{
	const char *lut     = (const char *)epd->dev_info.lut_version;
	size_t      lut_len = sizeof(epd->dev_info.lut_version);

	epd->lut_variant = epd_lut_classify(lut, lut_len,
					     &epd->a2_mode,
					     &epd->needs_4byte_align);

	dev_dbg(&epd->spi->dev,
		"LUT variant: %u (A2 mode %u, 4-byte align: %d)\n",
		epd->lut_variant, epd->a2_mode, epd->needs_4byte_align);
}






void epd_hw_reset(struct epd_device *epd)
{
	gpiod_set_value_cansleep(epd->gpio_rst, 0);
	msleep(10);
	gpiod_set_value_cansleep(epd->gpio_rst, 1);
	msleep(200);

	 
	epd_wait_busy(epd);
}



int epd_hw_init(struct epd_device *epd)
{
	struct it8951_dev_info *di = &epd->dev_info;
	u16 cur_vcom = 0;
	int ret;

	 
	epd_hw_reset(epd);

	 
	ret = epd_write_cmd(epd, IT8951_CMD_SYS_RUN);
	if (ret) {
		dev_err(&epd->spi->dev, "SYS_RUN failed: %d\n", ret);
		return ret;
	}

	 
	ret = epd_write_cmd(epd, IT8951_CMD_GET_DEV_INFO);
	if (ret) {
		dev_err(&epd->spi->dev, "GET_DEV_INFO cmd failed: %d\n", ret);
		return ret;
	}

	

	ret = epd_read_words(epd, (u16 *)di, sizeof(*di) / sizeof(u16));
	if (ret) {
		dev_err(&epd->spi->dev, "GET_DEV_INFO read failed: %d\n", ret);
		return ret;
	}

	epd->panel_w      = di->panel_w;
	epd->panel_h      = di->panel_h;
	epd->img_ram_addr = ((u32)di->mem_addr_h << 16) | di->mem_addr_l;

	dev_dbg(&epd->spi->dev,
		"IT8951 panel %u×%u img_ram=0x%08x\n",
		epd->panel_w, epd->panel_h, epd->img_ram_addr);

	 
	if (!epd->panel_w || !epd->panel_h ||
	    epd->panel_w > EPD_MAX_WIDTH || epd->panel_h > EPD_MAX_HEIGHT) {
		dev_err(&epd->spi->dev,
			"Implausible panel size %u×%u (max %u×%u)\n",
			epd->panel_w, epd->panel_h,
			EPD_MAX_WIDTH, EPD_MAX_HEIGHT);
		return -ENODEV;
	}

	 
	ret = epd_write_reg(epd, IT8951_REG_I80CPCR, 0x0001u);
	if (ret) {
		dev_err(&epd->spi->dev,
			"Failed to enable packed write mode: %d\n", ret);
		return ret;
	}

	 
	ret = epd_write_cmd(epd, IT8951_CMD_VCOM);
	if (ret) {
		dev_err(&epd->spi->dev, "VCOM read cmd failed: %d\n", ret);
		return ret;
	}
	ret = epd_write_data(epd, 0x0000u);   
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

	 
	epd_detect_lut(epd);

	epd->initialized = true;
	dev_dbg(&epd->spi->dev, "IT8951 hardware init complete\n");

	return 0;
}



void epd_hw_sleep(struct epd_device *epd)
{
	int ret;

	ret = epd_write_cmd(epd, IT8951_CMD_SLEEP);
	if (ret)
		dev_err(&epd->spi->dev, "SLEEP command failed: %d\n", ret);
}



void epd_hw_wakeup(struct epd_device *epd)
{
	int ret;

	epd_hw_reset(epd);

	ret = epd_write_cmd(epd, IT8951_CMD_SYS_RUN);
	if (ret) {
		dev_err(&epd->spi->dev, "SYS_RUN after wakeup failed: %d\n", ret);
		return;
	}

	 
	ret = epd_write_reg(epd, IT8951_REG_I80CPCR, 0x0001u);
	if (ret)
		dev_err(&epd->spi->dev, "I80CPCR restore after wakeup failed: %d\n", ret);
}






int epd_set_vcom(struct epd_device *epd, u16 vcom_mv)
{
	int ret;

	ret = epd_write_cmd(epd, IT8951_CMD_VCOM);
	if (ret) {
		dev_err(&epd->spi->dev, "VCOM write cmd failed: %d\n", ret);
		return ret;
	}

	ret = epd_write_data(epd, 0x0001u);   
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



int epd_enhance_driving(struct epd_device *epd)
{
	int ret;

	ret = epd_write_reg(epd, IT8951_REG_ENHANCE_DRV, IT8951_ENHANCE_DRV_VAL);
	if (ret)
		dev_err(&epd->spi->dev,
			"enhance_driving register write failed: %d\n", ret);
	return ret;
}



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






int epd_load_image_1bpp(struct epd_device *epd,
			 const u8 *fb_base, u32 fb_stride,
			 u16 x, u16 y, u16 w, u16 h)
{
	u16 args[5];
	u32 base_addr    = epd->img_ram_addr;
	size_t row_bytes     = w / 8;
	size_t out_bytes     = row_bytes * (size_t)h;
	size_t out_bytes_dma = ALIGN(out_bytes, 2);
	u8 *dst;
	size_t i;
	int row, ret;

	if (out_bytes_dma > epd->spi_buf_size) {
		dev_err(&epd->spi->dev,
			"load_image_1bpp: region %ux%u needs %zu bytes, spi_buf only %zu\n",
			w, h, out_bytes_dma, epd->spi_buf_size);
		return -EINVAL;
	}

	 
	ret = epd_write_reg(epd, IT8951_REG_LISAR + 2, (u16)(base_addr >> 16));
	if (ret)
		return ret;
	ret = epd_write_reg(epd, IT8951_REG_LISAR, (u16)(base_addr & 0xFFFFu));
	if (ret)
		return ret;

	 
	ret = epd_write_cmd(epd, IT8951_CMD_LD_IMG_AREA);
	if (ret)
		return ret;

	/* BIG endian: SPI sends high byte first, so the controller must not swap
	 * pairs of bytes. LITTLE would cause the IT8951 to byte-swap every 16-bit
	 * word, reversing each 16-pixel group and producing flipped columns. */
	args[0] = (u16)((IT8951_ENDIAN_BIG  << 8) |
			(IT8951_PIX_FMT_8BPP << 4) |
			IT8951_ROTATE_0);
	args[1] = x / 8;
	args[2] = y;
	args[3] = w / 8;
	args[4] = h;

	for (i = 0; i < ARRAY_SIZE(args); i++) {
		ret = epd_write_data(epd, args[i]);
		if (ret)
			return ret;
	}

	 
	dst = epd->spi_buf;
	for (row = 0; row < (int)h; row++) {
		const u8 *src = fb_base + (size_t)((int)y + row) * fb_stride + x / 8;

		memcpy(dst, src, row_bytes);
		dst += row_bytes;
	}
	if (out_bytes_dma > out_bytes)
		epd->spi_buf[out_bytes] = 0xFFu;

	 
	ret = epd_write_data_bulk(epd, epd->spi_buf, out_bytes_dma);
	if (ret)
		return ret;

	 
	ret = epd_write_cmd(epd, IT8951_CMD_LD_IMG_END);
	if (ret)
		dev_err(&epd->spi->dev, "LD_IMG_END failed: %d\n", ret);

	return ret;
}



int epd_display_area(struct epd_device *epd, u16 x, u16 y, u16 w, u16 h,
		     u8 mode)
{
	u16 args[5];
	size_t i;
	int ret;

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



int epd_display_area_1bpp(struct epd_device *epd, u16 x, u16 y, u16 w, u16 h,
			   u8 mode)
{
	u16 up1sr2;
	int ret, ret2;

	 
	ret = epd_read_reg(epd, IT8951_REG_UP1SR + 2, &up1sr2);
	if (ret) {
		dev_err(&epd->spi->dev,
			"display_area_1bpp: UP1SR+2 read failed: %d\n", ret);
		return ret;
	}
	ret = epd_write_reg(epd, IT8951_REG_UP1SR + 2,
			    up1sr2 | IT8951_UP1SR2_1BPP_EN);
	if (ret) {
		dev_err(&epd->spi->dev,
			"display_area_1bpp: UP1SR+2 set failed: %d\n", ret);
		return ret;
	}

	 
	ret = epd_write_reg(epd, IT8951_REG_BGVR, IT8951_BGVR_DEFAULT);
	if (ret) {
		dev_err(&epd->spi->dev,
			"display_area_1bpp: BGVR write failed: %d\n", ret);
		goto restore_up1sr;
	}

	 
	ret = epd_display_area(epd, x, y, w, h, mode);
	if (ret)
		goto restore_up1sr;

	 
	ret = epd_wait_display_ready(epd);
	if (ret)
		dev_err(&epd->spi->dev,
			"display_area_1bpp: waveform timeout: %d\n", ret);

restore_up1sr:
	 
	ret2 = epd_read_reg(epd, IT8951_REG_UP1SR + 2, &up1sr2);
	if (!ret2)
		epd_write_reg(epd, IT8951_REG_UP1SR + 2,
			      up1sr2 & ~IT8951_UP1SR2_1BPP_EN);
	else
		dev_err(&epd->spi->dev,
			"display_area_1bpp: UP1SR+2 restore read failed: %d\n", ret2);

	return ret;
}



int epd_full_clear(struct epd_device *epd)
{
	u16 args[5];
	u32 base_addr = epd->img_ram_addr;
	

	size_t out_bytes = (size_t)(epd->panel_w / 2) * epd->panel_h;
	size_t i;
	int ret;

	

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

	 
	memset(epd->spi_buf, 0xFF, out_bytes);

	 
	ret = epd_write_reg(epd, IT8951_REG_LISAR + 2, (u16)(base_addr >> 16));
	if (ret)
		return ret;
	ret = epd_write_reg(epd, IT8951_REG_LISAR, (u16)(base_addr & 0xFFFFu));
	if (ret)
		return ret;

	 
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

	 
	ret = epd_write_data_bulk(epd, epd->spi_buf, out_bytes);
	if (ret)
		return ret;

	ret = epd_write_cmd(epd, IT8951_CMD_LD_IMG_END);
	if (ret)
		return ret;

	 
	ret = epd_display_area(epd, 0, 0, epd->panel_w, epd->panel_h,
			       EPD_MODE_INIT);
	if (ret) {
		dev_err(&epd->spi->dev, "full_clear: display_area failed: %d\n", ret);
		return ret;
	}

	

	ret = epd_wait_display_ready(epd);
	if (ret)
		dev_err(&epd->spi->dev,
			"full_clear: INIT waveform timeout: %d\n", ret);

	return ret;
}
