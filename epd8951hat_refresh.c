



#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/mutex.h>

#include "epd8951hat.h"
#include "epd8951hat_pipeline.h"

void epd_do_refresh(struct epd_device *epd)
{
	u16 h      = epd->panel_h;
	u32 stride = epd->fb_stride;
	int y0, y1, b0, b1;
	int ret;

	epd_compute_dirty_rect(h, stride, epd->mirror_x,
			       epd->mono_buf, epd->flip_buf,
			       &y0, &y1, &b0, &b1);

	if (y0 > y1)
		return;  

	epd_align_dirty_bytes(stride, epd->needs_4byte_align, &b0, &b1);

	{
		u16 rx = (u16)(b0 * 8);
		u16 rw = (u16)((b1 - b0 + 1) * 8);
		u16 ry = (u16)y0;
		u16 rh = (u16)(y1 - y0 + 1);

		if (mutex_lock_interruptible(&epd->refresh_mutex))
			return;

		ret = epd_wait_display_ready(epd);
		if (ret) {
			dev_err(&epd->spi->dev, "display not ready: %d\n", ret);
			goto out;
		}

		ret = epd_load_image_1bpp(epd, epd->flip_buf, stride,
					   rx, ry, rw, rh);
		if (ret) {
			dev_err(&epd->spi->dev, "load image failed: %d\n", ret);
			goto out;
		}

		ret = epd_display_area_1bpp(epd, rx, ry, rw, rh, epd->a2_mode);
		if (ret)
			dev_err(&epd->spi->dev, "A2 refresh failed: %d\n", ret);
out:
		mutex_unlock(&epd->refresh_mutex);
	}
}
