



#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/mutex.h>

#include "epd8951hat.h"
#include "epd8951hat_pipeline.h"

void epd_do_refresh(struct epd_device *epd,
		    int clip_x0, int clip_x1,
		    int clip_y0, int clip_y1)
{
	u16 h      = epd->panel_h;
	u32 stride = epd->fb_stride;
	int b_clip0, b_clip1;
	int y0, y1, b0, b1;
	int ret;

	/* Convert pixel x bounds to inclusive byte-column bounds for the diff. */
	b_clip0 = max(clip_x0 / 8, 0);
	b_clip1 = min(clip_x1 / 8, (int)stride - 1);

	epd_compute_dirty_rect(h, stride, epd->mirror_x,
			       epd->mono_buf, epd->flip_buf,
			       b_clip0, b_clip1,
			       clip_y0, clip_y1,
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

int epd_clean_refresh_locked(struct epd_device *epd, u8 mode)
{
	int ret;

	ret = epd_wait_display_ready(epd);
	if (ret) {
		dev_err(&epd->spi->dev, "clean refresh: display not ready: %d\n", ret);
		return ret;
	}

	/* Reload the whole shadow buffer and re-issue it through the clean
	 * waveform. Content is identical, so flip_buf stays valid and the fast
	 * A2 diff path resumes unchanged on the next update. */
	ret = epd_load_image_1bpp(epd, epd->flip_buf, epd->fb_stride,
				   0, 0, epd->panel_w, epd->panel_h);
	if (ret) {
		dev_err(&epd->spi->dev, "clean refresh: load image failed: %d\n", ret);
		return ret;
	}

	ret = epd_display_area_1bpp(epd, 0, 0, epd->panel_w, epd->panel_h, mode);
	if (ret)
		dev_err(&epd->spi->dev, "clean refresh failed: %d\n", ret);

	return ret;
}
