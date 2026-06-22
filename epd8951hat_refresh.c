// SPDX-License-Identifier: GPL-2.0
/*
 * epd8951hat_refresh.c – Refresh policy and SPI upload logic.
 *
 * Called from the worker in epd8951hat_drv.c after XRGB8888→mono conversion
 * has already populated epd->mono_buf for the full panel.
 *
 * Policy:
 *   - damage area ≥ EPD_FULL_REFRESH_THRESHOLD% → INIT + GC16 full panel
 *   - A2 count ≥ EPD_A2_GHOSTING_LIMIT         → INIT clear + GC16 full panel
 *   - Otherwise                                 → A2 partial on damage rect
 *   - Double-buffer memcmp skips unchanged regions in all paths
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/string.h>

#include <drm/drm_rect.h>

#include "epd8951hat.h"

/* =========================================================================
 * Region alignment helpers
 * ========================================================================= */

/*
 * Align a pixel rectangle outward to IT8951 1bpp packed-mode requirements.
 *
 * In 1bpp packed mode the image is loaded with Area_X = x/8 and Area_W = w/8,
 * so both x and w must be multiples of 8.  This is stricter than the old 4bpp
 * requirement (2 pixels for M841, 8 pixels for M641) but is now uniform across
 * all LUT variants — the needs_4byte_align flag no longer changes behaviour.
 */
static void epd_align_region(struct epd_device *epd,
			      int *x, int *y, int *w, int *h)
{
	int align = 8;
	int x0 = *x, x1 = *x + *w;

	x0 = ALIGN_DOWN(x0, align);
	x1 = ALIGN(x1, align);
	x0 = max(x0, 0);
	x1 = min(x1, (int)epd->panel_w);

	*x = x0;
	*w = x1 - x0;

	*y = max(*y, 0);
	if (*y + *h > (int)epd->panel_h)
		*h = (int)epd->panel_h - *y;
}

/* =========================================================================
 * Double-buffer comparison
 *
 * Compares mono_buf vs screen_shadow row-by-row for the given region.
 * Shrinks the vertical extent to only the rows that actually differ.
 * Returns true if any row changed.
 * ========================================================================= */
static bool epd_region_changed(struct epd_device *epd,
				int *rx, int *ry, int *rw, int *rh,
				u64 *changed_pixels)
{
	const u8 *cur  = epd->mono_buf;
	const u8 *prev = epd->screen_shadow;
	u32 stride     = epd->fb_stride;
	int top = -1, bot = -1;
	int byte0 = *rx / 8;
	int byte1 = (*rx + *rw - 1) / 8;
	int row;
	u64 diff_bytes = 0;

	for (row = 0; row < *rh; row++) {
		int src_row = *ry + row;
		size_t off  = (size_t)src_row * stride + byte0;
		size_t len  = (size_t)(byte1 - byte0 + 1);
		const u8 *a = cur + off;
		const u8 *b = prev + off;
		size_t i;
		bool row_changed = false;

		for (i = 0; i < len; i++) {
			if (a[i] != b[i]) {
				diff_bytes++;
				row_changed = true;
			}
		}
		if (row_changed) {
			if (top < 0)
				top = row;
			bot = row;
		}
	}

	if (top < 0) {
		*changed_pixels = 0;
		return false;
	}

	*ry += top;
	*rh  = bot - top + 1;
	/* Each differing byte represents 8 pixels in 1bpp packed format */
	*changed_pixels = diff_bytes * 8;
	return true;
}

/* =========================================================================
 * Update screen_shadow for a refreshed region
 * ========================================================================= */
static void epd_update_shadow(struct epd_device *epd,
			       int rx, int ry, int rw, int rh)
{
	u32 stride = epd->fb_stride;
	int byte0  = rx / 8;
	int byte1  = (rx + rw - 1) / 8;
	size_t len = (size_t)(byte1 - byte0 + 1);
	int row;

	for (row = 0; row < rh; row++) {
		size_t off = (size_t)(ry + row) * stride + byte0;

		memcpy(epd->screen_shadow + off, epd->mono_buf + off, len);
	}
}

/* =========================================================================
 * epd_do_refresh – core refresh logic (runs in workqueue context)
 *
 * @damage: merged damage rectangle in panel pixel coordinates
 *
 * Precondition: epd->mono_buf has been populated from the current GEM
 * framebuffer via drm_fb_xrgb8888_to_mono() for the full panel.
 * ========================================================================= */
void epd_do_refresh(struct epd_device *epd, const struct drm_rect *damage)
{
	int x = damage->x1;
	int y = damage->y1;
	int w = drm_rect_width(damage);
	int h = drm_rect_height(damage);
	u64 panel_area = (u64)epd->panel_w * epd->panel_h;
	u64 changed_pixels;
	unsigned int pct;
	int ret;

	if ((u64)w * h < EPD_MIN_UPDATE_PIXELS) {
		dev_dbg(&epd->spi->dev, "skip tiny damage %dx%d\n", w, h);
		return;
	}

	if (mutex_lock_interruptible(&epd->refresh_mutex))
		return;

	/*
	 * Gate all refresh paths on shadow comparison first.  fbdev/fbcon
	 * reports full-screen damage on every commit, so basing the full/partial
	 * decision on the damage rect would always trigger a slow GC16.  Instead,
	 * scan the shadow here to find what actually changed; the trimmed region
	 * is then used to compute pct so small real changes (cursor, one line of
	 * text) correctly score low and take the A2 path.
	 */
	if (!epd_region_changed(epd, &x, &y, &w, &h, &changed_pixels)) {
		dev_dbg(&epd->spi->dev,
			"damage (%d,%d)+%dx%d unchanged vs shadow\n",
			damage->x1, damage->y1,
			drm_rect_width(damage), drm_rect_height(damage));
		goto out;
	}

	/*
	 * Percentage based on actually-differing pixels (diff_bytes×8), not the
	 * bounding-box area.  Fbcon reports full-screen damage on every commit,
	 * so using the rect area would always score ~100%; counting real diffs
	 * means a single line of text or a cursor correctly scores low.
	 */
	pct = (panel_area > 0) ?
		(unsigned int)min_t(u64, changed_pixels * 100 / panel_area, 100) : 100;

	/* ---- Full refresh ------------------------------------------------- */
	if (pct >= EPD_FULL_REFRESH_THRESHOLD) {
		dev_dbg(&epd->spi->dev, "full refresh (changed=%u%%)\n", pct);

		ret = epd_full_clear(epd);
		if (ret) {
			dev_err(&epd->spi->dev, "full clear failed: %d\n", ret);
			goto out;
		}

		ret = epd_wait_display_ready(epd);
		if (ret) {
			dev_err(&epd->spi->dev, "display not ready: %d\n", ret);
			goto out;
		}

		ret = epd_load_image_1bpp(epd, epd->mono_buf, epd->fb_stride,
					  0, 0, epd->panel_w, epd->panel_h);
		if (ret) {
			dev_err(&epd->spi->dev, "load image failed: %d\n", ret);
			goto out;
		}

		ret = epd_display_area_1bpp(epd, 0, 0, epd->panel_w, epd->panel_h,
					    EPD_MODE_GC16);
		if (ret)
			dev_err(&epd->spi->dev, "display_area_1bpp GC16 failed: %d\n", ret);

		memcpy(epd->screen_shadow, epd->mono_buf, epd->fb_size);
		atomic_set(&epd->a2_count, 0);
		goto out;
	}

	/* ---- A2 ghosting recovery ----------------------------------------- */
	if (atomic_read(&epd->a2_count) >= EPD_A2_GHOSTING_LIMIT) {
		dev_dbg(&epd->spi->dev, "A2 ghost limit; INIT clear + GC16\n");

		ret = epd_full_clear(epd);
		if (ret) {
			dev_err(&epd->spi->dev, "ghost-clear failed: %d\n", ret);
			goto out;
		}
		atomic_set(&epd->a2_count, 0);

		ret = epd_wait_display_ready(epd);
		if (ret) {
			dev_err(&epd->spi->dev, "not ready after ghost-clear: %d\n", ret);
			goto out;
		}

		ret = epd_load_image_1bpp(epd, epd->mono_buf, epd->fb_stride,
					  0, 0, epd->panel_w, epd->panel_h);
		if (ret) {
			dev_err(&epd->spi->dev, "post-clear reload failed: %d\n", ret);
			goto out;
		}

		ret = epd_display_area_1bpp(epd, 0, 0, epd->panel_w, epd->panel_h,
					    EPD_MODE_GC16);
		if (ret)
			dev_err(&epd->spi->dev, "post-clear GC16 failed: %d\n", ret);

		memcpy(epd->screen_shadow, epd->mono_buf, epd->fb_size);
		goto out;
	}

	/* ---- Partial A2 refresh ------------------------------------------- */
	epd_align_region(epd, &x, &y, &w, &h);
	if (w <= 0 || h <= 0)
		goto out;

	dev_dbg(&epd->spi->dev, "partial A2 (%d,%d)+%dx%d [%u%% changed]\n",
		x, y, w, h, pct);

	ret = epd_wait_display_ready(epd);
	if (ret) {
		dev_err(&epd->spi->dev, "not ready before partial load: %d\n", ret);
		goto out;
	}

	ret = epd_load_image_1bpp(epd, epd->mono_buf, epd->fb_stride,
				   (u16)x, (u16)y, (u16)w, (u16)h);
	if (ret) {
		dev_err(&epd->spi->dev, "partial load failed: %d\n", ret);
		goto out;
	}

	ret = epd_display_area_1bpp(epd, x, y, w, h, epd->a2_mode);
	if (ret) {
		dev_err(&epd->spi->dev, "partial display_area_1bpp failed: %d\n", ret);
		goto out;
	}

	epd_update_shadow(epd, x, y, w, h);
	atomic_inc(&epd->a2_count);

out:
	mutex_unlock(&epd->refresh_mutex);
}
