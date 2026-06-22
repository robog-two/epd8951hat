// SPDX-License-Identifier: GPL-2.0
/*
 * epd8951hat_refresh.c – Refresh scheduling and mode selection.
 *
 * Responsibilities:
 *   - Coalescing delayed-work queue for normal writes
 *   - Near-immediate priority work queue for cursor / tiny-area updates
 *   - Double-buffer comparison (fb_vaddr vs screen_shadow) to minimise SPI
 *   - A2 / GC16 / INIT mode selection
 *   - "White-out" optimisation: changed tiles only, not bounding-box rectangle
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/workqueue.h>

#include "epd8951hat.h"

/* =========================================================================
 * Region alignment helpers
 * ========================================================================= */

/*
 * For the IT8951 in 4bpp packed mode the x-coordinate and width must satisfy:
 *   - x      : multiple of 2 pixels (one 4bpp byte = 2 pixels)
 *   - width  : multiple of 2 pixels
 *   - M641 (6") additionally requires 8-pixel (4-byte) alignment.
 *
 * Align the supplied rectangle outward so it covers at least [x, x+w) and
 * is safe to pass to epd_load_image_1bpp / epd_display_area.
 */
static void epd_align_region(struct epd_device *epd,
			      int *x, int *y, int *w, int *h)
{
	int align = epd->needs_4byte_align ? 8 : 2;
	int x0 = *x, x1 = *x + *w;

	/* Round x down, x1 up, then recompute w */
	x0 = ALIGN_DOWN(x0, align);
	x1 = ALIGN(x1, align);

	/* Clamp to panel bounds */
	x0 = max(x0, 0);
	x1 = min(x1, (int)epd->panel_w);

	*x = x0;
	*w = x1 - x0;

	/* y and h: align to 1-pixel (no restriction), just clamp */
	*y = max(*y, 0);
	if (*y + *h > (int)epd->panel_h)
		*h = (int)epd->panel_h - *y;
}

/* =========================================================================
 * Double-buffer comparison
 *
 * Before sending a region to the display, check whether fb_vaddr actually
 * differs from screen_shadow.  Returns true if the region has changed.
 * Also shrinks the bounding box to the rows that actually differ (column
 * shrink is not performed to keep complexity low; tile tracking handles that).
 * ========================================================================= */
static bool epd_region_changed(struct epd_device *epd,
				int *rx, int *ry, int *rw, int *rh)
{
	const u8 *cur  = epd->fb_vaddr;
	const u8 *prev = epd->screen_shadow;
	u32 stride     = epd->fb_stride;
	int top = -1, bot = -1;
	int row;
	/* byte range for this region's columns */
	int byte0 = *rx / 8;
	int byte1 = (*rx + *rw - 1) / 8;

	for (row = 0; row < *rh; row++) {
		int src_row = *ry + row;
		size_t off  = (size_t)src_row * stride + byte0;
		size_t len  = (size_t)(byte1 - byte0 + 1);

		if (memcmp(cur + off, prev + off, len) != 0) {
			if (top < 0)
				top = row;
			bot = row;
		}
	}

	if (top < 0)
		return false;  /* nothing changed */

	/* Narrow the vertical extent to the changed rows */
	*ry += top;
	*rh  = bot - top + 1;
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
		memcpy(epd->screen_shadow + off, epd->fb_vaddr + off, len);
	}
}

/* =========================================================================
 * epd_do_refresh – core refresh logic (runs in work-queue context)
 *
 * Strategy:
 *   1. Snapshot and clear dirty state.
 *   2. If ≥ EPD_FULL_REFRESH_THRESHOLD % of tiles dirty → full INIT + GC16.
 *   3. Otherwise: A2 partial update on the bounding box of dirty tiles.
 *   4. After EPD_A2_GHOSTING_LIMIT consecutive A2 refreshes → INIT clear.
 * ========================================================================= */
void epd_do_refresh(struct epd_device *epd)
{
	int x, y, w, h;
	unsigned int pct;
	int ret;

	/* Do not talk to the hardware while suspended. */
	{
		unsigned long flags;

		spin_lock_irqsave(&epd->state_lock, flags);
		if (epd->suspended) {
			spin_unlock_irqrestore(&epd->state_lock, flags);
			return;
		}
		spin_unlock_irqrestore(&epd->state_lock, flags);
	}

	if (mutex_lock_interruptible(&epd->refresh_mutex))
		return;

	/* ---- Snapshot dirty state ---------------------------------------- */
	/*
	 * Each helper takes dirty.lock internally; do not hold it here.
	 * A brief TOCTOU window between reads and clear is harmless: any
	 * new dirty tiles added in that window will be caught on the next
	 * refresh cycle.  refresh_mutex prevents concurrent refreshes.
	 */

	pct = epd_dirty_percent(&epd->dirty, epd->panel_w, epd->panel_h);
	if (!epd_dirty_bbox_pixels(&epd->dirty, epd->panel_w, epd->panel_h,
				   &x, &y, &w, &h)) {
		/* Nothing to do */
		goto out_unlock;
	}
	epd_dirty_clear(&epd->dirty);

	/* ---- Full refresh ------------------------------------------------- */
	if (pct >= EPD_FULL_REFRESH_THRESHOLD) {
		dev_dbg(&epd->spi->dev, "full refresh (dirty=%u%%)\n", pct);

		/* Clear display (INIT waveform removes all ghosting) */
		ret = epd_full_clear(epd);
		if (ret) {
			dev_err(&epd->spi->dev, "full clear failed: %d\n", ret);
			goto out_unlock;
		}

		/* Wait for INIT waveform to finish before loading new pixels. */
		ret = epd_wait_display_ready(epd);
		if (ret) {
			dev_err(&epd->spi->dev, "display not ready before load: %d\n", ret);
			goto out_unlock;
		}

		/* Load entire framebuffer (1bpp→4bpp conversion inside) */
		ret = epd_load_image_1bpp(epd,
					  epd->fb_vaddr, epd->fb_stride,
					  0, 0, epd->panel_w, epd->panel_h);
		if (ret) {
			dev_err(&epd->spi->dev, "load image failed: %d\n", ret);
			goto out_unlock;
		}

		ret = epd_display_area(epd, 0, 0, epd->panel_w, epd->panel_h,
				       EPD_MODE_GC16);
		if (ret)
			dev_err(&epd->spi->dev, "display area failed: %d\n", ret);

		memcpy(epd->screen_shadow, epd->fb_vaddr, epd->fb_size);
		atomic_set(&epd->a2_count, 0);
		goto out_unlock;
	}

	/* ---- Partial refresh --------------------------------------------- */

	/* Ghosting recovery: after too many A2 passes do a silent INIT clear. */
	if (atomic_read(&epd->a2_count) >= EPD_A2_GHOSTING_LIMIT) {
		dev_dbg(&epd->spi->dev,
			"A2 ghost limit reached; doing INIT clear\n");
		ret = epd_full_clear(epd);
		if (ret) {
			dev_err(&epd->spi->dev,
				"ghost-recovery clear failed: %d\n", ret);
			goto out_unlock;
		}
		atomic_set(&epd->a2_count, 0);
		/*
		 * After INIT the display is blank (white).  We must reload the
		 * full current framebuffer so content is not lost.  Mark the
		 * entire frame as needing a GC16 re-draw, but only for the
		 * previously visible content in screen_shadow.
		 */
		/* Wait for ghosting-recovery INIT to finish before reloading. */
		ret = epd_wait_display_ready(epd);
		if (ret) {
			dev_err(&epd->spi->dev,
				"display not ready after ghost clear: %d\n", ret);
			goto out_unlock;
		}
		ret = epd_load_image_1bpp(epd,
					  epd->fb_vaddr, epd->fb_stride,
					  0, 0, epd->panel_w, epd->panel_h);
		if (ret) {
			dev_err(&epd->spi->dev,
				"post-clear reload failed: %d\n", ret);
			goto out_unlock;
		}
		ret = epd_display_area(epd, 0, 0, epd->panel_w, epd->panel_h,
				       EPD_MODE_GC16);
		if (ret)
			dev_err(&epd->spi->dev,
				"post-clear GC16 failed: %d\n", ret);
		memcpy(epd->screen_shadow, epd->fb_vaddr, epd->fb_size);
		goto out_unlock;
	}

	/* Align the bounding box to IT8951 4bpp requirements. */
	epd_align_region(epd, &x, &y, &w, &h);

	if (w <= 0 || h <= 0)
		goto out_unlock;

	/* Double-buffer check: skip if nothing actually changed. */
	if (!epd_region_changed(epd, &x, &y, &w, &h)) {
		dev_dbg(&epd->spi->dev,
			"dirty tiles at (%d,%d)+%dx%d unchanged vs shadow\n",
			x, y, w, h);
		goto out_unlock;
	}

	/* Ensure alignment again after row-shrink from epd_region_changed. */
	epd_align_region(epd, &x, &y, &w, &h);
	if (w <= 0 || h <= 0)
		goto out_unlock;

	dev_dbg(&epd->spi->dev,
		"partial A2 refresh (%d,%d)+%dx%d [dirty=%u%%]\n",
		x, y, w, h, pct);

	/* Wait for any prior waveform before writing new pixel data to DRAM. */
	ret = epd_wait_display_ready(epd);
	if (ret) {
		dev_err(&epd->spi->dev,
			"display not ready before partial load: %d\n", ret);
		goto out_unlock;
	}

	/* Load pixel data for the changed region (1bpp→4bpp inside). */
	ret = epd_load_image_1bpp(epd, epd->fb_vaddr, epd->fb_stride,
				   (u16)x, (u16)y, (u16)w, (u16)h);
	if (ret) {
		dev_err(&epd->spi->dev, "partial load failed: %d\n", ret);
		goto out_unlock;
	}

	/* Trigger display refresh on the hardware. */
	ret = epd_display_area(epd, x, y, w, h, epd->a2_mode);
	if (ret) {
		dev_err(&epd->spi->dev, "partial display failed: %d\n", ret);
		goto out_unlock;
	}

	/* Update the shadow to match what's now on screen. */
	epd_update_shadow(epd, x, y, w, h);
	atomic_inc(&epd->a2_count);

out_unlock:
	mutex_unlock(&epd->refresh_mutex);
}

/* =========================================================================
 * Work-queue handlers
 * ========================================================================= */

static void epd_normal_work_fn(struct work_struct *work)
{
	struct epd_device *epd =
		container_of(work, struct epd_device, normal_work.work);
	epd_do_refresh(epd);
}

static void epd_priority_work_fn(struct work_struct *work)
{
	struct epd_device *epd =
		container_of(work, struct epd_device, priority_work.work);
	epd_do_refresh(epd);
}

/* =========================================================================
 * Public API
 * ========================================================================= */

int epd_refresh_init(struct epd_device *epd)
{
	mutex_init(&epd->refresh_mutex);
	INIT_DELAYED_WORK(&epd->normal_work,   epd_normal_work_fn);
	INIT_DELAYED_WORK(&epd->priority_work, epd_priority_work_fn);
	atomic_set(&epd->a2_count, 0);
	return 0;
}

void epd_refresh_cleanup(struct epd_device *epd)
{
	cancel_delayed_work_sync(&epd->priority_work);
	cancel_delayed_work_sync(&epd->normal_work);
	mutex_destroy(&epd->refresh_mutex);
}

void epd_schedule_refresh(struct epd_device *epd, bool priority)
{
	if (priority) {
		/*
		 * Priority path: cursor or tiny-area update.  Cancel any
		 * pending normal refresh (it will be re-scheduled after this
		 * high-priority one completes) and submit with a short delay.
		 */
		mod_delayed_work(system_wq, &epd->priority_work,
				 msecs_to_jiffies(EPD_PRIORITY_DELAY_MS));
	} else {
		/*
		 * Normal path: coalesce rapid writes into one refresh cycle.
		 * mod_delayed_work resets the timer if already pending.
		 */
		mod_delayed_work(system_wq, &epd->normal_work,
				 msecs_to_jiffies(EPD_REFRESH_DELAY_MS));
	}
}
