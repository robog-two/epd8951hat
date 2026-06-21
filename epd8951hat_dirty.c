// SPDX-License-Identifier: GPL-2.0
/*
 * epd8951hat_dirty.c  –  Tile-based dirty-region tracker
 *
 * The screen is divided into tiles of EPD_TILE_SIZE × EPD_TILE_SIZE pixels
 * (16 × 16).  A DECLARE_BITMAP of EPD_TILE_TOTAL bits tracks which tiles
 * have been written since the last refresh.
 *
 * Tile index layout:
 *   tiles_per_row = DIV_ROUND_UP(panel_w, EPD_TILE_SIZE)
 *   tile_index    = tile_y * tiles_per_row + tile_x
 *
 * Thread safety:
 *   All mutations of tiles[], dirty_tile_count, bbox_*, and all_dirty are
 *   performed under d->lock (spinlock_t, irqsave variant) because the BUSY
 *   GPIO handler may fire from IRQ context and schedule a refresh that reads
 *   the dirty state.
 */

#include "epd8951hat.h"

#include <linux/bitmap.h>
#include <linux/kernel.h>
#include <linux/limits.h>
#include <linux/spinlock.h>
#include <linux/types.h>

/* =========================================================================
 * epd_dirty_init
 * ========================================================================= */

void epd_dirty_init(struct epd_dirty *d)
{
	spin_lock_init(&d->lock);
	bitmap_zero(d->tiles, EPD_TILE_TOTAL);

	/* Sentinel: nothing is dirty yet. */
	d->bbox_tx0 = INT_MAX;
	d->bbox_ty0 = INT_MAX;
	d->bbox_tx1 = -1;
	d->bbox_ty1 = -1;

	d->dirty_tile_count = 0;
	d->all_dirty        = false;
}

/* =========================================================================
 * epd_dirty_mark
 * ========================================================================= */

void epd_dirty_mark(struct epd_dirty *d, int x, int y, int w, int h,
		    int panel_w, int panel_h)
{
	unsigned long flags;
	int x1, y1;          /* exclusive right / bottom pixel edges      */
	int tx0, ty0;        /* first dirty tile (inclusive)              */
	int tx1, ty1;        /* last  dirty tile (inclusive)              */
	int tiles_per_row;
	int tx, ty;

	/* ----------------------------------------------------------------
	 * Clamp the incoming pixel rectangle to the panel bounds.
	 * After clamping: if the region is empty, bail out early.
	 * ---------------------------------------------------------------- */
	x  = max_t(int, x, 0);
	y  = max_t(int, y, 0);
	x1 = min_t(int, x + w, panel_w);
	y1 = min_t(int, y + h, panel_h);

	if (x1 <= x || y1 <= y)
		return;

	/* ----------------------------------------------------------------
	 * Compute the inclusive tile range that covers [x, x1) × [y, y1).
	 * ---------------------------------------------------------------- */
	tx0 = x  / EPD_TILE_SIZE;
	ty0 = y  / EPD_TILE_SIZE;
	tx1 = (x1 - 1) / EPD_TILE_SIZE;
	ty1 = (y1 - 1) / EPD_TILE_SIZE;

	tiles_per_row = DIV_ROUND_UP(panel_w, EPD_TILE_SIZE);

	spin_lock_irqsave(&d->lock, flags);

	for (ty = ty0; ty <= ty1; ty++) {
		for (tx = tx0; tx <= tx1; tx++) {
			int idx = ty * tiles_per_row + tx;

			/*
			 * test_and_set_bit returns the old value.
			 * Only count the bit if it was previously clear.
			 */
			if (!test_and_set_bit(idx, d->tiles))
				d->dirty_tile_count++;
		}
	}

	/* Expand bounding box to include the new tile range. */
	if (tx0 < d->bbox_tx0)
		d->bbox_tx0 = tx0;
	if (ty0 < d->bbox_ty0)
		d->bbox_ty0 = ty0;
	if (tx1 > d->bbox_tx1)
		d->bbox_tx1 = tx1;
	if (ty1 > d->bbox_ty1)
		d->bbox_ty1 = ty1;

	spin_unlock_irqrestore(&d->lock, flags);
}

/* =========================================================================
 * epd_dirty_mark_all
 * ========================================================================= */

void epd_dirty_mark_all(struct epd_dirty *d, int panel_w, int panel_h)
{
	unsigned long flags;
	int tiles_x = DIV_ROUND_UP(panel_w,  EPD_TILE_SIZE);
	int tiles_y = DIV_ROUND_UP(panel_h, EPD_TILE_SIZE);

	spin_lock_irqsave(&d->lock, flags);

	bitmap_fill(d->tiles, EPD_TILE_TOTAL);
	d->all_dirty        = true;
	d->dirty_tile_count = (unsigned int)(tiles_x * tiles_y);

	/* Bounding box covers every tile. */
	d->bbox_tx0 = 0;
	d->bbox_ty0 = 0;
	d->bbox_tx1 = tiles_x - 1;
	d->bbox_ty1 = tiles_y - 1;

	spin_unlock_irqrestore(&d->lock, flags);
}

/* =========================================================================
 * epd_dirty_mark_page
 *
 * Convert a 4 KiB framebuffer page to pixel rows and forward to
 * epd_dirty_mark().
 *
 * The framebuffer is 1 bpp, so stride = DIV_ROUND_UP(panel_w, 8) bytes/row.
 * We conservatively mark full rows (x=0, w=panel_w) for every row that falls
 * inside the page.
 * ========================================================================= */

void epd_dirty_mark_page(struct epd_dirty *d, unsigned long page_offset,
			  u32 stride, int panel_w, int panel_h)
{
	int y, y_end;

	if (stride == 0)
		return;

	y     = (int)(page_offset / stride);
	y_end = (int)((page_offset + PAGE_SIZE - 1) / stride);

	/* Clamp to valid row range. */
	if (y >= panel_h)
		return;
	y_end = min_t(int, y_end, panel_h - 1);

	epd_dirty_mark(d, 0, y, panel_w, y_end - y + 1, panel_w, panel_h);
}

/* =========================================================================
 * epd_dirty_clear
 * ========================================================================= */

void epd_dirty_clear(struct epd_dirty *d)
{
	unsigned long flags;

	spin_lock_irqsave(&d->lock, flags);

	bitmap_zero(d->tiles, EPD_TILE_TOTAL);

	d->bbox_tx0 = INT_MAX;
	d->bbox_ty0 = INT_MAX;
	d->bbox_tx1 = -1;
	d->bbox_ty1 = -1;

	d->dirty_tile_count = 0;
	d->all_dirty        = false;

	spin_unlock_irqrestore(&d->lock, flags);
}

/* =========================================================================
 * epd_dirty_percent
 *
 * Returns 0–100.  The lock is held for the snapshot to get a consistent
 * dirty_tile_count relative to the panel geometry.
 * ========================================================================= */

unsigned int epd_dirty_percent(const struct epd_dirty *d,
			       int panel_w, int panel_h)
{
	/*
	 * Cast away const for the spinlock: spin_lock_irqsave modifies the
	 * lock's internal state even when conceptually we are only reading the
	 * dirty counters.  This is the standard kernel pattern for const readers
	 * that still need a lock.
	 */
	struct epd_dirty *dw = (struct epd_dirty *)d;
	unsigned long flags;
	unsigned int count, total, result;

	spin_lock_irqsave(&dw->lock, flags);
	count = d->dirty_tile_count;
	spin_unlock_irqrestore(&dw->lock, flags);

	if (count == 0)
		return 0;

	total = (unsigned int)(DIV_ROUND_UP(panel_w, EPD_TILE_SIZE) *
			       DIV_ROUND_UP(panel_h, EPD_TILE_SIZE));

	if (total == 0)
		return 0;

	result = (count * 100u) / total;
	return min(result, 100u);
}

/* =========================================================================
 * epd_dirty_bbox_pixels
 *
 * Returns false if nothing is dirty (count == 0 or sentinel bbox).
 * ========================================================================= */

bool epd_dirty_bbox_pixels(const struct epd_dirty *d,
			   int panel_w, int panel_h,
			   int *x, int *y, int *w, int *h)
{
	struct epd_dirty *dw = (struct epd_dirty *)d;
	unsigned long flags;
	unsigned int count;
	int tx0, ty0, tx1, ty1;
	int px, py, pw, ph;

	spin_lock_irqsave(&dw->lock, flags);

	count = d->dirty_tile_count;
	tx0   = d->bbox_tx0;
	ty0   = d->bbox_ty0;
	tx1   = d->bbox_tx1;
	ty1   = d->bbox_ty1;

	spin_unlock_irqrestore(&dw->lock, flags);

	/* Nothing dirty or bbox still at sentinel values. */
	if (count == 0 || tx0 == INT_MAX || tx1 < 0)
		return false;

	/* Convert inclusive tile bbox to pixel coordinates. */
	px = tx0 * EPD_TILE_SIZE;
	py = ty0 * EPD_TILE_SIZE;
	pw = min_t(int, (tx1 + 1) * EPD_TILE_SIZE, panel_w) - px;
	ph = min_t(int, (ty1 + 1) * EPD_TILE_SIZE, panel_h) - py;

	if (pw <= 0 || ph <= 0)
		return false;

	*x = px;
	*y = py;
	*w = pw;
	*h = ph;

	return true;
}
