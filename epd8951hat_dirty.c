



#include "epd8951hat.h"

#include <linux/bitmap.h>
#include <linux/kernel.h>
#include <linux/limits.h>
#include <linux/spinlock.h>
#include <linux/types.h>




void epd_dirty_init(struct epd_dirty *d)
{
	spin_lock_init(&d->lock);
	bitmap_zero(d->tiles, EPD_TILE_TOTAL);

	 
	d->bbox_tx0 = INT_MAX;
	d->bbox_ty0 = INT_MAX;
	d->bbox_tx1 = -1;
	d->bbox_ty1 = -1;

	d->dirty_tile_count = 0;
	d->all_dirty        = false;
}




void epd_dirty_mark(struct epd_dirty *d, int x, int y, int w, int h,
		    int panel_w, int panel_h)
{
	unsigned long flags;
	int x1, y1;           
	int tx0, ty0;         
	int tx1, ty1;         
	int tiles_per_row;
	int tx, ty;

	

	x  = max_t(int, x, 0);
	y  = max_t(int, y, 0);
	x1 = min_t(int, x + w, panel_w);
	y1 = min_t(int, y + h, panel_h);

	if (x1 <= x || y1 <= y)
		return;

	

	tx0 = x  / EPD_TILE_SIZE;
	ty0 = y  / EPD_TILE_SIZE;
	tx1 = (x1 - 1) / EPD_TILE_SIZE;
	ty1 = (y1 - 1) / EPD_TILE_SIZE;

	tiles_per_row = DIV_ROUND_UP(panel_w, EPD_TILE_SIZE);

	spin_lock_irqsave(&d->lock, flags);

	for (ty = ty0; ty <= ty1; ty++) {
		for (tx = tx0; tx <= tx1; tx++) {
			int idx = ty * tiles_per_row + tx;

			

			if (!test_and_set_bit(idx, d->tiles))
				d->dirty_tile_count++;
		}
	}

	 
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




void epd_dirty_mark_all(struct epd_dirty *d, int panel_w, int panel_h)
{
	unsigned long flags;
	int tiles_x = DIV_ROUND_UP(panel_w,  EPD_TILE_SIZE);
	int tiles_y = DIV_ROUND_UP(panel_h, EPD_TILE_SIZE);

	spin_lock_irqsave(&d->lock, flags);

	bitmap_fill(d->tiles, EPD_TILE_TOTAL);
	d->all_dirty        = true;
	d->dirty_tile_count = (unsigned int)(tiles_x * tiles_y);

	 
	d->bbox_tx0 = 0;
	d->bbox_ty0 = 0;
	d->bbox_tx1 = tiles_x - 1;
	d->bbox_ty1 = tiles_y - 1;

	spin_unlock_irqrestore(&d->lock, flags);
}




void epd_dirty_mark_page(struct epd_dirty *d, unsigned long page_offset,
			  u32 stride, int panel_w, int panel_h)
{
	int y, y_end;

	if (stride == 0)
		return;

	y     = (int)(page_offset / stride);
	y_end = (int)((page_offset + PAGE_SIZE - 1) / stride);

	 
	if (y >= panel_h)
		return;
	y_end = min_t(int, y_end, panel_h - 1);

	epd_dirty_mark(d, 0, y, panel_w, y_end - y + 1, panel_w, panel_h);
}




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




unsigned int epd_dirty_percent(const struct epd_dirty *d,
			       int panel_w, int panel_h)
{
	

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

	 
	if (count == 0 || tx0 == INT_MAX || tx1 < 0)
		return false;

	 
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
