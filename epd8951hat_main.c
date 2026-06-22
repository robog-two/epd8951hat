



#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/fb.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/spi/spi.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>

#include "epd8951hat.h"




static int  vcom_mv_param = EPD_DEFAULT_VCOM;
module_param_named(vcom, vcom_mv_param, int, 0644);
MODULE_PARM_DESC(vcom, "VCOM magnitude in millivolts (e.g. 2510 for -2.51 V)");

static bool enhance_drv_param;
module_param_named(enhance_driving, enhance_drv_param, bool, 0644);
MODULE_PARM_DESC(enhance_driving, "Enable enhanced driving capability (fixes blur on weak FPC)");

static int rotation_param;
module_param_named(rotation, rotation_param, int, 0644);
MODULE_PARM_DESC(rotation, "Panel rotation: 0=0°, 1=90°, 2=180°, 3=270°");

static bool mirror_x_param = true;
module_param_named(mirror_x, mirror_x_param, bool, 0644);
MODULE_PARM_DESC(mirror_x, "Horizontally mirror output (default on; required for the 10.3\" panel orientation)");

 
static int gpio_rst_num  = 17;
static int gpio_busy_num = 24;
module_param_named(gpio_rst,  gpio_rst_num,  int, 0444);
module_param_named(gpio_busy, gpio_busy_num, int, 0444);
MODULE_PARM_DESC(gpio_rst,  "RST  GPIO number (default 17, fallback if DT absent)");
MODULE_PARM_DESC(gpio_busy, "BUSY GPIO number (default 24, fallback if DT absent)");



static struct gpio_desc *epd_get_gpio(struct device *dev,
				      const char *dt_name, int fallback_num,
				      enum gpiod_flags flags)
{
	struct gpio_desc *gd;

	gd = devm_gpiod_get_optional(dev, dt_name, flags);
	if (IS_ERR(gd))
		return gd;
	if (gd)
		return gd;    

	 
	if (fallback_num < 0) {
		dev_err(dev, "GPIO '%s' not in DT and no fallback number\n",
			dt_name);
		return ERR_PTR(-ENOENT);
	}

	gd = gpio_to_desc(fallback_num);
	if (!gd) {
		dev_err(dev, "gpio_to_desc(%d) failed for '%s'\n",
			fallback_num, dt_name);
		return ERR_PTR(-EINVAL);
	}

	if (flags == GPIOD_IN) {
		if (gpiod_direction_input(gd)) {
			dev_err(dev, "cannot set direction for GPIO %d ('%s')\n",
				fallback_num, dt_name);
			return ERR_PTR(-EIO);
		}
	} else {
		if (gpiod_direction_output(gd, (flags == GPIOD_OUT_HIGH) ? 1 : 0)) {
			dev_err(dev, "cannot set direction for GPIO %d ('%s')\n",
				fallback_num, dt_name);
			return ERR_PTR(-EIO);
		}
	}

	return gd;
}






static int epd_fb_check_var(struct fb_var_screeninfo *var,
			     struct fb_info *info)
{
	struct epd_device *epd = info->par;

	if (var->bits_per_pixel != 1) {
		dev_dbg(&epd->spi->dev,
			"check_var: only 1bpp supported (got %u)\n",
			var->bits_per_pixel);
		return -EINVAL;
	}

	if (var->xres != epd->panel_w || var->yres != epd->panel_h) {
		dev_dbg(&epd->spi->dev,
			"check_var: resolution %ux%u unsupported (panel %ux%u)\n",
			var->xres, var->yres, epd->panel_w, epd->panel_h);
		return -EINVAL;
	}

	var->xres_virtual = var->xres;
	var->yres_virtual = var->yres;
	var->red    = (struct fb_bitfield){ .offset = 0, .length = 1 };
	var->green  = (struct fb_bitfield){ .offset = 0, .length = 1 };
	var->blue   = (struct fb_bitfield){ .offset = 0, .length = 1 };
	var->transp = (struct fb_bitfield){ .offset = 0, .length = 0 };
	return 0;
}



static int epd_fb_set_par(struct fb_info *info)
{
	return 0;
}



static int epd_fb_setcolreg(unsigned int regno, unsigned int red,
			     unsigned int green, unsigned int blue,
			     unsigned int transp, struct fb_info *info)
{
	 
	if (regno > 1)
		return -EINVAL;

	((u32 *)info->pseudo_palette)[regno] = regno;
	return 0;
}



static int epd_fb_blank(int blank_mode, struct fb_info *info)
{
	struct epd_device *epd = info->par;
	unsigned long flags;

	spin_lock_irqsave(&epd->state_lock, flags);

	switch (blank_mode) {
	case FB_BLANK_POWERDOWN:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_VSYNC_SUSPEND:
		if (!epd->suspended) {
			epd->suspended = true;
			spin_unlock_irqrestore(&epd->state_lock, flags);
			epd_hw_sleep(epd);
			return 0;
		}
		break;
	case FB_BLANK_UNBLANK:
		if (epd->suspended) {
			epd->suspended = false;
			spin_unlock_irqrestore(&epd->state_lock, flags);
			epd_hw_wakeup(epd);
			 
			epd_dirty_mark_all(&epd->dirty,
					   epd->panel_w, epd->panel_h);
			epd_schedule_refresh(epd, false);
			return 0;
		}
		break;
	default:
		break;
	}

	spin_unlock_irqrestore(&epd->state_lock, flags);
	return 0;
}



static ssize_t epd_fb_write(struct fb_info *info, const char __user *buf,
			    size_t count, loff_t *ppos)
{
	struct epd_device *epd = info->par;
	ssize_t ret;
	unsigned long offset = *ppos;
	int y0, y1;

	 
	ret = fb_sys_write(info, buf, count, ppos);
	if (ret <= 0)
		return ret;

	 
	y0 = (int)(offset / epd->fb_stride);
	y1 = (int)(((unsigned long)offset + (unsigned long)ret - 1) /
		   epd->fb_stride);

	y0 = max(y0, 0);
	y1 = min(y1, (int)epd->panel_h - 1);

	if (y0 <= y1)
		epd_dirty_mark(&epd->dirty,
			       0, y0, (int)epd->panel_w, y1 - y0 + 1,
			       epd->panel_w, epd->panel_h);

	epd_schedule_refresh(epd, false);
	return ret;
}



static void epd_fb_fillrect(struct fb_info *info,
			     const struct fb_fillrect *rect)
{
	struct epd_device *epd = info->par;

	sys_fillrect(info, rect);

	epd_dirty_mark(&epd->dirty,
		       (int)rect->dx, (int)rect->dy,
		       (int)rect->width, (int)rect->height,
		       (int)epd->panel_w, (int)epd->panel_h);

	epd_schedule_refresh(epd, false);
}



static void epd_fb_copyarea(struct fb_info *info,
			     const struct fb_copyarea *area)
{
	struct epd_device *epd = info->par;

	sys_copyarea(info, area);

	 
	epd_dirty_mark(&epd->dirty,
		       (int)area->dx, (int)area->dy,
		       (int)area->width, (int)area->height,
		       (int)epd->panel_w, (int)epd->panel_h);

	 
	epd_dirty_mark(&epd->dirty,
		       (int)area->sx, (int)area->sy,
		       (int)area->width, (int)area->height,
		       (int)epd->panel_w, (int)epd->panel_h);

	epd_schedule_refresh(epd, false);
}



static void epd_fb_imageblit(struct fb_info *info, const struct fb_image *image)
{
	struct epd_device *epd = info->par;
	bool priority;

	sys_imageblit(info, image);

	epd_dirty_mark(&epd->dirty,
		       (int)image->dx, (int)image->dy,
		       (int)image->width, (int)image->height,
		       (int)epd->panel_w, (int)epd->panel_h);

	priority = epd_is_cursor_update((int)image->width, (int)image->height);
	epd_schedule_refresh(epd, priority);
}



static int epd_fb_sync(struct fb_info *info)
{
	struct epd_device *epd = info->par;

	cancel_delayed_work_sync(&epd->priority_work);
	cancel_delayed_work_sync(&epd->normal_work);
	epd_do_refresh(epd);
	return 0;
}




static void epd_deferred_io(struct fb_info *info, struct list_head *pagereflist)
{
	struct epd_device *epd = info->par;
	struct fb_deferred_io_pageref *pageref;

	list_for_each_entry(pageref, pagereflist, list) {
		epd_dirty_mark_page(&epd->dirty,
				    pageref->offset,
				    epd->fb_stride,
				    (int)epd->panel_w,
				    (int)epd->panel_h);
	}

	epd_schedule_refresh(epd, false);
}




static const struct fb_ops epd_fb_ops = {
	.owner          = THIS_MODULE,
	.fb_read        = fb_sys_read,
	.fb_write       = epd_fb_write,
	.fb_check_var   = epd_fb_check_var,
	.fb_set_par     = epd_fb_set_par,
	.fb_setcolreg   = epd_fb_setcolreg,
	.fb_blank       = epd_fb_blank,
	.fb_fillrect    = epd_fb_fillrect,
	.fb_copyarea    = epd_fb_copyarea,
	.fb_imageblit   = epd_fb_imageblit,
	.fb_sync        = epd_fb_sync,
};




static int epd_probe(struct spi_device *spi)
{
	struct epd_device *epd;
	struct fb_info *info;
	struct fb_deferred_io *defio;
	int ret;

	 
	info = framebuffer_alloc(sizeof(struct epd_device), &spi->dev);
	if (!info)
		return -ENOMEM;

	epd = info->par;
	epd->spi     = spi;
	epd->fb_info = info;
	spin_lock_init(&epd->state_lock);

	 
	epd->vcom_mv         = (u16)clamp(vcom_mv_param, 0, 5000);
	epd->rotation        = clamp(rotation_param, 0, 3);
	epd->mirror_x        = mirror_x_param;
	epd->enhance_driving = enhance_drv_param;

	 
	epd->gpio_rst = epd_get_gpio(&spi->dev, "rst", gpio_rst_num,
				     GPIOD_OUT_HIGH);
	if (IS_ERR(epd->gpio_rst)) {
		ret = PTR_ERR(epd->gpio_rst);
		dev_err(&spi->dev, "cannot get RST GPIO: %d\n", ret);
		goto err_free_info;
	}

	epd->gpio_busy = epd_get_gpio(&spi->dev, "busy", gpio_busy_num,
				      GPIOD_IN);
	if (IS_ERR(epd->gpio_busy)) {
		ret = PTR_ERR(epd->gpio_busy);
		dev_err(&spi->dev, "cannot get BUSY GPIO: %d\n", ret);
		goto err_free_info;
	}

	 
	spi->mode          = SPI_MODE_0;
	spi->bits_per_word = 8;
	spi->max_speed_hz  = 8000000;    
	ret = spi_setup(spi);
	if (ret) {
		dev_err(&spi->dev, "spi_setup failed: %d\n", ret);
		goto err_free_info;
	}

	 
	epd->spi_buf_size = EPD_SPI_BUF_SIZE;
	epd->spi_buf = kzalloc(epd->spi_buf_size, GFP_KERNEL);
	if (!epd->spi_buf) {
		ret = -ENOMEM;
		goto err_free_info;
	}

	 
	ret = epd_hw_init(epd);
	if (ret) {
		dev_err(&spi->dev, "hardware init failed: %d\n", ret);
		goto err_free_spi_buf;
	}

	if (epd->enhance_driving) {
		ret = epd_enhance_driving(epd);
		if (ret)
			dev_warn(&spi->dev,
				 "enhance_driving failed (non-fatal): %d\n", ret);
	}

	 
	epd->fb_stride = DIV_ROUND_UP((u32)epd->panel_w, 8u);
	epd->fb_size   = (size_t)epd->fb_stride * epd->panel_h;

	epd->fb_vaddr = vzalloc(epd->fb_size);
	if (!epd->fb_vaddr) {
		ret = -ENOMEM;
		goto err_hw_sleep;
	}

	epd->screen_shadow = kzalloc(epd->fb_size, GFP_KERNEL);
	if (!epd->screen_shadow) {
		ret = -ENOMEM;
		goto err_free_fb;
	}

	 
	info->fbops        = &epd_fb_ops;
	info->screen_base  = (char __iomem *)epd->fb_vaddr;
	info->screen_size  = epd->fb_size;
	info->pseudo_palette = devm_kzalloc(&spi->dev, 32 * sizeof(u32),
					    GFP_KERNEL);
	if (!info->pseudo_palette) {
		ret = -ENOMEM;
		goto err_free_shadow;
	}

	 
	strscpy(info->fix.id, "epd8951hat", sizeof(info->fix.id));
	info->fix.type        = FB_TYPE_PACKED_PIXELS;
	info->fix.visual      = FB_VISUAL_MONO01;  
	info->fix.line_length = epd->fb_stride;
	info->fix.smem_start  = 0;                
	info->fix.smem_len    = epd->fb_size;
	info->fix.accel       = FB_ACCEL_NONE;

	 
	info->var.xres         = epd->panel_w;
	info->var.yres         = epd->panel_h;
	info->var.xres_virtual = epd->panel_w;
	info->var.yres_virtual = epd->panel_h;
	info->var.bits_per_pixel = 1;
	info->var.grayscale    = 1;
	info->var.red    = (struct fb_bitfield){ .offset = 0, .length = 1 };
	info->var.green  = (struct fb_bitfield){ .offset = 0, .length = 1 };
	info->var.blue   = (struct fb_bitfield){ .offset = 0, .length = 1 };
	info->var.transp = (struct fb_bitfield){ .offset = 0, .length = 0 };
	info->var.activate = FB_ACTIVATE_NOW;

	 
	defio = devm_kzalloc(&spi->dev, sizeof(*defio), GFP_KERNEL);
	if (!defio) {
		ret = -ENOMEM;
		goto err_free_shadow;
	}
	defio->delay       = msecs_to_jiffies(EPD_REFRESH_DELAY_MS);
	defio->deferred_io = epd_deferred_io;

	info->fbdefio = defio;
	fb_deferred_io_init(info);

	 
	epd_dirty_init(&epd->dirty);
	ret = epd_refresh_init(epd);
	if (ret)
		goto err_defio_cleanup;

	 
	ret = register_framebuffer(info);
	if (ret) {
		dev_err(&spi->dev, "register_framebuffer failed: %d\n", ret);
		goto err_refresh_cleanup;
	}

	spi_set_drvdata(spi, epd);

	 
	ret = epd_full_clear(epd);
	if (ret)
		dev_warn(&spi->dev, "initial clear failed (non-fatal): %d\n", ret);

	dev_info(&spi->dev,
		 "epd8951hat: %s %u×%u 1bpp (VCOM=%u mV, A2 mode %u)\n",
		 info->fix.id, epd->panel_w, epd->panel_h,
		 epd->vcom_mv, epd->a2_mode);

	return 0;

err_refresh_cleanup:
	epd_refresh_cleanup(epd);
err_defio_cleanup:
	fb_deferred_io_cleanup(info);
err_free_shadow:
	kfree(epd->screen_shadow);
err_free_fb:
	vfree(epd->fb_vaddr);
err_hw_sleep:
	epd_hw_sleep(epd);
err_free_spi_buf:
	kfree(epd->spi_buf);
err_free_info:
	framebuffer_release(info);
	return ret;
}

static void epd_remove(struct spi_device *spi)
{
	struct epd_device *epd = spi_get_drvdata(spi);
	struct fb_info *info   = epd->fb_info;

	 
	epd_refresh_cleanup(epd);

	 
	unregister_framebuffer(info);

	fb_deferred_io_cleanup(info);

	 
	epd_hw_sleep(epd);

	kfree(epd->screen_shadow);
	vfree(epd->fb_vaddr);
	kfree(epd->spi_buf);

	framebuffer_release(info);

	dev_info(&spi->dev, "epd8951hat: removed\n");
}




static const struct of_device_id epd_of_match[] = {
	{ .compatible = "waveshare,it8951" },
	{   }
};
MODULE_DEVICE_TABLE(of, epd_of_match);

static const struct spi_device_id epd_spi_id[] = {
	{ "it8951", 0 },
	{   }
};
MODULE_DEVICE_TABLE(spi, epd_spi_id);




#ifdef CONFIG_PM_SLEEP

static int epd_suspend(struct device *dev)
{
	struct spi_device  *spi = to_spi_device(dev);
	struct epd_device  *epd = spi_get_drvdata(spi);

	 
	epd_fb_sync(epd->fb_info);
	epd_hw_sleep(epd);
	return 0;
}

static int epd_resume(struct device *dev)
{
	struct spi_device  *spi = to_spi_device(dev);
	struct epd_device  *epd = spi_get_drvdata(spi);

	epd_hw_wakeup(epd);

	 
	epd_dirty_mark_all(&epd->dirty, epd->panel_w, epd->panel_h);
	epd_schedule_refresh(epd, false);
	return 0;
}

static SIMPLE_DEV_PM_OPS(epd_pm_ops, epd_suspend, epd_resume);
#define EPD_PM_OPS (&epd_pm_ops)

#else
#define EPD_PM_OPS NULL
#endif  




static struct spi_driver epd_spi_driver = {
	.driver = {
		.name           = "epd8951hat",
		.of_match_table = epd_of_match,
		.pm             = EPD_PM_OPS,
	},
	.id_table = epd_spi_id,
	.probe    = epd_probe,
	.remove   = epd_remove,
};

module_spi_driver(epd_spi_driver);

MODULE_AUTHOR("epd8951hat contributors");
MODULE_DESCRIPTION("IT8951 e-Paper HAT Linux framebuffer driver with tile-based partial refresh");
MODULE_LICENSE("GPL v2");
