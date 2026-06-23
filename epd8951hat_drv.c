



#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio.h>
#include <linux/iosys-map.h>
#include <linux/kernel.h>
#include <linux/limits.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/spi/spi.h>
#include <linux/string.h>
#include <linux/vmalloc.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fbdev_shmem.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_mode_config.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_modes.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_rect.h>
#include <drm/drm_simple_kms_helper.h>

#include "epd8951hat.h"
#include "epd8951hat_pipeline.h"




static int  vcom_mv_param = EPD_DEFAULT_VCOM;
module_param_named(vcom, vcom_mv_param, int, 0644);
MODULE_PARM_DESC(vcom, "VCOM magnitude in millivolts (e.g. 2510 for -2.51 V)");

static bool enhance_drv_param;
module_param_named(enhance_driving, enhance_drv_param, bool, 0644);
MODULE_PARM_DESC(enhance_driving, "Enable enhanced driving capability");

static int rotation_param;
module_param_named(rotation, rotation_param, int, 0644);
MODULE_PARM_DESC(rotation, "Panel rotation: 0=0°, 1=90°, 2=180°, 3=270°");

static bool mirror_x_param = true;
module_param_named(mirror_x, mirror_x_param, bool, 0644);
MODULE_PARM_DESC(mirror_x, "Horizontally mirror output (default off; use if panel is physically mounted mirrored)");

static int idle_ms_param = EPD_IDLE_CLEAN_MS;
module_param_named(idle_ms, idle_ms_param, int, 0644);
MODULE_PARM_DESC(idle_ms, "Idle ms with no screen change before a single clean GC16 refresh (0 = disable)");

static int gpio_rst_num  = 17;
static int gpio_busy_num = 24;
module_param_named(gpio_rst,  gpio_rst_num,  int, 0444);
module_param_named(gpio_busy, gpio_busy_num, int, 0444);
MODULE_PARM_DESC(gpio_rst,  "RST GPIO number fallback (default 17)");
MODULE_PARM_DESC(gpio_busy, "BUSY GPIO number fallback (default 24)");




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
		dev_err(dev, "GPIO '%s' not in DT and no fallback number\n", dt_name);
		return ERR_PTR(-ENOENT);
	}

	gd = gpio_to_desc(fallback_num);
	if (!gd) {
		dev_err(dev, "gpio_to_desc(%d) failed for '%s'\n", fallback_num, dt_name);
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




static int epd_connector_get_modes(struct drm_connector *connector)
{
	struct epd_device *epd = container_of(connector, struct epd_device, connector);
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &epd->mode);
	if (!mode)
		return 0;

	drm_mode_probed_add(connector, mode);
	return 1;
}

static const struct drm_connector_helper_funcs epd_connector_helper_funcs = {
	.get_modes = epd_connector_get_modes,
};

static const struct drm_connector_funcs epd_connector_funcs = {
	.fill_modes             = drm_helper_probe_single_connector_modes,
	.reset                  = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state   = drm_atomic_helper_connector_destroy_state,
};




static void epd_dither_xrgb8888(struct epd_device *epd,
				  const void *src, u32 src_pitch,
				  int clip_y0, int clip_y1)
{
	epd_dither_xrgb8888_fn(epd->panel_w, epd->panel_h, epd->fb_stride,
				src, src_pitch, epd->mono_buf,
				clip_y0, clip_y1);
}




static void epd_refresh_work_fn(struct work_struct *work)
{
	struct epd_device *epd =
		container_of(work, struct epd_device, refresh_work);
	struct drm_framebuffer *fb;
	struct drm_gem_object *gem;
	struct drm_gem_shmem_object *shmem;
	struct iosys_map src_map;
	struct drm_rect damage;
	bool has_damage;
	bool caught_up;
	int rec_x0 = INT_MAX, rec_x1 = -1;
	int rec_y0 = INT_MAX, rec_y1 = -1;
	bool rec_full = false;
	unsigned long flags;
	int clip_x0, clip_x1;
	int clip_y0, clip_y1;
	int ret;

	spin_lock_irqsave(&epd->pending_lock, flags);
	fb                      = epd->pending_fb;
	epd->pending_fb         = NULL;
	damage                  = epd->pending_damage;
	has_damage              = epd->pending_has_damage;
	spin_unlock_irqrestore(&epd->pending_lock, flags);

	if (!fb)
		return;

	if (!epd->pipe_enabled) {
		drm_framebuffer_put(fb);
		return;
	}

	/* If no newer commit is queued we have caught up: this is the settle
	 * point, so reconcile the whole deferred-dirty region (and clear it) to
	 * converge the panel to the framebuffer. Otherwise stay responsive and
	 * render only the newest delta, leaving older regions to be reconciled
	 * once motion stops. */
	spin_lock_irqsave(&epd->pending_lock, flags);
	caught_up = (epd->pending_fb == NULL);
	if (caught_up) {
		rec_x0   = epd->acc_x0;
		rec_x1   = epd->acc_x1;
		rec_y0   = epd->acc_y0;
		rec_y1   = epd->acc_y1;
		rec_full = epd->acc_full;
		epd->acc_x0   = INT_MAX;
		epd->acc_x1   = -1;
		epd->acc_y0   = INT_MAX;
		epd->acc_y1   = -1;
		epd->acc_full = false;
	}
	spin_unlock_irqrestore(&epd->pending_lock, flags);

	if (caught_up) {
		if (rec_full || !has_damage) {
			clip_x0 = 0;
			clip_x1 = (int)epd->panel_w - 1;
			clip_y0 = 0;
			clip_y1 = (int)epd->panel_h - 1;
		} else {
			/* deferred rect unioned with the delta we just consumed */
			clip_x0 = max_t(int, min(rec_x0, (int)damage.x1), 0);
			clip_x1 = min_t(int, max(rec_x1, (int)damage.x2 - 1),
					(int)epd->panel_w - 1);
			clip_y0 = max_t(int, min(rec_y0, damage.y1), 0);
			clip_y1 = min_t(int, max(rec_y1, damage.y2 - 1),
					(int)epd->panel_h - 1);
		}
	} else if (has_damage) {
		clip_x0 = max_t(int, (int)damage.x1, 0);
		clip_x1 = min_t(int, (int)damage.x2 - 1, (int)epd->panel_w - 1);
		clip_y0 = max_t(int, damage.y1, 0);
		clip_y1 = min_t(int, damage.y2 - 1, (int)epd->panel_h - 1);
	} else {
		clip_x0 = 0;
		clip_x1 = (int)epd->panel_w - 1;
		clip_y0 = 0;
		clip_y1 = (int)epd->panel_h - 1;
	}

	if (clip_x0 > clip_x1 || clip_y0 > clip_y1) {
		drm_framebuffer_put(fb);
		return;
	}

	gem   = drm_gem_fb_get_obj(fb, 0);
	shmem = to_drm_gem_shmem_obj(gem);

	ret = drm_gem_shmem_vmap(shmem, &src_map);
	if (ret) {
		drm_err(&epd->drm, "vmap failed: %d\n", ret);
		drm_framebuffer_put(fb);
		return;
	}

	epd_dither_xrgb8888(epd, src_map.vaddr, fb->pitches[0], clip_y0, clip_y1);

	drm_gem_shmem_vunmap(shmem, &src_map);
	drm_framebuffer_put(fb);

	epd_do_refresh(epd, clip_x0, clip_x1, clip_y0, clip_y1);
}




static void epd_idle_work_fn(struct work_struct *work)
{
	struct epd_device *epd =
		container_of(work, struct epd_device, idle_work.work);
	int ret;

	/* Fires once the screen has been static for idle_ms. Re-armed by every
	 * epd_pipe_update, so this only runs after activity has fully stopped. */
	if (!epd->pipe_enabled || epd->suspended)
		return;

	mutex_lock(&epd->refresh_mutex);
	ret = epd_clean_refresh_locked(epd, EPD_CLEAN_MODE);
	mutex_unlock(&epd->refresh_mutex);

	if (ret)
		drm_warn(&epd->drm, "idle clean refresh failed: %d\n", ret);
}




static int epd_pipe_check(struct drm_simple_display_pipe *pipe,
			   struct drm_plane_state *plane_state,
			   struct drm_crtc_state *crtc_state)
{
	 
	crtc_state->no_vblank = true;
	return 0;
}

static void epd_pipe_enable(struct drm_simple_display_pipe *pipe,
			     struct drm_crtc_state *crtc_state,
			     struct drm_plane_state *plane_state)
{
	struct epd_device *epd = to_epd(pipe->crtc.dev);
	unsigned long flags;

	if (epd->suspended) {
		epd->suspended = false;
		epd_hw_wakeup(epd);
		epd_full_clear(epd);
	}

	epd->pipe_enabled = true;

	if (plane_state->fb) {
		spin_lock_irqsave(&epd->pending_lock, flags);
		if (epd->pending_fb)
			drm_framebuffer_put(epd->pending_fb);
		drm_framebuffer_get(plane_state->fb);
		epd->pending_fb          = plane_state->fb;
		epd->pending_has_damage  = false; /* force full-screen refresh on enable */
		spin_unlock_irqrestore(&epd->pending_lock, flags);

		schedule_work(&epd->refresh_work);
	}
}

static void epd_pipe_disable(struct drm_simple_display_pipe *pipe)
{
	struct epd_device *epd = to_epd(pipe->crtc.dev);

	epd->pipe_enabled = false;
	cancel_work_sync(&epd->refresh_work);
	cancel_delayed_work_sync(&epd->idle_work);

	 
	{
		unsigned long flags;
		struct drm_framebuffer *fb;

		spin_lock_irqsave(&epd->pending_lock, flags);
		fb = epd->pending_fb;
		epd->pending_fb = NULL;
		spin_unlock_irqrestore(&epd->pending_lock, flags);

		if (fb)
			drm_framebuffer_put(fb);
	}

	epd->suspended = true;
	epd_hw_sleep(epd);
}

static void epd_pipe_update(struct drm_simple_display_pipe *pipe,
			     struct drm_plane_state *old_state)
{
	struct epd_device *epd = to_epd(pipe->crtc.dev);
	struct drm_plane_state *state = pipe->plane.state;
	struct drm_rect damage;
	bool has_damage;
	unsigned long flags;

	if (!pipe->crtc.state->active || !state->fb)
		return;

	has_damage = drm_atomic_helper_damage_merged(old_state, state, &damage);

	spin_lock_irqsave(&epd->pending_lock, flags);
	if (epd->pending_fb)
		drm_framebuffer_put(epd->pending_fb);
	drm_framebuffer_get(state->fb);
	epd->pending_fb         = state->fb;
	epd->pending_damage     = damage;        /* newest delta: rendered first */
	epd->pending_has_damage = has_damage;

	/* Accumulate every commit into the deferred-dirty rect so coalesced
	 * intermediate damage is never forgotten; the worker reconciles it once
	 * it catches up. */
	if (!has_damage) {
		epd->acc_full = true;
	} else {
		epd->acc_x0 = min(epd->acc_x0, (int)damage.x1);
		epd->acc_x1 = max(epd->acc_x1, (int)damage.x2 - 1);
		epd->acc_y0 = min(epd->acc_y0, damage.y1);
		epd->acc_y1 = max(epd->acc_y1, damage.y2 - 1);
	}
	spin_unlock_irqrestore(&epd->pending_lock, flags);

	schedule_work(&epd->refresh_work);

	/* (Re)arm the idle clean: it only fires once updates stop arriving. */
	if (idle_ms_param > 0)
		mod_delayed_work(system_wq, &epd->idle_work,
				 msecs_to_jiffies(idle_ms_param));
}

static const struct drm_simple_display_pipe_funcs epd_pipe_funcs = {
	.check   = epd_pipe_check,
	.enable  = epd_pipe_enable,
	.disable = epd_pipe_disable,
	.update  = epd_pipe_update,
};




static const struct drm_mode_config_funcs epd_mode_config_funcs = {
	.fb_create     = drm_gem_fb_create_with_dirty,
	.atomic_check  = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

DEFINE_DRM_GEM_FOPS(epd_fops);

static const struct drm_driver epd_drm_driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops            = &epd_fops,
	DRM_GEM_SHMEM_DRIVER_OPS,
	.name    = "epd8951hat",
	.desc    = "IT8951 e-Paper HAT DRM driver",
	.date    = "20250101",
	.major   = 1,
	.minor   = 0,
};




static int epd_probe(struct spi_device *spi)
{
	struct epd_device *epd;
	static const uint32_t epd_formats[] = { DRM_FORMAT_XRGB8888 };
	int ret;

	 
	epd = devm_drm_dev_alloc(&spi->dev, &epd_drm_driver,
				  struct epd_device, drm);
	if (IS_ERR(epd))
		return PTR_ERR(epd);

	epd->spi = spi;
	spi_set_drvdata(spi, epd);

	spin_lock_init(&epd->pending_lock);
	mutex_init(&epd->refresh_mutex);
	INIT_WORK(&epd->refresh_work, epd_refresh_work_fn);
	INIT_DELAYED_WORK(&epd->idle_work, epd_idle_work_fn);
	epd->acc_x0   = INT_MAX;
	epd->acc_x1   = -1;
	epd->acc_y0   = INT_MAX;
	epd->acc_y1   = -1;
	epd->acc_full = false;

	 
	epd->vcom_mv         = (u16)clamp(vcom_mv_param, 0, 5000);
	epd->rotation        = clamp(rotation_param, 0, 3);
	epd->mirror_x        = mirror_x_param;
	epd->enhance_driving = enhance_drv_param;

	 
	epd->gpio_rst = epd_get_gpio(&spi->dev, "rst", gpio_rst_num,
				     GPIOD_OUT_HIGH);
	if (IS_ERR(epd->gpio_rst)) {
		ret = PTR_ERR(epd->gpio_rst);
		dev_err(&spi->dev, "cannot get RST GPIO: %d\n", ret);
		goto err_destroy_mutex;
	}

	epd->gpio_busy = epd_get_gpio(&spi->dev, "busy", gpio_busy_num,
				       GPIOD_IN);
	if (IS_ERR(epd->gpio_busy)) {
		ret = PTR_ERR(epd->gpio_busy);
		dev_err(&spi->dev, "cannot get BUSY GPIO: %d\n", ret);
		goto err_destroy_mutex;
	}

	 
	spi->mode          = SPI_MODE_0;
	spi->bits_per_word = 8;
	spi->max_speed_hz  = 12000000;   
	ret = spi_setup(spi);
	if (ret) {
		dev_err(&spi->dev, "spi_setup failed: %d\n", ret);
		goto err_destroy_mutex;
	}

	 
	epd->spi_buf_size = EPD_SPI_BUF_SIZE;
	epd->spi_buf = kzalloc(epd->spi_buf_size, GFP_KERNEL);
	if (!epd->spi_buf) {
		ret = -ENOMEM;
		goto err_destroy_mutex;
	}

	 
	ret = epd_hw_init(epd);
	if (ret) {
		dev_err(&spi->dev, "hardware init failed: %d\n", ret);
		goto err_free_spi_buf;
	}

	if (epd->enhance_driving) {
		ret = epd_enhance_driving(epd);
		if (ret)
			dev_warn(&spi->dev, "enhance_driving failed (non-fatal): %d\n", ret);
	}

	 
	epd->fb_stride = DIV_ROUND_UP((u32)epd->panel_w, 8u);
	epd->fb_size   = (size_t)epd->fb_stride * epd->panel_h;

	epd->mono_buf = vzalloc(epd->fb_size);
	if (!epd->mono_buf) {
		ret = -ENOMEM;
		goto err_hw_sleep;
	}

	epd->flip_buf = vzalloc(epd->fb_size);
	if (!epd->flip_buf) {
		ret = -ENOMEM;
		goto err_free_mono;
	}

	 
	ret = drmm_mode_config_init(&epd->drm);
	if (ret)
		goto err_free_flip;

	epd->drm.mode_config.min_width  = 0;
	epd->drm.mode_config.max_width  = EPD_MAX_WIDTH;
	epd->drm.mode_config.min_height = 0;
	epd->drm.mode_config.max_height = EPD_MAX_HEIGHT;
	epd->drm.mode_config.funcs      = &epd_mode_config_funcs;

	 
	memset(&epd->mode, 0, sizeof(epd->mode));
	epd->mode.type        = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	

	epd->mode.clock       = EPD_NOMINAL_VREFRESH_HZ *
				 (u32)epd->panel_w * epd->panel_h / 1000;
	epd->mode.hdisplay    = epd->panel_w;
	epd->mode.hsync_start = epd->panel_w;
	epd->mode.hsync_end   = epd->panel_w;
	epd->mode.htotal      = epd->panel_w;
	epd->mode.vdisplay    = epd->panel_h;
	epd->mode.vsync_start = epd->panel_h;
	epd->mode.vsync_end   = epd->panel_h;
	epd->mode.vtotal      = epd->panel_h;
	drm_mode_set_name(&epd->mode);

	 
	ret = drmm_connector_init(&epd->drm, &epd->connector,
				   &epd_connector_funcs,
				   DRM_MODE_CONNECTOR_SPI, NULL);
	if (ret) {
		dev_err(&spi->dev, "connector init failed: %d\n", ret);
		goto err_free_flip;
	}
	drm_connector_helper_add(&epd->connector, &epd_connector_helper_funcs);

	 
	ret = drm_simple_display_pipe_init(&epd->drm, &epd->pipe,
					   &epd_pipe_funcs,
					   epd_formats, ARRAY_SIZE(epd_formats),
					   NULL, &epd->connector);
	if (ret) {
		dev_err(&spi->dev, "pipe init failed: %d\n", ret);
		goto err_free_flip;
	}

	drm_plane_enable_fb_damage_clips(&epd->pipe.plane);

	drm_mode_config_reset(&epd->drm);

	 
	ret = drm_dev_register(&epd->drm, 0);
	if (ret) {
		dev_err(&spi->dev, "drm_dev_register failed: %d\n", ret);
		goto err_free_flip;
	}

	 
	drm_fbdev_shmem_setup(&epd->drm, 0);

	 
	ret = epd_full_clear(epd);
	if (ret)
		dev_warn(&spi->dev, "initial clear failed (non-fatal): %d\n", ret);

	dev_info(&spi->dev,
		 "epd8951hat: %u×%u DRM panel ready (VCOM=%u mV, A2 mode %u)\n",
		 epd->panel_w, epd->panel_h, epd->vcom_mv, epd->a2_mode);

	return 0;

err_free_flip:
	vfree(epd->flip_buf);
err_free_mono:
	vfree(epd->mono_buf);
err_hw_sleep:
	epd_hw_sleep(epd);
err_free_spi_buf:
	kfree(epd->spi_buf);
err_destroy_mutex:
	mutex_destroy(&epd->refresh_mutex);
	return ret;
}

static void epd_remove(struct spi_device *spi)
{
	struct epd_device *epd = spi_get_drvdata(spi);
	struct drm_framebuffer *pending_fb;
	unsigned long flags;

	drm_dev_unregister(&epd->drm);

	 
	cancel_work_sync(&epd->refresh_work);
	cancel_delayed_work_sync(&epd->idle_work);

	spin_lock_irqsave(&epd->pending_lock, flags);
	pending_fb      = epd->pending_fb;
	epd->pending_fb = NULL;
	spin_unlock_irqrestore(&epd->pending_lock, flags);

	if (pending_fb)
		drm_framebuffer_put(pending_fb);

	epd_hw_sleep(epd);

	vfree(epd->flip_buf);
	vfree(epd->mono_buf);
	kfree(epd->spi_buf);
	mutex_destroy(&epd->refresh_mutex);

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

static int epd_pm_suspend(struct device *dev)
{
	struct epd_device *epd = spi_get_drvdata(to_spi_device(dev));

	 
	cancel_work_sync(&epd->refresh_work);
	cancel_delayed_work_sync(&epd->idle_work);
	epd_hw_sleep(epd);
	return 0;
}

static int epd_pm_resume(struct device *dev)
{
	struct epd_device *epd = spi_get_drvdata(to_spi_device(dev));

	epd_hw_wakeup(epd);
	

	return 0;
}

static SIMPLE_DEV_PM_OPS(epd_pm_ops, epd_pm_suspend, epd_pm_resume);
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
MODULE_DESCRIPTION("IT8951 e-Paper HAT DRM/KMS driver with A2/GC16 partial refresh");
MODULE_LICENSE("GPL v2");
