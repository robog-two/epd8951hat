// SPDX-License-Identifier: GPL-2.0
/*
 * epd8951hat_drv.c  –  IT8951 e-Paper HAT DRM/KMS driver
 *
 * Registers a DRM device (/dev/dri/cardN) over a SPI-connected Waveshare
 * IT8951 e-Paper controller, using the simple display pipe + GEM SHMEM stack.
 *
 * Architecture:
 *   Userspace commits XRGB8888 framebuffers via the atomic KMS interface.
 *   The simple-pipe .update() callback merges the damage rectangle and
 *   schedules a workqueue item.  The worker converts XRGB8888→1bpp via
 *   drm_fb_xrgb8888_to_mono() into epd->mono_buf, then calls epd_do_refresh()
 *   which handles A2/GC16/INIT waveform selection and the IT8951 SPI upload.
 *
 *   drm_fbdev_shmem_setup() provides /dev/fb0 + fbcon emulation at no cost.
 *
 * Device-tree properties on the SPI child node:
 *   rst-gpios  = <&gpio 17 GPIO_ACTIVE_HIGH>;
 *   busy-gpios = <&gpio 24 GPIO_ACTIVE_HIGH>;
 *   reg        = <0>;   // CS0, driven by SPI core
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio.h>
#include <linux/iosys-map.h>
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
#include <linux/vmalloc.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fbdev_shmem.h>
#include <drm/drm_format_helper.h>
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

/* =========================================================================
 * Module parameters
 * ========================================================================= */

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
MODULE_PARM_DESC(mirror_x, "Horizontally mirror output (default on for 10.3\" panel)");

static int gpio_rst_num  = 17;
static int gpio_busy_num = 24;
module_param_named(gpio_rst,  gpio_rst_num,  int, 0444);
module_param_named(gpio_busy, gpio_busy_num, int, 0444);
MODULE_PARM_DESC(gpio_rst,  "RST GPIO number fallback (default 17)");
MODULE_PARM_DESC(gpio_busy, "BUSY GPIO number fallback (default 24)");

/* =========================================================================
 * GPIO acquisition helper
 * ========================================================================= */

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

/* =========================================================================
 * DRM connector
 * ========================================================================= */

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

/* =========================================================================
 * Refresh worker
 *
 * Runs in process context; handles the actual XRGB8888→1bpp conversion
 * and slow SPI upload off the atomic commit thread.
 * ========================================================================= */

static void epd_refresh_work_fn(struct work_struct *work)
{
	struct epd_device *epd =
		container_of(work, struct epd_device, refresh_work);
	struct drm_framebuffer *fb;
	struct drm_rect rect;
	struct drm_rect deferred;
	bool has_deferred;
	struct drm_gem_object *gem;
	struct drm_gem_shmem_object *shmem;
	struct iosys_map src_map;
	struct iosys_map dst_map;
	struct drm_format_conv_state conv_state = DRM_FORMAT_CONV_STATE_INIT;
	struct drm_rect full_clip;
	unsigned int dst_pitch;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&epd->pending_lock, flags);
	fb            = epd->pending_fb;
	rect          = epd->pending_rect;
	has_deferred  = epd->has_deferred;
	deferred      = epd->deferred_rect;
	epd->pending_fb   = NULL;
	epd->has_deferred = false;
	spin_unlock_irqrestore(&epd->pending_lock, flags);

	if (!fb)
		return;

	if (!epd->pipe_enabled) {
		drm_framebuffer_put(fb);
		return;
	}

	/*
	 * Convert the entire XRGB8888 framebuffer to 1bpp in mono_buf.
	 * We convert the full panel rather than just the damage clip so that
	 * epd_region_changed()'s shadow comparison operates on consistent data
	 * and epd_load_image_1bpp() can safely read the full panel if the
	 * refresh path decides on a full-screen update.
	 */
	gem  = drm_gem_fb_get_obj(fb, 0);
	shmem = to_drm_gem_shmem_obj(gem);

	ret = drm_gem_shmem_vmap(shmem, &src_map);
	if (ret) {
		drm_err(&epd->drm, "vmap failed: %d\n", ret);
		goto out_put_fb;
	}

	full_clip.x1 = 0;
	full_clip.y1 = 0;
	full_clip.x2 = epd->panel_w;
	full_clip.y2 = epd->panel_h;

	dst_pitch = epd->fb_stride;
	iosys_map_set_vaddr(&dst_map, epd->mono_buf);
	drm_fb_xrgb8888_to_mono(&dst_map, &dst_pitch, &src_map, fb,
				 &full_clip, &conv_state);

	drm_gem_shmem_vunmap(shmem, &src_map);
	drm_format_conv_state_release(&conv_state);

	drm_framebuffer_put(fb);

	/* Process the latest damage first ("prioritize now") */
	epd_do_refresh(epd, &rect);
	/* Then clean up any areas displaced by the latest update */
	if (has_deferred)
		epd_do_refresh(epd, &deferred);
	return;

out_put_fb:
	drm_format_conv_state_release(&conv_state);
	drm_framebuffer_put(fb);
}

/* =========================================================================
 * Simple display pipe callbacks
 * ========================================================================= */

static int epd_pipe_check(struct drm_simple_display_pipe *pipe,
			   struct drm_plane_state *plane_state,
			   struct drm_crtc_state *crtc_state)
{
	/* No hardware vblank; DRM core will synthesise the event. */
	crtc_state->no_vblank = true;
	return 0;
}

static void epd_pipe_enable(struct drm_simple_display_pipe *pipe,
			     struct drm_crtc_state *crtc_state,
			     struct drm_plane_state *plane_state)
{
	struct epd_device *epd = to_epd(pipe->crtc.dev);

	if (epd->suspended) {
		epd->suspended = false;
		epd_hw_wakeup(epd);
		/* Clear any ghosting left from before the sleep */
		epd_full_clear(epd);
	}

	epd->pipe_enabled = true;

	/*
	 * If a framebuffer is already attached (e.g. fbdev emulation wrote to
	 * it before the first modeset), schedule an immediate full-panel draw
	 * so the display shows current content on enable.
	 */
	if (plane_state->fb) {
		struct drm_rect full = {
			.x1 = 0, .y1 = 0,
			.x2 = epd->panel_w, .y2 = epd->panel_h,
		};
		unsigned long flags;

		spin_lock_irqsave(&epd->pending_lock, flags);
		if (epd->pending_fb)
			drm_framebuffer_put(epd->pending_fb);
		drm_framebuffer_get(plane_state->fb);
		epd->pending_fb   = plane_state->fb;
		epd->pending_rect = full;
		epd->has_deferred = false;
		spin_unlock_irqrestore(&epd->pending_lock, flags);

		schedule_work(&epd->refresh_work);
	}
}

static void epd_pipe_disable(struct drm_simple_display_pipe *pipe)
{
	struct epd_device *epd = to_epd(pipe->crtc.dev);

	epd->pipe_enabled = false;
	cancel_work_sync(&epd->refresh_work);

	/* Drop any pending framebuffer reference */
	{
		unsigned long flags;
		struct drm_framebuffer *fb;

		spin_lock_irqsave(&epd->pending_lock, flags);
		fb = epd->pending_fb;
		epd->pending_fb   = NULL;
		epd->has_deferred = false;
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
	struct drm_rect rect;
	unsigned long flags;

	if (!pipe->crtc.state->active)
		return;

	if (!drm_atomic_helper_damage_merged(old_state, state, &rect))
		return;

	spin_lock_irqsave(&epd->pending_lock, flags);
	if (epd->pending_fb) {
		/*
		 * Latest wins: the old pending rect is displaced by the newer
		 * damage.  Accumulate it into deferred_rect so the worker can
		 * clean up the stale screen region after finishing the latest
		 * update (ghost-cleanup pass).
		 */
		if (epd->has_deferred) {
			epd->deferred_rect.x1 = min(epd->deferred_rect.x1, epd->pending_rect.x1);
			epd->deferred_rect.y1 = min(epd->deferred_rect.y1, epd->pending_rect.y1);
			epd->deferred_rect.x2 = max(epd->deferred_rect.x2, epd->pending_rect.x2);
			epd->deferred_rect.y2 = max(epd->deferred_rect.y2, epd->pending_rect.y2);
		} else {
			epd->deferred_rect = epd->pending_rect;
			epd->has_deferred  = true;
		}
		drm_framebuffer_put(epd->pending_fb);
	}
	drm_framebuffer_get(state->fb);
	epd->pending_fb   = state->fb;
	epd->pending_rect = rect;
	spin_unlock_irqrestore(&epd->pending_lock, flags);

	schedule_work(&epd->refresh_work);
}

static const struct drm_simple_display_pipe_funcs epd_pipe_funcs = {
	.check   = epd_pipe_check,
	.enable  = epd_pipe_enable,
	.disable = epd_pipe_disable,
	.update  = epd_pipe_update,
};

/* =========================================================================
 * DRM mode config and driver
 * ========================================================================= */

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

/* =========================================================================
 * SPI driver probe / remove
 * ========================================================================= */

static int epd_probe(struct spi_device *spi)
{
	struct epd_device *epd;
	static const uint32_t epd_formats[] = { DRM_FORMAT_XRGB8888 };
	int ret;

	/* ---- Allocate DRM device with epd_device embedded ---------------- */
	epd = devm_drm_dev_alloc(&spi->dev, &epd_drm_driver,
				  struct epd_device, drm);
	if (IS_ERR(epd))
		return PTR_ERR(epd);

	epd->spi = spi;
	spi_set_drvdata(spi, epd);

	spin_lock_init(&epd->pending_lock);
	mutex_init(&epd->refresh_mutex);
	atomic_set(&epd->a2_count, 0);
	INIT_WORK(&epd->refresh_work, epd_refresh_work_fn);

	/* Apply module params */
	epd->vcom_mv         = (u16)clamp(vcom_mv_param, 0, 5000);
	epd->rotation        = clamp(rotation_param, 0, 3);
	epd->mirror_x        = mirror_x_param;
	epd->enhance_driving = enhance_drv_param;

	/* ---- GPIO setup --------------------------------------------------- */
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

	/* ---- Configure SPI bus -------------------------------------------- */
	spi->mode          = SPI_MODE_0;
	spi->bits_per_word = 8;
	spi->max_speed_hz  = 12000000;  /* 12 MHz; matches Waveshare reference (12.5 MHz) */
	ret = spi_setup(spi);
	if (ret) {
		dev_err(&spi->dev, "spi_setup failed: %d\n", ret);
		goto err_destroy_mutex;
	}

	/* ---- SPI scratch buffer (DMA-capable) ----------------------------- */
	epd->spi_buf_size = EPD_SPI_BUF_SIZE;
	epd->spi_buf = kzalloc(epd->spi_buf_size, GFP_KERNEL);
	if (!epd->spi_buf) {
		ret = -ENOMEM;
		goto err_destroy_mutex;
	}

	/* ---- Initialise IT8951 hardware ----------------------------------- */
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

	/* ---- Allocate 1bpp scratch and shadow buffers -------------------- */
	epd->fb_stride = DIV_ROUND_UP((u32)epd->panel_w, 8u);
	epd->fb_size   = (size_t)epd->fb_stride * epd->panel_h;

	epd->mono_buf = vzalloc(epd->fb_size);
	if (!epd->mono_buf) {
		ret = -ENOMEM;
		goto err_hw_sleep;
	}

	epd->screen_shadow = kzalloc(epd->fb_size, GFP_KERNEL);
	if (!epd->screen_shadow) {
		ret = -ENOMEM;
		goto err_free_mono;
	}

	/* ---- DRM mode config --------------------------------------------- */
	ret = drmm_mode_config_init(&epd->drm);
	if (ret)
		goto err_free_shadow;

	epd->drm.mode_config.min_width  = 0;
	epd->drm.mode_config.max_width  = EPD_MAX_WIDTH;
	epd->drm.mode_config.min_height = 0;
	epd->drm.mode_config.max_height = EPD_MAX_HEIGHT;
	epd->drm.mode_config.funcs      = &epd_mode_config_funcs;

	/* ---- Build fixed display mode from discovered panel size --------- */
	memset(&epd->mode, 0, sizeof(epd->mode));
	epd->mode.type        = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	/* clock (kHz) = vrefresh * htotal * vtotal / 1000.  E-paper has no real
	 * pixel clock; pick EPD_NOMINAL_VREFRESH_HZ so drm_mode_vrefresh() yields
	 * a non-zero value instead of rounding to 0. */
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

	/* ---- Connector ---------------------------------------------------- */
	ret = drmm_connector_init(&epd->drm, &epd->connector,
				   &epd_connector_funcs,
				   DRM_MODE_CONNECTOR_SPI, NULL);
	if (ret) {
		dev_err(&spi->dev, "connector init failed: %d\n", ret);
		goto err_free_shadow;
	}
	drm_connector_helper_add(&epd->connector, &epd_connector_helper_funcs);

	/* ---- Simple display pipe ----------------------------------------- */
	ret = drm_simple_display_pipe_init(&epd->drm, &epd->pipe,
					   &epd_pipe_funcs,
					   epd_formats, ARRAY_SIZE(epd_formats),
					   NULL, &epd->connector);
	if (ret) {
		dev_err(&spi->dev, "pipe init failed: %d\n", ret);
		goto err_free_shadow;
	}

	drm_mode_config_reset(&epd->drm);

	/* ---- Register DRM device ----------------------------------------- */
	ret = drm_dev_register(&epd->drm, 0);
	if (ret) {
		dev_err(&spi->dev, "drm_dev_register failed: %d\n", ret);
		goto err_free_shadow;
	}

	/* ---- fbdev emulation (provides /dev/fb0 + fbcon for free) -------- */
	drm_fbdev_shmem_setup(&epd->drm, 0);

	/* ---- Initial display clear --------------------------------------- */
	ret = epd_full_clear(epd);
	if (ret)
		dev_warn(&spi->dev, "initial clear failed (non-fatal): %d\n", ret);

	dev_info(&spi->dev,
		 "epd8951hat: %u×%u DRM panel ready (VCOM=%u mV, A2 mode %u)\n",
		 epd->panel_w, epd->panel_h, epd->vcom_mv, epd->a2_mode);

	return 0;

err_free_shadow:
	kfree(epd->screen_shadow);
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

	/* Stop the refresh worker and release any held FB reference */
	cancel_work_sync(&epd->refresh_work);

	spin_lock_irqsave(&epd->pending_lock, flags);
	pending_fb      = epd->pending_fb;
	epd->pending_fb = NULL;
	spin_unlock_irqrestore(&epd->pending_lock, flags);

	if (pending_fb)
		drm_framebuffer_put(pending_fb);

	epd_hw_sleep(epd);

	kfree(epd->screen_shadow);
	vfree(epd->mono_buf);
	kfree(epd->spi_buf);
	mutex_destroy(&epd->refresh_mutex);

	dev_info(&spi->dev, "epd8951hat: removed\n");
}

/* =========================================================================
 * SPI device ID tables
 * ========================================================================= */

static const struct of_device_id epd_of_match[] = {
	{ .compatible = "waveshare,it8951" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, epd_of_match);

static const struct spi_device_id epd_spi_id[] = {
	{ "it8951", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(spi, epd_spi_id);

/* =========================================================================
 * Power management
 * ========================================================================= */

#ifdef CONFIG_PM_SLEEP

static int epd_pm_suspend(struct device *dev)
{
	struct epd_device *epd = spi_get_drvdata(to_spi_device(dev));

	/* Flush any pending refresh before sleeping */
	cancel_work_sync(&epd->refresh_work);
	epd_hw_sleep(epd);
	return 0;
}

static int epd_pm_resume(struct device *dev)
{
	struct epd_device *epd = spi_get_drvdata(to_spi_device(dev));

	epd_hw_wakeup(epd);
	/*
	 * The DRM core will re-enable the pipe and send a new atomic commit
	 * with the current framebuffer, triggering a full redraw via
	 * epd_pipe_enable() + epd_pipe_update().  No explicit re-draw needed.
	 */
	return 0;
}

static SIMPLE_DEV_PM_OPS(epd_pm_ops, epd_pm_suspend, epd_pm_resume);
#define EPD_PM_OPS (&epd_pm_ops)

#else
#define EPD_PM_OPS NULL
#endif /* CONFIG_PM_SLEEP */

/* =========================================================================
 * SPI driver registration
 * ========================================================================= */

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
