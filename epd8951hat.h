/* SPDX-License-Identifier: GPL-2.0 */
/*
 * epd8951hat.h  –  IT8951 e-Paper HAT DRM driver
 *
 * Shared types, constants, and function declarations for the epd8951hat
 * kernel module.  Split across three translation units:
 *
 *   epd8951hat_drv.c     – DRM device, simple-pipe, probe/remove
 *   epd8951hat_spi.c     – IT8951 SPI/GPIO protocol layer
 *   epd8951hat_refresh.c – refresh scheduling and mode selection
 */

#ifndef EPD8951HAT_H
#define EPD8951HAT_H

#include <linux/atomic.h>
#include <linux/gpio/consumer.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include <drm/drm_connector.h>
#include <drm/drm_device.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_modes.h>
#include <drm/drm_rect.h>
#include <drm/drm_simple_kms_helper.h>

/* =========================================================================
 * Display geometry limits
 * ========================================================================= */

#define EPD_MAX_WIDTH   1872u
#define EPD_MAX_HEIGHT  1404u

/* =========================================================================
 * IT8951 command codes  (I80 host interface)
 * ========================================================================= */

#define IT8951_CMD_SYS_RUN          0x0001u
#define IT8951_CMD_STANDBY          0x0002u
#define IT8951_CMD_SLEEP            0x0003u
#define IT8951_CMD_REG_RD           0x0010u
#define IT8951_CMD_REG_WR           0x0011u
#define IT8951_CMD_MEM_BST_RD_T     0x0012u
#define IT8951_CMD_MEM_BST_RD_S     0x0013u
#define IT8951_CMD_MEM_BST_WR       0x0014u
#define IT8951_CMD_MEM_BST_END      0x0015u
#define IT8951_CMD_LD_IMG           0x0020u
#define IT8951_CMD_LD_IMG_AREA      0x0021u
#define IT8951_CMD_LD_IMG_END       0x0022u
#define IT8951_CMD_DPY_AREA         0x0034u
#define IT8951_CMD_GET_DEV_INFO     0x0302u
#define IT8951_CMD_DPY_BUF_AREA     0x0037u
#define IT8951_CMD_VCOM             0x0039u

/* =========================================================================
 * IT8951 register addresses
 * ========================================================================= */

#define IT8951_REG_I80CPCR          0x0004u
#define IT8951_DISP_REG_BASE        0x1000u
#define IT8951_REG_LUT01AF          (IT8951_DISP_REG_BASE + 0x0114u)
#define IT8951_REG_UP0SR            (IT8951_DISP_REG_BASE + 0x0134u)
#define IT8951_REG_UP1SR            (IT8951_DISP_REG_BASE + 0x0138u)
#define IT8951_REG_LUTAFSR          (IT8951_DISP_REG_BASE + 0x0224u)
#define IT8951_REG_BGVR             (IT8951_DISP_REG_BASE + 0x0250u)
#define IT8951_MCSR_BASE            0x0200u
#define IT8951_REG_MCSR             (IT8951_MCSR_BASE + 0x0000u)
#define IT8951_REG_LISAR            (IT8951_MCSR_BASE + 0x0008u)
#define IT8951_REG_ENHANCE_DRV      0x0038u
#define IT8951_ENHANCE_DRV_VAL      0x0602u

/* =========================================================================
 * SPI preamble words
 * ========================================================================= */

#define IT8951_PREAMBLE_CMD         0x6000u
#define IT8951_PREAMBLE_WRITE       0x0000u
#define IT8951_PREAMBLE_READ        0x1000u

/* =========================================================================
 * Display refresh modes
 * ========================================================================= */

#define EPD_MODE_INIT               0u
#define EPD_MODE_DU                 1u
#define EPD_MODE_GC16               2u
#define EPD_MODE_GL16               3u
#define EPD_MODE_A2_M641            4u
#define EPD_MODE_A2_M841            6u

/* =========================================================================
 * Pixel format codes used in LD_IMG args[0]
 * ========================================================================= */

#define IT8951_PIX_FMT_2BPP         0u
#define IT8951_PIX_FMT_3BPP         1u
#define IT8951_PIX_FMT_4BPP         2u
#define IT8951_PIX_FMT_8BPP         3u

#define IT8951_ENDIAN_LITTLE        0u
#define IT8951_ENDIAN_BIG           1u

#define IT8951_ROTATE_0             0u
#define IT8951_ROTATE_90            1u
#define IT8951_ROTATE_180           2u
#define IT8951_ROTATE_270           3u

/* =========================================================================
 * 1-bpp display mode helpers
 * ========================================================================= */

/*
 * BGVR for MONO01 1bpp display mode:
 *   high byte = foreground gray (bit=1 = ink = black) → 0x00
 *   low  byte = background gray (bit=0 = paper = white) → 0xF0
 * In IT8951 4bpp scale: 0x0=black, 0xF=white; each nibble is stored in the
 * upper nibble of its BGVR byte (e.g. 0xF0 encodes gray level 0xF = white).
 */
#define IT8951_BGVR_DEFAULT         0x00F0u
#define IT8951_UP1SR2_1BPP_EN       (1u << 2)

/* =========================================================================
 * Refresh policy thresholds
 * ========================================================================= */

#define EPD_NOMINAL_VREFRESH_HZ     30u   /* nominal vrefresh reported to compositor; must be >1
                                             so niri's FrameClock (which asserts interval < 1s)
                                             doesn't panic. The driver's workqueue drops frames
                                             that arrive faster than the SPI bus can flush them. */
#define EPD_FULL_REFRESH_THRESHOLD  95u   /* percent of panel area → INIT+GC16 */
#define EPD_MIN_UPDATE_PIXELS       16u   /* skip partial refresh if area < 4×4 */
#define EPD_A2_GHOSTING_LIMIT       20u   /* A2 refreshes before INIT clear */
#define EPD_CURSOR_MAX_PIXELS       (32u * 32u)
#define EPD_BUSY_TIMEOUT_MS        5000u
#define EPD_DEFAULT_VCOM           2000u

/* =========================================================================
 * SPI / DMA buffer
 * ========================================================================= */

#define EPD_SPI_BUF_SIZE \
	(DIV_ROUND_UP(EPD_MAX_WIDTH * EPD_MAX_HEIGHT, 2) + 32u)

/* =========================================================================
 * Hardware device-info structure (returned by GET_DEV_INFO command)
 * ========================================================================= */

struct it8951_dev_info {
	u16 panel_w;
	u16 panel_h;
	u16 mem_addr_l;
	u16 mem_addr_h;
	u16 fw_version[8];
	u16 lut_version[8];
} __packed;

/* =========================================================================
 * LUT variant
 * ========================================================================= */

enum epd_lut_variant {
	EPD_LUT_UNKNOWN,
	EPD_LUT_M641,
	EPD_LUT_M841,
	EPD_LUT_M841_TFA2812,
	EPD_LUT_M841_TFA5210,
};

/* =========================================================================
 * Main driver context
 * ========================================================================= */

struct epd_device {
	/* drm MUST be first – to_epd() uses container_of on this field */
	struct drm_device              drm;
	struct drm_simple_display_pipe pipe;
	struct drm_connector           connector;
	struct drm_display_mode        mode;   /* fixed panel mode */

	/* ---- SPI / GPIO ---- */
	struct spi_device  *spi;
	struct gpio_desc   *gpio_rst;
	struct gpio_desc   *gpio_busy;

	/* ---- Hardware info (from GET_DEV_INFO) ---- */
	struct it8951_dev_info dev_info;
	enum epd_lut_variant   lut_variant;
	u32  img_ram_addr;
	u8   a2_mode;
	bool needs_4byte_align;

	/* ---- Panel configuration ---- */
	u16  panel_w, panel_h;
	int  rotation;
	bool mirror_x;
	u16  vcom_mv;
	bool enhance_driving;

	/* ---- 1bpp scratch + shadow ---- */
	u8   *mono_buf;       /* XRGB8888→mono conversion target (1bpp, full panel) */
	u8   *screen_shadow;  /* copy of what is currently on the panel */
	size_t fb_size;       /* bytes (= fb_stride × panel_h) */
	u32    fb_stride;     /* bytes per row (= ceil(panel_w / 8)) */

	/* ---- SPI TX scratch buffer (kmalloc DMA-safe) ---- */
	u8   *spi_buf;
	size_t spi_buf_size;

	/* ---- Pending damage (set in .update(), consumed in worker) ---- */
	struct work_struct      refresh_work;
	struct drm_framebuffer *pending_fb;     /* held ref, protected by lock */
	struct drm_rect         pending_rect;   /* latest damage (newest update wins) */
	struct drm_rect         deferred_rect;  /* older displaced updates, processed after latest */
	bool                    has_deferred;   /* deferred_rect is valid */
	spinlock_t              pending_lock;

	/* ---- Refresh serialisation and ghosting counter ---- */
	struct mutex   refresh_mutex;
	atomic_t       a2_count;

	/* ---- Device state ---- */
	bool  initialized;
	bool  suspended;
	bool  pipe_enabled;
};

#define to_epd(_drm) container_of(_drm, struct epd_device, drm)

/* =========================================================================
 * Function declarations – epd8951hat_spi.c
 * ========================================================================= */

int  epd_hw_init(struct epd_device *epd);
void epd_hw_reset(struct epd_device *epd);
void epd_hw_sleep(struct epd_device *epd);
void epd_hw_wakeup(struct epd_device *epd);

int  epd_set_vcom(struct epd_device *epd, u16 vcom_mv);
int  epd_enhance_driving(struct epd_device *epd);
int  epd_wait_display_ready(struct epd_device *epd);

int  epd_load_image_1bpp(struct epd_device *epd,
			  const u8 *fb_base, u32 fb_stride,
			  u16 x, u16 y, u16 w, u16 h);

int  epd_display_area(struct epd_device *epd, u16 x, u16 y, u16 w, u16 h,
		       u8 mode);

/*
 * epd_display_area_1bpp - trigger a waveform update for 1bpp content.
 *
 * Wraps epd_display_area with the UP1SR 1bpp-expansion enable/disable
 * sequence and waits for the waveform to complete before returning, so
 * UP1SR is always restored to its previous state on exit.
 *
 * Use this (not epd_display_area) for all A2/GC16/DU updates after
 * epd_load_image_1bpp().  Do NOT use for INIT clears (epd_full_clear
 * loads 4bpp white data and calls epd_display_area directly).
 */
int  epd_display_area_1bpp(struct epd_device *epd, u16 x, u16 y, u16 w, u16 h,
			    u8 mode);

int  epd_full_clear(struct epd_device *epd);

/* =========================================================================
 * Function declarations – epd8951hat_refresh.c
 * ========================================================================= */

/*
 * Perform a display refresh for the given damage rectangle.
 * Selects A2 / GC16 / INIT based on damage area and ghosting counter.
 * Must be called from process context (may sleep waiting for SPI).
 */
void epd_do_refresh(struct epd_device *epd, const struct drm_rect *damage);

/* True if the write is small enough to be treated as a cursor update. */
static inline bool epd_is_cursor_update(int w, int h)
{
	return (unsigned int)(w * h) <= EPD_CURSOR_MAX_PIXELS;
}

#endif /* EPD8951HAT_H */
