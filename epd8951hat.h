/* SPDX-License-Identifier: GPL-2.0 */
/*
 * epd8951hat.h  –  IT8951 e-Paper HAT framebuffer driver
 *
 * Shared types, constants, and function declarations for the epd8951hat
 * kernel module.  Split across four translation units:
 *
 *   epd8951hat_main.c    – module entry, framebuffer registration, fb_ops
 *   epd8951hat_spi.c     – IT8951 SPI/GPIO protocol layer
 *   epd8951hat_dirty.c   – tile-based dirty-region tracker
 *   epd8951hat_refresh.c – refresh scheduling and mode selection
 */

#ifndef EPD8951HAT_H
#define EPD8951HAT_H

#include <linux/atomic.h>
#include <linux/bitops.h>
#include <linux/fb.h>
#include <linux/gpio/consumer.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/workqueue.h>

/* =========================================================================
 * Display geometry limits
 * ========================================================================= */

/* Largest panel the IT8951 supports (10.3", 7.8") */
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
/* User-defined / extended commands */
#define IT8951_CMD_DPY_AREA         0x0034u
#define IT8951_CMD_GET_DEV_INFO     0x0302u
#define IT8951_CMD_DPY_BUF_AREA     0x0037u
#define IT8951_CMD_VCOM             0x0039u

/* =========================================================================
 * IT8951 register addresses
 * ========================================================================= */

/* System registers */
#define IT8951_REG_I80CPCR          0x0004u  /* Enable packed-pixel write */

/* Display engine registers (base 0x1000) */
#define IT8951_DISP_REG_BASE        0x1000u
#define IT8951_REG_LUT01AF          (IT8951_DISP_REG_BASE + 0x0114u)
#define IT8951_REG_UP0SR            (IT8951_DISP_REG_BASE + 0x0134u)
#define IT8951_REG_UP1SR            (IT8951_DISP_REG_BASE + 0x0138u)
#define IT8951_REG_LUTAFSR          (IT8951_DISP_REG_BASE + 0x0224u)
#define IT8951_REG_BGVR             (IT8951_DISP_REG_BASE + 0x0250u)

/* Memory converter registers */
#define IT8951_MCSR_BASE            0x0200u
#define IT8951_REG_MCSR             (IT8951_MCSR_BASE + 0x0000u)
#define IT8951_REG_LISAR            (IT8951_MCSR_BASE + 0x0008u)

/* Enhanced driving capability register */
#define IT8951_REG_ENHANCE_DRV      0x0038u
#define IT8951_ENHANCE_DRV_VAL      0x0602u

/* =========================================================================
 * SPI preamble words (sent as big-endian u16 before cmd/data/read)
 * ========================================================================= */

#define IT8951_PREAMBLE_CMD         0x6000u
#define IT8951_PREAMBLE_WRITE       0x0000u
#define IT8951_PREAMBLE_READ        0x1000u

/* =========================================================================
 * Display refresh modes
 * ========================================================================= */

#define EPD_MODE_INIT               0u   /* Full clear (slow, removes ghosting) */
#define EPD_MODE_DU                 1u   /* Direct update (fastest, 2-level) */
#define EPD_MODE_GC16               2u   /* 16-level grayscale quality mode */
#define EPD_MODE_GL16               3u   /* 16-level mode for light backgrounds */
#define EPD_MODE_A2_M641            4u   /* Fast B/W for M641 (6") firmware */
#define EPD_MODE_A2_M841            6u   /* Fast B/W for M841/M841_TFAxxxx */

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
 *
 * In 1-bpp display mode the IT8951 maps pixel bits to grayscale via BGVR:
 *   BGVR[15:8] = Front_Gray_Val  (for bit = 1, i.e. foreground / black text)
 *   BGVR[ 7:0] = Back_Gray_Val   (for bit = 0, i.e. background / white paper)
 *
 * IT8951 grayscale encoding: 0x00 = white, 0xF0 = black.
 * Linux 1-bpp convention:    bit=1 → foreground (black), bit=0 → background (white).
 * ========================================================================= */

#define IT8951_BGVR_BLACK           0xF0u  /* foreground (bit=1) */
#define IT8951_BGVR_WHITE           0x00u  /* background (bit=0) */
#define IT8951_BGVR_DEFAULT         ((IT8951_BGVR_BLACK << 8) | IT8951_BGVR_WHITE)

/* UP1SR+2 bit[2]: enable 1-bpp display mode */
#define IT8951_UP1SR2_1BPP_EN       (1u << 2)

/* =========================================================================
 * Dirty-region tile configuration
 *
 * The screen is subdivided into tiles of EPD_TILE_SIZE×EPD_TILE_SIZE pixels.
 * A bitmap of unsigned-long words tracks which tiles are dirty.
 * ========================================================================= */

#define EPD_TILE_SIZE               16u   /* pixels per tile edge */

#define EPD_TILES_X_MAX \
	DIV_ROUND_UP(EPD_MAX_WIDTH,  EPD_TILE_SIZE)

#define EPD_TILES_Y_MAX \
	DIV_ROUND_UP(EPD_MAX_HEIGHT, EPD_TILE_SIZE)

#define EPD_TILE_TOTAL  (EPD_TILES_X_MAX * EPD_TILES_Y_MAX)

/* =========================================================================
 * Refresh policy thresholds
 * ========================================================================= */

/* If dirty-tile percentage exceeds this → full-screen refresh */
#define EPD_FULL_REFRESH_THRESHOLD  90u   /* percent */

/* After this many A2-mode refreshes → schedule an INIT clear */
#define EPD_A2_GHOSTING_LIMIT       20u

/* Pixel area below which a write is treated as a cursor/priority update */
#define EPD_CURSOR_MAX_PIXELS       (32u * 32u)

/* Coalescing delays for the deferred-work refresh */
#define EPD_REFRESH_DELAY_MS        50u   /* normal writes */
#define EPD_PRIORITY_DELAY_MS        5u   /* cursor / small writes */

/* Timeout for BUSY GPIO polling (ms) */
#define EPD_BUSY_TIMEOUT_MS        5000u

/* Default VCOM magnitude in millivolts (overridden by module param or DT) */
#define EPD_DEFAULT_VCOM           2000u

/* =========================================================================
 * SPI / DMA buffer
 * ========================================================================= */

/*
 * Largest possible payload: 1bpp full-screen = EPD_MAX_WIDTH * EPD_MAX_HEIGHT / 8
 * We allocate for 1bpp (the most compact format we actually send).
 * Add 32 bytes headroom for preamble, command args, and alignment padding.
 */
#define EPD_SPI_BUF_SIZE \
	(DIV_ROUND_UP(EPD_MAX_WIDTH * EPD_MAX_HEIGHT, 8) + 32u)

/* =========================================================================
 * Hardware device-info structure (returned by GET_DEV_INFO command)
 * Matches the on-wire layout (big-endian u16 fields).
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
 * LUT variant – determines the correct A2 mode code and alignment needs
 * ========================================================================= */

enum epd_lut_variant {
	EPD_LUT_UNKNOWN,
	EPD_LUT_M641,          /* 6"   800×600  / 1448×1072, A2 = mode 4, 4-byte align */
	EPD_LUT_M841,          /* 9.7" 1200×825,              A2 = mode 6 */
	EPD_LUT_M841_TFA2812,  /* 7.8" 1872×1404,             A2 = mode 6 */
	EPD_LUT_M841_TFA5210,  /* 10.3"1872×1404,             A2 = mode 6 */
};

/* =========================================================================
 * Dirty-region tracker
 * ========================================================================= */

struct epd_dirty {
	/* Tile bitmap (one bit per EPD_TILE_SIZE×EPD_TILE_SIZE block) */
	DECLARE_BITMAP(tiles, EPD_TILE_TOTAL);

	spinlock_t  lock;

	/* Bounding box in tile coordinates (inclusive, updated lazily) */
	int bbox_tx0, bbox_ty0;  /* top-left  tile */
	int bbox_tx1, bbox_ty1;  /* bot-right tile */

	unsigned int dirty_tile_count;  /* number of set bits (maintained) */
	bool         all_dirty;         /* shortcut flag */
};

/* =========================================================================
 * Main driver context  (stored in fb_info->par)
 * ========================================================================= */

struct epd_device {
	/* ---- SPI / GPIO ---- */
	struct spi_device  *spi;
	struct gpio_desc   *gpio_rst;
	struct gpio_desc   *gpio_busy;
	struct gpio_desc   *gpio_cs;

	/* ---- Hardware info (from GET_DEV_INFO) ---- */
	struct it8951_dev_info dev_info;
	enum epd_lut_variant   lut_variant;
	u32  img_ram_addr;   /* IT8951 internal DRAM base for image buffer */
	u8   a2_mode;        /* Actual A2 mode code (4 or 6, LUT-dependent) */
	bool needs_4byte_align; /* M641: 1-bpp width must be 32-bit aligned */

	/* ---- Panel configuration ---- */
	u16  panel_w, panel_h;
	int  rotation;
	u16  vcom_mv;           /* VCOM magnitude in millivolts (positive) */
	bool enhance_driving;

	/* ---- Framebuffer ---- */
	struct fb_info *fb_info;
	u8   *fb_vaddr;          /* vmalloc'd framebuffer (user mmap target) */
	u8   *screen_shadow;     /* copy of what is currently on the panel */
	size_t fb_size;          /* bytes (= stride × panel_h) */
	u32    fb_stride;        /* bytes per row (= ceil(panel_w / 8)) */

	/* ---- SPI TX scratch buffer (kmalloc DMA-safe) ---- */
	u8   *spi_buf;
	size_t spi_buf_size;

	/* ---- Dirty tracking ---- */
	struct epd_dirty dirty;

	/* ---- Refresh work ---- */
	struct delayed_work  normal_work;    /* coalesced write refresh */
	struct delayed_work  priority_work;  /* cursor / tiny-area refresh */
	struct mutex         refresh_mutex;  /* serialise actual SPI refresh */

	/* ---- A2 ghosting management ---- */
	atomic_t a2_count;   /* incremented every A2 refresh; reset after INIT */

	/* ---- Device state ---- */
	bool  initialized;
	bool  suspended;
	spinlock_t state_lock;
};

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

/*
 * Load a 1-bpp pixel region into the IT8951's internal framebuffer.
 *
 * Extracts the sub-region [x, x+w) × [y, y+h) from a 1-bpp framebuffer,
 * converts each bit to a 4-bpp nibble (0x0=white, 0xF=black), and streams
 * the result to the IT8951 using LD_IMG_AREA in 4BPP host format.
 *
 * @fb_base:   Base pointer to the full 1-bpp framebuffer.
 * @fb_stride: Bytes per row in the framebuffer (= ceil(panel_w / 8)).
 * @x, y, w, h: Region to upload.  w must already be 4-byte-aligned by caller
 *              on M641 hardware; must be a multiple of 2 on other hardware.
 */
int  epd_load_image_1bpp(struct epd_device *epd,
			  const u8 *fb_base, u32 fb_stride,
			  u16 x, u16 y, u16 w, u16 h);

/*
 * Trigger display-controller refresh of an already-loaded region.
 * @mode: EPD_MODE_A2_*, EPD_MODE_GC16, EPD_MODE_INIT, etc.
 */
int  epd_display_area(struct epd_device *epd, u16 x, u16 y, u16 w, u16 h,
		       u8 mode);

/* Full-screen clear using INIT mode (clears any A2 ghosting). */
int  epd_full_clear(struct epd_device *epd);

/* =========================================================================
 * Function declarations – epd8951hat_dirty.c
 * ========================================================================= */

void epd_dirty_init(struct epd_dirty *d);

/* Mark the pixel rectangle [x, x+w) × [y, y+h) dirty. */
void epd_dirty_mark(struct epd_dirty *d, int x, int y, int w, int h,
		    int panel_w, int panel_h);

/* Mark the entire screen dirty. */
void epd_dirty_mark_all(struct epd_dirty *d, int panel_w, int panel_h);

/*
 * Mark tiles covered by pages in the deferred-io page list dirty.
 * @page_offset: byte offset into the framebuffer for the start of the page.
 * @stride:      bytes per row.
 * @panel_w, panel_h: panel dimensions in pixels.
 */
void epd_dirty_mark_page(struct epd_dirty *d, unsigned long page_offset,
			  u32 stride, int panel_w, int panel_h);

/* Reset all dirty state. */
void epd_dirty_clear(struct epd_dirty *d);

/* Percentage of tiles that are dirty (0-100). */
unsigned int epd_dirty_percent(const struct epd_dirty *d,
			       int panel_w, int panel_h);

/*
 * Compute the bounding box of all dirty tiles in pixel coordinates.
 * Returns false if nothing is dirty.
 */
bool epd_dirty_bbox_pixels(const struct epd_dirty *d,
			   int panel_w, int panel_h,
			   int *x, int *y, int *w, int *h);

/* True if the write is small enough to be treated as a cursor update. */
static inline bool epd_is_cursor_update(int w, int h)
{
	return (unsigned int)(w * h) <= EPD_CURSOR_MAX_PIXELS;
}

/* =========================================================================
 * Function declarations – epd8951hat_refresh.c
 * ========================================================================= */

int  epd_refresh_init(struct epd_device *epd);
void epd_refresh_cleanup(struct epd_device *epd);

/*
 * Schedule a refresh.  If @priority is true, uses a much shorter delay
 * so cursor movements appear immediately on screen.
 */
void epd_schedule_refresh(struct epd_device *epd, bool priority);

/*
 * Perform the actual refresh now (called from work-queue context).
 * Examines dirty state, selects refresh mode, sends pixels, updates
 * screen_shadow.
 */
void epd_do_refresh(struct epd_device *epd);

#endif /* EPD8951HAT_H */
