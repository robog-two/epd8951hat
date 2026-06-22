# Porting epd8951hat from fbdev to DRM

Research notes for converting the IT8951 e-Paper HAT driver from a legacy
`/dev/fbN` framebuffer driver into a modern DRM/KMS driver.

Target: Linux 6.12 (Manjaro ARM, RPi4). All APIs below were verified against
`/lib/modules/6.12.41-1-MANJARO-RPI4/build/include/drm/`.

---

## 1. Why DRM, and what changes

The current driver is a classic `fb_ops` + `fb_deferred_io` SPI driver. fbdev is
deprecated for new drivers; the kernel's small-panel SPI displays all live under
`drivers/gpu/drm/tiny/` now. Converting buys:

- A `/dev/dri/cardN` + `/dev/dri/renderD` device with the atomic KMS uAPI.
- **Native damage clipping** (`DRM_IOCTL_MODE_DIRTYFB` / `FB_DAMAGE_CLIPS`) —
  this is the DRM equivalent of your tile dirty-tracker, and it comes from the
  core. Userspace (Wayland, X modesetting, libdrm) tells you exactly which
  rectangle changed.
- A **fbdev emulation** layer (`drm_fbdev_shmem`) so `/dev/fb0`, fbcon and old
  apps keep working *for free* — you do not lose the console.
- Standard format conversion helpers (`drm_fb_xrgb8888_to_mono`, `_to_gray8`),
  so userspace can hand you XRGB8888 and the core packs it down.

The cost: you stop owning the buffer and the refresh-policy plumbing, and adopt
the atomic-commit lifecycle. Your *hardware* layer (`epd8951hat_spi.c`) barely
changes — that's the good news.

### The reference driver

This kernel already builds `CONFIG_TINYDRM_REPAPER=m` — `drivers/gpu/drm/tiny/repaper.c`.
That is a complete SPI e-paper DRM driver (Pervasive Displays, mono, deferred
grayscale-free panel) and is the single best template. The other close
references are `st7586.c` (mono-ish, SPI, damage-based partial upload) and
`ili9225.c`/`mipi-dbi.c` (SPI, RGB565, damage upload). Read `repaper.c`
end-to-end before writing a line.

---

## 2. Architecture: which DRM helper stack

For a single fixed-resolution panel with no real CRTC/plane separation, use the
**simple KMS pipeline** + **GEM SHMEM** + **fbdev shmem emulation**. This is what
every `tiny/` SPI driver uses.

```
            userspace (Wayland / X / fbcon-via-emulation)
                              │  XRGB8888 dumb buffer + damage
                              ▼
   drm_simple_display_pipe  ── .update() callback ── your refresh
                              │
            drm_gem_shmem_object (the framebuffer, vmalloc-backed, mmappable)
                              │  drm_fb_xrgb8888_to_mono()  (or your own 1bpp packer)
                              ▼
   epd8951hat_spi.c  (UNCHANGED: LD_IMG_AREA / DPY_AREA / wait_busy / VCOM …)
                              ▼
                         IT8951 over SPI
```

Key building blocks (all present in this kernel's headers):

| Concern                | fbdev (now)                     | DRM (target)                                            |
|------------------------|---------------------------------|--------------------------------------------------------|
| Device model           | `spi_driver` + `framebuffer_alloc` | `spi_driver` + `devm_drm_dev_alloc` of your `drm_device` |
| Buffer                 | `vzalloc` + `screen_base`       | `drm_gem_shmem` dumb buffer (core-allocated, mmappable) |
| Pipeline               | `fb_ops`                        | `drm_simple_display_pipe_funcs` (`.enable/.disable/.update`) |
| Dirty tracking         | your tile bitmap + deferred_io  | `drm_atomic_helper_damage_iter_*` over plane damage    |
| Console / legacy `/dev/fb` | this *is* the fb driver     | `drm_fbdev_shmem_setup()` emulation                    |
| Power / blank          | `fb_blank`                      | pipe `.disable`/`.enable` (+ optional runtime PM)      |
| Mode list              | `fb_check_var` fixed res        | one `drm_display_mode` from panel_w×panel_h            |

---

## 3. Component-by-component mapping

### 3.1 Keep almost verbatim: `epd8951hat_spi.c`

Everything that talks to the IT8951 — `epd_hw_init`, `epd_hw_reset`,
`epd_hw_sleep/wakeup`, `epd_set_vcom`, `epd_wait_display_ready`,
`epd_load_image_1bpp`, `epd_display_area`, `epd_full_clear`, the CS-framing
discipline, the LUT-variant detection — is hardware-layer code that DRM does not
touch. It stays. The only edits:

- `dev_err(&epd->spi->dev, …)` can stay (you still have the `spi_device`), or
  switch to `drm_err(&epd->drm, …)`. Cosmetic.
- The `spi_buf` scratch buffer and `EPD_SPI_BUF_SIZE` logic are unchanged.

This is ~900 lines you do **not** rewrite. That's the bulk of the driver.

### 3.2 `epd8951hat_dirty.c` → mostly deleted

DRM gives you damage rectangles directly, so the tile bitmap, bbox math,
`epd_dirty_*`, and `fb_deferred_io` page tracking all go away. You replace the
*input* side (what got dirty) with the damage iterator; you may keep the *policy*
side (small-rect ⇒ A2 fast mode, large/most-of-screen ⇒ GC16/INIT) as plain
helpers that operate on a `struct drm_rect`.

The `epd_is_cursor_update(w,h)` heuristic survives as-is — feed it the damage
clip size.

### 3.3 `epd8951hat_refresh.c` → folds into `.update()`

In DRM the trigger model inverts. Instead of "writes mark tiles dirty → delayed
work coalesces → `epd_do_refresh`", you get: "atomic commit lands → simple-pipe
`.update()` is called with the new plane state, which carries the damage."

Two valid designs:

- **Synchronous (like repaper/st7586):** do the SPI upload directly inside
  `.update()`. Simple, correct, but `.update()` runs on the atomic commit path —
  for a slow 1872×1404 panel you want it on a worker, see below.
- **Worker-backed (recommended for this panel):** `.update()` snapshots the
  damage into a pending rect, then schedules a `kthread_worker`/`workqueue` item
  that does the real, slow SPI refresh — preserving your existing coalescing and
  the A2-ghosting-counter / periodic-INIT policy. This is the closest match to
  your current behaviour and keeps the big GC16 passes off the commit thread.

Your A2 ghosting counter, the 90%-dirty ⇒ full INIT+GC16 rule, the double-buffer
"did it actually change" `memcmp`, and the priority (cursor) vs normal delay all
remain meaningful — they just key off damage rects instead of tiles.

### 3.4 `epd8951hat_main.c` → the part that's genuinely rewritten

Replaces `fb_ops`/probe/`fb_deferred_io` with the DRM device + simple pipe.

---

## 4. Pixel format decision (important)

The IT8951 path you already have consumes **1bpp MSB-first** and converts to 4bpp
nibbles. Two options for the DRM-facing format:

1. **Advertise `DRM_FORMAT_XRGB8888` only** (what nearly all `tiny/` drivers do).
   Userspace + fbdev-emulation always speak XRGB8888. In `.update()` you call
   `drm_fb_xrgb8888_to_mono(dst_1bpp, &pitch, src_map, fb, clip, &conv_state)`
   to get exactly the MSB-first 1bpp buffer your `epd_load_image_1bpp()` already
   expects. **This is the least-friction route** and keeps your packer untouched.
   Verified signature in this tree:
   ```c
   void drm_fb_xrgb8888_to_mono(struct iosys_map *dst, const unsigned int *dst_pitch,
                                const struct iosys_map *src, const struct drm_framebuffer *fb,
                                const struct drm_rect *clip, struct drm_format_conv_state *state);
   ```
   `to_mono` packs MSB-first, threshold at 50% luma — matches MONO01 semantics.
   Note it converts a *clip-sized* destination; mind the pitch/origin like st7586.

2. **Advertise `DRM_FORMAT_R1`/`C1`** (native 1bpp). More "honest" but far less
   userspace support and more plumbing. Not worth it here.

   *Middle option:* `drm_fb_xrgb8888_to_gray8` if you later want to exploit the
   IT8951's real 16-level GC16 grayscale instead of pure B/W. The hardware and
   your 4bpp upload path can do grayscale; your current driver throws it away at
   the 1bpp boundary. DRM makes keeping gray8 easy — worth considering as a
   follow-up, but start with mono to match existing behaviour.

Recommendation: **XRGB8888 in, `to_mono` → existing 1bpp packer.** Minimal change,
fbcon works, you can add a gray8 path later without touching the uAPI.

---

## 5. The e-paper-specific hard parts (these are why repaper exists)

DRM was built around fast, full-frame, vblank-driven displays. E-paper breaks
several assumptions; here's how the existing tiny drivers cope and what you must
carry over from your current driver:

1. **No vblank.** Set `crtc_state->no_vblank = true` in the pipe `.check` (or rely
   on the simple-pipe helper faking it). The atomic helper then synthesizes the
   vblank event so `.update()` completion isn't gated on hardware you don't have.

2. **Slow, blocking refreshes (hundreds of ms to seconds for INIT/GC16).** Do
   **not** block the atomic commit thread for a full GC16 on a 1872×1404 panel.
   Use the worker-backed `.update()` (§3.3). repaper runs its work on a thread;
   mirror that.

3. **Waveform modes & ghosting policy is yours to keep.** DRM has no concept of
   A2 vs GC16 vs INIT, partial vs full, or ghosting accumulation. All of that
   stays in your code — now driven by damage-rect size and your counter rather
   than tile %.

4. **Partial-update alignment.** Your `epd_align_region()` (2-px / M641 8-px
   alignment, mirror-x origin remap) must still run on the damage clip before
   `LD_IMG_AREA`/`DPY_AREA`. Damage rects from userspace are arbitrary; align
   them outward exactly as today.

5. **First-frame / enable.** Pipe `.enable()` is the place for `epd_hw_init`'s
   runtime bits + an initial `epd_full_clear()` (your probe-time clear). `.disable()`
   is `epd_hw_sleep()`. Wakeup/redraw-all maps to `.enable()` again.

6. **Shadow buffer / "did it really change".** Keep `screen_shadow` + the
   `memcmp` skip. Damage from compositors is often conservative (whole window);
   your shadow compare avoids needless slow e-paper flashes. This is a genuine
   value-add your driver already has that generic DRM does not.

---

## 6. Concrete skeleton

`epd8951hat_drv.c` (replaces `_main.c`); `_spi.c` kept; `_dirty.c` deleted;
`_refresh.c` slimmed to worker + policy helpers.

```c
struct epd_device {
    struct drm_device         drm;
    struct drm_simple_display_pipe pipe;
    struct drm_connector      connector;
    struct drm_display_mode   mode;     /* panel_w x panel_h, fixed */

    struct spi_device   *spi;
    struct gpio_desc    *gpio_rst, *gpio_busy;
    /* ... all the existing IT8951 fields: dev_info, img_ram_addr,
       a2_mode, panel_w/h, vcom_mv, mirror_x, spi_buf, screen_shadow ... */

    struct work_struct   refresh_work;  /* slow SPI upload off commit thread */
    /* pending damage + policy counters (a2_count, etc.) */
};

#define to_epd(_drm) container_of(_drm, struct epd_device, drm)

static const uint32_t epd_formats[] = { DRM_FORMAT_XRGB8888 };

static void epd_pipe_enable(struct drm_simple_display_pipe *pipe,
                            struct drm_crtc_state *crtc_state,
                            struct drm_plane_state *plane_state)
{
    struct epd_device *epd = to_epd(pipe->crtc.dev);
    epd_hw_wakeup(epd);          /* or epd_hw_init bits */
    epd_full_clear(epd);
    /* upload current plane_state->fb fully (GC16) */
}

static void epd_pipe_disable(struct drm_simple_display_pipe *pipe)
{
    epd_hw_sleep(to_epd(pipe->crtc.dev));
}

static void epd_pipe_update(struct drm_simple_display_pipe *pipe,
                            struct drm_plane_state *old_state)
{
    struct epd_device *epd = to_epd(pipe->crtc.dev);
    struct drm_plane_state *state = pipe->plane.state;
    struct drm_rect rect;

    if (!pipe->crtc.state->active)
        return;

    if (drm_atomic_helper_damage_merged(old_state, state, &rect)) {
        /* stash rect (merge with any pending), then: */
        schedule_work(&epd->refresh_work);   /* slow path */
    }
}

static const struct drm_simple_display_pipe_funcs epd_pipe_funcs = {
    .enable  = epd_pipe_enable,
    .disable = epd_pipe_disable,
    .update  = epd_pipe_update,
    /* .check can set crtc_state->no_vblank = true */
};

/* refresh_work: vmap the GEM, drm_fb_xrgb8888_to_mono() into a 1bpp scratch
   for the (aligned) damage rect, run epd_region_changed()/shadow compare,
   pick A2 vs GC16 vs INIT by size+counter, call epd_load_image_1bpp() +
   epd_display_area() exactly as epd_do_refresh() does today. */

static const struct drm_mode_config_funcs epd_mode_config_funcs = {
    .fb_create     = drm_gem_fb_create_with_dirty,  /* enables DIRTYFB damage */
    .atomic_check  = drm_atomic_helper_check,
    .atomic_commit = drm_atomic_helper_commit,
};

DEFINE_DRM_GEM_FOPS(epd_fops);

static const struct drm_driver epd_drm_driver = {
    .driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
    .fops            = &epd_fops,
    DRM_GEM_SHMEM_DRIVER_OPS,        /* dumb_create, mmap, prime, etc. */
    .name = "epd8951hat", /* .desc, .date, .major ... */
};

static int epd_probe(struct spi_device *spi)
{
    struct epd_device *epd = devm_drm_dev_alloc(&spi->dev, &epd_drm_driver,
                                                struct epd_device, drm);
    /* gpio + spi_setup + epd_hw_init  (UNCHANGED from current probe) */
    /* fill epd->mode from panel_w/h; drmm_mode_config_init();
       set min/max width/height; mode_config.funcs = &epd_mode_config_funcs */
    /* drm_connector_init + drm_simple_display_pipe_init(... epd_formats ...) */
    /* drm_mode_config_reset(); drm_dev_register(&epd->drm, 0);
       drm_fbdev_shmem_setup(&epd->drm, 0);  // fbcon/legacy /dev/fb */
}
```

Notes verified against this kernel:
- `drm_fbdev_shmem_setup(struct drm_device *, unsigned int preferred_bpp)` exists
  (`include/drm/drm_fbdev_shmem.h`).
- `drm_gem_shmem_*` + `DRM_GEM_SHMEM_DRIVER_OPS` present.
- `drm_atomic_helper_damage_merged` + the damage iterator are available
  (`drm_damage_helper.h` / `drm_plane.h`).
- `drm_fb_xrgb8888_to_mono` / `_to_gray8` present (`drm_format_helper.h`).
- `drm_gem_fb_create_with_dirty` (in `drm_gem_framebuffer_helper.h`) is what wires
  up `DIRTYFB`/damage; use it as `mode_config.funcs.fb_create`.

---

## 7. Migration plan (suggested order)

1. **Don't touch `_spi.c` first.** Build a minimal DRM driver that advertises
   XRGB8888, on every `.update()` does `to_mono` of the *whole* frame →
   `epd_load_image_1bpp` whole panel → GC16. Ignore damage, ignore A2. Get a
   picture on screen via the fbdev-emulation console. This proves the plumbing.
2. **Add damage.** Switch `.update()` to `drm_atomic_helper_damage_merged`, align
   the rect, partial `to_mono`/load/`DPY_AREA`. Re-introduce the shadow compare.
3. **Re-add policy.** Port the A2/GC16/INIT selection, ghosting counter, and the
   cursor-priority vs normal coalescing onto a worker. This is your `_refresh.c`
   logic, lightly adapted.
4. **Power.** Map `.enable`/`.disable` to wakeup/sleep; optionally add runtime PM.
5. **Kconfig/Makefile/DT.** `depends on DRM && SPI`, `select DRM_KMS_HELPER`,
   `select DRM_GEM_SHMEM_HELPER`. DT `compatible` stays `waveshare,it8951`.

---

## 8. Gotchas

- **`.update()` runs under the commit; never sleep for a full waveform there.**
  Always hand the slow SPI work to your worker. (This is the #1 mistake.)
- **fbcon will hammer you** through the emulation with frequent small damages —
  your cursor-priority path and shadow compare matter more, not less, under DRM.
- **mmap'd dumb buffers + deferred flush:** unlike `fb_deferred_io`, dumb-buffer
  mmaps don't auto-generate damage. Apps must issue `DRM_IOCTL_MODE_DIRTYFB`
  (libdrm `drmModeDirtyFB`) or use atomic `FB_DAMAGE_CLIPS`. The fbdev emulation
  layer *does* call dirty for you, so the console path is fine; a raw dumb-buffer
  app that forgets to dirty will see nothing update. Document this.
- **Keep the mirror-x / rotation handling in the SPI layer**, not via DRM plane
  rotation — your `epd_load_image_1bpp` already bakes mirror into the column walk
  and `epd_display_area` into the origin. Simpler to leave it there than to wire
  `DRM_MODE_ROTATE_*`.
- **`drm_fb_xrgb8888_to_mono` writes a clip-sized, clip-origin buffer.** Your
  packer assumes full-panel stride/coords. Either convert into a correctly-strided
  scratch and adjust offsets (see how st7586 handles clip), or convert full-frame
  for simplicity in step 1.

---

## 9. Bottom line

- ~60% of the work is *deletion* (tile tracker, deferred_io, fb_ops).
- The IT8951 SPI layer — the hard, already-debugged part — is reused intact.
- The new code is the simple-pipe + GEM-SHMEM boilerplate (≈150 lines, closely
  modeled on `repaper.c`) plus re-homing your refresh policy onto a worker
  driven by damage rects.
- `repaper.c` in this very kernel tree is the template; read it first.
