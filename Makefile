# SPDX-License-Identifier: GPL-2.0
# Kernel module Makefile for the IT8951 e-Paper HAT DRM driver.
# Target: Raspberry Pi running Linux 6.x.
#
# Usage:
#   make [KDIR=/path/to/kernel/build]   - build the module
#   make install                         - install module, run depmod, enable autoload on boot
#   make uninstall                       - remove module and disable autoload
#   make clean                           - remove build artefacts
#
# Cross-compile example (arm64 RPi):
#   make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- \
#        KDIR=/path/to/rpi-kernel/build

obj-m := epd8951hat.o

epd8951hat-objs := \
	epd8951hat_drv.o      \
	epd8951hat_spi.o      \
	epd8951hat_refresh.o  \
	epd8951hat_pipeline.o

KDIR ?= /lib/modules/$(shell uname -r)/build
PWD  := $(shell pwd)

# Extra CFLAGS: treat warnings as errors, keep frame-pointer for ftrace
ccflags-y := -Wall -Wextra -Werror -fno-omit-frame-pointer

MODNAME      := epd8951hat
MODULES_LOAD := /etc/modules-load.d/$(MODNAME).conf
OVERLAY_SRC  := $(PWD)/epd8951hat-overlay.dts
OVERLAY_DTB  := $(PWD)/epd8951hat.dtbo
OVERLAYS_DIR := /boot/overlays
CONFIG_TXT   := /boot/config.txt

.PHONY: all dtbo clean install uninstall test

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

dtbo: $(OVERLAY_DTB)

$(OVERLAY_DTB): $(OVERLAY_SRC)
	dtc -@ -I dts -O dtb -o $@ $< 2>&1 | grep -v '^/' || true

test:
	$(MAKE) -C tests run

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	$(MAKE) -C tests clean
	rm -f $(OVERLAY_DTB)

install: all dtbo
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install
	depmod -A
	@echo "$(MODNAME)" > $(MODULES_LOAD)
	@install -m 644 $(OVERLAY_DTB) $(OVERLAYS_DIR)/$(MODNAME).dtbo
	@if grep -q "dtoverlay=$(MODNAME)" $(CONFIG_TXT); then \
		echo "dtoverlay=$(MODNAME) already in $(CONFIG_TXT)"; \
	else \
		echo "dtoverlay=$(MODNAME)" >> $(CONFIG_TXT); \
		echo "Added dtoverlay=$(MODNAME) to $(CONFIG_TXT)"; \
	fi
	@echo ""
	@echo "Module and overlay installed. Reboot to activate."
	@echo "After reboot, check:  dmesg | grep epd"

uninstall:
	modprobe -r $(MODNAME) 2>/dev/null || true
	rm -f $(MODULES_LOAD)
	find /lib/modules -name "$(MODNAME).ko*" -delete
	depmod -A
	rm -f $(OVERLAYS_DIR)/$(MODNAME).dtbo
	@sed -i "/^dtoverlay=$(MODNAME)/d" $(CONFIG_TXT) 2>/dev/null || true
	@echo "Module and overlay uninstalled."
