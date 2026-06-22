# SPDX-License-Identifier: GPL-2.0
# Kernel module Makefile for the IT8951 e-Paper HAT framebuffer driver.
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
	epd8951hat_main.o    \
	epd8951hat_spi.o     \
	epd8951hat_dirty.o   \
	epd8951hat_refresh.o

KDIR ?= /lib/modules/$(shell uname -r)/build
PWD  := $(shell pwd)

# Extra CFLAGS: treat warnings as errors, keep frame-pointer for ftrace
ccflags-y := -Wall -Wextra -Werror -fno-omit-frame-pointer

MODNAME      := epd8951hat
MODULES_LOAD := /etc/modules-load.d/$(MODNAME).conf

.PHONY: all clean install uninstall

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

install: all
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install
	depmod -A
	@echo "$(MODNAME)" > $(MODULES_LOAD)
	@echo "Module installed. It will be loaded automatically on next boot."
	@echo "To load it now without rebooting, run:  sudo modprobe $(MODNAME)"

uninstall:
	modprobe -r $(MODNAME) 2>/dev/null || true
	rm -f $(MODULES_LOAD)
	find /lib/modules -name "$(MODNAME).ko*" -delete
	depmod -A
	@echo "Module uninstalled and removed from autoload."
