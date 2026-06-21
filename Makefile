# SPDX-License-Identifier: GPL-2.0
# Kernel module Makefile for the IT8951 e-Paper HAT framebuffer driver.
# Target: Raspberry Pi running Linux 6.x.
#
# Usage:
#   make [KDIR=/path/to/kernel/build]   - build the module
#   make install                         - install + run depmod
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

.PHONY: all clean install

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

install: all
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install
	depmod -A
