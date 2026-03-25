# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for the IPU drivers
#

CONFIG_MODULE_SIG:=n

# camera sensors
export CONFIG_VIDEO_AR0234 = m
export CONFIG_VIDEO_LT6911UXC = m
export CONFIG_VIDEO_LT6911UXE = m
export CONFIG_VIDEO_IMX390 = m
export CONFIG_VIDEO_IMX477 = m
export CONFIG_VIDEO_IMX708 = m
export CONFIG_VIDEO_IMX219 = m
export CONFIG_VIDEO_OV5647 = m



subdir-ccflags-$(CONFIG_IPU_BRIDGE) += \
	-DCONFIG_IPU_BRIDGE
subdir-ccflags-y += $(subdir-ccflags-m)

KVERSION := $(shell uname -r)
KERNEL_SRC ?= /lib/modules/$(KVERSION)/build
MODSRC := $(shell pwd)

obj-$(CONFIG_IPU_BRIDGE) += drivers/media/pci/intel/ipu-bridge.o
obj-y += drivers/media/i2c/
#obj-y += drivers/media/i2c/gmsl

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(MODSRC) modules
modules_install:
	$(MAKE) INSTALL_MOD_DIR=updates -C $(KERNEL_SRC) M=$(MODSRC) modules_install
clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(MODSRC) clean

