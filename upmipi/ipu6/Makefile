# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for the IPU drivers
#

CONFIG_MODULE_SIG:=n

# camera sensors
# export CONFIG_VIDEO_HM11B1 = m
# export CONFIG_VIDEO_OV01A1S = m
# export CONFIG_VIDEO_OV01A10 = m
# export CONFIG_VIDEO_OV02C10 = m
# export CONFIG_VIDEO_OV02E10 = m
# export CONFIG_VIDEO_HM2170 = m
# export CONFIG_VIDEO_HM2172 = m
# export CONFIG_VIDEO_HI556 = m
export CONFIG_VIDEO_AR0234 = m
# export CONFIG_VIDEO_LT6911UXC = m
# export CONFIG_VIDEO_LT6911UXE = m
# export CONFIG_VIDEO_D4XX = m
# export CONFIG_VIDEO_TI960 = m
# export CONFIG_VIDEO_IMX390 = m
export CONFIG_VIDEO_IMX477 = m

subdir-ccflags-y += -I$(src)/include/ \
	-DCONFIG_VIDEO_V4L2_SUBDEV_API
subdir-ccflags-$(CONFIG_IPU_BRIDGE) += \
	-DCONFIG_IPU_BRIDGE
subdir-ccflags-$(CONFIG_INTEL_SKL_INT3472) += \
	-DCONFIG_INTEL_SKL_INT3472
subdir-ccflags-$(CONFIG_POWER_CTRL_LOGIC) += \
	-DCONFIG_POWER_CTRL_LOGIC
subdir-ccflags-y += $(subdir-ccflags-m)

KVERSION := $(shell uname -r)
KERNEL_SRC ?= /lib/modules/$(KVERSION)/build
MODSRC := $(shell pwd)

obj-$(CONFIG_IPU_BRIDGE) += drivers/media/pci/intel/ipu-bridge.o
# obj-$(CONFIG_VIDEO_INTEL_IPU6)	+= ipu6/
obj-y += drivers/media/i2c/

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(MODSRC) modules
modules_install:
	$(MAKE) INSTALL_MOD_DIR=updates -C $(KERNEL_SRC) M=$(MODSRC) modules_install
clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(MODSRC) clean

