# SPDX-License-Identifier: GPL-2.0
# Copyright (c) 2010 - 2025 Intel Corporation.

is_kernel_lt_6_10 = $(shell if [ $$(printf "6.10\n$(KERNELVERSION)" | sort -V | head -n1) != "6.10" ]; then echo 1; fi)
ifeq ($(is_kernel_lt_6_10), 1)
ifneq ($(EXTERNAL_BUILD), 1)
src := $(srctree)/$(src)
endif
endif

ccflags-y += -I$(src)/../../../../include/
ccflags-y += -I$(src)/../../pci/intel/ipu6/

obj-$(CONFIG_INTEL_IPU6_ACPI)         += ipu-acpi.o \
					ipu-acpi-pdata.o \
					ipu-acpi-common.o
