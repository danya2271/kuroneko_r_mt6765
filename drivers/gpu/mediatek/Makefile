#
# Copyright (C) 2015 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
ccflags-y += -Ofast

MTK_GPU_VERSION := $(subst ",,$(CONFIG_MTK_GPU_VERSION))
export MTK_GPU_VERSION
MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
export MTK_PLATFORM

ifneq ($(CONFIG_MTK_GPU_SUPPORT),)
	obj-y += hal/
	obj-y += gpufreq/
	obj-y += ged/
ifneq (, $(findstring $(CONFIG_MTK_PLATFORM), "mt6779"))
	obj-y += gpu_bm/
$(info *GPU defined GPU BM QoS 2.0)
endif

# GPU R5P0
ifeq ($(CONFIG_MTK_GPU_R5P0),y)
    obj-y += $(subst ",,$(CONFIG_MTK_PLATFORM))/
else ifneq ($(word 1,$(MTK_GPU_VERSION)),)
    obj-y += gpu_$(word 1,$(MTK_GPU_VERSION))/
else
    obj-y += $(subst ",,$(CONFIG_MTK_PLATFORM))/
endif
endif

ifeq ($(CONFIG_GPU_VERSION_MALI_MIDGARD_R26P0),y)
ifneq ($(CONFIG_MTK_GPU_SUPPORT),m)
	obj-y += hal/
	obj-y += gpu_mali/
endif
endif
