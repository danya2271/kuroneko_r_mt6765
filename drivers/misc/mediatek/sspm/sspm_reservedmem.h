/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __SSPM_RESERVED_H__
#define __SSPM_RESERVED_H__

#include <linux/types.h>
#include "sspm_define.h"

enum {
	SSPM_MEM_ID = 0,
	PWRAP_MEM_ID,
	PMIC_MEM_ID,
	UPD_MEM_ID,
	QOS_MEM_ID,
	SWPM_MEM_ID,
	SMI_MEM_ID,
	NUMS_MEM_ID,
};

struct sspm_reserve_mblock {
	u32 num;
	u64 start_phys;
	u64 start_virt;
	u64 size;
};

phys_addr_t sspm_reserve_mem_get_phys(unsigned int id);
phys_addr_t sspm_reserve_mem_get_virt(unsigned int id);
phys_addr_t sspm_reserve_mem_get_size(unsigned int id);
int sspm_reserve_memory_init(void);
void sspm_set_emi_mpu(unsigned int id, phys_addr_t base, phys_addr_t size);
void sspm_lock_emi_mpu(unsigned int region);

#ifdef SSPM_SHARE_BUFFER_SUPPORT
#define SSPM_SHARE_REGION_BASE  0x20000
#define SSPM_SHARE_REGION_SIZE  0x8000
#endif

#ifdef SSPM_SHARE_BUFFER_SUPPORT
extern struct platform_device *sspm_pdev;
phys_addr_t sspm_sbuf_get(unsigned int offset);
int sspm_sbuf_init(void);
#endif

extern struct sspm_reserve_mblock *sspm_reserve_mblock;

#endif
