/* SPDX-License-Identifier: GPL-2.0 */
/*
 * MediaTek MMProfile hooks are intentionally disabled for this target.
 * Native printk/dynamic-debug logging is provided by ddp_log.h instead.
 */
#ifndef __H_DDP_MMP__
#define __H_DDP_MMP__

#include "mmprofile.h"
#include "mmprofile_function.h"

struct DDP_MMP_Events {
	mmp_event esd_extte;
	mmp_event esd_rdlcm;
	mmp_event esd_check_t;
	mmp_event esd_recovery_t;
};

static inline struct DDP_MMP_Events *ddp_mmp_get_events(void)
{
	static struct DDP_MMP_Events disabled_events;

	return &disabled_events;
}

/*
 * A macro is required here: an empty inline function would still evaluate
 * register reads and other expensive arguments before entering the function.
 */
#undef mmprofile_log_ex
#define mmprofile_log_ex(...)			do { } while (0)

#define init_ddp_mmp_events(...)		do { } while (0)
#define ddp_mmp_init(...)			do { } while (0)
#define ddp_mmp_ovl_layer(...)			do { } while (0)
#define ddp_mmp_wdma_layer(...)			do { } while (0)
#define ddp_mmp_rdma_layer(...)			do { } while (0)

#endif
