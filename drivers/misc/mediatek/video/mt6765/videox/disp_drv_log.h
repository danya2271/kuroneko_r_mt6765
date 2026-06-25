/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __DISP_DRV_LOG_H__
#define __DISP_DRV_LOG_H__

#include <linux/printk.h>
#include <linux/ratelimit.h>
#include "display_recorder.h"
#include "ddp_debug.h"

#define DISP_LOG_PRINT(level, sub_module, fmt, args...)			\
	pr_debug("mtk-disp/%s: " fmt, sub_module, ##args)

#define DISPINFO(string, args...)					\
	pr_debug("mtk-disp: " string, ##args)

#define DISPMSG(string, args...)					\
	pr_debug("mtk-disp: " string, ##args)

#define DISPCHECK(string, args...)					\
	pr_debug("mtk-disp: " string, ##args)

#define DISPWARN(string, args...)					\
	pr_warn_ratelimited("mtk-disp: " string, ##args)

#define DISPERR(string, args...)					\
	pr_err_ratelimited("mtk-disp: " string, ##args)

#define DISPPR_FENCE(string, args...)					\
	pr_debug("mtk-disp/fence: " string, ##args)

#define DISPDBG(string, args...) DISPMSG(string, ##args)

#define DISPFUNC() pr_debug("mtk-disp: %s\n", __func__)

#define DISPDBGFUNC() DISPFUNC()

#define DISPPR_HWOP(string, args...)

#define disp_aee_print(string, args...) DISPERR(string, ##args)
#define disp_aee_db_print(string, args...) DISPERR(string, ##args)

#define _DISP_PRINT_FENCE_OR_ERR(is_err, string, args...) \
	do { \
		if (is_err) \
			DISPERR(string, ##args); \
		else \
			DISPPR_FENCE(string, ##args); \
	} while (0)


#endif /* __DISP_DRV_LOG_H__ */
