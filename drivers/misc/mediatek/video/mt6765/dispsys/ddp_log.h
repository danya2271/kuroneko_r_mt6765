/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef _H_DDP_LOG_
#define _H_DDP_LOG_

#include <linux/printk.h>
#include <linux/ratelimit.h>
#include "disp_drv_log.h"

#ifndef LOG_TAG
#define LOG_TAG "core"
#endif

#define DDPSVPMSG(fmt, args...) \
	pr_debug("mtk-disp/" LOG_TAG ": " fmt, ##args)
#define DISP_LOG_I(fmt, args...) \
	pr_debug("mtk-disp/" LOG_TAG ": " fmt, ##args)
#define DISP_LOG_V(fmt, args...) DISP_LOG_I(fmt, ##args)
#define DISP_LOG_D(fmt, args...) DISP_LOG_I(fmt, ##args)
#define DISP_LOG_W(fmt, args...) \
	pr_warn_ratelimited("mtk-disp/" LOG_TAG ": " fmt, ##args)
#define DISP_LOG_E(fmt, args...) \
	pr_err_ratelimited("mtk-disp/" LOG_TAG ": " fmt, ##args)

#define DDPIRQ(fmt, args...) \
	pr_debug_ratelimited("mtk-disp/irq: " fmt, ##args)
#define DDPDBG(fmt, args...) DISP_LOG_D(fmt, ##args)
#define DDPMSG(fmt, args...) DISP_LOG_I(fmt, ##args)
#define DDPWRN(fmt, args...) DISP_LOG_W(fmt, ##args)
#define DDPERR(fmt, args...) DISP_LOG_E(fmt, ##args)
#define DDPDUMP(fmt, args...) DISP_LOG_D(fmt, ##args)

#ifndef ASSERT
#define ASSERT(expr) WARN_ON_ONCE(!(expr))
#endif

#define DDPAEE(fmt, args...) DISP_LOG_E(fmt, ##args)

#endif
