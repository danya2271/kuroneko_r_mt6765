// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/sched/clock.h>
#include <linux/semaphore.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/printk.h>
#include <stdarg.h>
#include <linux/slab.h>

#include "ddp_mmp.h"
#include "debug.h"

#include "disp_drv_log.h"

#include "disp_lcm.h"
#include "disp_utils.h"
#include "mtkfb_info.h"
#include "mtkfb.h"

#include "ddp_hal.h"
#include "ddp_dump.h"
#include "ddp_path.h"
#include "ddp_drv.h"

#if defined(CONFIG_MTK_M4U)
#include "m4u.h"
#endif

#include "primary_display.h"
#if defined(CONFIG_MTK_CMDQ)
#include "cmdq_def.h"
#include "cmdq_record.h"
#include "cmdq_reg.h"
#include "cmdq_core.h"
#endif

#include "ddp_manager.h"
#include "disp_drv_platform.h"
#include "display_recorder.h"
#include "disp_session.h"
#include "ddp_mmp.h"
#include <linux/trace_events.h>



unsigned int gCapturePriLayerEnable;
unsigned int gCaptureWdmaLayerEnable;
unsigned int gCapturePriLayerDownX = 20;
unsigned int gCapturePriLayerDownY = 20;
unsigned int gCapturePriLayerNum = 4;

struct dprec_logger logger[DPREC_LOGGER_NUM] = { { 0 } };

unsigned int dprec_error_log_len;
unsigned int dprec_error_log_buflen = DPREC_ERROR_LOG_BUFFER_LENGTH;
unsigned int dprec_error_log_id;

int dprec_init(void)
{
	return 0;
}

void dprec_event_op(enum DPREC_EVENT event)
{
}

void dprec_logger_trigger(unsigned int type_logsrc, unsigned int val1,
			  unsigned int val2)
{
}

unsigned long long
dprec_logger_get_current_hold_period(unsigned int type_logsrc)
{
	return 0;
}

void dprec_logger_start(unsigned int type_logsrc, unsigned int val1,
			unsigned int val2)
{
}

void dprec_logger_done(unsigned int type_logsrc, unsigned int val1,
		       unsigned int val2)
{
}

void dprec_logger_event_init(struct dprec_logger_event *p, char *name,
			     uint32_t level, mmp_event *mmp_root)
{
}

void dprec_logger_frame_seq_begin(unsigned int session_id,
				  unsigned int frm_sequence)
{
}

void dprec_logger_frame_seq_end(unsigned int session_id,
				unsigned int frm_sequence)
{
}

void dprec_start(struct dprec_logger_event *event, unsigned int val1,
		 unsigned int val2)
{
}

void dprec_done(struct dprec_logger_event *event, unsigned int val1,
		unsigned int val2)
{
}

void dprec_trigger(struct dprec_logger_event *event, unsigned int val1,
		   unsigned int val2)
{
}

void dprec_submit(struct dprec_logger_event *event, unsigned int val1,
		  unsigned int val2)
{
}

void dprec_logger_submit(unsigned int type_logsrc, unsigned long long period,
			 unsigned int fence_idx)
{
}

void dprec_logger_reset_all(void)
{
}

void dprec_logger_reset(enum DPREC_LOGGER_ENUM source)
{
}

int dprec_logger_get_result_string(enum DPREC_LOGGER_ENUM source,
				   char *stringbuf, int strlen)
{
	return 0;
}

int dprec_logger_get_result_string_all(char *stringbuf, int strlen)
{
	return 0;
}

void dprec_stub_irq(unsigned int irq_bit)
{
}

void dprec_stub_event(enum DISP_PATH_EVENT event)
{
}

unsigned int dprec_get_vsync_count(void)
{
	return 0;
}

void dprec_reg_op(void *cmdq, unsigned int reg, unsigned int val,
		  unsigned int mask)
{
}

void dprec_logger_vdump(const char *fmt, ...)
{
}

void dprec_logger_dump(char *string)
{
}

void dprec_logger_dump_reset(void)
{
}

char *dprec_logger_get_dump_addr()
{
	return NULL;
}

unsigned int dprec_logger_get_dump_len(void)
{
	return 0;
}

int dprec_handle_option(unsigned int option)
{
	return 0;
}

int dprec_option_enabled(void)
{
	return 0;
}

int dprec_mmp_dump_ovl_layer(struct OVL_CONFIG_STRUCT *ovl_layer,
			     unsigned int l, unsigned int session)
{
	return 0;
}

int dprec_mmp_dump_wdma_layer(void *wdma_layer, unsigned int wdma_num)
{
	return 0;
}

int dprec_mmp_dump_rdma_layer(void *rdma_layer, unsigned int rdma_num)
{
	return 0;
}

int dprec_logger_pr(unsigned int type, char *fmt, ...)
{
	return 0;
}

int dprec_logger_get_buf(enum DPREC_LOGGER_PR_TYPE type, char *stringbuf,
			 int len)
{
	return 0;
}

/* fix build error for add visual debug info */
int dprec_logger_get_result_value(enum DPREC_LOGGER_ENUM source,
				  struct fpsEx *fps)
{
	return 0;
}

char *get_dprec_status_ptr(int buffer_idx)
{
	return NULL;
}

char *debug_buffer;
bool is_buffer_init;

void init_log_buffer(void)
{
}

