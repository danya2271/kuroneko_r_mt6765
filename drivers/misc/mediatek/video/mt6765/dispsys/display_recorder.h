/* SPDX-License-Identifier: GPL-2.0 */
/*
 * The original display recorder duplicated printk, MMProfile and systrace
 * work in frame and IRQ paths. Keep source compatibility while compiling all
 * recorder operations away.
 */
#ifndef _DISPLAY_RECOREDR_H_
#define _DISPLAY_RECOREDR_H_

#include <linux/types.h>
#include "mmprofile.h"
#include "mmprofile_function.h"

#define LOGGER_BUFFER_SIZE (16 * 1024)
#define DEBUG_BUFFER_SIZE 1240

#undef mmprofile_log_ex
#define mmprofile_log_ex(...)			do { } while (0)

enum DPREC_LOGGER_ENUM {
	DPREC_LOGGER_ESD_RECOVERY,
	DPREC_LOGGER_ESD_CHECK,
	DPREC_LOGGER_ESD_CMDQ,
};

enum DPREC_LOGGER_PR_TYPE {
	DPREC_LOGGER_ERROR,
	DPREC_LOGGER_FENCE,
	DPREC_LOGGER_DEBUG,
	DPREC_LOGGER_DUMP,
	DPREC_LOGGER_STATUS,
};

extern unsigned int gCapturePriLayerEnable;
extern unsigned int gCaptureWdmaLayerEnable;
extern unsigned int gCaptureRdmaLayerEnable;
extern unsigned int gCapturePriLayerDownX;
extern unsigned int gCapturePriLayerDownY;
extern unsigned int gCapturePriLayerNum;

#define dprec_event_op(...)			do { } while (0)
#define dprec_reg_op(...)			do { } while (0)
#define dprec_handle_option(...)		(0)
#define dprec_option_enabled(...)		(0)
#define dprec_init(...)				(0)
#define dprec_logger_trigger(...)		do { } while (0)
#define dprec_logger_start(...)			do { } while (0)
#define dprec_logger_done(...)			do { } while (0)
#define dprec_logger_reset(...)			do { } while (0)
#define dprec_logger_reset_all(...)		do { } while (0)
#define dprec_logger_get_result_string(...)	(0)
#define dprec_logger_get_result_string_all(...)	(0)
#define dprec_logger_get_result_value(...)	(0)
#define dprec_stub_irq(...)			do { } while (0)
#define dprec_stub_event(...)			do { } while (0)
#define dprec_get_vsync_count(...)		(0U)
#define dprec_logger_submit(...)		do { } while (0)
#define dprec_logger_dump(...)			do { } while (0)
#define dprec_logger_vdump(...)			do { } while (0)
#define dprec_logger_dump_reset(...)		do { } while (0)
#define dprec_logger_get_dump_addr(...)		(NULL)
#define dprec_logger_get_dump_len(...)		(0U)
#define dprec_logger_get_current_hold_period(...) (0ULL)
#define dprec_logger_get_buf(...)		(0)
#define dprec_logger_pr(...)			(0)
#define dprec_logger_event_init(...)		do { } while (0)
#define dprec_start(...)			do { } while (0)
#define dprec_done(...)				do { } while (0)
#define dprec_trigger(...)			do { } while (0)
#define dprec_submit(...)			do { } while (0)
#define dprec_mmp_dump_wdma_layer(...)		(0)
#define dprec_mmp_dump_rdma_layer(...)		(0)
#define dprec_logger_frame_seq_begin(...)	do { } while (0)
#define dprec_logger_frame_seq_end(...)		do { } while (0)
#define dprec_mmp_dump_ovl_layer(...)		(0)
#define init_log_buffer(...)			do { } while (0)
#define get_dprec_status_ptr(...)		(NULL)

#define DISP_SYSTRACE_BEGIN(...)		do { } while (0)
#define DISP_SYSTRACE_END()			do { } while (0)
#define _DISP_TRACE_CNT(...)			do { } while (0)
#define DISP_TRACE_CNT(...)			do { } while (0)

#endif
