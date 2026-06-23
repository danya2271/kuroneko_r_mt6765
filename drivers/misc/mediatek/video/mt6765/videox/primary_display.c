// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/delay.h>
#include <linux/sched.h>
#include <uapi/linux/sched/types.h>
#include <linux/semaphore.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/ktime.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/pm_wakeup.h>
#include <linux/cpu_suspend.h>
#include <linux/printk.h>

#include "disp_drv_log.h"
#include "disp_drv_platform.h"
#ifdef MTK_FB_ION_SUPPORT
#include "mtk_ion.h"
#include "ion_drv.h"
#endif

#ifdef CONFIG_MTK_M4U
#include "m4u.h"
#include "m4u_priv.h"
#endif

#include "ddp_m4u.h"
#include "disp_lcm.h"
#include "disp_utils.h"
#include "mtkfb.h"
#include "ddp_hal.h"
#include "ddp_path.h"
#include "ddp_drv.h"
#include "ddp_reg.h"
#include "disp_session.h"
#include "primary_display.h"
#include "cmdq_def.h"
#include "cmdq_record.h"
#include "cmdq_reg.h"
#include "cmdq_core.h"
#include "ddp_rdma.h"
#include "ddp_manager.h"
#include "mtkfb_fence.h"
#include "mtk_sync.h"
#include "ddp_irq.h"
#include "disp_helper.h"
#include "mtk_disp_mgr.h"
#include "ddp_dsi.h"

#if defined(CONFIG_MTK_LEGACY)
#include <mach/mtk_gpio.h>
#include <cust_gpio_usage.h>
#else
#include "disp_dts_gpio.h"
#endif

#include "ddp_clkmgr.h"
#ifdef MTK_FB_MMDVFS_SUPPORT
#ifdef CONFIG_MTK_SMI_EXT
#include "mmdvfs_mgr.h"
#endif
#endif

#include "disp_lowpower.h"
#include "disp_recovery.h"
#include "ddp_od.h"
#include "layering_rule.h"
#include "disp_rect.h"
#include "disp_arr.h"
#include "disp_partial.h"
#include "ddp_aal.h"
#include "ddp_gamma.h"
#ifdef MTK_FB_MMDVFS_SUPPORT
#include <linux/soc/mediatek/mtk-pm-qos.h>
#endif
#include <linux/cpumask.h>

#define MMSYS_CLK_LOW (0)
#define MMSYS_CLK_HIGH (1)
#define TUI_SINGLE_WINDOW_MODE (0)
#define TUI_MULTIPLE_WINDOW_MODE (1)
#define FRM_UPDATE_SEQ_CACHE_NUM (DISP_INTERNAL_BUFFER_COUNT+1)

static struct disp_internal_buffer_info *decouple_buffer_info[DISP_INTERNAL_BUFFER_COUNT];
static struct RDMA_CONFIG_STRUCT decouple_rdma_config;
static struct WDMA_CONFIG_STRUCT decouple_wdma_config;
static struct disp_mem_output_config mem_config;
atomic_t hwc_configing = ATOMIC_INIT(0);
static unsigned int primary_session_id = MAKE_DISP_SESSION(DISP_SESSION_PRIMARY, 0);
static struct disp_frm_seq_info frm_update_sequence[FRM_UPDATE_SEQ_CACHE_NUM];
static unsigned int frm_update_cnt;
static unsigned int gPresentFenceIndex;
unsigned int gTriggerDispMode;
static unsigned int g_keep;
static unsigned int g_skip;
int g_idle_skip;
int g_idle_skip_trigger;
bool g_force_cfg;
unsigned int g_force_cfg_id;

static struct hrtimer cmd_mode_update_timer;
static int is_fake_timer_inited;
extern int lcm_mode_status;

static struct task_struct *primary_display_switch_dst_mode_task;
#ifdef MTK_FB_ION_SUPPORT
static struct task_struct *present_fence_release_worker_task;
#endif
static struct task_struct *primary_path_aal_task;
static struct task_struct *primary_delay_trigger_task;
static struct task_struct *primary_od_trigger_task;
static struct task_struct *decouple_update_rdma_config_thread;
static struct task_struct *decouple_trigger_thread;
static struct task_struct *init_decouple_buffer_thread;

#ifdef MTK_FB_MMDVFS_SUPPORT
struct mtk_pm_qos_request primary_display_qos_request;
struct mtk_pm_qos_request primary_display_emi_opp_request;
struct mtk_pm_qos_request primary_display_mm_freq_request;
#endif

static int decouple_mirror_update_rdma_config_thread(void *data);
static int decouple_trigger_worker_thread(void *data);

struct task_struct *primary_display_frame_update_task;
wait_queue_head_t primary_display_frame_update_wq;
atomic_t primary_display_frame_update_event = ATOMIC_INIT(0);
enum DISP_PRIMARY_PATH_MODE primary_display_mode = DIRECT_LINK_MODE;
int primary_display_def_dst_mode;
int primary_display_cur_dst_mode;
unsigned long long last_primary_trigger_time;
bool is_switched_dst_mode;
int primary_trigger_cnt;
unsigned int dynamic_fps_changed;
unsigned int arr_fps_enable;
unsigned int arr_fps_backup;
atomic_t decouple_update_rdma_event = ATOMIC_INIT(0);
DECLARE_WAIT_QUEUE_HEAD(decouple_update_rdma_wq);
atomic_t decouple_trigger_event = ATOMIC_INIT(0);
DECLARE_WAIT_QUEUE_HEAD(decouple_trigger_wq);
static bool pf_thread_init;
wait_queue_head_t primary_display_present_fence_wq;
atomic_t primary_display_pt_fence_update_event = ATOMIC_INIT(0);
static unsigned int _need_lfr_check(void);

#ifdef CONFIG_MTK_DISPLAY_120HZ_SUPPORT
static int od_need_start;
#endif

#ifdef MTK_FB_MMDVFS_SUPPORT
static int dvfs_last_ovl_req = HRT_LEVEL_NUM - 1;
static unsigned int ovl_throughput_freq_req;
#endif

static atomic_t delayed_trigger_kick = ATOMIC_INIT(0);
static atomic_t od_trigger_kick = ATOMIC_INIT(0);

unsigned int round_corner_offset_enable;
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
unsigned int lcm_corner_en;
unsigned int full_content;
unsigned int top_mva, bottom_mva;
unsigned int corner_pattern_width, corner_pattern_height;
unsigned int corner_pattern_height_bot;
static int primary_display_get_round_corner_mva(unsigned int *tp_mva, unsigned int *bt_mva, unsigned int *pitch, unsigned int *height, unsigned int *height_bot);
#endif

struct wakeup_source *pri_wk_lock;

static int smart_ovl_try_switch_mode_nolock(void);

static inline void disp_layer_info_statistic(struct disp_ddp_path_config *config, struct disp_frame_cfg_t *cfg) {}

static int init_cmdq_slots(cmdqBackupSlotHandle *pSlot, int count, int init_val)
{
	int i;
	cmdqBackupAllocateSlot(pSlot, count);
	for (i = 0; i < count; i++) cmdqBackupWriteSlot(*pSlot, i, init_val);
	return 0;
}

static inline int _convert_disp_input_to_ovl(struct OVL_CONFIG_STRUCT *dst, struct disp_input_config *src)
{
	enum UNIFIED_COLOR_FMT tmp_fmt;
	unsigned int Bpp = 0;

	if (!src || !dst) return -1;

	dst->layer = src->layer_id;
	dst->isDirty = 1;
	dst->buff_idx = src->next_buff_idx;
	dst->layer_en = src->layer_enable;

	if (!src->layer_enable) return 0;

	tmp_fmt = disp_fmt_to_unified_fmt(src->src_fmt);
	ufmt_disable_X_channel(tmp_fmt, &dst->fmt, &dst->const_bld);

	Bpp = UFMT_GET_Bpp(dst->fmt);

	dst->addr = (unsigned long)(src->src_phy_addr);
	dst->vaddr = (unsigned long)(src->src_base_addr);
	dst->src_x = src->src_offset_x;
	dst->src_y = src->src_offset_y;
	dst->src_w = src->src_width;
	dst->src_h = src->src_height;
	dst->src_pitch = src->src_pitch * Bpp;
	dst->dst_x = src->tgt_offset_x;
	dst->dst_y = src->tgt_offset_y;

	dst->dst_w = min(src->src_width, src->tgt_width);
	dst->dst_h = min(src->src_height, src->tgt_height);

	dst->keyEn = src->src_use_color_key;
	dst->key = src->src_color_key;
	dst->aen = src->alpha_enable;
	dst->sur_aen = src->sur_aen;
	dst->alpha = src->alpha;
	dst->src_alpha = src->src_alpha;
	dst->dst_alpha = src->dst_alpha;

	dst->identity = src->identity;
	dst->connected_type = src->connected_type;
	dst->security = src->security;
	dst->yuv_range = src->yuv_range;

	if (src->buffer_source == DISP_BUFFER_ALPHA) {
		dst->source = OVL_LAYER_SOURCE_RESERVED;
	} else if (src->buffer_source == DISP_BUFFER_ION || src->buffer_source == DISP_BUFFER_MVA) {
		dst->source = OVL_LAYER_SOURCE_MEM;
	} else {
		dst->source = OVL_LAYER_SOURCE_MEM;
	}

	dst->ext_sel_layer = src->ext_sel_layer;

	return 0;
}

struct display_primary_path_context *_get_context(void)
{
	static int is_context_inited;
	static struct display_primary_path_context g_context;

	if (!is_context_inited) {
		memset(&g_context, 0, sizeof(g_context));
		is_context_inited = 1;
#ifdef CONFIG_MTK_HIGH_FRAME_RATE
		g_context.first_cfg = 1;
#endif
	}

	return &g_context;
}

void _primary_path_lock(const char *caller)
{
	disp_sw_mutex_lock(&(pgc->lock));
	pgc->mutex_locker = (char *)caller;
}

void _primary_path_unlock(const char *caller)
{
	pgc->mutex_locker = NULL;
	disp_sw_mutex_unlock(&(pgc->lock));
}

int primary_display_is_directlink_mode(void)
{
	return (pgc->session_mode == DISP_SESSION_DIRECT_LINK_MIRROR_MODE || pgc->session_mode == DISP_SESSION_DIRECT_LINK_MODE);
}

enum DISP_MODE primary_get_sess_mode(void) { return pgc->session_mode; }
unsigned int primary_get_sess_id(void) { return pgc->session_id; }
struct disp_lcm_handle *primary_get_lcm(void) { return pgc->plcm; }
void *primary_get_dpmgr_handle(void) { return pgc->dpmgr_handle; }
void *primary_get_ovl2mem_handle(void) { return pgc->ovl2mem_path_handle; }

int primary_display_is_decouple_mode(void)
{
	return (pgc->session_mode == DISP_SESSION_DECOUPLE_MODE || pgc->session_mode == DISP_SESSION_DECOUPLE_MIRROR_MODE);
}

int _is_mirror_mode(enum DISP_MODE mode)
{
	return (mode == DISP_SESSION_DIRECT_LINK_MIRROR_MODE || mode == DISP_SESSION_DECOUPLE_MIRROR_MODE);
}

int primary_display_is_mirror_mode(void) { return _is_mirror_mode(pgc->session_mode); }
int primary_is_sec(void) { return pgc->is_primary_sec; }

void _primary_path_switch_dst_lock(void) { mutex_lock(&(pgc->switch_dst_lock)); }
void _primary_path_switch_dst_unlock(void) { mutex_unlock(&(pgc->switch_dst_lock)); }

int primary_display_config_full_roi(struct disp_ddp_path_config *pconfig, disp_path_handle disp_handle, struct cmdqRecStruct *cmdq_handle)
{
	struct disp_rect total_dirty_roi = { 0, 0, 0, 0};

	if (!disp_partial_is_support()) return -1;

	assign_full_lcm_roi(&total_dirty_roi);
	if (!rect_equal(&total_dirty_roi, &pconfig->ovl_partial_roi)) {
		pconfig->ovl_partial_roi = total_dirty_roi;
		dpmgr_path_update_partial_roi(disp_handle, total_dirty_roi, cmdq_handle);
		if (disp_helper_get_option(DISP_OPT_DYNAMIC_RDMA_GOLDEN_SETTING)) {
			set_rdma_width_height(total_dirty_roi.width, total_dirty_roi.height);
			dpmgr_path_ioctl(disp_handle, cmdq_handle, DDP_RDMA_GOLDEN_SETTING, pconfig);
		}
		pconfig->ovl_dirty = 1;
		pconfig->ovl_partial_dirty = 0;
		dpmgr_path_config(disp_handle, pconfig, cmdq_handle);
		pconfig->ovl_layer_scanned = 0;
		pconfig->ovl_partial_dirty = 0;
		pconfig->ovl_dirty = 0;
	}
	return 0;
}

static int _disp_primary_path_switch_dst_mode_thread(void *data)
{
	while (1) {
		msleep(1000);
		if (((sched_clock() - last_primary_trigger_time) / 1000) > 500000) {
			primary_display_switch_dst_mode(0);
			is_switched_dst_mode = true;
		}
		if (kthread_should_stop()) break;
	}
	return 0;
}

static DECLARE_WAIT_QUEUE_HEAD(display_state_wait_queue);

enum DISP_POWER_STATE primary_get_state(void) { return pgc->state; }
static enum DISP_POWER_STATE primary_set_state(enum DISP_POWER_STATE new_state)
{
	enum DISP_POWER_STATE old_state = pgc->state;
	pgc->state = new_state;
	wake_up(&display_state_wait_queue);
	return old_state;
}

enum mtkfb_power_mode primary_display_set_power_mode_nolock(enum mtkfb_power_mode new_mode)
{
	enum mtkfb_power_mode prev_mode = pgc->pm;
	pgc->pm = new_mode;
	return prev_mode;
}

enum mtkfb_power_mode primary_display_set_power_mode(enum mtkfb_power_mode new_mode)
{
	enum mtkfb_power_mode prev_mode;
	_primary_path_lock(__func__);
	prev_mode = primary_display_set_power_mode_nolock(new_mode);
	_primary_path_unlock(__func__);
	return prev_mode;
}

enum mtkfb_power_mode primary_display_get_power_mode_nolock(void) { return pgc->pm; }

enum mtkfb_power_mode primary_display_get_power_mode(void)
{
	enum mtkfb_power_mode mode = MTKFB_POWER_MODE_UNKNOWN;
	_primary_path_lock(__func__);
	mode = primary_display_get_power_mode_nolock();
	_primary_path_unlock(__func__);
	return mode;
}

bool primary_is_aod_supported(void)
{
	return (disp_helper_get_option(DISP_OPT_AOD) && !disp_lcm_is_video_mode(pgc->plcm));
}

enum lcm_power_state primary_display_set_lcm_power_state_nolock(enum lcm_power_state new_state)
{
	enum lcm_power_state prev_state = pgc->lcm_ps;
	pgc->lcm_ps = new_state;
	return prev_state;
}

enum lcm_power_state primary_display_set_power_state(enum lcm_power_state new_state)
{
	enum lcm_power_state prev_state;
	_primary_path_lock(__func__);
	prev_state = primary_display_set_lcm_power_state_nolock(new_state);
	_primary_path_unlock(__func__);
	return prev_state;
}

enum lcm_power_state primary_display_get_lcm_power_state_nolock(void) { return pgc->lcm_ps; }

enum lcm_power_state primary_display_get_lcm_power_state(void)
{
	enum lcm_power_state ps;
	_primary_path_lock(__func__);
	ps = primary_display_get_lcm_power_state_nolock();
	_primary_path_unlock(__func__);
	return ps;
}

enum mtkfb_power_mode primary_display_check_power_mode(void)
{
	if (primary_display_is_sleepd() && (primary_display_get_lcm_power_state() == LCM_OFF)) return FB_SUSPEND;
	else if (primary_display_is_alive() && (primary_display_get_lcm_power_state() == LCM_ON)) return FB_RESUME;
	else if (primary_display_is_sleepd() && (primary_display_get_lcm_power_state() == LCM_ON_LOW_POWER)) return DOZE_SUSPEND;
	else if (primary_display_is_alive() && (primary_display_get_lcm_power_state() == LCM_ON_LOW_POWER)) return DOZE;

	return MTKFB_POWER_MODE_UNKNOWN;
}

void debug_print_power_mode_check(enum mtkfb_power_mode prev, enum mtkfb_power_mode cur) {}

#define __primary_display_wait_state(condition, timeout) wait_event_timeout(display_state_wait_queue, condition, timeout)

long primary_display_wait_state(enum DISP_POWER_STATE state, long timeout) { return __primary_display_wait_state(primary_get_state() == state, timeout); }
long primary_display_wait_not_state(enum DISP_POWER_STATE state, long timeout) { return __primary_display_wait_state(primary_get_state() != state, timeout); }

#define FPS_ARRAY_SZ 30
struct fps_ctx_t {
	unsigned long long last_trig;
	unsigned int array[FPS_ARRAY_SZ];
	unsigned int total;
	unsigned int wnd_sz;
	unsigned int cur_wnd_sz;
	struct mutex lock;
	int is_inited;
} primary_fps_ctx;

#define INTERVAL_MS 10000
struct fps_ext_ctx_t {
	unsigned long long last_trig;
	unsigned long long total;
	unsigned long long interval;
	unsigned int fps;
	unsigned int stable;
	unsigned int frequency;
	struct mutex lock;
	int is_inited;
} primary_fps_ext_ctx;

static int _fps_ctx_reset(struct fps_ctx_t *fps_ctx, int reserve_num)
{
	int i;
	if (reserve_num >= FPS_ARRAY_SZ) reserve_num = FPS_ARRAY_SZ - 1;
	for (i = reserve_num; i < FPS_ARRAY_SZ; i++) fps_ctx->array[i] = 0;
	if (reserve_num < fps_ctx->cur_wnd_sz) fps_ctx->cur_wnd_sz = reserve_num;
	fps_ctx->total = 0;
	for (i = 0; i < fps_ctx->cur_wnd_sz; i++) fps_ctx->total += fps_ctx->array[i];
	return 0;
}

static int _fps_ctx_update(struct fps_ctx_t *fps_ctx, unsigned int fps, unsigned long long time_ns)
{
	int i;
	fps_ctx->total -= fps_ctx->array[fps_ctx->wnd_sz-1];
	for (i = fps_ctx->wnd_sz - 1; i > 0; i--) fps_ctx->array[i] = fps_ctx->array[i-1];
	fps_ctx->array[0] = fps;
	fps_ctx->total += fps_ctx->array[0];
	if (fps_ctx->cur_wnd_sz < fps_ctx->wnd_sz) fps_ctx->cur_wnd_sz++;
	fps_ctx->last_trig = time_ns;
	return 0;
}

static int fps_ctx_init(struct fps_ctx_t *fps_ctx, int wnd_sz)
{
	if (fps_ctx->is_inited) return 0;
	memset(fps_ctx, 0, sizeof(*fps_ctx));
	mutex_init(&fps_ctx->lock);
	if (wnd_sz > FPS_ARRAY_SZ) wnd_sz = FPS_ARRAY_SZ;
	fps_ctx->wnd_sz = wnd_sz;
	fps_ctx->is_inited = 1;
	return 0;
}

static inline unsigned int _fps_ctx_calc_cur_fps(struct fps_ctx_t *fps_ctx, unsigned long long cur_ns)
{
	unsigned long long delta;
	unsigned long long fps = 1000000000;
	if (cur_ns > fps_ctx->last_trig) {
		delta = cur_ns - fps_ctx->last_trig;
		do_div(fps, delta);
	}
	if (fps > 120ULL) fps = 120ULL;
	return (unsigned int)fps;
}

static inline unsigned int _fps_ctx_get_avg_fps(struct fps_ctx_t *fps_ctx)
{
	if (fps_ctx->cur_wnd_sz == 0) return 0;
	return fps_ctx->total / fps_ctx->cur_wnd_sz;
}

static inline unsigned int _fps_ctx_get_avg_fps_ext(struct fps_ctx_t *fps_ctx, unsigned int abs_fps)
{
	return (fps_ctx->total + abs_fps) / (fps_ctx->cur_wnd_sz + 1);
}

static int fps_ctx_update(struct fps_ctx_t *fps_ctx)
{
	unsigned int abs_fps, avg_fps;
	unsigned long long ns = sched_clock();

	mutex_lock(&fps_ctx->lock);
	abs_fps = _fps_ctx_calc_cur_fps(fps_ctx, ns);
	avg_fps = _fps_ctx_get_avg_fps(fps_ctx);

	if (abs_fps < avg_fps / 2 && avg_fps > 10) _fps_ctx_reset(fps_ctx, 0);

	_fps_ctx_update(fps_ctx, abs_fps, ns);
	mutex_unlock(&fps_ctx->lock);
	return 0;
}

static int fps_ctx_get_fps(struct fps_ctx_t *fps_ctx, unsigned int *fps, int *stable)
{
	unsigned long long ns = sched_clock();
	unsigned int abs_fps = 0, avg_fps = 0;

	*stable = 1;
	mutex_lock(&fps_ctx->lock);

	abs_fps = _fps_ctx_calc_cur_fps(fps_ctx, ns);
	avg_fps = _fps_ctx_get_avg_fps(fps_ctx);

	if (abs_fps < avg_fps/2 && avg_fps > 10) {
		_fps_ctx_reset(fps_ctx, 0);
		*fps = abs_fps;
		*stable = 0;
		goto done;
	}

	if (fps_ctx->cur_wnd_sz < fps_ctx->wnd_sz/2) *stable = 0;

	if (abs_fps < avg_fps) *fps = _fps_ctx_get_avg_fps_ext(fps_ctx, abs_fps);
	else *fps = avg_fps;

done:
	mutex_unlock(&fps_ctx->lock);
	return 0;
}

static int fps_ctx_set_wnd_sz(struct fps_ctx_t *fps_ctx, unsigned int wnd_sz)
{
	int i;
	if (!fps_ctx->is_inited) return fps_ctx_init(fps_ctx, wnd_sz);
	if (wnd_sz > FPS_ARRAY_SZ) return -1;

	mutex_lock(&fps_ctx->lock);
	fps_ctx->total = 0;
	fps_ctx->wnd_sz = wnd_sz;
	for (i = 0; i < wnd_sz; i++) fps_ctx->total += fps_ctx->array[i];
	mutex_unlock(&fps_ctx->lock);
	return 0;
}

static int fps_ext_ctx_init(struct fps_ext_ctx_t *fps_ctx, unsigned int ms)
{
	if (!disp_helper_get_option(DISP_OPT_FPS_EXT) || !fps_ctx || ms == 0 || ms > INTERVAL_MS) return -1;
	if (fps_ctx->is_inited) return 0;

	memset(fps_ctx, 0, sizeof(*fps_ctx));
	mutex_init(&fps_ctx->lock);
	fps_ctx->interval = (unsigned long long)ms * 1000;
	fps_ctx->frequency = INTERVAL_MS / ms;
	fps_ctx->is_inited = 1;

	return 0;
}

void (*display_time_fps_stablizer)(unsigned long long ts);

static int fps_ext_ctx_update(struct fps_ext_ctx_t *fps_ctx)
{
	unsigned long long ns = sched_clock();
	unsigned long long delta_us, fps;

	if (!disp_helper_get_option(DISP_OPT_FPS_EXT) || !fps_ctx) return -1;
	if (display_time_fps_stablizer) display_time_fps_stablizer(ns);

	mutex_lock(&fps_ctx->lock);
	fps_ctx->total++;

	delta_us = ns - fps_ctx->last_trig;
	do_div(delta_us, 1000);
	if (delta_us > fps_ctx->interval) {
		fps = fps_ctx->interval * fps_ctx->total * fps_ctx->frequency;
		do_div(fps, delta_us);
		do_div(fps, INTERVAL_MS / 1000);
		fps_ctx->fps = (unsigned int)fps;
		fps_ctx->last_trig = ns;
		fps_ctx->total = 0;
		fps_ctx->stable = 0;
	}
	mutex_unlock(&fps_ctx->lock);

	return 0;
}

static int fps_ext_ctx_get_fps(struct fps_ext_ctx_t *fps_ctx, unsigned int *fps, int *stable)
{
	if (!disp_helper_get_option(DISP_OPT_FPS_EXT) || !fps_ctx || !fps || !stable) return -1;

	mutex_lock(&fps_ctx->lock);
	*fps = fps_ctx->fps;
	*stable = fps_ctx->stable;
	mutex_unlock(&fps_ctx->lock);

	return 0;
}

static int fps_ext_ctx_set_interval(struct fps_ext_ctx_t *fps_ctx, unsigned int ms)
{
	if (!fps_ctx || ms == 0 || ms > INTERVAL_MS) return -1;
	if (!fps_ctx->is_inited) return fps_ext_ctx_init(fps_ctx, ms);

	mutex_lock(&fps_ctx->lock);
	fps_ctx->total = 0;
	fps_ctx->interval = (unsigned long long)ms * 1000;
	fps_ctx->frequency = INTERVAL_MS / ms;
	mutex_unlock(&fps_ctx->lock);
	return 0;
}

int primary_fps_ctx_set_wnd_sz(unsigned int wnd_sz) { return fps_ctx_set_wnd_sz(&primary_fps_ctx, wnd_sz); }
int primary_fps_ctx_get_fps(unsigned int *fps, int *stable) { return fps_ctx_get_fps(&primary_fps_ctx, fps, stable); }
int primary_fps_ext_ctx_set_interval(unsigned int interval) { return fps_ext_ctx_set_interval(&primary_fps_ext_ctx, interval); }

#ifdef MTK_FB_MMDVFS_SUPPORT
int primary_display_get_dvfs_last_req(void) { return dvfs_last_ovl_req; }
#endif

int primary_display_get_debug_state(char *stringbuf, int buf_len) { return 0; }

enum DISP_MODULE_ENUM _get_dst_module_by_lcm(struct disp_lcm_handle *plcm)
{
	if (!plcm) return DISP_MODULE_UNKNOWN;
	if (plcm->params->type == LCM_TYPE_DSI) {
		if (plcm->lcm_if_id == LCM_INTERFACE_DSI0) return DISP_MODULE_DSI0;
		else if (plcm->lcm_if_id == LCM_INTERFACE_DSI1) return DISP_MODULE_DSI1;
		else if (plcm->lcm_if_id == LCM_INTERFACE_DSI_DUAL) return DISP_MODULE_DSIDUAL;
		else return DISP_MODULE_DSI0;
	} else if (plcm->params->type == LCM_TYPE_DPI) {
		return DISP_MODULE_DPI;
	}
	return DISP_MODULE_UNKNOWN;
}

int _should_wait_path_idle(void) { return primary_display_cmdq_enabled() ? 0 : dpmgr_path_is_busy(pgc->dpmgr_handle); }
int _should_update_lcm(void) { return primary_display_cmdq_enabled() || primary_display_is_video_mode() ? 0 : 1; }
int _should_start_path(void) { return primary_display_cmdq_enabled() ? 0 : (primary_display_is_video_mode() ? dpmgr_path_is_idle(pgc->dpmgr_handle) : 1); }
int _should_trigger_path(void) { return primary_display_cmdq_enabled() ? 0 : (primary_display_is_video_mode() ? dpmgr_path_is_idle(pgc->dpmgr_handle) : 1); }
int _should_set_cmdq_dirty(void) { return primary_display_cmdq_enabled() && !primary_display_is_video_mode(); }
int _should_flush_cmdq_config_handle(void) { return primary_display_cmdq_enabled(); }
int _should_reset_cmdq_config_handle(void) { return primary_display_cmdq_enabled(); }
int _should_insert_wait_frame_done_token(void) { return primary_display_cmdq_enabled(); }
int _should_trigger_interface(void) { return pgc->mode == DECOUPLE_MODE ? 0 : 1; }
int _should_config_ovl_input(void) { return (pgc->mode == SINGLE_LAYER_MODE || pgc->mode == DEBUG_RDMA1_DSI0_MODE) ? 0 : 1; }

static enum hrtimer_restart _DISP_CmdModeTimer_handler(struct hrtimer *timer)
{
	dpmgr_signal_event(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC);
	hrtimer_forward_now(timer, ns_to_ktime(16666666));
	return HRTIMER_RESTART;
}

int _init_vsync_fake_monitor(int fps)
{
	if (is_fake_timer_inited) return 0;
	is_fake_timer_inited = 1;
	hrtimer_init(&cmd_mode_update_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	cmd_mode_update_timer.function = _DISP_CmdModeTimer_handler;
	hrtimer_start(&cmd_mode_update_timer, ns_to_ktime(16666666), HRTIMER_MODE_REL);
	return 0;
}

static void change_dsi_vfp(struct cmdqRecStruct *handle, unsigned int fps)
{
#ifdef CONFIG_MTK_HIGH_FRAME_RATE
	unsigned int fps_levels = 0, vfp_to_set = 0, i = 0;
	struct dynamic_fps_info *p_fps_table = NULL;
	unsigned int VFP_PORTCH = pgc->plcm->params->dsi.vertical_frontporch;
	unsigned int line_num = VFP_PORTCH + pgc->plcm->params->dsi.vertical_sync_active + pgc->plcm->params->dsi.vertical_backporch + pgc->plcm->params->dsi.vertical_active_line;

	if (fps < 60) VFP_PORTCH = (line_num * 60) / fps - line_num + pgc->plcm->params->dsi.vertical_frontporch;

	fps_levels = pgc->plcm->params->dsi.dynamic_fps_levels;
	p_fps_table = pgc->plcm->params->dsi.dynamic_fps_table;

	if (!p_fps_table) {
		vfp_to_set = VFP_PORTCH;
	} else {
		for (i = 0; i < fps_levels; i++) {
			if (fps == p_fps_table[i].fps) {
				vfp_to_set = p_fps_table[i].vfp;
				break;
			}
		}
		if (i == fps_levels) vfp_to_set = VFP_PORTCH;
	}

	cmdqRecBackupUpdateSlot(handle, pgc->dsi_vfp_line, 0, vfp_to_set);
	cmdqRecBackupUpdateSlot(handle, pgc->next_working_fps, 0, fps);
	cmdqRecBackupUpdateSlot(handle, pgc->dsi_vfp_changed, 0, 1);
#else
	unsigned int VFP_PORTCH = pgc->plcm->params->dsi.vertical_frontporch;
	unsigned int line_num = VFP_PORTCH + pgc->plcm->params->dsi.vertical_sync_active + pgc->plcm->params->dsi.vertical_backporch + pgc->plcm->params->dsi.vertical_active_line;

	if (fps < 60) VFP_PORTCH = (line_num * 60) / fps - line_num + pgc->plcm->params->dsi.vertical_frontporch;
	cmdqRecBackupUpdateSlot(handle, pgc->dsi_vfp_line, 0, VFP_PORTCH);
#endif
}

static void _cmdq_set_config_handle_dirty(void)
{
	if (!primary_display_is_video_mode()) cmdqRecSetEventToken(pgc->cmdq_handle_config, CMDQ_SYNC_TOKEN_CONFIG_DIRTY);
}

static void _cmdq_handle_clear_dirty(struct cmdqRecStruct *cmdq_handle)
{
	if (!primary_display_is_video_mode()) cmdqRecClearEventToken(cmdq_handle, CMDQ_SYNC_TOKEN_CONFIG_DIRTY);
}

static void _cmdq_set_config_handle_dirty_mira(void *handle)
{
	if (!primary_display_is_video_mode()) cmdqRecSetEventToken(handle, CMDQ_SYNC_TOKEN_CONFIG_DIRTY);
}

static void _cmdq_reset_config_handle(void)
{
	cmdqRecReset(pgc->cmdq_handle_config);
}

static void _cmdq_flush_config_handle(int blocking, CmdqAsyncFlushCB callback, unsigned int userdata)
{
	if (blocking) cmdqRecFlush(pgc->cmdq_handle_config);
	else if (callback) cmdqRecFlushAsyncCallback(pgc->cmdq_handle_config, callback, userdata);
	else cmdqRecFlushAsync(pgc->cmdq_handle_config);
}

static void _cmdq_flush_config_handle_mira(void *handle, int blocking)
{
	if (blocking) cmdqRecFlush(handle);
	else cmdqRecFlushAsync(handle);
}

void _cmdq_insert_wait_primary_path_frame_done(void *handle)
{
	if (primary_display_is_video_mode()) cmdqRecWaitNoClear(handle, CMDQ_EVENT_MUTEX0_STREAM_EOF);
	else cmdqRecWaitNoClear(handle, CMDQ_SYNC_TOKEN_STREAM_EOF);
}

void _cmdq_insert_wait_frame_done_token_mira(void *handle)
{
	if (primary_display_is_video_mode()) {
		cmdqRecWaitNoClear(handle, CMDQ_EVENT_MUTEX0_STREAM_EOF);
		ddp_mutex_set_sof_wait(dpmgr_path_get_mutex(pgc->dpmgr_handle), handle, 0);
	} else {
		cmdqRecWaitNoClear(handle, CMDQ_SYNC_TOKEN_STREAM_EOF);
	}
}

static void _cmdq_build_trigger_loop(void)
{
	int ret = 0;
	unsigned long dsi_vfp_addr[2] = {0, 0};

	if (primary_display_is_video_mode() && disp_helper_get_option(DISP_OPT_ARR_PHASE_1)) {
		int VFP_PORTCH = pgc->plcm->params->dsi.vertical_frontporch;
		unsigned int gsync_fps = (pgc->dynamic_fps ? pgc->dynamic_fps : 60);
		int line_num = VFP_PORTCH + pgc->plcm->params->dsi.vertical_sync_active + pgc->plcm->params->dsi.vertical_backporch + pgc->plcm->params->dsi.vertical_active_line;

		if (gsync_fps < 60) VFP_PORTCH = (line_num * 60) / gsync_fps - line_num + pgc->plcm->params->dsi.vertical_frontporch;
		cmdqBackupWriteSlot(pgc->dsi_vfp_line, 0, VFP_PORTCH);
	}

	if (pgc->cmdq_handle_trigger == NULL) cmdqRecCreate(CMDQ_SCENARIO_TRIGGER_LOOP, &(pgc->cmdq_handle_trigger));

	cmdqRecReset(pgc->cmdq_handle_trigger);

	if (primary_display_is_video_mode()) {
		ddp_mutex_set_sof_wait(dpmgr_path_get_mutex(pgc->dpmgr_handle), pgc->cmdq_handle_trigger, 0);
		cmdqRecWait(pgc->cmdq_handle_trigger, CMDQ_EVENT_MUTEX0_STREAM_EOF);
		cmdqRecClearEventToken(pgc->cmdq_handle_trigger, CMDQ_EVENT_DISP_RDMA0_SOF);
		dpmgr_path_build_cmdq(pgc->dpmgr_handle, pgc->cmdq_handle_trigger, CMDQ_AFTER_STREAM_EOF, 0);

		if (disp_helper_get_option(DISP_OPT_ARR_PHASE_1)) {
			unsigned int addr;
			ret = cmdqRecWait(pgc->cmdq_handle_trigger, CMDQ_EVENT_DISP_RDMA0_SOF);
			dpmgr_path_ioctl(pgc->dpmgr_handle, pgc->cmdq_handle_trigger, DDP_DSI_PORCH_ADDR, (void *)(dsi_vfp_addr));
			addr = (unsigned int)(dsi_vfp_addr[0] & 0x0FFFFFFFF);
			if (dsi_vfp_addr[0]) cmdqRecBackupWriteRegisterFromSlot(pgc->cmdq_handle_trigger, pgc->dsi_vfp_line, 0, addr);

			addr = (unsigned int)(dsi_vfp_addr[1] & 0x0FFFFFFFF);
			if ((_get_dst_module_by_lcm(pgc->plcm) == DISP_MODULE_DSIDUAL) && dsi_vfp_addr[1])
				cmdqRecBackupWriteRegisterFromSlot(pgc->cmdq_handle_trigger, pgc->dsi_vfp_line, 0, addr);
		}
	} else {
		ret = cmdqRecWait(pgc->cmdq_handle_trigger, CMDQ_SYNC_TOKEN_CONFIG_DIRTY);

		if (need_wait_esd_eof()) ret = cmdqRecWaitNoClear(pgc->cmdq_handle_trigger, CMDQ_SYNC_TOKEN_ESD_EOF);

#ifndef CONFIG_FPGA_EARLY_PORTING
		if (islcmconnected) dpmgr_path_build_cmdq(pgc->dpmgr_handle, pgc->cmdq_handle_trigger, CMDQ_WAIT_LCM_TE, 0);
#endif
		ret = cmdqRecWaitNoClear(pgc->cmdq_handle_trigger, CMDQ_SYNC_TOKEN_CABC_EOF);
		ret = cmdqRecClearEventToken(pgc->cmdq_handle_trigger, CMDQ_SYNC_TOKEN_STREAM_EOF);
		ret = cmdqRecClearEventToken(pgc->cmdq_handle_trigger, CMDQ_SYNC_TOKEN_CONFIG_DIRTY);
		ret = cmdqRecClearEventToken(pgc->cmdq_handle_trigger, CMDQ_EVENT_DISP_RDMA0_EOF);

		dpmgr_path_build_cmdq(pgc->dpmgr_handle, pgc->cmdq_handle_trigger, CMDQ_BEFORE_STREAM_SOF, 0);
		dpmgr_path_trigger(pgc->dpmgr_handle, pgc->cmdq_handle_trigger, CMDQ_ENABLE);
		dpmgr_path_build_cmdq(pgc->dpmgr_handle, pgc->cmdq_handle_trigger, CMDQ_AFTER_STREAM_SOF, 1);

		ret = cmdqRecWait(pgc->cmdq_handle_trigger, CMDQ_EVENT_DISP_DSI0_EOF);
		ret = cmdqRecWait(pgc->cmdq_handle_trigger, CMDQ_EVENT_DSI0_DONE_EVENT);

		dpmgr_path_build_cmdq(pgc->dpmgr_handle, pgc->cmdq_handle_trigger, CMDQ_WAIT_STREAM_EOF_EVENT, 0);
		dpmgr_path_build_cmdq(pgc->dpmgr_handle, pgc->cmdq_handle_trigger, CMDQ_CHECK_IDLE_AFTER_STREAM_EOF, 0);
		dpmgr_path_build_cmdq(pgc->dpmgr_handle, pgc->cmdq_handle_trigger, CMDQ_AFTER_STREAM_EOF, 0);
		dpmgr_path_build_cmdq(pgc->dpmgr_handle, pgc->cmdq_handle_trigger, CMDQ_RESET_AFTER_STREAM_EOF, 0);

		ret = cmdqRecSetEventToken(pgc->cmdq_handle_trigger, CMDQ_SYNC_TOKEN_STREAM_EOF);
		ret = cmdqRecSetEventToken(pgc->cmdq_handle_trigger, CMDQ_SYNC_TOKEN_CABC_EOF);
	}
}

void disp_spm_enter_cg_mode(void) {}
void disp_spm_enter_power_down_mode(void) {}

void _cmdq_start_trigger_loop(void)
{
	cmdqRecStartLoop(pgc->cmdq_handle_trigger);
	if (!primary_display_is_video_mode()) {
		if (need_wait_esd_eof()) cmdqCoreSetEvent(CMDQ_SYNC_TOKEN_ESD_EOF);
		cmdqCoreSetEvent(CMDQ_SYNC_TOKEN_STREAM_EOF);
		cmdqCoreSetEvent(CMDQ_SYNC_TOKEN_CABC_EOF);
		cmdqCoreSetEvent(CMDQ_EVENT_DISP_WDMA0_EOF);
	}
}

void _cmdq_stop_trigger_loop(void)
{
	cmdqRecStopLoop(pgc->cmdq_handle_trigger);
}

static void update_frm_seq_info(unsigned int addr, unsigned int addr_offset, unsigned int seq, enum DISP_FRM_SEQ_STATE state)
{
	int i;

	if (state == FRM_CONFIG) {
		frm_update_sequence[frm_update_cnt].state = state;
		frm_update_sequence[frm_update_cnt].mva = addr;
		frm_update_sequence[frm_update_cnt].max_offset = addr_offset;
		if (seq > 0) frm_update_sequence[frm_update_cnt].seq = seq;
	} else if (state == FRM_TRIGGER) {
		frm_update_sequence[frm_update_cnt].state = FRM_TRIGGER;
		frm_update_cnt++;
		frm_update_cnt %= FRM_UPDATE_SEQ_CACHE_NUM;
	} else if (state == FRM_START) {
		for (i = 0; i < FRM_UPDATE_SEQ_CACHE_NUM; i++) {
			if ((abs(addr - frm_update_sequence[i].mva) > frm_update_sequence[i].max_offset) || (frm_update_sequence[i].state != FRM_TRIGGER)) continue;
			frm_update_sequence[i].state = FRM_START;
		}
	} else if (state == FRM_END) {
		for (i = 0; i < FRM_UPDATE_SEQ_CACHE_NUM; i++) {
			if (frm_update_sequence[i].state != FRM_START) continue;
			frm_update_sequence[i].state = FRM_END;
		}
	}
}

static int _config_wdma_output(struct WDMA_CONFIG_STRUCT *wdma_config, disp_path_handle disp_handle, struct cmdqRecStruct *cmdq_handle)
{
	struct disp_ddp_path_config *pconfig = dpmgr_path_get_last_config(disp_handle);
	pconfig->wdma_config = *wdma_config;
	pconfig->wdma_dirty = 1;
	dpmgr_path_config(disp_handle, pconfig, cmdq_handle);
	return 0;
}

static int _config_rdma_input_data(struct RDMA_CONFIG_STRUCT *rdma_config, disp_path_handle disp_handle, struct cmdqRecStruct *cmdq_handle)
{
	struct disp_ddp_path_config *pconfig = dpmgr_path_get_last_config(disp_handle);
	pconfig->rdma_config = *rdma_config;
	pconfig->rdma_dirty = 1;
	dpmgr_path_config(disp_handle, pconfig, cmdq_handle);
	return 0;
}

static void directlink_path_add_memory(struct WDMA_CONFIG_STRUCT *p_wdma, enum DISP_MODULE_ENUM after_engine)
{
	struct cmdqRecStruct *cmdq_handle = NULL;
	struct cmdqRecStruct *cmdq_wait_handle = NULL;
	struct disp_ddp_path_config *pconfig = NULL;
	int virtual_height = disp_helper_get_option(DISP_OPT_FAKE_LCM_HEIGHT);
	int virtual_width = disp_helper_get_option(DISP_OPT_FAKE_LCM_WIDTH);

	if (cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &cmdq_handle)) return;
	cmdqRecReset(cmdq_handle);

	if (cmdqRecCreate(CMDQ_SCENARIO_DISP_SCREEN_CAPTURE, &cmdq_wait_handle)) {
		cmdqRecDestroy(cmdq_handle);
		return;
	}
	cmdqRecReset(cmdq_wait_handle);

#ifdef MTK_FB_MMDVFS_SUPPORT
	primary_display_request_dvfs_perf(MMDVFS_SCEN_DISP, HRT_LEVEL_NUM - 1, layering_rule_get_mm_freq_table(HRT_OPP_LEVEL_LEVEL0));
#endif

	_cmdq_insert_wait_frame_done_token_mira(cmdq_handle);
	dpmgr_path_add_memout(pgc->dpmgr_handle, after_engine, cmdq_handle);

	pconfig = dpmgr_path_get_last_config(pgc->dpmgr_handle);
	primary_display_config_full_roi(pconfig, pgc->dpmgr_handle, cmdq_handle);
	pconfig->wdma_config = *p_wdma;

	if (disp_helper_get_option(DISP_OPT_DECOUPLE_MODE_USE_RGB565)) {
		pconfig->wdma_config.outputFormat = UFMT_RGB565;
		pconfig->wdma_config.dstPitch = pconfig->wdma_config.srcWidth * 2;
	}
	pconfig->wdma_config.srcHeight = virtual_height;
	pconfig->wdma_config.srcWidth = virtual_width;
	pconfig->wdma_dirty = 1;
	dpmgr_path_config(pgc->dpmgr_handle, pconfig, cmdq_handle);

	_cmdq_set_config_handle_dirty_mira(cmdq_handle);
	_cmdq_flush_config_handle_mira(cmdq_handle, 0);

	cmdqRecWait(cmdq_wait_handle, CMDQ_EVENT_DISP_WDMA0_SOF);
	cmdqRecFlush(cmdq_wait_handle);

	cmdqRecDestroy(cmdq_handle);
	cmdqRecDestroy(cmdq_wait_handle);
}

void disp_enable_emi_force_on(unsigned int enable, void *cmdq_handle) {}

#ifdef MTK_FB_ION_SUPPORT
static struct ion_client *ion_client;
static struct ion_handle *sec_ion_handle;
#endif
static u32 sec_mva;

static int sec_buf_ion_alloc(int buf_size)
{
#ifdef MTK_FB_ION_SUPPORT
	size_t mva_size = 0;
	struct ion_mm_data mm_data;
	ion_phys_addr_t phy_addr = 0;

	memset(&mm_data, 0, sizeof(struct ion_mm_data));
	ion_client = ion_client_create(g_ion_device, "display_dc_secmem");
	if (!ion_client) return -1;

	sec_ion_handle = ion_alloc(ion_client, buf_size, 0, ION_HEAP_MULTIMEDIA_SEC_MASK, 0);
	if (IS_ERR_OR_NULL(sec_ion_handle)) {
		ion_client_destroy(ion_client);
		return -1;
	}

	mm_data.mm_cmd = ION_MM_CONFIG_BUFFER;
	mm_data.config_buffer_param.kernel_handle = sec_ion_handle;
	mm_data.config_buffer_param.module_id = DISP_M4U_PORT_DISP_WDMA0;
	mm_data.config_buffer_param.security = 1;
	mm_data.config_buffer_param.coherent = 0;

	if (ion_kernel_ioctl(ion_client, ION_CMD_MULTIMEDIA_SEC, (unsigned long)&mm_data) < 0) {
		ion_free(ion_client, sec_ion_handle);
		ion_client_destroy(ion_client);
		return -1;
	}

	if (ion_phys(ion_client, sec_ion_handle, &phy_addr, &mva_size)) {
		ion_free(ion_client, sec_ion_handle);
		ion_client_destroy(ion_client);
		return -1;
	}

	if (!phy_addr) {
		ion_free(ion_client, sec_ion_handle);
		ion_client_destroy(ion_client);
		return -1;
	}
	sec_mva = (unsigned int)phy_addr;
#endif
	return 0;
}

static int sec_buf_ion_free(void)
{
#ifdef MTK_FB_ION_SUPPORT
	ion_free(ion_client, sec_ion_handle);
	ion_client_destroy(ion_client);
#endif
	return 0;
}

static int init_sec_buf(void)
{
	int height = disp_helper_get_option(DISP_OPT_FAKE_LCM_HEIGHT);
	int width = disp_helper_get_option(DISP_OPT_FAKE_LCM_WIDTH);
	int Bpp = primary_display_get_bpp() / 8;
	int buffer_size = width * height * Bpp;

	if (sec_mva) return 0;
	return sec_buf_ion_alloc(buffer_size);
}

static int deinit_sec_buf(void)
{
	if (sec_mva) {
		sec_buf_ion_free();
		sec_mva = 0;
	}
	return 0;
}

static int _DL_switch_to_DC_fast(int block)
{
	int ret = 0;
	enum DDP_SCENARIO_ENUM old_scenario, new_scenario;
	struct RDMA_CONFIG_STRUCT rdma_config = decouple_rdma_config;
	struct WDMA_CONFIG_STRUCT wdma_config = decouple_wdma_config;
	struct disp_ddp_path_config *data_config_dl = NULL;
	struct disp_ddp_path_config *data_config_dc = NULL;
	unsigned int mva;
	struct ddp_io_golden_setting_arg gset_arg;

	if (primary_is_sec() == 1) {
		init_sec_buf();
		mva = sec_mva;
		wdma_config.security = DISP_SECURE_BUFFER;
	} else {
		mva = pgc->dc_buf[pgc->dc_buf_id];
		wdma_config.security = DISP_NORMAL_BUFFER;
	}

	if (!mva) return -1;
	wdma_config.dstAddress = mva;

	directlink_path_add_memory(&wdma_config, DISP_MODULE_OVL0);

	_cmdq_reset_config_handle();
	_cmdq_handle_clear_dirty(pgc->cmdq_handle_config);
	_cmdq_insert_wait_frame_done_token_mira(pgc->cmdq_handle_config);

	old_scenario = dpmgr_get_scenario(pgc->dpmgr_handle);
	new_scenario = DDP_SCENARIO_PRIMARY_RDMA0_COLOR0_DISP;

	dpmgr_modify_path_power_on_new_modules(pgc->dpmgr_handle, new_scenario, 0);
	dpmgr_modify_path(pgc->dpmgr_handle, new_scenario, pgc->cmdq_handle_config, primary_display_is_video_mode() ? DDP_VIDEO_MODE : DDP_CMD_MODE, 0);

	rdma_config.address = mva;
	rdma_config.security = wdma_config.security;

	data_config_dl = dpmgr_path_get_last_config(pgc->dpmgr_handle);
	data_config_dl->rdma_config = rdma_config;
	data_config_dl->rdma_config.dst_x = 0;
	data_config_dl->rdma_config.dst_y = 0;
	data_config_dl->rdma_config.dst_h = data_config_dl->dst_h;
	data_config_dl->rdma_config.dst_w = data_config_dl->dst_w;
	data_config_dl->rdma_dirty = 1;

	set_is_dc(1);

	ret = dpmgr_path_config(pgc->dpmgr_handle, data_config_dl, pgc->cmdq_handle_config);

	memset(&gset_arg, 0, sizeof(gset_arg));
	gset_arg.dst_mod_type = dpmgr_path_get_dst_module_type(pgc->dpmgr_handle);
	gset_arg.is_decouple_mode = 1;
	dpmgr_path_ioctl(pgc->dpmgr_handle, pgc->cmdq_handle_config, DDP_OVL_GOLDEN_SETTING, &gset_arg);

	cmdqRecBackupUpdateSlot(pgc->cmdq_handle_config, pgc->rdma_buff_info, 0, rdma_config.address);
	cmdqRecBackupUpdateSlot(pgc->cmdq_handle_config, pgc->rdma_buff_info, 1, rdma_config.pitch);
	cmdqRecBackupUpdateSlot(pgc->cmdq_handle_config, pgc->rdma_buff_info, 2, rdma_config.inputFormat);

	_cmdq_set_config_handle_dirty();
	_cmdq_flush_config_handle(1, NULL, 0);

	dpmgr_modify_path_power_off_old_modules(old_scenario, new_scenario, 0);

#ifdef MTK_FB_MMDVFS_SUPPORT
#ifdef CONFIG_MTK_HIGH_FRAME_RATE
	primary_display_request_dvfs_perf(MMDVFS_SCEN_DISP, HRT_LEVEL_LEVEL1, 0);
#else
	primary_display_request_dvfs_perf(MMDVFS_SCEN_DISP, HRT_LEVEL_LEVEL0, 0);
#endif
#endif

	_cmdq_reset_config_handle();
	_cmdq_handle_clear_dirty(pgc->cmdq_handle_config);
	_cmdq_insert_wait_frame_done_token_mira(pgc->cmdq_handle_config);

	cmdqRecReset(pgc->cmdq_handle_ovl1to2_config);
	pgc->ovl2mem_path_handle = dpmgr_create_path(DDP_SCENARIO_PRIMARY_OVL_MEMOUT, pgc->cmdq_handle_ovl1to2_config);

	if (!pgc->ovl2mem_path_handle) return -1;

	dpmgr_path_set_video_mode(pgc->ovl2mem_path_handle, 0);
	dpmgr_path_init(pgc->ovl2mem_path_handle, CMDQ_ENABLE);

	data_config_dc = dpmgr_path_get_last_config(pgc->ovl2mem_path_handle);
	data_config_dc->dst_w = rdma_config.width;
	data_config_dc->dst_h = rdma_config.height;
	data_config_dc->dst_dirty = 1;

	memcpy(data_config_dc->ovl_config, data_config_dl->ovl_config, sizeof(data_config_dl->ovl_config));
	memcpy(&data_config_dc->rsz_enable, &data_config_dl->rsz_enable, sizeof(data_config_dl->rsz_enable));
	memcpy(&data_config_dc->rsz_src_roi, &data_config_dl->rsz_src_roi, sizeof(data_config_dl->rsz_src_roi));
	memcpy(&data_config_dc->rsz_dst_roi, &data_config_dl->rsz_dst_roi, sizeof(data_config_dl->rsz_dst_roi));

	data_config_dc->ovl_dirty = 1;
	data_config_dc->p_golden_setting_context = data_config_dl->p_golden_setting_context;
	ret = dpmgr_path_config(pgc->ovl2mem_path_handle, data_config_dc, pgc->cmdq_handle_ovl1to2_config);

	memset(&gset_arg, 0, sizeof(gset_arg));
	gset_arg.dst_mod_type = dpmgr_path_get_dst_module_type(pgc->ovl2mem_path_handle);
	gset_arg.is_decouple_mode = 1;
	dpmgr_path_ioctl(pgc->ovl2mem_path_handle, pgc->cmdq_handle_ovl1to2_config, DDP_OVL_GOLDEN_SETTING, &gset_arg);

	ret = dpmgr_path_start(pgc->ovl2mem_path_handle, CMDQ_ENABLE);

	_cmdq_flush_config_handle_mira(pgc->cmdq_handle_ovl1to2_config, block);
	cmdqRecReset(pgc->cmdq_handle_ovl1to2_config);
	cmdqRecWait(pgc->cmdq_handle_ovl1to2_config, CMDQ_EVENT_DISP_WDMA0_EOF);

	if (primary_display_is_video_mode()) {
		if (_need_lfr_check()) dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC, DDP_IRQ_DSI0_FRAME_DONE);
		else dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC, DDP_IRQ_RDMA0_DONE);
	}
	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC);
	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_START);

	deinit_sec_buf();

	return ret;
}

static int DL_switch_to_DC_fast(int sw_only, int block)
{
	if (!sw_only) return _DL_switch_to_DC_fast(block);
	return -1;
}

static int modify_path_power_off_callback(unsigned long userdata)
{
	enum DDP_SCENARIO_ENUM old_scenario = userdata >> 16;
	enum DDP_SCENARIO_ENUM new_scenario = userdata & ((1 << 16) - 1);
	int layer;

	dpmgr_modify_path_power_off_old_modules(old_scenario, new_scenario, 0);

	layer = disp_sync_get_output_interface_timeline_id();
	mtkfb_release_layer_fence(primary_session_id, layer);
	return 0;
}

static int _DC_switch_to_DL_fast(int block)
{
	int ret = 0, layer = 0;
	struct disp_ddp_path_config *data_config_dl = NULL, *data_config_dc = NULL;
	enum DDP_SCENARIO_ENUM old_scenario, new_scenario;
	struct ddp_io_golden_setting_arg gset_arg;

#ifdef CONFIG_MTK_HIGH_FRAME_RATE
	unsigned int vfp;
#endif

	data_config_dc = dpmgr_path_get_last_config(pgc->ovl2mem_path_handle);
	data_config_dl = dpmgr_path_get_last_config(pgc->dpmgr_handle);

	memcpy(data_config_dl->ovl_config, data_config_dc->ovl_config, sizeof(data_config_dl->ovl_config));
	memcpy(&data_config_dl->rsz_enable, &data_config_dc->rsz_enable, sizeof(data_config_dc->rsz_enable));
	memcpy(&data_config_dl->rsz_src_roi, &data_config_dc->rsz_src_roi, sizeof(data_config_dc->rsz_src_roi));
	memcpy(&data_config_dl->rsz_dst_roi, &data_config_dc->rsz_dst_roi, sizeof(data_config_dc->rsz_dst_roi));

	_cmdq_flush_config_handle_mira(pgc->cmdq_handle_ovl1to2_config, 1);
	cmdqRecReset(pgc->cmdq_handle_ovl1to2_config);

	dpmgr_path_deinit(pgc->ovl2mem_path_handle, (unsigned long)(pgc->cmdq_handle_ovl1to2_config));
	dpmgr_destroy_path(pgc->ovl2mem_path_handle, pgc->cmdq_handle_ovl1to2_config);
	cmdqRecClearEventToken(pgc->cmdq_handle_ovl1to2_config, CMDQ_EVENT_DISP_WDMA0_SOF);

	_cmdq_flush_config_handle_mira(pgc->cmdq_handle_ovl1to2_config, 1);
	cmdqRecReset(pgc->cmdq_handle_ovl1to2_config);
	pgc->ovl2mem_path_handle = NULL;

#ifdef MTK_FB_MMDVFS_SUPPORT
	primary_display_request_dvfs_perf(MMDVFS_SCEN_DISP, dvfs_last_ovl_req, ovl_throughput_freq_req);
#endif

	layer = disp_sync_get_output_timeline_id();
	mtkfb_release_layer_fence(primary_session_id, layer);

	_cmdq_reset_config_handle();
	_cmdq_handle_clear_dirty(pgc->cmdq_handle_config);
	_cmdq_insert_wait_frame_done_token_mira(pgc->cmdq_handle_config);

	old_scenario = dpmgr_get_scenario(pgc->dpmgr_handle);
	new_scenario = DDP_SCENARIO_PRIMARY_DISP;

	dpmgr_modify_path_power_on_new_modules(pgc->dpmgr_handle, new_scenario, 0);
	dpmgr_modify_path(pgc->dpmgr_handle, new_scenario, pgc->cmdq_handle_config, primary_display_is_video_mode() ? DDP_VIDEO_MODE : DDP_CMD_MODE, 0);

	data_config_dl->rdma_config = decouple_rdma_config;
	data_config_dl->rdma_config.address = 0;
	data_config_dl->rdma_config.pitch = 0;
	data_config_dl->rdma_config.security = DISP_NORMAL_BUFFER;
	data_config_dl->rdma_dirty = 1;
	data_config_dl->dst_dirty = 1;
	data_config_dl->ovl_dirty = 1;

	set_is_dc(0);

	ret = dpmgr_path_config(pgc->dpmgr_handle, data_config_dl, pgc->cmdq_handle_config);

	memset(&gset_arg, 0, sizeof(gset_arg));
	gset_arg.dst_mod_type = dpmgr_path_get_dst_module_type(pgc->dpmgr_handle);
	gset_arg.is_decouple_mode = 0;
	dpmgr_path_ioctl(pgc->dpmgr_handle, pgc->cmdq_handle_config, DDP_OVL_GOLDEN_SETTING, &gset_arg);

#ifdef CONFIG_MTK_HIGH_FRAME_RATE
	if (primary_display_is_support_DynFPS()) {
		primary_display_dynfps_get_vfp_info(&vfp, NULL);
		dpmgr_path_ioctl(pgc->dpmgr_handle, pgc->cmdq_handle_config, DDP_DSI_PORCH_CHANGE, &vfp);
	}
#endif

	cmdqRecBackupUpdateSlot(pgc->cmdq_handle_config, pgc->rdma_buff_info, 0, 0);
	_cmdq_set_config_handle_dirty();

	_cmdq_flush_config_handle(block, modify_path_power_off_callback, (old_scenario << 16) | new_scenario);

	_cmdq_reset_config_handle();
	_cmdq_handle_clear_dirty(pgc->cmdq_handle_config);
	_cmdq_insert_wait_frame_done_token_mira(pgc->cmdq_handle_config);

	if (primary_display_is_video_mode()) dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC, DDP_IRQ_RDMA0_DONE);
	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC);
	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE);
	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_START);

	return ret;
}

static int _DC_switch_to_DL_sw_only(void)
{
	int layer = 0;
	struct disp_ddp_path_config *data_config_dl = NULL;
	enum DDP_SCENARIO_ENUM old_scenario, new_scenario;

	data_config_dl = dpmgr_path_get_last_config(pgc->dpmgr_handle);
	dpmgr_destroy_path_handle(pgc->ovl2mem_path_handle);
	cmdqRecReset(pgc->cmdq_handle_ovl1to2_config);
	pgc->ovl2mem_path_handle = NULL;

	layer = disp_sync_get_output_timeline_id();
	mtkfb_release_layer_fence(primary_session_id, layer);

	_cmdq_reset_config_handle();
	_cmdq_handle_clear_dirty(pgc->cmdq_handle_config);
	_cmdq_insert_wait_frame_done_token_mira(pgc->cmdq_handle_config);

	old_scenario = dpmgr_get_scenario(pgc->dpmgr_handle);
	new_scenario = DDP_SCENARIO_PRIMARY_DISP;
	dpmgr_modify_path_power_on_new_modules(pgc->dpmgr_handle, new_scenario, 1);
	dpmgr_modify_path(pgc->dpmgr_handle, new_scenario, pgc->cmdq_handle_config, primary_display_is_video_mode() ? DDP_VIDEO_MODE : DDP_CMD_MODE, 1);
	dpmgr_modify_path_power_off_old_modules(old_scenario, new_scenario, 1);

	data_config_dl->rdma_config = decouple_rdma_config;
	data_config_dl->rdma_config.address = 0;
	data_config_dl->rdma_config.pitch = 0;
	data_config_dl->rdma_config.security = DISP_NORMAL_BUFFER;
	set_is_dc(0);

	layer = disp_sync_get_output_interface_timeline_id();
	mtkfb_release_layer_fence(primary_session_id, layer);

	_cmdq_reset_config_handle();
	_cmdq_handle_clear_dirty(pgc->cmdq_handle_config);
	_cmdq_insert_wait_frame_done_token_mira(pgc->cmdq_handle_config);

	if (primary_display_is_video_mode()) {
		if (_need_lfr_check()) dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC, DDP_IRQ_DSI0_FRAME_DONE);
		else dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC, DDP_IRQ_RDMA0_DONE);
	}
	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC);
	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE);
	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_START);

	if (!primary_display_is_video_mode()) _cmdq_build_trigger_loop();

	return 0;
}

static int DC_switch_to_DL_fast(int sw_only, int block)
{
	if (!sw_only) return _DC_switch_to_DL_fast(block);
	else return _DC_switch_to_DL_sw_only();
}

static int DL_switch_to_rdma_mode(struct cmdqRecStruct *handle, int block)
{
	int ret, need_flush = 0;
	enum DDP_SCENARIO_ENUM old_scenario, new_scenario;
	struct ddp_io_golden_setting_arg gset_arg;

	if (!handle) {
		ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &handle);
		if (ret) return -1;
		_cmdq_insert_wait_frame_done_token_mira(handle);
		need_flush = 1;
	}

	old_scenario = dpmgr_get_scenario(pgc->dpmgr_handle);
	new_scenario = DDP_SCENARIO_PRIMARY_RDMA0_COLOR0_DISP;
	dpmgr_modify_path_power_on_new_modules(pgc->dpmgr_handle, new_scenario, 0);
	dpmgr_modify_path(pgc->dpmgr_handle, new_scenario, handle, primary_display_is_video_mode() ? DDP_VIDEO_MODE : DDP_CMD_MODE, 0);
	dpmgr_modify_path_power_off_old_modules(old_scenario, new_scenario, 0);

	memset(&gset_arg, 0, sizeof(gset_arg));
	gset_arg.dst_mod_type = dpmgr_path_get_dst_module_type(pgc->dpmgr_handle);
	gset_arg.is_decouple_mode = 1;
	dpmgr_path_ioctl(pgc->dpmgr_handle, handle, DDP_OVL_GOLDEN_SETTING, &gset_arg);

	if (need_flush) {
		if (block) cmdqRecFlush(handle);
		else cmdqRecFlushAsync(handle);
		cmdqRecDestroy(handle);
	}

	return 0;
}

static int rdma_mode_switch_to_DL(struct cmdqRecStruct *handle, int block)
{
	int ret, need_flush = 0;
	enum DDP_SCENARIO_ENUM old_scenario, new_scenario;
	struct disp_ddp_path_config *pconfig;
	struct ddp_io_golden_setting_arg gset_arg;

	if (!handle) {
		ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &handle);
		if (ret) return -1;
		_cmdq_insert_wait_frame_done_token_mira(handle);
		need_flush = 1;
	}

	old_scenario = dpmgr_get_scenario(pgc->dpmgr_handle);
	new_scenario = DDP_SCENARIO_PRIMARY_DISP;
	dpmgr_modify_path_power_on_new_modules(pgc->dpmgr_handle, new_scenario, 0);
	dpmgr_modify_path(pgc->dpmgr_handle, new_scenario, handle, primary_display_is_video_mode() ? DDP_VIDEO_MODE : DDP_CMD_MODE, 0);
	dpmgr_modify_path_power_off_old_modules(old_scenario, new_scenario, 0);

	pconfig = dpmgr_path_get_last_config(pgc->dpmgr_handle);
	pconfig->rdma_config.address = 0;
	pconfig->rdma_config.pitch = 0;
	pconfig->rdma_config.width = pconfig->dst_w;
	pconfig->rdma_config.height = pconfig->dst_h;
	pconfig->rdma_config.security = DISP_NORMAL_BUFFER;
	pconfig->rdma_dirty = 1;
	pconfig->dst_dirty = 1;
	if (need_flush) pconfig->ovl_dirty = 1;

	set_is_dc(0);
	ret = dpmgr_path_config(pgc->dpmgr_handle, pconfig, handle);

	memset(&gset_arg, 0, sizeof(gset_arg));
	gset_arg.dst_mod_type = dpmgr_path_get_dst_module_type(pgc->dpmgr_handle);
	gset_arg.is_decouple_mode = 0;
	dpmgr_path_ioctl(pgc->dpmgr_handle, handle, DDP_OVL_GOLDEN_SETTING, &gset_arg);

	if (need_flush) {
		if (block) cmdqRecFlush(handle);
		else cmdqRecFlushAsync(handle);
		cmdqRecDestroy(handle);
	}

	return 0;
}

static struct disp_internal_buffer_info *allocat_decouple_buffer(int size)
{
	struct disp_internal_buffer_info *buf_info = NULL;
#ifdef MTK_FB_ION_SUPPORT
	void *buffer_va = NULL;
	struct ion_mm_data mm_data;
	struct ion_client *client = NULL;
	struct ion_handle *handle = NULL;

	client = ion_client_create(g_ion_device, "disp_decouple");

	buf_info = kzalloc(sizeof(struct disp_internal_buffer_info), GFP_KERNEL);
	if (buf_info) {
		handle = ion_alloc(client, size, 0, ION_HEAP_MULTIMEDIA_MASK, 0);
		if (IS_ERR(handle)) {
			ion_free(client, handle);
			ion_client_destroy(client);
			kfree(buf_info);
			return NULL;
		}

		buffer_va = ion_map_kernel(client, handle);
		if (!buffer_va) {
			ion_free(client, handle);
			ion_client_destroy(client);
			kfree(buf_info);
			return NULL;
		}

		memset(&mm_data, 0, sizeof(mm_data));
		mm_data.mm_cmd = ION_MM_GET_IOVA;
		mm_data.get_phys_param.kernel_handle = handle;
		mm_data.get_phys_param.module_id = 0;

		if (ion_kernel_ioctl(client, ION_CMD_MULTIMEDIA, (unsigned long)&mm_data) < 0 || !mm_data.get_phys_param.phy_addr) {
			ion_free(client, handle);
			ion_client_destroy(client);
			kfree(buf_info);
			return NULL;
		}

		buf_info->handle = handle;
		buf_info->mva = (uint32_t)mm_data.get_phys_param.phy_addr;
		buf_info->size = mm_data.get_phys_param.len;
		buf_info->va = buffer_va;
	}
#endif
	return buf_info;
}

static int init_decouple_buffers(void)
{
	int i;
	enum UNIFIED_COLOR_FMT fmt = UFMT_RGB888;
	int height = primary_display_get_height();
	int width = primary_display_get_width();
	int virtual_height = disp_helper_get_option(DISP_OPT_FAKE_LCM_HEIGHT);
	int virtual_width = disp_helper_get_option(DISP_OPT_FAKE_LCM_WIDTH);
	int Bpp = UFMT_GET_Bpp(fmt);
	int buffer_size = width * height * Bpp;

	if (disp_helper_get_option(DISP_OPT_GMO_OPTIMIZE)) {
		decouple_buffer_info[0] = allocat_decouple_buffer(buffer_size);
		if (decouple_buffer_info[0]) pgc->dc_buf[0] = decouple_buffer_info[0]->mva;
		for (i = 1; i < DISP_INTERNAL_BUFFER_COUNT; i++) {
			decouple_buffer_info[i] = decouple_buffer_info[0];
			pgc->dc_buf[i] = pgc->dc_buf[0];
		}
	} else {
		for (i = 0; i < DISP_INTERNAL_BUFFER_COUNT; i++) {
			pgc->dc_buf[i] = i * buffer_size + primary_display_get_frame_buffer_mva_address();
		}
	}

	decouple_rdma_config.height = height;
	decouple_rdma_config.width = width;
	decouple_rdma_config.idx = 0;
	decouple_rdma_config.inputFormat = fmt;
	decouple_rdma_config.pitch = width * Bpp;
	decouple_rdma_config.security = DISP_NORMAL_BUFFER;
	decouple_rdma_config.dst_x = 0;
	decouple_rdma_config.dst_y = 0;
	decouple_rdma_config.dst_w = virtual_width;
	decouple_rdma_config.dst_h = virtual_height;

	decouple_wdma_config.srcHeight = height;
	decouple_wdma_config.srcWidth = width;
	decouple_wdma_config.clipX = 0;
	decouple_wdma_config.clipY = 0;
	decouple_wdma_config.clipHeight = height;
	decouple_wdma_config.clipWidth = width;
	decouple_wdma_config.outputFormat = fmt;
	decouple_wdma_config.useSpecifiedAlpha = 1;
	decouple_wdma_config.alpha = 0xFF;
	decouple_wdma_config.dstPitch = width * Bpp;
	decouple_wdma_config.security = DISP_NORMAL_BUFFER;

	return 0;
}

static int _init_decouple_buffers_thread(void *data)
{
	init_decouple_buffers();
	return 0;
}

static int _build_path_direct_link(void)
{
	pgc->mode = DIRECT_LINK_MODE;
	pgc->dpmgr_handle = dpmgr_create_path(DDP_SCENARIO_PRIMARY_DISP, pgc->cmdq_handle_config);
	if (!pgc->dpmgr_handle) return -1;

	dpmgr_set_lcm_utils(pgc->dpmgr_handle, pgc->plcm->drv);
	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC);
	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE);
	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_START);

	return 0;
}

int _trigger_display_interface(int blocking, void *callback, unsigned int userdata)
{
	if (_should_wait_path_idle()) {
		dpmgr_wait_event_timeout(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE, HZ * 1);
	}

	if (_should_update_lcm()) {
		int x = disp_helper_get_option(DISP_OPT_FAKE_LCM_X);
		int y = disp_helper_get_option(DISP_OPT_FAKE_LCM_Y);
		int width = disp_helper_get_option(DISP_OPT_FAKE_LCM_WIDTH);
		int height = disp_helper_get_option(DISP_OPT_FAKE_LCM_HEIGHT);
		disp_lcm_update(pgc->plcm, x, y, width, height, 0);
	}

	if (_should_start_path()) dpmgr_path_start(pgc->dpmgr_handle, primary_display_cmdq_enabled());

	if (disp_helper_get_option(DISP_OPT_ARR_PHASE_1) && primary_display_is_video_mode() && dynamic_fps_changed) {
		change_dsi_vfp(pgc->cmdq_handle_config, pgc->dynamic_fps);
		dynamic_fps_changed = 0;
	}

	if (_should_trigger_path()) {
#ifndef CONFIG_FPGA_EARLY_PORTING
		if (islcmconnected) dpmgr_wait_event(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC);
#endif
		dpmgr_path_trigger(pgc->dpmgr_handle, NULL, primary_display_cmdq_enabled());
	}

	if (_should_set_cmdq_dirty()) _cmdq_set_config_handle_dirty();
	if (_should_flush_cmdq_config_handle()) _cmdq_flush_config_handle(blocking, callback, userdata);
	if (_should_reset_cmdq_config_handle()) _cmdq_reset_config_handle();
	if (_should_set_cmdq_dirty()) _cmdq_handle_clear_dirty(pgc->cmdq_handle_config);
	if (_should_insert_wait_frame_done_token()) _cmdq_insert_wait_frame_done_token_mira(pgc->cmdq_handle_config);
	if (_need_lfr_check()) dpmgr_path_build_cmdq(pgc->dpmgr_handle, pgc->cmdq_handle_config, CMDQ_DSI_LFR_MODE, 0);

	return 0;
}

int _trigger_ovl_to_memory(disp_path_handle disp_handle, struct cmdqRecStruct *cmdq_handle, CmdqAsyncFlushCB callback, unsigned int data)
{
	int layer, i;
	unsigned int rdma_pitch_sec;

	dpmgr_path_trigger(disp_handle, cmdq_handle, CMDQ_ENABLE);
	cmdqRecWaitNoClear(cmdq_handle, CMDQ_EVENT_DISP_WDMA0_EOF);

	layer = disp_sync_get_output_timeline_id();
	cmdqRecBackupUpdateSlot(cmdq_handle, pgc->cur_config_fence, layer, mem_config.buff_idx);

	layer = disp_sync_get_output_interface_timeline_id();
	cmdqRecBackupUpdateSlot(cmdq_handle, pgc->cur_config_fence, layer, mem_config.interface_idx);

	cmdqRecBackupUpdateSlot(cmdq_handle, pgc->rdma_buff_info, 0, (unsigned int)mem_config.addr);

	rdma_pitch_sec = mem_config.pitch | (mem_config.security << 30);
	cmdqRecBackupUpdateSlot(cmdq_handle, pgc->rdma_buff_info, 1, rdma_pitch_sec);
	cmdqRecBackupUpdateSlot(cmdq_handle, pgc->rdma_buff_info, 2, (unsigned int)mem_config.fmt);

	cmdqRecBackupUpdateSlot(cmdq_handle, pgc->night_light_params, 0, mem_config.m_ccorr_config.mode);
	for (i = 0; i < 16; i++) cmdqRecBackupUpdateSlot(cmdq_handle, pgc->night_light_params, i + 1, mem_config.m_ccorr_config.color_matrix[i]);

	cmdqRecFlushAsyncCallback(cmdq_handle, callback, data);
	cmdqRecReset(cmdq_handle);
	cmdqRecWait(cmdq_handle, CMDQ_EVENT_DISP_WDMA0_EOF);

	return 0;
}

static unsigned int _need_lfr_check(void)
{
#ifdef CONFIG_OF
	return ((pgc->plcm->params->dsi.lfr_enable == 1) && (islcmconnected == 1)) ? 1 : 0;
#else
	return pgc->plcm->params->dsi.lfr_enable == 1 ? 1 : 0;
#endif
}

static int __primary_check_trigger(void)
{
	_primary_path_lock(__func__);
	if (pgc->state != DISP_ALIVE || primary_display_is_video_mode()) goto out;

	if (disp_helper_get_option(DISP_OPT_USE_CMDQ)) {
		static struct cmdqRecStruct *handle;
		struct disp_ddp_path_config *data_config = NULL;

		if (!handle) cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &handle);
		cmdqRecReset(handle);

		primary_display_idlemgr_kick((char *)__func__, 0);
		_cmdq_insert_wait_frame_done_token_mira(handle);

		data_config = dpmgr_path_get_last_config(pgc->dpmgr_handle);
		primary_display_config_full_roi(data_config, pgc->dpmgr_handle, handle);

#ifdef CONFIG_MTK_DISPLAY_120HZ_SUPPORT
		if (od_need_start) {
			od_need_start = 0;
			disp_od_start_read(handle);
		}
		disp_od_update_status(handle);
#endif
		_cmdq_set_config_handle_dirty_mira(handle);
		_cmdq_flush_config_handle_mira(handle, 0);
	} else {
		dpmgr_wait_event(pgc->dpmgr_handle, DISP_PATH_EVENT_TRIGGER);
		primary_display_trigger(1, NULL, 0);
	}

	atomic_set(&delayed_trigger_kick, 1);

out:
	_primary_path_unlock(__func__);
	return 0;
}

static int _disp_primary_path_check_trigger(void *data)
{
	struct sched_param param = {.sched_priority = 94 };
	sched_setscheduler(current, SCHED_RR, &param);

	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_TRIGGER);
	while (1) {
		dpmgr_wait_event(pgc->dpmgr_handle, DISP_PATH_EVENT_TRIGGER);
		__primary_check_trigger();
		if (kthread_should_stop()) break;
	}
	return 0;
}

static int _disp_primary_path_check_trigger_delay_33ms(void *data)
{
	struct sched_param param = {.sched_priority = 94 };
	sched_setscheduler(current, SCHED_RR, &param);
	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_DELAYED_TRIGGER_33ms);

	while (1) {
		unsigned int hwc_fps = 0;
		int stable = 0;

		dpmgr_wait_event(pgc->dpmgr_handle, DISP_PATH_EVENT_DELAYED_TRIGGER_33ms);
		fps_ctx_get_fps(&primary_fps_ctx, &hwc_fps, &stable);

		if (hwc_fps < 20) {
			__primary_check_trigger();
			continue;
		}
		atomic_set(&delayed_trigger_kick, 0);

		if (disp_helper_get_option(DISP_OPT_DELAYED_TRIGGER)) usleep_range(32000, 33000);

		if (!atomic_read(&delayed_trigger_kick)) __primary_check_trigger();
		if (kthread_should_stop()) break;
	}
	return 0;
}

static int _disp_primary_path_check_trigger_od(void *data)
{
	struct sched_param param = {.sched_priority = 94 };
	sched_setscheduler(current, SCHED_RR, &param);

	dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_OD_TRIGGER);
	while (1) {
		dpmgr_wait_event(pgc->dpmgr_handle, DISP_PATH_EVENT_OD_TRIGGER);
		atomic_set(&od_trigger_kick, 1);
		if (kthread_should_stop()) break;
	}
	return 0;
}

unsigned int cmdqDdpClockOn(uint64_t engineFlag) { return 0; }
unsigned int cmdqDdpClockOff(uint64_t engineFlag) { return 0; }
unsigned int cmdqDdpDumpInfo(uint64_t engineFlag, char *pOutBuf, unsigned int bufSize)
{
	primary_display_diagnose();
	if (primary_display_is_decouple_mode()) ddp_dump_analysis(DISP_MODULE_OVL0);
	ddp_dump_analysis(DISP_MODULE_WDMA0);
	return 0;
}
unsigned int cmdqDdpResetEng(uint64_t engineFlag) { return 0; }

int primary_display_change_lcm_resolution(unsigned int width, unsigned int height)
{
	if (!pgc->plcm) return -1;
	if (width > pgc->plcm->params->width || height > pgc->plcm->params->height || width == 0 || height == 0 || width % 4 || height % 4) return -1;
	if (primary_display_is_video_mode()) return -1;

	pgc->plcm->params->width = width;
	pgc->plcm->params->height = height;
	return 0;
}

static int _wdma_fence_release_callback(unsigned long userdata)
{
	int fence_idx, layer;
	layer = disp_sync_get_output_timeline_id();
	cmdqBackupReadSlot(pgc->cur_config_fence, layer, &fence_idx);
	mtkfb_release_fence(primary_session_id, layer, fence_idx);
	return 0;
}

static int _Interface_fence_release_callback(unsigned long userdata)
{
	int layer = disp_sync_get_output_interface_timeline_id();
	if (userdata > 0) mtkfb_release_fence(primary_session_id, layer, userdata);
	return 0;
}

static void DC_config_nightlight(struct cmdqRecStruct *cmdq_handle)
{
	int i, mode, ccorr_matrix[16], all_zero = 1;

	cmdqBackupReadSlot(pgc->night_light_params, 0, &mode);
	for (i = 0; i < 16; i++) cmdqBackupReadSlot(pgc->night_light_params, i + 1, &(ccorr_matrix[i]));
	for (i = 0; i <= 15; i += 5) {
		if (ccorr_matrix[i] != 0) {
			all_zero = 0;
			break;
		}
	}
	if (!all_zero) disp_ccorr_set_color_matrix(cmdq_handle, ccorr_matrix, mode);
}

static int _decouple_update_rdma_config_nolock(void)
{
	int interface_fence = 0, layer = 0, ret = 0;

	if (primary_display_is_decouple_mode()) {
		static struct cmdqRecStruct *cmdq_handle;
		unsigned int rdma_pitch_sec;
		struct RDMA_CONFIG_STRUCT tmpConfig = decouple_rdma_config;

		layer = disp_sync_get_output_interface_timeline_id();
		cmdqBackupReadSlot(pgc->cur_config_fence, layer, &interface_fence);

		if (primary_get_state() != DISP_ALIVE) {
			_Interface_fence_release_callback(interface_fence > 1 ? interface_fence - 1 : 0);
			return -1;
		}

		if (cmdq_handle == NULL) {
			ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &cmdq_handle);
			if (ret) return 0;
		}

		cmdqRecReset(cmdq_handle);
		_cmdq_insert_wait_frame_done_token_mira(cmdq_handle);
		DC_config_nightlight(cmdq_handle);

		cmdqBackupReadSlot(pgc->rdma_buff_info, 0, (uint32_t *)(&(tmpConfig.address)));
		cmdqBackupReadSlot(pgc->rdma_buff_info, 1, &(rdma_pitch_sec));
		tmpConfig.pitch = rdma_pitch_sec & ~(3<<30);
		tmpConfig.security = rdma_pitch_sec >> 30;
		cmdqBackupReadSlot(pgc->rdma_buff_info, 2, &(tmpConfig.inputFormat));

		tmpConfig.height = primary_display_get_height();
		tmpConfig.width = primary_display_get_width();
		tmpConfig.yuv_range = 1;

		_config_rdma_input_data(&tmpConfig, pgc->dpmgr_handle, cmdq_handle);
		_cmdq_set_config_handle_dirty_mira(cmdq_handle);

		if (disp_helper_get_option(DISP_OPT_ARR_PHASE_1) && primary_display_is_video_mode() && dynamic_fps_changed) {
			change_dsi_vfp(cmdq_handle, pgc->dynamic_fps);
			dynamic_fps_changed = 0;
		}

		cmdqRecFlushAsyncCallback(cmdq_handle, (CmdqAsyncFlushCB)_Interface_fence_release_callback, interface_fence > 1 ? interface_fence - 1 : 0);
	}
	return 0;
}

static int decouple_update_rdma_config(void)
{
	int ret;
	_primary_path_lock(__func__);
	ret = _decouple_update_rdma_config_nolock();
	_primary_path_unlock(__func__);
	return ret;
}

static int _ovl_fence_release_callback(unsigned long userdata)
{
	int i = 0, ret = 0, real_hrt_level = 0;
#ifdef CONFIG_MTK_HIGH_FRAME_RATE
	unsigned int config_id = 0;
#endif

	cmdqBackupReadSlot(pgc->subtractor_when_free, 0, &real_hrt_level);
	real_hrt_level >>= 16;

	_primary_path_lock(__func__);
#ifdef CONFIG_MTK_HIGH_FRAME_RATE
	cmdqBackupReadSlot(pgc->config_id_slot, 0, &config_id);
#endif
#ifdef MTK_FB_MMDVFS_SUPPORT
	if ((real_hrt_level >= dvfs_last_ovl_req) && (!primary_display_is_decouple_mode()))
		primary_display_request_dvfs_perf(MMDVFS_SCEN_DISP, dvfs_last_ovl_req, ovl_throughput_freq_req);
#endif
	_primary_path_unlock(__func__);

	for (i = 0; i < PRIMARY_SESSION_INPUT_LAYER_COUNT; i++) {
		int fence_idx = 0, subtractor = 0;

		if (i == (PRIMARY_SESSION_INPUT_LAYER_COUNT - 1) && is_DAL_Enabled()) {
			mtkfb_release_layer_fence(primary_session_id, i);
		} else {
			cmdqBackupReadSlot(pgc->cur_config_fence, i, &fence_idx);
			cmdqBackupReadSlot(pgc->subtractor_when_free, i, &subtractor);
			subtractor &= 0xFFFF;
			mtkfb_release_fence(primary_session_id, i, fence_idx - subtractor);
		}
	}

	if (disp_helper_get_option(DISP_OPT_PERFORMANCE_DEBUG)) {
		unsigned int addr = ddp_ovl_get_cur_addr(!_should_config_ovl_input(), 0);
		if ((primary_display_is_decouple_mode() == 0)) update_frm_seq_info(addr, 0, 2, FRM_START);
	}

	if (primary_display_is_video_mode()) primary_display_wakeup_pf_thread();

	return ret;
}

static int _ovl_wdma_fence_release_callback(unsigned long userdata)
{
	int ret = _ovl_fence_release_callback(userdata);
	ret |= _wdma_fence_release_callback(userdata);
	return ret;
}

static void decouple_mirror_irq_callback(enum DISP_MODULE_ENUM module, unsigned int reg_value)
{
#if (defined(CONFIG_TEE) || defined(CONFIG_TRUSTONIC_TEE_SUPPORT)) && defined(CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT)
	if ((module == DISP_MODULE_OVL0) && (primary_display_is_decouple_mode())) {
		if (reg_value & 0x2) {
			atomic_set(&decouple_update_rdma_event, 1);
			wake_up_interruptible(&decouple_update_rdma_wq);
		}
	}
#else
	if ((module == DISP_MODULE_WDMA0) && (primary_display_is_decouple_mode())) {
		if (reg_value & 0x1) {
			atomic_set(&decouple_update_rdma_event, 1);
			wake_up_interruptible(&decouple_update_rdma_wq);
		}
	}
#endif
}

static int decouple_mirror_update_rdma_config_thread(void *data)
{
	int ret;
	struct sched_param param = {.sched_priority = 94 };

	sched_setscheduler(current, SCHED_RR, &param);
	disp_register_module_irq_callback(DISP_MODULE_WDMA0, decouple_mirror_irq_callback);
	disp_register_module_irq_callback(DISP_MODULE_OVL0, decouple_mirror_irq_callback);

	while (1) {
		ret = wait_event_interruptible(decouple_update_rdma_wq, atomic_read(&decouple_update_rdma_event));
		if (ret == 0) {
			atomic_set(&decouple_update_rdma_event, 0);
			decouple_update_rdma_config();
		}
		if (kthread_should_stop()) break;
	}
	return 0;
}

static int _remove_memout_callback(unsigned long userdata)
{
	int ret = 0;
	CmdqInterruptCB orig_callback = (CmdqInterruptCB)userdata;
	struct DDP_MODULE_DRIVER *ddp_module = ddp_get_module_driver(DISP_MODULE_WDMA0);

	if (ddp_module->deinit != 0) ddp_module->deinit(DISP_MODULE_WDMA0, NULL);
	if (orig_callback) ret = orig_callback(0);

	return ret;
}

static int primary_display_remove_output(void *callback, unsigned int userdata)
{
	int ret = 0;
	static struct cmdqRecStruct *cmdq_handle;
	static struct cmdqRecStruct *cmdq_wait_handle;

	if (cmdq_handle == NULL) ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &cmdq_handle);

	if (ret == 0) {
		if (cmdq_wait_handle == NULL) ret = cmdqRecCreate(CMDQ_SCENARIO_DISP_SCREEN_CAPTURE, &cmdq_wait_handle);

		if (ret == 0) {
			cmdqRecReset(cmdq_wait_handle);
			cmdqRecWait(cmdq_wait_handle, CMDQ_EVENT_DISP_WDMA0_SOF);
			cmdqRecFlush(cmdq_wait_handle);
		}
		cmdqRecReset(cmdq_handle);

		_cmdq_insert_wait_frame_done_token_mira(cmdq_handle);
		cmdqRecBackupUpdateSlot(cmdq_handle, pgc->cur_config_fence, disp_sync_get_output_timeline_id(), mem_config.buff_idx);
		dpmgr_path_remove_memout(pgc->dpmgr_handle, cmdq_handle);
		cmdqRecClearEventToken(cmdq_handle, CMDQ_EVENT_DISP_WDMA0_SOF);

		_cmdq_set_config_handle_dirty_mira(cmdq_handle);
		cmdqRecFlushAsyncCallback(cmdq_handle, _remove_memout_callback, (unsigned long)callback);
		pgc->need_trigger_ovl1to2 = 0;
	} else {
		ret = -1;
	}
	return ret;
}

static void primary_display_frame_update_irq_callback(enum DISP_MODULE_ENUM module, unsigned int param)
{
	if (module == DISP_MODULE_RDMA0) {
		if (param & 0x2) {
			if (pgc->session_id > 0 && primary_display_is_decouple_mode())
				update_frm_seq_info(ddp_ovl_get_cur_addr(1, 0), 0, 1, FRM_START);
		}
		if (param & 0x4) {
			atomic_set(&primary_display_frame_update_event, 1);
			wake_up_interruptible(&primary_display_frame_update_wq);
		}
	}

	if ((module == DISP_MODULE_OVL0) && !primary_display_is_decouple_mode()) {
		if (param & 0x2) {
			atomic_set(&primary_display_frame_update_event, 1);
			wake_up_interruptible(&primary_display_frame_update_wq);
		}
	}
}

static int primary_display_frame_update_kthread(void *data)
{
	struct sched_param param = {.sched_priority = 94 };

	sched_setscheduler(current, SCHED_RR, &param);
	for (;;) {
		wait_event_interruptible(primary_display_frame_update_wq, atomic_read(&primary_display_frame_update_event));
		atomic_set(&primary_display_frame_update_event, 0);

		if (pgc->session_id > 0) update_frm_seq_info(0, 0, 0, FRM_END);
		if (kthread_should_stop()) break;
	}
	return 0;
}

#ifdef MTK_FB_ION_SUPPORT
static int _present_fence_release_worker_thread(void *data)
{
	struct sched_param param = {.sched_priority = 87 };

	sched_setscheduler(current, SCHED_RR, &param);

	while (1) {
		unsigned int pf_idx = 0;

		wait_event_interruptible(primary_display_present_fence_wq, atomic_read(&primary_display_pt_fence_update_event));
		atomic_set(&primary_display_pt_fence_update_event, 0);

		_primary_path_lock(__func__);
		cmdqBackupReadSlot(pgc->cur_config_fence, disp_sync_get_present_timeline_id(), &pf_idx);
		mtkfb_release_present_fence(primary_session_id, pf_idx);
		_primary_path_unlock(__func__);

		if (atomic_read(&od_trigger_kick)) {
			atomic_set(&od_trigger_kick, 0);
			__primary_check_trigger();
		}
	}
	return 0;
}
#endif

int primary_display_set_frame_buffer_address(unsigned long va, unsigned long mva, unsigned long pa)
{
	pgc->framebuffer_va = va;
	pgc->framebuffer_mva = mva;
	pgc->framebuffer_pa = pa;
	return 0;
}

unsigned long primary_display_get_frame_buffer_mva_address(void) { return pgc->framebuffer_mva; }
unsigned long primary_display_get_frame_buffer_va_address(void) { return pgc->framebuffer_va; }

int is_dim_layer(unsigned long mva)
{
	return mva == get_dim_layer_mva_addr();
}

unsigned long get_dim_layer_mva_addr(void)
{
	static unsigned long dim_layer_mva;
	if (dim_layer_mva == 0) {
		int frame_buffer_size = ALIGN_TO(DISP_GetScreenWidth(), MTK_FB_ALIGNMENT) * ALIGN_TO(DISP_GetScreenHeight(), MTK_FB_ALIGNMENT) * 4;
		dim_layer_mva = pgc->framebuffer_mva + (DISP_GetPages() - 1) * frame_buffer_size;
	}
	return dim_layer_mva;
}

static int update_primary_intferface_module(void)
{
	enum DISP_MODULE_ENUM interface_module = _get_dst_module_by_lcm(pgc->plcm);
	ddp_set_dst_module(DDP_SCENARIO_PRIMARY_DISP, interface_module);
	ddp_set_dst_module(DDP_SCENARIO_PRIMARY_RDMA0_COLOR0_DISP, interface_module);
	ddp_set_dst_module(DDP_SCENARIO_PRIMARY_RDMA0_DISP, interface_module);
	ddp_set_dst_module(DDP_SCENARIO_PRIMARY_ALL, interface_module);
	return 0;
}

static void replace_fb_addr_to_mva(void)
{
#ifdef CONFIG_MTK_M4U
	struct ddp_fb_info fb_info;
	fb_info.fb_mva = pgc->framebuffer_mva;
	fb_info.fb_pa = pgc->framebuffer_pa;
	fb_info.fb_size = DISP_GetFBRamSize();
	dpmgr_path_ioctl(pgc->dpmgr_handle, pgc->cmdq_handle_config, DDP_OVL_MVA_REPLACEMENT, &fb_info);
	DISP_REG_SET_FIELD(pgc->cmdq_handle_config, REG_FLD(1, 0), DISPSYS_SMI_LARB0_BASE + 0x380, 0x1);
#endif
}

int primary_display_init(char *lcm_name, unsigned int lcm_fps, int is_lcm_inited)
{
	enum DISP_STATUS ret = DISP_STATUS_OK;
	struct LCM_PARAMS *lcm_param = NULL;
	int use_cmdq = disp_helper_get_option(DISP_OPT_USE_CMDQ);
	struct disp_ddp_path_config *data_config;
	struct ddp_io_golden_setting_arg gset_arg;
	int i = 0;

	dpmgr_init();

	init_cmdq_slots(&(pgc->ovl_config_time), 3, 0);
	init_cmdq_slots(&(pgc->cur_config_fence), DISP_SESSION_TIMELINE_COUNT, 0);
	init_cmdq_slots(&(pgc->subtractor_when_free), DISP_SESSION_TIMELINE_COUNT, 0);
	init_cmdq_slots(&(pgc->rdma_buff_info), 3, 0);
	init_cmdq_slots(&(pgc->ovl_status_info), 4, 0);
	init_cmdq_slots(&(pgc->dither_status_info), 1, 0x10001);
	init_cmdq_slots(&(pgc->dsi_vfp_line), 1, 0);
	init_cmdq_slots(&(pgc->night_light_params), 17, 0);
	init_cmdq_slots(&(pgc->ovl_dummy_info), OVL_NUM, 0);
	init_cmdq_slots(&(pgc->config_id_slot), 1, 0);

	mem_config.m_ccorr_config.is_dirty = 1;
	mem_config.m_ccorr_config.mode = 1;
	cmdqBackupWriteSlot(pgc->night_light_params, 0, 1);

	for (i = 0; i <= 15; i += 5) {
		mem_config.m_ccorr_config.color_matrix[i] = 1024;
		cmdqBackupWriteSlot(pgc->night_light_params, i + 1, 1024);
	}

	mutex_init(&(pgc->capture_lock));
	mutex_init(&(pgc->lock));
	mutex_init(&(pgc->switch_dst_lock));
	mutex_init(&(pgc->dynfps_lock));

	fps_ctx_init(&primary_fps_ctx, disp_helper_get_option(DISP_OPT_FPS_CALC_WND));
	if (disp_helper_get_option(DISP_OPT_FPS_EXT))
		fps_ext_ctx_init(&primary_fps_ext_ctx, disp_helper_get_option(DISP_OPT_FPS_EXT_INTERVAL));

#ifdef MTK_FB_MMDVFS_SUPPORT
	mtk_pm_qos_add_request(&primary_display_qos_request, MTK_PM_QOS_MEMORY_BANDWIDTH, MTK_PM_QOS_MEMORY_BANDWIDTH_DEFAULT_VALUE);
	mtk_pm_qos_add_request(&primary_display_emi_opp_request, MTK_PM_QOS_DDR_OPP, MTK_PM_QOS_DDR_OPP_DEFAULT_VALUE);
	mtk_pm_qos_add_request(&primary_display_mm_freq_request, PM_QOS_DISP_FREQ, PM_QOS_MM_FREQ_DEFAULT_VALUE);
#endif

	_primary_path_lock(__func__);

	pgc->plcm = disp_lcm_probe(lcm_name, LCM_INTERFACE_NOTDEFINED, is_lcm_inited);
	if (unlikely(pgc->plcm == NULL)) {
		ret = DISP_STATUS_ERROR;
		goto done;
	}

	lcm_param = disp_lcm_get_params(pgc->plcm);
	if (unlikely(lcm_param == NULL)) {
		ret = DISP_STATUS_ERROR;
		goto done;
	}

	update_primary_intferface_module();

	if (use_cmdq) {
		ret = cmdqCoreRegisterCB(CMDQ_GROUP_DISP, (CmdqClockOnCB)cmdqDdpClockOn, (CmdqDumpInfoCB)cmdqDdpDumpInfo, (CmdqResetEngCB)cmdqDdpResetEng, (CmdqClockOffCB)cmdqDdpClockOff);
		if (ret) {
			ret = DISP_STATUS_ERROR;
			goto done;
		}

		ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &(pgc->cmdq_handle_config));
		if (ret) {
			ret = DISP_STATUS_ERROR;
			goto done;
		}

		ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_MEMOUT, &(pgc->cmdq_handle_ovl1to2_config));
		if (ret) {
			ret = DISP_STATUS_ERROR;
			goto done;
		}
	} else {
		pgc->cmdq_handle_config = NULL;
		pgc->cmdq_handle_ovl1to2_config = NULL;
	}

	if (likely(primary_display_mode == DIRECT_LINK_MODE)) {
		_build_path_direct_link();
		pgc->session_mode = DISP_SESSION_DIRECT_LINK_MODE;
	}
	if (use_cmdq && is_lcm_inited) {
		_cmdq_reset_config_handle();
		_cmdq_insert_wait_frame_done_token_mira(pgc->cmdq_handle_config);
	}

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	lcm_corner_en = primary_display_get_lcm_corner_en();
	full_content = primary_display_get_corner_full_content();
	if (lcm_corner_en)
		primary_display_get_round_corner_mva(&top_mva, &bottom_mva, &corner_pattern_width, &corner_pattern_height, &corner_pattern_height_bot);
#endif

	primary_display_set_max_layer(PRIMARY_SESSION_INPUT_LAYER_COUNT);

	init_decouple_buffer_thread = kthread_run(_init_decouple_buffers_thread, NULL, "init_decouple_buffer");

	dpmgr_path_set_video_mode(pgc->dpmgr_handle, primary_display_is_video_mode());
	dpmgr_path_init(pgc->dpmgr_handle, use_cmdq);

	if (disp_helper_get_option(DISP_OPT_NO_LCM_FOR_LOW_POWER_MEASUREMENT)) {
		islcmconnected = 0;
		if (!primary_display_is_video_mode()) {
			_init_vsync_fake_monitor(lcm_fps);
			dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC, DDP_IRQ_UNKNOWN);
		}
	}

	if (use_cmdq) {
		if (primary_display_is_video_mode() && pgc->dynamic_fps == 0) pgc->dynamic_fps = 60;
		_cmdq_build_trigger_loop();
		_cmdq_start_trigger_loop();
	}

	data_config = dpmgr_path_get_last_config(pgc->dpmgr_handle);
	memcpy(&(data_config->dispif_config), lcm_param, sizeof(struct LCM_PARAMS));
	data_config->dst_w = disp_helper_get_option(DISP_OPT_FAKE_LCM_WIDTH);
	data_config->dst_h = disp_helper_get_option(DISP_OPT_FAKE_LCM_HEIGHT);
	data_config->p_golden_setting_context = get_golden_setting_pgc();

	if (lcm_param->type == LCM_TYPE_DSI) {
		if (lcm_param->dsi.data_format.format == LCM_DSI_FORMAT_RGB888) data_config->lcm_bpp = 24;
		else if (lcm_param->dsi.data_format.format == LCM_DSI_FORMAT_RGB565) data_config->lcm_bpp = 16;
		else if (lcm_param->dsi.data_format.format == LCM_DSI_FORMAT_RGB666) data_config->lcm_bpp = 18;
	} else if (lcm_param->type == LCM_TYPE_DPI) {
		if (lcm_param->dpi.format == LCM_DPI_FORMAT_RGB888) data_config->lcm_bpp = 24;
		else if (lcm_param->dpi.format == LCM_DPI_FORMAT_RGB565) data_config->lcm_bpp = 16;
		if (lcm_param->dpi.format == LCM_DPI_FORMAT_RGB666) data_config->lcm_bpp = 18;
	}

	data_config->fps = lcm_fps;
	data_config->dst_dirty = 1;
	ret = dpmgr_path_config(pgc->dpmgr_handle, data_config, pgc->cmdq_handle_config);

	memset(&gset_arg, 0, sizeof(gset_arg));
	gset_arg.dst_mod_type = dpmgr_path_get_dst_module_type(pgc->dpmgr_handle);
	gset_arg.is_decouple_mode = 0;
	dpmgr_path_ioctl(pgc->dpmgr_handle, pgc->cmdq_handle_config, DDP_OVL_GOLDEN_SETTING, &gset_arg);
	replace_fb_addr_to_mva();

	if (!is_lcm_inited) {
		if (use_cmdq) {
			_cmdq_flush_config_handle(1, NULL, 0);
			_cmdq_reset_config_handle();
		}
		ret = disp_lcm_init(pgc->plcm, 1);
	}
	if (!ret) primary_display_set_lcm_power_state_nolock(LCM_ON);
	dpmgr_path_start(pgc->dpmgr_handle, use_cmdq);

	if (use_cmdq) {
		_cmdq_flush_config_handle(1, NULL, 0);
		_cmdq_reset_config_handle();
	}

#ifdef CONFIG_MTK_M4U
	config_display_m4u_port();
#endif

	if (use_cmdq) _cmdq_insert_wait_frame_done_token_mira(pgc->cmdq_handle_config);
	if (!is_lcm_inited && primary_display_is_video_mode()) dpmgr_path_trigger(pgc->dpmgr_handle, NULL, 0);

	if (disp_helper_get_option(DISP_OPT_MET_LOG)) set_enterulps(0);

	primary_display_check_recovery_init();

	if (disp_helper_get_option(DISP_OPT_SWITCH_DST_MODE)) {
		primary_display_switch_dst_mode_task = kthread_create(_disp_primary_path_switch_dst_mode_thread, NULL, "display_switch_dst_mode");
		wake_up_process(primary_display_switch_dst_mode_task);
	}

	if (decouple_update_rdma_config_thread == NULL) {
		decouple_update_rdma_config_thread = kthread_create(decouple_mirror_update_rdma_config_thread, NULL, "decouple_update_rdma_cfg");
		wake_up_process(decouple_update_rdma_config_thread);
	}

	if (decouple_trigger_thread == NULL) {
		decouple_trigger_thread = kthread_create(decouple_trigger_worker_thread, NULL, "decouple_trigger");
		wake_up_process(decouple_trigger_thread);
	}

	if (disp_helper_get_stage() == DISP_HELPER_STAGE_NORMAL) {
		primary_path_aal_task = kthread_create(_disp_primary_path_check_trigger, NULL, "display_check_aal");
		wake_up_process(primary_path_aal_task);

		primary_delay_trigger_task = kthread_create(_disp_primary_path_check_trigger_delay_33ms, NULL, "disp_delay_trigger");
		wake_up_process(primary_delay_trigger_task);

		primary_od_trigger_task = kthread_create(_disp_primary_path_check_trigger_od, NULL, "disp_od_trigger");
		wake_up_process(primary_od_trigger_task);
	}

#ifdef MTK_FB_ION_SUPPORT
	if (disp_helper_get_option(DISP_OPT_PRESENT_FENCE)) {
		init_waitqueue_head(&primary_display_present_fence_wq);
		present_fence_release_worker_task = kthread_create(_present_fence_release_worker_thread, NULL, "present_fence_worker");
		wake_up_process(present_fence_release_worker_task);
		pf_thread_init = true;
	}
#endif

	if (disp_helper_get_option(DISP_OPT_PERFORMANCE_DEBUG)) {
		if (primary_display_frame_update_task == NULL) {
			init_waitqueue_head(&primary_display_frame_update_wq);
			disp_register_module_irq_callback(DISP_MODULE_RDMA0, primary_display_frame_update_irq_callback);
			disp_register_module_irq_callback(DISP_MODULE_OVL0, primary_display_frame_update_irq_callback);
			primary_display_frame_update_task = kthread_create(primary_display_frame_update_kthread, NULL, "frame_update_worker");
			wake_up_process(primary_display_frame_update_task);
		}
	}

	if (primary_display_is_video_mode()) {
		if (disp_helper_get_option(DISP_OPT_SWITCH_DST_MODE)) {
			primary_display_cur_dst_mode = 1;
			primary_display_def_dst_mode = 1;
		}
		if (_need_lfr_check()) {
			dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC, DDP_IRQ_DSI0_FRAME_DONE);
		} else {
			dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC, DDP_IRQ_RDMA0_DONE);
			if (disp_helper_get_option(DISP_OPT_ARR_PHASE_1)) {
				dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_START, DDP_IRQ_RDMA0_START);
			}
		}
	}

	pgc->lcm_fps = 6400;
	pgc->lcm_refresh_rate = 64;
	primary_display_lowpower_init();

	primary_set_state(DISP_ALIVE);
#ifdef CONFIG_MTK_HIGH_FRAME_RATE
	primary_display_init_multi_cfg_info();
#endif

done:
	pri_wk_lock = wakeup_source_register(NULL, "pri_disp_wakelock");
	__pm_stay_awake(pri_wk_lock);

	layering_rule_init();
	_primary_path_unlock(__func__);
	return ret;
}

static void _primary_protect_mode_switch(void)
{
	int try_cnt = 50;

	while ((--try_cnt) && atomic_read(&hwc_configing)) {
		udelay(1000);
	}
}

static int request_lcm_refresh_rate_change(int fps);
int primary_display_set_lcm_refresh_rate(int fps)
{
	int ret = 0;

	_primary_protect_mode_switch();

#ifdef CONFIG_MTK_DISPLAY_120HZ_SUPPORT
	if (fps == 60 && primary_display_is_video_mode()) primary_display_switch_dst_mode(0);
#endif

	_primary_path_lock(__func__);
	if (pgc->state == DISP_SLEPT) {
		_primary_path_unlock(__func__);
		return -1;
	}

	if (fps == 120) ret = request_lcm_refresh_rate_change(fps);
	else ret = _display_set_lcm_refresh_rate(fps);
	_primary_path_unlock(__func__);
	return ret;
}

int primary_display_get_lcm_refresh_rate(void) { return pgc->lcm_refresh_rate; }

#ifdef CONFIG_MTK_DISPLAY_120HZ_SUPPORT
static int od_by_pass;

void primary_display_od_bypass(int bypass)
{
	od_by_pass = bypass;
}

static void _display_set_refresh_rate_post_proc(int fps)
{
	if (fps == 60) {
		od_need_start = 0;
		spm_enable_sodi(1);
	} else if (fps == 120) {
		if (!od_by_pass) od_need_start = 1;
		spm_enable_sodi(0);
	}
}
#endif

static int request_lcm_refresh_rate_change(int fps)
{
#ifdef CONFIG_MTK_DISPLAY_120HZ_SUPPORT
	static struct cmdqRecStruct *cmdq_handle, cmdq_pre_handle;
	int ret;

	if (pgc->state == DISP_SLEPT) return -EPERM;
	if (primary_display_get_lcm_max_refresh_rate() <= 60) return -EPERM;
	if (fps == pgc->lcm_refresh_rate) {
		pgc->request_fps = 0;
		return 0;
	}

	if (cmdq_handle == NULL) {
		ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &cmdq_handle);
		if (ret) return -EINVAL;
	}
	if (cmdq_pre_handle == NULL) {
		ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_MEMOUT, &cmdq_pre_handle);
		if (ret) {
			cmdqRecDestroy(cmdq_handle);
			cmdq_handle = NULL;
			return -EINVAL;
		}
	}
	primary_display_idlemgr_kick(__func__, 0);

	pgc->request_fps = fps;
	pgc->lcm_refresh_rate = fps;
#endif
	return 0;
}

int _display_set_lcm_refresh_rate(int fps)
{
#ifdef CONFIG_MTK_DISPLAY_120HZ_SUPPORT
	static struct cmdqRecStruct *cmdq_handle, cmdq_pre_handle;
	disp_path_handle disp_handle;
	struct disp_ddp_path_config *pconfig = NULL;
	int ret = 0;

	if (pgc->state == DISP_SLEPT) return -EPERM;
	if (primary_display_get_lcm_max_refresh_rate() <= 60) return -EPERM;

	if (fps == pgc->lcm_refresh_rate && pgc->request_fps == 0) return 0;
	if (fps == 60 && pgc->request_fps == 120) {
		pgc->lcm_refresh_rate = fps;
		pgc->request_fps = 0;
		return 0;
	}

	if (cmdq_handle == NULL) {
		ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &cmdq_handle);
		if (ret) return -EINVAL;
	}
	if (cmdq_pre_handle == NULL) {
		ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_MEMOUT, &cmdq_pre_handle);
		if (ret) {
			cmdqRecDestroy(cmdq_handle);
			cmdq_handle = NULL;
			return -EINVAL;
		}
	}
	primary_display_idlemgr_kick(__func__, 0);

	pgc->lcm_refresh_rate = fps;
	pgc->request_fps = 0;

	cmdqRecReset(cmdq_handle);
	_cmdq_insert_wait_frame_done_token_mira(cmdq_handle);
	ret = cmdqRecClearEventToken(cmdq_handle, CMDQ_EVENT_DSI_TE);
	ret = cmdqRecWait(cmdq_handle, CMDQ_EVENT_DSI_TE);

	disp_lcm_adjust_fps(cmdq_handle, pgc->plcm, fps);

	disp_handle = pgc->dpmgr_handle;
	pconfig = dpmgr_path_get_last_config(disp_handle);
	pconfig->p_golden_setting_context->fps = fps;
	pconfig->dispif_config.dsi.PLL_CLOCK = pgc->plcm->params->dsi.PLL_CLOCK;
	dpmgr_path_ioctl(primary_get_dpmgr_handle(), cmdq_handle, DDP_RDMA_GOLDEN_SETTING, pconfig);
	dpmgr_path_ioctl(pgc->dpmgr_handle, cmdq_handle, DDP_PHY_CLK_CHANGE, &pgc->plcm->params->dsi.PLL_CLOCK);

	if (!od_by_pass) {
		if (fps == 120) disp_od_set_enabled(cmdq_handle, 1);
		else disp_od_set_enabled(cmdq_handle, 0);
	}

	if (pgc->session_mode == DISP_SESSION_DECOUPLE_MODE) {
		_cmdq_flush_config_handle_mira(cmdq_handle, 1);
	} else {
		_cmdq_flush_config_handle_mira(cmdq_handle, 0);
	}

	_display_set_refresh_rate_post_proc(fps);
#endif
	return 0;
}

int primary_display_get_lcm_max_refresh_rate(void) { return 120; }

int primary_display_deinit(void)
{
	_primary_path_lock(__func__);

	_cmdq_stop_trigger_loop();
	dpmgr_path_deinit(pgc->dpmgr_handle, CMDQ_DISABLE);
	_primary_path_unlock(__func__);

#ifdef MTK_FB_MMDVFS_SUPPORT
	mtk_pm_qos_remove_request(&primary_display_qos_request);
	mtk_pm_qos_remove_request(&primary_display_emi_opp_request);
	mtk_pm_qos_remove_request(&primary_display_mm_freq_request);
#endif

	return 0;
}

int primary_display_wait_for_idle(void) { return 0; }
int primary_display_wait_for_dump(void) { return 0; }

int primary_display_release_fence_fake(void)
{
	unsigned int layer_en = 0, addr = 0, fence_idx = -1;
	unsigned int session_id = MAKE_DISP_SESSION(DISP_SESSION_PRIMARY, 0);
	int i = 0;

	for (i = 0; i < PRIMARY_SESSION_INPUT_LAYER_COUNT; i++) {
		if (i == (PRIMARY_SESSION_INPUT_LAYER_COUNT - 1) && is_DAL_Enabled()) {
			mtkfb_release_layer_fence(session_id, 3);
		} else {
			disp_sync_get_cached_layer_info(session_id, i, &layer_en, (unsigned long *)&addr, &fence_idx);
			if (fence_idx != -1 && fence_idx >= 0) {
				if (layer_en) mtkfb_release_fence(session_id, i, fence_idx - 1);
				else mtkfb_release_fence(session_id, i, fence_idx);
			}
		}
	}

	return 0;
}

int primary_display_wait_for_vsync(void *config)
{
	struct disp_session_vsync_config *c = (struct disp_session_vsync_config *)config;
	int ret = 0, has_vsync = 1;
	unsigned long long ts = 0ULL;

	primary_display_idlemgr_kick(__func__, 1);

#ifdef CONFIG_FPGA_EARLY_PORTING
	if (!primary_display_is_video_mode()) has_vsync = 0;
#endif

	if (!islcmconnected || !has_vsync) {
		msleep(20);
		return 0;
	}

	if (pgc->force_fps_keep_count && pgc->force_fps_skip_count) {
		g_keep++;
		if (g_keep == pgc->force_fps_keep_count) {
			g_keep = 0;
			while (g_skip != pgc->force_fps_skip_count) {
				g_skip++;
				ret = dpmgr_wait_event_timeout(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC, HZ / 10);
				if (ret == -2) return -1;
				else if (ret == 0) primary_display_release_fence_fake();
			}
			g_skip = 0;
		}
	} else {
		g_keep = 0;
		g_skip = 0;
	}

	ret = dpmgr_wait_event_ts(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC, &ts);

	if (ret == -2) goto out;
	if (pgc->vsync_drop) ret = dpmgr_wait_event_ts(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC, &ts);

out:
	c->vsync_ts = ts;
	c->vsync_cnt++;
	c->lcm_fps = pgc->lcm_refresh_rate;

	return ret;
}

unsigned int primary_display_get_ticket(void) { return 0; }

int primary_suspend_release_fence(void)
{
	unsigned int session = (unsigned int)((DISP_SESSION_PRIMARY) << 16 | (0));
	unsigned int i = 0;

	for (i = 0; i < DISP_SESSION_TIMELINE_COUNT; i++) {
		mtkfb_release_layer_fence(session, i);
	}
	return 0;
}

int suspend_to_full_roi(void)
{
	int ret = 0;
	struct cmdqRecStruct *handle = NULL;
	struct disp_ddp_path_config *data_config = NULL;

	if (!disp_partial_is_support() || !primary_display_is_directlink_mode()) return -1;

	ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &handle);
	if (ret) return -1;

	cmdqRecReset(handle);
	_cmdq_insert_wait_frame_done_token_mira(handle);

	data_config = dpmgr_path_get_last_config(pgc->dpmgr_handle);
	primary_display_config_full_roi(data_config, pgc->dpmgr_handle, handle);

	cmdqRecFlush(handle);
	cmdqRecDestroy(handle);
	return ret;
}

int primary_display_suspend(void)
{
	enum DISP_STATUS ret = DISP_STATUS_OK;
	unsigned int cpu;
	for_each_present_cpu(cpu)
		if ((cpu != 0) && (cpu != 4) && cpu_online(cpu))
				cpu_down(cpu);
	screen_off = true;

	primary_display_idlemgr_kick(__func__, 1);

	if (disp_helper_get_option(DISP_OPT_SWITCH_DST_MODE))
		primary_display_switch_dst_mode(primary_display_def_dst_mode);

#ifdef CONFIG_MTK_DISPLAY_120HZ_SUPPORT
	if (primary_display_is_video_mode()) primary_display_switch_dst_mode(0);
#endif

	_primary_path_switch_dst_lock();
	disp_sw_mutex_lock(&(pgc->capture_lock));
	_primary_path_lock(__func__);

	while (primary_get_state() == DISP_BLANK) {
		_primary_path_unlock(__func__);
		primary_display_wait_state(DISP_ALIVE, MAX_SCHEDULE_TIMEOUT);
		_primary_path_lock(__func__);
	}

	if (pgc->state == DISP_SLEPT) goto done;

	primary_display_idlemgr_kick(__func__, 0);

	if (pgc->session_mode == DISP_SESSION_RDMA_MODE) {
		do_primary_display_switch_mode(DISP_SESSION_DIRECT_LINK_MODE, pgc->session_id, 0, NULL, 1);
	}

	if (pgc->session_mode == DISP_SESSION_DECOUPLE_MIRROR_MODE || pgc->session_mode == DISP_SESSION_DECOUPLE_MODE) {
		do_primary_display_switch_mode(DISP_SESSION_DIRECT_LINK_MODE, pgc->session_id, 0, NULL, 1);
	}

	suspend_to_full_roi();

	if (disp_helper_get_option(DISP_OPT_SHARE_SRAM))
		leave_share_sram(CMDQ_SYNC_RESOURCE_WROT1);

	if (disp_helper_get_option(DISP_OPT_USE_CMDQ)) _cmdq_stop_trigger_loop();

	dpmgr_path_stop(pgc->dpmgr_handle, CMDQ_DISABLE);

	if (dpmgr_path_is_busy(pgc->dpmgr_handle)) {
		dpmgr_path_reset(pgc->dpmgr_handle, CMDQ_DISABLE);
		ret = -1;
	}

	if (primary_display_get_power_mode_nolock() == DOZE_SUSPEND) {
		if (primary_display_get_lcm_power_state_nolock() != LCM_ON_LOW_POWER) {
			if (pgc->plcm->drv->aod) disp_lcm_aod(pgc->plcm, 1);
			primary_display_set_lcm_power_state_nolock(LCM_ON_LOW_POWER);
		}
	} else if (primary_display_get_power_mode_nolock() == FB_SUSPEND) {
		disp_lcm_suspend(pgc->plcm);
		primary_suspend_release_fence();
		primary_display_set_lcm_power_state_nolock(LCM_OFF);
	}

	if (disp_helper_get_option(DISP_OPT_SODI_SUPPORT)) ddp_set_spm_mode(DDP_PD_MODE, NULL);

	dpmgr_path_power_off(pgc->dpmgr_handle, CMDQ_DISABLE);
	if (disp_helper_get_option(DISP_OPT_MET_LOG)) set_enterulps(1);

#ifdef MTK_FB_MMDVFS_SUPPORT
	mtk_pm_qos_update_request(&primary_display_qos_request, 0);
#endif

	pgc->lcm_refresh_rate = 64;
#ifdef CONFIG_MTK_HIGH_FRAME_RATE
	pgc->lcm_refresh_rate = primary_display_get_default_disp_fps(0) / 100;
	pgc->lcm_fps = primary_display_get_default_disp_fps(0);
	pgc->active_cfg = 0;
#endif

done:
	primary_set_state(DISP_SLEPT);

	if (primary_display_get_power_mode_nolock() == DOZE_SUSPEND)
		primary_display_esd_check_enable(0);

	__pm_relax(pri_wk_lock);

	_primary_path_unlock(__func__);
	disp_sw_mutex_unlock(&(pgc->capture_lock));
	_primary_path_switch_dst_unlock();

	primary_trigger_cnt = 0;
	ddp_clk_check();
#ifdef MTK_FB_MMDVFS_SUPPORT
	primary_display_request_dvfs_perf(MMDVFS_SCEN_DISP, HRT_LEVEL_DEFAULT, 0);
#endif
	return ret;
}

int primary_display_get_lcm_index(void)
{
	if (pgc->plcm == NULL) return 0;
	return pgc->plcm->index;
}

static int check_switch_lcm_mode_for_debug(void)
{
	static enum LCM_DSI_MODE_CON vdo_mode_type;
	struct LCM_PARAMS *lcm_param_cv = NULL;

	if (lcm_mode_status == 0) return 0;

	lcm_param_cv = disp_lcm_get_params(pgc->plcm);
	if (lcm_param_cv->dsi.mode != CMD_MODE) vdo_mode_type = lcm_param_cv->dsi.mode;

	if (lcm_mode_status == 1) {
		lcm_dsi_mode = CMD_MODE;
	} else if (lcm_mode_status == 2) {
		if (vdo_mode_type) lcm_dsi_mode = vdo_mode_type;
		else lcm_dsi_mode = SYNC_PULSE_VDO_MODE;
	} else {
		lcm_dsi_mode = lcm_param_cv->dsi.mode;
	}

	lcm_param_cv->dsi.mode = lcm_dsi_mode;
	lcm_mode_status = 0;
	set_esd_check_mode(GPIO_EINT_MODE);
	if (disp_helper_get_option(DISP_OPT_SODI_SUPPORT)) primary_display_sodi_rule_init();

	return 1;
}

int primary_display_resume(void)
{
	enum DISP_STATUS ret = DISP_STATUS_OK;
	struct ddp_io_golden_setting_arg gset_arg;
	int i, skip_update = 0;
	unsigned int cpu;

	_primary_path_lock(__func__);
	if (pgc->state == DISP_ALIVE) goto done;

	if (is_ipoh_bootup) {
		primary_display_esd_check_enable(1);
		is_ipoh_bootup = false;
		if (disp_helper_get_option(DISP_OPT_USE_CMDQ)) _cmdq_start_trigger_loop();
		enable_idlemgr(1);
		goto done;
	}

	if (disp_helper_get_option(DISP_OPT_CV_BYSUSPEND)) {
		int dsi_force_config = 0;
		dsi_force_config |= check_switch_lcm_mode_for_debug();
		if (dsi_force_config) DSI_ForceConfig(1);
	}
#ifdef CONFIG_MTK_HIGH_FRAME_RATE
	primary_display_init_multi_cfg_info();
	pgc->first_cfg = 1;
#endif

	dpmgr_path_power_on(pgc->dpmgr_handle, CMDQ_DISABLE);

	if (disp_helper_get_option(DISP_OPT_SODI_SUPPORT)) ddp_set_spm_mode(DDP_CG_MODE, NULL);
	if (disp_helper_get_option(DISP_OPT_MET_LOG)) set_enterulps(0);

	dpmgr_path_reset(pgc->dpmgr_handle, CMDQ_DISABLE);

	{
		struct LCM_PARAMS *lcm_param;
		struct disp_ddp_path_config *data_config;

		ddp_disconnect_path(DDP_SCENARIO_PRIMARY_ALL, NULL);
		ddp_disconnect_path(DDP_SCENARIO_PRIMARY_RDMA0_COLOR0_DISP, NULL);
		dpmgr_path_set_video_mode(pgc->dpmgr_handle, primary_display_is_video_mode());
		dpmgr_path_connect(pgc->dpmgr_handle, CMDQ_DISABLE);
		if (primary_display_is_decouple_mode()) {
			if (pgc->ovl2mem_path_handle) dpmgr_path_connect(pgc->ovl2mem_path_handle, CMDQ_DISABLE);
		}

		lcm_param = disp_lcm_get_params(pgc->plcm);
		data_config = dpmgr_path_get_last_config(pgc->dpmgr_handle);
		memcpy(&(data_config->dispif_config), lcm_param, sizeof(struct LCM_PARAMS));

		data_config->dst_w = disp_helper_get_option(DISP_OPT_FAKE_LCM_WIDTH);
		data_config->dst_h = disp_helper_get_option(DISP_OPT_FAKE_LCM_HEIGHT);
		if (lcm_param->type == LCM_TYPE_DSI) {
			if (lcm_param->dsi.data_format.format == LCM_DSI_FORMAT_RGB888) data_config->lcm_bpp = 24;
			else if (lcm_param->dsi.data_format.format == LCM_DSI_FORMAT_RGB565) data_config->lcm_bpp = 16;
			else if (lcm_param->dsi.data_format.format == LCM_DSI_FORMAT_RGB666) data_config->lcm_bpp = 18;
		} else if (lcm_param->type == LCM_TYPE_DPI) {
			if (lcm_param->dpi.format == LCM_DPI_FORMAT_RGB888) data_config->lcm_bpp = 24;
			else if (lcm_param->dpi.format == LCM_DPI_FORMAT_RGB565) data_config->lcm_bpp = 16;
			if (lcm_param->dpi.format == LCM_DPI_FORMAT_RGB666) data_config->lcm_bpp = 18;
		}

		data_config->fps = pgc->lcm_fps;
		if (disp_partial_is_support()) {
			data_config->ovl_partial_roi.x = 0;
			data_config->ovl_partial_roi.y = 0;
			data_config->ovl_partial_roi.width = primary_display_get_width();
			data_config->ovl_partial_roi.height = primary_display_get_height();
			if (disp_helper_get_option(DISP_OPT_DYNAMIC_RDMA_GOLDEN_SETTING)) {
				set_rdma_width_height(data_config->ovl_partial_roi.width, data_config->ovl_partial_roi.height);
			}
		}
		data_config->dst_dirty = 1;

		for (i = 0; i < ARRAY_SIZE(data_config->ovl_config); i++) {
			if (is_DAL_Enabled() && data_config->ovl_config[i].layer == primary_display_get_option("ASSERT_LAYER"))
				continue;
			data_config->ovl_config[i].layer_en = 0;
		}
		data_config->ovl_dirty = 1;
		ret = dpmgr_path_config(pgc->dpmgr_handle, data_config, NULL);
		data_config->dst_dirty = 0;

		memset(&gset_arg, 0, sizeof(gset_arg));
		gset_arg.dst_mod_type = dpmgr_path_get_dst_module_type(pgc->dpmgr_handle);
		gset_arg.is_decouple_mode = primary_display_is_decouple_mode();
		dpmgr_path_ioctl(pgc->dpmgr_handle, NULL, DDP_OVL_GOLDEN_SETTING, &gset_arg);
	}

	if (primary_display_get_power_mode_nolock() == DOZE) {
		if (primary_display_get_lcm_power_state_nolock() != LCM_ON_LOW_POWER) {
			if (pgc->plcm->drv->aod) disp_lcm_aod(pgc->plcm, 1);
			else {
				disp_lcm_resume(pgc->plcm);
				for_each_present_cpu(cpu)
					if ((cpu != 0) && (cpu != 4) && !cpu_online(cpu))
						cpu_up(cpu);
				screen_off = false;
			}
			primary_display_set_lcm_power_state_nolock(LCM_ON_LOW_POWER);
		}
	} else if (primary_display_get_power_mode_nolock() == FB_RESUME) {
		if (primary_display_get_lcm_power_state_nolock() != LCM_ON) {
			if (primary_display_get_lcm_power_state_nolock() != LCM_ON_LOW_POWER) {
				disp_lcm_resume(pgc->plcm);
				for_each_present_cpu(cpu)
					if ((cpu != 0) && (cpu != 4) && !cpu_online(cpu))
						cpu_up(cpu);
				screen_off = false;
			} else {
				disp_lcm_aod(pgc->plcm, 0);
				skip_update = 1;
			}
			primary_display_set_lcm_power_state_nolock(LCM_ON);
		}
	}

	dpmgr_path_start(pgc->dpmgr_handle, CMDQ_DISABLE);

	if (primary_display_is_decouple_mode())
		dpmgr_path_start(pgc->ovl2mem_path_handle, CMDQ_DISABLE);

	if (disp_helper_get_option(DISP_OPT_USE_CMDQ)) _cmdq_build_trigger_loop();

	if (primary_display_is_video_mode()) {
		if (_should_reset_cmdq_config_handle()) _cmdq_reset_config_handle();
		if (_should_insert_wait_frame_done_token()) _cmdq_insert_wait_frame_done_token_mira(pgc->cmdq_handle_config);

		dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC, DDP_IRQ_RDMA0_DONE);
		dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC);

		dpmgr_path_trigger(pgc->dpmgr_handle, NULL, CMDQ_DISABLE);

		if (disp_helper_get_option(DISP_OPT_ARR_PHASE_1)) {
			dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_START, DDP_IRQ_RDMA0_START);
			dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_START);
		}
	}

	if (disp_helper_get_option(DISP_OPT_USE_CMDQ)) _cmdq_start_trigger_loop();

	if (!primary_display_is_video_mode()) {
		if (_should_reset_cmdq_config_handle()) _cmdq_reset_config_handle();
		if (_should_insert_wait_frame_done_token()) _cmdq_insert_wait_frame_done_token_mira(pgc->cmdq_handle_config);

		dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC, DDP_IRQ_DSI0_EXT_TE);
		dpmgr_enable_event(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC);

		if (primary_display_get_power_mode_nolock() == FB_RESUME && !skip_update) {
			_trigger_display_interface(1, NULL, 0);
			mdelay(16);
		}
	}

#ifdef MTK_FB_MMDVFS_SUPPORT
	mtk_pm_qos_update_request(&primary_display_qos_request, 0); // Temporary generic bandwidth request update.
#endif

	cmdqCoreSetEvent(CMDQ_EVENT_DISP_WDMA0_EOF);

	if (disp_helper_get_option(DISP_OPT_NO_LCM_FOR_LOW_POWER_MEASUREMENT)) {
		islcmconnected = 0;
		if (!primary_display_is_video_mode()) {
			_init_vsync_fake_monitor(pgc->lcm_fps);
			dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC, DDP_IRQ_UNKNOWN);
		}
	}

done:
	primary_set_state(DISP_ALIVE);

	if (disp_helper_get_option(DISP_OPT_SHARE_SRAM))
		enter_share_sram(CMDQ_SYNC_RESOURCE_WROT1);
	if (disp_helper_get_option(DISP_OPT_CV_BYSUSPEND))
		DSI_ForceConfig(0);

	if (primary_display_get_power_mode_nolock() == DOZE)
		primary_display_esd_check_enable(1);

	__pm_stay_awake(pri_wk_lock);

	_primary_path_unlock(__func__);

	ddp_clk_check();
	return ret;
}

int primary_display_ipoh_restore(void)
{
	enable_idlemgr(0);
	primary_display_esd_check_enable(0);
	if (pgc->cmdq_handle_trigger && pgc->cmdq_handle_trigger->running_task) {
		_cmdq_stop_trigger_loop();
		ddp_mutex_set_sof_wait(dpmgr_path_get_mutex(pgc->dpmgr_handle), NULL, 0);
	}
	return 0;
}

int primary_display_start(void)
{
	enum DISP_STATUS ret = DISP_STATUS_OK;
	_primary_path_lock(__func__);
	dpmgr_path_start(pgc->dpmgr_handle, CMDQ_DISABLE);
	if (dpmgr_path_is_busy(pgc->dpmgr_handle)) ret = -1;
	_primary_path_unlock(__func__);
	return ret;
}

int primary_display_stop(void)
{
	enum DISP_STATUS ret = DISP_STATUS_OK;
	_primary_path_lock(__func__);
	if (dpmgr_path_is_busy(pgc->dpmgr_handle)) dpmgr_wait_event_timeout(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE, HZ * 1);
	dpmgr_path_stop(pgc->dpmgr_handle, CMDQ_DISABLE);
	if (dpmgr_path_is_busy(pgc->dpmgr_handle)) ret = -1;
	_primary_path_unlock(__func__);
	return ret;
}

void primary_display_update_present_fence(struct cmdqRecStruct *cmdq_handle, unsigned int fence_idx)
{
	cmdqRecBackupUpdateSlot(cmdq_handle, pgc->cur_config_fence, disp_sync_get_present_timeline_id(), fence_idx);
	gPresentFenceIndex = fence_idx;
}

void primary_display_wakeup_pf_thread(void)
{
	if (!pf_thread_init) return;
	atomic_set(&primary_display_pt_fence_update_event, 1);
	if (disp_helper_get_option(DISP_OPT_PRESENT_FENCE)) wake_up_interruptible(&primary_display_present_fence_wq);
}

static int trigger_decouple_mirror(void)
{
	if (pgc->need_trigger_dcMirror_out) {
		pgc->need_trigger_dcMirror_out = 0;
		if (pgc->session_mode == DISP_SESSION_DECOUPLE_MIRROR_MODE) {
			_trigger_ovl_to_memory(pgc->ovl2mem_path_handle, pgc->cmdq_handle_ovl1to2_config, (CmdqAsyncFlushCB)_ovl_wdma_fence_release_callback, DISP_SESSION_DECOUPLE_MIRROR_MODE);
		}
	}
	return 0;
}

static int primary_display_trigger_nolock(int blocking, void *callback, int need_merge)
{
	int ret = 0;
	last_primary_trigger_time = sched_clock();

	if (is_switched_dst_mode) {
		primary_display_switch_dst_mode(1);
		is_switched_dst_mode = false;
	}

	if (gTriggerDispMode > 0) {
		primary_display_release_fence_fake();
		return ret;
	}

	primary_trigger_cnt++;
	if (pgc->state == DISP_SLEPT) goto done;

	if (g_idle_skip_trigger == 0){
		g_idle_skip++;
		g_idle_skip_trigger++;
	}
	primary_display_idlemgr_kick(__func__, 0);

	if (pgc->session_mode == DISP_SESSION_DIRECT_LINK_MODE || pgc->session_mode == DISP_SESSION_RDMA_MODE) {
		_trigger_display_interface(blocking, _ovl_fence_release_callback, DISP_SESSION_DIRECT_LINK_MODE);
	} else if (pgc->session_mode == DISP_SESSION_DIRECT_LINK_MIRROR_MODE) {
		_trigger_display_interface(0, _ovl_fence_release_callback, DISP_SESSION_DIRECT_LINK_MIRROR_MODE);
		if (pgc->need_trigger_ovl1to2) {
			primary_display_remove_output(_wdma_fence_release_callback, DISP_SESSION_DIRECT_LINK_MIRROR_MODE);
			pgc->need_trigger_ovl1to2 = 0;
		}
	} else if (pgc->session_mode == DISP_SESSION_DECOUPLE_MODE) {
		_trigger_ovl_to_memory(pgc->ovl2mem_path_handle, pgc->cmdq_handle_ovl1to2_config, (CmdqAsyncFlushCB)_ovl_fence_release_callback, DISP_SESSION_DECOUPLE_MODE);
	} else if (pgc->session_mode == DISP_SESSION_DECOUPLE_MIRROR_MODE) {
		if (need_merge == 0 || primary_is_sec()) {
			trigger_decouple_mirror();
		} else {
			atomic_set(&decouple_trigger_event, 1);
			wake_up(&decouple_trigger_wq);
		}
	}

	smart_ovl_try_switch_mode_nolock();
	atomic_set(&delayed_trigger_kick, 1);

done:
	if (pgc->session_id > 0) update_frm_seq_info(0, 0, 0, FRM_TRIGGER);
	return ret;
}

int primary_display_trigger(int blocking, void *callback, int need_merge)
{
	int ret;
	_primary_path_lock(__func__);
	ret = primary_display_trigger_nolock(blocking, callback, need_merge);
	atomic_set(&hwc_configing, 0);
	_primary_path_unlock(__func__);
	return ret;
}

static int decouple_trigger_worker_thread(void *data)
{
	struct sched_param param = {.sched_priority = 94 };

	sched_setscheduler(current, SCHED_RR, &param);

	while (1) {
		wait_event(decouple_trigger_wq, atomic_read(&decouple_trigger_event));
		dpmgr_wait_event(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC);

		_primary_path_lock(__func__);
		trigger_decouple_mirror();
		atomic_set(&decouple_trigger_event, 0);
		wake_up(&decouple_trigger_wq);
		_primary_path_unlock(__func__);

		if (kthread_should_stop()) break;
	}
	return 0;
}

static int config_wdma_output(disp_path_handle disp_handle, struct cmdqRecStruct *cmdq_handle, struct disp_output_config *output)
{
	struct disp_ddp_path_config *pconfig = dpmgr_path_get_last_config(disp_handle);

	pconfig->wdma_config.dstAddress = (unsigned long)output->pa;
	pconfig->wdma_config.srcHeight = primary_display_get_height();
	pconfig->wdma_config.srcWidth = primary_display_get_width();
	pconfig->wdma_config.clipX = output->x;
	pconfig->wdma_config.clipY = output->y;
	pconfig->wdma_config.clipHeight = output->height;
	pconfig->wdma_config.clipWidth = output->width;
	pconfig->wdma_config.outputFormat = disp_fmt_to_unified_fmt(output->fmt);
	pconfig->wdma_config.useSpecifiedAlpha = 1;
	pconfig->wdma_config.alpha = 0xFF;
	pconfig->wdma_config.dstPitch = output->pitch * UFMT_GET_Bpp(pconfig->wdma_config.outputFormat);
	pconfig->wdma_config.security = output->security;
	pconfig->wdma_dirty = 1;

	return dpmgr_path_config(disp_handle, pconfig, cmdq_handle);
}

static int _convert_disp_output_to_memout(struct disp_output_config *src, struct disp_mem_output_config *dst)
{
	dst->fmt = disp_fmt_to_unified_fmt(src->fmt);
	dst->vaddr = (unsigned long)src->va;
	dst->security = src->security;
	dst->w = src->width;
	dst->h = src->height;
	dst->addr = (unsigned long)src->pa;
	dst->buff_idx = src->buff_idx;
	dst->interface_idx = src->interface_idx;
	dst->x = src->x;
	dst->y = src->y;
	dst->pitch = src->pitch * UFMT_GET_Bpp(dst->fmt);
	return 0;
}

static int primary_frame_cfg_output(struct disp_frame_cfg_t *cfg)
{
	int ret = 0;
	disp_path_handle disp_handle;
	struct cmdqRecStruct *cmdq_handle = NULL;

	if (pgc->state == DISP_SLEPT || !primary_display_is_mirror_mode()) goto done;

	if (primary_display_is_decouple_mode()) {
		disp_handle = pgc->ovl2mem_path_handle;
		cmdq_handle = pgc->cmdq_handle_ovl1to2_config;
		pgc->need_trigger_dcMirror_out = 1;
	} else {
		disp_handle = pgc->dpmgr_handle;
		cmdq_handle = pgc->cmdq_handle_config;
		dpmgr_path_add_memout(pgc->dpmgr_handle, DISP_MODULE_OVL0, cmdq_handle);
		pgc->need_trigger_ovl1to2 = 1;
	}

	ret = config_wdma_output(disp_handle, cmdq_handle, &cfg->output_cfg);

	if ((pgc->session_id > 0) && primary_display_is_decouple_mode())
		update_frm_seq_info((unsigned long)(cfg->output_cfg.pa), 0, mtkfb_query_frm_seq_by_addr(pgc->session_id, 0, 0), FRM_CONFIG);

	_convert_disp_output_to_memout(&cfg->output_cfg, &mem_config);

done:
	return ret;
}

enum SVP_STATE { SVP_NOMAL = 0, SVP_IN_POINT, SVP_SEC, SVP_2_NOMAL, SVP_EXIT_POINT };

static enum SVP_STATE svp_state = SVP_NOMAL;
static int svp_sum;

#ifndef OPT_BACKUP_NUM
#define OPT_BACKUP_NUM 4
#endif

static enum DISP_HELPER_OPT opt_backup_name[OPT_BACKUP_NUM] = { DISP_OPT_SMART_OVL, DISP_OPT_IDLEMGR_SWTCH_DECOUPLE, DISP_OPT_BYPASS_OVL, DISP_OPT_OVL_SBCH };
static int opt_backup_value[OPT_BACKUP_NUM];
static unsigned int idlemgr_flag_backup;
static int svp_inited;

static int disp_enter_svp(enum SVP_STATE state)
{
	int i;
	if (state != SVP_IN_POINT) return 0;

	if (svp_inited == 0) {
		for (i = 0; i < OPT_BACKUP_NUM; i++) {
			opt_backup_value[i] = disp_helper_get_option(opt_backup_name[i]);
			disp_helper_set_option(opt_backup_name[i], 0);
		}
		idlemgr_flag_backup = set_idlemgr(0, 0);
		svp_inited = 1;
	}

	if (primary_display_is_decouple_mode() && (!primary_display_is_mirror_mode())) {
		do_primary_display_switch_mode(DISP_SESSION_DIRECT_LINK_MODE, pgc->session_id, 0, NULL, 0);
	}

	return 0;
}

static int disp_leave_svp(enum SVP_STATE state)
{
	int i;
	if (state != SVP_EXIT_POINT || svp_inited != 1) return 0;

	for (i = 0; i < OPT_BACKUP_NUM; i++) disp_helper_set_option(opt_backup_name[i], opt_backup_value[i]);
	set_idlemgr(idlemgr_flag_backup, 0);
	svp_inited = 0;

	return 0;
}

static int setup_disp_sec(struct disp_ddp_path_config *data_config, struct cmdqRecStruct *cmdq_handle, int is_locked)
{
	int i, has_sec_layer = 0;

	for (i = 0; i < ARRAY_SIZE(data_config->ovl_config); i++) {
		if (data_config->ovl_config[i].layer_en && (data_config->ovl_config[i].security == DISP_SECURE_BUFFER)) has_sec_layer = 1;
	}

	if (has_sec_layer != primary_is_sec()) {
		cmdqRecReset(cmdq_handle);
		if (primary_display_is_decouple_mode()) cmdqRecWait(cmdq_handle, CMDQ_EVENT_DISP_WDMA0_EOF);
		else _cmdq_insert_wait_frame_done_token_mira(pgc->cmdq_handle_config);

		if (has_sec_layer) {
			svp_state = SVP_IN_POINT;
			disp_enter_svp(svp_state);
		} else {
			svp_state = SVP_2_NOMAL;
			svp_sum = 0;
		}
	}

	if ((has_sec_layer == primary_is_sec()) && (primary_is_sec() == 0)) {
		if (svp_state == SVP_2_NOMAL) {
			svp_sum++;
			if (svp_sum > 2) {
				svp_sum = 0;
				svp_state = SVP_EXIT_POINT;
				disp_leave_svp(svp_state);
			}
		} else {
			svp_state = SVP_NOMAL;
		}
	}

	if ((has_sec_layer == primary_is_sec()) && (primary_is_sec() == 1)) svp_state = SVP_SEC;
	pgc->is_primary_sec = has_sec_layer;
	return 0;
}

static int can_bypass_ovl(struct disp_ddp_path_config *data_config, int *bypass_layer_id)
{
	int total_layer = 0, i;
	unsigned int w, h;
	struct OVL_CONFIG_STRUCT *oc;

#ifdef CONFIG_MTK_LCM_PHYSICAL_ROTATION_HW
	return 0;
#endif

	if (!disp_helper_get_option(DISP_OPT_BYPASS_OVL)) return 0;

	for (i = 0; i < ARRAY_SIZE(data_config->ovl_config); i++) {
		if (data_config->ovl_config[i].layer_en) {
			total_layer++;
			*bypass_layer_id = i;
		}
	}

	if (total_layer != 1) return 0;

	oc = &data_config->ovl_config[*bypass_layer_id];
	if (oc->src_w != oc->dst_w || oc->src_h != oc->dst_h) return 0;
	if (data_config->ovl_config[*bypass_layer_id].source != OVL_LAYER_SOURCE_MEM) return 0;
	if (data_config->ovl_config[*bypass_layer_id].dst_y == 0 && data_config->ovl_config[*bypass_layer_id].dst_x != 0) return 0;

	h = data_config->ovl_config[*bypass_layer_id].dst_h;
	w = data_config->ovl_config[*bypass_layer_id].dst_w;

	if (w & 0x1) return 0;
	if (w * h <= 512 * 16 / 2) return 0;

	return 1;
}

static int evaluate_bandwidth_save(struct disp_ddp_path_config *cfg, int *ori, int *act)
{
	int i = 0, pixel = 0, partial_pixel = 0, save = 0;

	for (i = 0; i < TOTAL_OVL_LAYER_NUM; i++) {
		int layer_pixel = 0;
		struct disp_rect layer_roi = {0, 0, 0, 0};
		struct disp_rect layer_partial_roi = {0, 0, 0, 0};
		struct OVL_CONFIG_STRUCT *layer = &cfg->ovl_config[i];

		if (!layer->layer_en) continue;

		layer_pixel = layer->dst_w * layer->dst_h;
		pixel += layer_pixel;
		if (cfg->ovl_partial_dirty) {
			layer_roi.x = layer->dst_x;
			layer_roi.y = layer->dst_y;
			layer_roi.width = layer->dst_w;
			layer_roi.height = layer->dst_h;
			if (rect_intersect(&layer_roi, &cfg->ovl_partial_roi, &layer_partial_roi))
				partial_pixel += layer_partial_roi.width * layer_partial_roi.height;
		} else {
			partial_pixel += layer_pixel;
		}
	}

	if (pixel) save = (pixel - partial_pixel) * 100 / pixel;

	*ori = pixel;
	*act = partial_pixel;

	return 0;
}

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
static int primary_display_get_round_corner_mva(unsigned int *tp_mva, unsigned int *bt_mva, unsigned int *pitch, unsigned int *height, unsigned int *height_bot)
{
	unsigned char argb4444_bpp = 2;
	unsigned int corner_size = 0, bt_size = 0;
	unsigned int dal_buf_size = DAL_GetLayerSize();
	unsigned int frame_buf_size = DISP_GetFBRamSize();
	unsigned int vram_buf_size = mtkfb_get_fb_size();
	unsigned long frame_buf_mva = primary_display_get_frame_buffer_mva_address();

	if (vram_buf_size > dal_buf_size + frame_buf_size) {
		*height = primary_display_get_corner_pattern_height();
		*height_bot = primary_display_get_corner_pattern_height_bot();
		*pitch = primary_display_get_width();
		corner_size = (*pitch) * (*height) * argb4444_bpp;
		bt_size = (*pitch) * (*height_bot) * argb4444_bpp;

		*tp_mva = frame_buf_mva + vram_buf_size - corner_size - bt_size;
		*bt_mva = frame_buf_mva + vram_buf_size - corner_size;
	}
	return 0;
}

void add_round_corner_layers(struct disp_ddp_path_config *cfg, unsigned int w, unsigned int h, unsigned int h_bot, unsigned int pitch, unsigned long mva_top, unsigned long mva_bot, enum DISP_MODULE_ENUM module, unsigned int layer, unsigned int phy_layer, unsigned int enable)
{
	struct OVL_CONFIG_STRUCT *input_phy, *input_ext;
	unsigned int offset = round_corner_offset_enable ? 100 : 0;

	input_phy = &(cfg->ovl_config[layer]);
	input_phy->ovl_index = module;
	input_phy->layer = layer;
	input_phy->isDirty = 1;
	input_phy->buff_idx = -1;
	input_phy->layer_en = enable;
	input_phy->fmt = UFMT_RGBA4444;
	input_phy->addr = (unsigned long)mva_top;
	input_phy->const_bld = 0;
	input_phy->src_x = 0;
	input_phy->src_y = 0;
	input_phy->src_w = w;
	input_phy->src_h = h;
	input_phy->src_pitch = pitch*2;
	input_phy->dst_x = 0;
	input_phy->dst_y = offset;
	input_phy->dst_w = w;
	input_phy->dst_h = h;
	input_phy->aen = 1;
	input_phy->sur_aen = 0;
	input_phy->alpha = 255;
	input_phy->keyEn = 0;
	input_phy->key = 0;
	input_phy->src_alpha = 0;
	input_phy->dst_alpha = 0;
	input_phy->source = OVL_LAYER_SOURCE_MEM;
	input_phy->security = 0;
	input_phy->yuv_range = 0;
	input_phy->ext_layer = -1;
	input_phy->ext_sel_layer = -1;
	input_phy->phy_layer = phy_layer;

	input_ext = &(cfg->ovl_config[layer + 1]);
	input_ext->ovl_index = module;
	input_ext->layer = layer + 1;
	input_ext->isDirty = 1;
	input_ext->buff_idx = -1;
	input_ext->layer_en = enable;
	input_ext->fmt = UFMT_RGBA4444;
	input_ext->addr = (unsigned long)mva_bot;
	input_ext->const_bld = 0;
	input_ext->src_x = 0;
	input_ext->src_y = 0;
	input_ext->src_w = w;
	input_ext->src_h = h;
	input_ext->src_pitch = pitch*2;
	input_ext->dst_x = 0;
	input_ext->dst_y = primary_display_get_height() - h - offset;
	input_ext->dst_w = w;
	input_ext->dst_h = h;
	input_ext->aen = 1;
	input_ext->sur_aen = 0;
	input_ext->alpha = 255;
	input_ext->keyEn = 0;
	input_ext->key = 0;
	input_ext->src_alpha = 0;
	input_ext->dst_alpha = 0;
	input_ext->source = OVL_LAYER_SOURCE_MEM;
	input_ext->security = 0;
	input_ext->yuv_range = 0;
	input_ext->ext_layer = 2;
	input_ext->ext_sel_layer = phy_layer;
	input_ext->phy_layer = phy_layer;

	if (full_content) {
		input_ext->src_h = h_bot;
		input_ext->dst_y = primary_display_get_height() - h_bot - offset;
		input_ext->dst_h = h_bot;
	}
}
#endif

static bool disp_rsz_frame_has_rsz_layer(struct disp_frame_cfg_t *cfg)
{
	int i = 0;
	bool rsz = false;

	for (i = 0; i < cfg->input_layer_num; i++) {
		struct disp_input_config *input_cfg = &cfg->input_cfg[i];
		if (!input_cfg->layer_enable) continue;
		if (input_cfg->src_width != input_cfg->tgt_width || input_cfg->src_height != input_cfg->tgt_height) {
			rsz = true;
			break;
		}
	}
	return rsz;
}

static void rsz_in_out_roi(struct disp_frame_cfg_t *cfg, struct disp_ddp_path_config *data_config)
{
	int i = 0;
	struct disp_rect dst_layer_roi = {0, 0, 0, 0}, dst_total_roi = {0, 0, 0, 0};
	struct disp_rect src_layer_roi = {0, 0, 0, 0}, src_total_roi = {0, 0, 0, 0};
	struct disp_input_config *input_cfg = NULL;

	data_config->rsz_enable = FALSE;

	for (i = 0; i < cfg->input_layer_num; i++) {
		input_cfg = &cfg->input_cfg[i];
		if (input_cfg->layer_enable) {
			if (i == 0 && input_cfg->buffer_source == DISP_BUFFER_ALPHA) continue;
			if (input_cfg->src_width < input_cfg->tgt_width || input_cfg->src_height < input_cfg->tgt_height) {
				rect_make(&src_layer_roi, (input_cfg->tgt_offset_x * input_cfg->src_width) / input_cfg->tgt_width, (input_cfg->tgt_offset_y * input_cfg->src_height) / input_cfg->tgt_height, input_cfg->src_width, input_cfg->src_height);
				rect_make(&dst_layer_roi, input_cfg->tgt_offset_x, input_cfg->tgt_offset_y, input_cfg->tgt_width, input_cfg->tgt_height);
				rect_join(&src_layer_roi, &src_total_roi, &src_total_roi);
				rect_join(&dst_layer_roi, &dst_total_roi, &dst_total_roi);
				data_config->rsz_enable = TRUE;
			} else break;
		}
	}
	data_config->rsz_src_roi = src_total_roi;
	data_config->rsz_dst_roi = dst_total_roi;
}

static inline int _is_overlap(struct disp_input_config *src, struct disp_input_config *dst)
{
	if ((src->tgt_offset_y + src->tgt_height <= dst->tgt_offset_y) || (dst->tgt_offset_y + dst->tgt_height <= src->tgt_offset_y)) return 0;
	else if ((src->tgt_offset_x + src->tgt_width <= dst->tgt_offset_x) || (dst->tgt_offset_x + dst->tgt_width <= src->tgt_offset_x)) return 0;
	return 1;
}

static inline int _is_yuv_overlap(struct disp_frame_cfg_t *cfg)
{
	int i = 0, j = 0, is_yuv_overlap = 0;
	unsigned int yuv_num = 0;
	struct disp_input_config *yuv_cfg[cfg->input_layer_num];

	if (cfg->input_layer_num < 2) return 0;

	for (i = 0; i < cfg->input_layer_num; i++) {
		struct disp_input_config *input_cfg = &cfg->input_cfg[i];
		enum DISP_FORMAT src_fmt = input_cfg->src_fmt;
		if (!input_cfg->layer_enable) continue;
		if (UFMT_GET_RGB(disp_fmt_to_unified_fmt(src_fmt))) continue;
		else {
			yuv_cfg[yuv_num] = input_cfg;
			yuv_num += 1;
		}
	}

	if (yuv_num < 2) return 0;

	for (i = 0; i < yuv_num - 1; i++)
		for (j = i + 1; j < yuv_num; j++)
			if (_is_overlap(yuv_cfg[i], yuv_cfg[j])) is_yuv_overlap = 1;

	return is_yuv_overlap;
}

static void _ovl_yuv_throughput_freq_request(struct disp_frame_cfg_t *cfg)
{
	int overlap_yuv_num = 0;
	long long panel_height = (long long)disp_helper_get_option(DISP_OPT_FAKE_LCM_HEIGHT);
	long long panel_width = (long long)disp_helper_get_option(DISP_OPT_FAKE_LCM_WIDTH);
	unsigned long long blank_ratio = 0, pixel_total = 0, blank_field = 0;
	struct LCM_PARAMS *lcm_param;
	int throughput_freq = 0;
	unsigned long long throughput_freq_temp = 0;
	enum HRT_OPP_LEVEL mm_dvfs_level;

	overlap_yuv_num = _is_yuv_overlap(cfg) + 1;

	if (overlap_yuv_num < 2) {
#ifdef MTK_FB_MMDVFS_SUPPORT
		ovl_throughput_freq_req = 0;
#endif
		return;
	}

	lcm_param = disp_lcm_get_params(pgc->plcm);
	blank_field = (long long)(((panel_height + lcm_param->dsi.horizontal_sync_active + lcm_param->dsi.horizontal_backporch + lcm_param->dsi.horizontal_frontporch) *
		(panel_width + lcm_param->dsi.vertical_sync_active + lcm_param->dsi.vertical_backporch + lcm_param->dsi.vertical_frontporch)) - (panel_height * panel_width));

	pixel_total = panel_height * panel_width;
	blank_ratio = blank_field * 10000;

	if (pixel_total == 0) return;

	do_div(blank_ratio, pixel_total);
	throughput_freq_temp = panel_height * panel_width * overlap_yuv_num * 60 * (10000 + blank_ratio) * 10500;

	do_div(throughput_freq_temp, 10000);
	do_div(throughput_freq_temp, 10000);
	do_div(throughput_freq_temp, 1000000);
	throughput_freq = (int)throughput_freq_temp;

	for (mm_dvfs_level = HRT_OPP_LEVEL_NUM - 1; mm_dvfs_level >= HRT_OPP_LEVEL_LEVEL0; mm_dvfs_level--) {
		if (throughput_freq < layering_rule_get_mm_freq_table(mm_dvfs_level)) {
#ifdef MTK_FB_MMDVFS_SUPPORT
			mtk_pm_qos_update_request(&primary_display_mm_freq_request, layering_rule_get_mm_freq_table(mm_dvfs_level));
			ovl_throughput_freq_req = layering_rule_get_mm_freq_table(mm_dvfs_level);
#endif
			break;
		}
#ifdef MTK_FB_MMDVFS_SUPPORT
		ovl_throughput_freq_req = 0;
#endif
	}
}

static void _ovl_sbch_invalid_config(struct cmdqRecStruct *cmdq_handle)
{
	int i;
	CMDQ_VARIABLE sbch_invalid_status, result, shift;

	cmdq_op_init_variable(&sbch_invalid_status);
	cmdq_op_init_variable(&result);
	cmdq_op_init_variable(&shift);

	for (i = 0; i < OVL_NUM; i++) {
		unsigned long ovl_base = ovl_base_addr(i);
		if (ovl_base == 0) continue;

		cmdq_op_read_reg(cmdq_handle, disp_addr_convert(DISP_REG_OVL_SBCH_CON + ovl_base), &sbch_invalid_status, ~0x0);

		for (i = 0; i < OVL_MODULE_MAX_PHY_LAYER; i++) {
			cmdq_op_assign(cmdq_handle, &shift, (1 << (16 + i)));
			cmdq_op_and(cmdq_handle, &result, sbch_invalid_status, shift);
			cmdq_op_if(cmdq_handle, result, CMDQ_NOT_EQUAL, 0);
			cmdq_op_write_reg(cmdq_handle, disp_addr_convert(DISP_REG_OVL_SBCH + ovl_base), 0, (1 << (16 + (i * 4))));
			cmdq_op_end_if(cmdq_handle);
		}

		for (i = 0; i < OVL_MODULE_MAX_EXT_LAYER; i++) {
			cmdq_op_assign(cmdq_handle, &shift, (1 << (20 + i)));
			cmdq_op_and(cmdq_handle, &result, sbch_invalid_status, shift);
			cmdq_op_if(cmdq_handle, result, CMDQ_NOT_EQUAL, 0);
			cmdq_op_write_reg(cmdq_handle, disp_addr_convert(DISP_REG_OVL_SBCH_EXT + ovl_base), 0, (1 << (16 + (i * 4))));
			cmdq_op_end_if(cmdq_handle);
		}
	}
}

static int _config_ovl_input(struct disp_frame_cfg_t *cfg, disp_path_handle disp_handle, struct cmdqRecStruct *cmdq_handle)
{
	int ret = 0, i, layer, aee_layer;
	struct disp_ddp_path_config *data_config = dpmgr_path_get_last_config(disp_handle);
	int max_layer_id_configed = 0, bypass = 0, bypass_layer_id = 0, hrt_level;
	struct disp_rect total_dirty_roi = {0, 0, 0, 0};
	static long long total_ori, total_partial;

	disp_layer_info_statistic(data_config, cfg);

	if (disp_partial_is_support()) {
		if (primary_display_is_directlink_mode()) disp_partial_compute_ovl_roi(cfg, data_config, &total_dirty_roi);
		else assign_full_lcm_roi(&total_dirty_roi);
	}

	if (disp_rsz_frame_has_rsz_layer(cfg)) assign_full_lcm_roi(&total_dirty_roi);
	rsz_in_out_roi(cfg, data_config);

	aee_layer = PRIMARY_SESSION_INPUT_LAYER_COUNT - 1;

	for (i = 0; i < cfg->input_layer_num; i++) {
		struct disp_input_config *input_cfg = &cfg->input_cfg[i];
		struct OVL_CONFIG_STRUCT *ovl_cfg;

		layer = input_cfg->layer_id;
		ovl_cfg = &(data_config->ovl_config[layer]);

		if (cfg->setter != SESSION_USER_AEE) {
			if (is_DAL_Enabled() && layer == aee_layer) continue;
		}

		_convert_disp_input_to_ovl(ovl_cfg, input_cfg);

		if ((ovl_cfg->layer == 0) && (!primary_display_is_decouple_mode()))
			update_frm_seq_info(ovl_cfg->addr, ovl_cfg->src_x * 4 + ovl_cfg->src_y * ovl_cfg->src_pitch,
			    mtkfb_query_frm_seq_by_addr(pgc->session_id, 0, 0), FRM_CONFIG);

		if (max_layer_id_configed < layer) max_layer_id_configed = layer;
		data_config->ovl_layer_dirty |= (1 << i);
	}

	hrt_level = HRT_GET_DVFS_LEVEL(cfg->overlap_layer_num);
	data_config->overlap_layer_num = hrt_level;

	if (_should_wait_path_idle()) dpmgr_wait_event_timeout(disp_handle, DISP_PATH_EVENT_FRAME_DONE, HZ * 1);

	if (cmdq_handle) {
		int sess_mode = pgc->session_mode;
		setup_disp_sec(data_config, cmdq_handle, 1);
		if (sess_mode != pgc->session_mode) {
			if (primary_display_is_decouple_mode()) {
				disp_handle = pgc->ovl2mem_path_handle;
				cmdq_handle = pgc->cmdq_handle_ovl1to2_config;
			} else {
				disp_handle = pgc->dpmgr_handle;
				cmdq_handle = pgc->cmdq_handle_config;
			}
			data_config = dpmgr_path_get_last_config(disp_handle);
			data_config->overlap_layer_num = hrt_level;
		}
	}

	if (cfg->setter != SESSION_USER_INVALID) {
		bypass = can_bypass_ovl(data_config, &bypass_layer_id);
		if (bypass) {
			if (pgc->session_mode == DISP_SESSION_DIRECT_LINK_MODE) {
				assign_full_lcm_roi(&total_dirty_roi);
				primary_display_config_full_roi(data_config, disp_handle, cmdq_handle);
				do_primary_display_switch_mode(DISP_SESSION_RDMA_MODE, pgc->session_id, 0, cmdq_handle, 0);
			}
		} else {
			if (pgc->session_mode == DISP_SESSION_RDMA_MODE) {
				do_primary_display_switch_mode(DISP_SESSION_DIRECT_LINK_MODE, pgc->session_id, 0, cmdq_handle, 0);
				assign_full_lcm_roi(&total_dirty_roi);
			}
		}
	}

	data_config->sbch_enable = (cfg->setter == SESSION_USER_HWC) ? 1 : 0;

	if (pgc->session_mode != DISP_SESSION_RDMA_MODE) {
		data_config->ovl_dirty = 1;
	} else {
		ret = ddp_convert_ovl_input_to_rdma(&data_config->rdma_config, &data_config->ovl_config[bypass_layer_id], data_config->dst_w, data_config->dst_h);
		data_config->rdma_dirty = 1;
		set_is_dc(1);
	}

#ifdef MTK_FB_MMDVFS_SUPPORT
	_ovl_yuv_throughput_freq_request(cfg);
	if (primary_display_is_decouple_mode() && primary_display_is_mirror_mode()) {
		primary_display_request_dvfs_perf(MMDVFS_SCEN_DISP, HRT_LEVEL_LEVEL3, ovl_throughput_freq_req);
		dvfs_last_ovl_req = HRT_LEVEL_LEVEL3;
	} else if (hrt_level > HRT_LEVEL_LEVEL2 && primary_display_is_directlink_mode()) {
		primary_display_request_dvfs_perf(MMDVFS_SCEN_DISP, HRT_LEVEL_LEVEL3, ovl_throughput_freq_req);
		dvfs_last_ovl_req = HRT_LEVEL_LEVEL3;
	} else if (hrt_level > HRT_LEVEL_LEVEL1 && primary_display_is_directlink_mode()) {
		primary_display_request_dvfs_perf(MMDVFS_SCEN_DISP, HRT_LEVEL_LEVEL2, ovl_throughput_freq_req);
		dvfs_last_ovl_req = HRT_LEVEL_LEVEL2;
	} else if (hrt_level > HRT_LEVEL_LEVEL0) {
		dvfs_last_ovl_req = HRT_LEVEL_LEVEL1;
	} else {
		dvfs_last_ovl_req = HRT_LEVEL_LEVEL0;
	}
#endif

	if (disp_helper_get_option(DISP_OPT_DYNAMIC_SWITCH_MMSYSCLK)) set_one_layer(bypass ? 1 : 0);

	if (disp_partial_is_support() && primary_display_is_directlink_mode()) {
		disp_patial_lcm_validate_roi(pgc->plcm, &total_dirty_roi);
		aal_request_partial_support(is_equal_full_lcm(&total_dirty_roi) ? 0 : 1);

		if ((!aal_is_partial_support() || PanelMaster_is_enable() == 1)) assign_full_lcm_roi(&total_dirty_roi);

		if (!rect_equal(&total_dirty_roi, &data_config->ovl_partial_roi)) {
			disp_partial_update_roi_to_lcm(disp_handle, total_dirty_roi, cmdq_handle);
			data_config->ovl_partial_roi = total_dirty_roi;
			if (disp_helper_get_option(DISP_OPT_DYNAMIC_RDMA_GOLDEN_SETTING)) {
				set_rdma_width_height(total_dirty_roi.width, total_dirty_roi.height);
				dpmgr_path_ioctl(disp_handle, cmdq_handle, DDP_RDMA_GOLDEN_SETTING, data_config);
			}
		}
		data_config->ovl_partial_dirty = is_equal_full_lcm(&total_dirty_roi) ? 0 : 1;
	}

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	if (primary_display_is_mirror_mode()) {
		if (lcm_corner_en) add_round_corner_layers(data_config, primary_display_get_width(), corner_pattern_height, corner_pattern_height_bot, corner_pattern_width, top_mva, bottom_mva, DISP_MODULE_OVL0, TOTAL_REAL_OVL_LAYER_NUM-1, 3, 0);
	} else {
		if (lcm_corner_en) {
			if (disp_helper_get_option(DISP_OPT_ROUND_CORNER)) add_round_corner_layers(data_config, primary_display_get_width(), corner_pattern_height, corner_pattern_height_bot, corner_pattern_width, top_mva, bottom_mva, DISP_MODULE_OVL0, TOTAL_REAL_OVL_LAYER_NUM-1, 3, 1);
			else add_round_corner_layers(data_config, primary_display_get_width(), corner_pattern_height, corner_pattern_height_bot, corner_pattern_width, top_mva, bottom_mva, DISP_MODULE_OVL0, TOTAL_REAL_OVL_LAYER_NUM-1, 3, 0);
		}
	}
#endif

	ret = dpmgr_path_config(disp_handle, data_config, cmdq_handle);

	if (!cmdq_handle) goto done;

	for (i = 0; i < cfg->input_layer_num; i++) {
		unsigned int last_fence, cur_fence, sub;
		struct disp_input_config *input_cfg = &cfg->input_cfg[i];
		layer = input_cfg->layer_id;

		cmdqBackupReadSlot(pgc->cur_config_fence, layer, &last_fence);
		cur_fence = input_cfg->next_buff_idx;

		if (cur_fence != -1 && cur_fence > last_fence)
			cmdqRecBackupUpdateSlot(cmdq_handle, pgc->cur_config_fence, layer, cur_fence);

		if (input_cfg->buffer_source == DISP_BUFFER_ALPHA || input_cfg->layer_enable == 0 || cur_fence == -1) sub = 0;
		else sub = 1;

		if (layer == 0) sub |= hrt_level << 16;
		cmdqRecBackupUpdateSlot(cmdq_handle, pgc->subtractor_when_free, layer, sub);
	}

	if (disp_helper_get_option(DISP_OPT_OVL_SBCH) && (data_config->sbch_enable == 1)) _ovl_sbch_invalid_config(cmdq_handle);

	for (i = 0; i < OVL_NUM; i++) {
		if (data_config->read_dum_reg[i]) {
			unsigned long ovl_base = ovl_base_addr(i);
			data_config->read_dum_reg[i] = 0;
			cmdqRecBackupRegisterToSlot(cmdq_handle, pgc->ovl_dummy_info, i, disp_addr_convert(DISP_REG_OVL_DUMMY_REG + ovl_base));
		}
	}
#ifdef CONFIG_MTK_HIGH_FRAME_RATE
	data_config->last_dynfps = data_config->dynfps;
#endif

done:
	return ret;
}

static int primary_frame_cfg_input(struct disp_frame_cfg_t *cfg)
{
	int ret = 0;
	unsigned int wdma_mva = 0;
	disp_path_handle disp_handle;
	struct cmdqRecStruct *cmdq_handle;
	struct disp_ccorr_config m_ccorr_config = cfg->ccorr_config;

	if (gTriggerDispMode > 0) return 0;

	if (disp_helper_get_option(DISP_OPT_IDLE_MGR)) primary_display_idlemgr_kick(__func__, 0);

	if (primary_display_is_decouple_mode()) {
		disp_handle = pgc->ovl2mem_path_handle;
		cmdq_handle = pgc->cmdq_handle_ovl1to2_config;
	} else {
		disp_handle = pgc->dpmgr_handle;
		cmdq_handle = pgc->cmdq_handle_config;
	}

	if (pgc->state == DISP_SLEPT) {
		if (is_DAL_Enabled() && cfg->setter == SESSION_USER_AEE && (cfg->input_cfg[0].layer_id == (PRIMARY_SESSION_INPUT_LAYER_COUNT - 1))) {
			struct disp_ddp_path_config *data_config = dpmgr_path_get_last_config(disp_handle);
			int layer = cfg->input_cfg[0].layer_id;
			ret = _convert_disp_input_to_ovl(&(data_config->ovl_config[layer]), &cfg->input_cfg[0]);
		}
		goto done;
	}

	if (disp_helper_get_stage() == DISP_HELPER_STAGE_NORMAL) fps_ctx_update(&primary_fps_ctx);
	if (disp_helper_get_option(DISP_OPT_FPS_EXT)) fps_ext_ctx_update(&primary_fps_ext_ctx);

	_config_ovl_input(cfg, disp_handle, cmdq_handle);
	if (cfg->present_fence_idx != (unsigned int)-1)
		primary_display_update_present_fence(cmdq_handle, cfg->present_fence_idx);

	if (m_ccorr_config.is_dirty) {
		int i = 0, all_zero = 1;
		for (i = 0; i <= 15; i += 5) {
			if (m_ccorr_config.color_matrix[i] != 0) {
				all_zero = 0;
				break;
			}
		}
		if (!all_zero && !primary_display_is_decouple_mode()) {
			disp_ccorr_set_color_matrix(cmdq_handle, m_ccorr_config.color_matrix, m_ccorr_config.mode);
			mem_config.m_ccorr_config = m_ccorr_config;
			cmdqRecBackupUpdateSlot(cmdq_handle, pgc->night_light_params, 0, mem_config.m_ccorr_config.mode);
			for (i = 0; i < 16; i++) cmdqRecBackupUpdateSlot(cmdq_handle, pgc->night_light_params, i + 1, mem_config.m_ccorr_config.color_matrix[i]);
		} else {
			mem_config.m_ccorr_config = m_ccorr_config;
		}
	}

	if (primary_display_is_decouple_mode() && !primary_display_is_mirror_mode()) {
		pgc->dc_buf_id++;
		pgc->dc_buf_id %= DISP_INTERNAL_BUFFER_COUNT;
		wdma_mva = pgc->dc_buf[pgc->dc_buf_id];
		if (!wdma_mva) {
			ret = -1;
			goto done;
		}

		decouple_wdma_config.dstAddress = wdma_mva;
		_config_wdma_output(&decouple_wdma_config, pgc->ovl2mem_path_handle, pgc->cmdq_handle_ovl1to2_config);
		mem_config.addr = wdma_mva;
		mem_config.buff_idx = -1;
		mem_config.interface_idx = -1;
		mem_config.security = DISP_NORMAL_BUFFER;
		mem_config.pitch = decouple_wdma_config.dstPitch;
		mem_config.fmt = decouple_wdma_config.outputFormat;
	}
done:
	return ret;
}

int primary_display_config_input_multiple(struct disp_session_input_config *session_input)
{
	int ret = 0;
	struct disp_frame_cfg_t *frame_cfg;

	frame_cfg = kzalloc(sizeof(struct disp_frame_cfg_t), GFP_KERNEL);
	if (!frame_cfg) return -ENOMEM;

	frame_cfg->session_id = primary_session_id;
	frame_cfg->setter = session_input->setter;
	frame_cfg->input_layer_num = session_input->config_layer_num;
	frame_cfg->overlap_layer_num = HRT_LEVEL_LEVEL2;
	frame_cfg->ccorr_config = session_input->ccorr_config;

	memcpy(frame_cfg->input_cfg, session_input->config, sizeof(frame_cfg->input_cfg));

	if (disp_validate_ioctl_params(frame_cfg)) {
		ret = -EINVAL;
		goto out;
	}

	_primary_path_lock(__func__);
	atomic_set(&hwc_configing, 1);
	ret = primary_frame_cfg_input(frame_cfg);
	_primary_path_unlock(__func__);

out:
	kfree(frame_cfg);
	return ret;
}

int primary_display_frame_cfg(struct disp_frame_cfg_t *cfg)
{
	int ret = 0;
#ifdef CONFIG_MTK_HIGH_FRAME_RATE
	unsigned int default_fps = 60;
#endif

	_primary_path_lock(__func__);

#ifdef CONFIG_MTK_DISPLAY_120HZ_SUPPORT
	if (pgc->request_fps && HRT_FPS(cfg->overlap_layer_num) == 120) _display_set_lcm_refresh_rate(pgc->request_fps);
#endif
#ifdef CONFIG_MTK_HIGH_FRAME_RATE
	if (pgc->first_cfg) {
		default_fps = disp_lcm_dynfps_get_def_fps(pgc->plcm);
		default_fps = default_fps ? default_fps : 6400;
		if (default_fps != 6000) disp_invoke_fps_chg_callbacks(default_fps / 100);
		pgc->first_cfg = 0;
	}
	if (g_force_cfg) cfg->active_config = g_force_cfg_id;
#endif

	primary_frame_cfg_input(cfg);

	if (cfg->output_en) {
		primary_frame_cfg_output(cfg);
	}

	primary_display_trigger_nolock(0, NULL, 0);

#ifdef CONFIG_MTK_HIGH_FRAME_RATE
	if (primary_display_is_support_DynFPS()) primary_display_dynfps_chg_fps(cfg->active_config);
#endif

	_primary_path_unlock(__func__);
	return ret;
}

int primary_display_user_cmd(unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct cmdqRecStruct *handle = NULL;
	int cmdqsize = 0;

	if (cmd == DISP_IOCTL_AAL_GET_HIST || cmd == DISP_IOCTL_AAL_GET_SIZE || cmd == DISP_IOCTL_CCORR_GET_IRQ) {
		_primary_path_lock(__func__);

		if (disp_helper_get_option(DISP_OPT_USE_CMDQ)) {
			ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &handle);
			cmdqRecReset(handle);
			_cmdq_insert_wait_frame_done_token_mira(handle);
			cmdqsize = cmdqRecGetInstructionCount(handle);
		}

		if (pgc->state == DISP_SLEPT && handle) {
			cmdqRecDestroy(handle);
			handle = NULL;
		}
		_primary_path_unlock(__func__);

		if (disp_helper_get_option(DISP_OPT_IDLEMGR_ENTER_ULPS) && !primary_display_is_video_mode()) primary_display_idlemgr_kick(__func__, 1);

		ret = dpmgr_path_user_cmd(pgc->dpmgr_handle, cmd, arg, handle);

		if (handle) {
			if (cmdqRecGetInstructionCount(handle) > cmdqsize) {
				_primary_path_lock(__func__);
				if (pgc->state == DISP_ALIVE) _cmdq_flush_config_handle_mira(handle, 0);
				_primary_path_unlock(__func__);
			}
			cmdqsize = cmdqRecGetInstructionCount(handle);
			cmdqRecDestroy(handle);
		}
	} else {
		_primary_path_switch_dst_lock();
		_primary_path_lock(__func__);

		if (disp_helper_get_option(DISP_OPT_USE_CMDQ)) {
			ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &handle);
			cmdqRecReset(handle);
			_cmdq_insert_wait_frame_done_token_mira(handle);
			cmdqsize = cmdqRecGetInstructionCount(handle);
		}

		if (pgc->state == DISP_SLEPT && handle) {
			cmdqRecDestroy(handle);
			handle = NULL;
			goto user_cmd_unlock;
		}
		if (disp_helper_get_option(DISP_OPT_IDLEMGR_ENTER_ULPS) && !primary_display_is_video_mode()) primary_display_idlemgr_kick(__func__, 0);

		ret = dpmgr_path_user_cmd(pgc->dpmgr_handle, cmd, arg, handle);

		if (handle) {
			if (cmdqRecGetInstructionCount(handle) > cmdqsize && pgc->state == DISP_ALIVE) {
				_cmdq_flush_config_handle_mira(handle, 0);
			}
			cmdqsize = cmdqRecGetInstructionCount(handle);
			cmdqRecDestroy(handle);
		}

user_cmd_unlock:
		_primary_path_unlock(__func__);
		_primary_path_switch_dst_unlock();
	}

	return ret;
}

int do_primary_display_switch_mode(int sess_mode, unsigned int session, int need_lock, struct cmdqRecStruct *handle, int block)
{
	int ret = 0, sw_only = 0;

	if (need_lock) _primary_path_lock(__func__);

	if (pgc->session_mode == sess_mode) goto done;

	if (pgc->state == DISP_SLEPT) sw_only = 1;

	if (pgc->session_mode == DISP_SESSION_DIRECT_LINK_MODE && sess_mode == DISP_SESSION_DECOUPLE_MODE) {
		ret = DL_switch_to_DC_fast(sw_only, block);
	} else if (pgc->session_mode == DISP_SESSION_DECOUPLE_MODE && sess_mode == DISP_SESSION_DIRECT_LINK_MODE) {
		ret = DC_switch_to_DL_fast(sw_only, block);
	} else if (pgc->session_mode == DISP_SESSION_DIRECT_LINK_MODE && sess_mode == DISP_SESSION_DECOUPLE_MIRROR_MODE) {
		ret = DL_switch_to_DC_fast(sw_only, block);
	} else if (pgc->session_mode == DISP_SESSION_DECOUPLE_MIRROR_MODE && sess_mode == DISP_SESSION_DIRECT_LINK_MODE) {
		ret = DC_switch_to_DL_fast(sw_only, block);
	} else if (pgc->session_mode == DISP_SESSION_DIRECT_LINK_MODE && sess_mode == DISP_SESSION_RDMA_MODE) {
		ret = DL_switch_to_rdma_mode(handle, block);
	} else if (pgc->session_mode == DISP_SESSION_RDMA_MODE && sess_mode == DISP_SESSION_DIRECT_LINK_MODE) {
		ret = rdma_mode_switch_to_DL(handle, block);
	} else if (pgc->session_mode == DISP_SESSION_RDMA_MODE && sess_mode == DISP_SESSION_DECOUPLE_MIRROR_MODE) {
		ret = rdma_mode_switch_to_DL(NULL, 0);
		if (!ret) ret = DL_switch_to_DC_fast(0, 0);
	}

done:
	if (!ret) pgc->session_mode = sess_mode;
	if (need_lock) _primary_path_unlock(__func__);
	pgc->session_id = session;
	return ret;
}

int primary_display_switch_mode(int sess_mode, unsigned int session, int force)
{
	int ret = 0;

	_primary_path_lock(__func__);
	primary_display_idlemgr_kick(__func__, 0);

	if (!force && primary_display_is_mirror_mode() == _is_mirror_mode(sess_mode)) goto done;
	if (pgc->session_mode == sess_mode) goto done;

	while (primary_get_state() == DISP_BLANK) {
		_primary_path_unlock(__func__);
		primary_display_wait_not_state(DISP_BLANK, MAX_SCHEDULE_TIMEOUT);
		_primary_path_lock(__func__);
	}

	ret = do_primary_display_switch_mode(sess_mode, session, 0, NULL, 0);

done:
	_primary_path_unlock(__func__);
	return ret;
}

int primary_display_switch_mode_blocked(int sess_mode, unsigned int session, int force)
{
	int ret = 0;

	_primary_path_lock(__func__);
	primary_display_idlemgr_kick(__func__, 0);

	if (!force && primary_display_is_mirror_mode() == _is_mirror_mode(sess_mode)) goto done;
	if (pgc->session_mode == sess_mode) goto done;

	while (primary_get_state() == DISP_BLANK) {
		_primary_path_unlock(__func__);
		primary_display_wait_not_state(DISP_BLANK, MAX_SCHEDULE_TIMEOUT);
		_primary_path_lock(__func__);
	}

	ret = do_primary_display_switch_mode(sess_mode, session, 0, NULL, 1);

done:
	_primary_path_unlock(__func__);
	return ret;
}

static int smart_ovl_try_switch_mode_nolock(void)
{
	unsigned int hwc_fps, lcm_fps;
	unsigned long long ovl_sz, rdma_sz;
	disp_path_handle disp_handle = NULL;
	struct disp_ddp_path_config *data_config = NULL;
	int i, stable;
	unsigned long long DL_bw, DC_bw, bw_th;

	if (!disp_helper_get_option(DISP_OPT_SMART_OVL) || !primary_display_is_video_mode()) return 0;
	if (pgc->session_mode != DISP_SESSION_DIRECT_LINK_MODE && pgc->session_mode != DISP_SESSION_DECOUPLE_MODE) return 0;
	if (pgc->state != DISP_ALIVE) return 0;

	lcm_fps = pgc->lcm_fps / 100;
	fps_ctx_get_fps(&primary_fps_ctx, &hwc_fps, &stable);

	if (pgc->session_mode == DISP_SESSION_DIRECT_LINK_MODE && !stable) return 0;
	if (hwc_fps > lcm_fps) hwc_fps = lcm_fps;

	disp_handle = (pgc->session_mode == DISP_SESSION_DECOUPLE_MODE) ? pgc->ovl2mem_path_handle : pgc->dpmgr_handle;
	data_config = dpmgr_path_get_last_config(disp_handle);

	rdma_sz = (unsigned long long)data_config->dst_h * data_config->dst_w * 3;
	ovl_sz = 0;

	for (i = 0; i < ARRAY_SIZE(data_config->ovl_config); i++) {
		struct OVL_CONFIG_STRUCT *ovl_cfg = &(data_config->ovl_config[i]);
		if (ovl_cfg->layer_en) {
			unsigned int Bpp = UFMT_GET_Bpp(ovl_cfg->fmt);
			ovl_sz += (unsigned long long)ovl_cfg->dst_w * ovl_cfg->dst_h * Bpp;
		}
	}

	DL_bw = ovl_sz * lcm_fps;
	DC_bw = (ovl_sz + rdma_sz) * hwc_fps + rdma_sz * lcm_fps;

	if (pgc->session_mode == DISP_SESSION_DIRECT_LINK_MODE) {
		bw_th = DL_bw * 4;
		do_div(bw_th, 5);
		if (DC_bw < bw_th) do_primary_display_switch_mode(DISP_SESSION_DECOUPLE_MODE, pgc->session_id, 0, NULL, 0);
	} else {
		bw_th = DC_bw * 4;
		do_div(bw_th, 5);
		if (DL_bw < bw_th) do_primary_display_switch_mode(DISP_SESSION_DIRECT_LINK_MODE, pgc->session_id, 0, NULL, 0);
	}

	return 0;
}

int primary_display_is_alive(void)
{
	unsigned int temp = 0;
	_primary_path_lock(__func__);
	if (pgc->state == DISP_ALIVE) temp = 1;
	_primary_path_unlock(__func__);
	return temp;
}

int primary_display_is_sleepd(void)
{
	unsigned int temp = 0;
	_primary_path_lock(__func__);
	if (pgc->state == DISP_SLEPT) temp = 1;
	_primary_path_unlock(__func__);
	return temp;
}

int primary_display_get_width(void) { return (pgc->plcm && pgc->plcm->params) ? pgc->plcm->params->width : 0; }
int primary_display_get_height(void) { return (pgc->plcm && pgc->plcm->params) ? pgc->plcm->params->height : 0; }
int primary_display_get_virtual_width(void) { return (pgc->plcm && pgc->plcm->params) ? pgc->plcm->params->virtual_width : 0; }
int primary_display_get_virtual_height(void) { return (pgc->plcm && pgc->plcm->params) ? pgc->plcm->params->virtual_height : 0; }
int primary_display_get_original_width(void) { return (pgc->plcm && pgc->plcm->params) ? pgc->plcm->lcm_original_width : 0; }
int primary_display_get_original_height(void) { return (pgc->plcm && pgc->plcm->params) ? pgc->plcm->lcm_original_height : 0; }
int primary_display_get_bpp(void) { return 32; }

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
int primary_display_get_corner_full_content(void) { return (pgc->plcm && pgc->plcm->params) ? pgc->plcm->params->full_content : 0; }
int primary_display_get_lcm_corner_en(void) { return (pgc->plcm && pgc->plcm->params) ? pgc->plcm->params->round_corner_en : 0; }
int primary_display_get_corner_pattern_width(void) { return (pgc->plcm && pgc->plcm->params) ? pgc->plcm->params->corner_pattern_width : 0; }
int primary_display_get_corner_pattern_height(void) { return (pgc->plcm && pgc->plcm->params) ? pgc->plcm->params->corner_pattern_height : 0; }
int primary_display_get_corner_pattern_height_bot(void) { return (pgc->plcm && pgc->plcm->params) ? pgc->plcm->params->corner_pattern_height_bot : 0; }
#endif

void primary_display_set_max_layer(int maxlayer) { pgc->max_layer = maxlayer; }
int primary_display_get_max_layer(void) { return pgc->max_layer; }

int primary_display_get_info(struct disp_session_info *info)
{
	struct disp_session_info *dispif_info = (struct disp_session_info *)info;
	struct LCM_PARAMS *lcm_param = disp_lcm_get_params(pgc->plcm);

	if (!lcm_param) return -1;

	memset(dispif_info, 0, sizeof(struct disp_session_info));
	dispif_info->maxLayerNum = is_DAL_Enabled() ? (pgc->max_layer - 1) : pgc->max_layer;
	dispif_info->const_layer_num = 0;

	switch (lcm_param->type) {
	case LCM_TYPE_DBI:
		dispif_info->displayType = DISP_IF_TYPE_DBI;
		dispif_info->displayMode = DISP_IF_MODE_COMMAND;
		dispif_info->isHwVsyncAvailable = 1;
		break;
	case LCM_TYPE_DPI:
		dispif_info->displayType = DISP_IF_TYPE_DPI;
		dispif_info->displayMode = DISP_IF_MODE_VIDEO;
		dispif_info->isHwVsyncAvailable = 1;
		break;
	case LCM_TYPE_DSI:
		dispif_info->displayType = DISP_IF_TYPE_DSI0;
		dispif_info->displayMode = (lcm_param->dsi.mode == CMD_MODE) ? DISP_IF_MODE_COMMAND : DISP_IF_MODE_VIDEO;
		dispif_info->isHwVsyncAvailable = 1;
		break;
	default: break;
	}

	dispif_info->displayFormat = DISP_IF_FORMAT_RGB888;
	dispif_info->displayWidth = primary_display_get_width();
	dispif_info->displayHeight = primary_display_get_height();
	dispif_info->physicalWidth = DISP_GetActiveWidth();
	dispif_info->physicalHeight = DISP_GetActiveHeight();
	dispif_info->physicalWidthUm = DISP_GetActiveWidthUm();
	dispif_info->physicalHeightUm = DISP_GetActiveHeightUm();
	dispif_info->density = DISP_GetDensity();
	dispif_info->vsyncFPS = pgc->lcm_fps;
	dispif_info->isConnected = 1;

	if (disp_helper_get_option(DISP_OPT_FPS_EXT))
		fps_ext_ctx_get_fps(&primary_fps_ext_ctx, &dispif_info->updateFPS, &dispif_info->is_updateFPS_stable);
	else
		fps_ctx_get_fps(&primary_fps_ctx, &dispif_info->updateFPS, &dispif_info->is_updateFPS_stable);

	dispif_info->updateFPS *= 100;
	return 0;
}

int primary_display_get_pages(void) { return 3; }
int primary_display_is_video_mode(void) { return disp_lcm_is_video_mode(pgc->plcm); }
int primary_display_diagnose(void)
{
	dpmgr_check_status(pgc->dpmgr_handle);
	if (primary_display_is_decouple_mode()) dpmgr_check_status_by_scenario(DDP_SCENARIO_PRIMARY_OVL_MEMOUT);
	return 0;
}

enum CMDQ_SWITCH primary_display_cmdq_enabled(void) { return disp_helper_get_option(DISP_OPT_USE_CMDQ); }
int primary_display_manual_lock(void) { _primary_path_lock(__func__); return 0; }
int primary_display_manual_unlock(void) { _primary_path_unlock(__func__); return 0; }
void primary_display_reset(void) { dpmgr_path_reset(pgc->dpmgr_handle, CMDQ_DISABLE); }
unsigned int primary_display_get_fps_nolock(void) { return pgc->lcm_fps; }
unsigned int primary_display_get_fps(void)
{
	unsigned int fps;
	_primary_path_lock(__func__);
	fps = pgc->lcm_fps;
	_primary_path_unlock(__func__);
	return fps;
}

int primary_display_force_set_fps(unsigned int keep, unsigned int skip)
{
	_primary_path_lock(__func__);
	pgc->force_fps_keep_count = keep;
	pgc->force_fps_skip_count = skip;
	g_keep = 0;
	g_skip = 0;
	_primary_path_unlock(__func__);
	return 0;
}

unsigned int primary_display_force_get_vsync_fps(void)
{
	if (primary_display_is_idle()) return (pgc->plcm->params->min_refresh_rate != 0) ? pgc->plcm->params->min_refresh_rate : 64;
	else if (disp_helper_get_option(DISP_OPT_ARR_PHASE_1)) return pgc->dynamic_fps;
	return 64;
}

int primary_display_force_set_vsync_fps(unsigned int fps, unsigned int scenario)
{
	if (!primary_display_is_video_mode()) return -1;

	if ((scenario == 0) && disp_helper_get_option(DISP_OPT_ARR_PHASE_1)) {
		_primary_path_lock(__func__);
		pgc->dynamic_fps = fps;
		arr_fps_backup = fps;

		if ((fps < primary_display_get_min_refresh_rate()) && (fps > primary_display_get_max_refresh_rate())) arr_fps_enable = 1;
		else if (fps == 60) arr_fps_enable = 0;

		if (primary_display_is_idle()) dynamic_fps_changed = 0;
		else dynamic_fps_changed = 1;
		_primary_path_unlock(__func__);
	}

	if (scenario == 1) {
		pgc->dynamic_fps = fps;
		dynamic_fps_changed = 1;
	}

	if (scenario == 2) {
		if (disp_helper_get_option(DISP_OPT_ARR_PHASE_1) && (arr_fps_enable == 1)) pgc->dynamic_fps = arr_fps_backup;
		else pgc->dynamic_fps = fps;
		dynamic_fps_changed = 1;
	}

	return 0;
}

int primary_display_vsync_switch(int method)
{
	if (method == 0) dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC, DDP_IRQ_RDMA0_DONE);
	else if (method == 1) dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC, DDP_IRQ_DSI0_EXT_TE);
	else if (method == 2) dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC, DDP_IRQ_DSI0_FRAME_DONE);
	return 0;
}

int _set_backlight_by_cmdq(unsigned int level)
{
	struct cmdqRecStruct *cmdq_handle_backlight = NULL;

	if (cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &cmdq_handle_backlight)) return -1;

	if (primary_display_is_video_mode()) {
		cmdqRecReset(cmdq_handle_backlight);
		_cmdq_insert_wait_frame_done_token_mira(cmdq_handle_backlight);
		disp_lcm_set_backlight(pgc->plcm, cmdq_handle_backlight, level);
		_cmdq_flush_config_handle_mira(cmdq_handle_backlight, 0);
	} else {
		cmdqRecReset(cmdq_handle_backlight);
		cmdqRecWait(cmdq_handle_backlight, CMDQ_SYNC_TOKEN_CABC_EOF);
		_cmdq_handle_clear_dirty(cmdq_handle_backlight);
		_cmdq_insert_wait_frame_done_token_mira(cmdq_handle_backlight);
		disp_lcm_set_backlight(pgc->plcm, cmdq_handle_backlight, level);
		cmdqRecSetEventToken(cmdq_handle_backlight, CMDQ_SYNC_TOKEN_CONFIG_DIRTY);
		cmdqRecSetEventToken(cmdq_handle_backlight, CMDQ_SYNC_TOKEN_CABC_EOF);
		_cmdq_flush_config_handle_mira(cmdq_handle_backlight, 1);
	}
	cmdqRecDestroy(cmdq_handle_backlight);
	return 0;
}

int _set_backlight_by_cpu(unsigned int level)
{
	if (disp_helper_get_stage() != DISP_HELPER_STAGE_NORMAL) return 0;

	if (primary_display_is_video_mode()) {
		disp_lcm_set_backlight(pgc->plcm, NULL, level);
	} else {
		if (primary_display_cmdq_enabled()) _cmdq_stop_trigger_loop();

		if (dpmgr_path_is_busy(pgc->dpmgr_handle))
			dpmgr_wait_event_timeout(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE, HZ * 1);

		dpmgr_path_stop(pgc->dpmgr_handle, CMDQ_DISABLE);
		if (dpmgr_path_is_busy(pgc->dpmgr_handle))
			dpmgr_wait_event_timeout(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE, HZ * 1);

		dpmgr_path_reset(pgc->dpmgr_handle, CMDQ_DISABLE);
		disp_lcm_set_backlight(pgc->plcm, NULL, level);

		dpmgr_path_start(pgc->dpmgr_handle, CMDQ_DISABLE);
		if (primary_display_cmdq_enabled()) _cmdq_start_trigger_loop();
	}
	return 0;
}

int primary_display_setbacklight(unsigned int level)
{
	static unsigned int last_level;
	bool aal_is_support = disp_aal_is_support();

	if (disp_helper_get_stage() != DISP_HELPER_STAGE_NORMAL) return 0;
	if (last_level == level) return 0;

	if (aal_is_support == false) {
		_primary_path_switch_dst_lock();
		_primary_path_lock(__func__);
	}

	if (pgc->state != DISP_SLEPT) {
		primary_display_idlemgr_kick(__func__, 0);
		if (primary_display_cmdq_enabled()) {
			_set_backlight_by_cmdq(level);
			atomic_set(&delayed_trigger_kick, 1);
		} else {
			_set_backlight_by_cpu(level);
		}
		last_level = level;
	}

	if (aal_is_support == false) {
		_primary_path_unlock(__func__);
		_primary_path_switch_dst_unlock();
	}

	return 0;
}

int _set_lcm_cmd_by_cmdq(unsigned int *lcm_cmd, unsigned int *lcm_count, unsigned int *lcm_value)
{
	struct cmdqRecStruct *cmdq_handle_lcm_cmd = NULL;

	if (cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &cmdq_handle_lcm_cmd)) return -1;

	if (primary_display_is_video_mode()) {
		cmdqRecReset(cmdq_handle_lcm_cmd);
		disp_lcm_set_lcm_cmd(pgc->plcm, cmdq_handle_lcm_cmd, lcm_cmd, lcm_count, lcm_value);
		_cmdq_flush_config_handle_mira(cmdq_handle_lcm_cmd, 1);
	} else {
		cmdqRecReset(cmdq_handle_lcm_cmd);
		_cmdq_handle_clear_dirty(cmdq_handle_lcm_cmd);
		_cmdq_insert_wait_frame_done_token_mira(cmdq_handle_lcm_cmd);
		disp_lcm_set_lcm_cmd(pgc->plcm, cmdq_handle_lcm_cmd, lcm_cmd, lcm_count, lcm_value);
		cmdqRecSetEventToken(cmdq_handle_lcm_cmd, CMDQ_SYNC_TOKEN_CONFIG_DIRTY);
		_cmdq_flush_config_handle_mira(cmdq_handle_lcm_cmd, 1);
	}
	cmdqRecDestroy(cmdq_handle_lcm_cmd);
	return 0;
}

int primary_display_setlcm_cmd(unsigned int *lcm_cmd, unsigned int *lcm_count, unsigned int *lcm_value)
{
	if (disp_helper_get_stage() != DISP_HELPER_STAGE_NORMAL) return 0;

	_primary_path_switch_dst_lock();
	_primary_path_lock(__func__);

	if (pgc->state != DISP_SLEPT && primary_display_cmdq_enabled()) {
		_set_lcm_cmd_by_cmdq(lcm_cmd, lcm_count, lcm_value);
	}

	_primary_path_unlock(__func__);
	_primary_path_switch_dst_lock();

	return 0;
}

int primary_display_mipi_clk_change(unsigned int clk_value)
{
	struct cmdqRecStruct *cmdq_handle = NULL;

	if (pgc->state == DISP_SLEPT) return 0;

	_primary_path_lock(__func__);
	if (!primary_display_is_video_mode()) {
		_primary_path_unlock(__func__);
		return 0;
	}

	cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &cmdq_handle);
	cmdqRecReset(cmdq_handle);
	_cmdq_insert_wait_frame_done_token_mira(cmdq_handle);

	pgc->plcm->params->dsi.PLL_CLOCK = clk_value;

	dpmgr_path_build_cmdq(pgc->dpmgr_handle, cmdq_handle, CMDQ_STOP_VDO_MODE, 0);
	dpmgr_path_ioctl(primary_get_dpmgr_handle(), cmdq_handle, DDP_PHY_CLK_CHANGE, &clk_value);
	dpmgr_path_build_cmdq(pgc->dpmgr_handle, cmdq_handle, CMDQ_START_VDO_MODE, 0);

	cmdqRecClearEventToken(cmdq_handle, CMDQ_EVENT_MUTEX0_STREAM_EOF);
	cmdqRecClearEventToken(cmdq_handle, CMDQ_EVENT_DISP_RDMA0_EOF);

	dpmgr_path_trigger(pgc->dpmgr_handle, cmdq_handle, CMDQ_ENABLE);
	ddp_mutex_set_sof_wait(dpmgr_path_get_mutex(pgc->dpmgr_handle), pgc->cmdq_handle_config_esd, 0);
	_cmdq_flush_config_handle_mira(cmdq_handle, 1);

	cmdqRecDestroy(cmdq_handle);
	_primary_path_unlock(__func__);

	return 0;
}

UINT32 DISP_GetScreenWidth(void) { return primary_display_get_width(); }
UINT32 DISP_GetScreenHeight(void) { return primary_display_get_height(); }
UINT32 DISP_GetActiveHeight(void) { return (pgc->plcm && pgc->plcm->params) ? pgc->plcm->params->physical_height : 0; }
UINT32 DISP_GetActiveWidth(void) { return (pgc->plcm && pgc->plcm->params) ? pgc->plcm->params->physical_width : 0; }
uint32_t DISP_GetActiveHeightUm(void) { return (pgc->plcm && pgc->plcm->params) ? pgc->plcm->params->physical_height_um : 0; }
uint32_t DISP_GetActiveWidthUm(void) { return (pgc->plcm && pgc->plcm->params) ? pgc->plcm->params->physical_width_um : 0; }
uint32_t DISP_GetDensity(void) { return (pgc->plcm && pgc->plcm->params) ? pgc->plcm->params->density : 0; }
struct LCM_PARAMS *DISP_GetLcmPara(void) { return (pgc->plcm && pgc->plcm->params) ? pgc->plcm->params : NULL; }
struct LCM_DRIVER *DISP_GetLcmDrv(void) { return (pgc->plcm && pgc->plcm->drv) ? pgc->plcm->drv : NULL; }

static int _screen_cap_by_cmdq(unsigned int mva, enum UNIFIED_COLOR_FMT ufmt, enum DISP_MODULE_ENUM after_eng)
{
	struct cmdqRecStruct *cmdq_handle = NULL, *cmdq_wait_handle = NULL;
	struct disp_ddp_path_config *pconfig = NULL;
	unsigned int w_xres = primary_display_get_width(), h_yres = primary_display_get_height();

	if (cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &cmdq_handle)) return -1;
	cmdqRecReset(cmdq_handle);

	if (cmdqRecCreate(CMDQ_SCENARIO_DISP_SCREEN_CAPTURE, &cmdq_wait_handle)) {
		cmdqRecDestroy(cmdq_handle);
		return -1;
	}
	cmdqRecReset(cmdq_wait_handle);

	dpmgr_path_memout_clock(pgc->dpmgr_handle, 1);
	_cmdq_handle_clear_dirty(cmdq_handle);
	_cmdq_insert_wait_frame_done_token_mira(cmdq_handle);

	_primary_path_lock(__func__);
	primary_display_idlemgr_kick(__func__, 0);
	dpmgr_path_add_memout(pgc->dpmgr_handle, after_eng, cmdq_handle);
	cmdqRecClearEventToken(cmdq_handle, CMDQ_EVENT_DISP_WDMA0_EOF);

	pconfig = dpmgr_path_get_last_config(pgc->dpmgr_handle);
	pconfig->wdma_dirty = 1;
	pconfig->ovl_dirty = 1;
	pconfig->dst_dirty = 1;
	pconfig->rdma_dirty = 1;
	pconfig->wdma_config.dstAddress = mva;
	pconfig->wdma_config.srcHeight = h_yres;
	pconfig->wdma_config.srcWidth = w_xres;
	pconfig->wdma_config.clipX = 0;
	pconfig->wdma_config.clipY = 0;
	pconfig->wdma_config.clipHeight = h_yres;
	pconfig->wdma_config.clipWidth = w_xres;
	pconfig->wdma_config.outputFormat = ufmt;
	pconfig->wdma_config.useSpecifiedAlpha = 1;
	pconfig->wdma_config.alpha = 0xFF;
	pconfig->wdma_config.dstPitch = w_xres * UFMT_GET_bpp(ufmt) / 8;
	dpmgr_path_config(pgc->dpmgr_handle, pconfig, cmdq_handle);
	pconfig->wdma_dirty = 0;

	_cmdq_set_config_handle_dirty_mira(cmdq_handle);
	_cmdq_flush_config_handle_mira(cmdq_handle, 0);

	cmdqRecWait(cmdq_wait_handle, CMDQ_EVENT_DISP_WDMA0_SOF);
	cmdqRecWait(cmdq_wait_handle, CMDQ_EVENT_DISP_WDMA0_EOF);
	cmdqRecFlush(cmdq_wait_handle);

	cmdqRecReset(cmdq_handle);
	_cmdq_handle_clear_dirty(cmdq_handle);
	_cmdq_insert_wait_frame_done_token_mira(cmdq_handle);

	dpmgr_path_remove_memout(pgc->dpmgr_handle, cmdq_handle);
	cmdqRecClearEventToken(cmdq_handle, CMDQ_EVENT_DISP_WDMA0_SOF);
	_cmdq_set_config_handle_dirty_mira(cmdq_handle);
	cmdqRecFlushAsyncCallback(cmdq_handle, _remove_memout_callback, 0);

	dpmgr_path_memout_clock(pgc->dpmgr_handle, 0);
	_primary_path_unlock(__func__);

	cmdqRecDestroy(cmdq_handle);
	cmdqRecDestroy(cmdq_wait_handle);
	return 0;
}

static int _screen_cap_by_cpu(unsigned int mva, enum UNIFIED_COLOR_FMT ufmt, enum DISP_MODULE_ENUM after_eng)
{
	struct disp_ddp_path_config *pconfig = NULL;
	unsigned int w_xres = primary_display_get_width(), h_yres = primary_display_get_height();

	dpmgr_path_memout_clock(pgc->dpmgr_handle, 1);

	if (_should_wait_path_idle()) dpmgr_wait_event_timeout(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE, HZ * 1);

	_primary_path_lock(__func__);
	primary_display_idlemgr_kick(__func__, 1);

	dpmgr_path_add_memout(pgc->dpmgr_handle, after_eng, NULL);

	pconfig = dpmgr_path_get_last_config(pgc->dpmgr_handle);
	pconfig->wdma_dirty = 1;
	pconfig->wdma_config.dstAddress = mva;
	pconfig->wdma_config.srcHeight = h_yres;
	pconfig->wdma_config.srcWidth = w_xres;
	pconfig->wdma_config.clipX = 0;
	pconfig->wdma_config.clipY = 0;
	pconfig->wdma_config.clipHeight = h_yres;
	pconfig->wdma_config.clipWidth = w_xres;
	pconfig->wdma_config.outputFormat = ufmt;
	pconfig->wdma_config.useSpecifiedAlpha = 1;
	pconfig->wdma_config.alpha = 0xFF;
	pconfig->wdma_config.dstPitch = w_xres * UFMT_GET_bpp(ufmt) / 8;
	dpmgr_path_config(pgc->dpmgr_handle, pconfig, NULL);
	pconfig->wdma_dirty = 0;

	_trigger_display_interface(1, NULL, 0);
	msleep(20);
	if (_should_wait_path_idle()) dpmgr_wait_event_timeout(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE, HZ * 1);

	dpmgr_path_remove_memout(pgc->dpmgr_handle, NULL);
	dpmgr_path_memout_clock(pgc->dpmgr_handle, 0);
	_primary_path_unlock(__func__);
	return 0;
}

int primary_display_capture_framebuffer(unsigned long pbuf)
{
	return -1;
}

static UINT32 disp_fb_bpp = 32;
static UINT32 disp_fb_pages = 3;

UINT32 DISP_GetScreenBpp(void) { return disp_fb_bpp; }
UINT32 DISP_GetPages(void) { return disp_fb_pages; }
UINT32 DISP_GetFBRamSize(void) { return ALIGN_TO(DISP_GetScreenWidth(), MTK_FB_ALIGNMENT) * ALIGN_TO(DISP_GetScreenHeight(), MTK_FB_ALIGNMENT) * ((DISP_GetScreenBpp() + 7) >> 3) * DISP_GetPages(); }
UINT32 DISP_GetVRamSize(void) { return 0; }
UINT32 DISP_GetVRamSizeBoot(char *cmdline) { return mtkfb_get_fb_size(); }

unsigned int primary_display_get_option(const char *option)
{
	if (option[0] == 'F' && option[1] == 'B') return 0;
	if (option[0] == 'A' && option[1] == 'S') return PRIMARY_SESSION_INPUT_LAYER_COUNT - 1;
	if (option[0] == 'M' && option[1] == '4') return disp_helper_get_option(DISP_OPT_USE_M4U);
	return -1;
}

int primary_display_lcm_ATA(void)
{
	_primary_path_switch_dst_lock();
	primary_display_esd_check_enable(0);
	_primary_path_lock(__func__);
	disp_irq_esd_cust_bycmdq(0);
	if (pgc->state == 0) goto done;

	if (primary_display_is_video_mode()) dpmgr_path_ioctl(pgc->dpmgr_handle, NULL, DDP_STOP_VIDEO_MODE, NULL);

	disp_lcm_ATA(pgc->plcm);
	dpmgr_path_start(pgc->dpmgr_handle, CMDQ_DISABLE);
	if (primary_display_is_video_mode()) dpmgr_path_trigger(pgc->dpmgr_handle, NULL, CMDQ_DISABLE);

done:
	disp_irq_esd_cust_bycmdq(1);
	_primary_path_unlock(__func__);
	primary_display_esd_check_enable(1);
	_primary_path_switch_dst_unlock();
	return 0;
}

static int Panel_Master_primary_display_config_dsi(const char *name, UINT32 config_value)
{
	struct disp_ddp_path_config *data_config = dpmgr_path_get_last_config(pgc->dpmgr_handle);
	if (!strcmp(name, "PM_CLK")) data_config->dispif_config.dsi.PLL_CLOCK = config_value;
	else if (!strcmp(name, "PM_SSC")) data_config->dispif_config.dsi.ssc_range = config_value;
	return dpmgr_path_config(pgc->dpmgr_handle, data_config, NULL);
}

int fbconfig_get_esd_check_test(UINT32 dsi_id, UINT32 cmd, UINT8 *buffer, UINT32 num)
{
	int ret = 0;

	_primary_path_lock(__func__);
	if (pgc->state == DISP_SLEPT) {
		_primary_path_unlock(__func__);
		return 0;
	}

	primary_display_idlemgr_kick(__func__, 0);

	primary_display_esd_check_enable(0);
	disp_irq_esd_cust_bycmdq(0);

	_cmdq_stop_trigger_loop();
	if (dpmgr_path_is_busy(pgc->dpmgr_handle)) {}

	dpmgr_path_stop(pgc->dpmgr_handle, CMDQ_DISABLE);
	if (dpmgr_path_is_busy(pgc->dpmgr_handle)) {}

	dpmgr_path_reset(pgc->dpmgr_handle, CMDQ_DISABLE);
	ret = fbconfig_get_esd_check(dsi_id, cmd, buffer, num);
	dpmgr_path_start(pgc->dpmgr_handle, CMDQ_DISABLE);

	if (primary_display_is_video_mode()) dpmgr_path_trigger(pgc->dpmgr_handle, NULL, CMDQ_DISABLE);

	_cmdq_start_trigger_loop();
	cmdqCoreSetEvent(CMDQ_EVENT_DISP_WDMA0_EOF);

	disp_irq_esd_cust_bycmdq(1);
	primary_display_esd_check_enable(1);
	_primary_path_unlock(__func__);

	return ret;
}

int Panel_Master_dsi_config_entry(const char *name, void *config_value)
{
	int force_trigger_path = 0;
	UINT32 *config_dsi = (UINT32 *)config_value;
	struct LCM_DRIVER *pLcm_drv = DISP_GetLcmDrv();

	if (!strcmp(name, "DRIVER_IC_RESET") || !strcmp(name, "PM_DDIC_CONFIG")) {
		primary_display_esd_check_enable(0);
		msleep(2500);
	}

	_primary_path_lock(__func__);
	if (pgc->state == DISP_SLEPT) goto done;

	primary_display_idlemgr_kick(__func__, 0);
	_cmdq_stop_trigger_loop();

	if (dpmgr_path_is_busy(pgc->dpmgr_handle)) dpmgr_wait_event_timeout(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE, HZ * 1);
	dpmgr_path_stop(pgc->dpmgr_handle, CMDQ_DISABLE);
	dpmgr_path_reset(pgc->dpmgr_handle, CMDQ_DISABLE);
	dsi_basic_irq_enable(DISP_MODULE_DSI0, NULL);

	if ((!strcmp(name, "PM_CLK")) || (!strcmp(name, "PM_SSC"))) Panel_Master_primary_display_config_dsi(name, *config_dsi);
	else if (!strcmp(name, "PM_DDIC_CONFIG")) {
		Panel_Master_DDIC_config();
		force_trigger_path = 1;
	} else if (!strcmp(name, "DRIVER_IC_RESET")) {
		if (pLcm_drv && pLcm_drv->init_power) pLcm_drv->init_power();
		if (pLcm_drv) pLcm_drv->init();
		force_trigger_path = 1;
	}

	dpmgr_path_start(pgc->dpmgr_handle, CMDQ_DISABLE);
	if (primary_display_is_video_mode()) {
		dpmgr_path_trigger(pgc->dpmgr_handle, NULL, CMDQ_DISABLE);
		force_trigger_path = 0;
	}
	_cmdq_start_trigger_loop();
	cmdqCoreSetEvent(CMDQ_EVENT_DISP_WDMA0_EOF);

done:
	_primary_path_unlock(__func__);
	if (force_trigger_path) primary_display_trigger(0, NULL, 0);
	if (!strcmp(name, "DRIVER_IC_RESET") || !strcmp(name, "PM_DDIC_CONFIG")) primary_display_esd_check_enable(1);

	return 0;
}

int primary_display_switch_dst_mode(int mode)
{
	disp_path_handle disp_handle = NULL;
	struct disp_ddp_path_config *pconfig = NULL;
	void *lcm_cmd = NULL;
	int temp_mode = 0;

	if (!disp_helper_get_option(DISP_OPT_CV_BYSUSPEND)) return 0;

	_primary_path_switch_dst_lock();
	disp_sw_mutex_lock(&(pgc->capture_lock));
	_primary_path_lock(__func__);

	if (pgc->plcm->params->type != LCM_TYPE_DSI) goto done;
	if (pgc->state == DISP_SLEPT) goto done;
	if (pgc->lcm_refresh_rate != 120) goto done;
	if (mode == primary_display_cur_dst_mode) goto done;

	lcm_cmd = disp_lcm_switch_mode(pgc->plcm, mode);
	if (!lcm_cmd) goto done;

	if (disp_helper_get_option(DISP_OPT_SMART_OVL) && !primary_display_is_video_mode()) {
		do_primary_display_switch_mode(DISP_SESSION_DIRECT_LINK_MODE, pgc->session_id, 0, NULL, 0);
		set_is_dc(0);
	}

#ifndef CONFIG_FPGA_EARLY_PORTING
	if (disp_helper_get_option(DISP_OPT_SODI_SUPPORT)) ddp_set_spm_mode(DDP_CG_MODE, NULL);
#endif

	_cmdq_reset_config_handle();

	temp_mode = (int)(pgc->plcm->params->dsi.mode);
	pgc->plcm->params->dsi.mode = pgc->plcm->params->dsi.switch_mode;
	pgc->plcm->params->dsi.switch_mode = temp_mode;
	dpmgr_path_set_video_mode(pgc->dpmgr_handle, primary_display_is_video_mode());

	disp_lcm_adjust_fps(pgc->cmdq_handle_config, pgc->plcm, pgc->lcm_refresh_rate);
	disp_handle = pgc->dpmgr_handle;
	pconfig = dpmgr_path_get_last_config(disp_handle);
	pconfig->dispif_config.dsi.PLL_CLOCK = pgc->plcm->params->dsi.PLL_CLOCK;

	dpmgr_path_ioctl(pgc->dpmgr_handle, pgc->cmdq_handle_config, DDP_UPDATE_PLL_CLK_ONLY, &pgc->plcm->params->dsi.PLL_CLOCK);

	if (dpmgr_path_ioctl(pgc->dpmgr_handle, pgc->cmdq_handle_config, DDP_SWITCH_DSI_MODE, lcm_cmd) != 0) goto done;

	_cmdq_build_trigger_loop();
	_cmdq_stop_trigger_loop();
	_cmdq_start_trigger_loop();
	_cmdq_reset_config_handle();
	_cmdq_insert_wait_frame_done_token_mira(pgc->cmdq_handle_config);

	primary_display_cur_dst_mode = mode;
	if (primary_display_is_video_mode()) dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC, DDP_IRQ_RDMA0_DONE);
	else dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC, DDP_IRQ_DSI0_EXT_TE);

done:
	_primary_path_unlock(__func__);
	disp_sw_mutex_unlock(&(pgc->capture_lock));
	_primary_path_switch_dst_unlock();
	return 0;
}

static int width_array[] = {2560, 1440, 1920, 1280, 1200, 800, 960, 640};
static int heigh_array[] = {1440, 2560, 1200, 800, 1920, 1280, 640, 960};
static int array_id[] = {6, 2, 7, 4, 3, 0, 5, 1};
struct LCM_PARAMS *lcm_param2;
struct disp_ddp_path_config data_config2;

int primary_display_te_test(void)
{
	int ret = 0, try_cnt = 3, time_interval = 0, time_interval_max = 0;
	long long time_te = 0, time_framedone = 0;

	if (primary_display_is_video_mode()) return 0;

	while (try_cnt >= 0) {
		try_cnt--;
		dpmgr_wait_event_timeout(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC, HZ*1);
		time_te = sched_clock();

		dpmgr_path_trigger(pgc->dpmgr_handle, NULL, CMDQ_DISABLE);
		dpmgr_wait_event_timeout(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE, HZ*1);
		time_framedone = sched_clock();
		time_interval = (int) (time_framedone - time_te) / 1000;
		if (time_interval > time_interval_max) time_interval_max = time_interval;
	}

	return (time_interval_max > 20000) ? 0 : -1;
}

int primary_display_roi_test(int x, int y)
{
	DSI_set_roi(x, y);
	msleep(50);

	dpmgr_path_trigger(pgc->dpmgr_handle, NULL, CMDQ_DISABLE);
	msleep(50);

	DSI_check_roi();
	msleep(20);

	DSI_set_roi(0, 0);
	msleep(20);
	return 0;
}

int primary_display_resolution_test(void)
{
	int i = 0;
	unsigned int w_backup = 0, h_backup = 0;
	int dst_width = 0, dst_heigh = 0;
	enum LCM_DSI_MODE_CON dsi_mode_backup = primary_display_is_video_mode();

	memset(&data_config2, 0, sizeof(data_config2));
	lcm_param2 = NULL;
	memcpy(&data_config2, dpmgr_path_get_last_config(pgc->dpmgr_handle), sizeof(struct disp_ddp_path_config));
	w_backup = data_config2.dst_w;
	h_backup = data_config2.dst_h;

	DSI_ForceConfig(1);
	for (i = 0; i < sizeof(width_array)/sizeof(int); i++) {
		dst_width = width_array[i];
		dst_heigh = heigh_array[i];
		lcm_param2 = disp_lcm_get_params(pgc->plcm);
		lcm_param2->dsi.mode = CMD_MODE;
		lcm_param2->dsi.horizontal_active_pixel = dst_width;
		lcm_param2->dsi.vertical_active_line = dst_heigh;

		data_config2.dispif_config.dsi.mode = CMD_MODE;
		data_config2.dispif_config.dsi.horizontal_active_pixel = dst_width;
		data_config2.dispif_config.dsi.vertical_active_line = dst_heigh;
		data_config2.dst_w = dst_width;
		data_config2.dst_h = dst_heigh;

		data_config2.ovl_config[0].layer    = 0;
		data_config2.ovl_config[0].layer_en = 0;
		data_config2.ovl_config[1].layer    = 1;
		data_config2.ovl_config[1].layer_en = 0;
		data_config2.ovl_config[2].layer    = 2;
		data_config2.ovl_config[2].layer_en = 0;
		data_config2.ovl_config[3].layer    = 3;
		data_config2.ovl_config[3].layer_en = 0;

		data_config2.dst_dirty = 1;
		data_config2.ovl_dirty = 1;

		dpmgr_path_set_video_mode(pgc->dpmgr_handle, primary_display_is_video_mode());
		dpmgr_path_config(pgc->dpmgr_handle, &data_config2, NULL);
		data_config2.dst_dirty = 0;
		data_config2.ovl_dirty = 0;

		dpmgr_path_start(pgc->dpmgr_handle, CMDQ_DISABLE);
		dpmgr_path_trigger(pgc->dpmgr_handle, NULL, CMDQ_DISABLE);
		dpmgr_wait_event_timeout(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE, HZ*1);
		dpmgr_path_reset(pgc->dpmgr_handle, CMDQ_DISABLE);
	}

	dpmgr_path_stop(pgc->dpmgr_handle, CMDQ_DISABLE);
	lcm_param2 = disp_lcm_get_params(pgc->plcm);
	lcm_param2->dsi.mode = dsi_mode_backup;
	lcm_param2->dsi.vertical_active_line = h_backup;
	lcm_param2->dsi.horizontal_active_pixel = w_backup;
	data_config2.dispif_config.dsi.vertical_active_line = h_backup;
	data_config2.dispif_config.dsi.horizontal_active_pixel = w_backup;
	data_config2.dispif_config.dsi.mode = dsi_mode_backup;
	data_config2.dst_w = w_backup;
	data_config2.dst_h = h_backup;
	data_config2.dst_dirty = 1;

	dpmgr_path_set_video_mode(pgc->dpmgr_handle, primary_display_is_video_mode());
	dpmgr_path_connect(pgc->dpmgr_handle, CMDQ_DISABLE);
	dpmgr_path_config(pgc->dpmgr_handle, &data_config2, NULL);
	data_config2.dst_dirty = 0;
	DSI_ForceConfig(0);
	return 0;
}

int primary_display_check_test(void)
{
	int esd_backup = 1;
	_primary_path_lock(__func__);
	primary_display_esd_check_enable(0);
	msleep(2000);

	if (pgc->state == DISP_SLEPT) goto done;

	_cmdq_stop_trigger_loop();
	if (dpmgr_path_is_busy(pgc->dpmgr_handle)) dpmgr_wait_event_timeout(pgc->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE, HZ*1);

	primary_display_resolution_test();

	dpmgr_path_start(pgc->dpmgr_handle, CMDQ_DISABLE);
	_cmdq_start_trigger_loop();

done:
	if (esd_backup == 1) primary_display_esd_check_enable(1);
	_primary_path_unlock(__func__);
	return 0;
}

struct OPT_BACKUP tui_opt_backup[4] = {
	{DISP_OPT_SHARE_SRAM, 0},
	{DISP_OPT_IDLEMGR_SWTCH_DECOUPLE, 0},
	{DISP_OPT_SMART_OVL, 0},
	{DISP_OPT_BYPASS_OVL, 0}
};

void stop_smart_ovl_nolock(void)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(tui_opt_backup); i++) {
		tui_opt_backup[i].value = disp_helper_get_option(tui_opt_backup[i].option);
		disp_helper_set_option(tui_opt_backup[i].option, 0);
	}
	for (i = 0; i < ARRAY_SIZE(tui_opt_backup); i++) {
		if ((tui_opt_backup[i].option == DISP_OPT_SHARE_SRAM) && (tui_opt_backup[i].value == 1))
			leave_share_sram(CMDQ_SYNC_RESOURCE_WROT0);
	}
}

void restart_smart_ovl_nolock(void)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(tui_opt_backup); i++)
		disp_helper_set_option(tui_opt_backup[i].option, tui_opt_backup[i].value);

	for (i = 0; i < ARRAY_SIZE(tui_opt_backup); i++) {
		if ((tui_opt_backup[i].option == DISP_OPT_SHARE_SRAM) && (tui_opt_backup[i].value == 1))
			enter_share_sram(CMDQ_SYNC_RESOURCE_WROT0);
	}
}

static enum DISP_POWER_STATE tui_power_stat_backup;
static int tui_session_mode_backup;
static struct DDP_MODULE_DRIVER *ddp_module_backup;

int display_vsync_switch_to_dsi(unsigned int flg)
{
	if (!primary_display_is_video_mode()) return 0;
	if (!flg) {
		dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC, DDP_IRQ_RDMA0_DONE);
		dsi_enable_irq(DISP_MODULE_DSI0, NULL, 0);
	} else {
		dpmgr_map_event_to_irq(pgc->dpmgr_handle, DISP_PATH_EVENT_IF_VSYNC, DDP_IRQ_DSI0_FRAME_DONE);
		dsi_enable_irq(DISP_MODULE_DSI0, NULL, 1);
	}
	return 0;
}

int display_enter_tui(void)
{
	int i;
	_primary_path_lock(__func__);

	if (primary_get_state() != DISP_ALIVE) goto err0;
	tui_power_stat_backup = primary_set_state(DISP_BLANK);

	primary_display_idlemgr_kick(__func__, 0);

	if (primary_display_is_mirror_mode()) goto err1;

	stop_smart_ovl_nolock();

	tui_session_mode_backup = pgc->session_mode;

	if (tui_session_mode_backup == DISP_SESSION_RDMA_MODE) {
		do_primary_display_switch_mode(DISP_SESSION_DIRECT_LINK_MODE, pgc->session_id, 0, NULL, 0);
		tui_session_mode_backup = DISP_SESSION_DIRECT_LINK_MODE;
	}

	if (disp_helper_get_option(DISP_OPT_TUI_MODE) == TUI_SINGLE_WINDOW_MODE) {
		do_primary_display_switch_mode(DISP_SESSION_DECOUPLE_MODE, pgc->session_id, 0, NULL, 0);
	} else if (disp_helper_get_option(DISP_OPT_TUI_MODE) == TUI_MULTIPLE_WINDOW_MODE) {
		do_primary_display_switch_mode(DISP_SESSION_DIRECT_LINK_MODE, pgc->session_id, 0, NULL, 0);
		ddp_module_backup = ddp_get_module_driver(DISP_MODULE_OVL0_2L);
		ddp_set_module_driver(DISP_MODULE_OVL0_2L, 0);
	}

	display_vsync_switch_to_dsi(1);
	_primary_path_unlock(__func__);
	return 0;

err1:
	primary_set_state(tui_power_stat_backup);
err0:
	_primary_path_unlock(__func__);
	return -1;
}

int display_exit_tui(void)
{
	_primary_path_lock(__func__);
	primary_set_state(tui_power_stat_backup);

	_decouple_update_rdma_config_nolock();
	do_primary_display_switch_mode(tui_session_mode_backup, pgc->session_id, 0, NULL, 0);
	if (disp_helper_get_option(DISP_OPT_TUI_MODE) == TUI_MULTIPLE_WINDOW_MODE)
		ddp_set_module_driver(DISP_MODULE_OVL0_2L, ddp_module_backup);

	restart_smart_ovl_nolock();
	display_vsync_switch_to_dsi(0);
	_primary_path_unlock(__func__);

	return 0;
}

void ddp_irq_callback(enum DISP_MODULE_ENUM module, unsigned int reg_value)
{
	cmdqCoreSetEvent(CMDQ_SYNC_TOKEN_CONFIG_DIRTY);
}

static int self_refresh_idlemgr_status_backup;
static int primary_display_enter_self_refresh(void)
{
	_primary_path_lock(__func__);
	if (primary_display_is_mirror_mode()) goto out;

	self_refresh_idlemgr_status_backup = set_idlemgr(0, 0);
	stop_smart_ovl_nolock();
	do_primary_display_switch_mode(DISP_SESSION_DIRECT_LINK_MODE, pgc->session_id, 0, NULL, 1);

	if (!primary_display_is_video_mode()) disp_register_irq_callback(ddp_irq_callback);

	pgc->primary_display_scenario = DISP_SCENARIO_SELF_REFRESH;
out:
	_primary_path_unlock(__func__);
	return 0;
}

static int primary_display_exit_self_refresh(void)
{
	_primary_path_lock(__func__);
	if (primary_display_is_mirror_mode()) goto out;

	set_idlemgr(self_refresh_idlemgr_status_backup, 0);
	restart_smart_ovl_nolock();

	if (!primary_display_is_video_mode()) disp_unregister_irq_callback(ddp_irq_callback);

	pgc->primary_display_scenario = DISP_SCENARIO_NORMAL;
out:
	_primary_path_unlock(__func__);
	return 0;
}

int primary_display_set_scenario(int scenario)
{
	if (scenario != DISP_SCENARIO_NORMAL && pgc->primary_display_scenario != DISP_SCENARIO_NORMAL) return -EINVAL;
	if (scenario == DISP_SCENARIO_SELF_REFRESH) return primary_display_enter_self_refresh();
	if (scenario == DISP_SCENARIO_NORMAL && pgc->primary_display_scenario == DISP_SCENARIO_SELF_REFRESH) return primary_display_exit_self_refresh();
	return 0;
}

#ifdef CONFIG_MTK_HIGH_FRAME_RATE
unsigned int primary_display_is_support_DynFPS(void)
{
	return (disp_helper_get_option(DISP_OPT_DYNAMIC_FPS) && primary_display_is_video_mode() && disp_lcm_is_dynfps_support(pgc->plcm));
}

int primary_display_get_multi_configs(struct multi_configs *p_cfgs)
{
	struct LCM_PARAMS *params;
	struct dfps_info *dfps_info;
	unsigned int def_width = primary_display_get_width();
	unsigned int def_height = primary_display_get_height();
	unsigned int def_disp_fps = primary_display_get_default_disp_fps(0);
	unsigned int multi_cfg_num = 1, i = 0;

	if (!p_cfgs) return -1;

	memcpy(p_cfgs, &(pgc->multi_cfg_table), sizeof(pgc->multi_cfg_table));

	if (!primary_display_is_support_DynFPS()) {
		p_cfgs->config_num = 1;
		p_cfgs->dyn_cfgs[0].vsyncFPS = def_disp_fps;
		p_cfgs->dyn_cfgs[0].vact_timing_fps = def_disp_fps;
		p_cfgs->dyn_cfgs[0].width = def_width;
		p_cfgs->dyn_cfgs[0].height = def_height;
	} else {
		params = pgc->plcm->params;
		dfps_info = params->dsi.dfps_params;

		multi_cfg_num = params->dsi.dfps_num;
		if (multi_cfg_num > MULTI_CONFIG_NUM) multi_cfg_num = MULTI_CONFIG_NUM;

		for (i = 0; i < multi_cfg_num; i++) {
			p_cfgs->dyn_cfgs[i].vsyncFPS = dfps_info[i].fps;
			p_cfgs->dyn_cfgs[i].vact_timing_fps = dfps_info[i].vact_timing_fps;
			p_cfgs->dyn_cfgs[i].width = def_width;
			p_cfgs->dyn_cfgs[i].height = def_height;
		}
		p_cfgs->config_num = multi_cfg_num;
	}
	return 0;
}

unsigned int primary_display_get_current_disp_fps(void) { return pgc->current_disp_fps; }
unsigned int primary_display_get_current_cfg_id(void) { return pgc->active_cfg; }
void primary_display_update_cfg_id(int cfg_id) { pgc->active_cfg = cfg_id; }

void primary_display_init_multi_cfg_info(void)
{
	unsigned int def_vsync_fps = disp_lcm_dynfps_get_def_fps(pgc->plcm);
	unsigned int def_width = primary_display_get_width();
	unsigned int def_height = primary_display_get_height();
	struct LCM_PARAMS *params = NULL;
	struct dfps_info *dfps_info;
	unsigned int multi_cfg_num = 1, i = 0;

	def_vsync_fps = def_vsync_fps ? def_vsync_fps : 6400;

	if (primary_display_is_support_DynFPS() && disp_lcm_dynfps_get_dfps_num(pgc->plcm) > 0) {
		params = pgc->plcm->params;
		dfps_info = params->dsi.dfps_params;

		multi_cfg_num = params->dsi.dfps_num;
		if (multi_cfg_num > MULTI_CONFIG_NUM) multi_cfg_num = MULTI_CONFIG_NUM;

		for (i = 0; i < multi_cfg_num; i++) {
			pgc->multi_cfg_table.dyn_cfgs[i].vsyncFPS = dfps_info[i].fps;
			pgc->multi_cfg_table.dyn_cfgs[i].vact_timing_fps = dfps_info[i].vact_timing_fps;
			pgc->multi_cfg_table.dyn_cfgs[i].width = def_width;
			pgc->multi_cfg_table.dyn_cfgs[i].height = def_height;
		}
		pgc->multi_cfg_table.config_num = multi_cfg_num;
	} else {
		pgc->multi_cfg_table.config_num = 1;
		pgc->multi_cfg_table.dyn_cfgs[0].vsyncFPS = def_vsync_fps;
		pgc->multi_cfg_table.dyn_cfgs[0].vact_timing_fps = def_vsync_fps;
		pgc->multi_cfg_table.dyn_cfgs[0].width = def_width;
		pgc->multi_cfg_table.dyn_cfgs[0].height = def_height;
	}

	pgc->lcm_refresh_rate = def_vsync_fps / 100;
	pgc->lcm_fps = def_vsync_fps;
	pgc->active_cfg = 0;
}

unsigned int primary_display_get_default_disp_fps(int need_lock)
{
	unsigned int _default_disp_fps = 6400;
	if (need_lock) _primary_path_lock(__func__);
	_default_disp_fps = disp_lcm_dynfps_get_def_fps(pgc->plcm);
	_default_disp_fps = _default_disp_fps ? _default_disp_fps : 6400;
	if (need_lock) _primary_path_unlock(__func__);
	return _default_disp_fps;
}

unsigned int primary_display_get_def_timing_fps(int need_lock)
{
	unsigned int _def_timing_fps = 6400;
	unsigned int _def_disp_fps = 6400;

	if (need_lock) _primary_path_lock(__func__);
	_def_disp_fps = primary_display_get_default_disp_fps(0);
	_def_timing_fps = disp_lcm_dynfps_get_def_timing_fps(pgc->plcm);
	_def_timing_fps = _def_timing_fps ? _def_timing_fps : _def_disp_fps;
	if (need_lock) _primary_path_unlock(__func__);
	return _def_timing_fps;
}

int primary_display_get_cfg_fps(int config_id, unsigned int *fps, unsigned int *vact_timing_fps)
{
	unsigned int _vsyncFPS = primary_display_get_default_disp_fps(0);
	unsigned int _timing_fps = primary_display_get_def_timing_fps(0);
	struct multi_configs *p_cfgs = NULL;
	struct dyn_config_info *dyn_cfgs = NULL;

	if (primary_display_is_support_DynFPS()) {
		p_cfgs = &(pgc->multi_cfg_table);
		if (p_cfgs->config_num > 0 && config_id < p_cfgs->config_num) {
			dyn_cfgs = p_cfgs->dyn_cfgs;
			_vsyncFPS = dyn_cfgs[config_id].vsyncFPS;
			_timing_fps = dyn_cfgs[config_id].vact_timing_fps;
		}
	}
	if (fps) *fps = _vsyncFPS;
	if (vact_timing_fps) *vact_timing_fps = _timing_fps;

	return 0;
}

unsigned int primary_display_get_vfp(unsigned int fps)
{
	unsigned int vfp;
	struct LCM_PARAMS *params = pgc->plcm->params;
	unsigned int i = 0;
	unsigned int fps_levels = (params->dsi).dynamic_fps_levels;

	vfp = (params->dsi).vertical_frontporch;
	for (i = 0; i < fps_levels; i++) {
		if (fps == (params->dsi).dynamic_fps_table[i].fps) {
			vfp = (params->dsi).dynamic_fps_table[i].vfp;
			break;
		}
	}
	return vfp;
}

unsigned int primary_display_get_idle_interval(unsigned int fps)
{
	return (3 * 1000) / fps + 1;
}

void primary_display_dynfps_chg_fps(int cfg_id)
{
	int last_cfg_id;
	unsigned int new_dynfps, last_dynfps, fps_change_index;
	bool need_send_cmd = false;
	struct cmdqRecStruct *qhandle = NULL;
	int ret = 0;
	unsigned int _idle_timeout = 50;

	last_cfg_id = primary_display_get_current_cfg_id();
	if (cfg_id == last_cfg_id) return;

	primary_display_get_cfg_fps(last_cfg_id, &last_dynfps, NULL);
	primary_display_get_cfg_fps(cfg_id, &new_dynfps, NULL);

	if (new_dynfps == last_dynfps) return;

	fps_change_index = ddp_dsi_fps_change_index(last_dynfps, new_dynfps);
	need_send_cmd = disp_lcm_need_send_cmd(pgc->plcm, last_dynfps, new_dynfps);

	if (fps_change_index & DYNFPS_DSI_MIPI_CLK || fps_change_index & DYNFPS_DSI_HFP) {
		ret = cmdqRecCreate(CMDQ_SCENARIO_DISP_ESD_CHECK, &qhandle);
		if (ret) return;

		cmdqRecReset(qhandle);
		cmdqRecWait(qhandle, CMDQ_EVENT_MUTEX0_STREAM_EOF);

		if (need_send_cmd || (fps_change_index & DYNFPS_DSI_MIPI_CLK)) dpmgr_path_build_cmdq(pgc->dpmgr_handle, qhandle, CMDQ_STOP_VDO_MODE, 0);
		if (need_send_cmd) disp_lcm_dynfps_send_cmd(pgc->plcm, qhandle, last_dynfps, new_dynfps);

		ddp_dsi_dynfps_chg_fps(DISP_MODULE_DSI0, qhandle, last_dynfps, new_dynfps, fps_change_index);

		if (need_send_cmd || (fps_change_index & DYNFPS_DSI_MIPI_CLK)) {
			dpmgr_path_build_cmdq(pgc->dpmgr_handle, qhandle, CMDQ_START_VDO_MODE, 0);
			cmdqRecClearEventToken(qhandle, CMDQ_EVENT_MUTEX0_STREAM_EOF);
			dpmgr_path_trigger(primary_get_dpmgr_handle(), qhandle, CMDQ_ENABLE);
		}
		cmdqRecFlushAsync(qhandle);

	} else if (fps_change_index & DYNFPS_DSI_VFP) {
		if (!need_send_cmd) {
			ret = cmdqRecCreate(CMDQ_SCENARIO_DISP_ESD_CHECK, &qhandle);
			if (ret) return;
			cmdqRecReset(qhandle);
			cmdqRecClearEventToken(qhandle, CMDQ_EVENT_DISP_RDMA0_SOF);
			cmdqRecWaitNoClear(qhandle, CMDQ_EVENT_DISP_RDMA0_SOF);
			ddp_dsi_dynfps_chg_fps(DISP_MODULE_DSI0, qhandle, last_dynfps, new_dynfps, fps_change_index);
			cmdqRecFlushAsync(qhandle);
		}
	}
	cmdqRecDestroy(qhandle);

	disp_invoke_fps_chg_callbacks(new_dynfps / 100);
	_idle_timeout = primary_display_get_idle_interval(new_dynfps / 100);
	disp_lp_set_idle_check_interval(_idle_timeout);

	primary_display_update_cfg_id(cfg_id);
	pgc->lcm_refresh_rate = new_dynfps / 100;
	pgc->lcm_fps = new_dynfps;
}

unsigned int primary_display_is_support_ARR(void)
{
	return (disp_helper_get_option(DISP_OPT_ARR_PHASE_1) && primary_display_is_video_mode() && disp_lcm_is_arr_support(pgc->plcm));
}

void primary_display_dynfps_get_vfp_info(unsigned int *vfp, unsigned int *vfp_for_lp)
{
	unsigned int cfg_id = primary_display_get_current_cfg_id();
	unsigned int fps;

	primary_display_get_cfg_fps(cfg_id, &fps, NULL);
	ddp_dsi_dynfps_get_vfp_info(fps, vfp, vfp_for_lp);
}
#endif
