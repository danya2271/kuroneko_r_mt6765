// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/pm_wakeup.h>
#include <linux/atomic.h>
#include <linux/printk.h>

#include "disp_utils.h"
#include "ddp_path.h"
#include "ddp_drv.h"
#include "ddp_clkmgr.h"
#include "disp_helper.h"
#include "disp_session.h"
#include "primary_display.h"
#include "disp_lowpower.h"

#ifdef CONFIG_MTK_M4U
#include "m4u.h"
#include "m4u_port.h"
#endif

#include "cmdq_def.h"
#include "cmdq_record.h"
#include "cmdq_reg.h"
#include "cmdq_core.h"

#include "ddp_manager.h"
#include "disp_drv_platform.h"
#include "mtk_ovl.h"
#include "mtkfb_fence.h"
#include "extd_platform.h"

static int is_context_inited;
static int ovl2mem_layer_num;
#if defined(CONFIG_MTK_M4U)
static int ovl2mem_use_m4u = 1;
#endif
static int ovl2mem_use_cmdq = CMDQ_ENABLE;

struct wakeup_source *memout_wk_lock;

struct ovl2mem_path_context {
	int state;
	unsigned int session;
	unsigned int lcm_fps;
	int max_layer;
	int need_trigger_path;
	struct mutex lock;
	struct cmdqRecStruct *cmdq_handle_config;
	struct cmdqRecStruct *cmdq_handle_trigger;
	disp_path_handle dpmgr_handle;
	char *mutex_locker;
	cmdqBackupSlotHandle ovl2mem_cur_config_fence;
	cmdqBackupSlotHandle ovl2mem_subtractor_when_free;
};

atomic_t g_trigger_ticket = ATOMIC_INIT(1);
atomic_t g_release_ticket = ATOMIC_INIT(1);

#define pgcl	_get_context_l()
#define MEMORY_SESSION_ID 0x30002

static struct ovl2mem_path_context *_get_context_l(void)
{
	static struct ovl2mem_path_context g_context;

	if (!is_context_inited) {
		memset(&g_context, 0, sizeof(struct ovl2mem_path_context));
		mutex_init(&(g_context.lock));
		is_context_inited = 1;
		memout_wk_lock = wakeup_source_register(NULL, "mem_disp_wakelock");
	}

	return &g_context;
}

enum CMDQ_SWITCH ovl2mem_cmdq_enabled(void)
{
	return ovl2mem_use_cmdq;
}

static void _ovl2mem_path_lock(const char *caller)
{
	disp_sw_mutex_lock(&(pgcl->lock));
	pgcl->mutex_locker = (char *)caller;
}

static void _ovl2mem_path_unlock(const char *caller)
{
	pgcl->mutex_locker = NULL;
	disp_sw_mutex_unlock(&(pgcl->lock));
}

void ovl2mem_context_init(void)
{
	is_context_inited = 0;
	ovl2mem_layer_num = 0;
}

void ovl2mem_setlayernum(int layer_num)
{
	ovl2mem_layer_num = layer_num;
}

int ovl2mem_get_info(void *info)
{
	struct disp_session_info *dispif_info = (struct disp_session_info *) info;

	memset(dispif_info, 0, sizeof(struct disp_session_info));

	dispif_info->maxLayerNum = ovl2mem_layer_num;
	dispif_info->displayType = DISP_IF_TYPE_DPI;
	dispif_info->displayMode = DISP_IF_MODE_VIDEO;
	dispif_info->isHwVsyncAvailable = 1;
	dispif_info->displayFormat = DISP_IF_FORMAT_RGB888;
	dispif_info->displayWidth = primary_display_get_width();
	dispif_info->displayHeight = primary_display_get_height();
	dispif_info->vsyncFPS = pgcl->lcm_fps;
	dispif_info->physicalHeight = 0;
	dispif_info->physicalWidth = 0;
	dispif_info->isConnected = 1;

	return 0;
}

static int _convert_disp_input_to_ovl(struct OVL_CONFIG_STRUCT *dst, struct disp_input_config *src)
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

static int ovl2mem_callback(unsigned int userdata)
{
	int fence_idx = 0;
	int layid = 0;
	int subtractor = 0;

	_ovl2mem_path_lock(__func__);

	for (layid = 0; layid < (MEMORY_SESSION_INPUT_LAYER_COUNT); layid++) {
		cmdqBackupReadSlot(pgcl->ovl2mem_cur_config_fence, layid, &fence_idx);
		cmdqBackupReadSlot(pgcl->ovl2mem_subtractor_when_free, layid, &subtractor);
		mtkfb_release_fence(pgcl->session, layid, fence_idx - subtractor);
	}

	layid = disp_sync_get_output_timeline_id();
	fence_idx = mtkfb_query_idx_by_ticket(pgcl->session, layid, userdata);
	if (fence_idx >= 0) {
		mtkfb_release_fence(pgcl->session, layid, fence_idx);
	}

	atomic_set(&g_release_ticket, userdata);

	_ovl2mem_path_unlock(__func__);

	return 0;
}

int get_ovl2mem_ticket(void)
{
	return atomic_read(&g_trigger_ticket);
}

static int init_cmdq_slots(cmdqBackupSlotHandle *pSlot, int count, int init_val)
{
	int i;
	cmdqBackupAllocateSlot(pSlot, count);
	for (i = 0; i < count; i++) cmdqBackupWriteSlot(*pSlot, i, init_val);
	return 0;
}

static void deinit_cmdq_slots(cmdqBackupSlotHandle hSlot)
{
	cmdqBackupFreeSlot(hSlot);
}

int ovl2mem_init(unsigned int session)
{
	int ret = -1;
#if defined(CONFIG_MTK_M4U)
	struct m4u_port_config_struct sPort;
#endif

	dpmgr_init();
	_ovl2mem_path_lock(__func__);

	if (pgcl->state > 0) goto Exit;

	if (pgcl->state == 0) {
		init_cmdq_slots(&(pgcl->ovl2mem_cur_config_fence), MEMORY_SESSION_INPUT_LAYER_COUNT, 0);
		init_cmdq_slots(&(pgcl->ovl2mem_subtractor_when_free), MEMORY_SESSION_INPUT_LAYER_COUNT, 0);
	}

	if (pgcl->cmdq_handle_config == NULL) {
		ret = cmdqRecCreate(CMDQ_SCENARIO_SUB_DISP, &(pgcl->cmdq_handle_config));
		if (ret) goto Exit;
	}

	cmdqRecSetEngine(pgcl->cmdq_handle_config, (1LL << CMDQ_ENG_DISP_2L_OVL1) | (1LL << CMDQ_ENG_DISP_WDMA0));
	cmdqRecReset(pgcl->cmdq_handle_config);
	cmdqRecClearEventToken(pgcl->cmdq_handle_config, CMDQ_EVENT_DISP_WDMA0_EOF);

	pgcl->dpmgr_handle = dpmgr_create_path(DDP_SCENARIO_SUB_OVL_MEMOUT, pgcl->cmdq_handle_config);

	if (!pgcl->dpmgr_handle) goto Exit;

	dpmgr_path_set_video_mode(pgcl->dpmgr_handle, false);
	dpmgr_path_init(pgcl->dpmgr_handle, CMDQ_DISABLE);
	dpmgr_path_reset(pgcl->dpmgr_handle, CMDQ_DISABLE);

#if defined(CONFIG_MTK_M4U)
	sPort.ePortID = M4U_PORT_UNKNOWN;
	sPort.Virtuality = ovl2mem_use_m4u;
	sPort.Security = 0;
	sPort.Distance = 1;
	sPort.Direction = 0;
	ret = m4u_config_port(&sPort);
	if (ret != 0) goto Exit;

	sPort.ePortID = M4U_PORT_DISP_WDMA0;
	ret = m4u_config_port(&sPort);
	if (ret != 0) goto Exit;
#endif

	dpmgr_enable_event(pgcl->dpmgr_handle, DISP_PATH_EVENT_FRAME_COMPLETE);

	pgcl->max_layer = 4;
	pgcl->state = 1;
	pgcl->session = session;
	atomic_set(&g_trigger_ticket, 1);
	atomic_set(&g_release_ticket, 0);
	__pm_stay_awake(memout_wk_lock);
	ret = 0;

Exit:
	_ovl2mem_path_unlock(__func__);
	return ret;
}

int ovl2mem_trigger(int blocking, void *callback, unsigned int userdata)
{
	if (pgcl->need_trigger_path == 0) return -1;

	cmdqRecClearEventToken(pgcl->cmdq_handle_config, CMDQ_SYNC_DISP_EXT_STREAM_EOF);
	cmdqRecClearEventToken(pgcl->cmdq_handle_config, CMDQ_EVENT_DISP_WDMA0_EOF);
	dpmgr_path_start(pgcl->dpmgr_handle, ovl2mem_cmdq_enabled());

	dpmgr_path_trigger(pgcl->dpmgr_handle, pgcl->cmdq_handle_config, ovl2mem_cmdq_enabled());

	cmdqRecWait(pgcl->cmdq_handle_config, CMDQ_EVENT_DISP_WDMA0_SOF);
	cmdqRecWait(pgcl->cmdq_handle_config, CMDQ_EVENT_DISP_WDMA0_EOF);
	cmdqRecSetEventToken(pgcl->cmdq_handle_config, CMDQ_SYNC_DISP_EXT_STREAM_EOF);
	dpmgr_path_stop(pgcl->dpmgr_handle, ovl2mem_cmdq_enabled());

	cmdqRecFlushAsyncCallback(pgcl->cmdq_handle_config, (CmdqAsyncFlushCB)ovl2mem_callback, atomic_read(&g_trigger_ticket));
	cmdqRecReset(pgcl->cmdq_handle_config);

	pgcl->need_trigger_path = 0;
	atomic_add(1, &g_trigger_ticket);

	return 0;
}

static int ovl2mem_frame_cfg_input(struct disp_frame_cfg_t *cfg)
{
	int ret = -1;
	int i = 0;
	int config_layer_id = 0;
	struct disp_ddp_path_config *data_config;
	struct ddp_io_golden_setting_arg gset_arg;
	unsigned int ext_last_fence, ext_cur_fence, ext_sub;

	data_config = dpmgr_path_get_last_config(pgcl->dpmgr_handle);
	data_config->dst_dirty = 0;
	data_config->ovl_dirty = 0;
	data_config->rdma_dirty = 0;
	data_config->p_golden_setting_context = get_golden_setting_pgc();

	for (i = 0; i < cfg->input_layer_num; i++) {
		config_layer_id = cfg->input_cfg[i].layer_id;
		if (config_layer_id < 0 || config_layer_id > (TOTAL_OVL_LAYER_NUM - 1)) return -1;

		_convert_disp_input_to_ovl(&(data_config->ovl_config[config_layer_id]), &(cfg->input_cfg[i]));
		data_config->ovl_dirty = 1;
	}

	if (dpmgr_path_is_busy(pgcl->dpmgr_handle))
		dpmgr_wait_event_timeout(pgcl->dpmgr_handle, DISP_PATH_EVENT_FRAME_COMPLETE, HZ / 5);

	ret = dpmgr_path_config(pgcl->dpmgr_handle, data_config, pgcl->cmdq_handle_config);

	memset(&gset_arg, 0, sizeof(gset_arg));
	gset_arg.dst_mod_type = dpmgr_path_get_dst_module_type(pgcl->dpmgr_handle);
	gset_arg.is_decouple_mode = 1;

	dpmgr_path_ioctl(pgcl->dpmgr_handle, pgcl->cmdq_handle_config, DDP_OVL_GOLDEN_SETTING, &gset_arg);

	for (i = 0; i < cfg->input_layer_num; i++) {
		cmdqBackupReadSlot(pgcl->ovl2mem_cur_config_fence, i, &ext_last_fence);
		ext_cur_fence = cfg->input_cfg[i].next_buff_idx;

		if (ext_cur_fence != -1 && ext_cur_fence > ext_last_fence) {
			cmdqRecBackupUpdateSlot(pgcl->cmdq_handle_config, pgcl->ovl2mem_cur_config_fence, i, ext_cur_fence);
		}

		if (cfg->input_cfg[i].buffer_source == DISP_BUFFER_ALPHA || cfg->input_cfg[i].layer_enable == 0 || ext_cur_fence == -1)
			ext_sub = 0;
		else
			ext_sub = 1;

		cmdqRecBackupUpdateSlot(pgcl->cmdq_handle_config, pgcl->ovl2mem_subtractor_when_free, i, ext_sub);
	}

	return ret;
}

static int ovl2mem_frame_cfg_output(struct disp_frame_cfg_t *cfg)
{
	int ret = -1;
	unsigned int dst_mva = 0;
	struct disp_ddp_path_config *data_config;
	unsigned int session_id = cfg->session_id;

	if (cfg->output_cfg.pa) {
		dst_mva = (unsigned long)(cfg->output_cfg.pa);
	} else {
		dst_mva = mtkfb_query_buf_mva(session_id, disp_sync_get_output_timeline_id(), (unsigned int)(cfg->output_cfg.buff_idx));
	}

	mtkfb_update_buf_ticket(session_id, disp_sync_get_output_timeline_id(), cfg->output_cfg.buff_idx, get_ovl2mem_ticket());

	data_config = dpmgr_path_get_last_config(pgcl->dpmgr_handle);
	data_config->dst_dirty = 1;
	data_config->dst_h = cfg->output_cfg.height;
	data_config->dst_w = cfg->output_cfg.width;
	data_config->ovl_dirty = 0;
	data_config->rdma_dirty = 0;
	data_config->wdma_dirty = 1;

#if defined(MTK_FB_ION_SUPPORT)
	if (cfg->output_cfg.pa != NULL)
		data_config->wdma_config.dstAddress = (unsigned long)(cfg->output_cfg.pa);
	else
		data_config->wdma_config.dstAddress = (unsigned long)dst_mva;
#else
	data_config->wdma_config.dstAddress = (unsigned long)cfg->output_cfg.pa;
#endif

	data_config->wdma_config.srcHeight = cfg->output_cfg.height;
	data_config->wdma_config.srcWidth = cfg->output_cfg.width;
	data_config->wdma_config.clipX = cfg->output_cfg.x;
	data_config->wdma_config.clipY = cfg->output_cfg.y;
	data_config->wdma_config.clipHeight = cfg->output_cfg.height;
	data_config->wdma_config.clipWidth = cfg->output_cfg.width;
	data_config->wdma_config.outputFormat = disp_fmt_to_unified_fmt(cfg->output_cfg.fmt);
	data_config->wdma_config.dstPitch = cfg->output_cfg.pitch * UFMT_GET_Bpp(data_config->wdma_config.outputFormat);
	data_config->wdma_config.useSpecifiedAlpha = 1;
	data_config->wdma_config.alpha = 0xFF;
	data_config->wdma_config.security = cfg->output_cfg.security;
	data_config->p_golden_setting_context = get_golden_setting_pgc();

	if (dpmgr_path_is_busy(pgcl->dpmgr_handle))
		dpmgr_wait_event_timeout(pgcl->dpmgr_handle, DISP_PATH_EVENT_FRAME_DONE, HZ / 5);

	ret = dpmgr_path_config(pgcl->dpmgr_handle, data_config, pgcl->cmdq_handle_config);

	pgcl->need_trigger_path = 1;

	return ret;
}

int ovl2mem_frame_cfg(struct disp_frame_cfg_t *cfg)
{
	int ret = 0;

	_ovl2mem_path_lock(__func__);

	if (pgcl->state == 0) {
		_ovl2mem_path_unlock(__func__);
		return 0;
	}

	ovl2mem_frame_cfg_input(cfg);

	if (cfg->output_en) {
		ovl2mem_frame_cfg_output(cfg);
	}

	ovl2mem_trigger(0, NULL, 0);

	_ovl2mem_path_unlock(__func__);
	return ret;
}

int ovl2mem_get_max_layer(void)
{
	return MEMORY_SESSION_INPUT_LAYER_COUNT;
}

void ovl2mem_wait_done(void)
{
	int loop_cnt = 0;

	if ((atomic_read(&g_trigger_ticket) - atomic_read(&g_release_ticket)) <= 1) return;

	while ((atomic_read(&g_trigger_ticket) - atomic_read(&g_release_ticket)) > 1) {
		dpmgr_wait_event_timeout(pgcl->dpmgr_handle, DISP_PATH_EVENT_FRAME_COMPLETE, HZ / 30);
		if (loop_cnt > 5) break;
		loop_cnt++;
	}
}

int ovl2mem_deinit(void)
{
	int ret = -1;
	int loop_cnt = 0;
	int i = 0;

	_ovl2mem_path_lock(__func__);

	if (pgcl->state == 0) goto Exit;

	ovl2mem_layer_num = 0;
	while (((atomic_read(&g_trigger_ticket) - atomic_read(&g_release_ticket)) != 1) && (loop_cnt < 10)) {
		_ovl2mem_path_unlock(__func__);
		usleep_range(5000, 6000);
		_ovl2mem_path_lock(__func__);
		loop_cnt++;
	}

	switch_module_to_nonsec(pgcl->dpmgr_handle, NULL, __func__);

	dpmgr_path_stop(pgcl->dpmgr_handle, CMDQ_DISABLE);
	dpmgr_path_reset(pgcl->dpmgr_handle, CMDQ_DISABLE);
	dpmgr_path_deinit(pgcl->dpmgr_handle, CMDQ_DISABLE);

	dpmgr_destroy_path_handle(pgcl->dpmgr_handle);
	cmdqRecDestroy(pgcl->cmdq_handle_config);

	for (i = 0; i < MEMORY_SESSION_INPUT_LAYER_COUNT; i++)
		mtkfb_release_layer_fence(pgcl->session, i);
	mtkfb_release_layer_fence(pgcl->session, disp_sync_get_output_timeline_id());

	if (pgcl->state == 1) {
		deinit_cmdq_slots(pgcl->ovl2mem_cur_config_fence);
		deinit_cmdq_slots(pgcl->ovl2mem_subtractor_when_free);
	}

	pgcl->dpmgr_handle = NULL;
	pgcl->cmdq_handle_config = NULL;
	pgcl->state = 0;
	pgcl->need_trigger_path = 0;
	atomic_set(&g_trigger_ticket, 1);
	atomic_set(&g_release_ticket, 0);
	__pm_relax(memout_wk_lock);
	ret = 0;

Exit:
	_ovl2mem_path_unlock(__func__);
	return ret;
}
