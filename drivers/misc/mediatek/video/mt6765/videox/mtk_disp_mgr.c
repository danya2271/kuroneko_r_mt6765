// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/kdev_t.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/param.h>
#include <linux/uaccess.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/workqueue.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/compat.h>
#include <linux/ratelimit.h>
#include <linux/printk.h>

#ifdef CONFIG_MTK_M4U
#include "m4u.h"
#endif

#include "mtk_sync.h"
#include "disp_lcm.h"
#include "disp_utils.h"
#include "mtkfb_console.h"
#include "ddp_hal.h"
#include "ddp_path.h"
#include "ddp_drv.h"
#include "ddp_info.h"
#include "primary_display.h"
#include "cmdq_def.h"
#include "cmdq_record.h"
#include "cmdq_reg.h"
#include "cmdq_core.h"
#include "ddp_manager.h"
#include "disp_drv_platform.h"
#include "mtk_disp_mgr.h"
#include "disp_session.h"
#include "mtk_ovl.h"
#include "mtkfb_fence.h"
#include "extd_multi_control.h"
#include "external_display.h"
#include "extd_platform.h"
#include "layering_rule.h"
#include "compat_mtk_disp_mgr.h"
#include "disp_partial.h"
#include "frame_queue.h"
#include "disp_lowpower.h"
#include "ddp_irq.h"
#include "ddp_rsz.h"

#define DDP_OUTPUT_LAYID 4

#if defined MTK_FB_SHARE_WDMA0_SUPPORT
static int idle_flag = 1;
static int smartovl_flag;
static int has_memory_session;
#endif

static unsigned int session_config[MAX_SESSION_COUNT];
static DEFINE_MUTEX(disp_session_lock);
static DEFINE_MUTEX(disp_layer_lock);

static dev_t mtk_disp_mgr_devno;
static struct cdev *mtk_disp_mgr_cdev;
static struct class *mtk_disp_mgr_class;

static DEFINE_MUTEX(repaint_queue_lock);
static DECLARE_WAIT_QUEUE_HEAD(repaint_wq);
static LIST_HEAD(repaint_job_queue);
static LIST_HEAD(repaint_job_pool);

static int mtk_disp_mgr_open(struct inode *inode, struct file *file) { return 0; }
static ssize_t mtk_disp_mgr_read(struct file *file, char __user *data, size_t len, loff_t *ppos) { return 0; }
static int mtk_disp_mgr_release(struct inode *inode, struct file *file) { return 0; }
static int mtk_disp_mgr_flush(struct file *a_pstFile, fl_owner_t a_id) { return 0; }

int _session_inited(struct disp_session_config config) { return 0; }

int disp_mgr_has_mem_session(void)
{
#if defined(MTK_FB_SHARE_WDMA0_SUPPORT)
	return has_memory_session;
#else
	return 0;
#endif
}

int disp_create_session(struct disp_session_config *config)
{
	int ret = 0;
	int is_session_inited = 0;
	unsigned int session = MAKE_DISP_SESSION(config->type, config->device_id);
	int i, idx = -1;

	mutex_lock(&disp_session_lock);
	for (i = 0; i < MAX_SESSION_COUNT; i++) {
		if (session_config[i] == session) {
			is_session_inited = 1;
			idx = i;
			break;
		}
	}

	if (is_session_inited == 1) {
		config->session_id = session;
		goto done;
	}

	for (i = 0; i < MAX_SESSION_COUNT; i++) {
		if (session_config[i] == 0 && idx == -1) {
			idx = i;
			break;
		}
	}

	if (idx != -1) {
		config->session_id = session;
		session_config[idx] = session;
#if defined MTK_FB_SHARE_WDMA0_SUPPORT
		if (session == MAKE_DISP_SESSION(DISP_SESSION_MEMORY, DEV_WFD)) {
			if (idle_flag) idle_flag = set_idlemgr(0, 1);
			smartovl_flag = disp_helper_get_option(DISP_OPT_SMART_OVL);
			disp_helper_set_option(DISP_OPT_SMART_OVL, 0);
			has_memory_session = 1;
		}
#endif
	} else {
		ret = -1;
	}
done:
	mutex_unlock(&disp_session_lock);
	return ret;
}

static int release_session_buffer(unsigned int session)
{
	mutex_lock(&disp_session_lock);
	if (session == 0) {
		mutex_unlock(&disp_session_lock);
		return -1;
	}
	mtkfb_release_session_fence(session);
	mutex_unlock(&disp_session_lock);
	return 0;
}

int disp_destroy_session(struct disp_session_config *config)
{
	int ret = -1;
	unsigned int session = config->session_id;
	int i;

	mutex_lock(&disp_session_lock);
	for (i = 0; i < MAX_SESSION_COUNT; i++) {
		if (session_config[i] != session) continue;
		session_config[i] = 0;
		ret = 0;
		break;
	}
	mutex_unlock(&disp_session_lock);

	if (DISP_SESSION_TYPE(config->session_id) != DISP_SESSION_PRIMARY)
		external_display_switch_mode(config->mode, session_config, config->session_id);

	if (DISP_SESSION_TYPE(config->session_id) != DISP_SESSION_PRIMARY)
		release_session_buffer(config->session_id);

	if (ret == 0) {
#if defined MTK_FB_SHARE_WDMA0_SUPPORT
		if (session == MAKE_DISP_SESSION(DISP_SESSION_MEMORY, DEV_WFD)) {
			if (idle_flag) set_idlemgr(idle_flag, 1);
			if (smartovl_flag) disp_helper_set_option(DISP_OPT_SMART_OVL, smartovl_flag);
			has_memory_session = 0;
		}
#endif
	}

	return ret;
}

int _ioctl_create_session(unsigned long arg)
{
	int ret = 0;
	void __user *argp = (void __user *)arg;
	struct disp_session_config config;

	if (copy_from_user(&config, argp, sizeof(config))) return -EFAULT;
	if (disp_create_session(&config) != 0) ret = -EFAULT;
	if (copy_to_user(argp, &config, sizeof(config))) ret = -EFAULT;

	return ret;
}

int _ioctl_destroy_session(unsigned long arg)
{
	int ret = 0;
	void __user *argp = (void __user *)arg;
	struct disp_session_config config;

	if (copy_from_user(&config, argp, sizeof(config))) return -EFAULT;
	if (disp_destroy_session(&config) != 0) ret = -EFAULT;

	return ret;
}

int _ioctl_prepare_present_fence(unsigned long arg)
{
	int ret = 0;
	void __user *argp = (void __user *)arg;
	struct mtk_sync_create_fence_data data;
	struct disp_present_fence pnt_fence;
	static unsigned int fence_idx;
	struct disp_sync_info *layer_info = NULL;
	int timeline_id = disp_sync_get_present_timeline_id();

	if (copy_from_user(&pnt_fence, argp, sizeof(struct disp_present_fence))) return -EFAULT;

	if (DISP_SESSION_TYPE(pnt_fence.session_id) != DISP_SESSION_PRIMARY) {
		data.fence = MTK_FB_INVALID_FENCE_FD;
		data.value = 0;
	} else {
		layer_info = _get_sync_info(pnt_fence.session_id, timeline_id);
		if (layer_info == NULL) return -EFAULT;

		data.fence = MTK_FB_INVALID_FENCE_FD;
		data.value = ++fence_idx;
		ret = fence_create(layer_info->timeline, &data);
		if (ret != 0) ret = -EFAULT;
	}

	pnt_fence.present_fence_fd = data.fence;
	pnt_fence.present_fence_index = data.value;
	if (copy_to_user(argp, &pnt_fence, sizeof(pnt_fence))) ret = -EFAULT;

	return ret;
}

static void _prepare_output_buffer(struct disp_buffer_info *info, struct mtkfb_fence_buf_info *output_buf)
{
	if (primary_display_is_decouple_mode() && primary_display_is_mirror_mode()) {
		info->layer_id = disp_sync_get_output_interface_timeline_id();
		output_buf = disp_sync_prepare_buf(info);
		if (output_buf != NULL) {
			info->interface_fence_fd = output_buf->fence;
			info->interface_index = output_buf->idx;
		} else {
			info->interface_fence_fd = MTK_FB_INVALID_FENCE_FD;
			info->interface_index = 0;
		}
	} else {
		info->interface_fence_fd = MTK_FB_INVALID_FENCE_FD;
		info->interface_index = 0;
	}
}

int _ioctl_prepare_buffer(unsigned long arg, enum PREPARE_FENCE_TYPE type)
{
	int ret = 0;
	void __user *argp = (void __user *)arg;
	struct disp_buffer_info info;
	struct mtkfb_fence_buf_info *buf, *buf2 = NULL;

	if (copy_from_user(&info, argp, sizeof(info))) return -EFAULT;

	if (type == PREPARE_PRESENT_FENCE) info.layer_id = disp_sync_get_present_timeline_id();
	else if (type == PREPARE_OUTPUT_FENCE) info.layer_id = disp_sync_get_output_timeline_id();

	if (info.layer_en) {
		buf = disp_sync_prepare_buf(&info);
		if (buf != NULL) {
			info.fence_fd = buf->fence;
			info.index = buf->idx;
		} else {
			info.fence_fd = MTK_FB_INVALID_FENCE_FD;
			info.index = 0;
		}

		if (type == PREPARE_OUTPUT_FENCE) _prepare_output_buffer(&info, buf2);
	} else {
		info.fence_fd = MTK_FB_INVALID_FENCE_FD;
		info.index = 0;
	}
	if (copy_to_user(argp, &info, sizeof(info))) ret = -EFAULT;
	return ret;
}

static int _get_max_layer(unsigned int session_id)
{
	if (DISP_SESSION_TYPE(session_id) == DISP_SESSION_PRIMARY)
		return primary_display_get_max_layer();
#if ((defined CONFIG_MTK_HDMI_SUPPORT) || (defined(CONFIG_MTK_DUAL_DISPLAY_SUPPORT) && (CONFIG_MTK_DUAL_DISPLAY_SUPPORT == 2)))
	else if (DISP_SESSION_TYPE(session_id) == DISP_SESSION_EXTERNAL)
		return ext_disp_get_max_layer();
#endif
	else if (DISP_SESSION_TYPE(session_id) == DISP_SESSION_MEMORY)
		return ovl2mem_get_max_layer();

	return 0;
}

static int disp_validate_input_params(struct disp_input_config *cfg, int layer_num)
{
	if (cfg->layer_id >= layer_num) return -1;
	if (cfg->layer_enable) {
		if ((cfg->src_fmt <= 0) || ((cfg->src_fmt >> 8) == 15) || ((cfg->src_fmt >> 8) > (DISP_FORMAT_DIM >> 8))) {
			return -1;
		}
	}
	return 0;
}

static int disp_validate_output_params(struct disp_output_config *cfg)
{
	if ((cfg->fmt <= 0) || ((cfg->fmt >> 8) == 15) || ((cfg->fmt >> 8) > (DISP_FORMAT_DIM >> 8))) return -1;
	return 0;
}

int disp_validate_ioctl_params(struct disp_frame_cfg_t *cfg)
{
	int i;

	if (cfg->input_layer_num > _get_max_layer(cfg->session_id)) return -1;

	for (i = 0; i < cfg->input_layer_num; i++)
		if (disp_validate_input_params(&cfg->input_cfg[i], _get_max_layer(cfg->session_id)) != 0) return -1;

	if (cfg->output_en && disp_validate_output_params(&cfg->output_cfg) != 0) return -1;

	return 0;
}

static int disp_input_get_dirty_roi(struct disp_frame_cfg_t *cfg)
{
	int i;

	for (i = 0; i < cfg->input_layer_num; i++) {
		void *addr;
		unsigned long size;

		if (!cfg->input_cfg[i].layer_enable || !cfg->input_cfg[i].dirty_roi_num) goto layer_err;
		if (WARN_ON(cfg->input_cfg[i].dirty_roi_num > 20)) goto layer_err;

		size = cfg->input_cfg[i].dirty_roi_num * sizeof(struct layer_dirty_roi);
		addr = kmalloc(size, GFP_KERNEL);
		if (IS_ERR_OR_NULL(addr)) goto layer_err;

		if (copy_from_user(addr, cfg->input_cfg[i].dirty_roi_addr, size)) {
			kfree(addr);
			goto layer_err;
		} else {
			cfg->input_cfg[i].dirty_roi_addr = addr;
		}
		continue;
layer_err:
		cfg->input_cfg[i].dirty_roi_num = 0;
		cfg->input_cfg[i].dirty_roi_addr = NULL;
	}
	return 0;
}

int disp_input_free_dirty_roi(struct disp_frame_cfg_t *cfg)
{
	int i;

	for (i = 0; i < cfg->input_layer_num; i++) {
		if (i >= _get_max_layer(cfg->session_id)) break;
		if (!cfg->input_cfg[i].layer_enable || !cfg->input_cfg[i].dirty_roi_num) continue;
		if (cfg->input_cfg[i].dirty_roi_addr != NULL) {
			kfree(cfg->input_cfg[i].dirty_roi_addr);
			cfg->input_cfg[i].dirty_roi_addr = NULL;
		}
	}
	return 0;
}

static int input_config_preprocess(struct disp_frame_cfg_t *cfg)
{
	int i = 0;
	int layer_id = 0;
	unsigned int dst_size = 0;
	unsigned long dst_mva = 0;
	unsigned int session_id = cfg->session_id;
	unsigned int mva_offset = 0;
	enum DISP_FORMAT src_fmt;

	if (cfg->input_layer_num == 0 || cfg->input_layer_num > _get_max_layer(session_id)) return -1;

	disp_input_get_dirty_roi(cfg);
	for (i = 0; i < cfg->input_layer_num; i++) {
		dst_mva = 0;
		layer_id = cfg->input_cfg[i].layer_id;
		if (layer_id >= _get_max_layer(session_id)) continue;

		if (cfg->input_cfg[i].layer_enable) {
			unsigned int Bpp, x, y, pitch;
#ifdef DISP_SYNC_ENABLE
			struct sync_fence *src_fence = NULL;
#endif
			if (cfg->input_cfg[i].buffer_source == DISP_BUFFER_ALPHA) {
				cfg->input_cfg[i].src_offset_x = 0;
				cfg->input_cfg[i].src_offset_y = 0;
				cfg->input_cfg[i].sur_aen = 1;
				cfg->input_cfg[i].src_fmt = DISP_FORMAT_RGB888;
				cfg->input_cfg[i].src_pitch = cfg->input_cfg[i].src_width;
				cfg->input_cfg[i].src_phy_addr = (void *)get_dim_layer_mva_addr();
				cfg->input_cfg[i].next_buff_idx = 0;
				cfg->input_cfg[i].src_fence_struct = NULL;
				cfg->input_cfg[i].security = DISP_NORMAL_BUFFER;
			}

			dst_mva = (unsigned long)(cfg->input_cfg[i].src_phy_addr);
			if (!dst_mva) {
				disp_sync_query_buf_info(session_id, layer_id, (unsigned int)cfg->input_cfg[i].next_buff_idx, &dst_mva, &dst_size);
			}

			cfg->input_cfg[i].src_phy_addr = (void *)dst_mva;

			if (dst_mva == 0) cfg->input_cfg[i].layer_enable = 0;

#ifdef DISP_SYNC_ENABLE
			src_fence = sync_fence_fdget(cfg->input_cfg[i].src_fence_fd);
			cfg->input_cfg[i].src_fence_struct = src_fence;
#endif
			x = cfg->input_cfg[i].src_offset_x;
			y = cfg->input_cfg[i].src_offset_y;
			pitch = cfg->input_cfg[i].src_pitch;
			src_fmt = cfg->input_cfg[i].src_fmt;
			Bpp = UFMT_GET_bpp(disp_fmt_to_unified_fmt(src_fmt)) / 8;

			mva_offset = (x + y * pitch) * Bpp;
			mtkfb_update_buf_info(cfg->session_id, cfg->input_cfg[i].layer_id, cfg->input_cfg[i].next_buff_idx, mva_offset, cfg->input_cfg[i].frm_sequence);
		} else {
			cfg->input_cfg[i].src_fence_struct = NULL;
		}

		disp_sync_put_cached_layer_info(session_id, layer_id, &cfg->input_cfg[i], dst_mva);
	}
	return 0;
}

static int output_config_preprocess(struct disp_frame_cfg_t *cfg)
{
	unsigned int session_id = cfg->session_id;
	unsigned long dst_mva = 0;
	unsigned int dst_size;
#ifdef DISP_SYNC_ENABLE
	struct sync_fence *src_fence;
#endif

	if (cfg->output_cfg.pa) {
		dst_mva = (unsigned long)(cfg->output_cfg.pa);
	} else {
		disp_sync_query_buf_info_nosync(session_id, disp_sync_get_output_timeline_id(), cfg->output_cfg.buff_idx, &dst_mva, &dst_size);
	}
	cfg->output_cfg.pa = (void *)dst_mva;
	if (!dst_mva) {
		cfg->output_en = 0;
		goto out;
	}

#ifdef DISP_SYNC_ENABLE
	src_fence = sync_fence_fdget(cfg->output_cfg.src_fence_fd);
	cfg->output_cfg.src_fence_struct = src_fence;
#endif
	if (DISP_SESSION_TYPE(session_id) == DISP_SESSION_PRIMARY) {
		if (primary_display_is_decouple_mode()) {
			disp_sync_put_cached_layer_info_v2(session_id, disp_sync_get_output_interface_timeline_id(), cfg->output_cfg.interface_idx, 1, dst_mva);
		}
	}

	mtkfb_update_buf_info(cfg->session_id, disp_sync_get_output_interface_timeline_id(), cfg->output_cfg.buff_idx, 0, cfg->output_cfg.frm_sequence);

out:
	return 0;
}

static int do_frame_config(struct frame_queue_t *frame_node)
{
	struct disp_frame_cfg_t *frame_cfg = &frame_node->frame_cfg;

	if (DISP_SESSION_TYPE(frame_cfg->session_id) == DISP_SESSION_PRIMARY)
		primary_display_frame_cfg(frame_cfg);
#if ((defined CONFIG_MTK_HDMI_SUPPORT) || (defined(CONFIG_MTK_DUAL_DISPLAY_SUPPORT) && (CONFIG_MTK_DUAL_DISPLAY_SUPPORT == 2)))
	else if (DISP_SESSION_TYPE(frame_cfg->session_id) == DISP_SESSION_EXTERNAL)
		external_display_frame_cfg(frame_cfg);
#endif
	else if (DISP_SESSION_TYPE(frame_cfg->session_id) == DISP_SESSION_MEMORY)
		ovl2mem_frame_cfg(frame_cfg);

	return 0;
}

static long _frame_queue_config(unsigned long arg)
{
	void *ret_val = NULL;
	struct frame_queue_head_t *head;
	struct disp_frame_cfg_t *frame_cfg;
	struct sync_fence *present_fence = NULL;
	struct frame_queue_t *frame_node;

	frame_node = frame_queue_node_create();
	if (IS_ERR_OR_NULL(frame_node)) {
		ret_val = ERR_PTR(-ENOMEM);
		goto Error;
	}

	frame_cfg = &frame_node->frame_cfg;

	if (copy_from_user(frame_cfg, (void __user *)arg, sizeof(*frame_cfg))) {
		ret_val = ERR_PTR(-EFAULT);
		goto Error;
	}

	if (disp_validate_ioctl_params(frame_cfg)) {
		ret_val = ERR_PTR(-EINVAL);
		goto Error;
	}

	head = get_frame_queue_head(frame_cfg->session_id);
	if (!head) return -EINVAL;

	frame_cfg->setter = SESSION_USER_HWC;

#ifdef DISP_SYNC_ENABLE
	if (frame_cfg->prev_present_fence_fd != -1) {
		present_fence = sync_fence_fdget(frame_cfg->prev_present_fence_fd);
	}
#endif
	frame_cfg->prev_present_fence_struct = present_fence;
	frame_node->do_frame_cfg = do_frame_config;

	input_config_preprocess(frame_cfg);
	if (frame_cfg->output_en) output_config_preprocess(frame_cfg);

	frame_queue_push(head, frame_node);

	return 0;

Error:
	if (frame_node) frame_queue_node_destroy(frame_node);
	return PTR_ERR(ret_val);
}

long _frame_config(unsigned long arg)
{
	struct disp_frame_cfg_t *frame_cfg = kzalloc(sizeof(struct disp_frame_cfg_t), GFP_KERNEL);

	if (frame_cfg == NULL) return -EFAULT;
	if (copy_from_user(frame_cfg, (void __user *)arg, sizeof(*frame_cfg))) {
		kfree(frame_cfg);
		return -EFAULT;
	}
	if (disp_validate_ioctl_params(frame_cfg)) {
		kfree(frame_cfg);
		return -EINVAL;
	}

	frame_cfg->setter = SESSION_USER_HWC;

	if (input_config_preprocess(frame_cfg) != 0) {
		kfree(frame_cfg);
		return -EINVAL;
	}
	if (frame_cfg->output_en) output_config_preprocess(frame_cfg);

	if (DISP_SESSION_TYPE(frame_cfg->session_id) == DISP_SESSION_PRIMARY)
		primary_display_frame_cfg(frame_cfg);
#if ((defined CONFIG_MTK_HDMI_SUPPORT) || (defined(CONFIG_MTK_DUAL_DISPLAY_SUPPORT) && (CONFIG_MTK_DUAL_DISPLAY_SUPPORT == 2)))
	else if (DISP_SESSION_TYPE(frame_cfg->session_id) == DISP_SESSION_EXTERNAL)
		external_display_frame_cfg(frame_cfg);
#endif
	else if (DISP_SESSION_TYPE(frame_cfg->session_id) == DISP_SESSION_MEMORY)
		ovl2mem_frame_cfg(frame_cfg);

	disp_input_free_dirty_roi(frame_cfg);
	kfree(frame_cfg);

	return 0;
}

static long _ioctl_frame_config(unsigned long arg)
{
	if (disp_helper_get_option(DISP_OPT_FRAME_QUEUE))
		return _frame_queue_config(arg);
	else
		return _frame_config(arg);
}

static int _ioctl_wait_all_jobs_done(unsigned long arg)
{
	unsigned int session_id = (unsigned int)arg;
	struct frame_queue_head_t *head;

	head = get_frame_queue_head(session_id);
	if (!head) return -EINVAL;

	return frame_queue_wait_all_jobs_done(head);
}

int disp_mgr_get_session_info(struct disp_session_info *info)
{
	unsigned int session_id = info->session_id;

	if (DISP_SESSION_TYPE(session_id) == DISP_SESSION_PRIMARY) {
		primary_display_get_info(info);
#if ((defined CONFIG_MTK_HDMI_SUPPORT) || (defined(CONFIG_MTK_DUAL_DISPLAY_SUPPORT) && (CONFIG_MTK_DUAL_DISPLAY_SUPPORT == 2)))
	} else if (DISP_SESSION_TYPE(session_id) == DISP_SESSION_EXTERNAL) {
		external_display_get_info(info, session_id);
#endif
	} else if (DISP_SESSION_TYPE(session_id) == DISP_SESSION_MEMORY) {
		ovl2mem_get_info(info);
	} else {
		return -1;
	}
	return 0;
}

int _ioctl_get_info(unsigned long arg)
{
	int ret = 0;
	void __user *argp = (void __user *)arg;
	struct disp_session_info info;

	if (copy_from_user(&info, argp, sizeof(info))) return -EFAULT;
	ret = disp_mgr_get_session_info(&info);
	if (copy_to_user(argp, &info, sizeof(info))) ret = -EFAULT;

	return ret;
}

int _ioctl_get_display_caps(unsigned long arg)
{
	int ret = 0;
	struct disp_caps_info caps_info = {0};
	void __user *argp = (void __user *)arg;

#ifdef DISP_HW_MODE_CAP
	caps_info.output_mode = DISP_HW_MODE_CAP;
#else
	caps_info.output_mode = DISP_OUTPUT_CAP_DIRECT_LINK;
#endif

#ifdef DISP_HW_PASS_MODE
	caps_info.output_pass = DISP_HW_PASS_MODE;
#else
	caps_info.output_pass = DISP_OUTPUT_CAP_SINGLE_PASS;
#endif

#ifdef DISP_HW_MAX_LAYER
	caps_info.max_layer_num = DISP_HW_MAX_LAYER;
#else
	caps_info.max_layer_num = 4;
#endif
	caps_info.is_support_frame_cfg_ioctl = 1;

#ifdef CONFIG_MTK_LCM_PHYSICAL_ROTATION_HW
	caps_info.is_output_rotated = 1;
	caps_info.lcm_degree = 180;
#endif

	if (disp_partial_is_support()) caps_info.disp_feature |= DISP_FEATURE_PARTIAL;
	if (disp_helper_get_option(DISP_OPT_HRT)) caps_info.disp_feature |= DISP_FEATURE_HRT;
	if (disp_helper_get_option(DISP_OPT_FRAME_QUEUE)) caps_info.disp_feature |= DISP_FEATURE_FENCE_WAIT;
	if (disp_helper_get_option(DISP_OPT_RSZ)) caps_info.disp_feature |= DISP_FEATURE_RSZ;
	if (disp_helper_get_option(DISP_OPT_RPO)) caps_info.disp_feature |= DISP_FEATURE_RPO;

	if (disp_helper_get_option(DISP_OPT_RSZ) || disp_helper_get_option(DISP_OPT_RPO)) {
		caps_info.rsz_in_max[0] = RSZ_TILE_LENGTH - RSZ_ALIGNMENT_MARGIN;
		caps_info.rsz_in_max[1] = RSZ_IN_MAX_HEIGHT;
	}
#ifdef CONFIG_MTK_HIGH_FRAME_RATE
		if (primary_display_is_support_DynFPS()) {
			caps_info.disp_feature |= DISP_FEATURE_DYNFPS;
		}
#endif

	if (copy_to_user(argp, &caps_info, sizeof(caps_info))) ret = -EFAULT;
	return ret;
}

int _ioctl_wait_vsync(unsigned long arg)
{
	int ret = 0;
	void __user *argp = (void __user *)arg;
	struct disp_session_vsync_config vsync_config;

	if (copy_from_user(&vsync_config, argp, sizeof(vsync_config))) return -EFAULT;

	if (DISP_SESSION_TYPE(vsync_config.session_id) == DISP_SESSION_EXTERNAL)
		ret = external_display_wait_for_vsync(&vsync_config, vsync_config.session_id);
	else
		ret = primary_display_wait_for_vsync(&vsync_config);

	if (copy_to_user(argp, &vsync_config, sizeof(vsync_config))) return -EFAULT;
	return ret;
}

int _ioctl_get_vsync(unsigned long arg)
{
	unsigned int fps = primary_display_force_get_vsync_fps();
	return copy_to_user((void __user *)arg, &fps, sizeof(int)) ? -EFAULT : 0;
}

int _ioctl_set_vsync(unsigned long arg)
{
	unsigned int fps = (unsigned int)arg;
	if (fps < 50 || fps > 100) return -EFAULT;
	return primary_display_force_set_vsync_fps(fps, 1);
}

static long _ioctl_query_valid_layer(unsigned long arg)
{
	struct disp_layer_info disp_info_user;
	void __user *argp = (void __user *)arg;

	if (copy_from_user(&disp_info_user, argp, sizeof(disp_info_user))) return -EFAULT;

	if (disp_info_user.layer_num[0] < 0 || disp_info_user.layer_num[0] > 0x300 ||
		disp_info_user.layer_num[1] < 0 || disp_info_user.layer_num[1] > 0x300) {
		return -EINVAL;
	}

	mutex_lock(&disp_layer_lock);
	layering_rule_start(&disp_info_user, 0);
	mutex_unlock(&disp_layer_lock);

	if (copy_to_user(argp, &disp_info_user, sizeof(disp_info_user))) return -EFAULT;

	return 0;
}

int _ioctl_set_scenario(unsigned long arg)
{
	int ret = -1;
	struct disp_scenario_config_t scenario_cfg;

	if (copy_from_user(&scenario_cfg, (void __user *)arg, sizeof(scenario_cfg))) return -EFAULT;

	if (DISP_SESSION_TYPE(scenario_cfg.session_id) == DISP_SESSION_PRIMARY)
		ret = primary_display_set_scenario(scenario_cfg.scenario);

	return ret;
}

int set_session_mode(struct disp_session_config *config_info, int force)
{
	int ret = 0;

#if defined(MTK_FB_SHARE_WDMA0_SUPPORT)
	if (config_info->mode == DISP_SESSION_DIRECT_LINK_MIRROR_MODE ||
	    config_info->mode == DISP_SESSION_DECOUPLE_MIRROR_MODE) {
		external_display_switch_mode(config_info->mode, session_config, config_info->session_id);

		if (has_memory_session)
			primary_display_switch_mode_blocked(config_info->mode, config_info->session_id, 0);
		else if (DISP_SESSION_TYPE(config_info->session_id) == DISP_SESSION_PRIMARY)
			primary_display_switch_mode(config_info->mode, config_info->session_id, 0);
	} else {
		if (has_memory_session)
			primary_display_switch_mode_blocked(config_info->mode, config_info->session_id, 0);
		else if (DISP_SESSION_TYPE(config_info->session_id) == DISP_SESSION_PRIMARY)
			primary_display_switch_mode(config_info->mode, config_info->session_id, 0);
		external_display_switch_mode(config_info->mode, session_config, config_info->session_id);
	}
#else
	if (DISP_SESSION_TYPE(config_info->session_id) == DISP_SESSION_PRIMARY)
		primary_display_switch_mode(config_info->mode, config_info->session_id, 0);
	external_display_switch_mode(config_info->mode, session_config, config_info->session_id);
#endif
	return ret;
}

void trigger_repaint(int type)
{
	if (type > WAIT_FOR_REFRESH && type < REFRESH_TYPE_NUM) {
		struct repaint_job_t *repaint_job;

		mutex_lock(&repaint_queue_lock);
		if (!list_empty(&repaint_job_pool)) {
			repaint_job = list_first_entry(&repaint_job_pool, struct repaint_job_t, link);
			list_del_init(&repaint_job->link);
		} else {
			repaint_job = kzalloc(sizeof(struct repaint_job_t), GFP_KERNEL);
			if (IS_ERR_OR_NULL(repaint_job)) return;
			INIT_LIST_HEAD(&repaint_job->link);
		}

		repaint_job->type = type;
		list_add_tail(&repaint_job->link, &repaint_job_queue);
		mutex_unlock(&repaint_queue_lock);

		wake_up_interruptible(&repaint_wq);
	}
}

int _ioctl_set_session_mode(unsigned long arg)
{
	int ret = -1;
	struct disp_session_config config_info;

	if (copy_from_user(&config_info, (void __user *)arg, sizeof(struct disp_session_config))) return -EFAULT;

	if (config_info.mode > DISP_INVALID_SESSION_MODE && config_info.mode < DISP_SESSION_MODE_NUM) {
		ret = set_session_mode(&config_info, 0);
	}
	return ret;
}

#ifdef CONFIG_MTK_HIGH_FRAME_RATE
int _ioctl_get_multi_configs(unsigned long arg)
{
	int ret = 0;
	void __user *argp = (void __user *)arg;
	struct multi_configs multi_cfgs;

	if (copy_from_user(&multi_cfgs, argp, sizeof(multi_cfgs))) return -EFAULT;
	ret = primary_display_get_multi_configs(&multi_cfgs);
	if (ret != 0) return -EFAULT;
	if (copy_to_user(argp, &multi_cfgs, sizeof(multi_cfgs))) ret = -EFAULT;

	return ret;
}
#endif

long mtk_disp_mgr_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = -1;

	switch (cmd) {
	case DISP_IOCTL_CREATE_SESSION: return _ioctl_create_session(arg);
	case DISP_IOCTL_DESTROY_SESSION: return _ioctl_destroy_session(arg);
	case DISP_IOCTL_GET_PRESENT_FENCE: return _ioctl_prepare_present_fence(arg);
	case DISP_IOCTL_PREPARE_INPUT_BUFFER: return _ioctl_prepare_buffer(arg, PREPARE_INPUT_FENCE);
	case DISP_IOCTL_WAIT_FOR_VSYNC: return _ioctl_wait_vsync(arg);
	case DISP_IOCTL_GET_SESSION_INFO: return _ioctl_get_info(arg);
	case DISP_IOCTL_GET_DISPLAY_CAPS: return _ioctl_get_display_caps(arg);
	case DISP_IOCTL_GET_VSYNC_FPS: return _ioctl_get_vsync(arg);
	case DISP_IOCTL_SET_VSYNC_FPS: return _ioctl_set_vsync(arg);
	case DISP_IOCTL_SET_SESSION_MODE: return _ioctl_set_session_mode(arg);
	case DISP_IOCTL_PREPARE_OUTPUT_BUFFER: return _ioctl_prepare_buffer(arg, PREPARE_OUTPUT_FENCE);
	case DISP_IOCTL_FRAME_CONFIG: return _ioctl_frame_config(arg);
	case DISP_IOCTL_WAIT_ALL_JOBS_DONE: return _ioctl_wait_all_jobs_done(arg);
	case DISP_IOCTL_GET_LCMINDEX: return primary_display_get_lcm_index();
	case DISP_IOCTL_QUERY_VALID_LAYER: return _ioctl_query_valid_layer(arg);
	case DISP_IOCTL_SET_SCENARIO: return _ioctl_set_scenario(arg);
#ifdef CONFIG_MTK_HIGH_FRAME_RATE
	case DISP_IOCTL_GET_MULTI_CONFIGS: return _ioctl_get_multi_configs(arg);
#endif
#ifndef NO_PQ_IOCTL
	case DISP_IOCTL_AAL_EVENTCTL: case DISP_IOCTL_AAL_GET_HIST: case DISP_IOCTL_AAL_INIT_REG:
	case DISP_IOCTL_AAL_SET_PARAM: case DISP_IOCTL_AAL_GET_SIZE: case DISP_IOCTL_SET_SMARTBACKLIGHT:
	case DISP_IOCTL_SET_GAMMALUT: case DISP_IOCTL_SET_CCORR: case DISP_IOCTL_CCORR_EVENTCTL:
	case DISP_IOCTL_CCORR_GET_IRQ: case DISP_IOCTL_SUPPORT_COLOR_TRANSFORM: case DISP_IOCTL_SET_PQPARAM:
	case DISP_IOCTL_GET_PQPARAM: case DISP_IOCTL_SET_PQINDEX: case DISP_IOCTL_GET_PQINDEX:
	case DISP_IOCTL_SET_COLOR_REG: case DISP_IOCTL_SET_TDSHPINDEX: case DISP_IOCTL_GET_TDSHPINDEX:
	case DISP_IOCTL_SET_PQ_CAM_PARAM: case DISP_IOCTL_GET_PQ_CAM_PARAM: case DISP_IOCTL_SET_PQ_GAL_PARAM:
	case DISP_IOCTL_GET_PQ_GAL_PARAM: case DISP_IOCTL_PQ_SET_BYPASS_COLOR: case DISP_IOCTL_PQ_SET_WINDOW:
	case DISP_IOCTL_OD_CTL: case DISP_IOCTL_WRITE_REG: case DISP_IOCTL_READ_REG:
	case DISP_IOCTL_MUTEX_CONTROL: case DISP_IOCTL_PQ_GET_TDSHP_FLAG: case DISP_IOCTL_PQ_SET_TDSHP_FLAG:
	case DISP_IOCTL_PQ_GET_DC_PARAM: case DISP_IOCTL_PQ_SET_DC_PARAM: case DISP_IOCTL_PQ_GET_DS_PARAM:
	case DISP_IOCTL_PQ_GET_MDP_COLOR_CAP: case DISP_IOCTL_PQ_GET_MDP_TDSHP_REG:
	case DISP_IOCTL_WRITE_SW_REG: case DISP_IOCTL_READ_SW_REG:
		ret = primary_display_user_cmd(cmd, arg);
		break;
#endif
	default: break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long mtk_disp_mgr_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = -ENOIOCTLCMD;

	switch (cmd) {
	case COMPAT_DISP_IOCTL_CREATE_SESSION: return _compat_ioctl_create_session(file, arg);
	case COMPAT_DISP_IOCTL_DESTROY_SESSION: return _compat_ioctl_destroy_session(file, arg);
	case COMPAT_DISP_IOCTL_TRIGGER_SESSION: return _compat_ioctl_trigger_session(file, arg);
	case COMPAT_DISP_IOCTL_GET_PRESENT_FENCE: return _compat_ioctl_prepare_present_fence(file, arg);
	case COMPAT_DISP_IOCTL_PREPARE_INPUT_BUFFER: return _compat_ioctl_prepare_buffer(file, arg, PREPARE_INPUT_FENCE);
	case COMPAT_DISP_IOCTL_SET_INPUT_BUFFER: return _compat_ioctl_set_input_buffer(file, arg);
	case COMPAT_DISP_IOCTL_FRAME_CONFIG: return _compat_ioctl_frame_config(file, arg);
	case COMPAT_DISP_IOCTL_WAIT_FOR_VSYNC: return _compat_ioctl_wait_vsync(file, arg);
	case COMPAT_DISP_IOCTL_GET_SESSION_INFO: return _compat_ioctl_get_info(file, arg);
	case COMPAT_DISP_IOCTL_GET_DISPLAY_CAPS: return _compat_ioctl_get_display_caps(file, arg);
	case COMPAT_DISP_IOCTL_GET_VSYNC_FPS: return _compat_ioctl_get_vsync(file, arg);
	case COMPAT_DISP_IOCTL_SET_VSYNC_FPS: return _compat_ioctl_set_vsync(file, arg);
	case COMPAT_DISP_IOCTL_SET_SESSION_MODE: return _compat_ioctl_set_session_mode(file, arg);
	case COMPAT_DISP_IOCTL_PREPARE_OUTPUT_BUFFER: return _compat_ioctl_prepare_buffer(file, arg, PREPARE_OUTPUT_FENCE);
	case COMPAT_DISP_IOCTL_SET_OUTPUT_BUFFER: return _compat_ioctl_set_output_buffer(file, arg);
	case COMPAT_DISP_IOCTL_INSERT_SESSION_BUFFERS: return _compat_ioctl_inset_session_buffer(file, arg);
	case COMPAT_DISP_IOCTL_QUERY_VALID_LAYER: return _compat_ioctl_query_valid_layer(file, arg);
	case COMPAT_DISP_IOCTL_SET_SCENARIO: return _compat_ioctl_set_scenario(file, arg);
	case COMPAT_DISP_IOCTL_WAIT_ALL_JOBS_DONE: return _compat_ioctl_wait_all_jobs_done(file, arg);
#ifndef NO_PQ_IOCTL
	case DISP_IOCTL_AAL_GET_HIST: case DISP_IOCTL_AAL_EVENTCTL: case DISP_IOCTL_AAL_INIT_REG:
	case DISP_IOCTL_AAL_SET_PARAM: case DISP_IOCTL_AAL_GET_SIZE: case DISP_IOCTL_SET_SMARTBACKLIGHT:
		return file->f_op->unlocked_ioctl(file, cmd, (unsigned long)compat_ptr(arg));

	case DISP_IOCTL_SET_GAMMALUT: case DISP_IOCTL_SET_CCORR: case DISP_IOCTL_CCORR_EVENTCTL:
	case DISP_IOCTL_CCORR_GET_IRQ: case DISP_IOCTL_SUPPORT_COLOR_TRANSFORM: case DISP_IOCTL_SET_PQPARAM:
	case DISP_IOCTL_GET_PQPARAM: case DISP_IOCTL_SET_PQINDEX: case DISP_IOCTL_GET_PQINDEX:
	case DISP_IOCTL_SET_COLOR_REG: case DISP_IOCTL_SET_TDSHPINDEX: case DISP_IOCTL_GET_TDSHPINDEX:
	case DISP_IOCTL_SET_PQ_CAM_PARAM: case DISP_IOCTL_GET_PQ_CAM_PARAM: case DISP_IOCTL_SET_PQ_GAL_PARAM:
	case DISP_IOCTL_GET_PQ_GAL_PARAM: case DISP_IOCTL_PQ_SET_BYPASS_COLOR: case DISP_IOCTL_PQ_SET_WINDOW:
	case DISP_IOCTL_OD_CTL: case DISP_IOCTL_WRITE_REG: case DISP_IOCTL_READ_REG:
	case DISP_IOCTL_MUTEX_CONTROL: case DISP_IOCTL_PQ_GET_TDSHP_FLAG: case DISP_IOCTL_PQ_SET_TDSHP_FLAG:
	case DISP_IOCTL_PQ_GET_DC_PARAM: case DISP_IOCTL_PQ_GET_DS_PARAM: case DISP_IOCTL_PQ_SET_DC_PARAM:
	case DISP_IOCTL_PQ_GET_MDP_COLOR_CAP: case DISP_IOCTL_PQ_GET_MDP_TDSHP_REG:
	case DISP_IOCTL_WRITE_SW_REG: case DISP_IOCTL_READ_SW_REG:
		return primary_display_user_cmd(cmd, arg);
#endif
	default:
		return file->f_op->unlocked_ioctl(file, cmd, (unsigned long)compat_ptr(arg));
	}
	return ret;
}
#endif

static const struct file_operations mtk_disp_mgr_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = mtk_disp_mgr_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = mtk_disp_mgr_compat_ioctl,
#endif
	.open = mtk_disp_mgr_open,
	.release = mtk_disp_mgr_release,
	.flush = mtk_disp_mgr_flush,
	.read = mtk_disp_mgr_read,
};

static int mtk_disp_mgr_probe(struct platform_device *pdev)
{
	if (alloc_chrdev_region(&mtk_disp_mgr_devno, 0, 1, DISP_SESSION_DEVICE)) return -EFAULT;

	mtk_disp_mgr_cdev = cdev_alloc();
	mtk_disp_mgr_cdev->owner = THIS_MODULE;
	mtk_disp_mgr_cdev->ops = &mtk_disp_mgr_fops;

	if (cdev_add(mtk_disp_mgr_cdev, mtk_disp_mgr_devno, 1)) {
		unregister_chrdev_region(mtk_disp_mgr_devno, 1);
		return -EFAULT;
	}

	mtk_disp_mgr_class = class_create(THIS_MODULE, DISP_SESSION_DEVICE);
	device_create(mtk_disp_mgr_class, NULL, mtk_disp_mgr_devno, NULL, DISP_SESSION_DEVICE);

	disp_sync_init();
	external_display_control_init();

	return 0;
}

static int mtk_disp_mgr_remove(struct platform_device *pdev) { return 0; }
static void mtk_disp_mgr_shutdown(struct platform_device *pdev) {}
static int mtk_disp_mgr_suspend(struct platform_device *pdev, pm_message_t mesg) { return 0; }
static int mtk_disp_mgr_resume(struct platform_device *pdev) { return 0; }

static struct platform_driver mtk_disp_mgr_driver = {
	.probe = mtk_disp_mgr_probe,
	.remove = mtk_disp_mgr_remove,
	.shutdown = mtk_disp_mgr_shutdown,
	.suspend = mtk_disp_mgr_suspend,
	.resume = mtk_disp_mgr_resume,
	.driver = { .name = DISP_SESSION_DEVICE },
};

static void mtk_disp_mgr_device_release(struct device *dev) {}

static u64 mtk_disp_mgr_dmamask = ~(u32) 0;
static struct platform_device mtk_disp_mgr_device = {
	.name = DISP_SESSION_DEVICE,
	.id = 0,
	.dev = {
		.release = mtk_disp_mgr_device_release,
		.dma_mask = &mtk_disp_mgr_dmamask,
		.coherent_dma_mask = 0xffffffff,
		},
	.num_resources = 0,
};

static int __init mtk_disp_mgr_init(void)
{
	if (platform_device_register(&mtk_disp_mgr_device)) return -ENODEV;
	if (platform_driver_register(&mtk_disp_mgr_driver)) {
		platform_device_unregister(&mtk_disp_mgr_device);
		return -ENODEV;
	}
	return 0;
}

static void __exit mtk_disp_mgr_exit(void)
{
	cdev_del(mtk_disp_mgr_cdev);
	unregister_chrdev_region(mtk_disp_mgr_devno, 1);
	platform_driver_unregister(&mtk_disp_mgr_driver);
	platform_device_unregister(&mtk_disp_mgr_device);
	device_destroy(mtk_disp_mgr_class, mtk_disp_mgr_devno);
	class_destroy(mtk_disp_mgr_class);
}
module_init(mtk_disp_mgr_init);
module_exit(mtk_disp_mgr_exit);

MODULE_DESCRIPTION("MediaTek Display Manager");
