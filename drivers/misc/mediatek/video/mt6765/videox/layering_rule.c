// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#define pr_fmt(fmt) "disp_layering: " fmt

#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>

#include "mmdvfs_pmqos.h"
#include "layering_rule.h"
#include "ddp_rsz.h"
#include "primary_display.h"
#include "disp_lowpower.h"
#include "mtk_disp_mgr.h"
#include "disp_rect.h"

static struct layering_rule_ops l_rule_ops;
static struct layering_rule_info_t l_rule_info;

int emi_bound_table[HRT_BOUND_NUM][HRT_LEVEL_NUM] = {
	{500, 600, 700, 700}, {400, 500, 600, 600}, {350, 350, 350, 350},
	{250, 250, 250, 250}, {350, 350, 350, 350}, {400, 400, 400, 600},
	{750, 750, 750, 750}, {1100, 1350, 1550, 1550}, {550, 550, 550, 550},
	{900, 1100, 1350, 1350},
};

int larb_bound_table[HRT_BOUND_NUM][HRT_LEVEL_NUM] = {
	{1200, 1200, 1200, 1200}, {1200, 1200, 1200, 1200}, {1200, 1200, 1200, 1200},
	{1200, 1200, 1200, 1200}, {1200, 1200, 1200, 1200}, {1200, 1200, 1200, 1200},
	{1200, 1200, 1200, 1200}, {1200, 1200, 1200, 1200}, {1200, 1200, 1200, 1200},
	{1200, 1200, 1200, 1200},
};

int mm_freq_table[HRT_DRAMC_TYPE_NUM][HRT_OPP_LEVEL_NUM] = {
	{457, 312, 228}, {457, 312, 228}, {457, 312, 228},
};

#ifndef CONFIG_MTK_ROUND_CORNER_SUPPORT
static int layer_mapping_table[HRT_TB_NUM][TOTAL_OVL_LAYER_NUM] = {
	{0x00010001, 0x00030003, 0x00030007, 0x0003000F, 0x0003001F, 0x0003003F, 0x0003003F, 0x0003003F, 0x0003003F, 0x0003003F, 0x0003003F, 0x0003003F},
	{0x00010001, 0x00030005, 0x0003000D, 0x0003001D, 0x0003003D, 0x0003003D, 0x0003003D, 0x0003003D, 0x0003003D, 0x0003003D, 0x0003003D, 0x0003003D},
	{0x00010001, 0x00030003, 0x00030007, 0x0003000F, 0x0003001F, 0x0003003F, 0x0003003F, 0x0003003F, 0x0003003F, 0x0003003F, 0x0003003F, 0x0003003F},
	{0x00010001, 0x00030003, 0x00030007, 0x0003000F, 0x0003001F, 0x0003003F, 0x0003003F, 0x0003003F, 0x0003003F, 0x0003003F, 0x0003003F, 0x0003003F},
};
static int ovl_mapping_table[HRT_TB_NUM] = { 0x00020022, 0x00020022, 0x00020022, 0x00020022 };
#else
static int layer_mapping_table[HRT_TB_NUM][TOTAL_OVL_LAYER_NUM] = {
	{0x00010001, 0x00030003, 0x00030007, 0x0003000F, 0x0003001F, 0x0003001F, 0x0003001F, 0x0003001F, 0x0003001F, 0x0003001F},
	{0x00010001, 0x00030005, 0x0003000D, 0x0003001D, 0x0003001D, 0x0003001D, 0x0003001D, 0x0003001D, 0x0003001D, 0x0003001D},
	{0x00010001, 0x00030003, 0x00030007, 0x0003000F, 0x0003001F, 0x0003001F, 0x0003001F, 0x0003001F, 0x0003001F, 0x0003001F},
	{0x00010001, 0x00030003, 0x00030007, 0x0003000F, 0x0003001F, 0x0003001F, 0x0003001F, 0x0003001F, 0x0003001F, 0x0003001F},
};
static int ovl_mapping_table[HRT_TB_NUM] = { 0x00020012, 0x00020012, 0x00020012, 0x00020012 };
#endif

static int larb_mapping_table[HRT_TB_NUM] = { 0x00010010, 0x00010010, 0x00010010, 0x00010010 };

#define GET_SYS_STATE(sys_state) ((l_rule_info.hrt_sys_state >> sys_state) & 0x1)

static bool has_rsz_layer(struct disp_layer_info *disp_info, int disp_idx)
{
	int i;
	struct layer_config *c;

	for (i = 0; i < disp_info->layer_num[disp_idx]; i++) {
		c = &disp_info->input_config[disp_idx][i];
		if (!is_gles_layer(disp_info, disp_idx, i) &&
			(c->src_height != c->dst_height || c->src_width != c->dst_width))
			return true;
	}
	return false;
}

static bool same_ratio(struct layer_config *input, struct layer_config *tgt)
{
	int diff_w = (tgt->dst_width * input->src_width + (input->src_width - 1)) / input->dst_width - tgt->src_width;
	int diff_h = (tgt->dst_height * input->src_height + (input->src_height - 1)) / input->dst_height - tgt->src_height;
	return (diff_w <= 1 && diff_w >= -1 && diff_h <= 1 && diff_h >= -1);
}

#define RATIO_LIMIT  2
static bool same_ratio_limitation(struct layer_config *tgt, int limitation)
{
	int diff_w = tgt->dst_width - tgt->src_width;
	int diff_h = tgt->dst_height - tgt->src_height;
	int pw = primary_display_get_width(), ph = primary_display_get_height();

	if (pw <= 0 || ph <= 0) return false;
	return ((100 * diff_w / pw < limitation) && (diff_w > 0)) ||
	((100 * diff_h / ph < limitation) && (diff_h > 0));
}

static bool is_RPO(struct disp_layer_info *disp_info, int disp_idx, int *rsz_idx, bool *has_dim_layer)
{
	int i, gpu_rsz_idx = 0;
	struct layer_config *c, *basic_layer = NULL;
	struct disp_rect src_layer_roi = {0}, src_total_roi = {0}, dst_layer_roi = {0}, dst_total_roi = {0};

	*has_dim_layer = false;
	*rsz_idx = -1;

	if (disp_info->layer_num[disp_idx] <= 0)
		return false;

	for (i = 0; i < disp_info->layer_num[disp_idx] && i < 2; i++) {
		c = &disp_info->input_config[disp_idx][i];

		if (i == 0 && c->src_fmt == DISP_FORMAT_DIM) {
			*has_dim_layer = true;
			continue;
		}

		if (disp_info->gles_head[disp_idx] >= 0 && disp_info->gles_head[disp_idx] <= i) break;
		if (c->src_width == c->dst_width && c->src_height == c->dst_height) break;
		if (c->src_width > c->dst_width || c->src_height > c->dst_height) break;

		if ((has_layer_cap(c, MDP_RSZ_LAYER) || has_layer_cap(c, MDP_ROT_LAYER)) &&
			(c->dst_width - c->src_width <= MDP_ALIGNMENT_MARGIN ||
			c->dst_height - c->src_height <= MDP_ALIGNMENT_MARGIN))
			break;

		if ((i == 0 && !*has_dim_layer) || (i == 1 && *has_dim_layer))
			basic_layer = c;
		else if (!same_ratio(basic_layer, c) || same_ratio_limitation(c, RATIO_LIMIT))
			break;

		rect_make(&src_layer_roi, (c->dst_offset_x * c->src_width) / c->dst_width,
				  (c->dst_offset_y * c->src_height) / c->dst_height, c->src_width, c->src_height);
		rect_join(&src_layer_roi, &src_total_roi, &src_total_roi);

		rect_make(&dst_layer_roi, c->dst_offset_x, c->dst_offset_y, c->dst_width, c->dst_height);
		rect_join(&dst_layer_roi, &dst_total_roi, &dst_total_roi);

		if (src_total_roi.width > dst_total_roi.width || src_total_roi.height > dst_total_roi.height) {
			pr_err("RSZ layer%d scale-down detected\n", i);
			break;
		}

		if (src_total_roi.width > RSZ_TILE_LENGTH - RSZ_ALIGNMENT_MARGIN || src_total_roi.height > RSZ_IN_MAX_HEIGHT)
			break;

		c->layer_caps |= DISP_RSZ_LAYER;
		*rsz_idx = i;
	}

	if (*rsz_idx == -1)
		return false;

	for (i = *rsz_idx + 1; i < disp_info->layer_num[disp_idx]; i++) {
		c = &disp_info->input_config[disp_idx][i];
		if ((c->src_width != c->dst_width || c->src_height != c->dst_height) && !has_layer_cap(c, MDP_RSZ_LAYER)) {
			gpu_rsz_idx = i;
			break;
		}
	}

	if (gpu_rsz_idx)
		rollback_resize_layer_to_GPU_range(disp_info, disp_idx, gpu_rsz_idx, disp_info->layer_num[disp_idx] - 1);

	return true;
}

static bool lr_rsz_layout(struct disp_layer_info *disp_info)
{
	int disp_idx;

	if (is_ext_path(disp_info))
		rollback_all_resize_layer_to_GPU(disp_info, HRT_SECONDARY);

	for (disp_idx = 0; disp_idx < 2; disp_idx++) {
		int rsz_idx = 0;
		bool has_dim_layer = false;

		if (disp_info->layer_num[disp_idx] <= 0 || disp_idx == HRT_SECONDARY)
			continue;

		if (!has_rsz_layer(disp_info, disp_idx)) {
			l_rule_info.scale_rate = HRT_SCALE_NONE;
			l_rule_info.disp_path = HRT_PATH_UNKNOWN;
		} else if (is_RPO(disp_info, disp_idx, &rsz_idx, &has_dim_layer)) {
			if (rsz_idx == 0) l_rule_info.disp_path = HRT_PATH_RPO_L0;
			else if (rsz_idx == 1 && has_dim_layer) l_rule_info.disp_path = HRT_PATH_RPO_DIM_L0;
			else if (rsz_idx != -1) l_rule_info.disp_path = HRT_PATH_RPO_BOTH;
		} else {
			rollback_all_resize_layer_to_GPU(disp_info, HRT_PRIMARY);
			l_rule_info.scale_rate = HRT_SCALE_NONE;
			l_rule_info.disp_path = HRT_PATH_UNKNOWN;
		}
	}
	return 0;
}

static bool lr_unset_disp_rsz_attr(struct disp_layer_info *disp_info, int disp_idx)
{
	struct layer_config *lc = &disp_info->input_config[disp_idx][0];

	if (l_rule_info.disp_path == HRT_PATH_RPO_L0 && has_layer_cap(lc, MDP_RSZ_LAYER) && has_layer_cap(lc, DISP_RSZ_LAYER)) {
		lc->layer_caps &= ~DISP_RSZ_LAYER;
		l_rule_info.disp_path = HRT_PATH_GENERAL;
		l_rule_info.layer_tb_idx = HRT_TB_TYPE_GENERAL;
		return true;
	}
	return false;
}

static void layering_rule_senario_decision(struct disp_layer_info *disp_info)
{
	mmprofile_log_ex(ddp_mmp_get_events()->hrt, MMPROFILE_FLAG_START, l_rule_info.disp_path,
					 l_rule_info.layer_tb_idx | (l_rule_info.bound_tb_idx << 16));

	if (GET_SYS_STATE(DISP_HRT_MULTI_TUI_ON)) {
		l_rule_info.disp_path = HRT_PATH_GENERAL;
		l_rule_info.layer_tb_idx = HRT_TB_TYPE_GENERAL;
	} else {
		if (l_rule_info.disp_path == HRT_PATH_RPO_L0) l_rule_info.layer_tb_idx = HRT_TB_TYPE_RPO_L0;
		else if (l_rule_info.disp_path == HRT_PATH_RPO_DIM_L0) l_rule_info.layer_tb_idx = HRT_TB_TYPE_RPO_DIM_L0;
		else if (l_rule_info.disp_path == HRT_PATH_RPO_BOTH) l_rule_info.layer_tb_idx = HRT_TB_TYPE_RPO_BOTH;
		else {
			l_rule_info.layer_tb_idx = HRT_TB_TYPE_GENERAL;
			l_rule_info.disp_path = HRT_PATH_GENERAL;
		}
	}
	l_rule_info.primary_fps = 60;

	mmprofile_log_ex(ddp_mmp_get_events()->hrt, MMPROFILE_FLAG_END, l_rule_info.disp_path,
					 l_rule_info.layer_tb_idx | (l_rule_info.bound_tb_idx << 16));
}

static bool filter_by_hw_limitation(struct disp_layer_info *disp_info)
{
	bool flag = false;
	unsigned int i, disp_idx, layer_cnt = 0;
	struct layer_config *info;

	for (disp_idx = 0; disp_idx < 2; ++disp_idx) {
		for (i = 0; i < disp_info->layer_num[disp_idx]; i++) {
			info = &(disp_info->input_config[disp_idx][i]);
			if (info->src_fmt != DISP_FORMAT_RGBA1010102 && info->src_fmt != DISP_FORMAT_RGBA_FP16)
				continue;

			if (disp_info->gles_head[disp_idx] == -1 || i < disp_info->gles_head[disp_idx])
				disp_info->gles_head[disp_idx] = i;
			if (disp_info->gles_tail[disp_idx] == -1 || i > disp_info->gles_tail[disp_idx])
				disp_info->gles_tail[disp_idx] = i;
		}
	}

	for (i = 0; i < disp_info->layer_num[1]; i++) {
		if (is_gles_layer(disp_info, 1, i)) continue;

		if (++layer_cnt > SECONDARY_OVL_LAYER_NUM) {
			if (disp_info->gles_head[1] == -1 || i < disp_info->gles_head[1])
				disp_info->gles_head[1] = i;
			if (disp_info->gles_tail[1] == -1 || i > disp_info->gles_tail[1])
				disp_info->gles_tail[1] = i;
			flag = false;
		}
	}
	return flag;
}

unsigned int layering_rule_get_hrt_idx(void) { return l_rule_info.hrt_idx; }

static void clear_layer(struct disp_layer_info *disp_info)
{
	int di, i;
	struct layer_config *c;

	for (di = 0; di < 2; di++) {
		int g_head = disp_info->gles_head[di];
		int top = -1;

		if (disp_info->layer_num[di] <= 0 || g_head == -1) continue;

		for (i = disp_info->layer_num[di] - 1; i >= g_head; i--) {
			c = &disp_info->input_config[di][i];
			if (has_layer_cap(c, LAYERING_OVL_ONLY) && has_layer_cap(c, CLIENT_CLEAR_LAYER)) {
				top = i;
				break;
			}
		}
		if (top == -1 || !is_gles_layer(disp_info, di, top)) continue;

		c = &disp_info->input_config[di][top];
		c->layer_caps |= DISP_CLIENT_CLEAR_LAYER;

		disp_info->gles_head[di] = 0;
		disp_info->gles_tail[di] = disp_info->layer_num[di] - 1;
		for (i = 0; i < disp_info->layer_num[di]; i++) {
			c = &disp_info->input_config[di][i];
			c->ext_sel_layer = -1;
			c->ovl_id = (i == top) ? 0 : 1;
		}
	}
}

static int get_hrt_bound(int is_larb, int hrt_level)
{
	return is_larb ? larb_bound_table[l_rule_info.bound_tb_idx][hrt_level] : emi_bound_table[l_rule_info.bound_tb_idx][hrt_level];
}

static int *get_bound_table(enum DISP_HW_MAPPING_TB_TYPE tb_type)
{
	if (tb_type == DISP_HW_EMI_BOUND_TB) return emi_bound_table[l_rule_info.bound_tb_idx];
	if (tb_type == DISP_HW_LARB_BOUND_TB) return larb_bound_table[l_rule_info.bound_tb_idx];
	return NULL;
}

static int get_mapping_table(enum DISP_HW_MAPPING_TB_TYPE tb_type, int param)
{
	if (tb_type == DISP_HW_OVL_TB) return ovl_mapping_table[l_rule_info.layer_tb_idx];
	if (tb_type == DISP_HW_LARB_TB) return larb_mapping_table[l_rule_info.layer_tb_idx];
	if (tb_type == DISP_HW_LAYER_TB) return (param < MAX_PHY_OVL_CNT && param >= 0) ? layer_mapping_table[l_rule_info.layer_tb_idx][param] : -1;
	return -1;
}

#ifdef CONFIG_MTK_HIGH_FRAME_RATE
unsigned long long layering_get_frame_bw(int active_cfg_id)
{
	static unsigned long long bw_base;
	unsigned int timing_fps = 6000;

	if (bw_base) return bw_base;

	primary_display_get_cfg_fps(active_cfg_id, NULL, &timing_fps);
	bw_base = (unsigned long long)primary_display_get_width() * primary_display_get_height() * (timing_fps / 100) * 125 * 4;
	bw_base /= 100 * 1024 * 1024;
	return bw_base;
}

#ifdef MTK_FB_MMDVFS_SUPPORT
int layering_get_valid_hrt(int active_config_id)
{
	unsigned long long dvfs_bw = 200;
	int tmp_bw = mm_hrt_get_available_hrt_bw(get_virtual_port(VIRTUAL_DISP));
	unsigned long long tmp = layering_get_frame_bw(active_config_id);

	if (tmp_bw >= 0) {
		dvfs_bw = (10000ULL * tmp_bw) / (tmp * 100);
		if (dvfs_bw < 200) dvfs_bw = 200;
	}
	return dvfs_bw;
}
#endif
#endif

int set_emi_bound_tb(int idx, int num, int *val)
{
	int i;
	if (idx >= HRT_BOUND_NUM || idx < 0 || num > HRT_LEVEL_NUM) return -EINVAL;
	for (i = 0; i < num; i++) emi_bound_table[idx][i] = val[i];
	return 0;
}

void layering_rule_init(void)
{
	int opt;
	l_rule_info.primary_fps = 60;
	register_layering_rule_ops(&l_rule_ops, &l_rule_info);

	if ((opt = disp_helper_get_option(DISP_OPT_RPO)) != -1) set_layering_opt(LYE_OPT_RPO, opt);
	if ((opt = disp_helper_get_option(DISP_OPT_OVL_EXT_LAYER)) != -1) set_layering_opt(LYE_OPT_EXT_LAYER, opt);
}

int layering_rule_get_mm_freq_table(enum HRT_OPP_LEVEL opp_level)
{
	if (opp_level > HRT_OPP_LEVEL_DEFAULT) return 0;
	mmprofile_log_ex(ddp_mmp_get_events()->dvfs, MMPROFILE_FLAG_PULSE, HRT_DRAMC_TYPE_LP4_3733, opp_level);
	return mm_freq_table[HRT_DRAMC_TYPE_LP4_3733][opp_level];
}

static struct layering_rule_ops l_rule_ops = {
	.resizing_rule = lr_rsz_layout,
	.scenario_decision = layering_rule_senario_decision,
	.get_bound_table = get_bound_table,
	.get_hrt_bound = get_hrt_bound,
	.get_mapping_table = get_mapping_table,
	.rollback_to_gpu_by_hw_limitation = filter_by_hw_limitation,
	.unset_disp_rsz_attr = lr_unset_disp_rsz_attr,
	.clear_layer = clear_layer,
};
