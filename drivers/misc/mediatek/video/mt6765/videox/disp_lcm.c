// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 * Copyright (C) 2021 XiaoMi, Inc.
 */

#define pr_fmt(fmt) "disp_lcm: " fmt

#include <linux/slab.h>
#include <linux/types.h>
#include <linux/printk.h>
#include "lcm_drv.h"
#include "lcm_define.h"
#include "disp_drv_platform.h"
#include "ddp_manager.h"
#include "disp_lcm.h"
#include "disp_feature.h"

#if defined(MTK_LCM_DEVICE_TREE_SUPPORT)
#include <linux/of.h>
#endif
#include <linux/delay.h>

extern void DSI_set_cmdq_V2_Wrapper_DSI0(unsigned int cmd, unsigned char count, unsigned char *para_list, unsigned char force_update);
extern int do_lcm_vdo_lp_write(struct ddp_lcm_write_cmd_table *write_table, unsigned int count);
extern int do_lcm_vdo_lp_read(struct ddp_lcm_read_cmd_table *read_table);
struct LCM_setting_table *hbm0_on, *hbm1_on, *hbm2_on;
extern char mtkfb_lcm_name[256];
extern unsigned int islcmconnected;
extern void disp_aal_set_ess_level(int level);
int esd_backlight_level;

int _lcm_count(void) { return lcm_count; }

int _is_lcm_inited(struct disp_lcm_handle *plcm)
{
	if (plcm && plcm->params && plcm->drv) return 1;
	pr_warn("invalid lcm handle: %p\n", plcm);
	return 0;
}

struct LCM_PARAMS *_get_lcm_params_by_handle(struct disp_lcm_handle *plcm)
{
	return plcm ? plcm->params : NULL;
}

struct LCM_DRIVER *_get_lcm_driver_by_handle(struct disp_lcm_handle *plcm)
{
	return plcm ? plcm->drv : NULL;
}

void _dump_lcm_info(struct disp_lcm_handle *plcm)
{
	struct LCM_DRIVER *l = plcm ? plcm->drv : NULL;
	struct LCM_PARAMS *p = plcm ? plcm->params : NULL;

	if (!l || !p) return;

	pr_debug("[LCM] name: %s, res: %dx%d, phys: %dx%d\n",
		l->name, p->width, p->height, p->physical_width, p->physical_height);
}

#if defined(MTK_LCM_DEVICE_TREE_SUPPORT)
static unsigned char dts[sizeof(struct LCM_DATA)*MAX_SIZE];
static struct LCM_DTS lcm_dts;

int disp_of_getprop_u32(const struct device_node *np, const char *propname, u32 *out_value)
{
	int len = 0, i;
	const unsigned int *prop = of_get_property(np, propname, &len);
	if (!prop) return 0;
	len /= sizeof(*prop);
	for (i = 0; i < len; i++) out_value[i] = be32_to_cpup(prop++);
	return len;
}

int disp_of_getprop_u8(const struct device_node *np, const char *propname, u8 *out_value)
{
	int len = 0, i;
	const unsigned int *prop = of_get_property(np, propname, &len);
	if (!prop) return 0;
	len /= sizeof(*prop);
	for (i = 0; i < len; i++) out_value[i] = (u8)(be32_to_cpup(prop++) & 0xFF);
	return len;
}

void parse_lcm_params_dt_node(struct device_node *np, struct LCM_PARAMS *p)
{
	if (!p) return;
	memset(p, 0x0, sizeof(*p));

	disp_of_getprop_u32(np, "lcm_params-types", &p->type);
	disp_of_getprop_u32(np, "lcm_params-resolution", &p->width);
	disp_of_getprop_u32(np, "lcm_params-io_select_mode", &p->io_select_mode);

	disp_of_getprop_u32(np, "lcm_params-dsi-mode", &p->dsi.mode);
	disp_of_getprop_u32(np, "lcm_params-dsi-switch_mode", &p->dsi.switch_mode);
	disp_of_getprop_u32(np, "lcm_params-dsi-lane_num", &p->dsi.LANE_NUM);
	disp_of_getprop_u32(np, "lcm_params-dsi-data_format", (u32 *)&p->dsi.data_format);
	disp_of_getprop_u32(np, "lcm_params-dsi-vertical_sync_active", &p->dsi.vertical_sync_active);
	disp_of_getprop_u32(np, "lcm_params-dsi-vertical_backporch", &p->dsi.vertical_backporch);
	disp_of_getprop_u32(np, "lcm_params-dsi-vertical_frontporch", &p->dsi.vertical_frontporch);
	disp_of_getprop_u32(np, "lcm_params-dsi-vertical_active_line", &p->dsi.vertical_active_line);
	disp_of_getprop_u32(np, "lcm_params-dsi-horizontal_sync_active", &p->dsi.horizontal_sync_active);
	disp_of_getprop_u32(np, "lcm_params-dsi-horizontal_backporch", &p->dsi.horizontal_backporch);
	disp_of_getprop_u32(np, "lcm_params-dsi-horizontal_frontporch", &p->dsi.horizontal_frontporch);
	disp_of_getprop_u32(np, "lcm_params-dsi-horizontal_blanking_pixel", &p->dsi.horizontal_blanking_pixel);
	disp_of_getprop_u32(np, "lcm_params-dsi-horizontal_active_pixel", &p->dsi.horizontal_active_pixel);
	disp_of_getprop_u32(np, "lcm_params-dsi-pll_select", &p->dsi.pll_select);
	disp_of_getprop_u32(np, "lcm_params-dsi-pll_clock", &p->dsi.PLL_CLOCK);
	disp_of_getprop_u32(np, "lcm_params-dsi-dsi_clock", &p->dsi.dsi_clock);

	disp_of_getprop_u32(np, "lcm_params-physical_width", &p->physical_width);
	disp_of_getprop_u32(np, "lcm_params-physical_height", &p->physical_height);
}

static void parse_table(struct device_node *np, const char *name, u8 *dts, struct LCM_DATA *table, unsigned int *out_size, int max_size)
{
	int len = disp_of_getprop_u8(np, name, dts);
	int i = 0, tmp_len;
	u8 *tmp = dts;

	if (len <= 0 || len > (sizeof(struct LCM_DATA) * max_size)) return;

	while (len > 0 && i < max_size) {
		table[i].func = tmp[0];
		table[i].type = tmp[1];
		table[i].size = tmp[2];
		tmp_len = 3 + table[i].size;

		if (table[i].size > 0)
			memcpy(&table[i].data_t1, tmp + 3, table[i].size);

		tmp += tmp_len;
		len -= tmp_len;
		i++;
	}
	*out_size = i;
}

void parse_lcm_ops_dt_node(struct device_node *np, struct LCM_DTS *lcm_dts, unsigned char *dts)
{
	if (!lcm_dts) return;
	parse_table(np, "init", dts, lcm_dts->init, &lcm_dts->init_size, INIT_SIZE);
	parse_table(np, "compare_id", dts, lcm_dts->compare_id, &lcm_dts->compare_id_size, COMPARE_ID_SIZE);
	parse_table(np, "suspend", dts, lcm_dts->suspend, &lcm_dts->suspend_size, SUSPEND_SIZE);
	parse_table(np, "backlight", dts, lcm_dts->backlight, &lcm_dts->backlight_size, BACKLIGHT_SIZE);
	parse_table(np, "backlight_cmdq", dts, lcm_dts->backlight_cmdq, &lcm_dts->backlight_cmdq_size, BACKLIGHT_CMDQ_SIZE);
}

int check_lcm_node_from_DT(void)
{
	char lcm_node[128];
	sprintf(lcm_node, "mediatek,lcm_params-%s", lcm_name_list[0]);
	if (!of_find_compatible_node(NULL, NULL, lcm_node)) return -1;
	sprintf(lcm_node, "mediatek,lcm_ops-%s", lcm_name_list[0]);
	if (!of_find_compatible_node(NULL, NULL, lcm_node)) return -1;
	return 0;
}

void load_lcm_resources_from_DT(struct LCM_DRIVER *lcm_drv)
{
	char lcm_node[128];
	struct device_node *np;

	if (!lcm_drv) return;
	memset(&lcm_dts, 0x0, sizeof(lcm_dts));

	sprintf(lcm_node, "mediatek,lcm_params-%s", lcm_name_list[0]);
	if ((np = of_find_compatible_node(NULL, NULL, lcm_node)))
		parse_lcm_params_dt_node(np, &lcm_dts.params);

	sprintf(lcm_node, "mediatek,lcm_ops-%s", lcm_name_list[0]);
	if ((np = of_find_compatible_node(NULL, NULL, lcm_node)))
		parse_lcm_ops_dt_node(np, &lcm_dts, dts);

	if (lcm_drv->parse_dts) lcm_drv->parse_dts(&lcm_dts, 1);
}
#endif

static void diff_panel_set_cmd(struct disp_lcm_handle *plcm)
{
	if (_is_lcm_inited(plcm)) {
		if (strnstr(plcm->drv->name, "nt36525b", strlen(plcm->drv->name))) {
			hbm0_on = dijing_hbm0_on; hbm1_on = dijing_hbm1_on; hbm2_on = dijing_hbm2_on;
		}
		if (strnstr(plcm->drv->name, "ft8006s", strlen(plcm->drv->name))) {
			hbm0_on = helitai_hbm0_on; hbm1_on = helitai_hbm1_on; hbm2_on = helitai_hbm2_on;
		}
		if (strnstr(plcm->drv->name, "hx83102d", strlen(plcm->drv->name))) {
			hbm0_on = xinli_hbm0_on; hbm1_on = xinli_hbm1_on; hbm2_on = xinli_hbm2_on;
		}
	}
}

static void display_feature_push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	for (unsigned int i = 0; i < count; i++)
		DSI_set_cmdq_V2_Wrapper_DSI0(table[i].cmd, table[i].count, table[i].para_list, force_update);
}

static ssize_t dsi_display_set_hbm(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	int param = 0;
	struct ddp_lcm_write_cmd_table dimming_on[1] = { {0x53, 1, {0x2C}} };

	do_lcm_vdo_lp_write(dimming_on, 1);
	if (kstrtoint(buf, 10, &param)) return -EINVAL;

	switch (param) {
	case 0x1: display_feature_push_table(hbm1_on, 1, 1); break;
	case 0x2: display_feature_push_table(hbm2_on, 1, 1); break;
	case 0x0: display_feature_push_table(hbm0_on, 1, 1); break;
	}
	return len;
}

static ssize_t dsi_display_set_cabc(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	int param = 0;
	if (kstrtoint(buf, 10, &param)) return -EINVAL;

	switch (param) {
	case 0x1: disp_aal_set_ess_level(29); break;
	case 0x2: disp_aal_set_ess_level(88); break;
	case 0x3: disp_aal_set_ess_level(180); break;
	case 0x0: disp_aal_set_ess_level(0); break;
	}
	return len;
}

static ssize_t white_point_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ddp_lcm_read_cmd_table read_table = {0};
	if (islcmconnected == 1) {
		if (strnstr(mtkfb_lcm_name, "nt36525b", strlen(mtkfb_lcm_name))) {
			struct ddp_lcm_write_cmd_table w[2] = { {0xFF, 1, {0x10}}, {0xFB, 1, {0x01}} };
			do_lcm_vdo_lp_write(w, 2);
			read_table.cmd[0] = 0xDA; read_table.cmd[1] = 0xDB;
			do_lcm_vdo_lp_read(&read_table);
		} else if (strnstr(mtkfb_lcm_name, "ft8006s", strlen(mtkfb_lcm_name))) {
			struct LCM_setting_table w1[1] = { {0x41, 3, {0x5A, 0x02}} }, w2[1] = { {0x41, 3, {0x5A, 0x2F}} };
			display_feature_push_table(w1, 1, 1);
			read_table.cmd[0] = 0x8A; read_table.cmd[1] = 0x8B;
			do_lcm_vdo_lp_read(&read_table);
			display_feature_push_table(w2, 1, 1);
		} else if (strnstr(mtkfb_lcm_name, "hx83102d", strlen(mtkfb_lcm_name))) {
			struct LCM_setting_table w[2] = { {0xB9, 3, {0x83, 0x10, 0x2D}}, {0xBD, 1, {0x00}} };
			display_feature_push_table(w, 2, 1);
			read_table.cmd[0] = 0xDA; read_table.cmd[1] = 0xDB;
			do_lcm_vdo_lp_read(&read_table);
		}
		sprintf(buf, "val0=%d,val1=%d\n", read_table.data[0].byte1, read_table.data[1].byte1);
	} else sprintf(buf, "null\n");
	return strlen(buf) + 1;
}

static ssize_t brightness_light_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ddp_lcm_read_cmd_table read_table = {0};
	if (islcmconnected == 1) {
		if (strnstr(mtkfb_lcm_name, "nt36525b", strlen(mtkfb_lcm_name))) {
			struct ddp_lcm_write_cmd_table w[2] = { {0xFF, 1, {0x10}}, {0xFB, 1, {0x01}} };
			do_lcm_vdo_lp_write(w, 2);
			read_table.cmd[0] = 0xDC; do_lcm_vdo_lp_read(&read_table);
		} else if (strnstr(mtkfb_lcm_name, "ft8006s", strlen(mtkfb_lcm_name))) {
			struct LCM_setting_table w1[1] = { {0x41, 2, {0x5A, 0x02}} }, w2[1] = { {0x41, 3, {0x5A, 0x2F}} };
			display_feature_push_table(w1, 1, 1);
			read_table.cmd[0] = 0x8C; do_lcm_vdo_lp_read(&read_table);
			display_feature_push_table(w2, 1, 1);
		} else if (strnstr(mtkfb_lcm_name, "hx83102d", strlen(mtkfb_lcm_name))) {
			struct LCM_setting_table w[2] = { {0xB9, 3, {0x83, 0x10, 0x2D}}, {0xBD, 1, {0x00}} };
			display_feature_push_table(w, 1, 1);
			read_table.cmd[0] = 0xDC; do_lcm_vdo_lp_read(&read_table);
		}
		sprintf(buf, "%d\n", read_table.data[0].byte1);
	} else sprintf(buf, "null\n");
	return strlen(buf) + 1;
}

static DEVICE_ATTR(hbm_mode, 0644, NULL, dsi_display_set_hbm);
static DEVICE_ATTR(cabc_mode, 0644, NULL, dsi_display_set_cabc);
static DEVICE_ATTR(whitepoint, 0664, white_point_show, NULL);
static DEVICE_ATTR(brightness_light, 0664, brightness_light_show, NULL);

static void display_feature_create_sysfs(void)
{
	struct kobject *objs[] = {
		kobject_create_and_add("display_hbm", NULL),
		kobject_create_and_add("display_cabc", NULL),
		kobject_create_and_add("android_whitepoint", NULL),
		kobject_create_and_add("android_brightness", NULL)
	};
	struct device_attribute *attrs[] = {
		&dev_attr_hbm_mode, &dev_attr_cabc_mode, &dev_attr_whitepoint, &dev_attr_brightness_light
	};

	for (int i = 0; i < 4; i++) {
		if (objs[i] && sysfs_create_file(objs[i], &attrs[i]->attr))
			kobject_del(objs[i]);
	}
}

struct disp_lcm_handle *disp_lcm_probe(char *plcm_name, enum LCM_INTERFACE_ID lcm_id, int is_lcm_inited)
{
	bool isLCMFound = false, isLCMInited = false;
	struct LCM_DRIVER *lcm_drv = NULL;
	struct disp_lcm_handle *plcm = NULL;

#if defined(MTK_LCM_DEVICE_TREE_SUPPORT)
	if (check_lcm_node_from_DT() == 0) {
		lcm_drv = &lcm_common_drv;
		lcm_drv->name = lcm_name_list[0];
		if (strcmp(lcm_drv->name, plcm_name)) return NULL;
		isLCMFound = true;
		isLCMInited = is_lcm_inited;
	} else
#endif
	if (_lcm_count() == 1) {
		lcm_drv = lcm_driver_list[0];
		if (plcm_name && strcmp(lcm_drv->name, plcm_name)) return NULL;
		isLCMFound = true;
		isLCMInited = is_lcm_inited && plcm_name;
	} else if (plcm_name) {
		for (int i = 0; i < _lcm_count(); i++) {
			if (!strcmp(lcm_driver_list[i]->name, plcm_name)) {
				lcm_drv = lcm_driver_list[i];
				isLCMFound = true;
				isLCMInited = is_lcm_inited;
				break;
			}
		}
	}

	if (!isLCMFound) return NULL;

	plcm = kzalloc(sizeof(*plcm), GFP_KERNEL);
	if (!plcm) return NULL;
	plcm->params = kzalloc(sizeof(*plcm->params), GFP_KERNEL);
	if (!plcm->params) {
		kfree(plcm);
		return NULL;
	}

	plcm->drv = lcm_drv;
	plcm->is_inited = isLCMInited;

#if defined(MTK_LCM_DEVICE_TREE_SUPPORT)
	load_lcm_resources_from_DT(plcm->drv);
#endif

	plcm->drv->get_params(plcm->params);
	plcm->lcm_if_id = plcm->params->lcm_if;

	esd_backlight_level = 0;
	diff_panel_set_cmd(plcm);
	display_feature_create_sysfs();

	if (plcm->params->lcm_if == LCM_INTERFACE_NOTDEFINED) {
		if (plcm->params->type == LCM_TYPE_DSI) plcm->lcm_if_id = LCM_INTERFACE_DSI0;
		if (plcm->params->type == LCM_TYPE_DPI) plcm->lcm_if_id = LCM_INTERFACE_DPI0;
		if (plcm->params->type == LCM_TYPE_DBI) plcm->lcm_if_id = LCM_INTERFACE_DBI0;
	}

	if (lcm_id == LCM_INTERFACE_NOTDEFINED || lcm_id == plcm->lcm_if_id) {
		plcm->lcm_original_width = plcm->params->width;
		plcm->lcm_original_height = plcm->params->height;
		return plcm;
	}

	kfree(plcm->params);
	kfree(plcm);
	return NULL;
}

struct disp_lcm_handle *disp_ext_lcm_probe(char *plcm_name, enum LCM_INTERFACE_ID lcm_id, int is_lcm_inited)
{
	bool isLCMFound = false;
	struct LCM_DRIVER *lcm_drv = NULL;
	struct disp_lcm_handle *plcm = NULL;

	if (_lcm_count() < 2) return NULL;
	if (plcm_name) {
		for (int i = 0; i < _lcm_count(); i++) {
			if (!strcmp(lcm_driver_list[i]->name, plcm_name)) {
				lcm_drv = lcm_driver_list[i];
				isLCMFound = true;
				break;
			}
		}
	} else {
#if defined(CONFIG_MTK_DUAL_DISPLAY_SUPPORT) && (CONFIG_MTK_DUAL_DISPLAY_SUPPORT == 2)
		lcm_drv = lcm_driver_list[1];
		isLCMFound = true;
#endif
	}

	if (!isLCMFound || !lcm_drv) return NULL;

	plcm = kzalloc(sizeof(*plcm), GFP_KERNEL);
	if (!plcm) return NULL;
	plcm->params = kzalloc(sizeof(*plcm->params), GFP_KERNEL);
	if (!plcm->params) {
		kfree(plcm);
		return NULL;
	}

	plcm->drv = lcm_drv;
	plcm->is_inited = is_lcm_inited && plcm_name;
	plcm->drv->get_params(plcm->params);
	plcm->lcm_if_id = (plcm->params->lcm_if == LCM_INTERFACE_NOTDEFINED) ? LCM_INTERFACE_DSI1 : plcm->params->lcm_if;

	if (lcm_id == LCM_INTERFACE_NOTDEFINED || lcm_id == plcm->lcm_if_id) {
		plcm->lcm_original_width = plcm->params->width;
		plcm->lcm_original_height = plcm->params->height;
		return plcm;
	}

	kfree(plcm->params);
	kfree(plcm);
	return NULL;
}

int disp_lcm_init(struct disp_lcm_handle *plcm, int force)
{
	if (!_is_lcm_inited(plcm) || !plcm->drv->init) return -1;
	if (plcm->drv->init_power && (!plcm->is_inited || force)) plcm->drv->init_power();
	if (!plcm->is_inited || force) plcm->drv->init();
	return 0;
}

struct LCM_PARAMS *disp_lcm_get_params(struct disp_lcm_handle *plcm) { return _is_lcm_inited(plcm) ? plcm->params : NULL; }
enum LCM_INTERFACE_ID disp_lcm_get_interface_id(struct disp_lcm_handle *plcm) { return _is_lcm_inited(plcm) ? plcm->lcm_if_id : LCM_INTERFACE_NOTDEFINED; }

int disp_lcm_update(struct disp_lcm_handle *plcm, int x, int y, int w, int h, int force)
{
	if (!_is_lcm_inited(plcm) || !plcm->drv->update) return -1;
	plcm->drv->update(x, y, w, h);
	return 0;
}

int disp_lcm_esd_check(struct disp_lcm_handle *plcm)
{
	return (_is_lcm_inited(plcm) && plcm->drv->esd_check) ? plcm->drv->esd_check() : 0;
}

int disp_lcm_esd_recover(struct disp_lcm_handle *plcm)
{
	if (!_is_lcm_inited(plcm)) return -1;
	if (plcm->drv->esd_recover) plcm->drv->esd_recover();
	else disp_lcm_init(plcm, 1);
	return 0;
}

#if defined(CONFIG_TOUCHSCREEN_COMMON)
extern int tpd_gesture_flag;
#endif

int disp_lcm_suspend(struct disp_lcm_handle *plcm)
{
	if (!_is_lcm_inited(plcm) || !plcm->drv->suspend) return -1;
	plcm->drv->suspend();
#if defined(CONFIG_TOUCHSCREEN_COMMON)
	if (!tpd_gesture_flag && plcm->drv->suspend_power) plcm->drv->suspend_power();
#else
	if (plcm->drv->suspend_power) plcm->drv->suspend_power();
#endif
	return 0;
}

int disp_lcm_resume(struct disp_lcm_handle *plcm)
{
	if (!_is_lcm_inited(plcm) || !plcm->drv->resume) return -1;
#if defined(CONFIG_TOUCHSCREEN_COMMON)
	if (!tpd_gesture_flag && plcm->drv->resume_power) plcm->drv->resume_power();
#else
	if (plcm->drv->resume_power) plcm->drv->resume_power();
#endif
	plcm->drv->resume();
	return 0;
}

int disp_lcm_aod(struct disp_lcm_handle *plcm, int enter)
{
	if (!_is_lcm_inited(plcm) || !plcm->drv->aod) return -1;
	plcm->drv->aod(enter);
	return 0;
}

int disp_lcm_is_support_adjust_fps(struct disp_lcm_handle *plcm) { return (_is_lcm_inited(plcm) && plcm->drv->adjust_fps) ? 1 : 0; }
int disp_lcm_adjust_fps(void *cmdq, struct disp_lcm_handle *plcm, int fps)
{
	if (!_is_lcm_inited(plcm) || !plcm->drv->adjust_fps) return -1;
	plcm->drv->adjust_fps(cmdq, fps, plcm->params);
	return 0;
}

int disp_lcm_set_backlight(struct disp_lcm_handle *plcm, void *handle, int level)
{
	if (!_is_lcm_inited(plcm) || !plcm->drv->set_backlight_cmdq) return -1;
	plcm->drv->set_backlight_cmdq(handle, level);
	esd_backlight_level = level;
	return 0;
}

int disp_lcm_ioctl(struct disp_lcm_handle *plcm, enum LCM_IOCTL ioctl, unsigned int arg) { return 0; }
int disp_lcm_is_inited(struct disp_lcm_handle *plcm) { return _is_lcm_inited(plcm) ? plcm->is_inited : 0; }
unsigned int disp_lcm_ATA(struct disp_lcm_handle *plcm) { return (_is_lcm_inited(plcm) && plcm->drv->ata_check) ? plcm->drv->ata_check(NULL) : 0; }

void *disp_lcm_switch_mode(struct disp_lcm_handle *plcm, int mode)
{
	struct LCM_DSI_MODE_SWITCH_CMD *lcm_cmd;
	if (!_is_lcm_inited(plcm) || !plcm->params->dsi.switch_mode_enable || !plcm->drv->switch_mode) return NULL;
	lcm_cmd = (struct LCM_DSI_MODE_SWITCH_CMD *)plcm->drv->switch_mode(mode);
	lcm_cmd->cmd_if = plcm->params->lcm_cmd_if;
	return lcm_cmd;
}

int disp_lcm_is_video_mode(struct disp_lcm_handle *plcm)
{
	if (!_is_lcm_inited(plcm)) return -1;
	if (plcm->params->type == LCM_TYPE_DBI) return 0;
	if (plcm->params->type == LCM_TYPE_DPI) return 1;
	if (plcm->params->type == LCM_TYPE_DSI && plcm->params->dsi.mode != CMD_MODE) return 1;
	return 0;
}

int disp_lcm_set_lcm_cmd(struct disp_lcm_handle *plcm, void *cmdq_handle, unsigned int *lcm_cmd, unsigned int *lcm_count, unsigned int *lcm_value)
{
	if (!_is_lcm_inited(plcm) || !plcm->drv->set_lcm_cmd) return -1;
	plcm->drv->set_lcm_cmd(cmdq_handle, lcm_cmd, lcm_count, lcm_value);
	return 0;
}

int disp_lcm_is_partial_support(struct disp_lcm_handle *plcm) { return (_is_lcm_inited(plcm) && plcm->drv->validate_roi) ? 1 : 0; }

int disp_lcm_validate_roi(struct disp_lcm_handle *plcm, int *x, int *y, int *w, int *h)
{
	if (!_is_lcm_inited(plcm) || !plcm->drv->validate_roi) return -1;
	plcm->drv->validate_roi(x, y, w, h);
	return 0;
}

int disp_lcm_is_arr_support(struct disp_lcm_handle *plcm)
{
	if (!_is_lcm_inited(plcm) || plcm->params->type != LCM_TYPE_DSI || plcm->params->dsi.mode == CMD_MODE) return 0;
	if (plcm->params->dsi.dynamic_fps_levels == 0 || plcm->params->dsi.dynamic_fps_levels > DYNAMIC_FPS_LEVELS) return 0;
	return 1;
}

#ifdef CONFIG_MTK_HIGH_FRAME_RATE
int disp_lcm_is_dynfps_support(struct disp_lcm_handle *plcm)
{
	return (_is_lcm_inited(plcm) && plcm->params->type == LCM_TYPE_DSI && plcm->params->dsi.mode != CMD_MODE &&
		plcm->params->dsi.dfps_enable && plcm->params->dsi.dfps_num >= 2) ? 1 : 0;
}

unsigned int disp_lcm_dynfps_get_def_fps(struct disp_lcm_handle *plcm) { return _is_lcm_inited(plcm) ? plcm->params->dsi.dfps_default_fps : 0; }
unsigned int disp_lcm_dynfps_get_dfps_num(struct disp_lcm_handle *plcm) { return _is_lcm_inited(plcm) ? plcm->params->dsi.dfps_num : 0; }
unsigned int disp_lcm_dynfps_get_def_timing_fps(struct disp_lcm_handle *plcm) { return _is_lcm_inited(plcm) ? plcm->params->dsi.dfps_def_vact_tim_fps : 0; }

bool disp_lcm_need_send_cmd(struct disp_lcm_handle *plcm, unsigned int last_dynfps, unsigned int new_dynfps)
{
	int from_level = -1, to_level = -1;
	if (!_is_lcm_inited(plcm) || !plcm->drv->dfps_send_lcm_cmd || !plcm->drv->dfps_need_send_cmd || !plcm->params->dsi.dfps_enable) return false;

	for (unsigned int j = 0; j < plcm->params->dsi.dfps_num; j++) {
		if (plcm->params->dsi.dfps_params[j].fps == last_dynfps) from_level = plcm->params->dsi.dfps_params[j].level;
		if (plcm->params->dsi.dfps_params[j].fps == new_dynfps) to_level = plcm->params->dsi.dfps_params[j].level;
	}
	return (from_level >= 0 && to_level >= 0) ? plcm->drv->dfps_need_send_cmd(from_level, to_level) : false;
}

void disp_lcm_dynfps_send_cmd(struct disp_lcm_handle *plcm, void *cmdq_handle, unsigned int from_fps, unsigned int to_fps)
{
	unsigned int from_level = 0, to_level = 0;
	if (!_is_lcm_inited(plcm) || !plcm->drv->dfps_send_lcm_cmd || !plcm->params->dsi.dfps_enable) return;

	for (unsigned int j = 0; j < plcm->params->dsi.dfps_num; j++) {
		if (plcm->params->dsi.dfps_params[j].fps == from_fps) from_level = plcm->params->dsi.dfps_params[j].level;
		if (plcm->params->dsi.dfps_params[j].fps == to_fps) to_level = plcm->params->dsi.dfps_params[j].level;
	}
	plcm->drv->dfps_send_lcm_cmd(cmdq_handle, from_level, to_level);
}
#endif
