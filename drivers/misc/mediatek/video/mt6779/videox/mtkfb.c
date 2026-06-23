// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/module.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/kthread.h>
#include <linux/vmalloc.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/suspend.h>
#include <linux/of_fdt.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/dma-buf.h>
#include <linux/uaccess.h>
#include <linux/atomic.h>
#include <linux/io.h>
#include <linux/compat.h>
#include <linux/printk.h>

#include "disp_assert_layer.h"
#include "ion_drv.h"
#include "mt-plat/mtk_boot.h"
#include "ddp_hal.h"
#include "disp_lcm.h"
#include "mtkfb.h"
#include "mtkfb_console.h"
#include "mtkfb_fence.h"
#include "mtkfb_info.h"
#include "ddp_ovl.h"
#include "disp_drv_platform.h"
#include "primary_display.h"
#include "mtk_ovl.h"
#include "disp_helper.h"
#include "compat_mtkfb.h"
#include "disp_dts_gpio.h"
#include "disp_recovery.h"
#include "ddp_clkmgr.h"
#include "ddp_m4u.h"
#include "extd_multi_control.h"
#include "external_display.h"
#include <mt-plat/mtk_ccci_common.h>
#include "ddp_dsi.h"

#ifdef CONFIG_MTK_SMI_EXT
#include "smi_public.h"
#endif

static u32 MTK_FB_XRES;
static u32 MTK_FB_YRES;
static u32 MTK_FB_BPP;
static u32 MTK_FB_PAGES;
static u32 fb_xres_update;
static u32 fb_yres_update;

static int sem_flipping_cnt = 1;
static int sem_early_suspend_cnt = 1;
static int vsync_cnt;
static bool no_update;
static struct disp_session_input_config session_input;

#define ALIGN_TO(x, n)  (((x) + ((n) - 1)) & ~((n) - 1))
#define MTK_FB_XRESV (ALIGN_TO(MTK_FB_XRES, MTK_FB_ALIGNMENT))
#define MTK_FB_YRESV (ALIGN_TO(MTK_FB_YRES, MTK_FB_ALIGNMENT) * MTK_FB_PAGES)
#define MTK_FB_BYPP  ((MTK_FB_BPP + 7) >> 3)
#define MTK_FB_LINE  (ALIGN_TO(MTK_FB_XRES, MTK_FB_ALIGNMENT) * MTK_FB_BYPP)
#define MTK_FB_SIZE  (MTK_FB_LINE * ALIGN_TO(MTK_FB_YRES, MTK_FB_ALIGNMENT))
#define MTK_FB_SIZEV (MTK_FB_LINE * ALIGN_TO(MTK_FB_YRES, MTK_FB_ALIGNMENT) \
	* MTK_FB_PAGES)

struct notifier_block pm_nb;
unsigned long fb_pa;
atomic_t has_pending_update = ATOMIC_INIT(0);
struct fb_overlay_layer video_layerInfo;
bool is_ipoh_bootup;
struct fb_info *mtkfb_fbi;
struct fb_overlay_layer fb_layer_context;
struct mtk_dispif_info dispif_info[MTKFB_MAX_DISPLAY_COUNT];
bool is_early_suspended = false;
unsigned int lcd_fps = 6400;
char mtkfb_lcm_name[256] = { 0 };

#if defined(CONFIG_MTK_DUAL_DISPLAY_SUPPORT) && (CONFIG_MTK_DUAL_DISPLAY_SUPPORT == 2)
struct fb_info *ext_mtkfb_fb;
unsigned long ext_fb_pa;
unsigned int ext_lcd_fps = 6400;
char ext_mtkfb_lcm_name[256] = { 0 };
#endif

DEFINE_SEMAPHORE(sem_flipping);
DEFINE_SEMAPHORE(sem_early_suspend);
DEFINE_SEMAPHORE(sem_overlay_buffer);

static int mtkfb_set_par(struct fb_info *fbi);
static int init_framebuffer(struct fb_info *info);

#ifdef CONFIG_OF
static int _parse_tag_videolfb(void);
#endif
static void mtkfb_late_resume(void);
static void mtkfb_early_suspend(void);

static int mtkfb_open(struct fb_info *info, int user) { return 0; }
static int mtkfb_release(struct fb_info *info, int user) { return 0; }

#if defined(CONFIG_MTK_DUAL_DISPLAY_SUPPORT) && (CONFIG_MTK_DUAL_DISPLAY_SUPPORT == 2)
static int mtkfb1_blank(int blank_mode, struct fb_info *info)
{
	switch (blank_mode) {
	case FB_BLANK_UNBLANK:
	case FB_BLANK_NORMAL:
		external_display_resume(0x20003);
		break;
	case FB_BLANK_POWERDOWN:
		external_display_suspend(0x20003);
		break;
	}
	return 0;
}
#endif

static int mtkfb_blank(int blank_mode, struct fb_info *info)
{
	switch (blank_mode) {
	case FB_BLANK_UNBLANK:
	case FB_BLANK_NORMAL:
		primary_display_set_power_mode(FB_RESUME);
		mtkfb_late_resume();
		break;
	case FB_BLANK_POWERDOWN:
		primary_display_set_power_mode(FB_SUSPEND);
		mtkfb_early_suspend();
		break;
	}
	return 0;
}

int mtkfb_set_backlight_level(unsigned int level)
{
	primary_display_setbacklight(level);
	return 0;
}
EXPORT_SYMBOL(mtkfb_set_backlight_level);

#if defined(CONFIG_MTK_DUAL_DISPLAY_SUPPORT) && (CONFIG_MTK_DUAL_DISPLAY_SUPPORT == 2)
int mtkfb1_set_backlight_level(unsigned int level)
{
	external_display_setbacklight(level);
	return 0;
}
EXPORT_SYMBOL(mtkfb1_set_backlight_level);
#endif

int mtkfb_set_backlight_mode(unsigned int mode)
{
	if (down_interruptible(&sem_flipping)) return -ERESTARTSYS;
	sem_flipping_cnt--;
	if (down_interruptible(&sem_early_suspend)) {
		sem_flipping_cnt++;
		up(&sem_flipping);
		return -ERESTARTSYS;
	}
	sem_early_suspend_cnt--;
	sem_flipping_cnt++;
	sem_early_suspend_cnt++;
	up(&sem_early_suspend);
	up(&sem_flipping);
	return 0;
}
EXPORT_SYMBOL(mtkfb_set_backlight_mode);

int mtkfb_set_backlight_pwm(int div)
{
	if (down_interruptible(&sem_flipping)) return -ERESTARTSYS;
	sem_flipping_cnt--;
	if (down_interruptible(&sem_early_suspend)) {
		sem_flipping_cnt++;
		up(&sem_flipping);
		return -ERESTARTSYS;
	}
	sem_early_suspend_cnt--;
	sem_flipping_cnt++;
	sem_early_suspend_cnt++;
	up(&sem_early_suspend);
	up(&sem_flipping);
	return 0;
}
EXPORT_SYMBOL(mtkfb_set_backlight_pwm);

int mtkfb_get_backlight_pwm(int div, unsigned int *freq) { return 0; }
EXPORT_SYMBOL(mtkfb_get_backlight_pwm);

void mtkfb_waitVsync(void)
{
	if (primary_display_is_sleepd()) {
		msleep(20);
		return;
	}
	vsync_cnt++;
	primary_display_wait_for_vsync(NULL);
	vsync_cnt--;
}
EXPORT_SYMBOL(mtkfb_waitVsync);

static int _convert_fb_layer_to_disp_input(struct fb_overlay_layer *src, struct disp_input_config *dst)
{
	dst->layer_id = src->layer_id;
	dst->dirty_roi_num = 0;
	if (!src->layer_enable) {
		dst->layer_enable = 0;
		return 0;
	}

	switch (src->src_fmt) {
	case MTK_FB_FORMAT_YUV422: dst->src_fmt = DISP_FORMAT_YUV422; break;
	case MTK_FB_FORMAT_RGB565: dst->src_fmt = DISP_FORMAT_RGB565; break;
	case MTK_FB_FORMAT_RGB888: dst->src_fmt = DISP_FORMAT_RGB888; break;
	case MTK_FB_FORMAT_BGR888: dst->src_fmt = DISP_FORMAT_BGR888; break;
	case MTK_FB_FORMAT_ARGB8888: dst->src_fmt = DISP_FORMAT_ARGB8888; break;
	case MTK_FB_FORMAT_ABGR8888: dst->src_fmt = DISP_FORMAT_ABGR8888; break;
	case MTK_FB_FORMAT_BGRA8888: dst->src_fmt = DISP_FORMAT_BGRA8888; break;
	case MTK_FB_FORMAT_XRGB8888: dst->src_fmt = DISP_FORMAT_XRGB8888; break;
	case MTK_FB_FORMAT_XBGR8888: dst->src_fmt = DISP_FORMAT_XBGR8888; break;
	case MTK_FB_FORMAT_UYVY: dst->src_fmt = DISP_FORMAT_UYVY; break;
	default: return -1;
	}

	dst->src_base_addr = src->src_base_addr;
	dst->security = src->security;
	dst->src_phy_addr = src->src_phy_addr;
	dst->isTdshp = src->isTdshp;
	dst->next_buff_idx = src->next_buff_idx;
	dst->identity = src->identity;
	dst->connected_type = src->connected_type;
	dst->alpha = src->alpha;
	dst->alpha_enable = (src->src_fmt == MTK_FB_FORMAT_ARGB8888 || src->src_fmt == MTK_FB_FORMAT_ABGR8888);

	dst->src_offset_x = src->src_offset_x;
	dst->src_offset_y = src->src_offset_y;
	dst->src_width = src->src_width;
	dst->src_height = src->src_height;
	dst->tgt_offset_x = src->tgt_offset_x;
	dst->tgt_offset_y = src->tgt_offset_y;
	dst->tgt_width = min_t(unsigned int, src->tgt_width, dst->src_width);
	dst->tgt_height = min_t(unsigned int, src->tgt_height, dst->src_height);
	dst->src_pitch = src->src_pitch;
	dst->src_color_key = src->src_color_key;
	dst->src_use_color_key = src->src_use_color_key;
	dst->layer_enable = src->layer_enable;
	dst->ext_sel_layer = -1;

	return 0;
}

static int mtkfb_pan_display_impl(struct fb_var_screeninfo *var, struct fb_info *info)
{
	UINT32 offset, paStart;
	char *vaStart;
	int ret = 0;
	struct disp_session_input_config *session_input;
	struct disp_input_config *input;

	if (no_update) return ret;

	info->var.yoffset = var->yoffset;
	offset = var->yoffset * info->fix.line_length;
	paStart = fb_pa + offset;
	vaStart = info->screen_base + offset;

	session_input = kzalloc(sizeof(*session_input), GFP_KERNEL);
	if (!session_input) return -ENOMEM;

	session_input->config_layer_num = 0;
	input = &session_input->config[0];
	input->layer_id = primary_display_get_option("FB_LAYER");
	input->src_phy_addr = (void *)((unsigned long)paStart);
	input->src_base_addr = (void *)((unsigned long)vaStart);
	input->layer_enable = 1;
	input->src_width = var->xres;
	input->src_height = var->yres;
	input->tgt_width = var->xres;
	input->tgt_height = var->yres;
	input->ext_sel_layer = -1;

	switch (var->bits_per_pixel) {
	case 16: input->src_fmt = DISP_FORMAT_RGB565; break;
	case 24: input->src_fmt = DISP_FORMAT_RGB888; break;
	case 32: input->src_fmt = (var->blue.offset == 0) ? DISP_FORMAT_BGRA8888 : DISP_FORMAT_RGBX8888; break;
	default: kfree(session_input); return -EINVAL;
	}

	input->alpha_enable = false;
	input->alpha = 0xFF;
	input->next_buff_idx = -1;
	input->src_pitch = ALIGN_TO(var->xres, MTK_FB_ALIGNMENT);
	session_input->config_layer_num++;
	session_input->setter = SESSION_USER_PANDISP;

	if (!is_DAL_Enabled()) {
		session_input->config[1].layer_id = primary_display_get_option("ASSERT_LAYER");
		session_input->config[1].next_buff_idx = -1;
		session_input->config[1].layer_enable = 0;
		session_input->config_layer_num++;
	}

	ret = primary_display_config_input_multiple(session_input);
	ret = primary_display_trigger(true, NULL, 0);

	kfree(session_input);
	return ret;
}

static void set_fb_fix(struct mtkfb_device *fbdev)
{
	struct fb_info *fbi = fbdev->fb_info;
	struct fb_fix_screeninfo *fix = &fbi->fix;
	struct fb_var_screeninfo *var = &fbi->var;

	strncpy(fix->id, MTKFB_DRIVER, sizeof(fix->id));
	fix->type = FB_TYPE_PACKED_PIXELS;
	fix->visual = (var->bits_per_pixel > 8) ? FB_VISUAL_TRUECOLOR : FB_VISUAL_PSEUDOCOLOR;
	fix->accel = FB_ACCEL_NONE;
	fix->line_length = ALIGN_TO(var->xres_virtual, MTK_FB_ALIGNMENT) * var->bits_per_pixel / 8;
	fix->smem_len = fbdev->fb_size_in_byte;
	fix->smem_start = fbdev->fb_pa_base;
	fix->xpanstep = 0;
	fix->ypanstep = 1;
	fbi->fbops->fb_fillrect = cfb_fillrect;
	fbi->fbops->fb_copyarea = cfb_copyarea;
	fbi->fbops->fb_imageblit = cfb_imageblit;
}

static int mtkfb_check_var(struct fb_var_screeninfo *var, struct fb_info *fbi)
{
	unsigned int bpp = var->bits_per_pixel;
	unsigned long max_frame_size, line_size;
	struct mtkfb_device *fbdev = (struct mtkfb_device *)fbi->par;

	if (bpp != 16 && bpp != 24 && bpp != 32) return -EINVAL;

	switch (var->rotate) {
	case 0: case 180:
		var->xres = MTK_FB_XRES; var->yres = MTK_FB_YRES; break;
	case 90: case 270:
		var->xres = MTK_FB_YRES; var->yres = MTK_FB_XRES; break;
	default: return -EINVAL;
	}

	if (var->xres_virtual < var->xres) var->xres_virtual = var->xres;
	if (var->yres_virtual < var->yres) var->yres_virtual = var->yres;

	max_frame_size = fbdev->fb_size_in_byte;
	line_size = var->xres_virtual * bpp / 8;

	if (line_size * var->yres_virtual > max_frame_size) {
		line_size = max_frame_size / var->yres_virtual;
		var->xres_virtual = line_size * 8 / bpp;
		if (var->xres_virtual < var->xres) {
			var->xres_virtual = var->xres;
			line_size = var->xres * bpp / 8;
			var->yres_virtual = max_frame_size / line_size;
		}
	}

	if (var->xres + var->xoffset > var->xres_virtual) var->xoffset = var->xres_virtual - var->xres;
	if (var->yres + var->yoffset > var->yres_virtual) var->yoffset = var->yres_virtual - var->yres;

	if (bpp == 16) {
		var->red.offset = 11; var->red.length = 5;
		var->green.offset = 5; var->green.length = 6;
		var->blue.offset = 0; var->blue.length = 5;
		var->transp.length = 0;
	} else if (bpp == 24 || bpp == 32) {
		var->red.length = var->green.length = var->blue.length = 8;
		var->transp.length = (bpp == 32) ? 8 : 0;
	}

	var->red.msb_right = var->green.msb_right = var->blue.msb_right = var->transp.msb_right = 0;
	no_update = !!(var->activate & FB_ACTIVATE_NO_UPDATE);
	var->activate = FB_ACTIVATE_NOW;
	var->vmode = FB_VMODE_NONINTERLACED;

	return 0;
}

static int mtkfb_set_par(struct fb_info *fbi)
{
	struct fb_var_screeninfo *var = &fbi->var;
	struct mtkfb_device *fbdev = (struct mtkfb_device *)fbi->par;
	struct fb_overlay_layer fb_layer = {0};
	struct disp_session_input_config *session_input;
	struct disp_input_config *input;

	switch (var->bits_per_pixel) {
	case 16:
		fb_layer.src_fmt = MTK_FB_FORMAT_RGB565;
		fb_layer.src_use_color_key = 1;
		fb_layer.src_color_key = 0xFF000000;
		break;
	case 24:
		fb_layer.src_use_color_key = 1;
		fb_layer.src_fmt = (var->blue.offset == 0) ? MTK_FB_FORMAT_RGB888 : MTK_FB_FORMAT_BGR888;
		fb_layer.src_color_key = 0xFF000000;
		break;
	case 32:
		fb_layer.src_use_color_key = 0;
		fb_layer.src_fmt = (var->blue.offset == 0) ? MTK_FB_FORMAT_ARGB8888 : MTK_FB_FORMAT_BGRA8888;
		break;
	default:
		return -EINVAL;
	}

	set_fb_fix(fbdev);

	fb_layer.layer_id = primary_display_get_option("FB_LAYER");
	fb_layer.layer_enable = 1;
	fb_layer.src_base_addr = (void *)((unsigned long)fbdev->fb_va_base + var->yoffset * fbi->fix.line_length);
	fb_layer.src_phy_addr = (void *)(fb_pa + var->yoffset * fbi->fix.line_length);
	fb_layer.src_pitch = ALIGN_TO(var->xres, MTK_FB_ALIGNMENT);
	fb_layer.src_width = fb_layer.tgt_width = var->xres;
	fb_layer.src_height = fb_layer.tgt_height = var->yres;
	fb_layer.alpha = 0xff;
	fb_layer.layer_rotation = MTK_FB_ORIENTATION_0;
	fb_layer.layer_type = LAYER_2D;

	session_input = kzalloc(sizeof(*session_input), GFP_KERNEL);
	if (!session_input) return -ENOMEM;

	if (!is_DAL_Enabled()) {
		input = &session_input->config[session_input->config_layer_num++];
		input->layer_id = primary_display_get_option("ASSERT_LAYER");
		input->layer_enable = 0;
	}

	input = &session_input->config[session_input->config_layer_num++];
	_convert_fb_layer_to_disp_input(&fb_layer, input);
	session_input->setter = SESSION_USER_INVALID;
	primary_display_config_input_multiple(session_input);
	kfree(session_input);

	memcpy(&fb_layer_context, &fb_layer, sizeof(fb_layer));
	return 0;
}

static int mtkfb_soft_cursor(struct fb_info *info, struct fb_cursor *cursor) { return 0; }
unsigned int mtkfb_fm_auto_test(void) { return 0; }

static int mtkfb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int r = 0;

	switch (cmd) {
	case MTKFB_GET_FRAMEBUFFER_MVA:
		return copy_to_user(argp, &fb_pa, sizeof(fb_pa)) ? -EFAULT : 0;

	case MTKFB_GET_DISPLAY_IF_INFORMATION:
	{
		int displayid = 0;
		if (copy_from_user(&displayid, argp, sizeof(displayid))) return -EFAULT;

		if (displayid == 0) {
			dispif_info[0].displayWidth = primary_display_get_width();
			dispif_info[0].displayHeight = primary_display_get_height();
			dispif_info[0].lcmOriginalWidth = primary_display_get_original_width();
			dispif_info[0].lcmOriginalHeight = primary_display_get_original_height();
			dispif_info[0].displayMode = primary_display_is_video_mode() ? 0 : 1;
		} else {
			return -EFAULT;
		}
		return copy_to_user(argp, &dispif_info[0], sizeof(struct mtk_dispif_info)) ? -EFAULT : 0;
	}
	case MTKFB_POWEROFF:
		if (!primary_display_is_sleepd()) {
			primary_display_suspend();
			is_early_suspended = true;
		}
		return 0;
	case MTKFB_POWERON:
		if (!primary_display_is_alive()) {
			primary_display_resume();
			is_early_suspended = false;
		}
		return 0;
	case MTKFB_GET_POWERSTATE:
	{
		int power_state = !primary_display_is_sleepd();
		return copy_to_user(argp, &power_state, sizeof(power_state)) ? -EFAULT : 0;
	}
	case MTKFB_CONFIG_IMMEDIATE_UPDATE:
		if (down_interruptible(&sem_early_suspend)) return -ERESTARTSYS;
		sem_early_suspend_cnt--;
		up(&sem_early_suspend);
		return 0;
	case MTKFB_SET_OVERLAY_LAYER:
	{
		struct fb_overlay_layer layerInfo;
		struct disp_input_config *input;

		if (copy_from_user(&layerInfo, argp, sizeof(layerInfo))) return -EFAULT;
		if (primary_display_is_sleepd()) return MTKFB_ERROR_IS_EARLY_SUSPEND;

		memset(&session_input, 0, sizeof(session_input));
		input = &session_input.config[session_input.config_layer_num++];
		session_input.setter = SESSION_USER_PANDISP;
		_convert_fb_layer_to_disp_input(&layerInfo, input);
		primary_display_config_input_multiple(&session_input);
		primary_display_trigger(1, NULL, 0);
		return 0;
	}
	case MTKFB_SET_VIDEO_LAYERS:
	{
		struct fb_overlay_layer layerInfo[VIDEO_LAYER_COUNT];
		int i;

		if (copy_from_user(layerInfo, argp, sizeof(layerInfo))) return -EFAULT;

		memset(&session_input, 0, sizeof(session_input));
		for (i = 0; i < VIDEO_LAYER_COUNT; ++i) {
			if (layerInfo[i].layer_id >= TOTAL_OVL_LAYER_NUM) continue;
			_convert_fb_layer_to_disp_input(&layerInfo[i], &session_input.config[session_input.config_layer_num++]);
		}
		session_input.setter = SESSION_USER_PANDISP;
		primary_display_config_input_multiple(&session_input);
		primary_display_trigger(1, NULL, 0);
		return 0;
	}
	case MTKFB_TRIG_OVERLAY_OUT:
		primary_display_trigger(1, NULL, 0);
		return 0;
	case MTKFB_META_RESTORE_SCREEN:
	{
		struct fb_var_screeninfo var;
		if (copy_from_user(&var, argp, sizeof(var))) return -EFAULT;

		if (var.xres > MTK_FB_XRES || var.yres > MTK_FB_YRES ||
			var.xres_virtual > MTK_FB_XRESV || var.yres_virtual > MTK_FB_YRESV ||
			var.xoffset > MTK_FB_XRES || var.yoffset > MTK_FB_YRESV * (MTK_FB_PAGES - 1))
			return -EINVAL;

		info->var.yoffset = var.yoffset;
		if (info->var.yres + info->var.yoffset > info->var.yres_virtual)
			info->var.yoffset = info->var.yres_virtual - info->var.yres;

		init_framebuffer(info);
		return mtkfb_pan_display_impl(&var, info);
	}
	case MTKFB_AEE_LAYER_EXIST:
	{
		int dal_en = is_DAL_Enabled();
		return copy_to_user(argp, &dal_en, sizeof(dal_en)) ? -EFAULT : 0;
	}
	default:
		return -EINVAL;
	}

	return r;
}

#ifdef CONFIG_COMPAT
static void compat_convert(struct compat_fb_overlay_layer *compat_info, struct fb_overlay_layer *info)
{
	info->layer_id = compat_info->layer_id;
	info->layer_enable = compat_info->layer_enable;
	info->src_base_addr = (void *)((unsigned long)compat_info->src_base_addr);
	info->src_phy_addr = (void *)((unsigned long)compat_info->src_phy_addr);
	info->src_direct_link = compat_info->src_direct_link;
	info->src_fmt = compat_info->src_fmt;
	info->src_use_color_key = compat_info->src_use_color_key;
	info->src_color_key = compat_info->src_color_key;
	info->src_pitch = compat_info->src_pitch;
	info->src_offset_x = compat_info->src_offset_x;
	info->src_offset_y = compat_info->src_offset_y;
	info->src_width = compat_info->src_width;
	info->src_height = compat_info->src_height;
	info->tgt_offset_x = compat_info->tgt_offset_x;
	info->tgt_offset_y = compat_info->tgt_offset_y;
	info->tgt_width = compat_info->tgt_width;
	info->tgt_height = compat_info->tgt_height;
	info->layer_rotation = compat_info->layer_rotation;
	info->layer_type = compat_info->layer_type;
	info->video_rotation = compat_info->video_rotation;
	info->isTdshp = compat_info->isTdshp;
	info->next_buff_idx = compat_info->next_buff_idx;
	info->identity = compat_info->identity;
	info->connected_type = compat_info->connected_type;
	info->security = compat_info->security;
	info->alpha_enable = compat_info->alpha_enable;
	info->alpha = compat_info->alpha;
	info->fence_fd = compat_info->fence_fd;
	info->ion_fd = compat_info->ion_fd;
}

static int mtkfb_compat_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	struct fb_overlay_layer layerInfo;

	switch (cmd) {
	case COMPAT_MTKFB_GET_FRAMEBUFFER_MVA:
	{
		__u32 data = (__u32) fb_pa;
		return put_user(data, (compat_uint_t __user *)compat_ptr(arg)) ? -EFAULT : 0;
	}
	case COMPAT_MTKFB_GET_DISPLAY_IF_INFORMATION:
	{
		compat_uint_t displayid = 0;
		if (get_user(displayid, (compat_uint_t __user *)compat_ptr(arg))) return -EFAULT;
		if (displayid != 0) return -EFAULT;

		dispif_info[0].displayWidth = primary_display_get_width();
		dispif_info[0].displayHeight = primary_display_get_height();
		dispif_info[0].lcmOriginalWidth = primary_display_get_original_width();
		dispif_info[0].lcmOriginalHeight = primary_display_get_original_height();
		dispif_info[0].displayMode = primary_display_is_video_mode() ? 0 : 1;

		return copy_to_user(compat_ptr(arg), &dispif_info[0], sizeof(struct compat_mtk_dispif_info)) ? -EFAULT : 0;
	}
	case COMPAT_MTKFB_POWEROFF: return mtkfb_ioctl(info, MTKFB_POWEROFF, arg);
	case COMPAT_MTKFB_POWERON: return mtkfb_ioctl(info, MTKFB_POWERON, arg);
	case COMPAT_MTKFB_GET_POWERSTATE:
	{
		int power_state = !primary_display_is_sleepd();
		return put_user(power_state, (compat_uint_t __user *)compat_ptr(arg)) ? -EFAULT : 0;
	}
	case COMPAT_MTKFB_TRIG_OVERLAY_OUT: return mtkfb_ioctl(info, MTKFB_TRIG_OVERLAY_OUT, (unsigned long)compat_ptr(arg));
	case COMPAT_MTKFB_META_RESTORE_SCREEN: return mtkfb_ioctl(info, MTKFB_META_RESTORE_SCREEN, (unsigned long)compat_ptr(arg));
	case COMPAT_MTKFB_SET_OVERLAY_LAYER:
	{
		struct compat_fb_overlay_layer compat_layerInfo;
		if (copy_from_user(&compat_layerInfo, compat_ptr(arg), sizeof(compat_layerInfo))) return -EFAULT;
		if (primary_display_is_sleepd()) return MTKFB_ERROR_IS_EARLY_SUSPEND;

		compat_convert(&compat_layerInfo, &layerInfo);
		memset(&session_input, 0, sizeof(session_input));
		_convert_fb_layer_to_disp_input(&layerInfo, &session_input.config[session_input.config_layer_num++]);
		session_input.setter = SESSION_USER_PANDISP;
		primary_display_config_input_multiple(&session_input);
		return 0;
	}
	case COMPAT_MTKFB_SET_VIDEO_LAYERS:
	{
		struct compat_fb_overlay_layer compat_layerInfo[VIDEO_LAYER_COUNT];
		int i;

		if (copy_from_user(compat_layerInfo, compat_ptr(arg), sizeof(compat_layerInfo))) return -EFAULT;

		memset(&session_input, 0, sizeof(session_input));
		for (i = 0; i < VIDEO_LAYER_COUNT; ++i) {
			compat_convert(&compat_layerInfo[i], &layerInfo);
			_convert_fb_layer_to_disp_input(&layerInfo, &session_input.config[session_input.config_layer_num++]);
		}
		session_input.setter = SESSION_USER_PANDISP;
		primary_display_config_input_multiple(&session_input);
		return 0;
	}
	case COMPAT_MTKFB_AEE_LAYER_EXIST:
	{
		int dal_en = is_DAL_Enabled();
		return put_user(dal_en, (compat_ulong_t __user *)compat_ptr(arg)) ? -EFAULT : 0;
	}
	default:
		return mtkfb_ioctl(info, cmd, (unsigned long)compat_ptr(arg));
	}
}
#endif

static int mtkfb_pan_display_proxy(struct fb_var_screeninfo *var, struct fb_info *info)
{
	return mtkfb_pan_display_impl(var, info);
}

static struct fb_ops mtkfb_ops = {
	.owner = THIS_MODULE,
	.fb_open = mtkfb_open,
	.fb_release = mtkfb_release,
	.fb_pan_display = mtkfb_pan_display_proxy,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_cursor = mtkfb_soft_cursor,
	.fb_check_var = mtkfb_check_var,
	.fb_set_par = mtkfb_set_par,
	.fb_ioctl = mtkfb_ioctl,
#ifdef CONFIG_COMPAT
	.fb_compat_ioctl = mtkfb_compat_ioctl,
#endif
	.fb_blank = mtkfb_blank,
};

static int mtkfb_fbinfo_init(struct fb_info *info)
{
	struct mtkfb_device *fbdev = (struct mtkfb_device *)info->par;
	struct fb_var_screeninfo var;
	int r = 0;

	info->fbops = &mtkfb_ops;
	info->flags = FBINFO_FLAG_DEFAULT;
	info->screen_base = (char *)fbdev->fb_va_base;
	info->screen_size = fbdev->fb_size_in_byte;
	info->pseudo_palette = fbdev->pseudo_palette;

	r = fb_alloc_cmap(&info->cmap, 32, 0);
	if (r != 0) return r;

	memset(&var, 0, sizeof(var));
	var.xres = MTK_FB_XRES;
	var.yres = MTK_FB_YRES;
	var.xres_virtual = MTK_FB_XRESV;
	var.yres_virtual = MTK_FB_YRESV;
	var.bits_per_pixel = 32;

	var.transp.offset = 24; var.transp.length = 8;
	var.red.offset = 0; var.red.length = 8;
	var.green.offset = 8; var.green.length = 8;
	var.blue.offset = 16; var.blue.length = 8;

	var.width = DISP_GetActiveWidth();
	var.height = DISP_GetActiveHeight();
	var.activate = FB_ACTIVATE_NOW;

	r = mtkfb_check_var(&var, info);
	if (r) return r;

	info->var = var;
	r = mtkfb_set_par(info);

	return r;
}

static void mtkfb_fbinfo_cleanup(struct mtkfb_device *fbdev)
{
	fb_dealloc_cmap(&fbdev->fb_info->cmap);
}

static int init_framebuffer(struct fb_info *info)
{
	void *buffer = info->screen_base + info->var.yoffset * info->fix.line_length;
	int size = info->var.xres_virtual * info->var.yres * info->var.bits_per_pixel / 8;
	memset_io(buffer, 0, size);
	return 0;
}

static void mtkfb_free_resources(struct mtkfb_device *fbdev, int state)
{
	switch (state) {
	case 2: unregister_framebuffer(fbdev->fb_info);
	case 1: mtkfb_fbinfo_cleanup(fbdev);
		dev_set_drvdata(fbdev->dev, NULL);
		framebuffer_release(fbdev->fb_info);
	}
}

void disp_get_fb_address(unsigned long *fbVirAddr, unsigned long *fbPhysAddr)
{
	struct mtkfb_device *fbdev = (struct mtkfb_device *)mtkfb_fbi->par;
	int fb_size = mtkfb_fbi->var.yoffset * mtkfb_fbi->fix.line_length;
	*fbVirAddr = (unsigned long)fbdev->fb_va_base + fb_size;
	*fbPhysAddr = (unsigned long)fbdev->fb_pa_base + fb_size;
}

#ifdef CONFIG_OF
unsigned int islcmconnected, is_lcm_inited, vramsize;
phys_addr_t fb_base;
static int is_videofb_parse_done;

static int __parse_tag_videolfb_extra(struct device_node *node)
{
	void *prop;
	u32 fb_base_h, fb_base_l;
	unsigned long size = 0;

	prop = (void *)of_get_property(node, "atag,videolfb-fb_base_h", NULL);
	if (!prop) return -1;
	fb_base_h = of_read_number(prop, 1);

	prop = (void *)of_get_property(node, "atag,videolfb-fb_base_l", NULL);
	if (!prop) return -1;
	fb_base_l = of_read_number(prop, 1);
	fb_base = ((u64) fb_base_h << 32) | (u64) fb_base_l;

	prop = (void *)of_get_property(node, "atag,videolfb-islcmfound", NULL);
	if (!prop) return -1;
	islcmconnected = of_read_number(prop, 1);

	prop = (void *)of_get_property(node, "atag,videolfb-fps", NULL);
	lcd_fps = prop ? of_read_number(prop, 1) : 6400;
	if (lcd_fps == 0) lcd_fps = 6400;

	prop = (void *)of_get_property(node, "atag,videolfb-vramSize", NULL);
	if (!prop) return -1;
	vramsize = of_read_number(prop, 1);

	prop = (void *)of_get_property(node, "atag,videolfb-lcmname", (int *)&size);
	if (!prop || size >= sizeof(mtkfb_lcm_name)) return -1;

	memset((void *)mtkfb_lcm_name, 0, sizeof(mtkfb_lcm_name));
	strncpy((char *)mtkfb_lcm_name, prop, sizeof(mtkfb_lcm_name));
	mtkfb_lcm_name[size] = '\0';
	return 0;
}

static int _parse_tag_videolfb(void)
{
	struct device_node *chosen_node;

	if (is_videofb_parse_done) return 0;

	chosen_node = of_find_node_by_path("/chosen");
	if (!chosen_node) chosen_node = of_find_node_by_path("/chosen@0");

	if (chosen_node) {
		if (__parse_tag_videolfb_extra(chosen_node) == 0) {
			is_videofb_parse_done = 1;
			return 0;
		}
	}
	return -1;
}

phys_addr_t mtkfb_get_fb_base(void)
{
	_parse_tag_videolfb();
	return fb_base;
}
EXPORT_SYMBOL(mtkfb_get_fb_base);

size_t mtkfb_get_fb_size(void)
{
	_parse_tag_videolfb();
	return vramsize;
}
EXPORT_SYMBOL(mtkfb_get_fb_size);
#endif

char *mtkfb_find_lcm_driver(void)
{
#ifdef CONFIG_OF
	_parse_tag_videolfb();
#endif
	return mtkfb_lcm_name;
}

static int mtkfb_probe(struct platform_device *pdev)
{
	struct mtkfb_device *fbdev;
	struct fb_info *fbi;
	int r = 0;

#ifdef CONFIG_MTK_SMI_EXT
	if (!smi_mm_first_get()) return -EPROBE_DEFER;
#endif
#ifdef CONFIG_OF
	_parse_tag_videolfb();
#endif

	disp_dts_gpio_init_repo(pdev);

	fbi = framebuffer_alloc(sizeof(struct mtkfb_device), &(pdev->dev));
	if (!fbi) return -ENOMEM;
	mtkfb_fbi = fbi;

	fbdev = (struct mtkfb_device *)fbi->par;
	fbdev->fb_info = fbi;
	fbdev->dev = &(pdev->dev);
	dev_set_drvdata(&(pdev->dev), fbdev);

	disp_hal_allocate_framebuffer(fb_base, (fb_base + vramsize - 1), (unsigned long *)(&fbdev->fb_va_base), &fb_pa);
	fbdev->fb_pa_base = fb_base;

	primary_display_set_frame_buffer_address((unsigned long)(fbdev->fb_va_base), fb_pa, fb_base);
	primary_display_init(mtkfb_find_lcm_driver(), lcd_fps, is_lcm_inited);

	MTK_FB_XRES = DISP_GetScreenWidth();
	MTK_FB_YRES = DISP_GetScreenHeight();
	MTK_FB_BPP = DISP_GetScreenBpp();
	MTK_FB_PAGES = DISP_GetPages();
	fbdev->fb_size_in_byte = MTK_FB_SIZEV;

	if (!fbdev->fb_va_base) {
		r = -ENOMEM;
		goto cleanup;
	}

	r = mtkfb_fbinfo_init(fbi);
	if (r) goto cleanup;

	if (disp_helper_get_stage() == DISP_HELPER_STAGE_NORMAL) {
		unsigned long fbVA = (unsigned long)(fbdev->fb_va_base) + DISP_GetFBRamSize();
		unsigned long fbPA = fb_pa + DISP_GetFBRamSize();
		DAL_Init(fbVA, fbPA);
	}

	r = register_framebuffer(fbi);
	if (r) goto cleanup;

#ifdef MTK_FB_ION_SUPPORT
	ion_drv_create_FB_heap(mtkfb_get_fb_base(), mtkfb_get_fb_size());
#endif
	fbdev->state = MTKFB_ACTIVE;
	return 0;

cleanup:
	mtkfb_free_resources(fbdev, (r == -ENOMEM) ? 1 : 2);
	return r;
}

static int mtkfb_remove(struct platform_device *pdev)
{
	struct mtkfb_device *fbdev = dev_get_drvdata(&pdev->dev);
	fbdev->state = MTKFB_DISABLED;
	mtkfb_free_resources(fbdev, 2);
	return 0;
}

static int mtkfb_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	ovl2mem_wait_done();
	return 0;
}

static int mtkfb_resume(struct platform_device *pdev) { return 0; }

static void mtkfb_shutdown(struct platform_device *pdev)
{
	if (!lcd_fps) msleep(30);
	else msleep(2 * 100000 / lcd_fps);

	if (!primary_display_is_sleepd()) {
		primary_display_set_power_mode(FB_SUSPEND);
		primary_display_suspend();
	}
}

bool mtkfb_is_suspend(void) { return primary_display_is_sleepd(); }
EXPORT_SYMBOL(mtkfb_is_suspend);

int mtkfb_ipoh_restore(struct notifier_block *nb, unsigned long val, void *ign)
{
	if (val == PM_RESTORE_PREPARE) primary_display_ipoh_restore();
	return NOTIFY_OK;
}

int mtkfb_ipo_init(void)
{
	pm_nb.notifier_call = mtkfb_ipoh_restore;
	pm_nb.priority = 0;
	register_pm_notifier(&pm_nb);
	return 0;
}

static void mtkfb_early_suspend(void)
{
	if (disp_helper_get_stage() == DISP_HELPER_STAGE_NORMAL)
		primary_display_suspend();
}

static void mtkfb_late_resume(void)
{
	if (disp_helper_get_stage() == DISP_HELPER_STAGE_NORMAL)
		primary_display_resume();
}

#ifdef CONFIG_PM
int mtkfb_pm_suspend(struct device *device) { return mtkfb_suspend(to_platform_device(device), PMSG_SUSPEND); }
int mtkfb_pm_resume(struct device *device) { return mtkfb_resume(to_platform_device(device)); }
int mtkfb_pm_freeze(struct device *device) { primary_display_esd_check_enable(0); return 0; }
int mtkfb_pm_restore_noirq(struct device *device)
{
	is_ipoh_bootup = true;
	dpmgr_path_power_on(primary_get_dpmgr_handle(), CMDQ_DISABLE);
	return 0;
}
#else
#define mtkfb_pm_suspend NULL
#define mtkfb_pm_resume  NULL
#define mtkfb_pm_restore_noirq NULL
#define mtkfb_pm_freeze NULL
#endif

static const struct dev_pm_ops mtkfb_pm_ops = {
	.suspend = mtkfb_pm_suspend,
	.resume = mtkfb_pm_resume,
	.freeze = mtkfb_pm_freeze,
	.thaw = mtkfb_pm_resume,
	.poweroff = mtkfb_pm_suspend,
	.restore = mtkfb_pm_resume,
	.restore_noirq = mtkfb_pm_restore_noirq,
};

static const struct of_device_id mtkfb_of_ids[] = {
	{.compatible = "mediatek,MTKFB",},
	{}
};

static struct platform_driver mtkfb_driver = {
	.probe = mtkfb_probe,
	.remove = mtkfb_remove,
	.suspend = mtkfb_suspend,
	.resume = mtkfb_resume,
	.shutdown = mtkfb_shutdown,
	.driver = {
		.name = MTKFB_DRIVER,
#ifdef CONFIG_PM
		.pm = &mtkfb_pm_ops,
#endif
		.of_match_table = mtkfb_of_ids,
	},
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend mtkfb_early_suspend_handler = {
	.level = EARLY_SUSPEND_LEVEL_DISABLE_FB,
	.suspend = mtkfb_early_suspend,
	.resume = mtkfb_late_resume,
};
#endif

int __init mtkfb_init(void)
{
	if (platform_driver_register(&mtkfb_driver)) return -ENODEV;
#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&mtkfb_early_suspend_handler);
#endif
	PanelMaster_Init();
	mtkfb_ipo_init();
	return 0;
}

static void __exit mtkfb_cleanup(void)
{
	platform_driver_unregister(&mtkfb_driver);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&mtkfb_early_suspend_handler);
#endif
	PanelMaster_Deinit();
}

module_init(mtkfb_init);
module_exit(mtkfb_cleanup);

MODULE_DESCRIPTION("MEDIATEK framebuffer driver");
MODULE_LICENSE("GPL");
