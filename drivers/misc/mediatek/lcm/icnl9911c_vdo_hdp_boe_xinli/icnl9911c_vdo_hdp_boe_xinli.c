/*
 * Copyright (C) 2015 MediaTek Inc.
 * Copyright (C) 2021 XiaoMi, Inc.
 * Copyright (C) 2022 TheKit
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#define LOG_TAG "LCM"

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"
#include "lcm_define.h"
#include "disp_dts_gpio.h"
#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>

#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_pm_ldo.h>
#include <mach/mt_gpio.h>

#ifndef CONFIG_FPGA_EARLY_PORTING
#include <cust_gpio_usage.h>
#include <cust_i2c.h>
#endif

#endif
#endif

static const unsigned int BL_MIN_LEVEL = 20;
static struct LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))
#define MDELAY(n)		(lcm_util.mdelay(n))
#define UDELAY(n)		(lcm_util.udelay(n))



#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
	lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
	lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) \
	lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
	lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
/* #include <linux/jiffies.h> */
/* #include <linux/delay.h> */
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
extern int tps65132_write_bytes(unsigned char addr, unsigned char value);
static int lcd_bl_en;
#endif

/* static unsigned char lcd_id_pins_value = 0xFF; */
static const unsigned char LCD_MODULE_ID = 0x01;
#define LCM_DSI_CMD_MODE                                    0
#define FRAME_WIDTH                                     (720)
#define FRAME_HEIGHT                                    (1600)

/* physical size in um */
#define LCM_PHYSICAL_WIDTH (68040)
#define LCM_PHYSICAL_HEIGHT (151200)
#define LCM_DENSITY	(320)


#define GPIO_LCD_BIAS_ENP_PIN 168
#define GPIO_65132_ENP GPIO_LCD_BIAS_ENP_PIN
#define GPIO_LCD_BIAS_ENN_PIN 173
#define GPIO_65132_ENN GPIO_LCD_BIAS_ENN_PIN

#define REGFLAG_DELAY		0xFFFC
#define REGFLAG_UDELAY	0xFFFB
#define REGFLAG_END_OF_TABLE	0xFFFD
#define REGFLAG_RESET_LOW	0xFFFE
#define REGFLAG_RESET_HIGH	0xFFFF

static struct LCM_DSI_MODE_SWITCH_CMD lcm_switch_mode_cmd;
extern char *saved_command_line;

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#if defined(CONFIG_TOUCHSCREEN_COMMON)
extern bool tpd_gesture_flag;
#endif
struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0xF0, 2, {0x5A, 0x59}},
	{0xF1, 2, {0xA5, 0xA6}},
	{0x26, 1, {0x08}},
	{REGFLAG_DELAY, 1, {}},
	{0x28, 1, {}},
	{REGFLAG_DELAY, 20, {}},
	{0x10, 1, {}},
	{REGFLAG_DELAY, 110, {}},
	{0xCC, 1, {0x01}},
	{REGFLAG_DELAY, 10, {}},
	{REGFLAG_END_OF_TABLE, 0, {}},
};

#if defined(CONFIG_TOUCHSCREEN_COMMON)
static struct LCM_setting_table lcm_suspend_gesture_setting[] = {
	{0x26, 1, {0x08}},
	{REGFLAG_DELAY, 1, {}},
	{0x28, 0, {}},
	{REGFLAG_DELAY, 2, {}},
	{0x10, 0, {}},
	{REGFLAG_DELAY, 145, {}},
	{REGFLAG_END_OF_TABLE, 0, {}},
};
#endif

static struct LCM_setting_table init_setting[] = {
	{0xF0, 2, {0x5A, 0x59}},
	{0xF1, 2, {0xA5, 0xA6}},
	{0xB0, 32, {0x84, 0x83, 0x84, 0x85, 0x00, 0x00, 0x00, 0x00, 0x22, 0x22, 0x22, 0x22, 0x00, 0x05, 0x05, 0x7E, 0x05, 0x00, 0x3F, 0x05, 0x04, 0x03, 0x02, 0x01, 0x02, 0x03, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{0xB1, 32, {0x13, 0xC2, 0x91, 0x00, 0x7E, 0x05, 0x00, 0x78, 0x05, 0x00, 0x04, 0x08, 0x54, 0x00, 0x00, 0x00, 0x44, 0x40, 0x02, 0x01, 0x40, 0x02, 0x01, 0x40, 0x02, 0x01, 0x40, 0x02, 0x01, 0x00, 0x00, 0x00}},
	{0xB2, 17, {0x97, 0xC3, 0x91, 0x00, 0x00, 0x05, 0x05, 0x82, 0x05, 0x00, 0x05, 0x05, 0x54, 0x0C, 0x0C, 0x0D, 0x0B}},
	{0xB4, 28, {0x07, 0xDC, 0x00, 0x00, 0xDD, 0x00, 0x00, 0x1B, 0x13, 0x19, 0x11, 0x17, 0x0F, 0x15, 0x0D, 0x05, 0x05, 0x03, 0x03, 0x03, 0x03, 0x03, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 0x00}},
	{0xB5, 28, {0x06, 0xDC, 0x00, 0x00, 0xDD, 0x00, 0x00, 0x1A, 0x12, 0x18, 0x10, 0x16, 0x0E, 0x14, 0x0C, 0x04, 0x04, 0x03, 0x03, 0x03, 0x03, 0x03, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 0x00}},
	{0xB8, 24, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{0xBB, 13, {0x01, 0x05, 0x09, 0x11, 0x0D, 0x19, 0x1D, 0x55, 0x25, 0x69, 0x00, 0x21, 0x25}},
	{0xBC, 14, {0x00, 0x00, 0x00, 0x00, 0x02, 0x20, 0xFF, 0x00, 0x03, 0x13, 0x01, 0x73, 0x44, 0x00}},
	{0xBD, 10, {0xE9, 0x02, 0x4F, 0xCF, 0x72, 0xA7, 0x0C, 0x44, 0xAE, 0x15}},
	{0xBE, 10, {0x7D, 0x7D, 0x50, 0x3C, 0x0C, 0x77, 0x43, 0x07, 0x0E, 0x0E}},
	{0xBF, 8, {0x07, 0x25, 0x07, 0x25, 0x7F, 0x00, 0x11, 0x04}},
	{0xC0, 9, {0x10, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0x00}},
	{0xC1, 19, {0xC0, 0x12, 0x20, 0x7C, 0x04, 0x50, 0x50, 0x04, 0x2A, 0x40, 0x36, 0x00, 0x07, 0xCF, 0xFF, 0xFF, 0x9E, 0x01, 0xC0}},
	{0xC2, 1, {0x00}},
	{0xC3, 9, {0x03, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0x00, 0x81, 0x01}},
	{0xC4, 10, {0x84, 0x03, 0x2B, 0x41, 0x00, 0x3C, 0x00, 0xA3, 0x03, 0x2E}},
	{0xC5, 11, {0x02, 0x1C, 0x96, 0xC8, 0x32, 0x10, 0x42, 0xBF, 0x13, 0x14, 0x26}},
	{0xC6, 10, {0x88, 0x16, 0x1E, 0x29, 0x28, 0x71, 0x00, 0x13, 0x08, 0x04}},
	{0xC9, 6, {0x45, 0x00, 0x1F, 0xFF, 0x3F, 0x03}},
	{0xCB, 1, {0x00}},
	{0xD0, 5, {0x80, 0x0D, 0xFF, 0x0F, 0x63}},
	{0xD2, 15, {0x42, 0x0C, 0x30, 0x01, 0x80, 0x26, 0x04, 0x00, 0x00, 0xC3, 0x00, 0x00, 0x00, 0x28, 0x00}},
	{0xE1, 23, {0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xEE, 0xF0, 0x80, 0xD0, 0x00, 0x00, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xF0, 0xF0, 0x00, 0x80, 0x00, 0x04, 0x20}},
	{0xE0, 13, {0x30, 0x00, 0xA0, 0x98, 0x01, 0x1F, 0x23, 0x62, 0xDF, 0xA0, 0x04, 0xCC, 0x01}},
	{0xFE, 1, {0xEF}},
	{0xF0, 2, {0xA5, 0xA6}},
	{0xF1, 2, {0x5A, 0x59}},
	{0x35, 1, {0x00}},
	{0x51, 2, {0x0F, 0xFF}},
	{0x53, 1, {0x2C}},
	{0x55, 1, {0x00}},
	{0x11, 0, {}},
	{REGFLAG_DELAY, 100, {}},
	{0x29, 0, {}},
	{REGFLAG_DELAY, 100, {}},
	{0x26, 1, {0x01}},
};

#if 0
static struct LCM_setting_table lcm_set_window[] = {
	{0x2A, 4, {0x00, 0x00, (FRAME_WIDTH >> 8), (FRAME_WIDTH & 0xFF)} },
	{0x2B, 4, {0x00, 0x00, (FRAME_HEIGHT >> 8), (FRAME_HEIGHT & 0xFF)} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};
#endif
#if 0
static struct LCM_setting_table lcm_sleep_out_setting[] = {
	/* Sleep Out */
	{0x11, 1, {0x00} },
	{REGFLAG_DELAY, 120, {} },

	/* Display ON */
	{0x29, 1, {0x00} },
	{REGFLAG_DELAY, 20, {} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	/* Display off sequence */
	{0x28, 1, {0x00} },
	{REGFLAG_DELAY, 20, {} },

	/* Sleep Mode On */
	{0x10, 1, {0x00} },
	{REGFLAG_DELAY, 120, {} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};
#endif
static struct LCM_setting_table bl_level[] = {
	{0x51, 2, {0xC2, 0x05}},
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static void push_table(struct LCM_setting_table *table,
		unsigned int count,
		unsigned char force_update)
{
	unsigned int i;
	unsigned int cmd;

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;

		switch (cmd) {

		case REGFLAG_DELAY:
			if (table[i].count <= 10)
				MDELAY(table[i].count);
			else
				MDELAY(table[i].count);
			break;

		case REGFLAG_UDELAY:
			UDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V2(cmd, table[i].count,
					table[i].para_list, force_update);
		}
	}
}


static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}


static void lcm_get_params(struct LCM_PARAMS *params)
{
	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type = LCM_TYPE_DSI;
	params->lcm_if              = LCM_INTERFACE_DSI0;
	params->lcm_cmd_if          = LCM_INTERFACE_DSI0;
	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	params->physical_width = LCM_PHYSICAL_WIDTH/1000;
	params->physical_height = LCM_PHYSICAL_HEIGHT/1000;
	params->physical_width_um = LCM_PHYSICAL_WIDTH;
	params->physical_height_um = LCM_PHYSICAL_HEIGHT;
	params->density		   = LCM_DENSITY;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode = CMD_MODE;
	params->dsi.switch_mode = SYNC_PULSE_VDO_MODE;
#else
	params->dsi.mode = SYNC_PULSE_VDO_MODE;
	params->dsi.switch_mode = CMD_MODE;
#endif
	params->dsi.switch_mode_enable = 0;

	/* DSI */
	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;
	/* video mode timing */

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active = 4;
	params->dsi.vertical_backporch = 18;
	params->dsi.vertical_frontporch = 32;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 4;
	params->dsi.horizontal_backporch = 80;
	params->dsi.horizontal_frontporch = 80;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	params->dsi.ssc_disable = 1;
#ifndef CONFIG_FPGA_EARLY_PORTING
	params->dsi.PLL_CLOCK = 277;
	/* this value must be in MTK suggested table */
#else
	params->dsi.pll_div1 = 0;
	params->dsi.pll_div2 = 0;
	params->dsi.fbk_div = 0x1;
#endif

	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
}

static void lcm_init_power(void)
{
	no_printk("[LCM]%s\n",__func__);
	lcd_bl_en = 1;
}

static void lcm_suspend_power(void)
{
	no_printk("[LCM]%s\n",__func__);

	if (tpd_gesture_flag) {
		return;
	}

	disp_dts_gpio_select_state(DTS_GPIO_STATE_LCD_BIAS_ENN0);
	MDELAY(2);
	disp_dts_gpio_select_state(DTS_GPIO_STATE_LCD_BIAS_ENP0);
	MDELAY(5);
}

static void lcm_resume_power(void)
{
	no_printk("[LCM]%s\n",__func__);
}

static void lcm_init(void)
{
	unsigned char cmd = 0x0;
	unsigned char data = 0xFF;
	int ret = 0;

	no_printk("[LCM]%s\n",__func__);

	disp_dts_gpio_select_state(DTS_GPIO_STATE_LCD_BIAS_ENP1);
	MDELAY(2);
	disp_dts_gpio_select_state(DTS_GPIO_STATE_LCD_BIAS_ENN1);
	cmd = 0x00;
	data = 0x13;

	ret = tps65132_write_bytes(cmd, data);

	if (ret < 0)
		no_printk("[LCM]icnl9911c--tps6132--cmd=%0x--i2c write error--\n",
				cmd);
	else
		no_printk("[LCM]icnl9911c--tps6132--cmd=%0x--i2c write success--\n",
				cmd);

	cmd = 0x01;
	data = 0x13;

	ret = tps65132_write_bytes(cmd, data);

	if (ret < 0)
		no_printk("[LCM]icnl9911c--tps6132--cmd=%0x--i2c write error--\n",
				cmd);
	else
		no_printk("[LCM]icnl9911c--tps6132--cmd=%0x--i2c write success--\n",
				cmd);

	MDELAY(2);
	MDELAY(9);
	disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_RST_OUT0);
	MDELAY(1);
	MDELAY(9);
	disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_RST_OUT1);
	MDELAY(1);
	MDELAY(9);
	disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_RST_OUT0);
	MDELAY(1);
	MDELAY(9);
	disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_RST_OUT1);
	MDELAY(1);
	MDELAY(9);

	push_table(init_setting, sizeof(init_setting) /
		sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
	no_printk("[LCM]%s,tpd_gesture_flag = %d\n",__func__,tpd_gesture_flag);
#if defined(CONFIG_TOUCHSCREEN_COMMON)
	if (tpd_gesture_flag){
		push_table(lcm_suspend_gesture_setting,
		sizeof(lcm_suspend_gesture_setting) / sizeof(struct LCM_setting_table),
			1);
	} else {
		push_table(lcm_suspend_setting,
		sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table),
			1);
	}
#else
	push_table(lcm_suspend_setting,
		sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table),
			1);
#endif
}

static void lcm_resume(void)
{
	no_printk("[LCM]%s\n",__func__);
	lcm_init();
}

static void lcm_update(unsigned int x, unsigned int y, unsigned int width,
		unsigned int height)
{
#if LCM_DSI_CMD_MODE
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);
	unsigned char y0_MSB = ((y0 >> 8) & 0xFF);
	unsigned char y0_LSB = (y0 & 0xFF);
	unsigned char y1_MSB = ((y1 >> 8) & 0xFF);
	unsigned char y1_LSB = (y1 & 0xFF);

	unsigned int data_array[16];

	data_array[0] = 0x00053902;
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00053902;
	data_array[1] = (y1_MSB << 24) | (y0_LSB << 16) | (y0_MSB << 8) | 0x2b;
	data_array[2] = (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
#endif
}

static unsigned int lcm_compare_id(void)
{
	unsigned int id0 = 0;
	unsigned int id1 = 0;
	unsigned int id2 = 0;
	unsigned char buffer[3];
	unsigned int array[16];

	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(1);

	SET_RESET_PIN(1);
	MDELAY(20);

	array[0] = 0x00033700;	/* read id return two byte,version and id */
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x04, buffer, 3);
	id0 = buffer[0];     /* we only need ID */
	id1 = buffer[1];     /* we only need ID */
	id2 = buffer[2];     /* we only need ID */

	no_printk("[LCM]%s,icnl9911c id0 = 0x%x,id1 = 0x%x, id2 = 0x%x\n",
		 __func__, id0,id1,id2);
	if(id0 == 0x00 && id1 == 0x80 && id2 == 0x00)
		return 1;
	else
		return 0;
}


/* return TRUE: need recovery */
/* return FALSE: No need recovery */
static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_LK
	char buffer[3];
	int array[4];

	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x0A, buffer, 1);

	if (buffer[0] != 0x9C) {
		pr_no_warn("[LCM][LCM ERROR] [0x0A]=0x%02x\n", buffer[0]);
		return TRUE;
	}
	no_printk("[LCM][LCM NORMAL] [0x0A]=0x%02x\n", buffer[0]);
	return FALSE;
#else
	return FALSE;
#endif

}

static unsigned int lcm_ata_check(unsigned char *buffer)
{
#ifndef BUILD_LK
	unsigned int ret = 0;
	unsigned int x0 = FRAME_WIDTH / 4;
	unsigned int x1 = FRAME_WIDTH * 3 / 4;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);

	unsigned int data_array[3];
	unsigned char read_buf[4];

	no_printk("[LCM]ATA check size = 0x%x,0x%x,0x%x,0x%x\n",
			x0_MSB, x0_LSB, x1_MSB, x1_LSB);
	data_array[0] = 0x0005390A;	/* HS packet */
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	/* read id return two byte,version and id */
	data_array[0] = 0x00043700;
	dsi_set_cmdq(data_array, 1, 1);

	read_reg_v2(0x2A, read_buf, 4);

	if ((read_buf[0] == x0_MSB) && (read_buf[1] == x0_LSB)
			&& (read_buf[2] == x1_MSB) && (read_buf[3] == x1_LSB))
		ret = 1;
	else
		ret = 0;

	x0 = 0;
	x1 = FRAME_WIDTH - 1;

	x0_MSB = ((x0 >> 8) & 0xFF);
	x0_LSB = (x0 & 0xFF);
	x1_MSB = ((x1 >> 8) & 0xFF);
	x1_LSB = (x1 & 0xFF);

	data_array[0] = 0x0005390A;	/* HS packet */
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	return ret;
#else
	return 0;
#endif
}

static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{

	no_printk("[LCM]%s,icnl9911c backlight: level = %d lcd_bl_en = %d\n", __func__, level, lcd_bl_en);
	if((0 != level) && (level <= 14))
		level = 14;
	level = level*72/100;
#ifdef CONFIG_BACKLIGHT_SUPPORT_2047_FEATURE
	bl_level[0].para_list[0] = level >> 3;
	bl_level[0].para_list[1] = (level & 0b111) << 1;
#else
	bl_level[0].para_list[0] = level;
#endif
	if(level > 0 && lcd_bl_en == 0){
		disp_dts_gpio_select_state(DTS_GPIO_STATE_LCD_BL_OUT1);
		lcd_bl_en = 1;
	}else if(level == 0 && lcd_bl_en == 1){
		disp_dts_gpio_select_state(DTS_GPIO_STATE_LCD_BL_OUT0);
		lcd_bl_en = 0;
	}
	push_table(bl_level,
			sizeof(bl_level) / sizeof(struct LCM_setting_table), 1);
}

static void *lcm_switch_mode(int mode)
{
#ifndef BUILD_LK
	/* customization: 1. V2C config 2 values, */
	/* C2V config 1 value; 2. config mode control register */
	if (mode == 0) {	/* V2C */
		lcm_switch_mode_cmd.mode = CMD_MODE;
		/* mode control addr */
		lcm_switch_mode_cmd.addr = 0xBB;
		/* enabel GRAM firstly, ensure writing one frame to GRAM */
		lcm_switch_mode_cmd.val[0] = 0x13;
		/* disable video mode secondly */
		lcm_switch_mode_cmd.val[1] = 0x10;
	} else {		/* C2V */
		lcm_switch_mode_cmd.mode = SYNC_PULSE_VDO_MODE;
		lcm_switch_mode_cmd.addr = 0xBB;
		/* disable GRAM and enable video mode */
		lcm_switch_mode_cmd.val[0] = 0x03;
	}
	return (void *)(&lcm_switch_mode_cmd);
#else
	return NULL;
#endif
}


struct LCM_DRIVER icnl9911c_vdo_hdp_boe_xinli_lcm_drv = {
	.name = "icnl9911c_vdo_hdp_boe_xinli_drv",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.compare_id = lcm_compare_id,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
	.esd_check = lcm_esd_check,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.ata_check = lcm_ata_check,
	.update = lcm_update,
	.switch_mode = lcm_switch_mode,
};
