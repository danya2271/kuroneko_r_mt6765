/*
 * Copyright (C) 2015 MediaTek Inc.
 * Copyright (C) 2021 XiaoMi, Inc.
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

#include "mt65xx_lcm_list.h"
#include <lcm_drv.h>
#ifdef BUILD_LK
#include <platform/disp_drv_platform.h>
#else
#include <linux/delay.h>
/* #include <mach/mt_gpio.h> */
#endif
enum LCM_DSI_MODE_CON lcm_dsi_mode;

/* used to identify float ID PIN status */
#define LCD_HW_ID_STATUS_LOW      0
#define LCD_HW_ID_STATUS_HIGH     1
#define LCD_HW_ID_STATUS_FLOAT 0x02
#define LCD_HW_ID_STATUS_ERROR  0x03

struct LCM_DRIVER *lcm_driver_list[] = {
#if defined(ICNL9911C_VDO_HDP_BOE_XINLI)
	&icnl9911c_vdo_hdp_boe_xinli_lcm_drv,
#endif
#if defined(ICNL9911C_VDO_HDP_BOE_TIANMA)
        &icnl9911c_vdo_hdp_boe_tianma_lcm_drv,
#endif
#if defined(NT36525B_VDO_HDP_BOE_DIJING)
	&nt36525b_vdo_hdp_boe_dijing_lcm_drv,
#endif
#if defined(NT36525B_VDO_HDP_BOE_XINLI)
        &nt36525b_vdo_hdp_boe_xinli_lcm_drv,
#endif
#if defined(FT8006S_VDO_HDP_BOE_HELITAI)
	&ft8006s_vdo_hdp_boe_helitai_lcm_drv,
#endif
#if defined(FT8006S_AB_VDO_HDP_BOE_HELITAI)
	&ft8006s_ab_vdo_hdp_boe_helitai_lcm_drv,
#endif
#if defined(FT8006S_AC_VDO_HDP_BOE_HELITAI)
	&ft8006s_ac_vdo_hdp_boe_helitai_lcm_drv,
#endif
#if defined(HX83102D_VDO_HDP_BOE_XINLI)
	&hx83102d_vdo_hdp_boe_xinli_lcm_drv,
#endif
#if defined(NT36525B_VDO_HDP_BOE_HELITAI)
	&nt36525b_vdo_hdp_boe_helitai_lcm_drv,
#endif
#if defined(NT36525B_VDO_HDP_PANDA_SHENGCHAO)
	&nt36525b_vdo_hdp_panda_shengchao_lcm_drv,
#endif
};

#define LCM_COMPILE_ASSERT(condition) \
	LCM_COMPILE_ASSERT_X(condition, __LINE__)
#define LCM_COMPILE_ASSERT_X(condition, line) \
	LCM_COMPILE_ASSERT_XX(condition, line)
#define LCM_COMPILE_ASSERT_XX(condition, line) \
	char assertion_failed_at_line_##line[(condition) ? 1 : -1]

unsigned int lcm_count =
	sizeof(lcm_driver_list) / sizeof(struct LCM_DRIVER *);
LCM_COMPILE_ASSERT(sizeof(lcm_driver_list) / sizeof(struct LCM_DRIVER *) != 0);
#if defined(NT35520_HD720_DSI_CMD_TM) | \
	defined(NT35520_HD720_DSI_CMD_BOE) | \
	defined(NT35521_HD720_DSI_VDO_BOE) | \
	defined(NT35521_HD720_DSI_VIDEO_TM)
static unsigned char lcd_id_pins_value = 0xFF;

/*
 * Function:    which_lcd_module_triple
 * Description: read LCD ID PIN status,could identify three status:highlowfloat
 * Input:       none
 * Output:      none
 * Return:      LCD ID1|ID0 value
 * Others:
 */
unsigned char which_lcd_module_triple(void)
{
	unsigned char  high_read0 = 0;
	unsigned char  low_read0 = 0;
	unsigned char  high_read1 = 0;
	unsigned char  low_read1 = 0;
	unsigned char  lcd_id0 = 0;
	unsigned char  lcd_id1 = 0;
	unsigned char  lcd_id = 0;
	/*Solve Coverity scan warning : check return value*/
	unsigned int ret = 0;

	/*only recognise once*/
	if (lcd_id_pins_value != 0xFF)
		return lcd_id_pins_value;

	/*Solve Coverity scan warning : check return value*/
	ret = mt_set_gpio_mode(GPIO_DISP_ID0_PIN, GPIO_MODE_00);
	if (ret != 0)
		no_printk("[LCM]ID0 mt_set_gpio_mode fail\n");

	ret = mt_set_gpio_dir(GPIO_DISP_ID0_PIN, GPIO_DIR_IN);
	if (ret != 0)
		no_printk("[LCM]ID0 mt_set_gpio_dir fail\n");

	ret = mt_set_gpio_pull_enable(GPIO_DISP_ID0_PIN, GPIO_PULL_ENABLE);
	if (ret != 0)
		no_printk("[LCM]ID0 mt_set_gpio_pull_enable fail\n");

	ret = mt_set_gpio_mode(GPIO_DISP_ID1_PIN, GPIO_MODE_00);
	if (ret != 0)
		no_printk("[LCM]ID1 mt_set_gpio_mode fail\n");

	ret = mt_set_gpio_dir(GPIO_DISP_ID1_PIN, GPIO_DIR_IN);
	if (ret != 0)
		no_printk("[LCM]ID1 mt_set_gpio_dir fail\n");

	ret = mt_set_gpio_pull_enable(GPIO_DISP_ID1_PIN, GPIO_PULL_ENABLE);
	if (ret != 0)
		no_printk("[LCM]ID1 mt_set_gpio_pull_enable fail\n");

	/*pull down ID0 ID1 PIN*/
	ret = mt_set_gpio_pull_select(GPIO_DISP_ID0_PIN, GPIO_PULL_DOWN);
	if (ret != 0)
		no_printk("[LCM]ID0 mt_set_gpio_pull_select->Down fail\n");

	ret = mt_set_gpio_pull_select(GPIO_DISP_ID1_PIN, GPIO_PULL_DOWN);
	if (ret != 0)
		no_printk("[LCM]ID1 mt_set_gpio_pull_select->Down fail\n");

	/* delay 100ms , for discharging capacitance*/
	mdelay(100);
	/* get ID0 ID1 status*/
	low_read0 = mt_get_gpio_in(GPIO_DISP_ID0_PIN);
	low_read1 = mt_get_gpio_in(GPIO_DISP_ID1_PIN);
	/* pull up ID0 ID1 PIN */
	ret = mt_set_gpio_pull_select(GPIO_DISP_ID0_PIN, GPIO_PULL_UP);
	if (ret != 0)
		no_printk("[LCM]ID0 mt_set_gpio_pull_select->UP fail\n");

	ret = mt_set_gpio_pull_select(GPIO_DISP_ID1_PIN, GPIO_PULL_UP);
	if (ret != 0)
		no_printk("[LCM]ID1 mt_set_gpio_pull_select->UP fail\n");

	/* delay 100ms , for charging capacitance */
	mdelay(100);
	/* get ID0 ID1 status */
	high_read0 = mt_get_gpio_in(GPIO_DISP_ID0_PIN);
	high_read1 = mt_get_gpio_in(GPIO_DISP_ID1_PIN);

	if (low_read0 != high_read0) {
		/*float status , pull down ID0 ,to prevent electric leakage*/
		ret = mt_set_gpio_pull_select(GPIO_DISP_ID0_PIN,
			GPIO_PULL_DOWN);
		if (ret != 0)
			no_printk("[LCM]ID0 mt_set_gpio_pull_select->Down fail\n");

		lcd_id0 = LCD_HW_ID_STATUS_FLOAT;
	} else if ((low_read0 == LCD_HW_ID_STATUS_LOW) &&
		(high_read0 == LCD_HW_ID_STATUS_LOW)) {
		/*low status , pull down ID0 ,to prevent electric leakage*/
		ret = mt_set_gpio_pull_select(GPIO_DISP_ID0_PIN,
			GPIO_PULL_DOWN);
		if (ret != 0)
			no_printk("[LCM]ID0 mt_set_gpio_pull_select->Down fail\n");

		lcd_id0 = LCD_HW_ID_STATUS_LOW;
	} else if ((low_read0 == LCD_HW_ID_STATUS_HIGH) &&
		(high_read0 == LCD_HW_ID_STATUS_HIGH)) {
		/*high status , pull up ID0 ,to prevent electric leakage*/
		ret = mt_set_gpio_pull_select(GPIO_DISP_ID0_PIN, GPIO_PULL_UP);
		if (ret != 0)
			no_printk("[LCM]ID0 mt_set_gpio_pull_select->UP fail\n");

		lcd_id0 = LCD_HW_ID_STATUS_HIGH;
	} else {
		no_printk("[LCM] Read LCD_id0 error\n");
		ret = mt_set_gpio_pull_select(GPIO_DISP_ID0_PIN,
			GPIO_PULL_DISABLE);
		if (ret != 0)
			no_printk("[KERNEL/LCM]ID0 mt_set_gpio_pull_select->Disbale fail\n");

		lcd_id0 = LCD_HW_ID_STATUS_ERROR;
	}


	if (low_read1 != high_read1) {
		/*float status , pull down ID1 ,to prevent electric leakage*/
		ret = mt_set_gpio_pull_select(GPIO_DISP_ID1_PIN,
			GPIO_PULL_DOWN);
		if (ret != 0)
			no_printk("[LCM]ID1 mt_set_gpio_pull_select->Down fail\n");

		lcd_id1 = LCD_HW_ID_STATUS_FLOAT;
	} else if ((low_read1 == LCD_HW_ID_STATUS_LOW) &&
		(high_read1 == LCD_HW_ID_STATUS_LOW)) {
		/*low status , pull down ID1 ,to prevent electric leakage*/
		ret = mt_set_gpio_pull_select(GPIO_DISP_ID1_PIN,
			GPIO_PULL_DOWN);
		if (ret != 0)
			no_printk("[LCM]ID1 mt_set_gpio_pull_select->Down fail\n");

		lcd_id1 = LCD_HW_ID_STATUS_LOW;
	} else if ((low_read1 == LCD_HW_ID_STATUS_HIGH) &&
		(high_read1 == LCD_HW_ID_STATUS_HIGH)) {
		/*high status , pull up ID1 ,to prevent electric leakage*/
		ret = mt_set_gpio_pull_select(GPIO_DISP_ID1_PIN, GPIO_PULL_UP);
		if (ret != 0)
			no_printk("[LCM]ID1 mt_set_gpio_pull_select->UP fail\n");

		lcd_id1 = LCD_HW_ID_STATUS_HIGH;
	} else {

		no_printk("[LCM] Read LCD_id1 error\n");
		ret = mt_set_gpio_pull_select(GPIO_DISP_ID1_PIN,
			GPIO_PULL_DISABLE);
		if (ret != 0)
			no_printk("[KERNEL/LCM]ID1 mt_set_gpio_pull_select->Disable fail\n");

		lcd_id1 = LCD_HW_ID_STATUS_ERROR;
	}
#ifdef BUILD_LK
	dprintf(CRITICAL, "which_lcd_module_triple,lcd_id0:%d\n", lcd_id0);
	dprintf(CRITICAL, "which_lcd_module_triple,lcd_id1:%d\n", lcd_id1);
#else
	no_printk("[LCM]which_lcd_module_triple,lcd_id0:%d\n", lcd_id0);
	no_printk("[LCM]which_lcd_module_triple,lcd_id1:%d\n", lcd_id1);
#endif
	lcd_id =  lcd_id0 | (lcd_id1 << 2);

#ifdef BUILD_LK
	dprintf(CRITICAL, "which_lcd_module_triple,lcd_id:%d\n", lcd_id);
#else
	no_printk("[LCM]which_lcd_module_triple,lcd_id:%d\n", lcd_id);
#endif

	lcd_id_pins_value = lcd_id;
	return lcd_id;
}
#endif
