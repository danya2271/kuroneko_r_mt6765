// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 * Copyright (C) 2021 XiaoMi, Inc.
 */

#include "regulator.h"
//#include "upmu_common.h"


#include <mt-plat/aee.h>


#include <linux/rcupdate.h>
#include <linux/sched.h>
#include <linux/notifier.h>
#include <linux/regulator/consumer.h>
#include <linux/sched/signal.h>

static struct REGULATOR *preg_own;
static bool Is_Notify_call[IMGSENSOR_SENSOR_IDX_MAX_NUM][REGULATOR_TYPE_MAX_NUM];

struct reg_oc_debug_t {
	const char *name;
	struct notifier_block nb;
	struct regulator *regulator;
	struct work_struct work;
	unsigned int times;
	unsigned int md_reg_idx;
	bool is_md_reg;
};

static struct reg_oc_debug_t
	reg_oc_debug[IMGSENSOR_SENSOR_IDX_MAX_NUM][REGULATOR_TYPE_MAX_NUM];

static bool regulator_status[IMGSENSOR_SENSOR_IDX_MAX_NUM][REGULATOR_TYPE_MAX_NUM] = {{false}};
static void check_for_regulator_get(struct REGULATOR *preg,
				    struct device *pdevice, unsigned int sensor_index,
				    unsigned int regulator_index);
static void check_for_regulator_put(struct REGULATOR *preg,
				    unsigned int sensor_index, unsigned int regulator_index);
static struct device_node *of_node_record = NULL;

static DEFINE_MUTEX(g_regulator_state_mutex);

static const int regulator_voltage[] = {
	REGULATOR_VOLTAGE_0,
	REGULATOR_VOLTAGE_1000,
	REGULATOR_VOLTAGE_1100,
	REGULATOR_VOLTAGE_1200,
	REGULATOR_VOLTAGE_1210,
	REGULATOR_VOLTAGE_1220,
	REGULATOR_VOLTAGE_1500,
	REGULATOR_VOLTAGE_1800,
	REGULATOR_VOLTAGE_2500,
	REGULATOR_VOLTAGE_2800,
	REGULATOR_VOLTAGE_2900,
};

struct REGULATOR_CTRL regulator_control[REGULATOR_TYPE_MAX_NUM] = {
	{"vcama"},
	{"vcamd"},
	{"vcamio"},
};

static struct REGULATOR reg_instance;

static struct regulator *regVCAMAF;

static int regulator_oc_notify(
	struct notifier_block *nb, unsigned long event, void *data)
{
		struct reg_oc_debug_t *reg_oc_dbg =
			container_of(nb, struct reg_oc_debug_t, nb);

		if (event != REGULATOR_EVENT_OVER_CURRENT)
			return NOTIFY_OK;

		/* Do OC handling */
		pr_info("Imgsensor OC notify regulator: %s OC pid %ld\n",
			reg_oc_dbg->name, (long)reg_instance.pid);

		gimgsensor.status.oc = 1;
		aee_kernel_warning("Imgsensor OC", "Over current");
		if (reg_instance.pid != -1 &&
		pid_task(find_get_pid(reg_instance.pid), PIDTYPE_PID) != NULL) {
			force_sig(SIGKILL,
				pid_task(find_get_pid(reg_instance.pid),
				PIDTYPE_PID));
		}
		return NOTIFY_OK;
}

#define OC_MODULE "camera"
enum IMGSENSOR_RETURN imgsensor_oc_interrupt(
	enum IMGSENSOR_SENSOR_IDX sensor_idx, bool enable)
{
	struct regulator *preg = NULL;
	struct device *pdevice = gimgsensor_device;
	char str_regulator_name[LENGTH_FOR_SNPRINTF];
	unsigned int i = 0;

	int ret = 0;

	gimgsensor.status.oc = 0;

	if (enable) {
		mdelay(5);
		for (i = 0; i < REGULATOR_TYPE_MAX_NUM; i++) {
			ret = snprintf(str_regulator_name,
					sizeof(str_regulator_name),
					"cam%d_%s",
					sensor_idx,
					regulator_control[i].pregulator_type);
			if (ret < 0) {
				pr_info(
				"[regulator]%s error, ret = %d", __func__, ret);
				return IMGSENSOR_RETURN_ERROR;
			}
			preg = regulator_get_optional(
					pdevice, str_regulator_name);
			if (IS_ERR(preg))
				preg = NULL;
			if (preg && regulator_is_enabled(preg)) {
				/* oc notifier callback function */
				reg_oc_debug[sensor_idx][i].nb.notifier_call =
				regulator_oc_notify;
#ifndef NO_OC
			ret = devm_regulator_register_notifier(preg,
				&reg_oc_debug[sensor_idx][i].nb);

			if (ret) {
				pr_info(
				"regulator notifier request error\n");
			}
#endif
			pr_debug(
				"[regulator] %s idx=%d %s enable=%d\n",
				__func__,
				sensor_idx,
				regulator_control[i].pregulator_type,
				enable);
			}
		}
		rcu_read_lock();
		reg_instance.pid = current->tgid;
		rcu_read_unlock();
	} else {
		reg_instance.pid = -1;
		/* Disable interrupt before power off */
		pr_debug("Unregister OC notifier");
		for (i = 0; i < REGULATOR_TYPE_MAX_NUM; i++) {
			ret = snprintf(str_regulator_name,
					sizeof(str_regulator_name),
					"cam%d_%s",
					sensor_idx,
					regulator_control[i].pregulator_type);
			if (ret < 0) {
				pr_info(
				"[regulator]%s error, ret = %d", __func__, ret);
				return IMGSENSOR_RETURN_ERROR;
			}
			preg = regulator_get_optional(
					pdevice, str_regulator_name);
			if (IS_ERR(preg))
				preg = NULL;
#ifndef NO_OC
			if (preg) {
				/* oc notifier callback function */
				devm_regulator_unregister_notifier(preg,
				&reg_oc_debug[sensor_idx][i].nb);
			}
#endif
		}

	}

	return IMGSENSOR_RETURN_SUCCESS;
}

enum IMGSENSOR_RETURN imgsensor_oc_init(void)
{
	/* Register your interrupt handler of OC interrupt at first */

	gimgsensor.status.oc  = 0;
	gimgsensor.imgsensor_oc_irq_enable = imgsensor_oc_interrupt;
	reg_instance.pid = -1;

	return IMGSENSOR_RETURN_SUCCESS;
}


static enum IMGSENSOR_RETURN regulator_init(void *pinstance)
{
	struct REGULATOR *preg = (struct REGULATOR *)pinstance;
	struct device            *pdevice;
	struct device_node       *pof_node;
	int j, i;
	char str_regulator_name[LENGTH_FOR_SNPRINTF];

	pdevice  = gimgsensor_device;
	pof_node = pdevice->of_node;
	pdevice->of_node =
		of_find_compatible_node(NULL, NULL, "mediatek,camera_hw");

	if (pdevice->of_node == NULL) {
		pr_err("regulator get cust camera node failed!\n");
		pdevice->of_node = pof_node;
		return IMGSENSOR_RETURN_ERROR;
	}

	of_node_record = pdevice->of_node;
	for (j = IMGSENSOR_SENSOR_IDX_MIN_NUM;
		j < IMGSENSOR_SENSOR_IDX_MAX_NUM;
		j++) {
		for (i = 0; i < REGULATOR_TYPE_MAX_NUM; i++) {
			snprintf(str_regulator_name,
					sizeof(str_regulator_name),
					"cam%d_%s",
					j,
					regulator_control[i].pregulator_type);
			preg->pregulator[j][i] =
			    regulator_get_optional(
				pdevice, str_regulator_name);
			if (IS_ERR(preg->pregulator[j][i]))
				preg->pregulator[j][i] = NULL;
			if (preg->pregulator[j][i] == NULL)
				pr_err("regulator[%d][%d]  %s fail!\n",
					j, i, str_regulator_name);

			atomic_set(&preg->enable_cnt[j][i], 0);
			regulator_status[j][i] = true;
		}
	}

	regVCAMAF = regulator_get(pdevice, "vldo28");
	pr_err("[chenxy] regulator_get vldo28 %p\n", regVCAMAF);

	pdevice->of_node = pof_node;
	imgsensor_oc_init();
	preg_own = (struct REGULATOR *)pinstance;
	return IMGSENSOR_RETURN_SUCCESS;
}
static enum IMGSENSOR_RETURN regulator_release(void *pinstance)
{
	struct REGULATOR *preg = (struct REGULATOR *)pinstance;
	int type, idx;
	struct regulator *pregulator = NULL;
	atomic_t *enable_cnt = NULL;

	for (idx = IMGSENSOR_SENSOR_IDX_MIN_NUM;
		idx < IMGSENSOR_SENSOR_IDX_MAX_NUM;
		idx++) {

		for (type = 0; type < REGULATOR_TYPE_MAX_NUM; type++) {
			pregulator = preg->pregulator[idx][type];
			enable_cnt = &preg->enable_cnt[idx][type];
			if (pregulator != NULL) {
				for (; atomic_read(enable_cnt) > 0; ) {
					regulator_disable(pregulator);
					atomic_dec(enable_cnt);
				}
			}
		}
	}
	return IMGSENSOR_RETURN_SUCCESS;
}

static enum IMGSENSOR_RETURN regulator_set(
	void *pinstance,
	enum IMGSENSOR_SENSOR_IDX   sensor_idx,
	enum IMGSENSOR_HW_PIN       pin,
	enum IMGSENSOR_HW_PIN_STATE pin_state)
{
	struct regulator     *pregulator;
	struct REGULATOR     *preg = (struct REGULATOR *)pinstance;
	int reg_type_offset;
	atomic_t             *enable_cnt;

	if(pin == IMGSENSOR_HW_PIN_AFVDD && regVCAMAF != NULL){
		if (pin_state != IMGSENSOR_HW_PIN_STATE_LEVEL_0) {

			if (regulator_set_voltage(
				    regVCAMAF,
				    regulator_voltage[
					    pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0],
				    regulator_voltage[
					    pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0])) {

				pr_err(
					"[regulator]fail to regulator_set_voltage, powertype:%d powerId:%d\n",
					pin,
					regulator_voltage[
						pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0]);
			}
			if (regulator_enable(regVCAMAF)) {
				pr_err(
					"[regulator]fail to regulator_enable, powertype:%d powerId:%d\n",
					pin,
					regulator_voltage[
						pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0]);

				return IMGSENSOR_RETURN_ERROR;
			}
			//atomic_inc(enable_cnt);
		} else {
			if (regulator_is_enabled(regVCAMAF)) {
				/*pr_debug("[regulator]%d is enabled\n", pin);*/

				if (regulator_disable(regVCAMAF)) {
					pr_err(
						"[regulator]fail to regulator_disable, powertype: %d\n",
						pin);
					return IMGSENSOR_RETURN_ERROR;
				}
			}
			//atomic_dec(enable_cnt);
		}
	}

	if (pin > IMGSENSOR_HW_PIN_DOVDD   ||
	    pin < IMGSENSOR_HW_PIN_AVDD    ||
	    pin_state < IMGSENSOR_HW_PIN_STATE_LEVEL_0 ||
	    pin_state >= IMGSENSOR_HW_PIN_STATE_LEVEL_HIGH ||
	    sensor_idx < 0)
		return IMGSENSOR_RETURN_ERROR;

	reg_type_offset = REGULATOR_TYPE_VCAMA;

	check_for_regulator_get(preg, gimgsensor_device, sensor_idx,
				(reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD));

	pregulator =
		preg->pregulator[sensor_idx][
			reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD];

	enable_cnt =
		&preg->enable_cnt[sensor_idx][
			reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD];

	if (pregulator) {
		if (pin_state != IMGSENSOR_HW_PIN_STATE_LEVEL_0) {

			if (regulator_set_voltage(
				pregulator,
				regulator_voltage[
				    pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0],
				regulator_voltage[
				 pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0])) {

				pr_err(
				    "[regulator]fail to regulator_set_voltage, powertype:%d powerId:%d\n",
				    pin,
				    regulator_voltage[
				   pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0]);
			}
			if (regulator_enable(pregulator)) {
				pr_err(
				    "[regulator]fail to regulator_enable, powertype:%d powerId:%d\n",
				    pin,
				    regulator_voltage[
				   pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0]);

				check_for_regulator_put(preg, sensor_idx,
							(reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD));

				return IMGSENSOR_RETURN_ERROR;
			}
			atomic_inc(enable_cnt);
		} else {
			if (regulator_is_enabled(pregulator)) {
				/*pr_no_debug("[regulator]%d is enabled\n", pin);*/

				if (regulator_disable(pregulator)) {
					pr_err(
					    "[regulator]fail to regulator_disable, powertype: %d\n",
					    pin);

					check_for_regulator_put(preg, sensor_idx,
								(reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD));
					return IMGSENSOR_RETURN_ERROR;
				}
			}

			check_for_regulator_put(preg, sensor_idx,
						(reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD));
			atomic_dec(enable_cnt);
		}
	} else {
		pr_err("regulator == NULL %d %d %d\n",
		    reg_type_offset,
		    pin,
		    IMGSENSOR_HW_PIN_AVDD);
	}

	return IMGSENSOR_RETURN_SUCCESS;
}

static void check_for_regulator_get(struct REGULATOR *preg,
				    struct device *pdevice, unsigned int sensor_index,
				    unsigned int regulator_index)
{
	struct device_node *pof_node = NULL;
	char str_regulator_name[LENGTH_FOR_SNPRINTF];

	if (!preg || !pdevice) {
		pr_err("Fatal: Null ptr.preg:%pK,pdevice:%pK\n", preg, pdevice);
		return;
	}

	if (sensor_index >= IMGSENSOR_SENSOR_IDX_MAX_NUM ||
	    regulator_index >= REGULATOR_TYPE_MAX_NUM ) {
		pr_err("[%s]Invalid sensor_idx:%d regulator_idx: %d\n",
		       __func__, sensor_index, regulator_index);
		return;
	}

	mutex_lock(&g_regulator_state_mutex);

	if (regulator_status[sensor_index][regulator_index] == false) {
		pof_node = pdevice->of_node;
		pdevice->of_node = of_node_record;

		snprintf(str_regulator_name,
			 sizeof(str_regulator_name),
			 "cam%d_%s",
			 sensor_index,
			 regulator_control[regulator_index].pregulator_type);
		preg->pregulator[sensor_index][regulator_index] =
			regulator_get(pdevice, str_regulator_name);

		if (preg != NULL)
			regulator_status[sensor_index][regulator_index] = true;
		else
			pr_err("get regulator failed.\n");
		pdevice->of_node = pof_node;
	}

	mutex_unlock(&g_regulator_state_mutex);

	return;
}

static void check_for_regulator_put(struct REGULATOR *preg,
				    unsigned int sensor_index, unsigned int regulator_index)
{
	if (!preg) {
		pr_err("Fatal: Null ptr.\n");
		return;
	}

	if (sensor_index >= IMGSENSOR_SENSOR_IDX_MAX_NUM ||
	    regulator_index >= REGULATOR_TYPE_MAX_NUM ) {
		pr_err("[%s]Invalid sensor_idx:%d regulator_idx: %d\n",
		       __func__, sensor_index, regulator_index);
		return;
	}

	mutex_lock(&g_regulator_state_mutex);

	if (regulator_status[sensor_index][regulator_index] == true) {
		regulator_put(preg->pregulator[sensor_index][regulator_index]);
		regulator_status[sensor_index][regulator_index] = false;
	}

	mutex_unlock(&g_regulator_state_mutex);

	return;
}

static struct IMGSENSOR_HW_DEVICE device = {
	.pinstance = (void *)&reg_instance,
	.init      = regulator_init,
	.set       = regulator_set,
	.release   = regulator_release,
	.id        = IMGSENSOR_HW_ID_REGULATOR
};

enum IMGSENSOR_RETURN imgsensor_hw_regulator_open(
	struct IMGSENSOR_HW_DEVICE **pdevice)
{
	*pdevice = &device;
	return IMGSENSOR_RETURN_SUCCESS;
}

