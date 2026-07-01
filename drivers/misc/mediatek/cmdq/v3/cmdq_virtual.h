/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2015 MediaTek Inc.
 */

#ifndef __CMDQ_CORE_VIRTUAL_H__
#define __CMDQ_CORE_VIRTUAL_H__

#include "cmdq_def.h"
#include "cmdq_helper_ext.h"

#ifdef __cplusplus
extern "C" {
#endif
	u32 cmdq_virtual_get_subsys_LSB_in_arg_a(void);
	bool cmdq_virtual_is_a_secure_thread(const s32 thread);
	bool cmdq_virtual_is_disp_scenario(
		const enum CMDQ_SCENARIO_ENUM scenario);
	bool cmdq_virtual_is_dynamic_scenario(
		const enum CMDQ_SCENARIO_ENUM scenario);
	bool cmdq_virtual_should_enable_prefetch(
		enum CMDQ_SCENARIO_ENUM scenario);
	int cmdq_virtual_disp_thread(enum CMDQ_SCENARIO_ENUM scenario);
	int cmdq_virtual_get_thread_index(enum CMDQ_SCENARIO_ENUM scenario,
		const bool secure);
	enum CMDQ_HW_THREAD_PRIORITY_ENUM cmdq_virtual_priority_from_scenario(
		enum CMDQ_SCENARIO_ENUM scenario);
	bool cmdq_virtual_force_loop_irq(enum CMDQ_SCENARIO_ENUM scenario);
	bool cmdq_virtual_is_disp_loop(enum CMDQ_SCENARIO_ENUM scenario);
	void cmdq_virtual_get_reg_id_from_hwflag(u64 hwflag,
		enum cmdq_gpr_reg *valueRegId,
		enum cmdq_gpr_reg *destRegId,
		enum cmdq_event *regAccessToken);
	const char *cmdq_virtual_module_from_event_id(const s32 event,
		struct CmdqCBkStruct *groupCallback, u64 engineFlag);
	const char *cmdq_virtual_parse_module_from_reg_addr(u32 reg_addr);
	s32 cmdq_virtual_can_module_entry_suspend(
		struct EngineStruct *engineList);
	ssize_t cmdq_virtual_print_status_clock(char *buf);
	void cmdq_virtual_print_status_seq_clock(struct seq_file *m);
	void cmdq_virtual_enable_gce_clock_locked(bool enable);
	const char *cmdq_virtual_parse_error_module_by_hwflag_impl(
		const struct cmdqRecStruct *pTask);
	const char *cmdq_virtual_parse_handle_error_module_by_hwflag_impl(
		const struct cmdqRecStruct *pHandle);
	int cmdq_virtual_dump_smi(const int showSmiDump);
	void cmdq_virtual_dump_gpr(void);
	u64 cmdq_virtual_flag_from_scenario(enum CMDQ_SCENARIO_ENUM scenario);
	void cmdq_virtual_event_backup(void);
	void cmdq_virtual_event_restore(void);
	void cmdq_virtual_test_setup(void);
	void cmdq_virtual_test_cleanup(void);
	void cmdq_virtual_init_module_PA_stat(void);

#ifdef __cplusplus
}
#endif
#endif				/* __CMDQ_CORE_VIRTUAL_H__ */
