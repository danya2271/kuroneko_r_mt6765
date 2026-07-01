/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2015 MediaTek Inc.
 */

#ifndef __CMDQ_MDP_COMMON_H__
#define __CMDQ_MDP_COMMON_H__

#include "cmdq_def.h"
#include "cmdq_helper_ext.h"
#include <linux/types.h>

/* track MDP task */
#define DEBUG_STR_LEN 1024 /* debug str length */
#define MDP_MAX_TASK_NUM 5 /* num of tasks to be keep */
#define MDP_MAX_PLANE_NUM 3 /* max color format plane num */
/* each plane has 2 info address and size */
#define MDP_PORT_BUF_INFO_NUM (MDP_MAX_PLANE_NUM * 2)
#define MDP_BUF_INFO_STR_LEN 8 /* each buf info length */
/* dispatch key format is MDP_(ThreadName) */
#define MDP_DISPATCH_KEY_STR_LEN (TASK_COMM_LEN + 5)
#define MDP_TOTAL_THREAD 16
#ifdef CMDQ_SECURE_PATH_SUPPORT
#define MDP_THREAD_START (CMDQ_MIN_SECURE_THREAD_ID + 2)
#else
#define MDP_THREAD_START CMDQ_DYNAMIC_THREAD_ID_START
#endif

/* MDP common kernel logic */

void cmdq_mdp_fix_command_scenario_for_user_space(
	struct cmdqCommandStruct *command);
bool cmdq_mdp_is_request_from_user_space(
	const enum CMDQ_SCENARIO_ENUM scenario);
s32 cmdq_mdp_query_usage(s32 *counters);
s32 cmdq_mdp_get_smi_usage(void);

void cmdq_mdp_reset_resource(void);
void cmdq_mdp_dump_thread_usage(void);
void cmdq_mdp_dump_engine_usage(void);
void cmdq_mdp_dump_resource(u32 event);
void cmdq_mdp_init_resource(u32 engine_id,
	enum cmdq_event res_event);
void cmdq_mdp_enable_res(u64 engine_flag, bool enable);
void cmdq_mdp_lock_resource(u64 engine_flag, bool from_notify);
bool cmdq_mdp_acquire_resource(enum cmdq_event res_event,
	u64 *engine_flag_out);
void cmdq_mdp_release_resource(enum cmdq_event res_event,
	u64 *engine_flag_out);
void cmdq_mdp_set_resource_callback(enum cmdq_event res_event,
	CmdqResourceAvailableCB res_available,
	CmdqResourceReleaseCB res_release);
void cmdq_mdp_unlock_thread(struct cmdqRecStruct *handle);
s32 cmdq_mdp_flush_async(struct cmdqCommandStruct *desc, bool user_space,
	struct cmdqRecStruct **handle_out);
s32 cmdq_mdp_flush_async_impl(struct cmdqRecStruct *handle);
struct cmdqRecStruct *cmdq_mdp_get_valid_handle(unsigned long job);
s32 cmdq_mdp_wait(struct cmdqRecStruct *handle,
	struct cmdqRegValueStruct *results);
s32 cmdq_mdp_flush(struct cmdqCommandStruct *desc, bool user_space);
void cmdq_mdp_suspend(void);
void cmdq_mdp_resume(void);
void cmdq_mdp_release_task_by_file_node(void *file_node);
void cmdq_mdp_init(void);
void cmdq_mdp_deinit_pmqos(void);
s32 cmdq_mdp_handle_create(struct cmdqRecStruct **handle_out);
s32 cmdq_mdp_handle_flush(struct cmdqRecStruct *handle);
s32 cmdq_mdp_handle_sec_setup(struct cmdqSecDataStruct *secData,
			struct cmdqRecStruct *handle);

struct op_meta;
struct mdp_submit;
s32 cmdq_mdp_update_sec_addr_index(struct cmdqRecStruct *handle,
	u32 sec_handle, u32 index, u32 instr_index);
u32 cmdq_mdp_handle_get_instr_count(struct cmdqRecStruct *handle);
void cmdq_mdp_meta_replace_sec_addr(struct op_meta *metas,
	struct mdp_submit *user_job, struct cmdqRecStruct *handle);

/* Common MDP callbacks */
#ifdef CONFIG_MTK_SMI_EXT
void cmdq_mdp_init_pmqos_mdp_virtual(s32 index,
	struct plist_head *owner_list);
void cmdq_mdp_init_pmqos_isp_virtual(s32 index,
	struct plist_head *owner_list);
#endif
const char *cmdq_mdp_dispatch_virtual(u64 engineFlag);
void cmdq_mdp_trackTask_virtual(const struct cmdqRecStruct *task);
void cmdq_mdp_error_reset_virtual(u64 engineFlag);
void cmdq_mdp_begin_task_virtual(struct cmdqRecStruct *handle,
	struct cmdqRecStruct **handle_list, u32 size);
void cmdq_mdp_end_task_virtual(struct cmdqRecStruct *handle,
	struct cmdqRecStruct **handle_list, u32 size);
u64 cmdq_mdp_get_secure_engine_virtual(u64 engine_flag);

/* Platform dependent function */

void cmdq_mdp_map_mmsys_VA(void);
long cmdq_mdp_get_module_base_VA_MMSYS_CONFIG(void);
void cmdq_mdp_unmap_mmsys_VA(void);

void cmdq_mdp_enable(u64 engineFlag, enum CMDQ_ENG_ENUM engine);

int cmdq_mdp_loop_reset(enum CMDQ_ENG_ENUM engine,
	const unsigned long resetReg,
	const unsigned long resetStateReg,
	const u32 resetMask,
	const u32 resetValue, const bool pollInitResult);

void cmdq_mdp_loop_off(enum CMDQ_ENG_ENUM engine,
	const unsigned long resetReg,
	const unsigned long resetStateReg,
	const u32 resetMask,
	const u32 resetValue, const bool pollInitResult);

const char *cmdq_mdp_get_rsz_state(const u32 state);

void cmdq_mdp_dump_venc(const unsigned long base, const char *label);
void cmdq_mdp_dump_rdma(const unsigned long base, const char *label);
void cmdq_mdp_dump_rot(const unsigned long base, const char *label);
void cmdq_mdp_dump_color(const unsigned long base, const char *label);
void cmdq_mdp_dump_wdma(const unsigned long base, const char *label);

void cmdq_mdp_check_TF_address(unsigned int mva, char *module);

const char *cmdq_mdp_parse_handle_error_module_by_hwflag(
	const struct cmdqRecStruct *handle);

/* Platform dependent function */

void cmdq_mdp_dump_mmsys_config(void);
void cmdq_mdp_init_module_base_VA(void);
void cmdq_mdp_deinit_module_base_VA(void);
bool cmdq_mdp_clock_is_on(enum CMDQ_ENG_ENUM engine);
void cmdq_mdp_enable_clock(bool enable, enum CMDQ_ENG_ENUM engine);
void cmdq_mdp_init_module_clk(void);
void cmdq_mdp_dump_rsz(const unsigned long base, const char *label);
void cmdq_mdp_dump_tdshp(const unsigned long base, const char *label);
s32 cmdqMdpClockOn(u64 engineFlag);
s32 cmdqMdpDumpInfo(u64 engineFlag, int level);
s32 cmdqVEncDumpInfo(u64 engineFlag, int level);
s32 cmdqMdpResetEng(u64 engineFlag);
s32 cmdqMdpClockOff(u64 engineFlag);
void cmdqMdpInitialSetting(void);

u32 cmdq_mdp_rdma_get_reg_offset_src_addr(void);
u32 cmdq_mdp_wrot_get_reg_offset_dst_addr(void);
u32 cmdq_mdp_wdma_get_reg_offset_dst_addr(void);
u64 cmdq_mdp_get_engine_group_bits(u32 engine_group);
void cmdq_mdp_enable_common_clock(bool enable);
void cmdq_mdp_check_hw_status(struct cmdqRecStruct *handle);
#ifdef CMDQ_SECURE_PATH_SUPPORT
u64 cmdq_mdp_get_secure_engine(u64 engine_flags);
#endif

u32 cmdq_mdp_get_hw_reg(enum MDP_ENG_BASE base, u16 offset);
u32 cmdq_mdp_get_hw_port(enum MDP_ENG_BASE base);

long cmdq_mdp_get_module_base_VA_MDP_WROT0(void);

#endif				/* __CMDQ_MDP_COMMON_H__ */
