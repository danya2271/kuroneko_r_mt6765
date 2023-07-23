/*
 * Author: andip71, 01.09.2017
 *
 * Version 1.1.0
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define BOEFFLA_WL_BLOCKER_VERSION	"1.1.0"

#define LIST_WL_DEFAULT				"wlan;wlan_wow_wl;wlan_extscan_wl;netmgr_wl;NETLINK;IPA_WS;[timerfd];wlan_ipa;wlan_pno_wl;wcnss_filter_lock;alarmtimer;ccci_imsc;isp_lock_wakelock;PowerManager.SuspendLockout;gpswakelock;situation_wakelock-3;cmdq_wakelock;pri_disp_wakelock;ttyC2;ccmni_md1;wmtFuncCtrl;ccci_aud;ccci_ipc_3;PTIM_wakelock;situation_wakelock-2;event4;mt-pmic:mt635x-auxadc;ps_wake_lock"

#define LENGTH_LIST_WL				2048
#define LENGTH_LIST_WL_DEFAULT		2048
#define LENGTH_LIST_WL_SEARCH		LENGTH_LIST_WL + LENGTH_LIST_WL_DEFAULT + 5
