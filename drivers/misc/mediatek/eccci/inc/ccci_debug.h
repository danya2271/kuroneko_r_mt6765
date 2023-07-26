/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2016 MediaTek Inc.
 */

#ifndef __CCCI_DEBUG_H__
#define __CCCI_DEBUG_H__

/* log tag defination */
#define CORE "cor"
#define BM "bfm"
#define FSM "fsm"
#define PORT "pot"
#define NET "net"
#define CHAR "chr"
#define IPC "ipc"
#define RPC "rpc"
#define SYS "sys"
#define SMEM "shm"
#define UDC "udc"

enum {
	CCCI_LOG_ALL_UART = 1,
	CCCI_LOG_ALL_MOBILE,
	CCCI_LOG_CRITICAL_UART,
	CCCI_LOG_CRITICAL_MOBILE,
	CCCI_LOG_ALL_OFF,
};

extern unsigned int ccci_debug_enable; /* Exported by CCCI core */
extern int ccci_log_write(const char *fmt, ...); /* Exported by CCCI Util */
extern int ccci_dump_write(int md_id, int buf_type,
	unsigned int flag, const char *fmt, ...);

/*****************************************************************************
 ** CCCI dump log define start ****************
 ****************************************************************************/
/*--------------------------------------------------------------------------*/
/* This is used for log to mobile log or uart log */
#define CCCI_LEGACY_DBG_LOG(idx, tag, fmt, args...) do {} while (0)

#define CCCI_LEGACY_ALWAYS_LOG(idx, tag, fmt, args...) do {} while (0)

#define CCCI_LEGACY_ERR_LOG(idx, tag, fmt, args...) do {} while (0)

/*--------------------------------------------------------------------------*/
/* This log is used for driver init and part of first boot up log */
#define CCCI_INIT_LOG(idx, tag, fmt, args...) do {} while (0)

/* This log is used for save runtime data */
/* The first line with time stamp */
#define CCCI_BOOTUP_LOG(idx, tag, fmt, args...) do {} while (0)

#define CCCI_BOOTUP_DUMP_LOG(idx, tag, fmt, args...) do {} while (0)

/* This log is used for modem boot up log and event */
#define CCCI_NORMAL_LOG(idx, tag, fmt, args...) do {} while (0)

#define CCCI_NOTICE_LOG(idx, tag, fmt, args...) do {} while (0)

#define CCCI_ERROR_LOG(idx, tag, fmt, args...) do {} while (0)

#define CCCI_DEBUG_LOG(idx, tag, fmt, args...) do {} while (0)

/* This log is used for periodic log */
#define CCCI_REPEAT_LOG(idx, tag, fmt, args...) do {} while (0)

/* This log is used for memory dump */
#define CCCI_MEM_LOG_TAG(idx, tag, fmt, args...) do {} while (0)

#define CCCI_MEM_LOG(idx, tag, fmt, args...) do {} while (0)

/* This log is used for history dump */
#define CCCI_HISTORY_LOG(idx, tag, fmt, args...) do {} while (0)
#define CCCI_HISTORY_TAG_LOG(idx, tag, fmt, args...) do {} while (0)
#define CCCI_BUF_LOG_TAG(idx, buf_type, tag, fmt, args...) do {} while (0)

/****************************************************************************
 ** CCCI dump log define end ****************
 ****************************************************************************/

/* #define CLDMA_TRACE */
/* #define PORT_NET_TRACE */
//#define CCCI_SKB_TRACE
/* #define CCCI_BM_TRACE */

#endif				/* __CCCI_DEBUG_H__ */
