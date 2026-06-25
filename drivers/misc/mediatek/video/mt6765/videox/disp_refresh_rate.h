/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __MTK_DISP_REFRESH_RATE_H__
#define __MTK_DISP_REFRESH_RATE_H__

/*
 * Override these from the build command line when tuning a panel, for example:
 * KCFLAGS=-DMTK_DISP_DEFAULT_REFRESH_RATE_HZ=60
 */
#ifndef MTK_DISP_MIN_REFRESH_RATE_HZ
#define MTK_DISP_MIN_REFRESH_RATE_HZ		60U
#endif

#ifndef MTK_DISP_DEFAULT_REFRESH_RATE_HZ
#define MTK_DISP_DEFAULT_REFRESH_RATE_HZ	64U
#endif

#ifndef MTK_DISP_MAX_REFRESH_RATE_HZ
#define MTK_DISP_MAX_REFRESH_RATE_HZ		120U
#endif

#define MTK_DISP_REFRESH_RATE_SCALE		100U
#define MTK_DISP_DEFAULT_REFRESH_RATE_X100	\
	(MTK_DISP_DEFAULT_REFRESH_RATE_HZ * MTK_DISP_REFRESH_RATE_SCALE)

#if MTK_DISP_MIN_REFRESH_RATE_HZ > MTK_DISP_DEFAULT_REFRESH_RATE_HZ
#error "MTK display minimum refresh rate exceeds the default"
#endif

#if MTK_DISP_DEFAULT_REFRESH_RATE_HZ > MTK_DISP_MAX_REFRESH_RATE_HZ
#error "MTK display default refresh rate exceeds the maximum"
#endif

#endif
