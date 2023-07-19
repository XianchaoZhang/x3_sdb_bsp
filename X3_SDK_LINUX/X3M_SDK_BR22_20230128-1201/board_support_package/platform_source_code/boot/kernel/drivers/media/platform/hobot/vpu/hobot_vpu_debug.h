/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/
#ifndef __HOBOT_VPU_DEBUG_H__
#define __HOBOT_VPU_DEBUG_H__

#include <linux/kernel.h>

#define DEBUG

#ifdef DEBUG
extern int vpu_debug_flag;

#define vpu_debug(level, fmt, args...)				\
	do {							\
		if (vpu_debug_flag >= level)				\
			pr_info("[VPUDRV]%s:%d: " fmt,	\
				__func__, __LINE__, ##args);	\
		else								\
			pr_debug("[VPUDRV]%s:%d: " fmt,	\
				__func__, __LINE__, ##args);	\
	} while (0)
#else
#define vpu_debug(level, fmt, args...)
#endif

#define vpu_debug_enter() vpu_debug(5, "enter\n")
#define vpu_debug_leave() vpu_debug(5, "leave\n")

#define vpu_err(fmt, args...)				\
	do {						\
		pr_err("[VPUDRV]%s:%d: " fmt,		\
		       __func__, __LINE__, ##args);	\
	} while (0)

#define vpu_err_dev(fmt, args...)			\
	do {						\
		pr_err("[VPUDRV][d:%d] %s:%d: " fmt,	\
			dev->id,			\
		       __func__, __LINE__, ##args);	\
	} while (0)

#define vpu_info(fmt, args...)				\
		do {						\
			pr_info("[VPUDRV]%s:%d: " fmt,		\
				   __func__, __LINE__, ##args); \
		} while (0)

#define vpu_info_dev(fmt, args...)			\
	do {						\
		pr_info(KERN_INFO "[VPUDRV][d:%d] %s:%d: " fmt,	\
			dev->id,			\
			__func__, __LINE__, ##args);	\
	} while (0)

#endif /* __HOBOT_VPU_DEBUG_H__ */
