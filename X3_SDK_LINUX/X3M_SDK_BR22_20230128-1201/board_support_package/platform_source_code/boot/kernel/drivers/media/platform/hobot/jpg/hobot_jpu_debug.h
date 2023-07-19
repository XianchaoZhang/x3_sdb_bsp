/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/
#ifndef __HOBOT_JPU_DEBUG_H__
#define __HOBOT_JPU_DEBUG_H__

#include <linux/kernel.h>

#define DEBUG

#ifdef DEBUG
extern int jpu_debug_flag;

#define jpu_debug(level, fmt, args...)				\
	do {							\
		if (jpu_debug_flag >= level)				\
			pr_info("[JPUDRV]%s:%d: " fmt,	\
				__func__, __LINE__, ##args);	\
		else								\
			pr_debug("[JPUDRV]%s:%d: " fmt,	\
				__func__, __LINE__, ##args);	\
	} while (0)
#else
#define jpu_debug(level, fmt, args...)
#endif

#define jpu_debug_enter() jpu_debug(5, "enter\n")
#define jpu_debug_leave() jpu_debug(5, "leave\n")

#define jpu_err(fmt, args...)				\
		pr_err("[JPUDRV]%s:%d: " fmt,		\
			__func__, __LINE__, ##args);

#define jpu_info(fmt, args...)				\
		pr_info("[JPUDRV]%s:%d: " fmt,		\
			__func__, __LINE__, ##args);

#define jpu_err_dev(fmt, args...)			\
	do {						\
		pr_err("[JPUDRV][d:%d] %s:%d: " fmt,	\
			dev->id,			\
		       __func__, __LINE__, ##args);	\
	} while (0)

#define jpu_info_dev(fmt, args...)			\
	do {						\
		pr_info("[JPUDRV][d:%d] %s:%d: " fmt,	\
			dev->id,			\
			__func__, __LINE__, ##args);	\
	} while (0)

#endif /* HOBOT_JPU_DEBUG_H */
