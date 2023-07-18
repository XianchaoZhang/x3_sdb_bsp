/*
 *    COPYRIGHT NOTICE
 *    Copyright 2020 Horizon Robotics, Inc.
 *    All rights reserved.
 */
#ifndef __DIAG_SERVICE_DIAG_LOG_H__
#define __DIAG_SERVICE_DIAG_LOG_H__
#include <stdint.h>
#include <linux/types.h>
#include <log.h>

#define STRINGIZE_NO_EXPANSION(x) #x
#define STRINGIZE(x) STRINGIZE_NO_EXPANSION(x)
#define HERE __FILE__ ":" STRINGIZE(__LINE__)
#define L_DIAG_INFO "[DIAG_INFO][" HERE "] "
#define L_DIAG_WARN "[DIAG_WARN]" HERE "] "
#define L_DIAG_ERROR "[DIAG_ERROR][" HERE "] "
#define L_DIAG_DEBUG "[DIAG_DEBUG][" HERE "] "

void DIAG_PRINT(const char *fmt, ...);
void DIAG_LOGERR(const char *fmt, ...);
void DIAG_LOGWARN(const char *fmt, ...);
void DIAG_LOGINFO(const char *fmt, ...);
void DIAG_LOGDEBUG(int32_t level, const char *fmt, ...);
void DIAG_print(char *level, const char *fmt, ...);

static inline int32_t check_debug_level(void)
{
	static int32_t debug_flag = -1;
	const char *dbg_env;
	int32_t ret;

	if (debug_flag >= 0) {
		return debug_flag;
	} else {
		dbg_env = getenv("DIAG_DEBUG_FLAG");
		if (dbg_env != NULL) {
			ret = atoi(dbg_env);
			if (ret <= 0) {
				debug_flag = 0;
			} else {
				debug_flag = ret;
			}
		} else {
			debug_flag = 0;
		}
	}

	return debug_flag;
}

#ifndef pr_fmt
#define pr_fmt(fmt)             fmt
#endif

#define DIAG_LOGERR(fmt, ...)                               \
	do {                                                            \
		fprintf(stderr, L_DIAG_ERROR "" pr_fmt(fmt), ##__VA_ARGS__);          \
		ALOGE(L_DIAG_ERROR "" fmt, ##__VA_ARGS__); \
	} while (0);

#define DIAG_LOGWARN(fmt, ...)                               \
	do {                                                            \
		fprintf(stderr, L_DIAG_WARN "" pr_fmt(fmt), ##__VA_ARGS__);          \
		ALOGE(L_DIAG_WARN "" fmt, ##__VA_ARGS__); \
	} while (0);

#define DIAG_LOGINFO(fmt, ...)                               \
	do {                                                            \
		fprintf(stdout, L_DIAG_INFO "" pr_fmt(fmt), ##__VA_ARGS__);          \
		ALOGI(L_DIAG_INFO "" fmt, ##__VA_ARGS__); \
	} while (0);

#define DIAG_LOGDEBUG(fmt, ...)                              \
	do {                                                            \
		int loglevel = check_debug_level();          \
		if (loglevel > 0) {            \
			fprintf(stdout, L_DIAG_DEBUG "" pr_fmt(fmt), ##__VA_ARGS__);         \
			ALOGD(L_DIAG_DEBUG "" fmt, ##__VA_ARGS__);              \
		}        \
	} while (0);

#endif
