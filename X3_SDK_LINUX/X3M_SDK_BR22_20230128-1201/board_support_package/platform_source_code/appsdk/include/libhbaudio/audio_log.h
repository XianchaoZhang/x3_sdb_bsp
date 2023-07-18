/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef AUDIO_SERVER_INCLUDE_AUDIO_LOG_H_
#define AUDIO_SERVER_INCLUDE_AUDIO_LOG_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <linux/types.h>
#include <log.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define STRINGIZE_NO_EXPANSION(x) #x
#define STRINGIZE(x) STRINGIZE_NO_EXPANSION(x)
#define HERE __FILE__ ":" STRINGIZE(__LINE__)
#define L_PR_INFO "[INFO][" HERE "] "
#define L_PR_WARN "[WARN]" HERE "] "
#define L_PR_ERR "[ERROR][" HERE "] "
#define L_PR_DEBUG "[DEBUG][" HERE "] "

#define AUDIO_DEBUG_LEVEL        4
#define AUDIO_INFO_LEVEL         3
#define AUDIO_WARNING_LEVEL      2
#define AUDIO_ERROR_LEVEL        1

void pr_err(const char *fmt, ...);
void pr_warn(const char *fmt, ...);
void pr_info(const char *fmt, ...);
void pr_debug(int32_t level, const char *fmt, ...);

static inline int32_t check_debug_level(void)
{
        static int32_t debug_flag = -1;
        const char *dbg_env;
        int32_t ret;

        if (debug_flag >= 0) {
                return debug_flag;
        } else {
                dbg_env = getenv("AUDIO_LOGLEVEL");
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

#define pr_err(fmt, ...)                               \
        do {                                                            \
                fprintf(stderr, L_PR_ERR "" pr_fmt(fmt), ##__VA_ARGS__);          \
                ALOGE(L_PR_ERR "" fmt, ##__VA_ARGS__); \
        } while (0);

#define pr_warn(fmt, ...)                               \
        do {                                                            \
                int loglevel = check_debug_level();          \
                if (loglevel >= AUDIO_WARNING_LEVEL) {            \
                    fprintf(stderr, L_PR_WARN "" pr_fmt(fmt), ##__VA_ARGS__);          \
                    ALOGW(L_PR_WARN "" fmt, ##__VA_ARGS__); \
        }        \
        } while (0);

#define pr_info(fmt, ...)                               \
        do {                                                            \
        int loglevel = check_debug_level();          \
        if (loglevel >= AUDIO_INFO_LEVEL) {            \
                    fprintf(stdout, L_PR_INFO "" pr_fmt(fmt), ##__VA_ARGS__);          \
                    ALOGI(L_PR_INFO "" fmt, ##__VA_ARGS__); \
        }        \
        } while (0);

#define pr_debug(fmt, ...)                              \
        do {                                                            \
                int loglevel = check_debug_level();          \
                if (loglevel >= AUDIO_DEBUG_LEVEL) {            \
                        fprintf(stdout, L_PR_DEBUG "" pr_fmt(fmt), ##__VA_ARGS__);         \
                        ALOGD(L_PR_DEBUG "" fmt, ##__VA_ARGS__);              \
                }        \
        } while (0);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif  // AUDIO_SERVER_INCLUDE_AUDIO_LOG_H_
