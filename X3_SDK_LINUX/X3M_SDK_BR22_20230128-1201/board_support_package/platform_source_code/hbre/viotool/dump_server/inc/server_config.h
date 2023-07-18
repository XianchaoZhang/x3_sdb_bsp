#ifndef __SERVER_CONFIG_H__
#define __SERVER_CONFIG_H__

#include "common.h"
#include <stdint.h>


int dump_config_init(const char *config_file_path);
char *dump_config_get_server_ip(void);
short dump_config_get_server_port(void);
char *dump_config_get_call_fun(void);
char *dump_config_get_test_case_name(void);
char *dump_config_get_cfg_file(void);
char *dump_config_get_log_file(void);
char *dump_config_get_patch_file(void);
char *dump_config_get_debug(void);
char *dump_config_get_test_op_code(void);
char *dump_config_get_frame_cnt(void);
char *dump_config_get_md5_file(void);
char *dump_config_get_golden_file(void);
void raw_config_info(uint32_t *raw_enable, uint32_t *pipe_line);
void yuv_config_info(uint32_t *yuv_enable, uint32_t *pipe_line,
	uint32_t *yuv_channel);
void video_config_info(uint32_t *video_enable,
	uint32_t *pipe_line, uint32_t *video_channel);
void jepg_config_info(uint32_t *jepg_enable,
	uint32_t *pipe_line, uint32_t *jepg_channel);


#endif // DUMPCONFIG_H
