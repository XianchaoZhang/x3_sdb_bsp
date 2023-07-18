#ifndef __SERVER_CORE_H
#define __SERVER_CORE_H

#include "common.h"

int dump_server_core_start_services(void);
void dump_server_core_stop_services(void);
int send_data(void *ptr, uint32_t len);
void err_handler(uint32_t err_flag);
void acquire_mutex(void);
void release_mutex(void);
uint32_t send_yuv_enable(void);
uint32_t send_jepg_enable(void);
uint32_t send_video_enable(void);
uint32_t send_raw_enable(void);
uint32_t get_raw_serial_num(void);
uint32_t get_yuv_serial_num(void);
void get_yuv_pipeline_info(uint32_t *pipe_line, uint32_t *channel);
void get_raw_pipeline_info(uint32_t *pipe_line);


#endif // DUMPSERVERCORE_H
