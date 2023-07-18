#ifndef __CODE_FUNC_H__
#define __CODE_FUNC_H__

#include "common.h"
#include "list_func.h"
#include "server_cmd.h"

int get_video_buf(struct buf_info *data);
int get_jepg_buf(struct buf_info *data);
int get_video_pps(struct buf_info *data);

void video_func_deinit(void);
int video_func_init(uint32_t pipeline, uint32_t channel);
void set_video_param(uint32_t width, uint32_t height,
        uint32_t encode_mode, uint32_t bit_rate, uint32_t intra_period);
void set_dynamic_param(struct param_buf video_cfg);

int video_start(void);
int jepg_start(void);
void video_stop(void);
void jepg_stop(void);

#endif
