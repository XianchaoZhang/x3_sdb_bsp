#ifndef __VIO_FUNC_H__
#define __VIO_FUNC_H__

#include "common.h"
#include "list_func.h"

int get_yuv_pipe_info(uint32_t pipe_line, uint32_t channel,
	uint32_t *width, uint32_t *height);
int get_raw_buf(struct buf_info *data);
int get_yuv_buf(struct buf_info *data);
void free_raw_buf(struct buf_info *data);
void free_yuv_buf(struct buf_info *data);
int init_process(uint32_t gdc_rotation);
void deinit_process(void);
int get_gdc_data(struct buf_info *data, hb_vio_buffer_t *dst_buf);
void free_gdc_data(uint32_t pipe_line, hb_vio_buffer_t *dst_buf);

#endif
