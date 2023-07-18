#ifndef __SERVER_CMD_H__
#define __SERVER_CMD_H__

#include "common.h"
#include <stdint.h>

#define MAX_PARAM_BUF_SIZE 512

struct param_buf {
	uint32_t param_id;
	uint32_t param_data;
};

struct send_buf {
	uint32_t param_num;
	struct param_buf *param;
};

struct cmd_header {
	uint32_t length;
	uint8_t type;//raw or yuv or jpg or video
	uint8_t format;//raw: raw8 raw10 raw12 raw14 raw16 yuv:yuv420 yuv444
	uint32_t width;
	uint32_t height;
	uint32_t stride;
	uint32_t frame_id;
	uint32_t frame_plane;//dol
	uint8_t code_type;//compression mode h264,h265
};

struct tranfer_info {
	uint8_t tcp_open;
	uint8_t raw_enable;
	uint8_t raw_serial_num;
	uint8_t yuv_enable;
	uint8_t yuv_serial_num;
	uint8_t jepg_enable;
	uint8_t video_enable;
	uint8_t video_code;
	uint16_t bit_stream;
	uint16_t fream_interval;
	uint16_t pipe_line;
	uint16_t channel_id;
	struct param_buf video_cfg;
};


#endif
