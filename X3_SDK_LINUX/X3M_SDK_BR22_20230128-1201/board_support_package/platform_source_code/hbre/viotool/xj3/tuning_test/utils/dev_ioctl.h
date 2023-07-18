/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef HB_X2A_VIO_DEV_IOCTL_H
#define HB_X2A_VIO_DEV_IOCTL_H
#include <stdint.h>
#include <poll.h>
#include "../inc/hb_vio_interface.h"
#include "./hb_vio_buffer_mgr.h"

#define DEV_POLL_TIMEOUT 200 //ms

struct special_buffer {
	uint32_t ds_y_addr[24];
	uint32_t ds_uv_addr[24];
	uint32_t us_y_addr[6];
	uint32_t us_uv_addr[6];
};

struct frame_info {
	uint32_t frame_id;
	uint64_t timestamps;
	struct timeval tv;
	int format;
	int height;
	int width;
	uint32_t addr[8];
	int bufferindex;
	int planes;
	struct special_buffer spec;
	int ion_share_fd[HB_VIO_BUFFER_MAX_PLANES];
};

//tool func
void print_frame_info(struct frame_info *info);
int dev_node_dqbuf(int fd, uint64_t cmd, struct frame_info *info);
int dev_node_qbuf(int fd, uint64_t cmd, hb_vio_buffer_t *buf);
buf_node_t *entity_node_dqbuf(int fd, buffer_mgr_t * buf_mgr,
								uint64_t dev_cmd, buffer_state_e buf_queue);
int entity_put_done_buf(buffer_mgr_t * mgr, hb_vio_buffer_t * buf,
				int fd, uint64_t dev_cmd);
int dev_get_buf_timeout(buffer_mgr_t * mgr,
								buffer_state_e buf_queue, int timeout);
int dev_buf_handle(buffer_mgr_t *mgr, buf_node_t * buf_node);
void frame_info_to_buf(image_info_t * to_buf, struct frame_info *info);
void * frame_info_to_oth_idx(buffer_mgr_t *mgr, struct frame_info *info);

#endif //HB_X2A_VIO_DEV_IOCTL_H
