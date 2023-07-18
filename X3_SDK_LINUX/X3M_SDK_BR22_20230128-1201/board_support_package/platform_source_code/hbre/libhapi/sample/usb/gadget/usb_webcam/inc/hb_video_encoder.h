/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * hb_video_encoder.h
 *	hobot video encoder api functions
 *
 * Copyright (C) 2019 Horizon Robotics, Inc.
 *
 * Contact: jianghe xu<jianghe.xu@horizon.ai>
 */

#ifndef _HB_VIDEO_ENCODER_H__
#define _HB_VIDEO_ENCODER_H__

#include <pthread.h>
#include "hb_venc.h"

typedef enum {
	VENC_INVALID = 0,
	VENC_H264,
	VENC_H265,
	VENC_MJPEG,
} venc_type;

typedef struct venc_single_buffer {
	VIDEO_STREAM_S vstream;

	int used;
	pthread_mutex_t mutex;
} venc_single_buffer;

typedef struct venc_context {
	pthread_t pym_to_venc_pid;
	pthread_t venc_to_uvc_pid;
	venc_type type;
	venc_single_buffer *mono_venc_q;
	int venc_chn;
	int width;
	int height;
	int bitrate;
	int vps_grp;
	int vps_chn;
	int quit;
} venc_context;

/* hb video function definition */
venc_context *hb_video_encoder_alloc_context();
int hb_video_encoder_init(venc_context *vctx);
int hb_video_encoder_start(venc_context *vctx);
int hb_video_encoder_stop(venc_context *vctx);
void hb_video_encoder_deinit(venc_context *vctx);

#endif /* _HB_VIDEO_ENCODER_H__ */
