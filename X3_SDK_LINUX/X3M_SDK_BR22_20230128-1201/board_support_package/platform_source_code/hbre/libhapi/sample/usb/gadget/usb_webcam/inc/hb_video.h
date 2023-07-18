/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * hb_video.h
 *	hobot video module functions(sensor->isp->vio->venc)
 *
 * Copyright (C) 2019 Horizon Robotics, Inc.
 *
 * Contact: jianghe xu<jianghe.xu@horizon.ai>
 */

#ifndef _HB_VIDEO_H_
#define _HB_VIDEO_H_

#include <stdio.h>
#include <stdint.h>
#include "hb_vin_api.h"
#include "hb_vps_api.h"
#include "hb_mipi_api.h"
#include "hb_video_encoder.h"
#include "hb_vio_interface.h"

#define UVC_DEBUG
// #undef UVC_DEBUG	// uncomment it to disable uvc debug
#define UVC_DUMP
// #undef UVC_DUMP		// uncomment it to disable uvc dump debug

#define MAX_SENSOR_NUM  6
#define MAX_MIPI_NUM  4
#define MAX_ID_NUM 4
#define MAX_PLANE 4

typedef enum {
	VIDEO_FMT_INVALID = 0,
	VIDEO_FMT_YUY2,
	VIDEO_FMT_NV12,
	VIDEO_FMT_H264,
	VIDEO_FMT_H265,
	VIDEO_FMT_MJPEG,
} video_format;

typedef enum mipi_sensor_type {
	SENSOR_INVALID = -1,				// -1
	SENSOR_SIF_TEST_PATTERN0_1080P,			// 0
	SENSOR_SIF_TEST_PATTERN_12M_RAW12_8M,		// 1
	SENSOR_SIF_TEST_PATTERN_12M_RAW12_12M,		// 2
	SENSOR_SIF_TEST_PATTERN_8M_RAW12,		// 3
	SENSOR_IMX327_30FPS_1952P_RAW12_LINEAR,		// 4
	SENSOR_OS8A10_30FPS_3840P_RAW10_LINEAR,		// 5
	SENSOR_OV10635_30FPS_720p_954_YUV,		// 6
	SENSOR_OV10635_30FPS_720p_960_YUV,		// 7
	SENSOR_AR0144_30FPS_720P_RAW12_MONO,		// 8
	SENSOR_S5KGM1SP_30FPS_4000x3000_RAW19,		// 9
	SENSOR_GC02M1B_25FPS_1600x1200_RAW8,		// 10
	SENSOR_F37_30FPS_1920x1080_RAW10,		// 11
	SENSOR_IMX307_30FPS_1080P_RAW12_LINEAR,		// 12
	SENSOR_AR0233_30FPS_1080P_RAW12_954_PWL,	// 13
	SENOSR_MAX,					// 14
} mipi_sensor_type;

typedef struct vin_attr_info {
	mipi_sensor_type sensor_type;
	const VIN_DEV_ATTR_S *dev_attr;
	const VIN_PIPE_ATTR_S *pipe_attr;
	const VIN_DIS_ATTR_S *dis_attr;
	const VIN_LDC_ATTR_S *ldc_attr;
	const MIPI_ATTR_S *mipi_attr;
	const MIPI_SENSOR_INFO_S *sensor_info;
} vin_attr_info;

typedef struct raw_single_buffer {
	pym_buffer_t pym_buffer;

	int used;
	pthread_mutex_t mutex;
} raw_single_buffer;

typedef struct video_context {
	pthread_t routine_id;
	video_format format;
	venc_context *venc_ctx;
	raw_single_buffer *mono_raw_q;
	uint32_t width;
	uint32_t height;
	uint32_t req_width;
	uint32_t req_height;
	uint8_t pipe_id_mask;
	uint8_t pipe_id_using;
	uint8_t need_clk;
	uint8_t use_pattern;
	uint8_t pattern_fps;
	uint8_t ipu_mask;
	uint8_t pym_mask;
	uint8_t has_venc;
	uint8_t sensor_id[MAX_SENSOR_NUM];
	uint8_t mipi_idx[MAX_MIPI_NUM];
	uint8_t bus_idx[MAX_SENSOR_NUM];
	uint8_t port_idx[MAX_SENSOR_NUM];
	uint8_t serdes_idx[MAX_SENSOR_NUM];
	uint8_t serdes_port[MAX_SENSOR_NUM];
	uint8_t vin_vps_mode[MAX_SENSOR_NUM];
	int quit;

	// helper temp buffer
	void *tmp_buffer;
	int buffer_size;

#ifdef UVC_DUMP
	int dump_fd;
#endif
} video_context;

/* hb video api functions */
video_context *hb_video_alloc_context();
void hb_video_free_context(video_context *video_ctx);

int hb_video_init(video_context *ctx);
int hb_video_prepare(video_context *ctx);
int hb_video_start(video_context *ctx);
int hb_video_stop(video_context *ctx);
void hb_video_finalize(video_context *ctx);
void hb_video_deinit(video_context *ctx);

/* helper function */
video_format fcc_to_video_format(unsigned int fcc);

#endif /* _HB_VIDEO_H_ */
