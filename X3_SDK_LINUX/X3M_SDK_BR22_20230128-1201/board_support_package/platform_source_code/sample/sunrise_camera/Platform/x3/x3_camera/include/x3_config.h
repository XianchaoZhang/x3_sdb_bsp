/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef X3_CONFIG_H_
#define X3_CONFIG_H_

typedef struct {
	char sensor_name[32];
	int venc_bitrate;
	int alog_id;
}x3_cfg_pileline_t;

typedef struct {
	int pipeline_num;
	x3_cfg_pileline_t pipelines[2];
} x3_cfg_ipc_t;

typedef struct {
	x3_cfg_pileline_t pipeline;
} x3_cfg_usb_cam_t;

typedef struct {
	int box_chns;
	int venc_bitrate;
	int alog_id;
} x3_cfg_box_t;

typedef struct {
	int solution_id;
	char solution_id_comment[128];
	char alog_id_comment[128];
	int disp_dev;
	x3_cfg_ipc_t ipc_solution;
	x3_cfg_usb_cam_t usb_cam_solution;
	x3_cfg_box_t box_solution;
} x3_cfg_t;

int x3_cfg_load_default_config();
int x3_cfg_load();
int x3_cfg_save();
char* x3_cfg_obj2string();
void x3_cfg_string2obj(char *in);

extern int g_x3_cfg_is_load;
extern x3_cfg_t g_x3_config;

#endif // X3_CONFIG_H_

