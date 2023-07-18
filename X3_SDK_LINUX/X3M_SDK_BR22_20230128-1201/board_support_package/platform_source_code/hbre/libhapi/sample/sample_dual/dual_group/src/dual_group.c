/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>
#include <pthread.h>
#include <sys/stat.h>
#include <time.h>
#include <sys/time.h>
#include <signal.h>
#include <fcntl.h>
#include <ctype.h>

#include "hb_vin_api.h"
#include "hb_mipi_api.h"
#include "hb_vio_interface.h"

#include "hb_comm_venc.h"
#include "hb_venc.h"
#include "hb_vdec.h"
#include "hb_vp_api.h"
#include "dual_common.h"
#include "hb_vps_api.h"

time_t start_time, now;
time_t run_time = 10000000;
time_t end_time = 0;
int g_vin_fd[MAX_SENSOR_NUM];
int sensorId[MAX_SENSOR_NUM];
int mipiIdx[MAX_MIPIID_NUM];
int bus[MAX_SENSOR_NUM];
int port[MAX_SENSOR_NUM];
int serdes_index[MAX_SENSOR_NUM];
int serdes_port[MAX_SENSOR_NUM];
int groupMask = 3;
int need_clk;
int raw_dump;
int yuv_dump;
int need_cam;
int sample_mode = 0;
int data_type;
int vin_vps_mode[MAX_SENSOR_NUM];
int need_grp_rotate;
int need_chn_rotate;
int need_ipu = 64;
int need_pym  = 64;
int vps_dump;
int need_md;
int vc_num = 0;
int need_dol2 = 0;

int g_bindflag = 1;
int g_save_flag = 0;
int g_exit = 0;
int g_venc_flag = 1;
int g_osd_flag = 31;  // 0x3f;
int g_4kmode = 0;
int g_iar_enable = 1;
int g_bpu_usesample = 1;
int g_use_ldc = 0;
int g_set_qos = 0;
int g_use_ipu = 0;
int g_testpattern_fps = 30;
int g_chanage_res = 100000000;
int g_use_x3clock;
char* g_ipu_feedback = NULL;

