/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2019 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef __USB_CAMERA_H__
#define __USB_CAMERA_H__

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif				/* __cplusplus */


#define MAX_SENSOR_NUM 6
#define MAX_MIPIID_NUM 4
#define MAX_ID_NUM 32
#define BIT(n)  (1UL << (n))

extern int g_exit;
extern int g_bindflag;
extern int g_save_flag;

extern time_t start_time, now;
extern time_t run_time;
extern time_t end_time;
extern int sensorId[];
extern int mipiIdx[];
extern int bus[];
extern int port[];
extern int serdes_index[];
extern int serdes_port[];
extern int groupMask;

extern int sample_mode;
extern int data_type;
extern int vin_vps_mode[];
extern int need_grp_rotate;
extern int need_chn_rotate;
extern int need_ipu;
extern int need_pym;
extern int vps_dump;
extern int need_md;
extern int vc_num;
extern int need_dol2;
extern int g_venc_flag;
extern int g_osd_flag;
extern int g_4kmode;
extern int g_iar_enable;
extern int g_bpu_usesample;
extern int g_use_ldc;
extern int g_set_qos;
extern int g_use_ipu;
extern int g_testpattern_fps;
extern int g_chanage_res;
extern char* g_ipu_feedback;
extern int g_use_x3clock;


typedef enum {
	VENC_INVALID = 0,
	VENC_H264,
	VENC_H265,
	VENC_MJPEG,
} venc_type;

typedef struct {
	int veChn;
	int type;
	int width;
	int height;
	int bitrate;
	int vpsGrp;
	int vpsChn;
	int quit;
} vencParam;

int dual_singlepipe_venc_init();
int dual_singlepipe_venc_deinit();
void dual_venc_pthread_start(void);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif				/* End of #ifdef __cplusplus */
#endif	/* __USB_CAMERA_H__ */
