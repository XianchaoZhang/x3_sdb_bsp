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


#define MAX_SENSOR_NUM 8
#define MAX_MIPIID_NUM 8
#define MAX_ID_NUM 8
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
	
typedef enum HB_MIPI_SNS_TYPE_E
{
	SENSOR_ID_INVALID,
	IMX327_30FPS_1952P_RAW12_LINEAR,   // 1
	IMX327_30FPS_2228P_RAW12_DOL2,	   // 2
	AR0233_30FPS_1080P_RAW12_954_PWL,	 // 3
	AR0233_30FPS_1080P_RAW12_960_PWL,	 // 4
	AR0233_30FPS_1080P_RAW12_954_LINEAR,	 // 5
	AR0233_30FPS_1080P_RAW12_960_LINEAR,	// 6
	OS8A10_30FPS_3840P_RAW10_LINEAR,	 // 7
	OS8A10_30FPS_3840P_RAW10_DOL2,		// 8
	OV10635_30FPS_720p_954_YUV, 		// 9
	OV10635_30FPS_720p_960_YUV, 		// 10
	SIF_TEST_PATTERN0_1080P,			// 11
	FEED_BACK_RAW12_1952P,				// 12
	SIF_TEST_PATTERN_YUV_720P,			// 13
	SIF_TEST_PATTERN_12M_RAW12, 		// 14
	IMX327_30FPS_2228P_DOL2_TWO_LINEAR, 	// 15
	IMX327_15FPS_3609P_RAW12_DOL3,			// 16
	IMX327_15FPS_3609P_DOL3_THREE_LINEAR,	// 17
	IMX327_15FPS_3609P_DOL3_LEF_SEF2,		// 18
	SAMPLE_SENOSR_ID_MAX,
} MIPI_SNS_TYPE_E;


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
int hb_vot_init(void);
int hb_vot_deinit(void);
int dual_vin_vps_init();
int dual_vin_vps_deinit();
void parse_mipiIdx_func(char *optarg);
void parse_bus_func(char *optarg);
void parse_port_func(char *optarg);
void parse_sensorid_func(char *optarg);
void parse_serdes_port_func(char *optarg);
void parse_serdes_index_func(char *optarg);
void parse_vin_vps_mode_func(char *optarg);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif				/* End of #ifdef __cplusplus */
#endif	/* __USB_CAMERA_H__ */
