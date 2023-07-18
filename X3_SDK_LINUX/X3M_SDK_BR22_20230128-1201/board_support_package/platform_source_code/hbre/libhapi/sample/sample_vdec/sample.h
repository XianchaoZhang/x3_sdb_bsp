/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2019 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef __SAMPLE_DAHUA_H
#define __SAMPLE_DAHUA_H

#define MAX_SENSOR_NUM 6
#define MAX_MIPIID_NUM 4
#define MAX_ID_NUM 4
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
// extern int need_clk;
// extern int raw_dump;
// extern int yuv_dump;
// extern int need_cam;
extern int sample_mode;
extern int data_type;
extern int vin_vps_mode;
extern int need_grp_rotate;
extern int need_chn_rotate;
extern int need_ipu;
extern int need_pym;
extern int vps_dump;
extern int need_md;

extern int g_venc_flag;
extern int g_osd_flag;
extern int g_4kmode;
extern char *g_vdecfilename[];
extern int g_width;
extern int g_height;

typedef enum HB_MIPI_SNS_TYPE_E
{
	SENSOR_ID_INVALID,
	IMX327_30FPS_1952P_RAW12_LINEAR,  // 1
	IMX327_30FPS_2228P_RAW12_DOL2,    // 2
	AR0233_30FPS_1080P_RAW12_954_PWL,	 // 3
	AR0233_30FPS_1080P_RAW12_960_PWL,	// 4
	OS8A10_25FPS_3840P_RAW10_LINEAR,	// 5
	OS8A10_25FPS_3840P_RAW10_DOL2,	 // 6
	OV10635_30FPS_720p_954_YUV,   // 7
	OV10635_30FPS_720p_960_YUV,   // 8
	SIF_TEST_PATTERN0_1080P,   // 9
	SIF_TEST_PATTERN0_1520P,   // 10
	SIF_TEST_PATTERN0_2160P,   // 11
	FEED_BACK_RAW12_1952P,    // 12
    SIF_TEST_PATTERN_YUV_720P,    // 13
	SAMPLE_SENOSR_ID_MAX,
} MIPI_SNS_TYPE_E;

typedef struct {
	int loop;
	int vdecchn;
	char fname[128];
} vdecParam;

typedef struct {
    int veChn;
    int type;
    int width;
    int height;
    int bitrate;
    int vpsGrp;
    int vpsChn;
	int quit;
}vencParam;

typedef struct {
    int rgnChn;
    int x;
    int y;
    int w;
    int h;
}rgnParam;

void venc_thread(void *vencpram);
int check_end(void);

void* sample_rgn_thread(void *rgnparams);
int hb_vin_vps_start(int pipeId);
void hb_vin_vps_stop(int pipeId);
int hb_vin_vps_init(int pipeId, int sensorId, int mipiIdx, uint32_t deseri_port, vencParam* vparam);
int hb_vin_vps_init_1(int pipeId, int sensorId, int mipiIdx, uint32_t deseri_port);
void hb_vin_vps_deinit(int pipeId, int sensorId);
int hb_sensor_init(int devId, int sensorId, int bus, int port, int mipiIdx, int sedres_index, int sedres_port);
int hb_mipi_init(int sensorId, int mipiIdx);
int hb_sensor_deinit(int devId);
int hb_mipi_deinit(int mipiIdx);
int hb_sensor_start(int devId);
int hb_mipi_start(int mipiIdx);
int hb_sensor_stop(int devId);
int hb_mipi_stop(int mipiIdx);
int sample_vin_bind_vps(int pipeId, int vin_vps_mode);
int sample_vin_unbind_vps(int pipeId, int vin_vps_mode);

void hb_vps_deinit(int pipeId);
int hb_vps_start(int pipeId);
void hb_vps_stop(int pipeId);
int hb_vps_init(int pipeId, int mode);
int sample_vps_bind_vot(int vpsGrp, int vpsChn, int votChn);
int sample_vps_bind_venc(int vpsGrp, int vpsChn, int vencChn);
int sample_vps_unbind_vot(int vpsGrp, int vpsChn, int votChn);
int sample_vps_unbind_venc(int vpsGrp, int vpsChn, int vencChn);

int sample_vot_wb_deinit();
int sample_vot_wb_init(int wb_src, int wb_format);
int sample_vot_deinit();
int sample_vot_init();
int sample_votwb_bind_venc(int venc_chn);
int sample_votwb_bind_vps(int vps_grp, int vps_chn);
int sample_votwb_unbind_venc(int venc_chn);
int sample_votwb_unbind_vps(int vps_grp, int vps_chn);


int sample_venc_common_init();
int sample_venc_common_deinit();
int sample_venc_init(int VeChn, int type, int width, int height, int bits);
int sample_venc_start(int VeChn);
int sample_venc_stop(int VeChn);
int sample_venc_deinit(int VeChn);

int sample_vdec_init(int p_enType, int vdecChn);
int sample_vdec_stop(int hb_VDEC_Chn);
int sample_vdec_deinit(int hb_VDEC_Chn);
void do_sync_decoding(void *param);
int sample_vdec_bind_vps(int vdecChn, int vpsGrp, int vpsChn);
int sample_vdec_bind_vot(int vdecChn, int votChn);
int sample_vdec_bind_venc(int vdecChn, int vencChn);
int sample_vdec_unbind_vps(int vdecChn, int vpsGrp, int vpsChn);
int sample_vdec_unbind_vot(int vdecChn, int votChn);
int sample_vdec_unbind_venc(int vdecChn, int vencChn);


int sample_singlepipe_5venc();
int sample_multpipe_multvenc();
int sample_vdec_vps_venc_vot();
int sample_jpeg_decoding();
int time_cost_ms(struct timeval *start, struct timeval *end);

#endif