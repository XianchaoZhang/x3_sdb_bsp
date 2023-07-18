/*************************************************************************
 *                     COPYRIGHT NOTICE
 *            Copyright 2019 Horizon Robotics, Inc.
 *                   All rights reserved.
 *************************************************************************/
#ifndef __HB_ISP_H__
#define __HB_ISP_H__

#include <stdint.h>
#include <stdbool.h>
#define HB_ISP_MAX_AWB_ZONES (33 * 33)
#define HB_ISP_FULL_HISTOGRAM_SIZE 1024

// copy from kernel - v4l2_dev/app/ctxsv/isp_ctxsv.h
#define CFG_NODE_SIZE   (0x1c310 - 0xab6c + 4)
#define CTX_SIZE		CFG_NODE_SIZE
#define CTX_OFFSET      0xab6c

#define AWB_NODE_SIZE   (33 * 33 * 8)
#define AE_NODE_SIZE    (HB_ISP_FULL_HISTOGRAM_SIZE * 4)
#define AF_NODE_SIZE   (33 * 33 * 8)
#define AE_5BIN_NODE_SIZE   (33 * 33 * 8)
#define LUMVAR_NODE_SIZE (32 * 16 * 4)

typedef enum {
	NORMAL_MODE = 1,
	DOL2_MODE,
	DOL3_MODE,
	DOL4_MODE,
	PWL_MODE,
	INVAILD_MODE,
} sensor_mode_e;

typedef enum {
	BAYER_RGGB = 0,
	BAYER_GRBG,
	BAYER_GBRG,
	BAYER_BGGR,
	MONOCHROME,
} sensor_cfa_pattern_e;

typedef enum {
	ISP_CTX = 0,
	ISP_AE,
	ISP_AWB,
	ISP_AF,
	ISP_AE_5BIN,
	ISP_LUMVAR,
	TYPE_MAX,
} isp_info_type_e;

typedef enum {
	AWB_AUTO = 0x35,
	AWB_MANUAL,
	AWB_DAY_LIGHT,
	AWB_CLOUDY,
	AWB_INCANDESCENT,
	AWB_FLOURESCENT,
	AWB_TWILIGHT,
	AWB_SHADE,
	AWB_WARM_FLOURESCENT,
} isp_awb_mode_e;

typedef struct {
	uint16_t rg;
	uint16_t bg;
	uint32_t sum;
} isp_awb_statistics_s;

typedef struct {
	uint32_t frame_id;
	uint64_t timestamp;
	uint16_t crc16;
	void *ptr;
	uint32_t len;
} isp_context_t;

typedef struct{
	bool crc_en;
	void *data;
	uint32_t len;
	uint32_t frame_id;
	uint64_t timestamp;
	uint32_t  buf_idx;
} isp_statistics_t;

typedef struct {
	uint32_t ae_exposure;
	uint32_t again;
	uint32_t dgain;
	uint32_t ispgain;
	uint32_t sys_exposure;
	uint32_t status_info_exp_log2;
	uint32_t cur_lux;
} ae_info_t;

typedef struct {
	uint32_t rgain;
	uint32_t bgain;
	uint32_t cct;
} awb_info_t;

typedef struct {
	uint8_t h;
	uint8_t v;
} isp_zone_info_t;

typedef struct {
	uint32_t frame_id;
	uint64_t timestamp;
	ae_info_t ae_info;
	awb_info_t awb_info;
	isp_context_t *ptx;
	void *ae_ptr;
	void *ae_5bin_ptr;
	void *lumvar_ptr;
	void *awb_ptr;
	void *af_ptr;
} isp_info_t;

extern int hb_isp_iridix_ctrl(uint32_t provider_ctx_id, uint32_t user_ctx_id, uint8_t flag);
extern int hb_isp_get_ae_statistics(uint32_t ctx_id, isp_statistics_t *ae_statistics, int time_out);
extern int hb_isp_get_awb_statistics(uint32_t ctx_id, isp_statistics_t *awb_statistics, int time_out);
extern int hb_isp_release_ae_statistics(uint32_t ctx_id, isp_statistics_t *ae_statistics);
extern int hb_isp_release_awb_statistics(uint32_t ctx_id, isp_statistics_t *awb_statistics);
extern int hb_isp_get_context(uint32_t ctx_id, isp_context_t *ptx);
extern int hb_isp_set_context(uint32_t ctx_id, isp_context_t *ptx);
extern int hb_isp_dev_init(void);
extern int hb_isp_dev_deinit(void);
extern int hb_isp_get_2a_info(uint32_t pipeline_id, isp_info_t *isp_info, int time_out);
extern int hb_isp_get_2a_info_conditional(uint32_t pipeline_id, isp_info_t *isp_info, int type, int frame_id, int time_out);
extern int hb_isp_statistics_ctrl(uint32_t pipeline_id, uint8_t type, int flag); // flag = 0 off, flag = 1 on
extern int hb_isp_get_zone_info(uint32_t pipeline_id, uint8_t type, isp_zone_info_t *zoneinfo);
extern int hb_isp_set_zone_info(uint32_t pipeline_id, uint8_t type, isp_zone_info_t *zoneinfo);
extern int hb_isp_set_ae_info(uint32_t pipeline_id, const ae_info_t *ae_info);
extern int hb_isp_get_ae_info(uint32_t pipeline_id, ae_info_t *ae_info);
extern int hb_isp_set_awb_info(uint32_t pipeline_id, const awb_info_t *awb_info);
extern int hb_isp_get_awb_info(uint32_t pipeline_id, awb_info_t *awb_info);
extern int hb_isp_get_version(uint32_t pipeline_id, char* isp_ver, char* algo_ver, char* calib_ver);
extern int hb_isp_get_awb_mode(uint32_t ctx_id, isp_awb_mode_e *mode);
extern int hb_isp_set_awb_mode(uint32_t ctx_id, isp_awb_mode_e mode);

#endif
