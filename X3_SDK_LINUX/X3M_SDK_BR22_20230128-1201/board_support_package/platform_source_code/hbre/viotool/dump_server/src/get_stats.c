#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <fcntl.h>
#include <signal.h>
#include <pthread.h>

#define __USE_GNU
#include<sched.h>
#include<ctype.h>

#include "hb_isp_api.h"
#include "server_cmd.h"
#include "server_core.h"
#include "common.h"
#include "com_api.h"
#include "list_func.h"

extern int hb_isp_lut_rw(uint8_t chn, uint8_t dir, uint8_t id, void *data, uint16_t size, uint8_t bytewidth);
#define CALIBRATION_STATIC_WB                             0x00000014
#define COMMAND_SET                                       0x00000000
#define COMMAND_GET                                       0x00000001

pthread_t pid_stats;
static uint32_t run_stats = 0;
ISP_AWB_ATTR_S stAwbAttr;
typedef struct _awb_stats_info_s {
	struct cmd_header header_info;
	ISP_STATISTICS_AWB_ZONE_ATTR_S stats[33*33];
	ISP_AWB_POS_STATUS_S pos;
	ISP_MESH_RGBG_WEIGHT_S weight;
	ISP_ZONE_ATTR_S zone;
	uint16_t wb_info[4];
	uint32_t rgain;
	uint32_t bgain;
} awb_stats_info;

typedef struct _ae_stats_info_s {
	struct cmd_header header_info;
	uint32_t stats[1024];
} ae_stats_info;

typedef struct _ae5bin_stats_info_s {
	struct cmd_header header_info;
	ISP_STATISTICS_AE_5BIN_ZONE_ATTR_S stats[33*33];
	ISP_ZONE_ATTR_S zone;
} ae5bin_stats_info;

typedef struct _lumvar_stats_info_s {
	struct cmd_header header_info;
	ISP_STATISTICS_LUMVAR_ZONE_ATTR_S stats[32*16];
} lumvar_stats_info;

typedef struct _af_stats_info_s {
	struct cmd_header header_info;
	uint32_t stats[33 * 33 * 2];
	ISP_ZONE_ATTR_S zone;
} af_stats_info;

awb_stats_info awb_s;
ae_stats_info ae_s;
ae5bin_stats_info ae5bin_s;
lumvar_stats_info lumvar_s;
af_stats_info af_s;

int get_awb_stats_info(uint8_t pipe)
{
	int ret = 0;
	uint32_t i = 0;
	if (pipe >= 8) {
		vmon_err("pipe %d is max than 8!\n", pipe);
		return -1;
	}

	ret = HB_ISP_GetAwbZoneHist(pipe, awb_s.stats);
	if (ret == 0)
		ret = HB_ISP_GetAwbPosStatusAttr(pipe, &awb_s.pos);
	if (ret == 0)
		ret = HB_ISP_GetAwbRgBgWeightAttr(pipe, &awb_s.weight);
	if (ret == 0)
		ret = HB_ISP_GetAwbZoneInfo(pipe, &awb_s.zone);
	if (ret == 0) {
		ret = hb_isp_lut_rw(pipe, COMMAND_GET, CALIBRATION_STATIC_WB, awb_s.wb_info, sizeof(awb_s.wb_info), sizeof(uint16_t));
	}
	if (ret == 0) {
		stAwbAttr.enOpType = OP_TYPE_MANUAL;
		ret = HB_ISP_GetAwbAttr(pipe, &stAwbAttr);
		if (ret == 0) {
			awb_s.rgain = stAwbAttr.u16RGain;
			awb_s.bgain = stAwbAttr.u16BGain;
		} else {
			awb_s.rgain = 256;
			awb_s.bgain = 256;
		}
	}
	if (ret == 0) {
		awb_s.header_info.type = STATS_AWB_DATA;
		awb_s.header_info.height = 0;
		awb_s.header_info.stride = 0;
		awb_s.header_info.length = sizeof(awb_s.stats) + sizeof(awb_s.pos) +
			sizeof(awb_s.weight) + sizeof(awb_s.zone) + sizeof(awb_s.wb_info) + 2 * sizeof(awb_s.rgain);
		awb_s.header_info.frame_id = 0;
		awb_s.header_info.frame_plane = 0;
		awb_s.header_info.code_type = 0;
	}

	return ret;
}

int get_ae_stats_info(uint8_t pipe)
{
	int ret = 0;
	uint32_t i = 0;
	if (pipe >= 8) {
		vmon_err("pipe %d is max than 8!\n", pipe);
		return -1;
	}

	ret = HB_ISP_GetAeFullHist(pipe, ae_s.stats);
	if (ret == 0) {
		ae_s.header_info.type = STATS_AEfull_DATA;
		ae_s.header_info.height = 0;
		ae_s.header_info.stride = 0;
		ae_s.header_info.length = sizeof(ae_s.stats);
		ae_s.header_info.frame_id = 0;
		ae_s.header_info.frame_plane = 0;
		ae_s.header_info.code_type = 0;
	}

	return ret;
}

int get_af_stats_info(uint8_t pipe)
{
	int ret = 0;
	uint32_t i = 0;
	af_stats_data_t af_attr;
	if (pipe >= 8) {
		vmon_err("pipe %d is max than 8!\n", pipe);
		return -1;
	}
	af_attr.zones_stats = (uint32_t *)&af_s.stats[0];
	ret = HB_ISP_GetAfZoneHist(pipe, &af_attr);
	if (ret == 0) {
		ret = HB_ISP_GetAfZoneInfo(pipe, &af_s.zone);
	}
	if (ret == 0) {
		af_s.header_info.type = STATS_AF_DATA;
		af_s.header_info.height = 0;
		af_s.header_info.stride = 0;
		af_s.header_info.length = sizeof(af_s.stats) + sizeof(af_s.zone);
		af_s.header_info.frame_id = 0;
		af_s.header_info.frame_plane = 0;
		af_s.header_info.code_type = 0;
	}

	return ret;
}

int get_lumvar_stats_info(uint8_t pipe)
{
	int ret = 0;
	uint32_t i = 0;
	if (pipe >= 8) {
		vmon_err("pipe %d is max than 8!\n", pipe);
		return -1;
	}

	ret = HB_ISP_GetLumaZoneHist(pipe, lumvar_s.stats);
	if (ret == 0) {
		lumvar_s.header_info.type = STATS_LUMVAR_DATA;
		lumvar_s.header_info.height = 0;
		lumvar_s.header_info.stride = 0;
		lumvar_s.header_info.length = sizeof(lumvar_s.stats);
		lumvar_s.header_info.frame_id = 0;
		lumvar_s.header_info.frame_plane = 0;
		lumvar_s.header_info.code_type = 0;
	}

	return ret;
}

/**
 * @brief send yuv data to pc
 *
 * @param info
 * @param ptr y channel data info
 * @param size y channel size
 * @param ptr1 uv channel data info
 * @param size1 uv channel size
 * @return
 *   @retval
 */
static void send_statsdata(struct cmd_header *info, void *ptr,
	uint32_t size, void *ptr1, uint32_t size1)
{
	char *out = NULL;
	uint32_t count = 0;
	int ret = 0;

	vmon_dbg("length = %d, type %d, format %d, weight %d, height %d, stride %d, code_type %d \n", info->length, info->type, info->format, info->width, info->height, info->stride, info->code_type);

	acquire_mutex();
	ret = send_data(info, sizeof(struct cmd_header));
	if (ret < 0) {
		goto err;
	}

	if ((ptr != NULL) && (size != 0)) {
		ret = send_data(ptr, size);
		if (ret < 0) {
			goto err;
		}
	}

	if ((ptr1 != NULL) && (size1 != 0)) {
		ret = send_data(ptr1, size1);
		if (ret < 0) {
			goto err;
		}
	}

	release_mutex();
	return;
err:
	vmon_dbg("send_data error. \n");
	err_handler(0);
	release_mutex();
}

static void *send_stats_data(void *param)
{
	int ret = 0;
	uint32_t size = 0;

	sleep(0.1);
	memset(&awb_s, 0, sizeof(awb_stats_info));

	while(run_stats) {
		usleep(200*1000);
		// awb
		ret = get_awb_stats_info(0);
		if (ret < 0) {
			usleep(1*1000);
		} else {
			// vmon_info("awb send!\n");
			print_timestamp();//time
			send_statsdata(&awb_s.header_info, &awb_s.stats, awb_s.header_info.length, NULL, 0);
		}

		//ae
		ret = get_ae_stats_info(0);
		if (ret < 0) {
			usleep(1*1000);
		} else {
			// vmon_info("ae full send!\n");
			print_timestamp();//time
			send_statsdata(&ae_s.header_info, &ae_s.stats, ae_s.header_info.length, NULL, 0);
		}

		// af
		ret = get_af_stats_info(0);
		if (ret < 0) {
			usleep(1*1000);
		} else {
			// vmon_info("af send!\n");
			print_timestamp();//time
			send_statsdata(&af_s.header_info, &af_s.stats, af_s.header_info.length, NULL, 0);
		}
		
		// lumvar
		ret = get_lumvar_stats_info(0);
		if (ret < 0) {
			// vmon_info("lumvar faile!\n");
			usleep(1*1000);
		} else {
			// vmon_info("lumvar send!\n");
			print_timestamp();//time
			send_statsdata(&lumvar_s.header_info, &lumvar_s.stats, lumvar_s.header_info.length, NULL, 0);
		}
	}

	pthread_exit(NULL);
}

int start_send_stats_data(void)
{
	run_stats = 1;
	if (pthread_create(&pid_stats, NULL, send_stats_data, NULL) != 0) {
		vmon_dbg("start send data!\n");
		run_stats = 0;
		return -1;
	}

	return 0;
}

void stop_send_stats_data(void)
{
	run_stats = 0;
	if (pid_stats) {
		pthread_join(pid_stats, NULL);
		pid_stats = 0;
	}
	vmon_dbg("stop send data!\n");
}
