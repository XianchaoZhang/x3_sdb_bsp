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

#include "hb_vin_api.h"
#include "hb_vps_api.h"
#include "hb_mipi_api.h"
#include "hb_vio_interface.h"
#include "dual_common.h"
#include "hb_sys.h"
#define BIT2CHN(chns, chn) (chns & (1 << chn))

MIPI_SENSOR_INFO_S SENSOR_4LANE_IMX327_30FPS_12BIT_LINEAR_INFO =
{
	.deseEnable = 0,
	.inputMode = INPUT_MODE_MIPI,
	.sensorInfo = {
		.bus_num = 5,
		.fps = 30,
		.resolution = 1097,
		.sensor_addr = 0x36,
		.entry_index = 1,
		.sensor_mode = 1,
		.reg_width = 16,
		.sensor_name = "imx327"
	}
};

MIPI_ATTR_S MIPI_4LANE_SENSOR_IMX327_30FPS_12BIT_NORMAL_ATTR =
{
	.mipi_host_cfg =
	{
		4,			  /* lane */
		0x2c,		  /* datatype */
		24, 		  /* mclk	 */
		891,		  /* mipiclk */
		30, 		  /* fps */
		1952,		  /* width	*/
		1097,		  /*height */
		2152,		  /* linlength */
		1150,		  /* framelength */
		20			  /* settle */
	},
	.dev_enable = 0  /*  mipi dev enable */
};

MIPI_ATTR_S MIPI_4LANE_SENSOR_IMX327_30FPS_12BIT_NORMAL_SENSOR_CLK_ATTR =
{
	.mipi_host_cfg =
	{
		4,			  /* lane */
		0x2c,		  /* datatype */
		3713,		  /* mclk	 */
		891,		  /* mipiclk */
		30, 		  /* fps */
		1952,		  /* width	*/
		1097,		  /*height */
		2152,		  /* linlength */
		1150,		  /* framelength */
		20			  /* settle */
	},
	.dev_enable = 0  /*  mipi dev enable */
};

VIN_DEV_ATTR_S DEV_ATTR_IMX327_LINEAR_BASE = {
	.stSize = { 0,		/*format*/
				1952,	/*width*/
				1097,	 /*height*/
				2		 /*pix_length*/
	},
	.mipiAttr = {
		.enable = 1,
		.ipi_channels = 1,
		.enable_mux_out = 1,
		.enable_frame_id = 1,
		.enable_bypass = 0,
		.enable_line_shift = 0,
		.enable_id_decoder = 0,
		.set_init_frame_id = 1,
		.set_line_shift_count = 0,
		.set_bypass_channels = 1,
	},
	.DdrIspAttr = {
		.buf_num = 4,
		.raw_feedback_en = 0,
		.data = {
			.format = 0,
			.width = 1952,
			.height = 1097,
			.pix_length = 2,
		}
	},
	.outDdrAttr = {
		.stride = 2928,
		.buffer_num = 8,
	},
	.outIspAttr = {
		.dol_exp_num = 1,
		.enable_dgain = 0,
		.set_dgain_short = 0,
		.set_dgain_medium = 0,
		.set_dgain_long = 0,
		.vc_short_seq = 0,
		.vc_medium_seq = 0,
	    .vc_long_seq = 0,
	}
};

VIN_PIPE_ATTR_S PIPE_ATTR_IMX327_LINEAR_BASE = {
	.ddrOutBufNum = 6,
	.snsMode = SENSOR_NORMAL_MODE,
	.stSize = {
		.format = 0,
		.width = 1920,
		.height = 1080,
	},
	.temperMode = 2,
	.ispBypassEn = 0,
	.ispAlgoState = 1,
	.bitwidth = 12,
	.calib = {
		.mode = 1,
		.lname = "libimx327_linear.so",
	}
};

VIN_DIS_ATTR_S DIS_ATTR_BASE = {
	.picSize = {
		.pic_w = 1919,
		.pic_h = 1079,
	},
	.disPath = {
		.rg_dis_enable = 0,
		.rg_dis_path_sel = 1,
	},
	.disHratio = 65536,
	.disVratio = 65536,
	.xCrop = {
		.rg_dis_start = 0,
		.rg_dis_end = 1919,
	},
	.yCrop = {
		.rg_dis_start = 0,
		.rg_dis_end = 1079,
	},
	.disBufNum = 8,
};

VIN_LDC_ATTR_S LDC_ATTR_BASE = {
  .ldcEnable = 0,
  .ldcPath = {
	 .rg_y_only = 0,
	 .rg_uv_mode = 0,
	 .rg_uv_interpo = 0,
	 .rg_h_blank_cyc = 32,
  },
  .yStartAddr = 524288,
  .cStartAddr = 786432,
  .picSize = {
	 .pic_w = 1919,
	 .pic_h = 1079,
  },
  .lineBuf = 99,
  .xParam = {
	 .rg_algo_param_b = 1,
	 .rg_algo_param_a = 1,
  },
  .yParam = {
	 .rg_algo_param_b = 1,
	 .rg_algo_param_a = 1,
  },
  .offShift = {
	 .rg_center_xoff = 0,
	 .rg_center_yoff = 0,
  },
  .xWoi = {
	 .rg_start = 0,
	 .rg_length = 1919,
  },
  .yWoi = {
	 .rg_start = 0,
	 .rg_length = 1079,
  }
};

int SAMPLE_MIPI_GetSnsAttrBySns(MIPI_SNS_TYPE_E enSnsType, MIPI_SENSOR_INFO_S* pstSnsAttr)
{
	 switch (enSnsType)
	 {
		 case IMX327_30FPS_1952P_RAW12_LINEAR:
			 memcpy(pstSnsAttr, &SENSOR_4LANE_IMX327_30FPS_12BIT_LINEAR_INFO, sizeof(MIPI_SENSOR_INFO_S));
			 break;
		 default:
			 printf("not surpport sensor type enSnsType %d\n", enSnsType);
			 break;
	 }
	 printf("SAMPLE_MIPI_GetSnsAttrBySns success\n");
	 return 0;
}


int SAMPLE_MIPI_GetMipiAttrBySns(MIPI_SNS_TYPE_E enSnsType, MIPI_ATTR_S* pstMipiAttr)
{
	 switch (enSnsType)
	 {
		 case IMX327_30FPS_1952P_RAW12_LINEAR:
			 memcpy(pstMipiAttr, &MIPI_4LANE_SENSOR_IMX327_30FPS_12BIT_NORMAL_ATTR, sizeof(MIPI_ATTR_S));
			 break;
		 default:
			 printf("not surpport sensor type\n");
			 break;
	 }
	 return 0;
}


int SAMPLE_VIN_GetDevAttrBySns(MIPI_SNS_TYPE_E enSnsType, VIN_DEV_ATTR_S *pstDevAttr)
{
    switch (enSnsType)
    {
		 case IMX327_30FPS_1952P_RAW12_LINEAR:
			 memcpy(pstDevAttr, &DEV_ATTR_IMX327_LINEAR_BASE, sizeof(VIN_DEV_ATTR_S));
			 break;
		default:
			printf("not surpport sensor type\n");
			break;
    }
	printf("SAMPLE_VIN_GetDevAttrBySns success\n");
    return 0;
}


int SAMPLE_VIN_GetPipeAttrBySns(MIPI_SNS_TYPE_E enSnsType, VIN_PIPE_ATTR_S *pstPipeAttr)
{
    switch (enSnsType)
    {
		 case IMX327_30FPS_1952P_RAW12_LINEAR:
			 memcpy(pstPipeAttr, &PIPE_ATTR_IMX327_LINEAR_BASE, sizeof(VIN_PIPE_ATTR_S));
			 break;

		default:
			printf("not surpport sensor type\n");
			break;
    }
	printf("SAMPLE_VIN_GetPipeAttrBySns success\n");
    return 0;
}

int SAMPLE_VIN_GetDisAttrBySns(MIPI_SNS_TYPE_E enSnsType, VIN_DIS_ATTR_S *pstDisAttr)
{
    switch (enSnsType)
    {
		 case IMX327_30FPS_1952P_RAW12_LINEAR:
			 memcpy(pstDisAttr, &DIS_ATTR_BASE, sizeof(VIN_DIS_ATTR_S));
			 break;
		default:
			printf("not surpport sensor type\n");
			break;
    }
	printf("SAMPLE_VIN_GetDisAttrBySns success\n");
    return 0;
}

int SAMPLE_VIN_GetLdcAttrBySns(MIPI_SNS_TYPE_E enSnsType, VIN_LDC_ATTR_S *pstLdcAttr)
{
    switch (enSnsType)
    {
		 case IMX327_30FPS_1952P_RAW12_LINEAR:
			 memcpy(pstLdcAttr, &LDC_ATTR_BASE, sizeof(VIN_LDC_ATTR_S));
			 break;
		default:
			printf("not surpport sensor type\n");
			break;
    }
	printf("SAMPLE_VIN_GetLdcAttrBySns success\n");
    return 0;
}
void parse_sensorid_func(char *optarg)
{
	char *p = optarg, *d;
	int i = 0;
	printf("optarg %s\n", optarg);
	while (p && *p && i < MAX_SENSOR_NUM) {
		d = strchr(p, ',');
		if (d)
			*d = '\0';
		sensorId[i] = atoi(p);
		printf("i %d sensorId[i] %d===========\n", i, sensorId[i]);
		i++;
		p = (d) ? (d + 1) : NULL;
	}
	return;
}

void parse_mipiIdx_func(char *optarg)
{
	char *p = optarg, *d;
	int i = 0;
	printf("optarg %s\n", optarg);
	while (p && *p && i < MAX_MIPIID_NUM) {
		d = strchr(p, ',');
		if (d)
			*d = '\0';
		mipiIdx[i] = atoi(p);
		printf("i %d mipiIdx[i] %d===========\n", i, mipiIdx[i]);
		i++;
		p = (d) ? (d + 1) : NULL;
	}
	return;
}

void parse_bus_func(char *optarg)
{
	char *p = optarg, *d;
	int i = 0;
	printf("optarg %s\n", optarg);
	while (p && *p && i < MAX_MIPIID_NUM) {
		d = strchr(p, ',');
		if (d)
			*d = '\0';
		bus[i] = atoi(p);
		printf("i %d bus[i] %d===========\n", i, bus[i]);
		i++;
		p = (d) ? (d + 1) : NULL;
	}
	return;
}

void parse_port_func(char *optarg)
{
	char *p = optarg, *d;
	int i = 0;
	printf("optarg %s\n", optarg);
	while (p && *p && i < MAX_MIPIID_NUM) {
		d = strchr(p, ',');
		if (d)
			*d = '\0';
		port[i] = atoi(p);
		printf("i %d port[i] %d===========\n", i, port[i]);
		i++;
		p = (d) ? (d + 1) : NULL;
	}
	return;
}

void parse_serdes_index_func(char *optarg)
{
	char *p = optarg, *d;
	int i = 0;
	printf("optarg %s\n", optarg);
	while (p && *p && i < MAX_MIPIID_NUM) {
		d = strchr(p, ',');
		if (d)
			*d = '\0';
		serdes_index[i] = atoi(p);
		printf("i %d serdes_index[i] %d===========\n", i,
		       serdes_index[i]);
		i++;
		p = (d) ? (d + 1) : NULL;
	}
	return;
}

void parse_vin_vps_mode_func(char *optarg)
{
	char *p = optarg, *d;
	int i = 0;
	printf("optarg %s\n", optarg);
	while (p && *p && i < MAX_MIPIID_NUM) {
		d = strchr(p, ',');
		if (d)
			*d = '\0';
		vin_vps_mode[i] = atoi(p);
		printf("i %d serdes_index[i] %d===========\n", i,
		       vin_vps_mode[i]);
		i++;
		p = (d) ? (d + 1) : NULL;
	}
	return;
}

void parse_serdes_port_func(char *optarg)
{
	char *p = optarg, *d;
	int i = 0;
	printf("optarg %s\n", optarg);
	while (p && *p && i < MAX_MIPIID_NUM) {
		d = strchr(p, ',');
		if (d)
			*d = '\0';
		serdes_port[i] = atoi(p);
		printf("i %d serdes_port[i] %d===========\n", i,
		       serdes_port[i]);
		i++;
		p = (d) ? (d + 1) : NULL;
	}
	return;
}

int time_cost_ms(struct timeval *start, struct timeval *end)
{
	int time_ms = -1;
	time_ms = ((end->tv_sec * 1000 + end->tv_usec /1000) -
		(start->tv_sec * 1000 + start->tv_usec /1000));
	printf("time cost %d ms \n", time_ms);
	return time_ms;
}

void print_sensor_dev_info(VIN_DEV_ATTR_S *devinfo)
{
	printf("devinfo->stSize.format %d\n", devinfo->stSize.format);
	printf("devinfo->stSize.height %d\n", devinfo->stSize.height);
	printf("devinfo->stSize.width %d\n", devinfo->stSize.width);
	printf("devinfo->stSize.pix_length %d\n", devinfo->stSize.pix_length);
	printf("devinfo->mipiAttr.enable_frame_id %d\n", devinfo->mipiAttr.enable_frame_id);
	printf("devinfo->mipiAttr.enable_mux_out %d\n", devinfo->mipiAttr.enable_mux_out);
	printf("devinfo->mipiAttr.set_init_frame_id %d\n", devinfo->mipiAttr.set_init_frame_id);
	printf("devinfo->mipiAttr.ipi_channels %d\n", devinfo->mipiAttr.ipi_channels);
	printf("devinfo->mipiAttr.enable_line_shift %d\n", devinfo->mipiAttr.enable_line_shift);
	printf("devinfo->mipiAttr.enable_id_decoder %d\n", devinfo->mipiAttr.enable_id_decoder);
	printf("devinfo->mipiAttr.set_bypass_channels %d\n", devinfo->mipiAttr.set_bypass_channels);
	printf("devinfo->mipiAttr.enable_bypass %d\n", devinfo->mipiAttr.enable_bypass);
	printf("devinfo->mipiAttr.set_line_shift_count %d\n", devinfo->mipiAttr.set_line_shift_count);
	printf("devinfo->mipiAttr.enable_pattern %d\n", devinfo->mipiAttr.enable_pattern);

	printf("devinfo->outDdrAttr.stride %d\n", devinfo->outDdrAttr.stride);
	printf("devinfo->outDdrAttr.buffer_num %d\n", devinfo->outDdrAttr.buffer_num);
	return ;
}

void print_sensor_pipe_info(VIN_PIPE_ATTR_S *pipeinfo)
{
	printf("isp_out ddr_out_buf_num %d\n", pipeinfo->ddrOutBufNum);
	printf("isp_out width %d\n", pipeinfo->stSize.width);
	printf("isp_out height %d\n", pipeinfo->stSize.height);
	printf("isp_out sensor_mode %d\n", pipeinfo->snsMode);
	printf("isp_out format %d\n", pipeinfo->stSize.format);
	return ;
}

void print_sensor_info(MIPI_SENSOR_INFO_S *snsinfo)
{
	printf("bus_num %d\n", snsinfo->sensorInfo.bus_num);
	printf("bus_type %d\n", snsinfo->sensorInfo.bus_type);
	printf("sensor_name %s\n", snsinfo->sensorInfo.sensor_name);
	printf("reg_width %d\n", snsinfo->sensorInfo.reg_width);
	printf("sensor_mode %d\n", snsinfo->sensorInfo.sensor_mode);
	printf("sensor_addr 0x%x\n", snsinfo->sensorInfo.sensor_addr);
	printf("serial_addr 0x%x\n", snsinfo->sensorInfo.serial_addr);
	printf("resolution %d\n", snsinfo->sensorInfo.resolution);

	return ;
}

int ion_open(void);
int ion_alloc_phy(int size, int *fd, char **vaddr, uint64_t * paddr);
int dumpToFile2plane(char *filename, char *srcBuf, char *srcBuf1,
		unsigned int size, unsigned int size1);
int dumpToFile(char *filename, char *srcBuf, unsigned int size);

int prepare_user_buf(void *buf, uint32_t size)
{
	int ret;
	hb_vio_buffer_t *buffer = (hb_vio_buffer_t*)buf;

	if (buffer == NULL)
		return -1;

	buffer->img_info.fd[0] = ion_open();

	ret  = ion_alloc_phy(size, &buffer->img_info.fd[0],
							&buffer->img_addr.addr[0], &buffer->img_addr.paddr[0]);
	if (ret) {
		printf("prepare user buf error\n");
		return ret;
	}
	printf("prepare user buf  vaddr = 0x%x paddr = 0x%x \n",
				buffer->img_addr.addr[0], buffer->img_addr.paddr[0]);
	return 0;
}

int prepare_user_buf_2lane(void *buf, uint32_t size_y, uint32_t size_uv)
{
	int ret;
	hb_vio_buffer_t *buffer = (hb_vio_buffer_t *)buf;

	if (buffer == NULL)
		return -1;

	buffer->img_info.fd[0] = ion_open();
	buffer->img_info.fd[1] = ion_open();

	ret  = ion_alloc_phy(size_y, &buffer->img_info.fd[0],
						&buffer->img_addr.addr[0], &buffer->img_addr.paddr[0]);
	if (ret) {
		printf("prepare user buf error\n");
		return ret;
	}
	ret = ion_alloc_phy(size_uv, &buffer->img_info.fd[1],
						&buffer->img_addr.addr[1], &buffer->img_addr.paddr[1]);
	if (ret) {
		printf("prepare user buf error\n");
		return ret;
	}

	printf("buf:y: vaddr = 0x%x paddr = 0x%x; uv: vaddr = 0x%x, paddr = 0x%x\n",
				buffer->img_addr.addr[0], buffer->img_addr.paddr[0],
				buffer->img_addr.addr[1], buffer->img_addr.paddr[1]);
	return 0;
}


static void normal_buf_info_print(hb_vio_buffer_t * buf)
{
	printf("normal pipe_id (%d)type(%d)frame_id(%d)buf_index(%d)w x h(%dx%d) data_type %d img_format %d\n",
		buf->img_info.pipeline_id,
		buf->img_info.data_type,
		buf->img_info.frame_id,
		buf->img_info.buf_index,
		buf->img_addr.width,
		buf->img_addr.height,
		buf->img_info.data_type,
		buf->img_info.img_format);
}
int hb_vin_vps_start(int pipeId)
{
	int ret = 0;

	ret = HB_VIN_EnableChn(pipeId,  0);  // dwe start
	if(ret < 0) {
		printf("HB_VIN_EnableChn error!\n");
		return ret;
	}

	ret = HB_VPS_StartGrp(pipeId);
	if (ret) {
		printf("HB_VPS_StartGrp error!!!\n");
	} else {
		printf("start grp ok: grp_id = %d\n", pipeId);
	}

	ret = HB_VIN_StartPipe(pipeId);  // isp start
	if(ret < 0) {
		printf("HB_VIN_StartPipe error!\n");
		return ret;
	}
	ret = HB_VIN_EnableDev(pipeId);  // sif start && start thread
	if(ret < 0) {
		printf("HB_VIN_EnableDev error!\n");
		return ret;
	}

	return ret;
}

void hb_vin_vps_stop(int pipeId)
{
	HB_VIN_DisableDev(pipeId);    // thread stop && sif stop
	HB_VIN_StopPipe(pipeId);    // isp stop
	HB_VIN_DisableChn(pipeId, 1);   // dwe stop
	HB_VPS_StopGrp(pipeId);
}

int chns2chn(int chns)
{
	int chn = 0;
	if (chns == 0)
		return 0xffffffff;
	while((chns & 1) != 1) {
		chns >>= 1;
		chn ++;
	}
	return chn;
}
void dis_crop_set(uint32_t pipe_id, uint32_t event, VIN_DIS_MV_INFO_S *data,
			 void *userdata)
{
		printf("dis_crop_set callback come in\n");
		printf("data gmvX %d\n", data->gmvX);
		printf("data gmvY %d\n", data->gmvY);
		printf("data xUpdate %d\n", data->xUpdate);
		printf("data yUpdate %d\n", data->yUpdate);
		return;
}
int hb_vin_vps_init(int pipeId, uint32_t sensorId, uint32_t mipiIdx, uint32_t deseri_port, uint32_t vin_vps_mode)
{
	int ret = 0;
	VIN_DEV_ATTR_S *devinfo = NULL;
	VIN_PIPE_ATTR_S *pipeinfo = NULL;
	VIN_DIS_ATTR_S *disinfo = NULL;
	VIN_LDC_ATTR_S * ldcinfo = NULL;
	VIN_DEV_ATTR_EX_S *devexinfo = NULL;
	VIN_DIS_CALLBACK_S pstDISCallback;
	pstDISCallback.VIN_DIS_DATA_CB = dis_crop_set;

	VPS_GRP_ATTR_S grp_attr;
	VPS_CHN_ATTR_S chn_attr;
	VPS_PYM_CHN_ATTR_S pym_chn_attr;

	devinfo = malloc(sizeof(VIN_DEV_ATTR_S));
	if(devinfo == NULL) {
		printf("malloc error\n");
		return -1;
	}
	devexinfo = malloc(sizeof(VIN_DEV_ATTR_EX_S));
	if(devexinfo == NULL) {
		printf("malloc error\n");
		ret = -1;
		goto malloc_err0;
	}
	pipeinfo = malloc(sizeof(VIN_PIPE_ATTR_S));
	if(pipeinfo == NULL) {
		printf("malloc error\n");
		ret = -1;
		goto malloc_err1;
	}
	disinfo = malloc(sizeof(VIN_DIS_ATTR_S));
	if(disinfo == NULL) {
		printf("malloc error\n");
		ret = -1;
		goto malloc_err2;
	}
	ldcinfo = malloc(sizeof(VIN_LDC_ATTR_S));
	if(ldcinfo == NULL) {
		printf("malloc error\n");
		ret = -1;
		goto malloc_err3;
	}
	memset(devinfo, 0, sizeof(VIN_DEV_ATTR_S));
	memset(pipeinfo, 0, sizeof(VIN_PIPE_ATTR_S));
	memset(disinfo, 0, sizeof(VIN_DIS_ATTR_S));
	memset(ldcinfo, 0, sizeof(VIN_LDC_ATTR_S));
	SAMPLE_VIN_GetDevAttrBySns(sensorId, devinfo);
	SAMPLE_VIN_GetPipeAttrBySns(sensorId, pipeinfo);
	SAMPLE_VIN_GetDisAttrBySns(sensorId, disinfo);
	SAMPLE_VIN_GetLdcAttrBySns(sensorId, ldcinfo);
	print_sensor_dev_info(devinfo);
	print_sensor_pipe_info(pipeinfo);

	ret = HB_SYS_SetVINVPSMode(pipeId, vin_vps_mode);
	if(ret < 0) {
		printf("HB_SYS_SetVINVPSMode%d error!\n", vin_vps_mode);
		goto malloc_err4;
	}
	ret = HB_VIN_CreatePipe(pipeId, pipeinfo);   // isp init
	if(ret < 0) {
		printf("HB_MIPI_InitSensor error!\n");
		goto malloc_err4;
	}
	ret = HB_VIN_SetMipiBindDev(pipeId, mipiIdx);
	if(ret < 0) {
		printf("HB_VIN_SetMipiBindDev error!\n");
		goto malloc_err4;
	}
	ret = HB_VIN_SetDevVCNumber(pipeId, deseri_port);
	if(ret < 0) {
		printf("HB_VIN_SetDevVCNumber error!\n");
		goto malloc_err4;
	}
	if(sensorId == IMX327_30FPS_2228P_RAW12_DOL2 ||
		sensorId == OS8A10_30FPS_3840P_RAW10_DOL2 ||
		sensorId == IMX327_15FPS_3609P_RAW12_DOL3 ||
		sensorId == IMX327_15FPS_3609P_DOL3_LEF_SEF2) {

		printf("HB_VIN_AddDevVCNumber error!\n");
		ret = HB_VIN_AddDevVCNumber(pipeId, vc_num);
		if(ret < 0) {
			printf("HB_VIN_AddDevVCNumber error!\n");
			goto malloc_err4;
		}
	}
	if(sensorId == IMX327_15FPS_3609P_RAW12_DOL3) {
		ret = HB_VIN_AddDevVCNumber(pipeId, 2);
		if(ret < 0) {
			printf("HB_VIN_AddDevVCNumber error!\n");
			goto malloc_err4;
		}
	}
	ret = HB_VIN_SetDevAttr(pipeId, devinfo);     // sif init
	if(ret < 0) {
		printf("HB_MIPI_InitSensor error!\n");
		goto malloc_err4;
	}

	ret = HB_VIN_SetPipeAttr(pipeId, pipeinfo);     // isp init
	if(ret < 0) {
		printf("HB_VIN_SetPipeAttr error!\n");
		goto malloc_err4;
	}
	ret = HB_VIN_SetChnDISAttr(pipeId, 1, disinfo);  //  dis init
	if(ret < 0) {
		printf("HB_VIN_SetChnDISAttr error!\n");
		goto malloc_err4;
	}

	ret = HB_VIN_SetChnLDCAttr(pipeId, 1, ldcinfo);   //  ldc init
	if(ret < 0) {
		printf("HB_VIN_SetChnLDCAttr error!\n");
		goto malloc_err4;
	}
	ret = HB_VIN_SetChnAttr(pipeId, 1);               //  dwe init
	if(ret < 0) {
		printf("HB_VIN_SetChnAttr error!\n");
		goto malloc_err4;
	}
	HB_VIN_SetDevBindPipe(pipeId, pipeId);    //  bind init
	memset(&grp_attr, 0, sizeof(VPS_GRP_ATTR_S));
	grp_attr.maxW = pipeinfo->stSize.width;
	grp_attr.maxH = pipeinfo->stSize.height;
	ret = HB_VPS_CreateGrp(pipeId, &grp_attr);
	if (ret) {
		printf("HB_VPS_CreateGrp error!!!\n");
	} else {
		printf("created a group ok:GrpId = %d\n", pipeId);
	}

	if (need_grp_rotate) {
		ret = HB_VPS_SetGrpRotate(pipeId, ROTATION_90);
		if (ret) {
			printf("HB_VPS_SetGrpRotate error!!!\n");
		} else {
			printf("HB_VPS_SetGrpRotate ok:GrpId = %d\n", pipeId);
		}
	}

	memset(&chn_attr, 0, sizeof(VPS_CHN_ATTR_S));
	chn_attr.enScale = 1;
	chn_attr.width = 1920;
	chn_attr.height = 1080;
	chn_attr.frameDepth = 6;

	if (BIT2CHN(need_ipu, 0)) {
		ret = HB_VPS_SetChnAttr(pipeId, 0, &chn_attr);
		if (ret) {
			printf("HB_VPS_SetChnAttr error!!!\n");
		} else {
			printf("set chn Attr ok: GrpId = %d, chn_id = %d\n",
									pipeId, 0);
		}
		HB_VPS_EnableChn(pipeId, 0);
	}

	chn_attr.width = 2688;//1920;//
	chn_attr.height = 1520;//1080;//

	if (BIT2CHN(need_ipu, 5)) {
		ret = HB_VPS_SetChnAttr(pipeId, 5, &chn_attr);
		if (ret) {
			printf("HB_VPS_SetChnAttr error!!!\n");
		} else {
			printf("set chn Attr ok: GrpId = %d, chn_id = %d\n",
									pipeId, 5);
		}
		HB_VPS_EnableChn(pipeId, 5);
	}

	chn_attr.width = 1920;
	chn_attr.height = 1080;
	if (BIT2CHN(need_ipu, 1)) {
		ret = HB_VPS_SetChnAttr(pipeId, 1, &chn_attr);
		if (ret) {
			printf("HB_VPS_SetChnAttr error!!!\n");
		} else {
			printf("set chn Attr ok: GrpId = %d, chn_id = %d\n",
									pipeId, 1);
		}
		HB_VPS_EnableChn(pipeId, 1);
	}

	if (BIT2CHN(need_ipu, 2)) {
		ret = HB_VPS_SetChnAttr(pipeId, 2, &chn_attr);
		if (ret) {
			printf("HB_VPS_SetChnAttr error!!!\n");
		} else {
			printf("set chn Attr ok: pipeId = %d, chn_id = %d\n",
									pipeId, 2);
		}
		HB_VPS_EnableChn(pipeId, 2);
	}
	chn_attr.width = 1920;
	chn_attr.height = 1080;
	if (BIT2CHN(need_ipu, 6)) {
		ret = HB_VPS_SetChnAttr(pipeId, 6, &chn_attr);
		if (ret) {
			printf("HB_VPS_SetChnAttr error!!!\n");
		} else {
			printf("set chn Attr ok: GrpId = %d, chn_id = %d\n",
									pipeId, 6);
		}
		HB_VPS_EnableChn(pipeId, 6);
	}

	chn_attr.width = 704;
	chn_attr.height = 576;

	if (BIT2CHN(need_ipu, 3)) {
		ret = HB_VPS_SetChnAttr(pipeId, 3, &chn_attr);
		if (ret) {
			printf("HB_VPS_SetChnAttr error!!!\n");
		} else {
			printf("set chn Attr ok: GrpId = %d, chn_id = %d\n",
									pipeId, 3);
		}
		HB_VPS_EnableChn(pipeId, 3);
	}

	if (BIT2CHN(need_ipu, 4)) {
		ret = HB_VPS_SetChnAttr(pipeId, 4, &chn_attr);
		if (ret) {
			printf("HB_VPS_SetChnAttr error!!!\n");
		} else {
			printf("set chn Attr ok: GrpId = %d, chn_id = %d\n",
									pipeId, 4);
		}
		HB_VPS_EnableChn(pipeId, 4);
	}

	if (need_pym) {
		memset(&pym_chn_attr, 0, sizeof(VPS_PYM_CHN_ATTR_S));
		pym_chn_attr.timeout = 2000;
		pym_chn_attr.ds_layer_en = 23;
		pym_chn_attr.us_layer_en = 0;
		pym_chn_attr.frame_id = 0;
		pym_chn_attr.frameDepth = 6;
		ret = HB_VPS_SetPymChnAttr(pipeId, chns2chn(need_pym), &pym_chn_attr);
		if (ret) {
			printf("HB_VPS_SetPymChnAttr error!!!\n");
		} else {
			printf("HB_VPS_SetPymChnAttr ok: grp_id = %d g_pym_chn = %d\n",
									pipeId, chns2chn(need_pym));
		}
		HB_VPS_EnableChn(pipeId, chns2chn(need_pym));
	}

	if (need_chn_rotate) {
		ret = HB_VPS_SetChnRotate(pipeId, chns2chn(need_chn_rotate), ROTATION_90);
		if (ret) {
			printf("HB_VPS_SetChnRotate error!!!\n");
		} else {
			printf("Set Chn Rotate ok: grp_id = %d, chn_id = %d\n",
								pipeId, chns2chn(need_chn_rotate));
		}
		HB_VPS_EnableChn(pipeId, chns2chn(need_chn_rotate));
	}

	struct HB_SYS_MOD_S src_mod, dst_mod;
	src_mod.enModId = HB_ID_VIN;
	src_mod.s32DevId = pipeId;
	if (vin_vps_mode == VIN_ONLINE_VPS_ONLINE ||
		vin_vps_mode == VIN_OFFLINE_VPS_ONLINE||
		vin_vps_mode == VIN_SIF_ONLINE_DDR_ISP_ONLINE_VPS_ONLINE||
		vin_vps_mode == VIN_SIF_OFFLINE_ISP_OFFLINE_VPS_ONLINE ||
		vin_vps_mode == VIN_FEEDBACK_ISP_ONLINE_VPS_ONLINE ||
		vin_vps_mode == VIN_SIF_VPS_ONLINE)
		src_mod.s32ChnId = 1;
	else
		src_mod.s32ChnId = 0;
	dst_mod.enModId = HB_ID_VPS;
	dst_mod.s32DevId = pipeId;
	dst_mod.s32ChnId = 0;
	ret = HB_SYS_Bind(&src_mod, &dst_mod);
	if (ret != 0)
		printf("HB_SYS_Bind failed\n");

	goto malloc_err4;
pipe_err:
	HB_VIN_DestroyDev(pipeId);  // sif deinit
chn_err:
	HB_VIN_DestroyPipe(pipeId);  // isp && dwe deinit

malloc_err4:
	free(ldcinfo);
malloc_err3:
	free(disinfo);
malloc_err2:
	free(pipeinfo);
malloc_err1:
	free(devexinfo);
malloc_err0:
	free(devinfo);
	return ret;
}

int hb_pym_feedback_init_start(int pipeId)
{
	int ret;
	VPS_GRP_ATTR_S grp_attr;
	VPS_CHN_ATTR_S chn_attr;
	VPS_PYM_CHN_ATTR_S pym_chn_attr;

	memset(&grp_attr, 0, sizeof(VPS_GRP_ATTR_S));
	grp_attr.maxW = 1280;
	grp_attr.maxH = 720;
	ret = HB_VPS_CreateGrp(pipeId, &grp_attr);
	if (ret) {
		printf("HB_VPS_CreateGrp error!!!\n");
	} else {
		printf("created a group ok:GrpId = %d\n", pipeId);
	}
	memset(&pym_chn_attr, 0, sizeof(VPS_PYM_CHN_ATTR_S));
	pym_chn_attr.timeout = 2000;
	pym_chn_attr.ds_layer_en = 23;
	pym_chn_attr.us_layer_en = 0;
	pym_chn_attr.frame_id = 0;
	pym_chn_attr.frameDepth = 6;
	ret = HB_VPS_SetPymChnAttr(pipeId, 0, &pym_chn_attr);
	if (ret) {
		printf("HB_VPS_SetPymChnAttr error!!!\n");
	} else {
		printf("HB_VPS_SetPymChnAttr ok: grp_id = %d g_pym_chn = %d\n",
								pipeId, 0);
	}
	HB_VPS_EnableChn(pipeId, 0);

	ret = HB_VPS_StartGrp(pipeId);
	if (ret) {
		printf("HB_VPS_StartGrp error!!!\n");
	} else {
		printf("start grp ok: grp_id = %d\n", pipeId);
	}
	return 0;
}

void hb_vin_vps_deinit(int pipeId, int sensorId)
{
	HB_VIN_DestroyDev(pipeId);  // sif deinit && destroy
	HB_VIN_DestroyChn(pipeId, 1);  // dwe deinit
	HB_VIN_DestroyPipe(pipeId);  // isp deinit && destroy
	HB_VPS_DestroyGrp(pipeId);
}
int hb_sensor_init(int devId, int sensorId, int bus, int port, int mipiIdx, int sedres_index, int sedres_port)
{
	int ret = 0;
	MIPI_SENSOR_INFO_S *snsinfo = NULL;

	snsinfo = malloc(sizeof(MIPI_SENSOR_INFO_S));
	if(snsinfo == NULL) {
		printf("malloc error\n");
		return -1;
	}
	memset(snsinfo, 0, sizeof(MIPI_SENSOR_INFO_S));
	SAMPLE_MIPI_GetSnsAttrBySns(sensorId, snsinfo);
	if(sensorId == IMX327_30FPS_2228P_RAW12_DOL2 ||
		sensorId == IMX327_30FPS_1952P_RAW12_LINEAR ||
		sensorId == IMX327_30FPS_2228P_DOL2_TWO_LINEAR) {
		if (bus == 0) {
		  system("echo 1 >/sys/class/vps/mipi_host0/param/stop_check_instart");
		} else if (bus == 5) {
		  system("echo 1 >/sys/class/vps/mipi_host1/param/stop_check_instart");
		}
	}
    HB_MIPI_SetBus(snsinfo, bus);
    HB_MIPI_SetPort(snsinfo, port);
    HB_MIPI_SensorBindSerdes(snsinfo, sedres_index, sedres_port);
    HB_MIPI_SensorBindMipi(snsinfo,  mipiIdx);

	ret = HB_MIPI_InitSensor(devId, snsinfo);
	if(ret < 0) {
		printf("HB_MIPI_InitSensor error!\n");
	}

	free(snsinfo);
	return ret;
}

int hb_mipi_init(int sensorId, int mipiIdx)
{
	int ret = 0;
	MIPI_ATTR_S *mipi_attr = NULL;

	mipi_attr = malloc(sizeof(MIPI_ATTR_S));
	if(mipi_attr == NULL) {
		printf("malloc error\n");
		return -1;
	}
	memset(mipi_attr, 0, sizeof(MIPI_ATTR_S));
	SAMPLE_MIPI_GetMipiAttrBySns(sensorId, mipi_attr);

	ret = HB_MIPI_SetMipiAttr(mipiIdx, mipi_attr);
	if(ret < 0) {
		printf("HB_MIPI_SetDevAttr error!\n");
	}
	free(mipi_attr);
	printf("HB_MIPI_SetDevAttr end\n");
	return ret;
}

int hb_sensor_deinit(int devId)
{
	int ret = 0;

	ret = HB_MIPI_DeinitSensor(devId);
	if(ret < 0) {
		printf("HB_MIPI_DeinitSensor error!\n");
		return ret;
	}
	return ret;
}

int hb_mipi_deinit(int mipiIdx)
{
	int ret = 0;

	printf("hb_sensor_deinit end==mipiIdx %d====\n", mipiIdx);
	ret = HB_MIPI_Clear(mipiIdx);
	if(ret < 0) {
		printf("HB_MIPI_Clear error!\n");
		return ret;
	}
	printf("hb_mipi_deinit end======\n");
	return ret;
}

int hb_sensor_start(int devId)
{
	int ret = 0;

	ret = HB_MIPI_ResetSensor(devId);
	if(ret < 0) {
		printf("HB_MIPI_ResetSensor error!\n");
		return ret;
	}
	return ret;
}


int hb_mipi_start(int mipiIdx)
{
	int ret = 0;

	ret = HB_MIPI_ResetMipi(mipiIdx);
	if(ret < 0) {
		printf("HB_MIPI_ResetMipi error!\n");
		return ret;
	}

	return ret;
}

int hb_sensor_stop(int devId)
{
	int ret = 0;

	ret = HB_MIPI_UnresetSensor(devId);
	if(ret < 0) {
		printf("HB_MIPI_UnresetSensor error!\n");
		return ret;
	}
	return ret;
}

int hb_mipi_stop(int mipiIdx)
{
	int ret = 0;
	printf("hb_mipi_stop======\n");
	ret = HB_MIPI_UnresetMipi(mipiIdx);
	if(ret < 0) {
		printf("HB_MIPI_UnresetMipi error!\n");
		return ret;
	}
	return ret;
}

int dual_vin_vps_init()
{
	int ret, i;

	if (g_venc_flag > 0) {
		ret = dual_singlepipe_venc_init();
		if (ret < 0) {
			printf("dual_singlepipe_venc_init error! do venc_deinit && vio_deinit\n");
			dual_singlepipe_venc_deinit();
			return ret;
		}
	}

	for(i = 0; i < MAX_ID_NUM; i++) {
		if (BIT(i) & groupMask) {
			printf("%d===groupMask %d ====sensorId[i] %d ===mipiIdx[i] %d=\n", i, groupMask, sensorId[i], mipiIdx[i]);
			ret = hb_vin_vps_init(i, sensorId[i], mipiIdx[i], serdes_port[i], vin_vps_mode[i]);
			if(ret < 0) {
				printf("hb_vin_init error!\n");
				return ret;
			}
		}
	}
		for(i = 0; i < MAX_ID_NUM; i++) {
			if (BIT(i) & groupMask) {
			    printf("%d===groupMask %d ====sensorId[i] %d ==mipiIdx[i] %d===\n", (BIT(i) & groupMask), groupMask, sensorId[i], mipiIdx[i]);
				if(g_use_x3clock == 1) {
			  		HB_MIPI_EnableSensorClock(mipiIdx[i]);
				}
	 			ret = hb_sensor_init(i, sensorId[i], bus[i], port[i], mipiIdx[i], serdes_index[i], serdes_port[i]);
				if(ret < 0) {
					printf("hb_sensor_init error! do vio deinit\n");
					return ret;
				}
				ret = hb_mipi_init(sensorId[i], mipiIdx[i]);
				if(ret < 0) {
					printf("hb_mipi_init error! do vio deinit\n");
					hb_vin_vps_deinit(i, mipiIdx[i]);
					return ret;
				}
			}
		}

	for(i = 0; i < MAX_ID_NUM; i++) {
		if (BIT(i) & groupMask) {
			ret = hb_vin_vps_start(i);
			printf("%d===groupMask %d ====sensorId[i] %d ====\n", (BIT(i) & groupMask), groupMask, sensorId[i]);
			if(ret < 0) {
				printf("hb_vin_sif_isp_start error! do cam && vio deinit\n");
				hb_sensor_deinit(i);
				hb_mipi_deinit(mipiIdx[i]);
				hb_vin_vps_stop(i);
				hb_vin_vps_deinit(i, sensorId[i]);
				return ret;
			}
		}
	}
	if (g_venc_flag > 0) {
		 dual_venc_pthread_start();
	}
	for(i = 0; i < MAX_ID_NUM; i++) {
		if (BIT(i) & groupMask) {
			ret = hb_sensor_start(i);
			if(ret < 0) {
				printf("hb_mipi_start error! do cam && vio deinit\n");
				hb_sensor_stop(i);
				hb_mipi_stop(mipiIdx[i]);
				hb_sensor_deinit(i);
				hb_mipi_deinit(mipiIdx[i]);
				hb_vin_vps_stop(i);
				hb_vin_vps_deinit(i, sensorId[i]);
				return ret;
			}

			ret = hb_mipi_start(mipiIdx[i]);
			if(ret < 0) {
				printf("hb_mipi_start error! do cam && vio deinit\n");
				hb_sensor_stop(i);
				hb_mipi_stop(mipiIdx[i]);
				hb_sensor_deinit(i);
				hb_mipi_deinit(mipiIdx[i]);
				hb_vin_vps_stop(i);
				hb_vin_vps_deinit(i, sensorId[i]);
				return ret;
			}
		  }
	}
	if (g_iar_enable == 1) {
		hb_vot_init();
	}

	printf("-dual_vin_vps_init success----------------\n");
	return 0;
}
int dual_vin_vps_deinit()
{
	int i;

	if (g_iar_enable == 1) {
		hb_vot_deinit();
	}

    for (i = 0; i < MAX_ID_NUM; i++) {
  		if (BIT(i) & groupMask) {
  			hb_sensor_stop(i);
  			hb_mipi_stop(mipiIdx[i]);
  	   }
    }
	if (g_venc_flag > 0) {
		dual_singlepipe_venc_deinit();
	}
	for (i = 0; i < MAX_ID_NUM; i++) {
		if (BIT(i) & groupMask) {
			hb_vin_vps_stop(i);
		}
	}
  for (i = 0; i < MAX_ID_NUM; i++) {
	if (BIT(i) & groupMask) {
		hb_mipi_deinit(mipiIdx[i]);
		hb_sensor_deinit(i);
		if(g_use_x3clock == 1) {
			HB_MIPI_DisableSensorClock(mipiIdx[i]);
		}
	 }
   }
	for (i = 0; i < MAX_ID_NUM; i++) {
		if (BIT(i) & groupMask) {
			hb_vin_vps_deinit(i, sensorId[i]);
		}
	}
	return 0;
}

