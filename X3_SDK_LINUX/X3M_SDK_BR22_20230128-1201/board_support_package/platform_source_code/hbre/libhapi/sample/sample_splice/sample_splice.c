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
#include "hb_sys.h"

#define MAX_SENSOR_NUM  8
#define MAX_MIPIID_NUM  8
#define MAX_ID_NUM 32
#define MAX_PLANE 4

#define BIT(n)  (1UL << (n))

#define YUV_ENABLE BIT(HB_VIO_ISP_YUV_DATA)   // for yuv dump
#define RAW_ENABLE BIT(HB_VIO_SIF_RAW_DATA)  //  for raw dump/fb

#define VPS_CHN0_ENABLE BIT(HB_VIO_IPU_DS0_DATA)
#define VPS_CHN1_ENABLE BIT(HB_VIO_IPU_DS1_DATA)
#define VPS_CHN2_ENABLE BIT(HB_VIO_IPU_DS2_DATA)
#define VPS_CHN3_ENABLE BIT(HB_VIO_IPU_DS3_DATA)
#define VPS_CHN4_ENABLE BIT(HB_VIO_IPU_DS4_DATA)
#define VPS_CHN5_ENABLE BIT(HB_VIO_IPU_US_DATA)
#define VPS_CHN6_ENABLE BIT(HB_VIO_PYM_FEEDBACK_SRC_DATA)

#define BIT2CHN(chns, chn) (chns & (1 << chn))

time_t start_time, now;
time_t run_time = 0;
time_t end_time = 0;
int need_clk;
int need_m_thread;
int data_type;
int need_chn_rotate;
int need_pym;
int vps_dump;
int need_ipu = 66;
int raw_dump;
int groupMask;
int extra_mode;
int g_exit = 0;
int sensorId;
int yuv_sensorId = 1, raw_sensorId = 2;

extern struct hb_vin_group_s *g_vin[MAX_SENSOR_NUM];

typedef enum group_id {
	GROUP_0 = 0,
	GROUP_1,
	GROUP_2,
	GROUP_3,
	GROUP_4,
	GROUP_5,
	GROUP_6,
	GROUP_7,
	GROUP_MAX
} group_id_e;

typedef struct work_info_s {
	uint32_t group_id;
	VIO_DATA_TYPE_E data_type;
	pthread_t thid;
	int running;
} work_info_t;
work_info_t work_info[GROUP_MAX][HB_VIO_DATA_TYPE_MAX];

struct feedback_arg_s {
	int grp_id;
	pthread_t thid;
	hb_vio_buffer_t *feedback_buf;
};

typedef struct {
	uint32_t frame_id;
	uint32_t plane_count;
	uint32_t xres[MAX_PLANE];
	uint32_t yres[MAX_PLANE];
	char *addr[MAX_PLANE];
	uint32_t size[MAX_PLANE];
} raw_t;

typedef struct {
	uint8_t ctx_id;
    raw_t raw;
} dump_info_t;

typedef enum HB_MIPI_SNS_TYPE_E
{
	SENSOR_ID_INVALID,
	OV10635_30FPS_720p_960_YUV_LINE_CONCATENATED,  // 1
	AR0233_30FPS_1080P_RAW12_954_PWL,	 // 2
	SAMPLE_SENOSR_ID_MAX,
} MIPI_SNS_TYPE_E;


MIPI_SENSOR_INFO_S SENSOR_2LANE_OV10635_30FPS_YUV_720P_960_INFO =
{
	.deseEnable = 1,
	.inputMode = INPUT_MODE_MIPI,
	.deserialInfo = {
		.bus_type = 0,
		.bus_num = 1,
		.deserial_addr = 0x30,
		.deserial_name = "s960"
	},
	.sensorInfo = {
		.port = 0,
		.dev_port = 0,
		.bus_type = 0,
		.bus_num = 1,
		.fps = 30,
		.resolution = 720,
		.sensor_addr = 0x40,
		.serial_addr = 0x1c,
		.entry_index = 0,
		.reg_width = 16,
		.sensor_name = "ov10635",
		.deserial_index = 0,
		.deserial_port = 0
	}
};

MIPI_SENSOR_INFO_S SENSOR_4LANE_AR0233_30FPS_12BIT_1080P_954_PWL_INFO =
{
	.deseEnable = 1,
	.inputMode = INPUT_MODE_MIPI,
	.deserialInfo = {
		.bus_type = 0,
		.bus_num = 4,
		.deserial_addr = 0x3d,
		.deserial_name = "s954",
	},
	.sensorInfo = {
		.port = 0,
		.dev_port = 0,
		.bus_type = 0,
		.bus_num = 4,
		.fps = 30,
		.resolution = 1080,
		.sensor_addr = 0x10,
		.serial_addr = 0x18,
		.entry_index = 1,
		.sensor_mode = PWL_M,
		.reg_width = 16,
		.sensor_name = "ar0233",
		.deserial_index = 0,
		.deserial_port = 0
	}
};

MIPI_ATTR_S MIPI_4LANE_SENSOR_AR0233_30FPS_12BIT_1080P_954_ATTR =
{
	.mipi_host_cfg =
	{
		4,			  /* lane */
		0x2c,		  /* datatype */
		24, 		  /* mclk	 */
		1224,		   /* mipiclk */
		30, 		  /* fps */
		1920,		  /* width	*/
		1080,		  /*height */
		2000,		  /* linlength */
		1700,		  /* framelength */
		30, 		  /* settle */
		 4,
		{0,1,2,3}
	},
	.dev_enable = 0  /*  mipi dev enable */
};

MIPI_ATTR_S MIPI_2LANE_OV10635_30FPS_YUV_720P_960_ATTR =
{
	 .mipi_host_cfg =
	 {
		 2, 		   /* lane */
		 0x1e,		   /* datatype */
		 24,		   /* mclk	  */
		 3200,			/* mipiclk */
		 30,		   /* fps */
		 1280,		   /* width  */
		 720,		   /*height */
		 3207,		   /* linlength */
		 748,		   /* framelength */
		 30,		   /* settle */
		  2,
		 {0, 1}
	 },
	 .dev_enable = 0  /*  mipi dev enable */
};

VIN_DEV_ATTR_S DEV_ATTR_AR0233_1080P_BASE = {
  .stSize = { 0,	  /*format*/
			  1920,   /*width*/
			  1080,    /*height*/
			  2 	   /*pix_length*/
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
		  .width = 1920,
		  .height = 1080,
		  .pix_length = 2,
	  }
  },
  .outDdrAttr = {
	  .stride = 2880,
	  .buffer_num = 8,
  },
  .outIspAttr = {
	  .dol_exp_num = 1,
	  .enable_dgain = 0,
	  .set_dgain_short = 0,
	  .set_dgain_medium = 0,
	  .set_dgain_long = 0,
  }
};

VIN_DEV_ATTR_S DEV_ATTR_OV10635_YUV_LINE_CONCATE_BASE = {
	.stSize = { 8,		/*format*/
				2560,	/*width*/
				720,	 /*height*/
				0		 /*pix_length*/
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
			.format = 8,
			.width = 2560,
			.height = 720,
			.pix_length = 0,
		}
	},
	.outDdrAttr = {
		.stride = 2560,
		.buffer_num = 8,
	},
	.outIspAttr = {
		.dol_exp_num = 1,
		.enable_dgain = 0,
		.set_dgain_short = 0,
		.set_dgain_medium = 0,
		.set_dgain_long = 0,
	}
};

VIN_PIPE_ATTR_S PIPE_ATTR_AR0233_1080P_PWL_BASE = {
	.ddrOutBufNum = 6,
	.snsMode = PWL_M,
	.stSize = {
		.format = 0,
		.width = 1920,
		.height = 1080,
	},
	.cfaPattern = 1,
	.temperMode = 2,
	.ispBypassEn = 0,
	.ispAlgoState = 1,
	.bitwidth = 12,
	.calib = {
		.mode = 1,
		.lname = "lib_ar0233_pwl.so",
	}
};

VIN_PIPE_ATTR_S PIPE_ATTR_OV10635_YUV_BASE = {
	.ddrOutBufNum = 6,
	.snsMode = SENSOR_NORMAL_MODE,
	.stSize = {
		.format = 0,
		.width = 2560,
		.height = 720,
	},
	.ispBypassEn = 1,
	.ispAlgoState = 0,
	.bitwidth = 12,
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

VIN_DIS_ATTR_S DIS_ATTR_OV10635_BASE = {
	.picSize = {
		.pic_w = 1279,
		.pic_h = 719,
	},
	.disPath = {
		.rg_dis_enable = 0,
		.rg_dis_path_sel = 1,
	},
	.disHratio = 65536,
	.disVratio = 65536,
	.xCrop = {
		.rg_dis_start = 0,
		.rg_dis_end = 1279,
	},
	.yCrop = {
		.rg_dis_start = 0,
		.rg_dis_end = 719,
	}
};

VIN_LDC_ATTR_S LDC_ATTR_OV10635_BASE = {
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
	  .pic_w = 1279,
	  .pic_h = 719,
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
	  .rg_length = 1279,
   },
   .yWoi = {
	  .rg_start = 0,
	  .rg_length = 719,
   }
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
		case OV10635_30FPS_720p_960_YUV_LINE_CONCATENATED:
		 	 memcpy(pstSnsAttr, &SENSOR_2LANE_OV10635_30FPS_YUV_720P_960_INFO, sizeof(MIPI_SENSOR_INFO_S));
			 break;
		 case AR0233_30FPS_1080P_RAW12_954_PWL:
			 memcpy(pstSnsAttr, &SENSOR_4LANE_AR0233_30FPS_12BIT_1080P_954_PWL_INFO, sizeof(MIPI_SENSOR_INFO_S));
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
		 case OV10635_30FPS_720p_960_YUV_LINE_CONCATENATED:
			 memcpy(pstMipiAttr, &MIPI_2LANE_OV10635_30FPS_YUV_720P_960_ATTR, sizeof(MIPI_ATTR_S));
			 break;
		 case AR0233_30FPS_1080P_RAW12_954_PWL:
			 memcpy(pstMipiAttr, &MIPI_4LANE_SENSOR_AR0233_30FPS_12BIT_1080P_954_ATTR, sizeof(MIPI_ATTR_S));
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
		case OV10635_30FPS_720p_960_YUV_LINE_CONCATENATED:
			memcpy(pstDevAttr, &DEV_ATTR_OV10635_YUV_LINE_CONCATE_BASE, sizeof(VIN_DEV_ATTR_S));
			break;
		case AR0233_30FPS_1080P_RAW12_954_PWL:
			 memcpy(pstDevAttr, &DEV_ATTR_AR0233_1080P_BASE, sizeof(VIN_DEV_ATTR_S));
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
		 case OV10635_30FPS_720p_960_YUV_LINE_CONCATENATED:
			 memcpy(pstPipeAttr, &PIPE_ATTR_OV10635_YUV_BASE, sizeof(VIN_PIPE_ATTR_S));
			 break;
		 case AR0233_30FPS_1080P_RAW12_954_PWL:
			 memcpy(pstPipeAttr, &PIPE_ATTR_AR0233_1080P_PWL_BASE, sizeof(VIN_PIPE_ATTR_S));
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
		 case OV10635_30FPS_720p_960_YUV_LINE_CONCATENATED:
			 memcpy(pstDisAttr, &DIS_ATTR_OV10635_BASE, sizeof(VIN_DIS_ATTR_S));
			 break;
		 case AR0233_30FPS_1080P_RAW12_954_PWL:
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
		 case OV10635_30FPS_720p_960_YUV_LINE_CONCATENATED:
			 memcpy(pstLdcAttr, &LDC_ATTR_OV10635_BASE, sizeof(VIN_LDC_ATTR_S));
			 break;
		 case AR0233_30FPS_1080P_RAW12_954_PWL:
			 memcpy(pstLdcAttr, &LDC_ATTR_BASE, sizeof(VIN_LDC_ATTR_S));
			 break;
		 default:
			printf("not surpport sensor type\n");
			break;
    }
	printf("SAMPLE_VIN_GetLdcAttrBySns success\n");
    return 0;
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

static int check_end(void)
{
	time_t now = time(NULL);
	//printf("Time info :: now(%ld), end_time(%ld) run time(%ld)!\n",now, end_time, run_time);
	return !(now > end_time && run_time > 0);
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

void print_usage(const char *prog)
{
	printf("Usage: %s \n", prog);
	puts("-p --group_id        group_id\n"
	     "  -r --run_time         time measured in seconds the program runs\n"
	     "  -t --data_type        data type\n"
	     "  -m --multi_thread	 multi thread get\n"
		"-P --need_pym\n"
		"-D --vps_dump\n"
		"-d --raw dump\n"
		"-E --extra_mode\n"
		"-I --need_ipu\n");
	exit(1);
}

void parse_opts(int argc, char *argv[])
{
	while (1) {
		static const char short_options[] =
		    "p:r:t:m:M:P:D:d:E:I:";
		static const struct option long_options[] = {
			{"groupMask", 1, 0, 'p'},
			{"run_time", 1, 0, 'r'},
			{"data_type", 1, 0, 't'},
			{"multi_thread", 1, 0, 'm'},
			{"need_pym", 1, 0, 'P'},
			{"vps_dump", 1, 0, 'D'},
			{"raw_dump", 1, 0, 'd'},
			{"extra_mode", 1, 0, 'E'},
			{"need_ipu", 1, 0, 'I'},
			{NULL, 0, 0, 0},
		};

		int cmd_ret;

		cmd_ret =
		    getopt_long(argc, argv, short_options, long_options, NULL);

		if (cmd_ret == -1)
			break;
		printf("cmd_ret %d optarg %s\n", cmd_ret, optarg);
	switch (cmd_ret) {
		case 'p':
			groupMask = atoi(optarg);;
			printf("groupMask = %d\n", groupMask);
			break;
		case 'r':
			run_time = atoi(optarg);
			printf("run_time = %ld\n", run_time);
			break;
		case 'm':
			need_m_thread = atoi(optarg);
			printf("need_m_thread = %d\n", need_m_thread);
			break;
		case 'd':
			raw_dump = atoi(optarg);
			printf("raw_dump = %d\n", raw_dump);
			break;
		case 't':
			data_type = atoi(optarg);
			printf("data_type = %d\n", data_type);
			break;
		case 'P':
			need_pym = atoi(optarg);
			printf("need_pym = %d\n", need_pym);
			break;
		case 'I':
			need_ipu = atoi(optarg);
			printf("need_ipu = %d\n", need_ipu);
			break;
		case 'D':
			vps_dump = atoi(optarg);
			printf("vps_dump = %d\n", vps_dump);
			break;
		case 'E':
			extra_mode = atoi(optarg);
			printf("extra_mode = %d\n", extra_mode);
			break;
		default:
			print_usage(argv[0]);
			break;
		}
	}
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

 void hb_vps_stop(int pipeId)
 {
	 HB_VPS_StopGrp(pipeId);
 }
 void hb_vps_deinit(int pipeId)
 {
	 HB_VPS_DestroyGrp(pipeId);
 }

 int hb_vps_init(int pipeId, uint32_t vin_vps_mode)
 {
 	int ret = 0;
	VPS_GRP_ATTR_S grp_attr;
	VPS_CHN_ATTR_S chn_attr;
	VPS_PYM_CHN_ATTR_S pym_chn_attr;
	VPS_CROP_INFO_S chn_crop_info;

    memset(&grp_attr, 0, sizeof(VPS_GRP_ATTR_S));
    grp_attr.maxW = 2560;
    grp_attr.maxH = 720;
    ret = HB_VPS_CreateGrp(pipeId, &grp_attr);
    if (ret) {
   	 printf("HB_VPS_CreateGrp error!!!\n");
    } else {
   	 printf("created a group ok:GrpId = %d\n", pipeId);
    }
	HB_SYS_SetVINVPSMode(pipeId, vin_vps_mode);

    memset(&chn_attr, 0, sizeof(VPS_CHN_ATTR_S));
    chn_attr.enScale = 1;
	chn_attr.width = 1280;
	chn_attr.height = 720;
	chn_attr.frameDepth = 6;

	memset(&chn_crop_info, 0, sizeof(VPS_CROP_INFO_S));
	chn_crop_info.en = 1;
	chn_crop_info.cropRect.x = 1280;
	chn_crop_info.cropRect.y = 0;
	chn_crop_info.cropRect.width = 1280;
	chn_crop_info.cropRect.height = 720;

	if (BIT2CHN(need_ipu, 2)) {
		ret = HB_VPS_SetChnAttr(pipeId, 2, &chn_attr);
		if (ret) {
			printf("HB_VPS_SetChnAttr error!!!\n");
		} else {
			printf("set chn Attr ok: GrpId = %d, chn_id = %d\n",
									pipeId, 2);
		}
		HB_VPS_EnableChn(pipeId, 2);
	}
	if (BIT2CHN(need_ipu, 1)) {
		ret = HB_VPS_SetChnAttr(pipeId, 1, &chn_attr);
		if (ret) {
			printf("HB_VPS_SetChnAttr error!!!\n");
		} else {
			printf("set chn Attr ok: GrpId = %d, chn_id = %d\n",
									pipeId, 1);
		}
		ret = HB_VPS_SetChnCrop(pipeId, 1, &chn_crop_info);
		if (ret)
			printf("dynamic set chn%d crop fail\n", 1);
		HB_VPS_EnableChn(pipeId, 1);
	}
	if (BIT2CHN(need_ipu, 3)) {
		ret = HB_VPS_SetChnAttr(pipeId, 3, &chn_attr);
		if (ret) {
			printf("HB_VPS_SetChnAttr error!!!\n");
		} else {
			printf("set chn Attr ok: GrpId = %d, chn_id = %d\n",
									pipeId, 3);
		}
		ret = HB_VPS_SetChnCrop(pipeId, 3, &chn_crop_info);
		if (ret)
			printf("dynamic set chn%d crop fail\n", 3);
		HB_VPS_EnableChn(pipeId, 3);
	}
	if (BIT2CHN(need_ipu, 6)) {
		ret = HB_VPS_SetChnAttr(pipeId, 6, &chn_attr);
		if (ret) {
			printf("HB_VPS_SetChnAttr error!!!\n");
		} else {
			printf("set chn Attr ok: GrpId = %d, chn_id = %d\n",
									pipeId, 6);
		}
		ret = HB_VPS_SetChnCrop(pipeId, 6, &chn_crop_info);
		if (ret)
			printf("dynamic set chn%d crop fail\n", 6);
		HB_VPS_EnableChn(pipeId, 6);
	}

	if (need_pym) {
		memset(&pym_chn_attr, 0, sizeof(VPS_PYM_CHN_ATTR_S));
		pym_chn_attr.timeout = 2000;
		pym_chn_attr.ds_layer_en = 23;
		pym_chn_attr.us_layer_en = 0;
		pym_chn_attr.frame_id = 0;
		pym_chn_attr.frameDepth = 6;
		ret = HB_VPS_SetPymChnAttr(pipeId, chns2chn(need_pym), &pym_chn_attr);  // pym chn6
		if (ret) {
			printf("HB_VPS_SetPymChnAttr error!!!\n");
		} else {
			printf("HB_VPS_SetPymChnAttr ok: grp_id = %d g_pym_chn = %d\n",
									pipeId, chns2chn(need_pym));
		}
		HB_VPS_EnableChn(pipeId, chns2chn(need_pym)); // chn6
	}

	 return ret;
 }
int hb_vin_vps_init(int pipeId, uint32_t sensorId, uint32_t mipiIdx, uint32_t deseri_port, uint32_t vin_vps_mode)
{
	int ret = 0;
	VIN_DEV_ATTR_S  devinfo;
	VIN_PIPE_ATTR_S pipeinfo;
	VIN_DIS_ATTR_S  disinfo;
	VIN_LDC_ATTR_S  ldcinfo;
	VPS_GRP_ATTR_S grp_attr;
	VPS_CHN_ATTR_S chn_attr;
	VPS_PYM_CHN_ATTR_S pym_chn_attr;
	VPS_CROP_INFO_S chn_crop_info;

	memset(&devinfo, 0, sizeof(VIN_DEV_ATTR_S));
	memset(&pipeinfo, 0, sizeof(VIN_PIPE_ATTR_S));
	memset(&disinfo, 0, sizeof(VIN_DIS_ATTR_S));
	memset(&ldcinfo, 0, sizeof(VIN_LDC_ATTR_S));
	SAMPLE_VIN_GetDevAttrBySns(sensorId,  &devinfo);
	SAMPLE_VIN_GetPipeAttrBySns(sensorId, &pipeinfo);
	SAMPLE_VIN_GetDisAttrBySns(sensorId,  &disinfo);
	SAMPLE_VIN_GetLdcAttrBySns(sensorId,  &ldcinfo);
	print_sensor_dev_info(&devinfo);
	print_sensor_pipe_info(&pipeinfo);

	ret = HB_SYS_SetVINVPSMode(pipeId, vin_vps_mode);
	if(ret < 0) {
		printf("HB_SYS_SetVINVPSMode%d error!\n", vin_vps_mode);
		return ret;
	}
	ret = HB_VIN_CreatePipe(pipeId, &pipeinfo);   // isp init
	if(ret < 0) {
		printf("HB_MIPI_InitSensor error!\n");
		return ret;
	}
	ret = HB_VIN_SetMipiBindDev(pipeId, mipiIdx);
	if(ret < 0) {
		printf("HB_VIN_SetMipiBindDev error!\n");
		return ret;
	}
	ret = HB_VIN_SetDevVCNumber(pipeId, deseri_port);
	if(ret < 0) {
		printf("HB_VIN_SetDevVCNumber error!\n");
		return ret;
	}
	ret = HB_VIN_SetDevAttr(pipeId, &devinfo);     // sif init
	if(ret < 0) {
		printf("HB_MIPI_InitSensor error!\n");
		return ret;
	}
	ret = HB_VIN_SetPipeAttr(pipeId, &pipeinfo);     // isp init
	if(ret < 0) {
		printf("HB_VIN_SetPipeAttr error!\n");
		goto pipe_err;
	}
	ret = HB_VIN_SetChnDISAttr(pipeId, 1, &disinfo);  //  dis init
	if(ret < 0) {
		printf("HB_VIN_SetChnDISAttr error!\n");
		goto pipe_err;
	}
	ret = HB_VIN_SetChnLDCAttr(pipeId, 1, &ldcinfo);   //  ldc init
	if(ret < 0) {
		printf("HB_VIN_SetChnLDCAttr error!\n");
		goto pipe_err;
	}
	ret = HB_VIN_SetChnAttr(pipeId, 1);               //  dwe init
	if(ret < 0) {
		printf("HB_VIN_SetChnAttr error!\n");
		goto chn_err;
	}
	HB_VIN_SetDevBindPipe(pipeId, pipeId);    //  bind init
	memset(&grp_attr, 0, sizeof(VPS_GRP_ATTR_S));
	grp_attr.maxW = 2560;
	grp_attr.maxH = 720;
	ret = HB_VPS_CreateGrp(pipeId, &grp_attr);
	if (ret) {
		printf("HB_VPS_CreateGrp error!!!\n");
	} else {
		printf("created a group ok:GrpId = %d\n", pipeId);
	}
	memset(&chn_crop_info, 0, sizeof(VPS_CROP_INFO_S));
	chn_crop_info.en = 1;
	chn_crop_info.cropRect.x = 0;
	chn_crop_info.cropRect.y = 0;
	chn_crop_info.cropRect.width = 1280;
	chn_crop_info.cropRect.height = 720;

	memset(&chn_attr, 0, sizeof(VPS_CHN_ATTR_S));
	chn_attr.enScale = 1;
	chn_crop_info.cropRect.x = 0;
	chn_crop_info.cropRect.y = 0;
	chn_crop_info.cropRect.width = 1280;
	chn_crop_info.cropRect.height = 720;
	chn_attr.width = 1280;
	chn_attr.height = 720;
	chn_attr.frameDepth = 6;

	if (BIT2CHN(need_ipu, 2)) {
		ret = HB_VPS_SetChnAttr(pipeId, 2, &chn_attr);
		if (ret) {
			printf("HB_VPS_SetChnAttr error!!!\n");
		} else {
			printf("set chn Attr ok: GrpId = %d, chn_id = %d\n",
									pipeId, 2);
		}
		HB_VPS_EnableChn(pipeId, 2);
	}
	if (BIT2CHN(need_ipu, 1)) {
		ret = HB_VPS_SetChnAttr(pipeId, 1, &chn_attr);
		if (ret) {
			printf("HB_VPS_SetChnAttr error!!!\n");
		} else {
			printf("set chn Attr ok: GrpId = %d, chn_id = %d\n",
									pipeId, 1);
		}
		ret = HB_VPS_SetChnCrop(pipeId, 1, &chn_crop_info);
		if (ret)
			printf("dynamic set chn%d crop fail\n", 1);
		HB_VPS_EnableChn(pipeId, 1);
	}
	if (BIT2CHN(need_ipu, 3)) {
		ret = HB_VPS_SetChnAttr(pipeId, 3, &chn_attr);
		if (ret) {
			printf("HB_VPS_SetChnAttr error!!!\n");
		} else {
			printf("set chn Attr ok: GrpId = %d, chn_id = %d\n",
									pipeId, 3);
		}
		ret = HB_VPS_SetChnCrop(pipeId, 3, &chn_crop_info);
		if (ret)
			printf("dynamic set chn%d crop fail\n", 3);
		HB_VPS_EnableChn(pipeId, 3);
	}
	if (BIT2CHN(need_ipu, 6)) {
		ret = HB_VPS_SetChnAttr(pipeId, 6, &chn_attr);
		if (ret) {
			printf("HB_VPS_SetChnAttr error!!!\n");
		} else {
			printf("set chn Attr ok: GrpId = %d, chn_id = %d\n",
									pipeId, 6);
		}
		ret = HB_VPS_SetChnCrop(pipeId, 6, &chn_crop_info);
		if (ret)
			printf("dynamic set chn%d crop fail\n", 6);
		HB_VPS_EnableChn(pipeId, 6);
	}

	if (need_pym) {
		memset(&pym_chn_attr, 0, sizeof(VPS_PYM_CHN_ATTR_S));
		pym_chn_attr.timeout = 2000;
		pym_chn_attr.ds_layer_en = 23;
		pym_chn_attr.us_layer_en = 0;
		pym_chn_attr.frame_id = 0;
		pym_chn_attr.frameDepth = 6;
		ret = HB_VPS_SetPymChnAttr(pipeId, chns2chn(need_pym), &pym_chn_attr);  // chn6
		if (ret) {
			printf("HB_VPS_SetPymChnAttr error!!!\n");
		} else {
			printf("HB_VPS_SetPymChnAttr ok: grp_id = %d g_pym_chn = %d\n",
									pipeId, chns2chn(need_pym));
		}
		HB_VPS_EnableChn(pipeId, chns2chn(need_pym));
	}

	struct HB_SYS_MOD_S src_mod, dst_mod;
	src_mod.enModId = HB_ID_VIN;
	src_mod.s32DevId = pipeId;
	src_mod.s32ChnId = 0;
	dst_mod.enModId = HB_ID_VPS;
	dst_mod.s32DevId = pipeId;
	dst_mod.s32ChnId = 0;
	ret = HB_SYS_Bind(&src_mod, &dst_mod);
	if (ret != 0)
		printf("HB_SYS_Bind failed\n");

	return ret;
pipe_err:
	HB_VIN_DestroyDev(pipeId);  // sif deinit
chn_err:
	HB_VIN_DestroyPipe(pipeId);  // isp && dwe deinit
	return ret;
}

int hb_vin_vps_raw_init(int pipeId, uint32_t sensorId, uint32_t mipiIdx, uint32_t deseri_port, uint32_t vin_vps_mode)
{
	int ret = 0;
	VIN_DEV_ATTR_S  devinfo;
	VIN_PIPE_ATTR_S pipeinfo;
	VIN_DIS_ATTR_S  disinfo;
	VIN_LDC_ATTR_S  ldcinfo;
	VPS_GRP_ATTR_S grp_attr;
	VPS_CHN_ATTR_S chn_attr;
	VPS_PYM_CHN_ATTR_S pym_chn_attr;

	memset(&devinfo, 0, sizeof(VIN_DEV_ATTR_S));
	memset(&pipeinfo, 0, sizeof(VIN_PIPE_ATTR_S));
	memset(&disinfo, 0, sizeof(VIN_DIS_ATTR_S));
	memset(&ldcinfo, 0, sizeof(VIN_LDC_ATTR_S));
	SAMPLE_VIN_GetDevAttrBySns(sensorId,  &devinfo);
	SAMPLE_VIN_GetPipeAttrBySns(sensorId, &pipeinfo);
	SAMPLE_VIN_GetDisAttrBySns(sensorId,  &disinfo);
	SAMPLE_VIN_GetLdcAttrBySns(sensorId,  &ldcinfo);
	print_sensor_dev_info(&devinfo);
	print_sensor_pipe_info(&pipeinfo);

	ret = HB_SYS_SetVINVPSMode(pipeId, vin_vps_mode);
	if(ret < 0) {
		printf("HB_SYS_SetVINVPSMode%d error!\n", vin_vps_mode);
		return ret;
	}
	ret = HB_VIN_CreatePipe(pipeId, &pipeinfo);   // isp init
	if(ret < 0) {
		printf("HB_MIPI_InitSensor error!\n");
		return ret;
	}
	ret = HB_VIN_SetMipiBindDev(pipeId, mipiIdx);
	if(ret < 0) {
		printf("HB_VIN_SetMipiBindDev error!\n");
		return ret;
	}
	ret = HB_VIN_SetDevVCNumber(pipeId, deseri_port);
	if(ret < 0) {
		printf("HB_VIN_SetDevVCNumber error!\n");
		return ret;
	}
	ret = HB_VIN_SetDevAttr(pipeId, &devinfo);     // sif init
	if(ret < 0) {
		printf("HB_MIPI_InitSensor error!\n");
		return ret;
	}
	ret = HB_VIN_SetPipeAttr(pipeId, &pipeinfo);     // isp init
	if(ret < 0) {
		printf("HB_VIN_SetPipeAttr error!\n");
		goto pipe_err;
	}
	ret = HB_VIN_SetChnDISAttr(pipeId, 1, &disinfo);  //  dis init
	if(ret < 0) {
		printf("HB_VIN_SetChnDISAttr error!\n");
		goto pipe_err;
	}
	ret = HB_VIN_SetChnLDCAttr(pipeId, 1, &ldcinfo);   //  ldc init
	if(ret < 0) {
		printf("HB_VIN_SetChnLDCAttr error!\n");
		goto pipe_err;
	}
	ret = HB_VIN_SetChnAttr(pipeId, 1);               //  dwe init
	if(ret < 0) {
		printf("HB_VIN_SetChnAttr error!\n");
		goto chn_err;
	}
	HB_VIN_SetDevBindPipe(pipeId, pipeId);    //  bind init
	memset(&grp_attr, 0, sizeof(VPS_GRP_ATTR_S));
	grp_attr.maxW = pipeinfo.stSize.width;
	grp_attr.maxH = pipeinfo.stSize.height;
	ret = HB_VPS_CreateGrp(pipeId, &grp_attr);
	if (ret) {
		printf("HB_VPS_CreateGrp error!!!\n");
	} else {
		printf("created a group ok:GrpId = %d\n", pipeId);
	}

	memset(&chn_attr, 0, sizeof(VPS_CHN_ATTR_S));
	chn_attr.enScale = 1;
	chn_attr.frameDepth = 6;
	chn_attr.width = 1280;
	chn_attr.height = 720;


	if (BIT2CHN(need_ipu, 2)) {
		ret = HB_VPS_SetChnAttr(pipeId, 2, &chn_attr);
		if (ret) {
			printf("HB_VPS_SetChnAttr error!!!\n");
		} else {
			printf("set chn Attr ok: GrpId = %d, chn_id = %d\n",
									pipeId, 2);
		}
		HB_VPS_EnableChn(pipeId, 2);
	}
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

	if (need_pym) {
		memset(&pym_chn_attr, 0, sizeof(VPS_PYM_CHN_ATTR_S));
		pym_chn_attr.timeout = 2000;
		pym_chn_attr.ds_layer_en = 23;
		pym_chn_attr.us_layer_en = 0;
		pym_chn_attr.frame_id = 0;
		pym_chn_attr.frameDepth = 6;
		ret = HB_VPS_SetPymChnAttr(pipeId, chns2chn(need_pym), &pym_chn_attr);  // chn6
		if (ret) {
			printf("HB_VPS_SetPymChnAttr error!!!\n");
		} else {
			printf("HB_VPS_SetPymChnAttr ok: grp_id = %d g_pym_chn = %d\n",
									pipeId, chns2chn(need_pym));
		}
		HB_VPS_EnableChn(pipeId, chns2chn(need_pym)); // chn6
	}

	struct HB_SYS_MOD_S src_mod, dst_mod;
	src_mod.enModId = HB_ID_VIN;
	src_mod.s32DevId = pipeId;
	src_mod.s32ChnId = 0;
	dst_mod.enModId = HB_ID_VPS;
	dst_mod.s32DevId = pipeId;
	dst_mod.s32ChnId = 0;
	ret = HB_SYS_Bind(&src_mod, &dst_mod);
	if (ret != 0)
		printf("HB_SYS_Bind failed\n");

	return ret;
pipe_err:
	HB_VIN_DestroyDev(pipeId);  // sif deinit
chn_err:
	HB_VIN_DestroyPipe(pipeId);  // isp && dwe deinit
	return ret;
}

void hb_vin_vps_deinit(int pipeId)
{
	HB_VIN_DestroyDev(pipeId);  // sif deinit && destroy
	HB_VIN_DestroyChn(pipeId, 1);  // dwe deinit
	HB_VIN_DestroyPipe(pipeId);  // isp deinit && destroy
	HB_VPS_DestroyGrp(pipeId);
}

int hb_sensor_init(int devId, int sensorId, int bus, uint32_t port, int mipiIdx, int sedres_index)
{
	int ret = 0;
	MIPI_SENSOR_INFO_S snsinfo;

	memset(&snsinfo, 0, sizeof(MIPI_SENSOR_INFO_S));
	SAMPLE_MIPI_GetSnsAttrBySns(sensorId, &snsinfo);
	  if (sensorId == OV10635_30FPS_720p_960_YUV_LINE_CONCATENATED) {
		HB_MIPI_SetExtraMode(&snsinfo, extra_mode);
	}

	printf("hb_sensor_init mipiIdx %d==port %d ===========\n", mipiIdx, port);
    HB_MIPI_SetBus(&snsinfo, bus);
    HB_MIPI_SetPort(&snsinfo, port);
	if(devId == 4) {
		port = 0;
	}
    HB_MIPI_SensorBindSerdes(&snsinfo, sedres_index, port);
    HB_MIPI_SensorBindMipi(&snsinfo,  mipiIdx);

	ret = HB_MIPI_InitSensor(devId, &snsinfo);
	if(ret < 0) {
		printf("HB_MIPI_InitSensor error!\n");
		return ret;
	}
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
	printf("HB_MIPI_SetDevAttr end\n");
	free(mipi_attr);
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
void intHandler(int dummy)
{
	int i;
	int yuv_mipiIdx = 0, raw_mipiIdx = 1;

	signal(SIGINT, SIG_IGN);

    g_exit = 1;
	for (i = 0; i < MAX_ID_NUM; i++) {
	   if (BIT(i) & groupMask) {
		   if (i == 0 || i == 2) {
			   if (i == 2) {
				   yuv_mipiIdx = 2;
			   } else if ( i == 0) {
				   yuv_mipiIdx = 0;
			   }
			   hb_sensor_stop(i);
			   hb_mipi_stop(yuv_mipiIdx);
		   }
		   if (i == 4) {
			   hb_sensor_stop(i);
			   hb_mipi_stop(raw_mipiIdx);
		   }
		}
	}
   for (i = 0; i < MAX_ID_NUM; i++) {
	   if (BIT(i) & groupMask) {
		   if (i == 0 || i == 2) {
			   hb_vin_vps_stop(i);
			   hb_vps_stop(i + 1);
		   }
		   if (i == 4) {
			   hb_vin_vps_stop(i);
		   }
	   }
   }
   for (i = 0; i < MAX_ID_NUM; i++) {
	     if (BIT(i) & groupMask) {
	  	   if (i == 0 || i == 2) {
	  		   if (i == 2) {
	  			   yuv_mipiIdx = 2;
	  		   } else if ( i == 0) {
	  			   yuv_mipiIdx = 0;
	  		   }
	  		   hb_mipi_deinit(yuv_mipiIdx);
	  		   hb_sensor_deinit(i);
	  	   }
	  	   if (i == 4) {
	  		   hb_mipi_deinit(raw_mipiIdx);
	  		   hb_sensor_deinit(i);
	  	   }
	  	}
    }
   for (i = 0; i < MAX_ID_NUM; i++) {
	   if (BIT(i) & groupMask) {
		   if (i == 0 || i == 2){
			   hb_vin_vps_deinit(i);
			   hb_vps_deinit(i + 1);
		   }
		   if (i == 4) {
			   hb_vin_vps_deinit(i);
		   }
	   }
   }

	printf("Test hb_vio_deinit done\n");
	exit(1);
}
int vps_get_dump_func(work_info_t * info)
{
	hb_vio_buffer_t *isp_yuv = NULL;
	struct timeval time_now = { 0 };
	struct timeval time_next = { 0 };
	int size = -1, ret = 0;
	char file_name[100] = { 0 };
	int time_ms = 0;
	int grp_id = info->group_id;
	VPS_CHN_ATTR_S chn_attr;
	memset(&chn_attr, 0, sizeof(VPS_CHN_ATTR_S));
	chn_attr.enScale = 1;
	chn_attr.width = 1000;
	chn_attr.height = 600;
	chn_attr.frameDepth = 6;

	gettimeofday(&time_now, NULL);
	time_ms = time_now.tv_sec * 1000 + time_now.tv_usec /1000;

	hb_vio_buffer_t out_buf;
	pym_buffer_t out_pym_buf;
	uint32_t size_y, size_uv, i;
	uint32_t frame_cnt = 0;
	time_t start, last = -1;
	int idx = 0;

	while (check_end()) {
		if (info->data_type != chns2chn(need_pym)) {
			frame_cnt++;
			ret = HB_VPS_GetChnFrame(info->group_id, info->data_type, &out_buf, 2000);
			if (ret != 0) {
				printf("HB_VPS_GetChnFrame error!!!\n");
			} else {
			    start = time(NULL);
		        idx++;
		        if (start > last) {
		            printf("\n\ngroup_id %d chn %d ipu fps %d\n", info->group_id, info->data_type, idx);
		            last = start;
		            idx = 0;
		        }
				//printf("Get Frame ok grpid=%d chn_id=%d bufindex=%d\n",
				//	info->group_id, info->data_type, out_buf.img_info.buf_index);

				if (vps_dump) {
					size_y = out_buf.img_addr.stride_size *
								out_buf.img_addr.height;
					size_uv = size_y / 2;
					printf("out:stride size = %d height = %d\n",
						out_buf.img_addr.stride_size, out_buf.img_addr.height);
					if (vps_dump != 2) {
						snprintf(file_name, sizeof(file_name),
							"grp%d_chn%d_out_%d_%d_%d.yuv", grp_id, info->data_type,
							out_buf.img_addr.width,
							out_buf.img_addr.height,
							out_buf.img_info.frame_id);
					} else {
						snprintf(file_name, sizeof(file_name),
							"grp%d_chn%d_out%d_%d_%d.yuv", grp_id, info->data_type,
							/*out_buf.img_info.frame_id,*/
							frame_cnt,
							out_buf.img_addr.width,
							out_buf.img_addr.height);
					}

					gettimeofday(&time_now, NULL);
					dumpToFile2plane(file_name, out_buf.img_addr.addr[0],
										out_buf.img_addr.addr[1],
										out_buf.img_addr.width *
										out_buf.img_addr.height,
										out_buf.img_addr.width *
										out_buf.img_addr.height / 2);
					gettimeofday(&time_next, NULL);
					int time_cost = time_cost_ms(&time_now, &time_next);
					printf("dumpToFile cost time %d ms", time_cost);
				}

				ret = HB_VPS_ReleaseChnFrame(info->group_id, info->data_type, &out_buf);
				if (ret) {
					printf("HB_VPS_ReleaseChnFrame error!!!\n");
				}
			}
			if ((frame_cnt % 10) == 0 && vps_dump == 2) {
				chn_attr.width = 1180;
				chn_attr.height = 620;
				ret = HB_VPS_SetChnAttr(info->group_id, info->data_type, &chn_attr);
				if (ret)
					printf("dynamic set chn%d attr fail\n", info->data_type);
			}

		} else if (info->data_type == chns2chn(need_pym)) {
			ret = HB_VPS_GetChnFrame(info->group_id, info->data_type, &out_pym_buf, 2000);
			if (ret) {
				printf("HB_VPS_GetChnFrame error!!!\n");
			} else {
				start = time(NULL);
		        idx++;
		        if (start > last) {
		            printf("\n\ngroup_id %d pym fps %d\n", info->group_id, idx);
		            last = start;
		            idx = 0;
		        }
				printf("Get Frame ok grpid=%d pym_chn=%d bufindex=%d\n",
					info->group_id, info->data_type, out_pym_buf.pym_img_info.buf_index);
				if (vps_dump) {
					for (i = 0; i < 6; i++) {
						snprintf(file_name, sizeof(file_name),
							"grp%d_pym_out_basic_layer_DS%d_%d_%d.yuv",
							grp_id, i * 4, out_pym_buf.pym[i].width,
							out_pym_buf.pym[i].height);
						dumpToFile2plane(file_name, out_pym_buf.pym[i].addr[0],
											out_pym_buf.pym[i].addr[1],
											out_pym_buf.pym[i].width *
											out_pym_buf.pym[i].height,
											out_pym_buf.pym[i].width *
											out_pym_buf.pym[i].height / 2);
						for (int j = 0; j < 3; j++) {
							snprintf(file_name, sizeof(file_name),
								"grp%d_pym_out_roi_layer_DS%d_%d_%d.yuv",
								grp_id, i * 4 + j + 1,
								out_pym_buf.pym_roi[i][j].width,
								out_pym_buf.pym_roi[i][j].height);
							if (out_pym_buf.pym_roi[i][j].width != 0)
								dumpToFile2plane(file_name,
								out_pym_buf.pym_roi[i][j].addr[0],
								out_pym_buf.pym_roi[i][j].addr[1],
								out_pym_buf.pym_roi[i][j].width *
								out_pym_buf.pym_roi[i][j].height,
								out_pym_buf.pym_roi[i][j].width *
								out_pym_buf.pym_roi[i][j].height / 2);
						}
						snprintf(file_name, sizeof(file_name),
							"grp%d_pym_out_us_layer_US%d_%d_%d.yuv", grp_id,
							i, out_pym_buf.us[i].width, out_pym_buf.us[i].height);
						if (out_pym_buf.us[i].width != 0)
							dumpToFile2plane(file_name, out_pym_buf.us[i].addr[0],
											out_pym_buf.us[i].addr[1],
											out_pym_buf.us[i].width *
											out_pym_buf.us[i].height,
											out_pym_buf.us[i].width *
											out_pym_buf.us[i].height / 2);
					}
				}
				ret = HB_VPS_ReleaseChnFrame(info->group_id, info->data_type, &out_pym_buf);
				if (ret) {
					printf("HB_VPS_ReleaseChnFrame error!!!\n");
				}
			}
		} else {
			printf("no type to get\n");
		}
	}

	return 0;
}

int sif_dump_func(work_info_t * info)
{
	struct timeval time_now = { 0 };
	struct timeval time_next = { 0 };
	int size = -1;
	char file_name[100] = { 0 };
	dump_info_t dump_info = {0};
	int ret = 0, i;
	hb_vio_buffer_t *sif_raw = NULL;
	int pipeId = info->group_id;

	sif_raw = (hb_vio_buffer_t *) malloc(sizeof(hb_vio_buffer_t));
	memset(sif_raw, 0, sizeof(hb_vio_buffer_t));

	ret = HB_VIN_GetDevFrame(pipeId, 0, sif_raw, 2000);
	if (ret < 0) {
		printf("HB_VIN_GetDevFrame error!!!\n");
	} else {
		if(raw_dump) {
		normal_buf_info_print(sif_raw);
		size = sif_raw->img_addr.stride_size * sif_raw->img_addr.height;
		printf("stride_size(%u) w x h%u x %u  size %d\n",
		sif_raw->img_addr.stride_size,
		sif_raw->img_addr.width, sif_raw->img_addr.height, size);

		dump_info.ctx_id = info->group_id;
		dump_info.raw.frame_id = sif_raw->img_info.frame_id;
		dump_info.raw.plane_count = sif_raw->img_info.planeCount;
		for (int i = 0; i < sif_raw->img_info.planeCount; i ++) {
			dump_info.raw.xres[i] = sif_raw->img_addr.width;
			dump_info.raw.yres[i] = sif_raw->img_addr.height;
			dump_info.raw.addr[i] = sif_raw->img_addr.addr[i];
			dump_info.raw.size[i] = size;
		}

		if(sif_raw->img_info.img_format == 8) {
			sprintf(file_name, "pipe%d_%ux%u_frame_%03d.yuv",
							dump_info.ctx_id,
							dump_info.raw.xres[0],
							dump_info.raw.yres[0],
							dump_info.raw.frame_id);
			gettimeofday(&time_now, NULL);
			dumpToFile2plane(file_name, sif_raw->img_addr.addr[0],
							sif_raw->img_addr.addr[1], size, size/2);
			gettimeofday(&time_next, NULL);
			int time_cost = time_cost_ms(&time_now, &time_next);
			printf("dumpToFile yuv cost time %d ms", time_cost);
		}
	 }
	 ret = HB_VPS_SendFrame(pipeId+1, sif_raw, 1000);
	 if (ret) {
		printf("HB_VPS_SendFrame error!!!\n");
	 } else {
		printf("HB_VPS_SendFrame ok\n");
	 }
	 ret = HB_VIN_ReleaseDevFrame(pipeId, 0, sif_raw);
	 if (ret < 0) {
		printf("HB_VIN_ReleaseDevFrame error!!!\n");
	 }
  }
	free(sif_raw);
	sif_raw = NULL;

	return 0;
}

void  *work_sif_dump_thread(void* arg)
{
	printf("work_sif_dump_thread begin\n");
	while (check_end()) {
		sif_dump_func(arg);
	}
	printf("work_sif_dump_thread end\n");

	return NULL;
}

void  *work_vps_get_frame_thread(void* arg)
{
	printf("work_vps_get_frame_thread begin\n");
	while (check_end()) {
		vps_get_dump_func(arg);
	}
	printf("work_vps_get_frame_thread end\n");

	return NULL;
}

void work_info_init(void)
{
	for (int i = 0; i < GROUP_MAX; i++) {
		for (int j = 0; j < HB_VIO_DATA_TYPE_MAX; j++) {
			work_info[i][j].group_id = i;
			work_info[i][j].data_type = j;
			work_info[i][j].running = 0;
		}
	}
}
int hb_vps_start(int pipeId)
{
	int ret = 0;

	ret = HB_VPS_StartGrp(pipeId);
	if (ret) {
		printf("HB_VPS_StartGrp error!!!\n");
	} else {
		printf("start grp ok: grp_id = %d\n", pipeId);
	}

	return ret;
}


int main(int argc, char *argv[])
{
	int ret = 0, i;
	int yuv_mode = 9, raw_mode = 3;
	int yuv_bus = 1, raw_bus = 4, port = 0;
	int vc_num = 0;
	int yuv_mipiIdx = 0, raw_mipiIdx = 1;
	int yuv_serdes = 0, raw_serdes = 1;

	parse_opts(argc, argv);

	if (argc < 2) {
		print_usage(argv[0]);
		printf("leave, World! \n");
		exit(1);
	}

	start_time = time(NULL);
	end_time = start_time + run_time;
	work_info_init();

	for(i = 0; i < MAX_ID_NUM; i++) {
		if (BIT(i) & groupMask) {
			if (i == 0 || i == 2) {
				printf("%d===groupMask %d ===\n", i, groupMask);
				if (i == 2) {
					yuv_mipiIdx = 2;
				} else if ( i == 0) {
					yuv_mipiIdx = 0;
				}
				ret = hb_vin_vps_init(i, yuv_sensorId, yuv_mipiIdx, vc_num, yuv_mode);
				if (ret < 0) {
					printf("hb_vin_init error!\n");
					return ret;
				}
				ret = hb_vps_init(i + 1, yuv_mode);
				if (ret < 0) {
					printf("hb_vps_init error!\n");
					return ret;
				}
			}
			if(i == 4) {
				printf("%d===groupMask %d ==raw_sensorId %d =\n", i, groupMask, raw_sensorId);
				ret = hb_vin_vps_raw_init(i, raw_sensorId, raw_mipiIdx, vc_num, raw_mode);
				if (ret < 0) {
					printf("hb_vin_init error!\n");
					return ret;
				}
			}
		}
	}
	for(i = 0; i < MAX_ID_NUM; i++) {
		if (BIT(i) & groupMask) {
		    printf("%d===groupMask %d =======\n", i, groupMask);
			if (i == 0 || i == 2) {
				if (i == 2) {
					yuv_mipiIdx = 2;
					port = 2;
				} else if(i == 0) {
					printf("i %d ===============\n");
					yuv_mipiIdx = 0;
					port = 0;
				}
	 			ret = hb_sensor_init(i, yuv_sensorId, yuv_bus, port, yuv_mipiIdx, yuv_serdes);
				if(ret < 0) {
					printf("hb_sensor_init error! do vio deinit\n");
					return ret;
				}
				ret = hb_mipi_init(yuv_sensorId, yuv_mipiIdx);
				if (ret < 0) {
					printf("hb_mipi_init error! do vio deinit\n");
					hb_vin_vps_deinit(i);
					return ret;
				}
			}

			if (i == 4) {
				port = 4;
				ret = hb_sensor_init(i, raw_sensorId, raw_bus, port, raw_mipiIdx, raw_serdes);
				if (ret < 0) {
					printf("hb_sensor_init error! do vio deinit\n");
					return ret;
				}
				ret = hb_mipi_init(raw_sensorId, raw_mipiIdx);
				if (ret < 0) {
					printf("hb_mipi_init error! do vio deinit\n");
					return ret;
				}
			}
		}
	}

	for(i = 0; i < MAX_ID_NUM; i++) {
		if (BIT(i) & groupMask) {
			if (i == 0 || i == 2) {
				if (i == 2) {
					yuv_mipiIdx = 2;
				} else if ( i == 0) {
					yuv_mipiIdx = 0;
				}
				ret = hb_vin_vps_start(i);
				if(ret < 0) {
					printf("hb_vin_sif_isp_start error! do cam && vio deinit\n");
					hb_sensor_deinit(i);
					hb_mipi_deinit(yuv_mipiIdx);
					hb_vin_vps_stop(i);
					hb_vin_vps_deinit(i);
					return ret;
				}
				ret = hb_vps_start(i + 1);
				if(ret < 0) {
					printf("hb_vin_sif_isp_start error! do cam && vio deinit\n");
					hb_sensor_deinit(i);
					hb_mipi_deinit(yuv_mipiIdx);
					hb_vin_vps_stop(i);
					hb_vps_deinit(i);
					hb_vin_vps_deinit(i);
					return ret;
			    }
			}
			if (i == 4) {
				ret = hb_vin_vps_start(i);
				if (ret < 0) {
					printf("hb_vin_sif_isp_start error! do cam && vio deinit\n");
					hb_sensor_deinit(i);
					hb_mipi_deinit(raw_mipiIdx);
					hb_vin_vps_stop(i);
					hb_vin_vps_deinit(i);
					return ret;
				}
			}
		}
	}
	for(i = 0; i < MAX_ID_NUM; i++) {
	if (BIT(i) & groupMask) {
		if (i == 0 || i == 2) {
			if (i == 2) {
				yuv_mipiIdx = 2;
			} else if ( i == 0) {
				yuv_mipiIdx = 0;
			}
			ret = hb_sensor_start(i);
			if (ret < 0) {
				printf("hb_mipi_start error! do cam && vio deinit\n");
				hb_sensor_stop(i);
				hb_mipi_stop(yuv_mipiIdx);
				hb_sensor_deinit(i);
				hb_mipi_deinit(yuv_mipiIdx);
				hb_vin_vps_stop(i);
				hb_vin_vps_deinit(i);
				return ret;
			}
			ret = hb_mipi_start(yuv_mipiIdx);
			if (ret < 0) {
				printf("hb_mipi_start error! do cam && vio deinit\n");
				hb_sensor_stop(i);
				hb_mipi_stop(yuv_mipiIdx);
				hb_sensor_deinit(i);
				hb_mipi_deinit(yuv_mipiIdx);
				hb_vin_vps_stop(i);
				hb_vin_vps_deinit(i);
				return ret;
			}
		}
		if (i == 4) {
			ret = hb_sensor_start(i);
			if (ret < 0) {
				printf("hb_mipi_start error! do cam && vio deinit\n");
				hb_sensor_stop(i);
				hb_mipi_stop(raw_mipiIdx);
				hb_sensor_deinit(i);
				hb_mipi_deinit(raw_mipiIdx);
				hb_vin_vps_stop(i);
				hb_vin_vps_deinit(i);
				return ret;
			}
			ret = hb_mipi_start(raw_mipiIdx);
			if (ret < 0) {
				printf("hb_mipi_start error! do cam && vio deinit\n");
				hb_sensor_stop(i);
				hb_mipi_stop(raw_mipiIdx);
				hb_sensor_deinit(i);
				hb_mipi_deinit(raw_mipiIdx);
				hb_vin_vps_stop(i);
				hb_vin_vps_deinit(i);
				return ret;
			}
		}
	 }
	}

	signal(SIGINT, intHandler);

	if (need_m_thread) {
		for (int id = 0; id < MAX_SENSOR_NUM; id++) {
			if (BIT(id) & groupMask) {
				if (data_type & RAW_ENABLE) {
					if (id == 0 || id == 2) {
						printf("data_type & RAW_ENABLE %d\n", data_type & RAW_ENABLE);
						ret = pthread_create(&work_info[id][HB_VIO_SIF_RAW_DATA].thid,
									NULL,
								   (void *)work_sif_dump_thread,
								   (void *)(&work_info[id][HB_VIO_SIF_RAW_DATA]));
						printf("pipe(%d)Test work_sif_dump_thread---running.\n", id);
					}
				}
				if(data_type & VPS_CHN0_ENABLE) {
				printf("data_type & VPS_CHN0_ENABLE %d\n", data_type & VPS_CHN0_ENABLE);
				ret = pthread_create(
							&work_info[id][HB_VIO_IPU_DS0_DATA].thid,
							NULL,
						   (void *)work_vps_get_frame_thread,
						   (void *)(&work_info[id][HB_VIO_IPU_DS0_DATA]));
				   printf("pipe(%d)Test work_vps_get_frame thread---running.\n", id);
				}
				if(data_type & VPS_CHN1_ENABLE) {
				printf("data_type & VPS_CHN0_ENABLE %d\n", data_type & VPS_CHN1_ENABLE);
				ret = pthread_create(
							&work_info[id][HB_VIO_IPU_DS1_DATA].thid,
							NULL,
						   (void *)work_vps_get_frame_thread,
						   (void *)(&work_info[id][HB_VIO_IPU_DS1_DATA]));
				   printf("pipe(%d)Test work_vps_get_frame thread---running.\n", id);
				}
				if(data_type & VPS_CHN2_ENABLE) {
				printf("data_type & VPS_CHN0_ENABLE %d\n", data_type & VPS_CHN2_ENABLE);
				ret = pthread_create(
							&work_info[id][HB_VIO_IPU_DS2_DATA].thid,
							NULL,
						   (void *)work_vps_get_frame_thread,
						   (void *)(&work_info[id][HB_VIO_IPU_DS2_DATA]));
				   printf("pipe(%d)Test work_vps_get_frame thread---running.\n", id);
				}
				if(data_type & VPS_CHN3_ENABLE) {
				printf("data_type & VPS_CHN0_ENABLE %d\n", data_type & VPS_CHN3_ENABLE);
				ret = pthread_create(
							&work_info[id][HB_VIO_IPU_DS3_DATA].thid,
							NULL,
						   (void *)work_vps_get_frame_thread,
						   (void *)(&work_info[id][HB_VIO_IPU_DS3_DATA]));
				   printf("pipe(%d)Test work_vps_get_frame thread---running.\n", id);
				}
				if(data_type & VPS_CHN4_ENABLE) {
				printf("data_type & VPS_CHN0_ENABLE %d\n", data_type & VPS_CHN4_ENABLE);
				ret = pthread_create(
							&work_info[id][HB_VIO_IPU_DS4_DATA].thid,
							NULL,
						   (void *)work_vps_get_frame_thread,
						   (void *)(&work_info[id][HB_VIO_IPU_DS4_DATA]));
				   printf("pipe(%d)Test work_vps_get_frame thread---running.\n", id);
				}
				if(data_type & VPS_CHN5_ENABLE) {
				printf("data_type & VPS_CHN0_ENABLE %d\n", data_type & VPS_CHN5_ENABLE);
				ret = pthread_create(
							&work_info[id][HB_VIO_IPU_US_DATA].thid,
							NULL,
						   (void *)work_vps_get_frame_thread,
						   (void *)(&work_info[id][HB_VIO_IPU_US_DATA]));
				   printf("pipe(%d)Test work_vps_get_frame thread---running.\n", id);
				}
				if(data_type & VPS_CHN6_ENABLE) {
				printf("data_type & VPS_CHN0_ENABLE %d\n", data_type & VPS_CHN6_ENABLE);
				ret = pthread_create(
							&work_info[id][HB_VIO_PYM_FEEDBACK_SRC_DATA].thid,
							NULL,
						   (void *)work_vps_get_frame_thread,
						   (void *)(&work_info[id][HB_VIO_PYM_FEEDBACK_SRC_DATA]));
				   printf("pipe(%d)Test work_vps_get_frame thread---running.\n", id);
				}
			}
		}
	}

	while (check_end()) {
		if (g_exit == 1)
			break;
		sleep(1);
	}
	if (need_m_thread) {
		for (int id = 0; id < MAX_SENSOR_NUM; id++) {
			if (BIT(id) & groupMask) {
				if (data_type & RAW_ENABLE) {
				   if (id == 0 || id == 2) {
				   	pthread_join(work_info[id][HB_VIO_SIF_RAW_DATA].thid, NULL);
				  	 printf("pipe(%d)Test work dump_data thread quit done..\n", id);
				   }
				}
				if(data_type & VPS_CHN0_ENABLE) {
				   pthread_join(work_info[id][HB_VIO_IPU_DS0_DATA].thid,
							NULL);
				   printf("pipe(%d)Test work raw_feedback thread quit done.\n", id);
				}
				if(data_type & VPS_CHN1_ENABLE) {
				   pthread_join(work_info[id][HB_VIO_IPU_DS1_DATA].thid,
							NULL);
				   printf("pipe(%d)Test work raw_feedback thread quit done.\n", id);
				}
				if(data_type & VPS_CHN2_ENABLE) {
				   pthread_join(work_info[id][HB_VIO_IPU_DS2_DATA].thid,
							NULL);
				   printf("pipe(%d)Test work raw_feedback thread quit done.\n", id);
				}
				if(data_type & VPS_CHN3_ENABLE) {
				   pthread_join(work_info[id][HB_VIO_IPU_DS3_DATA].thid,
							NULL);
				   printf("pipe(%d)Test work raw_feedback thread quit done.\n", id);
				}
				if(data_type & VPS_CHN4_ENABLE) {
				   pthread_join(work_info[id][HB_VIO_IPU_DS4_DATA].thid,
							NULL);
				   printf("pipe(%d)Test work raw_feedback thread quit done.\n", id);
				}
				if(data_type & VPS_CHN5_ENABLE) {
				   pthread_join(work_info[id][HB_VIO_IPU_US_DATA].thid,
							NULL);
				   printf("pipe(%d)Test work raw_feedback thread quit done.\n", id);
				}
				if(data_type & VPS_CHN6_ENABLE) {
				   pthread_join(work_info[id][HB_VIO_PYM_FEEDBACK_SRC_DATA].thid,
							NULL);
				   printf("pipe(%d)Test work raw_feedback thread quit done.\n", id);
				}
			}
		}
	}

	 for (i = 0; i < MAX_ID_NUM; i++) {
		if (BIT(i) & groupMask) {
			if (i == 0 || i == 2) {
				if (i == 2) {
					yuv_mipiIdx = 2;
				} else if( i == 0) {
					yuv_mipiIdx = 0;
				}
				hb_sensor_stop(i);
				hb_mipi_stop(yuv_mipiIdx);
			}
			if (i == 4) {
				hb_sensor_stop(i);
				hb_mipi_stop(raw_mipiIdx);
			}
		 }
	 }
	for (i = 0; i < MAX_ID_NUM; i++) {
		if (BIT(i) & groupMask) {
			if (i == 0 || i == 2) {
				hb_vin_vps_stop(i);
				hb_vps_stop(i + 1);
			}
			if (i == 4) {
				hb_vin_vps_stop(i);
			}
		}
	}

  for (i = 0; i < MAX_ID_NUM; i++) {
	if (BIT(i) & groupMask) {
		if (i == 0 || i == 2) {
			if (i == 2) {
				yuv_mipiIdx = 2;
			} else if ( i == 0) {
				yuv_mipiIdx = 0;
			}
			hb_mipi_deinit(yuv_mipiIdx);
			hb_sensor_deinit(i);
		}
		if(i == 4) {
			hb_mipi_deinit(raw_mipiIdx);
			hb_sensor_deinit(i);
		}
	 }
   }
	for (i = 0; i < MAX_ID_NUM; i++) {
		if (BIT(i) & groupMask) {
			if (i == 0 || i == 2){
				hb_vin_vps_deinit(i);
				hb_vps_deinit(i + 1);
			}
			if (i == 4) {
				hb_vin_vps_deinit(i);
			}
		}
	}
	printf("-----------------Test done success-----------------\n");
	return 0;
}
