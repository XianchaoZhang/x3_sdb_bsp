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
#include "hb_vot.h"
#include "hb_sys.h"

#define MAX_SENSOR_NUM  8
#define MAX_MIPIID_NUM  8
#define MAX_ID_NUM 8
#define MAX_PLANE 4
#define HW_TIMER 24000

#define BIT(n)  (1UL << (n))

#define RAW_ENABLE BIT(HB_VIO_SIF_RAW_DATA)  //  for raw dump/fb 9
#define YUV_ENABLE BIT(HB_VIO_ISP_YUV_DATA)   // for yuv dump
#define RAW_FEEDBACK_ENABLE BIT(HB_VIO_SIF_FEEDBACK_SRC_DATA)  // feedback
#define MD_ENABLE BIT(HB_VIO_MD_DATA)  // MD  18

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
int g_vin_fd[MAX_SENSOR_NUM];
int sensorId[MAX_SENSOR_NUM];
int mipiIdx[MAX_MIPIID_NUM];
int bus[MAX_SENSOR_NUM];
int port[MAX_SENSOR_NUM];
int serdes_index[MAX_SENSOR_NUM];
int serdes_port[MAX_SENSOR_NUM];
int extra_mode[MAX_SENSOR_NUM];

int virtual[MAX_SENSOR_NUM];

int groupMask;
int need_clk;
int raw_dump;
int yuv_dump;
int need_cam;
int need_m_thread;
int data_type;
int vin_vps_mode[MAX_SENSOR_NUM];
int need_grp_rotate;
int need_chn_rotate;
int need_ipu;
int need_pym;
int vps_dump;
int need_md;
int need_chnfd;
int need_dis;
int vc_num;
int testpattern_fps;
int need_devclk, devclk, vpuclk;
int need_raw8;
int vps_frame_rate = 0;
int mirror_ctrl = 0;
int g_exit = 0;
int vot_enable = 0;

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
	IMX327_30FPS_1952P_RAW12_LINEAR,   // 1
	IMX327_30FPS_2228P_RAW12_DOL2,     // 2
	AR0233_30FPS_1080P_RAW12_954_PWL,	 // 3
	AR0233_30FPS_1080P_RAW12_960_PWL,	 // 4
	AR0233_30FPS_1080P_RAW12_954_LINEAR,	 // 5
	AR0233_30FPS_1080P_RAW12_960_LINEAR,	// 6
	OS8A10_30FPS_3840P_RAW10_LINEAR,	 // 7
	OS8A10_30FPS_3840P_RAW10_DOL2,	    // 8
	OV10635_30FPS_720p_954_YUV,         // 9
	OV10635_30FPS_720p_960_YUV,         // 10
	SIF_TEST_PATTERN0_1080P,            // 11
	FEED_BACK_RAW12_1952P,              // 12
	SIF_TEST_PATTERN_YUV_720P,          // 13
	SIF_TEST_PATTERN_12M_RAW12,         // 14
	IMX327_30FPS_2228P_DOL2_TWO_LINEAR,     // 15
	IMX327_15FPS_3609P_RAW12_DOL3,          // 16
	IMX327_15FPS_3609P_DOL3_THREE_LINEAR,   // 17
	IMX327_15FPS_3609P_DOL3_LEF_SEF2,       // 18
	IMX327_15FPS_3609P_DOL3_THREE_FIRST_LINEAR,   // 19
	IMX327_30FPS_2228P_DOL2_TWO_FIRST_LINEAR,     // 20
	OV10635_30FPS_720p_960_YUV_LINE_CONCATENATED,  // 21
	AR0233_30FPS_1080P_RAW12_960_PWL_2LANE,	     // 22
	AR0233_30FPS_1080P_RAW12_954_PWL_2LANE,	     // 23
	SIF_TEST_PATTERN_YUV_3840P,                  // 24
	SIF_TEST_PATTERN_8M_RAW10,                   // 25
	SAMPLE_SENOSR_ID_MAX,
} MIPI_SNS_TYPE_E;

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

MIPI_SENSOR_INFO_S SENSOR_4LANE_IMX327_30FPS_12BIT_DOL2_INFO =
{
	.deseEnable = 0,
	.inputMode = INPUT_MODE_MIPI,
	.sensorInfo = {
		.bus_num = 5,
		.fps = 30,
		.resolution = 2228,
		.sensor_addr = 0x36,
		.entry_index = 1,
		.sensor_mode = 2,
		.reg_width = 16,
		.sensor_name = "imx327"
	}
};

MIPI_SENSOR_INFO_S SENSOR_4LANE_IMX327_15FPS_12BIT_DOL3_INFO =
{
	.deseEnable = 0,
	.inputMode = INPUT_MODE_MIPI,
	.sensorInfo = {
		.bus_num = 5,
		.fps = 15,
		.resolution = 3609,
		.sensor_addr = 0x36,
		.entry_index = 1,
		.sensor_mode = DOL3_M,
		.reg_width = 16,
		.sensor_name = "imx327",
	}
};

MIPI_SENSOR_INFO_S SENSOR_OS8A10_30FPS_10BIT_LINEAR_INFO =
{
	.deseEnable = 0,
	.inputMode = INPUT_MODE_MIPI,
	.sensorInfo = {
		.fps = 30,
		.resolution = 2160,
		.sensor_addr = 0x36,
		.entry_index = 1,
		.sensor_mode = 1,
		.reg_width = 16,
		.sensor_name = "os8a10"
	}
};

MIPI_SENSOR_INFO_S SENSOR_OS8A10_30FPS_10BIT_DOL2_INFO =
{
	.deseEnable = 0,
	.inputMode = INPUT_MODE_MIPI,
	.sensorInfo = {
		.fps = 30,
		.resolution = 2160,
		.sensor_addr = 0x36,
		.entry_index = 1,
		.sensor_mode = 2,
		.reg_width = 16,
		.sensor_name = "os8a10"
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

MIPI_SENSOR_INFO_S SENSOR_4LANE_AR0233_30FPS_12BIT_1080P_954_LINEAR_INFO =
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
		.sensor_mode = NORMAL_M,
		.reg_width = 16,
		.sensor_name = "ar0233",
		.deserial_index = 0,
		.deserial_port = 0
	}
};

MIPI_SENSOR_INFO_S SENSOR_4LANE_AR0233_30FPS_12BIT_1080P_960_PWL_INFO =
{
	.deseEnable = 1,
	.inputMode = INPUT_MODE_MIPI,
	.deserialInfo = {
		.bus_type = 0,
		.bus_num = 4,
		.deserial_addr = 0x30,
		.deserial_name = "s960"
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

MIPI_SENSOR_INFO_S SENSOR_4LANE_AR0233_30FPS_12BIT_1080P_960_LINEAR_INFO =
{
	.deseEnable = 1,
	.inputMode = INPUT_MODE_MIPI,
	.deserialInfo = {
		.bus_type = 0,
		.bus_num = 4,
		.deserial_addr = 0x30,
		.deserial_name = "s960"
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
		.sensor_mode = NORMAL_M,
		.reg_width = 16,
		.sensor_name = "ar0233",
		.deserial_index = 0,
		.deserial_port = 0
	}
};

MIPI_SENSOR_INFO_S SENSOR_2LANE_OV10635_30FPS_YUV_720P_954_INFO =
{
	.deseEnable = 1,
	.inputMode = INPUT_MODE_MIPI,
	.deserialInfo = {
		.bus_type = 0,
		.bus_num = 4,
		.deserial_addr = 0x3d,
		.deserial_name = "s954"
	},
	.sensorInfo = {
		.port = 0,
		.dev_port = 0,
		.bus_type = 0,
		.bus_num = 4,
		.fps = 30,
		.resolution = 720,
		.sensor_addr = 0x40,
		.serial_addr = 0x1c,
		.entry_index = 1,
		.reg_width = 16,
		.sensor_name = "ov10635",
		.deserial_index = 0,
		.deserial_port = 0
	}
};

MIPI_SENSOR_INFO_S SENSOR_2LANE_OV10635_30FPS_YUV_720P_960_INFO =
{
	.deseEnable = 1,
	.inputMode = INPUT_MODE_MIPI,
	.deserialInfo = {
		.bus_type = 0,
		.bus_num = 4,
		.deserial_addr = 0x30,
		.deserial_name = "s960"
	},
	.sensorInfo = {
		.port = 0,
		.dev_port = 0,
		.bus_type = 0,
		.bus_num = 4,
		.fps = 30,
		.resolution = 720,
		.sensor_addr = 0x40,
		.serial_addr = 0x1c,
		.entry_index = 1,
		.reg_width = 16,
		.sensor_name = "ov10635",
		.deserial_index = 0,
		.deserial_port = 0
	},
	.lpwmInfo = {
		.lpwm_enable = 0x0,
	 	.offset_us = {10, 10, 10, 10},
	 	.period_us = {33330, 33330, 33330, 33330},
	 	.duty_us = {160, 160, 160, 160},
	}
};

MIPI_SENSOR_INFO_S SENSOR_TESTPATTERN_INFO =
{
	.sensorInfo = {
		.sensor_name = "virtual",
	}
};

MIPI_ATTR_S MIPI_2LANE_OV10635_30FPS_YUV_720P_954_ATTR =
{
	.mipi_host_cfg =
	{
		2,			  /* lane */
		0x1e,		  /* datatype */
		24, 		  /* mclk	 */
		1600,		   /* mipiclk */
		30, 		  /* fps */
		1280,		  /* width	*/
		720,		  /*height */
		3207,		  /* linlength */
		748,		  /* framelength */
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
		  4,
		 {0, 1, 2, 3}
	 },
	 .dev_enable = 0  /*  mipi dev enable */
};

MIPI_ATTR_S MIPI_2LANE_OV10635_720P_LINE_CONCATE_960_ATTR =
{
   .mipi_host_cfg =
   {
	   2,			 /* lane */
	   0x1e,		 /* datatype */
	   24,			 /* mclk	*/
	   3200,		  /* mipiclk */
	   30,			 /* fps */
	   1280,		 /* width  */
	   720, 		 /*height */
	   3207,		 /* linlength */
	   748, 		 /* framelength */
	   30,			 /* settle */
		2,
	   {0, 1}
   },
   .dev_enable = 0	/*	mipi dev enable */
};

MIPI_ATTR_S MIPI_2LANE_SENSOR_AR0233_30FPS_12BIT_1080P_954_ATTR =
{
	.mipi_host_cfg =
	{
		2,			  /* lane */
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
		{0, 1, 2, 3}
	},
	.dev_enable = 0  /*  mipi dev enable */
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

MIPI_ATTR_S MIPI_2LANE_SENSOR_AR0233_30FPS_12BIT_1080P_960_ATTR =
{
 .mipi_host_cfg =
 {
	 2, 		   /* lane */
	 0x2c,		   /* datatype */
	 24,		   /* mclk	  */
	 3200,			/* mipiclk */
	 30,		   /* fps */
	 1920,		   /* width  */
	 1080,		   /*height */
	 2000,		   /* linlength */
	 1111,		   /* framelength */
	 30,		   /* settle */
	  2,
	 {0, 1}
 },
 .dev_enable = 0  /*  mipi dev enable */
};

MIPI_ATTR_S MIPI_4LANE_SENSOR_AR0233_30FPS_12BIT_1080P_960_ATTR =
{
	.mipi_host_cfg =
	{
		4,			  /* lane */
		0x2c,		  /* datatype */
		24, 		  /* mclk	 */
		3200,		   /* mipiclk */
		30, 		  /* fps */
		1920,		  /* width	*/
		1080,		  /*height */
		2000,		  /* linlength */
		1111,		  /* framelength */
		30, 		  /* settle */
		 4,
		{0,1,2,3}
	},
	.dev_enable = 0  /*  mipi dev enable */
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


MIPI_ATTR_S MIPI_4LANE_SENSOR_IMX327_30FPS_12BIT_DOL2_ATTR =
{
	.mipi_host_cfg =
	{
		4,			/* lane */
		0x2c,		/* datatype */
		24, 		/* mclk    */
		1782,		/* mipiclk */
		30, 		/* fps */
		1952,		/* width  */
		2228,		/*height */
		2152,		/* linlength */
		2300,		/* framelength */
		20			/* settle */
	},
	.dev_enable = 0 /* mipi dev enable */
};

MIPI_ATTR_S MIPI_4LANE_SENSOR_IMX327_15FPS_12BIT_DOL3_ATTR =
{
	.mipi_host_cfg =
	{
		4,			/* lane */
		0x2c,		/* datatype */
		24, 		/* mclk    */
		1782,		/* mipiclk */
		15, 		/* fps */
		1952,		/* width  */
		3609,		/*height */
		2152,		/* linlength */
		4600,		/* framelength */
		20			/* settle */
	},
	.dev_enable = 0 /* mipi dev enable */
};


MIPI_ATTR_S MIPI_SENSOR_OS8A10_30FPS_10BIT_LINEAR_ATTR =
{
	.mipi_host_cfg =
	{
		4,			  /* lane */
		0x2b,		  /* datatype */
		24, 		  /* mclk	 */
		1440,		  /* mipiclk */
		30, 		  /* fps */
		3840,		  /* width	*/
		2160,		  /*height */
		6326,		  /* linlength */
		4474,		  /* framelength */
		50, 		  /* settle */
		 4, 		  /*chnnal_num*/
		{0, 1, 2, 3}	  /*vc */
	},
	.dev_enable = 0  /*  mipi dev enable */
};

 MIPI_ATTR_S MIPI_SENSOR_OS8A10_30FPS_10BIT_LINEAR_SENSOR_CLK_ATTR =
 {
	 .mipi_host_cfg =
	 {
		 4, 		   /* lane */
		 0x2b,		   /* datatype */
		 2400,		   /* mclk	  */
		 1440,		   /* mipiclk */
		 30,		   /* fps */
		 3840,		   /* width  */
		 2160,		   /*height */
		 6326,		   /* linlength */
		 4474,		   /* framelength */
		 50,		   /* settle */
		  4,		   /*chnnal_num*/
		 {0, 1, 2, 3}	   /*vc */
	 },
	 .dev_enable = 0  /*  mipi dev enable */
};

MIPI_ATTR_S MIPI_SENSOR_OS8A10_30FPS_10BIT_DOL2_ATTR =
{
	.mipi_host_cfg =
	{
		4,			  /* lane */
		0x2b,		  /* datatype */
		24, 		  /* mclk	 */
		2880,		   /* mipiclk */
		30, 		  /* fps */
		3840,		  /* width	*/
		2160,		  /*height */
		5084,		  /* linlength */
		4474,		  /* framelength */
		20, 		  /* settle */
		 4, 		  /*chnnal_num*/
		{0, 1, 2, 3}	  /*vc */
	},
	.dev_enable = 0  /*  mipi dev enable */
};
MIPI_ATTR_S MIPI_SENSOR_OS8A10_30FPS_10BIT_DOL2_CLK_ATTR =
{
	 .mipi_host_cfg =
	 {
		 4, 		   /* lane */
		 0x2b,		   /* datatype */
		 2400,		   /* mclk	  */
		 2880,			/* mipiclk */
		 30,		   /* fps */
		 3840,		   /* width  */
		 2160,		   /*height */
		 5084,		   /* linlength */
		 4474,		   /* framelength */
		 20,		   /* settle */
		  4,		   /*chnnal_num*/
		 {0, 1, 2, 3}	   /*vc */
	 },
	 .dev_enable = 0  /*  mipi dev enable */
};

VIN_DEV_ATTR_S DEV_ATTR_AR0233_1080P_BASE = {
	.stSize = { 0,		/*format*/
				1920,	/*width*/
				1080,	 /*height*/
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

VIN_DEV_ATTR_S DEV_ATTR_IMX327_DOL2_BASE = {
	.stSize = { 0, 		/*format*/
				1948, 	/*width*/
				1109,	 /*height*/
				2    	 /*pix_length*/
	},
	.mipiAttr = {
		.enable = 1,
		.ipi_channels = 2,
		.enable_mux_out = 1,
		.enable_frame_id = 1,
		.enable_bypass = 0,
		.enable_line_shift = 0,
		.enable_id_decoder = 1,
		.set_init_frame_id = 0,
		.set_line_shift_count = 0,
		.set_bypass_channels = 1,
	},
	.DdrIspAttr = {
		.buf_num = 4,
		.raw_feedback_en = 0,
		.data = {
			.format = 0,
			.width = 1948,
			.height = 1109,
			.pix_length = 2,
		}
	},
	.outDdrAttr = {
		.stride = 2928,
		.buffer_num = 8,
	},
	.outIspAttr = {
		.dol_exp_num = 2,
		.enable_dgain = 0,
		.set_dgain_short = 0,
		.set_dgain_medium = 0,
		.set_dgain_long = 0,
		.vc_short_seq = 0,
		.vc_medium_seq = 0,
	    .vc_long_seq = 1,
	},
};

VIN_DEV_ATTR_S DEV_ATTR_IMX327_DOL3_BASE = {
	.stSize = { 0,		/*format*/
				1948,	/*width*/
				1109,	 /*height*/
				2		 /*pix_length*/
	},
	.mipiAttr = {
		.enable = 1,
		.ipi_channels = 3,
		.enable_mux_out = 1,
		.enable_frame_id = 1,
		.enable_bypass = 0,
		.enable_line_shift = 0,
		.enable_id_decoder = 1,
		.set_init_frame_id = 0,
		.set_line_shift_count = 0,
		.set_bypass_channels = 1,
	},
	.DdrIspAttr = {
		.buf_num = 4,
		.raw_feedback_en = 0,
		.data = {
			.format = 0,
			.width = 1948,
			.height = 1109,
			.pix_length = 2,
		}
	},
	.outDdrAttr = {
		.stride = 2928,
		.buffer_num = 8,
	},
	.outIspAttr = {
		.dol_exp_num = 3,
		.enable_dgain = 0,
		.set_dgain_short = 0,
		.set_dgain_medium = 0,
		.set_dgain_long = 0,
		.vc_short_seq = 0,
		.vc_medium_seq = 1,
		.vc_long_seq = 2,
	},
};

VIN_DEV_ATTR_S DEV_ATTR_IMX327_DOL2_TWO_LINEAR_BASE = {
	.stSize = { 0,		/*format*/
				1948,	/*width*/
				1109,	 /*height*/
				2		 /*pix_length*/
	},
	.mipiAttr = {
		.enable = 1,
		.ipi_channels = 1,
		.ipi_mode = 2,
		.enable_mux_out = 1,
		.enable_frame_id = 1,
		.enable_bypass = 0,
		.enable_line_shift = 0,
		.enable_id_decoder = 1,
		.set_init_frame_id = 0,
		.set_line_shift_count = 0,
		.set_bypass_channels = 1,
	},
	.DdrIspAttr = {
		.buf_num = 4,
		.raw_feedback_en = 0,
		.data = {
			.format = 0,
			.width = 1948,
			.height = 1109,
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
	},
};
VIN_DEV_ATTR_S DEV_ATTR_IMX327_DOL2_TWO_LINEAR_FIRST_BASE = {
	.stSize = { 0,		/*format*/
				1948,	/*width*/
				1109,	 /*height*/
				2		 /*pix_length*/
	},
	.mipiAttr = {
		.enable = 1,
		.ipi_channels = 2,
		.ipi_mode = 2,
		.enable_mux_out = 1,
		.enable_frame_id = 1,
		.enable_bypass = 0,
		.enable_line_shift = 0,
		.enable_id_decoder = 1,
		.set_init_frame_id = 0,
		.set_line_shift_count = 0,
		.set_bypass_channels = 1,
	},
	.DdrIspAttr = {
		.buf_num = 4,
		.raw_feedback_en = 0,
		.data = {
			.format = 0,
			.width = 1948,
			.height = 1109,
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
	},
};

VIN_DEV_ATTR_S DEV_ATTR_IMX327_DOL3_LEF_SEF1_BASE = {
	.stSize = { 0,		/*format*/
				1948,	/*width*/
				1109,	 /*height*/
				2		 /*pix_length*/
	},
	.mipiAttr = {
		.enable = 1,
		.ipi_channels = 2,
		.ipi_mode = 3,
		.enable_mux_out = 1,
		.enable_frame_id = 1,
		.enable_bypass = 0,
		.enable_line_shift = 0,
		.enable_id_decoder = 1,
		.set_init_frame_id = 0,
		.set_line_shift_count = 0,
		.set_bypass_channels = 1,
	},
	.DdrIspAttr = {
		.buf_num = 4,
		.raw_feedback_en = 0,
		.data = {
			.format = 0,
			.width = 1948,
			.height = 1109,
			.pix_length = 2,
		}
	},
	.outDdrAttr = {
		.stride = 2928,
		.buffer_num = 8,
	},
	.outIspAttr = {
		.dol_exp_num = 2,
		.enable_dgain = 0,
		.set_dgain_short = 0,
		.set_dgain_medium = 0,
		.set_dgain_long = 0,
		.vc_short_seq = 0,
		.vc_medium_seq = 0,
		.vc_long_seq = 1,
	},
};


VIN_DEV_ATTR_S DEV_ATTR_IMX327_DOL3_THREE_LINEAR_BASE = {
	.stSize = { 0,		/*format*/
				1948,	/*width*/
				1109,	 /*height*/
				2		 /*pix_length*/
	},
	.mipiAttr = {
		.enable = 1,
		.ipi_channels = 1,
		.ipi_mode = 3,
		.enable_mux_out = 1,
		.enable_frame_id = 1,
		.enable_bypass = 0,
		.enable_line_shift = 0,
		.enable_id_decoder = 1,
		.set_init_frame_id = 0,
		.set_line_shift_count = 0,
		.set_bypass_channels = 1,
	},
	.DdrIspAttr = {
		.buf_num = 4,
		.raw_feedback_en = 0,
		.data = {
			.format = 0,
			.width = 1948,
			.height = 1109,
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
	},
};

VIN_DEV_ATTR_S DEV_ATTR_IMX327_DOL3_THREE_LINEAR_FIRST_BASE = {
	.stSize = { 0,		/*format*/
				1948,	/*width*/
				1109,	 /*height*/
				2		 /*pix_length*/
	},
	.mipiAttr = {
		.enable = 1,
		.ipi_channels = 3,
		.ipi_mode = 3,
		.enable_mux_out = 1,
		.enable_frame_id = 1,
		.enable_bypass = 0,
		.enable_line_shift = 0,
		.enable_id_decoder = 1,
		.set_init_frame_id = 0,
		.set_line_shift_count = 0,
		.set_bypass_channels = 1,
	},
	.DdrIspAttr = {
		.buf_num = 4,
		.raw_feedback_en = 0,
		.data = {
			.format = 0,
			.width = 1948,
			.height = 1109,
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
	},
};


VIN_DEV_ATTR_S DEV_ATTR_OS8A10_LINEAR_BASE = {
	.stSize = { 0,		/*format*/
				3840,	/*width*/
				2160,	 /*height*/
				1		 /*pix_length*/
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
			.width = 3840,
			.height = 2160,
			.pix_length = 1,
		}
	},
	.outDdrAttr = {
		.stride = 4800,
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

VIN_DEV_ATTR_S DEV_ATTR_OS8A10_DOL2_BASE = {
	.stSize = { 0,		/*format*/
				3840,	/*width*/
				2160,	 /*height*/
				1		 /*pix_length*/
	},
	.mipiAttr = {
		.enable = 1,
		.ipi_channels = 2,
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
			.width = 3840,
			.height = 2160,
			.pix_length = 1,
		}
	},
	.outDdrAttr = {
		.stride = 4800,
		.buffer_num = 8,
	},
	.outIspAttr = {
		.dol_exp_num = 2,
		.enable_dgain = 0,
		.set_dgain_short = 0,
		.set_dgain_medium = 0,
		.set_dgain_long = 0,
		.short_maxexp_lines = 231,
		.medium_maxexp_lines = 0,
		.vc_short_seq = 0,
		.vc_medium_seq = 0,
		.vc_long_seq = 1,
	}
};
VIN_DEV_ATTR_S DEV_ATTR_OV10635_YUV_RAW8_BASE = {
	.stSize = { 1,		/*format*/
				3840,	/*width*/
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
			.width = 1280,
			.height = 720,
			.pix_length = 0,
		}
	},
	.outDdrAttr = {
		.stride = 3840,
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

VIN_DEV_ATTR_S DEV_ATTR_OV10635_YUV_BASE = {
	.stSize = { 8,		/*format*/
				1280,	/*width*/
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
			.width = 1280,
			.height = 720,
			.pix_length = 0,
		}
	},
	.outDdrAttr = {
		.stride = 1280,
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

VIN_DEV_ATTR_S DEV_ATTR_FEED_BACK_1097P_BASE = {
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
		.raw_feedback_en = 1,
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
	}
};

VIN_DEV_ATTR_EX_S DEV_ATTR_IMX327_MD_BASE = {
	.path_sel = 0,
	.roi_top = 0,
	.roi_left = 0,
	.roi_width = 1280,
    .roi_height = 640,
    .grid_step = 128,
    .grid_tolerance = 10,
    .threshold = 10,
    .weight_decay = 128,
    .precision = 0
};
VIN_DEV_ATTR_EX_S DEV_ATTR_OV10635_MD_BASE = {
	.path_sel = 1,
	.roi_top = 0,
	.roi_left = 0,
	.roi_width = 1280,
	.roi_height = 640,
	.grid_step = 128,
	.grid_tolerance = 10,
	.threshold = 10,
	.weight_decay = 128,
	.precision = 0
};

VIN_PIPE_ATTR_S PIPE_ATTR_OV10635_YUV_BASE = {
	.ddrOutBufNum = 6,
	.snsMode = SENSOR_NORMAL_MODE,
	.stSize = {
		.format = 0,
		.width = 1280,
		.height = 720,
	},
	.ispBypassEn = 1,
	.ispAlgoState = 0,
	.bitwidth = 12,
};

VIN_PIPE_ATTR_S PIPE_ATTR_IMX327_DOL2_BASE = {
	.ddrOutBufNum = 5,
	.snsMode = SENSOR_DOL2_MODE,
	.stSize = {
		.format = 0,
		.width = 1920,
		.height = 1080,
	},
	.temperMode = 2,
	.ispBypassEn = 0,
	.ispAlgoState = 1,
	.bitwidth = 12,
	.startX = 0,
	.startY = 12,
	.calib = {
		.mode = 1,
		.lname = "libimx327_linear.so",
	}
};

VIN_PIPE_ATTR_S PIPE_ATTR_IMX327_DOL3_BASE = {
	.ddrOutBufNum = 8,
	.snsMode = SENSOR_DOL3_MODE,
	.stSize = {
		.format = 0,
		.width = 1920,
		.height = 1080,
	},
	.temperMode = 2,
	.ispBypassEn = 0,
	.ispAlgoState = 1,
	.bitwidth = 12,
	.startX = 0,
	.startY = 12,
	.calib = {
		.mode = 1,
		.lname = "libimx327_linear.so",
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
	.startX = 0,
	.startY = 12,
	.calib = {
		.mode = 1,
		.lname = "libimx327_linear.so",
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

VIN_PIPE_ATTR_S PIPE_ATTR_AR0233_1080P_LINEAR_BASE = {
	.ddrOutBufNum = 6,
	.snsMode = NORMAL_M,
	.stSize = {
		.format = 0,
		.width = 1920,
		.height = 1080,
	},
	.cfaPattern = 0,
	.temperMode = 2,
	.ispBypassEn = 0,
	.ispAlgoState = 1,
	.bitwidth = 12,
	.calib = {
		.mode = 1,
		.lname = "lib_ar0233_linear.so",
	}
};

VIN_PIPE_ATTR_S PIPE_ATTR_OS8A10_LINEAR_BASE = {
	.ddrOutBufNum = 6,
	.snsMode = SENSOR_NORMAL_MODE,
	.stSize = {
		.format = 0,
		.width = 3840,
		.height = 2160,
	},
	.cfaPattern = 3,
	.temperMode = 2,
	.ispBypassEn = 0,
	.ispAlgoState = 1,
	.bitwidth = 10,
	.calib = {
		.mode = 1,
		.lname = "libos8a10_linear.so",
	}
};

VIN_PIPE_ATTR_S PIPE_ATTR_OS8A10_DOL2_BASE = {
	.ddrOutBufNum = 6,
	.snsMode = SENSOR_DOL2_MODE,
	.stSize = {
		.format = 0,
		.width = 3840,
		.height = 2160,
	},
	.cfaPattern = 3,
	.temperMode = 2,
	.ispBypassEn = 0,
	.ispAlgoState = 1,
	.bitwidth = 10,
	.calib = {
		.mode = 1,
		.lname = "libos8a10_dol2.so",
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

VIN_DIS_ATTR_S DIS_ATTR_OS8A10_BASE = {
 .picSize = {
	 .pic_w = 3839,
	 .pic_h = 2159,
 },
 .disPath = {
	 .rg_dis_enable = 0,
	 .rg_dis_path_sel = 1,
 },
 .disHratio = 65536,
 .disVratio = 65536,
 .xCrop = {
	 .rg_dis_start = 0,
	 .rg_dis_end = 3839,
 },
 .yCrop = {
	 .rg_dis_start = 0,
	 .rg_dis_end = 2159,
 }
};

VIN_DIS_ATTR_S DIS_ATTR_12M_BASE = {
 .picSize = {
	 .pic_w = 3999,
	 .pic_h = 2999,
 },
 .disPath = {
	 .rg_dis_enable = 0,
	 .rg_dis_path_sel = 1,
 },
 .disHratio = 65536,
 .disVratio = 65536,
 .xCrop = {
	 .rg_dis_start = 0,
	 .rg_dis_end = 3999,
 },
 .yCrop = {
	 .rg_dis_start = 0,
	 .rg_dis_end = 2999,
 }
};

VIN_LDC_ATTR_S LDC_ATTR_12M_BASE = {
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
	 .pic_w = 3999,
	 .pic_h = 2999,
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
	 .rg_length = 3999,
  },
  .yWoi = {
	 .rg_start = 0,
	 .rg_length = 2999,
  }
};


VIN_LDC_ATTR_S LDC_DOL3_ATTR_BASE = {
  .ldcEnable = 0,
  .ldcPath = {
	 .rg_y_only = 0,
	 .rg_uv_mode = 0,
	 .rg_uv_interpo = 0,
	 .rg_h_blank_cyc = 32,
  },
  .yStartAddr = 1146880,
  .cStartAddr = 1163264,
  .picSize = {
	 .pic_w = 1919,
	 .pic_h = 1079,
  },
  .lineBuf = 5,
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

VIN_LDC_ATTR_S LDC_ATTR_OS8A10_DOL2_BASE = {
  .ldcEnable = 0,
  .ldcPath = {
	 .rg_y_only = 0,
	 .rg_uv_mode = 0,
	 .rg_uv_interpo = 0,
	 .rg_h_blank_cyc = 32,
  },
  .yStartAddr = 1146880,
  .cStartAddr = 1163264,
  .picSize = {
	 .pic_w = 3839,
	 .pic_h = 2159,
  },
  .lineBuf = 5,
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
	 .rg_length = 3839,
  },
  .yWoi = {
	 .rg_start = 0,
	 .rg_length = 2159,
  }
};

VIN_LDC_ATTR_S LDC_ATTR_OS8A10_LINEAR_BASE = {
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
	 .pic_w = 3839,
	 .pic_h = 2159,
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
	 .rg_length = 3839,
  },
  .yWoi = {
	 .rg_start = 0,
	 .rg_length = 2159,
  }
};

VIN_DEV_ATTR_S DEV_ATTR_TEST_PATTERN_YUV422_BASE = {
  .stSize = { 8,	/*format*/
			1280,	/*width*/
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
	.enable_pattern = 1,
  },
  .DdrIspAttr = {
	.buf_num = 8,
	.raw_feedback_en = 0,
	.data = {
		.format = 8,
		.width = 1280,
		.height = 720,
		.pix_length = 2,
	}
  },
  .outDdrAttr = {
	.stride = 1280,
	.buffer_num = 6,
  },
  .outIspAttr = {
	.dol_exp_num = 1,
	.enable_dgain = 0,
	.set_dgain_short = 0,
	.set_dgain_medium = 0,
	.set_dgain_long = 0,
  }
};

VIN_DEV_ATTR_S DEV_ATTR_TEST_PATTERN_YUV422_8M_BASE = {
  .stSize = { 8,	/*format*/
			3840,	/*width*/
			2160,	 /*height*/
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
	.enable_pattern = 1,
  },
  .DdrIspAttr = {
	.buf_num = 8,
	.raw_feedback_en = 0,
	.data = {
		.format = 8,
		.width = 1280,
		.height = 720,
		.pix_length = 2,
	}
  },
  .outDdrAttr = {
	.stride = 3840,
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


VIN_DEV_ATTR_S DEV_ATTR_TEST_PATTERN_1080P_BASE = {
  .stSize = { 0,	/*format*/
			1920,	/*width*/
			1080,	 /*height*/
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
	.enable_pattern = 1,
  },
  .DdrIspAttr = {
	.buf_num = 6,
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

VIN_DEV_ATTR_S DEV_ATTR_TEST_PATTERN_4K_BASE = {
  .stSize = { 0,	/*format*/
			3840,	/*width*/
			2160,	 /*height*/
			1		 /*pix_length*/
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
	.enable_pattern = 1,
  },
  .DdrIspAttr = {
	.buf_num = 8,
	.raw_feedback_en = 0,
	.data = {
		.format = 0,
		.width = 3840,
		.height = 2160,
		.pix_length = 1,
	}
  },
  .outDdrAttr = {
	.stride = 4800,
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

VIN_DEV_ATTR_S DEV_ATTR_TEST_PATTERN_12M_BASE = {
  .stSize = { 0,	/*format*/
			4000,	/*width*/
			3000,	 /*height*/
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
	.enable_pattern = 1,
  },
  .DdrIspAttr = {
	.buf_num = 8,
	.raw_feedback_en = 0,
	.data = {
		.format = 0,
		.width = 4000,
		.height = 3000,
		.pix_length = 2,
	}
  },
  .outDdrAttr = {
	.stride = 6000,
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

VIN_DEV_ATTR_S DEV_ATTR_TEST_PATTERN_8M_BASE = {
  .stSize = { 0,	/*format*/
			3840,	/*width*/
			2160,	 /*height*/
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
	.enable_pattern = 1,
  },
  .DdrIspAttr = {
	.buf_num = 8,
	.raw_feedback_en = 0,
	.data = {
		.format = 0,
		.width = 3840,
		.height = 2160,
		.pix_length = 2,
	}
  },
  .outDdrAttr = {
	.stride = 4800,
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


VIN_PIPE_ATTR_S PIPE_ATTR_TEST_PATTERN_1080P_BASE = {
	  .ddrOutBufNum = 6,
	  .snsMode = SENSOR_NORMAL_MODE,
	  .stSize = {
		 .format = 0,
		 .width = 1920,
		 .height = 1080,
	  },
	  .temperMode = 0,
	  .ispBypassEn = 0,
	  .ispAlgoState = 1,
	  .bitwidth = 12,
};

VIN_PIPE_ATTR_S PIPE_ATTR_TEST_PATTERN_12M_BASE = {
	  .ddrOutBufNum = 6,
	  .snsMode = SENSOR_NORMAL_MODE,
	  .stSize = {
		 .format = 0,
		 .width = 4000,
		 .height = 3000,
	  },
	  .temperMode = 0,
	  .ispBypassEn = 0,
	  .ispAlgoState = 1,
	  .bitwidth = 12,
};

VIN_PIPE_ATTR_S PIPE_ATTR_TEST_PATTERN_4K_BASE = {
   .ddrOutBufNum = 6,
   .snsMode = SENSOR_NORMAL_MODE,
   .stSize = {
	  .format = 0,
	  .width = 3840,
	  .height = 2160,
   },
   .temperMode = 3,
   .ispBypassEn = 0,
   .ispAlgoState = 0,
   .bitwidth = 10,
};

int SAMPLE_MIPI_GetSnsAttrBySns(MIPI_SNS_TYPE_E enSnsType, MIPI_SENSOR_INFO_S* pstSnsAttr)
{
	 switch (enSnsType)
	 {
		 case IMX327_30FPS_1952P_RAW12_LINEAR:
			 memcpy(pstSnsAttr, &SENSOR_4LANE_IMX327_30FPS_12BIT_LINEAR_INFO, sizeof(MIPI_SENSOR_INFO_S));
			 break;
		 case IMX327_30FPS_2228P_RAW12_DOL2:
		 case IMX327_30FPS_2228P_DOL2_TWO_LINEAR:
		 case IMX327_30FPS_2228P_DOL2_TWO_FIRST_LINEAR:
			 memcpy(pstSnsAttr, &SENSOR_4LANE_IMX327_30FPS_12BIT_DOL2_INFO, sizeof(MIPI_SENSOR_INFO_S));
			 break;
		 case IMX327_15FPS_3609P_RAW12_DOL3:
		 case IMX327_15FPS_3609P_DOL3_THREE_LINEAR:
		 case IMX327_15FPS_3609P_DOL3_LEF_SEF2:
		 case IMX327_15FPS_3609P_DOL3_THREE_FIRST_LINEAR:
			 memcpy(pstSnsAttr, &SENSOR_4LANE_IMX327_15FPS_12BIT_DOL3_INFO, sizeof(MIPI_SENSOR_INFO_S));
			 break;
		 case AR0233_30FPS_1080P_RAW12_954_PWL:
		 case AR0233_30FPS_1080P_RAW12_954_PWL_2LANE:
			 memcpy(pstSnsAttr, &SENSOR_4LANE_AR0233_30FPS_12BIT_1080P_954_PWL_INFO, sizeof(MIPI_SENSOR_INFO_S));
			 break;
		 case AR0233_30FPS_1080P_RAW12_960_PWL:
		 case AR0233_30FPS_1080P_RAW12_960_PWL_2LANE:
			 memcpy(pstSnsAttr, &SENSOR_4LANE_AR0233_30FPS_12BIT_1080P_960_PWL_INFO, sizeof(MIPI_SENSOR_INFO_S));
			 break;
		  case AR0233_30FPS_1080P_RAW12_954_LINEAR:
			 memcpy(pstSnsAttr, &SENSOR_4LANE_AR0233_30FPS_12BIT_1080P_954_LINEAR_INFO, sizeof(MIPI_SENSOR_INFO_S));
			 break;
		 case AR0233_30FPS_1080P_RAW12_960_LINEAR:
			 memcpy(pstSnsAttr, &SENSOR_4LANE_AR0233_30FPS_12BIT_1080P_960_LINEAR_INFO, sizeof(MIPI_SENSOR_INFO_S));
			 break;
		 case OS8A10_30FPS_3840P_RAW10_LINEAR:
			 memcpy(pstSnsAttr, &SENSOR_OS8A10_30FPS_10BIT_LINEAR_INFO, sizeof(MIPI_SENSOR_INFO_S));
			 break;
		 case OS8A10_30FPS_3840P_RAW10_DOL2:
			 memcpy(pstSnsAttr, &SENSOR_OS8A10_30FPS_10BIT_DOL2_INFO, sizeof(MIPI_SENSOR_INFO_S));
			 break;
		 case OV10635_30FPS_720p_954_YUV:
			 memcpy(pstSnsAttr, &SENSOR_2LANE_OV10635_30FPS_YUV_720P_954_INFO, sizeof(MIPI_SENSOR_INFO_S));
			 break;
		 case OV10635_30FPS_720p_960_YUV:
		 case OV10635_30FPS_720p_960_YUV_LINE_CONCATENATED:
		 	 memcpy(pstSnsAttr, &SENSOR_2LANE_OV10635_30FPS_YUV_720P_960_INFO, sizeof(MIPI_SENSOR_INFO_S));
			 break;
		 case SIF_TEST_PATTERN0_1080P:
		 case SIF_TEST_PATTERN_YUV_720P:
		 case SIF_TEST_PATTERN_12M_RAW12:
		 case SIF_TEST_PATTERN_YUV_3840P:
		 case SIF_TEST_PATTERN_8M_RAW10:
		 	 memcpy(pstSnsAttr, &SENSOR_TESTPATTERN_INFO, sizeof(MIPI_SENSOR_INFO_S));
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
		 case IMX327_30FPS_2228P_RAW12_DOL2:
		 case IMX327_30FPS_2228P_DOL2_TWO_LINEAR:
		 case IMX327_30FPS_2228P_DOL2_TWO_FIRST_LINEAR:
			 memcpy(pstMipiAttr, &MIPI_4LANE_SENSOR_IMX327_30FPS_12BIT_DOL2_ATTR, sizeof(MIPI_ATTR_S));
			 break;
		 case IMX327_15FPS_3609P_RAW12_DOL3:
		 case IMX327_15FPS_3609P_DOL3_THREE_LINEAR:
		 case IMX327_15FPS_3609P_DOL3_THREE_FIRST_LINEAR:
		 case IMX327_15FPS_3609P_DOL3_LEF_SEF2:
		 	 memcpy(pstMipiAttr, &MIPI_4LANE_SENSOR_IMX327_15FPS_12BIT_DOL3_ATTR, sizeof(MIPI_ATTR_S));
			 break;
		 case AR0233_30FPS_1080P_RAW12_954_PWL:
		 case AR0233_30FPS_1080P_RAW12_954_LINEAR:
			 memcpy(pstMipiAttr, &MIPI_4LANE_SENSOR_AR0233_30FPS_12BIT_1080P_954_ATTR, sizeof(MIPI_ATTR_S));
			 break;
		 case AR0233_30FPS_1080P_RAW12_960_PWL:
		 case AR0233_30FPS_1080P_RAW12_960_LINEAR:
			 memcpy(pstMipiAttr, &MIPI_4LANE_SENSOR_AR0233_30FPS_12BIT_1080P_960_ATTR, sizeof(MIPI_ATTR_S));
			 break;
		 case AR0233_30FPS_1080P_RAW12_960_PWL_2LANE:
			 memcpy(pstMipiAttr, &MIPI_2LANE_SENSOR_AR0233_30FPS_12BIT_1080P_960_ATTR, sizeof(MIPI_ATTR_S));
			 break;
		 case AR0233_30FPS_1080P_RAW12_954_PWL_2LANE:
		 	 memcpy(pstMipiAttr, &MIPI_2LANE_SENSOR_AR0233_30FPS_12BIT_1080P_960_ATTR, sizeof(MIPI_ATTR_S));
			 break;
		 case OS8A10_30FPS_3840P_RAW10_LINEAR:
			 if (need_clk == 1) {
			     memcpy(pstMipiAttr, &MIPI_SENSOR_OS8A10_30FPS_10BIT_LINEAR_SENSOR_CLK_ATTR,
				 		sizeof(MIPI_ATTR_S));
			 } else {
		         memcpy(pstMipiAttr, &MIPI_SENSOR_OS8A10_30FPS_10BIT_LINEAR_ATTR,
				 		sizeof(MIPI_ATTR_S));
			 }
			 break;
		 case OS8A10_30FPS_3840P_RAW10_DOL2:
		 	 if (need_clk == 1) {
			    memcpy(pstMipiAttr, &MIPI_SENSOR_OS8A10_30FPS_10BIT_DOL2_CLK_ATTR,
							sizeof(MIPI_ATTR_S));
		 	 } else {
		 	    memcpy(pstMipiAttr, &MIPI_SENSOR_OS8A10_30FPS_10BIT_DOL2_ATTR,
							sizeof(MIPI_ATTR_S));
			 }
			 break;
		 case OV10635_30FPS_720p_954_YUV:
			 memcpy(pstMipiAttr, &MIPI_2LANE_OV10635_30FPS_YUV_720P_954_ATTR, sizeof(MIPI_ATTR_S));
			 break;
		 case OV10635_30FPS_720p_960_YUV:
			 memcpy(pstMipiAttr, &MIPI_2LANE_OV10635_30FPS_YUV_720P_960_ATTR, sizeof(MIPI_ATTR_S));
			 break;
		 case OV10635_30FPS_720p_960_YUV_LINE_CONCATENATED:
			 memcpy(pstMipiAttr, &MIPI_2LANE_OV10635_720P_LINE_CONCATE_960_ATTR, sizeof(MIPI_ATTR_S));
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
		 case IMX327_30FPS_2228P_RAW12_DOL2:
			 memcpy(pstDevAttr, &DEV_ATTR_IMX327_DOL2_BASE, sizeof(VIN_DEV_ATTR_S));
			 break;
		 case IMX327_30FPS_2228P_DOL2_TWO_LINEAR:
			 memcpy(pstDevAttr, &DEV_ATTR_IMX327_DOL2_TWO_LINEAR_BASE, sizeof(VIN_DEV_ATTR_S));
			 break;
		 case IMX327_30FPS_2228P_DOL2_TWO_FIRST_LINEAR:
		 	 memcpy(pstDevAttr, &DEV_ATTR_IMX327_DOL2_TWO_LINEAR_FIRST_BASE,
					 sizeof(VIN_DEV_ATTR_S));
			 break;
		 case IMX327_15FPS_3609P_RAW12_DOL3:
		 	 memcpy(pstDevAttr, &DEV_ATTR_IMX327_DOL3_BASE, sizeof(VIN_DEV_ATTR_S));
			 break;
		 case IMX327_15FPS_3609P_DOL3_THREE_LINEAR:
		 	 memcpy(pstDevAttr, &DEV_ATTR_IMX327_DOL3_THREE_LINEAR_BASE, sizeof(VIN_DEV_ATTR_S));
			 break;
		 case IMX327_15FPS_3609P_DOL3_THREE_FIRST_LINEAR:
		 	 memcpy(pstDevAttr, &DEV_ATTR_IMX327_DOL3_THREE_LINEAR_FIRST_BASE,
					 sizeof(VIN_DEV_ATTR_S));
			 break;
		 case IMX327_15FPS_3609P_DOL3_LEF_SEF2:
		 	 memcpy(pstDevAttr, &DEV_ATTR_IMX327_DOL3_LEF_SEF1_BASE, sizeof(VIN_DEV_ATTR_S));
			 break;
		 case AR0233_30FPS_1080P_RAW12_954_PWL:
		 case AR0233_30FPS_1080P_RAW12_960_PWL:
		 case AR0233_30FPS_1080P_RAW12_954_LINEAR:
		 case AR0233_30FPS_1080P_RAW12_960_LINEAR:
		 case AR0233_30FPS_1080P_RAW12_960_PWL_2LANE:
		 case AR0233_30FPS_1080P_RAW12_954_PWL_2LANE:
			 memcpy(pstDevAttr, &DEV_ATTR_AR0233_1080P_BASE, sizeof(VIN_DEV_ATTR_S));
			 break;
		 case OS8A10_30FPS_3840P_RAW10_LINEAR:
			 memcpy(pstDevAttr, &DEV_ATTR_OS8A10_LINEAR_BASE, sizeof(VIN_DEV_ATTR_S));
			 break;
		 case OS8A10_30FPS_3840P_RAW10_DOL2:
			 memcpy(pstDevAttr, &DEV_ATTR_OS8A10_DOL2_BASE, sizeof(VIN_DEV_ATTR_S));
			 break;
		 case OV10635_30FPS_720p_954_YUV:
		 case OV10635_30FPS_720p_960_YUV:
			if(need_raw8) {
			  memcpy(pstDevAttr, &DEV_ATTR_OV10635_YUV_RAW8_BASE, sizeof(VIN_DEV_ATTR_S));
		 	} else {
			  memcpy(pstDevAttr, &DEV_ATTR_OV10635_YUV_BASE, sizeof(VIN_DEV_ATTR_S));
			}
			break;
		case OV10635_30FPS_720p_960_YUV_LINE_CONCATENATED:
			memcpy(pstDevAttr, &DEV_ATTR_OV10635_YUV_LINE_CONCATE_BASE, sizeof(VIN_DEV_ATTR_S));
			break;
		case SIF_TEST_PATTERN0_1080P:
			 memcpy(pstDevAttr, &DEV_ATTR_TEST_PATTERN_1080P_BASE, sizeof(VIN_DEV_ATTR_S));
			 break;
		case FEED_BACK_RAW12_1952P:
			 memcpy(pstDevAttr, &DEV_ATTR_FEED_BACK_1097P_BASE, sizeof(VIN_DEV_ATTR_S));
			 break;
	    case SIF_TEST_PATTERN_YUV_720P:
			 memcpy(pstDevAttr, &DEV_ATTR_TEST_PATTERN_YUV422_BASE, sizeof(VIN_DEV_ATTR_S));
			 break;
		case SIF_TEST_PATTERN_YUV_3840P:
			 memcpy(pstDevAttr, &DEV_ATTR_TEST_PATTERN_YUV422_8M_BASE, sizeof(VIN_DEV_ATTR_S));
			 break;
		case SIF_TEST_PATTERN_12M_RAW12:
			 memcpy(pstDevAttr, &DEV_ATTR_TEST_PATTERN_12M_BASE, sizeof(VIN_DEV_ATTR_S));
			 break;
		case SIF_TEST_PATTERN_8M_RAW10:
			 memcpy(pstDevAttr, &DEV_ATTR_TEST_PATTERN_8M_BASE, sizeof(VIN_DEV_ATTR_S));
			 break;
		default:
			printf("not surpport sensor type\n");
			break;
    }
	printf("SAMPLE_VIN_GetDevAttrBySns success\n");
    return 0;
}

int SAMPLE_VIN_GetDevAttrExBySns(MIPI_SNS_TYPE_E enSnsType, VIN_DEV_ATTR_EX_S *pstDevAttrEx)
{
    switch (enSnsType)
    {
		 case IMX327_30FPS_1952P_RAW12_LINEAR:
			 memcpy(pstDevAttrEx, &DEV_ATTR_IMX327_MD_BASE, sizeof(VIN_DEV_ATTR_EX_S));
			 break;
		 case OV10635_30FPS_720p_960_YUV:
		 case OV10635_30FPS_720p_954_YUV:
			 memcpy(pstDevAttrEx, &DEV_ATTR_OV10635_MD_BASE, sizeof(VIN_DEV_ATTR_EX_S));
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
		 case IMX327_30FPS_2228P_DOL2_TWO_LINEAR:
		 case IMX327_30FPS_2228P_DOL2_TWO_FIRST_LINEAR:
		 case IMX327_15FPS_3609P_DOL3_THREE_LINEAR:
		 case IMX327_15FPS_3609P_DOL3_THREE_FIRST_LINEAR:
		 case FEED_BACK_RAW12_1952P:
		 case IMX327_15FPS_3609P_DOL3_LEF_SEF2:
			 memcpy(pstPipeAttr, &PIPE_ATTR_IMX327_LINEAR_BASE, sizeof(VIN_PIPE_ATTR_S));
			 break;
		 case IMX327_30FPS_2228P_RAW12_DOL2:
			 memcpy(pstPipeAttr, &PIPE_ATTR_IMX327_DOL2_BASE, sizeof(VIN_PIPE_ATTR_S));
			 break;
		 case IMX327_15FPS_3609P_RAW12_DOL3:
		 	 memcpy(pstPipeAttr, &PIPE_ATTR_IMX327_DOL3_BASE, sizeof(VIN_PIPE_ATTR_S));
			 break;
		 case AR0233_30FPS_1080P_RAW12_954_LINEAR:
		 case AR0233_30FPS_1080P_RAW12_960_LINEAR:
		 	 memcpy(pstPipeAttr, &PIPE_ATTR_AR0233_1080P_LINEAR_BASE, sizeof(VIN_PIPE_ATTR_S));
			 break;
		 case AR0233_30FPS_1080P_RAW12_954_PWL:
		 case AR0233_30FPS_1080P_RAW12_960_PWL:
		 case AR0233_30FPS_1080P_RAW12_960_PWL_2LANE:
		 case AR0233_30FPS_1080P_RAW12_954_PWL_2LANE:
			 memcpy(pstPipeAttr, &PIPE_ATTR_AR0233_1080P_PWL_BASE, sizeof(VIN_PIPE_ATTR_S));
			 break;
		 case OS8A10_30FPS_3840P_RAW10_LINEAR:
		 case SIF_TEST_PATTERN_YUV_3840P:
		 case SIF_TEST_PATTERN_8M_RAW10:
			 memcpy(pstPipeAttr, &PIPE_ATTR_OS8A10_LINEAR_BASE, sizeof(VIN_PIPE_ATTR_S));
			 break;
		 case OS8A10_30FPS_3840P_RAW10_DOL2:
			 memcpy(pstPipeAttr, &PIPE_ATTR_OS8A10_DOL2_BASE, sizeof(VIN_PIPE_ATTR_S));
			 break;
		 case OV10635_30FPS_720p_954_YUV:
		 case OV10635_30FPS_720p_960_YUV:
		 case SIF_TEST_PATTERN_YUV_720P:
		 case OV10635_30FPS_720p_960_YUV_LINE_CONCATENATED:
			 memcpy(pstPipeAttr, &PIPE_ATTR_OV10635_YUV_BASE, sizeof(VIN_PIPE_ATTR_S));
			 break;
		 case SIF_TEST_PATTERN0_1080P:
			 memcpy(pstPipeAttr, &PIPE_ATTR_TEST_PATTERN_1080P_BASE, sizeof(VIN_PIPE_ATTR_S));
			 break;
		case SIF_TEST_PATTERN_12M_RAW12:
			 memcpy(pstPipeAttr, &PIPE_ATTR_TEST_PATTERN_12M_BASE, sizeof(VIN_PIPE_ATTR_S));
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
		 case IMX327_30FPS_2228P_RAW12_DOL2:
		 case IMX327_30FPS_2228P_DOL2_TWO_LINEAR:
		 case IMX327_30FPS_2228P_DOL2_TWO_FIRST_LINEAR:
		 case IMX327_15FPS_3609P_DOL3_THREE_LINEAR:
		 case IMX327_15FPS_3609P_DOL3_THREE_FIRST_LINEAR:
		 case AR0233_30FPS_1080P_RAW12_954_PWL:
		 case AR0233_30FPS_1080P_RAW12_960_PWL:
		 case AR0233_30FPS_1080P_RAW12_954_LINEAR:
		 case AR0233_30FPS_1080P_RAW12_960_LINEAR:
		 case SIF_TEST_PATTERN0_1080P:
		 case FEED_BACK_RAW12_1952P:
		 case IMX327_15FPS_3609P_RAW12_DOL3:
		 case IMX327_15FPS_3609P_DOL3_LEF_SEF2:
		 case AR0233_30FPS_1080P_RAW12_960_PWL_2LANE:
		 case AR0233_30FPS_1080P_RAW12_954_PWL_2LANE:
			 memcpy(pstDisAttr, &DIS_ATTR_BASE, sizeof(VIN_DIS_ATTR_S));
			 break;
		 case OS8A10_30FPS_3840P_RAW10_LINEAR:
		 case OS8A10_30FPS_3840P_RAW10_DOL2:
		 case SIF_TEST_PATTERN_8M_RAW10:
			 memcpy(pstDisAttr, &DIS_ATTR_OS8A10_BASE, sizeof(VIN_DIS_ATTR_S));
			 break;
		 case OV10635_30FPS_720p_954_YUV:
		 case OV10635_30FPS_720p_960_YUV:
		 case SIF_TEST_PATTERN_YUV_720P:
		 case OV10635_30FPS_720p_960_YUV_LINE_CONCATENATED:
			 memcpy(pstDisAttr, &DIS_ATTR_OV10635_BASE, sizeof(VIN_DIS_ATTR_S));
			 break;
		case SIF_TEST_PATTERN_12M_RAW12:
			 memcpy(pstDisAttr, &DIS_ATTR_12M_BASE, sizeof(VIN_DIS_ATTR_S));
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
		 case IMX327_30FPS_2228P_RAW12_DOL2:
		 case IMX327_30FPS_2228P_DOL2_TWO_LINEAR:
		 case IMX327_30FPS_2228P_DOL2_TWO_FIRST_LINEAR:
		 case IMX327_15FPS_3609P_DOL3_THREE_LINEAR:
		 case IMX327_15FPS_3609P_DOL3_THREE_FIRST_LINEAR:
		 case AR0233_30FPS_1080P_RAW12_954_PWL:
		 case AR0233_30FPS_1080P_RAW12_960_PWL:
		 case AR0233_30FPS_1080P_RAW12_954_LINEAR:
		 case AR0233_30FPS_1080P_RAW12_960_LINEAR:
		 case SIF_TEST_PATTERN0_1080P:
		 case FEED_BACK_RAW12_1952P:
		 case IMX327_15FPS_3609P_DOL3_LEF_SEF2:
		 case AR0233_30FPS_1080P_RAW12_960_PWL_2LANE:
		 case AR0233_30FPS_1080P_RAW12_954_PWL_2LANE:
			 memcpy(pstLdcAttr, &LDC_ATTR_BASE, sizeof(VIN_LDC_ATTR_S));
			 break;
		 case IMX327_15FPS_3609P_RAW12_DOL3:
		 	 memcpy(pstLdcAttr, &LDC_DOL3_ATTR_BASE, sizeof(VIN_LDC_ATTR_S));
			 break;
		 case OS8A10_30FPS_3840P_RAW10_DOL2:
			 memcpy(pstLdcAttr, &LDC_ATTR_OS8A10_DOL2_BASE, sizeof(VIN_LDC_ATTR_S));
			 break;
		 case OS8A10_30FPS_3840P_RAW10_LINEAR:
		 case SIF_TEST_PATTERN_8M_RAW10:
		 	 memcpy(pstLdcAttr, &LDC_ATTR_OS8A10_LINEAR_BASE, sizeof(VIN_LDC_ATTR_S));
			 break;
		 case OV10635_30FPS_720p_954_YUV:
		 case OV10635_30FPS_720p_960_YUV:
		 case SIF_TEST_PATTERN_YUV_720P:
		 case OV10635_30FPS_720p_960_YUV_LINE_CONCATENATED:
			 memcpy(pstLdcAttr, &LDC_ATTR_OV10635_BASE, sizeof(VIN_LDC_ATTR_S));
			 break;
	     case SIF_TEST_PATTERN_12M_RAW12:
		 	 memcpy(pstLdcAttr, &LDC_ATTR_12M_BASE, sizeof(VIN_LDC_ATTR_S));
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
	while (p && *p && i < MAX_SENSOR_NUM) {
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
		printf("i %d serdes_index[i] %d===========\n", i, serdes_index[i]);
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
	while (p && *p && i < MAX_SENSOR_NUM) {
		d = strchr(p, ',');
		if (d)
			*d = '\0';
		serdes_port[i] = atoi(p);
		printf("i %d serdes_index[i] %d===========\n", i, serdes_port[i]);
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
	while (p && *p && i < MAX_SENSOR_NUM) {
		d = strchr(p, ',');
		if (d)
			*d = '\0';
		vin_vps_mode[i] = atoi(p);
		printf("i %d serdes_index[i] %d===========\n", i, vin_vps_mode[i]);
		i++;
		p = (d) ? (d + 1) : NULL;
	}
	return;
}

void parse_extra_mode_func(char *optarg)
{
	char *p = optarg, *d;
	int i = 0;
	printf("optarg %s\n", optarg);
	while (p && *p && i < MAX_SENSOR_NUM) {
		d = strchr(p, ',');
		if (d)
			*d = '\0';
		extra_mode[i] = atoi(p);
		printf("i %d extra_mode[i] %d===========\n", i, extra_mode[i]);
		i++;
		p = (d) ? (d + 1) : NULL;
	}
	return;
}
void parse_virtual_func(char *optarg)
{
	char *p = optarg, *d;
	int i = 0;
	printf("optarg %s\n", optarg);
	while (p && *p && i < MAX_SENSOR_NUM) {
		d = strchr(p, ',');
		if (d)
			*d = '\0';
		virtual[i] = atoi(p);
		printf("i %d virtual[i] %d===========\n", i, virtual[i]);
		i++;
		p = (d) ? (d + 1) : NULL;
	}
	return;
}

int ion_open(void);
int ion_alloc_phy(int size, int *fd, char **vaddr, uint64_t * paddr);
int dumpToFile2plane(char *filename, char *srcBuf, char *srcBuf1,
		unsigned int size, unsigned int size1);
int dumpToFile(char *filename, char *srcBuf, unsigned int size);
int dumpToFile2planeStride(char *filename, char *srcBuf, char *srcBuf1,
		unsigned int size, unsigned int size1, int width, int height, int stride);

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
	puts("  -s --sensor_id       sensor_id\n"
		 "  -i --mipi_idx        mipi_idx\n"
	     "  -p --group_id        group_id\n"
	     "  -c --real_camera     real camera enable\n"
	     "  -r --run_time         time measured in seconds the program runs\n"
		 "  -d --dump_raw		  dump raw img\n"
		 "  -y --dump_yuv	      dump yuvimg\n"
	     "  -t --data_type        data type\n"
	     "  -m --multi_thread	 multi thread get\n"
	     "  -k --need_clk        need_clk from som\n"
	     "  -b --bus             bus number\n"
   	     "  -o --port            port\n"
		 "  -v --vc_num			 dol2 use vc_num\n"
		 "  -f --testpattern_fps set testpattern fps\n"
		 "  -a --need_raw8        if_use_raw8\n"
		 "  -K --need_devclk     need_devclk\n"
		 "  -E --devclk          devclk_value\n"
		 "  -V --vpuclk           vpuclk_calue\n"
   	     "  -S --serdes_index    serdes_index\n"
   	     "  -O --sedres_port     serdes_port\n"
		"-M --vin_vps_mode\n"
		"-X --need_grp_rotate\n"
		"-Y --need_chn_rotate\n"
		"-I --need_ipu\n"
		"-P --need_pym\n"
		"-D --vps_dump\n"
		"-N --need_md\n"
		"-F --need_chnfd\n"
		"-W --need_dis\n"
		"-R --extra_mode\n"
		"-z --vps_frame_rate\n"
		"-u --virtual_channel\n");
	exit(1);
}

void parse_opts(int argc, char *argv[])
{
	while (1) {
		static const char short_options[] =
		    "s:i:p:c:r:d:y:t:m:k:b:o:v:f:a:K:E:V:S:O:M:X:Y:I:P:D:N:F:W:R:z:h:u:T:";
		static const struct option long_options[] = {
			{"sensorId", 1, 0, 's'},
			{"mipiIdx",  1, 0, 'i'},
			{"groupMask", 1, 0, 'p'},
			{"real_camera", 1, 0, 'c'},
			{"run_time", 1, 0, 'r'},
			{"raw_dump", 1, 0, 'd'},
			{"yuv_dump", 1, 0, 'y'},
			{"data_type", 1, 0, 't'},
			{"multi_thread", 1, 0, 'm'},
			{"need_clk", 1, 0, 'k'},
			{"bus", 1, 0, 'b'},
			{"port", 1, 0, 'o'},
			{"vc_num", 1, 0, 'v'},
			{"testpattern_fps", 1, 0, 'f'},
			{"need_raw8", 1, 0, 'a'},
			{"need_devclk", 1, 0, 'K'},
			{"devclk", 1, 0, 'E'},
			{"vpuclk", 1, 0, 'V'},
			{"serdes_index", 1, 0, 'S'},
			{"serdes_port", 1, 0, 'O'},
			{"vin_vps_mode", 1, 0, 'M'},
			{"need_grp_rotate", 1, 0, 'X'},
			{"need_chn_rotate", 1, 0, 'Y'},
			{"need_ipu", 1, 0, 'I'},
			{"need_pym", 1, 0, 'P'},
			{"vps_dump", 1, 0, 'D'},
			{"need_md", 1, 0, 'N'},
			{"need_chnfd", 1, 0, 'F'},
			{"need_dis", 1, 0, 'W'},
			{"extra_mode", 1, 0, 'R'},
			{"vps_frame_rate", 1, 0, 'z'},
			{"mirror_ctrl", 1, 0, 'h'},
			{"virtual_channel", 1, 0, 'u'},
			{"vot_enable", 1, 0, 'V'},
			{NULL, 0, 0, 0},
		};

		int cmd_ret;

		cmd_ret =
		    getopt_long(argc, argv, short_options, long_options, NULL);

		if (cmd_ret == -1)
			break;
		printf("cmd_ret %d optarg %s\n", cmd_ret, optarg);
	switch (cmd_ret) {
		case 's':
			parse_sensorid_func(optarg);
			break;
		case 'i':
			parse_mipiIdx_func(optarg);
			break;
		case 'p':
			groupMask = atoi(optarg);;
			printf("groupMask = %d\n", groupMask);
			break;
		case 'c':
			need_cam = atoi(optarg);
			printf("need_cam = %d\n", need_cam);
			break;
		case 'r':
			run_time = atoi(optarg);
			printf("run_time = %ld\n", run_time);
			break;
		case 'm':
			need_m_thread = atoi(optarg);
			printf("need_m_thread = %d\n", need_m_thread);
			break;
		case 't':
			data_type = atoi(optarg);
			printf("data_type = %d\n", data_type);
			break;
		case 'd':
			raw_dump = atoi(optarg);
			printf("raw_dump = %d\n", raw_dump);
			break;
		case 'y':
			yuv_dump = atoi(optarg);
			printf("yuv_dump = %d\n", yuv_dump);
			break;
		case 'k':
			need_clk = atoi(optarg);
			printf("need_clk = %d\n", need_clk);
			break;
		case 'v':
			vc_num = atoi(optarg);
			printf("vc_num = %d\n", vc_num);
			break;
		case 'a':
			need_raw8 = atoi(optarg);
			printf("need_raw8 = %d\n", need_raw8);
			break;
		case 'b':
			parse_bus_func(optarg);
			break;
		case 'o':
			parse_port_func(optarg);
			break;
		case 'S':
			parse_serdes_index_func(optarg);
			break;
		case 'O':
			parse_serdes_port_func(optarg);
			break;
		case 'M':
			parse_vin_vps_mode_func(optarg);
			break;
	   case 'R':
			parse_extra_mode_func(optarg);
			break;
	  case 'u':
			parse_virtual_func(optarg);
			break;
		case 'X':
			need_grp_rotate = atoi(optarg);
			printf("need_grp_rotate = %d\n", need_grp_rotate);
			break;
		case 'Y':
			need_chn_rotate = atoi(optarg);
			printf("need_chn_rotate = %d\n", need_chn_rotate);
			break;
		case 'I':
			need_ipu = atoi(optarg);
			printf("need_ipu = %d\n", need_ipu);
			break;
		case 'P':
			need_pym = atoi(optarg);
			printf("need_pym = %d\n", need_pym);
			break;
		case 'D':
			vps_dump = atoi(optarg);
			printf("vps_dump = %d\n", vps_dump);
			break;
		case 'N':
			need_md = atoi(optarg);
			printf("need_md = %d\n", need_md);
			break;
		case 'F':
			need_chnfd = atoi(optarg);
			printf("need_chnfd = %d\n", need_chnfd);
			break;
		case 'W':
			need_dis = atoi(optarg);
			printf("need_dis = %d\n", need_dis);
			break;
		case 'f':
			testpattern_fps = atoi(optarg);
			printf("testpattern_fps = %d\n", testpattern_fps);
			break;
		case 'K':
			need_devclk = atoi(optarg);
			printf("need_devclk = %d\n", need_devclk);
			break;
		case 'E':
			devclk = atoi(optarg);
			printf("devclk = %d\n", devclk);
			break;
		case 'V':
			vpuclk = atoi(optarg);
			printf("vpuclk = %d\n", vpuclk);
			break;
		case 'z':
			vps_frame_rate = atoi(optarg);
			printf("vps_frame_rate = %d\n", vps_frame_rate);
			break;
		case 'h':
				mirror_ctrl = atoi(optarg);
				printf("mirror_ctrl = %d\n", mirror_ctrl);
				break;
		case 'T':
				vot_enable = atoi(optarg);
				printf("vot_enable = %d\n", vot_enable);
				break;
		default:
			print_usage(argv[0]);
			break;
		}
	}
}

static void normal_buf_info_print(hb_vio_buffer_t * buf)
{
		printf("sif normal pipe_id (%d) frame_id(%d) time_stamp %d ms\n",
			buf->img_info.pipeline_id, buf->img_info.frame_id, buf->img_info.time_stamp/HW_TIMER);
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
	if(need_md) {
		ret = HB_VIN_EnableDevMd(pipeId);
		if(ret < 0) {
			printf("HB_VIN_EnableDevMd error!\n");
			return ret;
		}
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
int hb_vin_vps_init(int pipeId, uint32_t sensorId, uint32_t mipiIdx, uint32_t deseri_port, uint32_t vin_vps_mode, uint32_t virtual)
{
	int ret = 0;
	VIN_DEV_ATTR_S  devinfo;
	VIN_PIPE_ATTR_S pipeinfo;
	VIN_DIS_ATTR_S  disinfo;
	VIN_LDC_ATTR_S  ldcinfo;
	VIN_DEV_ATTR_EX_S devexinfo;
	VIN_DIS_CALLBACK_S pstDISCallback;
	pstDISCallback.VIN_DIS_DATA_CB = dis_crop_set;

	VPS_GRP_ATTR_S grp_attr;
	VPS_CHN_ATTR_S chn_attr;
	VPS_PYM_CHN_ATTR_S pym_chn_attr;
	VPS_CROP_INFO_S chn_crop_info;

	memset(&devinfo, 0, sizeof(VIN_DEV_ATTR_S));
	memset(&pipeinfo, 0, sizeof(VIN_PIPE_ATTR_S));
	memset(&disinfo, 0, sizeof(VIN_DIS_ATTR_S));
	memset(&ldcinfo, 0, sizeof(VIN_LDC_ATTR_S));
	memset(&devexinfo, 0, sizeof(VIN_DEV_ATTR_EX_S));
	SAMPLE_VIN_GetDevAttrBySns(sensorId,  &devinfo);
	SAMPLE_VIN_GetPipeAttrBySns(sensorId, &pipeinfo);
	SAMPLE_VIN_GetDisAttrBySns(sensorId,  &disinfo);
	SAMPLE_VIN_GetLdcAttrBySns(sensorId,  &ldcinfo);
	SAMPLE_VIN_GetDevAttrExBySns(sensorId, &devexinfo);
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
	if(need_devclk) {
		ret = HB_VIN_SetDevMclk(pipeId, devclk, vpuclk);   // isp init
		if(ret < 0) {
			printf("HB_MIPI_InitSensor error!\n");
			return ret;
		}
	}
	ret = HB_VIN_SetMipiBindDev(pipeId, mipiIdx);
	if(ret < 0) {
		printf("HB_VIN_SetMipiBindDev error!\n");
		return ret;
	}
	if(sensorId == OV10635_30FPS_720p_960_YUV_LINE_CONCATENATED) {
		deseri_port = 0;
	}
	if(sensorId == AR0233_30FPS_1080P_RAW12_960_PWL_2LANE) {
		ret = HB_VIN_SetDevVCNumber(pipeId, virtual);
		if(ret < 0) {
			printf("HB_VIN_SetDevVCNumber error!\n");
			return ret;
		}
	} else {
		ret = HB_VIN_SetDevVCNumber(pipeId, deseri_port);
		if(ret < 0) {
			printf("HB_VIN_SetDevVCNumber error!\n");
			return ret;
		}
	}
	if(sensorId == IMX327_30FPS_2228P_RAW12_DOL2 ||
		sensorId == OS8A10_30FPS_3840P_RAW10_DOL2 ||
		sensorId == IMX327_15FPS_3609P_RAW12_DOL3 ||
		sensorId == IMX327_15FPS_3609P_DOL3_LEF_SEF2 ||
		sensorId == IMX327_15FPS_3609P_DOL3_THREE_FIRST_LINEAR ||
		sensorId == IMX327_30FPS_2228P_DOL2_TWO_FIRST_LINEAR) {
		ret = HB_VIN_AddDevVCNumber(pipeId, 1);
		if(ret < 0) {
			printf("HB_VIN_AddDevVCNumber error!\n");
			return ret;
		}
	}
	if(sensorId == IMX327_15FPS_3609P_RAW12_DOL3 ||
		sensorId == IMX327_15FPS_3609P_DOL3_THREE_FIRST_LINEAR) {
		ret = HB_VIN_AddDevVCNumber(pipeId, 2);
		if(ret < 0) {
			printf("HB_VIN_AddDevVCNumber error!\n");
			return ret;
		}
	}
	ret = HB_VIN_SetDevAttr(pipeId, &devinfo);     // sif init
	if(ret < 0) {
		printf("HB_MIPI_InitSensor error!\n");
		return ret;
	}
	if(need_md) {
		ret = HB_VIN_SetDevAttrEx(pipeId, &devexinfo);
		if(ret < 0) {
			printf("HB_VIN_SetDevAttrEx error!\n");
			return ret;
		}
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
	if(need_dis) {
		HB_VIN_RegisterDisCallback(pipeId, &pstDISCallback);
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

	/*
	 * todo delete
	 * bind other group
	 */
       /* grp_attr.maxW = 1920;*/
	/*grp_attr.maxH = 1080;*/
	/*grp_attr.frameDepth = 2;*/
	/*HB_VPS_CreateGrp(1, &grp_attr);*/
	/*chn_attr.enScale = 1;*/
	/*chn_attr.width = 1280;*/
	/*chn_attr.height = 720;*/
	/*chn_attr.frameDepth = 6;*/
	/*HB_VPS_SetChnAttr(1, 2, &chn_attr);*/
	/*HB_VPS_EnableChn(1, 2);*/
	/*HB_VPS_StartGrp(1);*/



	grp_attr.maxW = pipeinfo.stSize.width;
	grp_attr.maxH = pipeinfo.stSize.height;
	grp_attr.frameDepth = 3;
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
	if (vps_frame_rate) {
		chn_attr.frameRate.srcFrameRate = 30;
		chn_attr.frameRate.dstFrameRate = 15;
	}
	chn_attr.enScale = 1;
	chn_attr.width = 1280;
	chn_attr.height = 720;
	chn_attr.frameDepth = 4;

	chn_crop_info.en = 1;
	chn_crop_info.cropRect.x = 0;
	chn_crop_info.cropRect.y = 0;
	chn_crop_info.cropRect.width = 1920;
	chn_crop_info.cropRect.height = 1080;

	if (BIT2CHN(need_ipu, 0)) {
		ret = HB_VPS_SetChnAttr(pipeId, 0, &chn_attr);
		if (ret) {
			printf("HB_VPS_SetChnAttr error!!!\n");
		} else {
			printf("set chn Attr ok: GrpId = %d, chn_id = %d\n",
									pipeId, 0);
		}
		HB_VPS_EnableChn(pipeId, 0);
		/*ret = HB_VPS_SetChnCrop(pipeId, 0, &chn_crop_info);*/
	}

	chn_attr.width = 1920;//1920;//
	chn_attr.height = 1080;//1080;//

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

	chn_attr.width = 1280;
	chn_attr.height = 720;
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

	chn_crop_info.en = 1;
	chn_crop_info.cropRect.x = 1920;
	chn_crop_info.cropRect.y = 0;
	chn_crop_info.cropRect.width = 1920;
	chn_crop_info.cropRect.height = 1080;
	/*chn_crop_info.cropRect.x = 0;*/
	/*chn_crop_info.cropRect.y = 0;*/
	/*chn_crop_info.cropRect.width = 3840;*/
	/*chn_crop_info.cropRect.height = 2160;*/

	chn_attr.width = 1920;
	chn_attr.height = 1080;
	if (BIT2CHN(need_ipu, 2)) {
		ret = HB_VPS_SetChnAttr(pipeId, 2, &chn_attr);
		if (ret) {
			printf("HB_VPS_SetChnAttr error!!!\n");
		} else {
			printf("set chn Attr ok: pipeId = %d, chn_id = %d\n",
									pipeId, 2);
		}
		HB_VPS_EnableChn(pipeId, 2);
		/*ret = HB_VPS_SetChnCrop(pipeId, 2, &chn_crop_info);*/
	}

       /* FRAME_RATE_CTRL_S frame_rate;*/
	/*frame_rate.srcFrameRate = 30;*/
	/*frame_rate.dstFrameRate = 15;*/
	/*ret = HB_VPS_SetChnFrameRate(pipeId, 2, &frame_rate);*/
	/*if (ret)*/
		/*printf("dynamic set framerate fail\n");*/

	chn_attr.width = 1280;
	chn_attr.height = 720;
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

	/*
	 * todo delete
	 */
	/*src_mod.enModId = HB_ID_VPS;*/
	/*src_mod.s32DevId = 0;*/
	/*src_mod.s32ChnId = 2;*/
	/*dst_mod.enModId = HB_ID_VPS;*/
	/*dst_mod.s32DevId = 1;*/
	/*dst_mod.s32ChnId = 0;*/
	/*ret = HB_SYS_Bind(&src_mod, &dst_mod);*/
	/*if (ret != 0)*/
		/*printf("HB_SYS_Bind failed\n");*/


if(need_chnfd) {
	g_vin_fd[pipeId] = HB_VIN_GetChnFd(pipeId, 0);
	if(g_vin_fd[pipeId] < 0) {
		printf("HB_VIN_GetChnFd error!!!\n");
	}
}
	return ret;
pipe_err:
	HB_VIN_DestroyDev(pipeId);  // sif deinit
chn_err:
	HB_VIN_DestroyPipe(pipeId);  // isp && dwe deinit
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

void hb_vin_vps_deinit(int pipeId)
{
	HB_VIN_DestroyDev(pipeId);  // sif deinit && destroy
	HB_VIN_DestroyChn(pipeId, 1);  // dwe deinit
	if(need_chnfd) {
		HB_VIN_CloseFd();
	}
	HB_VIN_DestroyPipe(pipeId);  // isp deinit && destroy
	HB_VPS_DestroyGrp(pipeId);
}
#if 0
void mipi_sensor_printf(MIPI_SENSOR_INFO_S *snsinfo)
{
	printf("sensor_info.sensor_name %s\n", snsinfo->sensorInfo.sensor_name);
	printf("sensor_info.dev_port %d\n", snsinfo->sensorInfo.dev_port);
	printf("sensor_info.bus_num %d\n", snsinfo->sensorInfo.bus_num);
	printf("sensor_info.fps %d\n", snsinfo->sensorInfo.fps);
	printf("sensor_info.resolution %d\n", snsinfo->sensorInfo.resolution);
	printf("sensor_info.sensor_addr 0x%x\n", snsinfo->sensorInfo.sensor_addr);
	printf("sensor_info.entry_num %d\n", snsinfo->sensorInfo.entry_index);
	printf("sensor_info.sensor_mode %d\n", snsinfo->sensorInfo.sensor_mode);
	printf("sensor_info.reg_width %d\n", snsinfo->sensorInfo.reg_width);
	printf("deseEnable %d\n", snsinfo->deseEnable);
	if(snsinfo->deseEnable) {
		printf("sensor_info.deserial_index %d\n",
				snsinfo->sensorInfo.deserial_index);
		printf("sensor_info.deserial_port %d\n",
				snsinfo->sensorInfo.deserial_port);
		printf("sensor_info.serial_addr 0x%x\n",
			snsinfo->sensorInfo.serial_addr);
	}

	return;
}
#endif
int hb_sensor_init(int devId, int sensorId, int bus, int port, int mipiIdx, int sedres_index, int sedres_port, int extra_mode)
{
	int ret = 0;
	MIPI_SENSOR_INFO_S snsinfo;

	memset(&snsinfo, 0, sizeof(MIPI_SENSOR_INFO_S));
	SAMPLE_MIPI_GetSnsAttrBySns(sensorId, &snsinfo);
	if(sensorId == IMX327_30FPS_2228P_RAW12_DOL2 ||
		sensorId == IMX327_30FPS_1952P_RAW12_LINEAR ||
		sensorId == IMX327_30FPS_2228P_DOL2_TWO_LINEAR ||
		sensorId == IMX327_15FPS_3609P_RAW12_DOL3) {
		if (bus == 0) {
		  system("echo 1 >/sys/class/vps/mipi_host0/param/stop_check_instart");
		} else if (bus == 5) {
		  system("echo 1 >/sys/class/vps/mipi_host1/param/stop_check_instart");
		}
	}

	HB_MIPI_SetExtraMode(&snsinfo, extra_mode);

    HB_MIPI_SetBus(&snsinfo, bus);
    HB_MIPI_SetPort(&snsinfo, port);
    HB_MIPI_SensorBindSerdes(&snsinfo, sedres_index, sedres_port);
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
	MIPI_ATTR_S mipi_attr;

	memset(&mipi_attr, 0, sizeof(MIPI_ATTR_S));
	SAMPLE_MIPI_GetMipiAttrBySns(sensorId, &mipi_attr);

	ret = HB_MIPI_SetMipiAttr(mipiIdx, &mipi_attr);
	if(ret < 0) {
		printf("HB_MIPI_SetDevAttr error!\n");
		return ret;
	}
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
void intHandler(int dummy)
{
	int i;
	signal(SIGINT, SIG_IGN);

    g_exit = 1;

  if(need_cam == 1) {
	 for (i = 0; i < MAX_ID_NUM; i++) {
		if (BIT(i) & groupMask) {
			hb_sensor_stop(i);
			hb_mipi_stop(mipiIdx[i]);
		  }
	  }
	}
	for (i = 0; i < MAX_ID_NUM; i++) {
		if (BIT(i) & groupMask) {
			hb_vin_vps_stop(i);
		}
	}
	if (need_cam == 1) {
	  for (i = 0; i < MAX_ID_NUM; i++) {
		if (BIT(i) & groupMask) {
			hb_mipi_deinit(mipiIdx[i]);
			hb_sensor_deinit(i);
		 }
	   }
	}
	for (i = 0; i < MAX_ID_NUM; i++) {
		if (BIT(i) & groupMask) {
			hb_vin_vps_deinit(i);
		}
	}
	printf("Test hb_vio_deinit done\n");
	exit(1);
}

int md_func(work_info_t * info)
{
	int ret = 0;
	int pipeId = info->group_id;

	printf("HB_VIN_MotionDetect begin!!! ret %d \n", ret);
	ret =  HB_VIN_MotionDetect(pipeId);
	if (ret < 0) {
		printf("HB_VIN_MotionDetect error!!! ret %d \n", ret);
	} else {
		printf("md_func success!!! ret %d \n", ret);
		 HB_VIN_DisableDevMd(pipeId);
		printf("HB_VIN_DisableDevMd success!!! ret %d \n", ret);
	}
	return ret;
}

int sif_raw_dump_func(work_info_t * info)
{
	struct timeval time_now = { 0 };
	struct timeval time_next = { 0 };
	int size = -1;
	char file_name[100] = { 0 };
	dump_info_t dump_info = {0};
	int ret = 0, i = 0;
	VIN_FRAME_RATE_CTRL_S frame_rate;
	static uint32_t sif_frame_cnt = 0;
	hb_vio_buffer_t *sif_raw = NULL;
	int pipeId = info->group_id;

	sif_raw = (hb_vio_buffer_t *) malloc(sizeof(hb_vio_buffer_t));
	memset(sif_raw, 0, sizeof(hb_vio_buffer_t));

	ret = HB_VIN_GetDevFrame(pipeId, 0, sif_raw, 2000);
	if (ret < 0) {
		printf("HB_VIN_GetDevFrame error!!!\n");
	} else {
		sif_frame_cnt++;
		normal_buf_info_print(sif_raw);
		if(raw_dump) {
			size = sif_raw->img_addr.stride_size * sif_raw->img_addr.height;
			printf("stride_size(%u) w x h%u x %u  size %d\n",
				sif_raw->img_addr.stride_size,
				sif_raw->img_addr.width, sif_raw->img_addr.height, size);

			if (sif_raw->img_info.planeCount == 1) {
				dump_info.ctx_id = info->group_id;
				dump_info.raw.frame_id = sif_raw->img_info.frame_id;
				dump_info.raw.plane_count = sif_raw->img_info.planeCount;
				dump_info.raw.xres[0] = sif_raw->img_addr.width;
				dump_info.raw.yres[0] = sif_raw->img_addr.height;
				dump_info.raw.addr[0] = sif_raw->img_addr.addr[0];
				dump_info.raw.size[0] = size;

			printf("pipe(%d)dump normal raw frame id(%d),plane(%d)size(%d)\n",
				dump_info.ctx_id, dump_info.raw.frame_id,
				dump_info.raw.plane_count, size);
			} else if (sif_raw->img_info.planeCount == 2) {
				dump_info.ctx_id = info->group_id;
				dump_info.raw.frame_id = sif_raw->img_info.frame_id;
				dump_info.raw.plane_count = sif_raw->img_info.planeCount;
				for (int i = 0; i < sif_raw->img_info.planeCount; i ++) {
					dump_info.raw.xres[i] = sif_raw->img_addr.width;
					dump_info.raw.yres[i] = sif_raw->img_addr.height;
					dump_info.raw.addr[i] = sif_raw->img_addr.addr[i];
					dump_info.raw.size[i] = size;
				}
				if(sif_raw->img_info.img_format == 0) {
			printf("pipe(%d)dump dol2 raw frame id(%d),plane(%d)size(%d)\n",
				dump_info.ctx_id, dump_info.raw.frame_id,
				dump_info.raw.plane_count, size);
				}
			} else if (sif_raw->img_info.planeCount == 3) {
				dump_info.ctx_id = info->group_id;
				dump_info.raw.frame_id = sif_raw->img_info.frame_id;
				dump_info.raw.plane_count = sif_raw->img_info.planeCount;
				for (int i = 0; i < sif_raw->img_info.planeCount; i ++) {
					dump_info.raw.xres[i] = sif_raw->img_addr.width;
					dump_info.raw.yres[i] = sif_raw->img_addr.height;
					dump_info.raw.addr[i] = sif_raw->img_addr.addr[i];
					dump_info.raw.size[i] = size;
			}
			printf("pipe(%d)dump dol3 raw frame id(%d),plane(%d)size(%d)\n",
				dump_info.ctx_id, dump_info.raw.frame_id,
				dump_info.raw.plane_count, size);
			} else {
				printf("pipe(%d)raw buf planeCount wrong !!!\n", info->group_id);
			}

		 for (int i = 0; i < dump_info.raw.plane_count; i ++) {
		 	if(sif_raw->img_info.img_format == 0) {
			sprintf(file_name, "pipe%d_plane%d_%ux%u_frame_%03d.raw",
							dump_info.ctx_id,
							i,
							dump_info.raw.xres[i],
							dump_info.raw.yres[i],
							dump_info.raw.frame_id);

		   gettimeofday(&time_now, NULL);
		   dumpToFile(file_name,  dump_info.raw.addr[i], dump_info.raw.size[i]);
		   gettimeofday(&time_next, NULL);
		   int time_cost = time_cost_ms(&time_now, &time_next);
		   printf("dumpToFile raw cost time %d ms", time_cost);
		 }
		}
		if(sif_raw->img_info.img_format == 8) {
			sprintf(file_name, "pipe%d_%ux%u_frame_%03d.yuv",
							dump_info.ctx_id,
							dump_info.raw.xres[i],
							dump_info.raw.yres[i],
							dump_info.raw.frame_id);
			gettimeofday(&time_now, NULL);
			dumpToFile2plane(file_name, sif_raw->img_addr.addr[0],
							sif_raw->img_addr.addr[1], size, size/2);
			gettimeofday(&time_next, NULL);
			int time_cost = time_cost_ms(&time_now, &time_next);
			printf("dumpToFile yuv cost time %d ms", time_cost);
		}
	}
		if ((sif_frame_cnt % 10) == 0 && raw_dump == 3) {
			frame_rate.srcFrameRate = 30;
			frame_rate.dstFrameRate  = 30 - (sif_frame_cnt / 2);
			frame_rate.keepRaw = 0;
			ret = HB_VIN_SetDevFrameRate(info->group_id, &frame_rate);
			if (ret)
				printf("dynamic set pipe %d framerate fail\n", info->group_id);
			else
				printf("dynamic set pipe %d framerate %d/30\n", info->group_id,
						frame_rate.dstFrameRate);
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

int isp_yuv_dump_func(work_info_t * info)
{
	hb_vio_buffer_t *isp_yuv = NULL;
	struct timeval time_now = { 0 };
	struct timeval time_next = { 0 };
	int size = -1, ret = 0;
	char file_name[100] = { 0 };
	int time_ms = 0;
	struct timeval select_timeout = {0};
	int pipeId = info->group_id;

	select_timeout.tv_usec = 500 * 1000;
	isp_yuv = (hb_vio_buffer_t *) malloc(sizeof(hb_vio_buffer_t));
	memset(isp_yuv, 0, sizeof(hb_vio_buffer_t));

	gettimeofday(&time_now, NULL);
	time_ms = time_now.tv_sec * 1000 + time_now.tv_usec /1000;
	ret = HB_VIN_GetChnFrame(pipeId, 0, isp_yuv, 2000);
	if (ret < 0) {
		printf("HB_VIN_GetPipeFrame error!!!\n");
	} else {
		if(yuv_dump) {
			normal_buf_info_print(isp_yuv);
			size = isp_yuv->img_addr.stride_size * isp_yuv->img_addr.height;
			printf("yuv stride_size(%u) w x h%u x %u, size %d\n",
						isp_yuv->img_addr.stride_size,
						isp_yuv->img_addr.width, isp_yuv->img_addr.height, size);
			snprintf(file_name, sizeof(file_name),
						"./isp_pipeId%d_yuv_%d_index%d.yuv", pipeId, time_ms,
													isp_yuv->img_info.buf_index);
			gettimeofday(&time_now, NULL);
			dumpToFile2plane(file_name, isp_yuv->img_addr.addr[0],
							isp_yuv->img_addr.addr[1], size, size/2);
			gettimeofday(&time_next, NULL);
			int time_cost = time_cost_ms(&time_now, &time_next);
			printf("dumpToFile yuv cost time %d ms", time_cost);
		}
		ret = HB_VIN_ReleaseChnFrame(pipeId, 0, isp_yuv);
		if (ret < 0) {
			printf("HB_VIN_ReleaseChnFrame error!!!\n");
		}
	}
	free(isp_yuv);
	isp_yuv = NULL;

	return 0;
}
int isp_yuv_dump_getchnfd_func(work_info_t * info)
{
	hb_vio_buffer_t *isp_yuv = NULL;
	struct timeval time_now = { 0 };
	struct timeval time_next = { 0 };
	int size = -1, ret = 0;
	char file_name[100] = { 0 };
	int time_ms = 0;
	struct timeval select_timeout = {0};
	int pipeId = info->group_id;
	fd_set readfd;

	FD_ZERO(&readfd);

	if (g_vin_fd[pipeId] != 0) {
		FD_SET(g_vin_fd[pipeId], &readfd);
		printf("select_isp_dump_yuv_func g_vin_fd[pipeId] %d chn0 need listen\n", g_vin_fd[pipeId]);
	}
	select_timeout.tv_usec = 500 * 1000;
	isp_yuv = (hb_vio_buffer_t *) malloc(sizeof(hb_vio_buffer_t));
	memset(isp_yuv, 0, sizeof(hb_vio_buffer_t));

	ret = select(g_vin_fd[pipeId] + 1, &readfd, NULL, NULL, &select_timeout);
	if (ret == -1) {
			printf("select_isp_dump_yuv_func select error\n");
	} else if (ret) {
		printf("select_isp_dump_yuv_func select return\n");
		if (g_vin_fd[pipeId] != 0) {
			if (FD_ISSET(g_vin_fd[pipeId], &readfd)) {
					gettimeofday(&time_now, NULL);
					time_ms = time_now.tv_sec * 1000 + time_now.tv_usec /1000;
					ret = HB_VIN_GetChnFrame(pipeId, 0, isp_yuv, 2000);
					if (ret < 0) {
						printf("HB_VIN_GetPipeFrame error!!!\n");
					} else {
						if(yuv_dump) {
							normal_buf_info_print(isp_yuv);
							size = isp_yuv->img_addr.stride_size * isp_yuv->img_addr.height;
							printf("yuv stride_size(%u) w x h%u x %u, size %d\n",
										isp_yuv->img_addr.stride_size,
										isp_yuv->img_addr.width, isp_yuv->img_addr.height, size);
							snprintf(file_name, sizeof(file_name),
										"./isp_pipeId%d_yuv_%d_index%d.yuv", pipeId, time_ms,
																	isp_yuv->img_info.buf_index);
							gettimeofday(&time_now, NULL);
							dumpToFile2plane(file_name, isp_yuv->img_addr.addr[0],
											isp_yuv->img_addr.addr[1], size, size/2);
							gettimeofday(&time_next, NULL);
							int time_cost = time_cost_ms(&time_now, &time_next);
							printf("dumpToFile yuv cost time %d ms", time_cost);
						}
						ret = HB_VIN_ReleaseChnFrame(pipeId, 0, isp_yuv);
						if (ret < 0) {
							printf("HB_VIN_ReleaseChnFrame error!!!\n");
						}
				}
			}
		}
	}

	free(isp_yuv);
	isp_yuv = NULL;
}

int vps_feedback_get_dump_thread(struct feedback_arg_s *arg)
{
	int ret;
	pym_buffer_t out_pym_buf;
	int i;
	char file_name[64];
	int grp_id = arg->grp_id;

	printf("vps_feedback_get_dump_thread in\n");
	while (check_end()) {
		ret = HB_VPS_SendFrame(grp_id, arg->feedback_buf, 1000);
		if (ret) {
			printf("HB_VPS_SendFrame error!!!\n");
		} else {
			printf("HB_VPS_SendFrame ok\n");
		}
		ret = HB_VPS_GetChnFrame(grp_id, 0, &out_pym_buf, 2000);
		if (ret) {
			printf("HB_VPS_GetChnFrame error!!!\n");
		} else {
			printf("Get Frame ok grp=%d pym_chn=%d bufindex=%d\n",
				grp_id, 0, out_pym_buf.pym_img_info.buf_index);
			if (vps_dump) {
				for (i = 0; i < 6; i++) {
					snprintf(file_name, sizeof(file_name),
						"grp%d_pym_out_basic_layer_DS%d_%d_%d.yuv",
						grp_id, i * 4, out_pym_buf.pym[i].width,
						out_pym_buf.pym[i].height);
					dumpToFile2planeStride(file_name, out_pym_buf.pym[i].addr[0],
										out_pym_buf.pym[i].addr[1],
										out_pym_buf.pym[i].width *
										out_pym_buf.pym[i].height,
										out_pym_buf.pym[i].width *
										out_pym_buf.pym[i].height / 2,
										out_pym_buf.pym[i].width,
										out_pym_buf.pym[i].height,
										out_pym_buf.pym[i].stride_size);

					for (int j = 0; j < 3; j++) {
						snprintf(file_name, sizeof(file_name),
							"grp%d_pym_out_roi_layer_DS%d_%d_%d.yuv",
							grp_id, i * 4 + j + 1,
							out_pym_buf.pym_roi[i][j].width,
							out_pym_buf.pym_roi[i][j].height);
						if (out_pym_buf.pym_roi[i][j].width != 0)
							dumpToFile2planeStride(file_name,
							out_pym_buf.pym_roi[i][j].addr[0],
							out_pym_buf.pym_roi[i][j].addr[1],
							out_pym_buf.pym_roi[i][j].width *
							out_pym_buf.pym_roi[i][j].height,
							out_pym_buf.pym_roi[i][j].width *
							out_pym_buf.pym_roi[i][j].height / 2,
							out_pym_buf.pym_roi[i][j].width,
							out_pym_buf.pym_roi[i][j].height,
							out_pym_buf.pym_roi[i][j].stride_size);
					}

					snprintf(file_name, sizeof(file_name),
						"grp%d_pym_out_us_layer_US%d_%d_%d.yuv", grp_id,
						i, out_pym_buf.us[i].width, out_pym_buf.us[i].height);
					if (out_pym_buf.us[i].width != 0)
						dumpToFile2planeStride(file_name, out_pym_buf.us[i].addr[0],
										out_pym_buf.us[i].addr[1],
										out_pym_buf.us[i].width *
										out_pym_buf.us[i].height,
										out_pym_buf.us[i].width *
										out_pym_buf.us[i].height / 2,
										out_pym_buf.us[i].width,
										out_pym_buf.us[i].height,
										out_pym_buf.us[i].stride_size);
				}
			}
			ret = HB_VPS_ReleaseChnFrame(grp_id, 0, &out_pym_buf);
			if (ret) {
				printf("HB_VPS_ReleaseChnFrame error!!!\n");
			}
		}
	}
	printf("vps_feedback_get_dump_thread out\n");

	return 0;
}

int vps_get_dump_func(work_info_t * info)
{
	struct timeval time_now = { 0 };
	struct timeval time_next = { 0 };
	int size = -1, ret = 0;
	char file_name[100] = { 0 };
	int time_ms = 0;
	int grp_id = info->group_id;
	VPS_CHN_ATTR_S chn_attr;
	FRAME_RATE_CTRL_S frame_rate;
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
	float idx = 0.0;

	while (check_end()) {
		frame_cnt++;
		if (info->data_type != chns2chn(need_pym)) {
			/* coverity[overrun-buffer-val] */
			ret = HB_VPS_GetChnFrame(info->group_id, info->data_type, &out_buf, 2000);
			if (ret != 0) {
				printf("HB_VPS_GetChnFrame error!!!\n");
				usleep(200 * 1000);
			} else {
				start = time(NULL);
				idx += 1.0;
				if (start > last + 5) {
					printf("\n\ngroup_id %d ipu fps %2f\n", info->group_id, idx / 5.0);
					last = start;
					idx = 0.0;
				}
				/*printf("Get Frame ok grpid=%d chn_id=%d bufindex=%d\n",*/
					/*info->group_id, info->data_type, out_buf.img_info.buf_index);*/
				/*printf("ipu out: info->group_id = %d height = %d frameid %d time_stamp %d ms\n",*/
						/*info->group_id, out_buf.img_addr.height, out_buf.img_info.frame_id,*/
						/*out_buf.img_info.time_stamp/HW_TIMER);*/
				if (vps_dump) {
					size_y = out_buf.img_addr.stride_size *
								out_buf.img_addr.height;
					size_uv = size_y / 2;
					snprintf(file_name, sizeof(file_name),
						"grp%d_chn%d_frmid%03d_%d_%d.yuv", grp_id, info->data_type,
						out_buf.img_info.frame_id,out_buf.img_addr.width,
						out_buf.img_addr.height);

					gettimeofday(&time_now, NULL);
					dumpToFile2planeStride(file_name, out_buf.img_addr.addr[0],
										out_buf.img_addr.addr[1],
										out_buf.img_addr.width *
										out_buf.img_addr.height,
										out_buf.img_addr.width *
										out_buf.img_addr.height / 2,
										out_buf.img_addr.width,
										out_buf.img_addr.height,
										out_buf.img_addr.stride_size);
					gettimeofday(&time_next, NULL);
					int time_cost = time_cost_ms(&time_now, &time_next);
					printf("dumpToFile cost time %d ms", time_cost);
				}

				ret = HB_VPS_ReleaseChnFrame(info->group_id, info->data_type, &out_buf);
				if (ret) {
					printf("HB_VPS_ReleaseChnFrame error!!!\n");
				}
			}
			/*
			 *dynamic scale
			 */
			if ((frame_cnt % 10) == 0 && vps_dump == 2) {
				chn_attr.width = 1180;
				chn_attr.height = 620;
				ret = HB_VPS_SetChnAttr(info->group_id, info->data_type, &chn_attr);
				if (ret)
					printf("dynamic set chn%d attr fail\n", info->data_type);
			}
			/*
			 *dynamic frame_rate
			 */
			if ((frame_cnt % 10) == 0 && vps_dump == 3) {
				frame_rate.srcFrameRate = 30;
				frame_rate.dstFrameRate  = 30 - (frame_cnt / 2);
				ret = HB_VPS_SetChnFrameRate(info->group_id,
						info->data_type, &frame_rate);
				if (ret)
					printf("dynamic set chn%d framerate fail\n", info->data_type);
				else
					printf("dynamic set framerate %d/30\n", frame_rate.dstFrameRate);
			}


		} else if (info->data_type == chns2chn(need_pym)) {
			ret = HB_VPS_GetChnFrame(info->group_id, info->data_type, &out_pym_buf, 2000);
			if (ret) {
				printf("HB_VPS_GetChnFrame error!!!\n");
				usleep(200 * 1000);
			} else {
				start = time(NULL);
				idx += 1.0;
				if (start > last + 5) {
					printf("\n\ngroup_id %d pym fps %d\n", info->group_id, idx / 5.0);
					last = start;
					idx = 0.0;
				}
				/*printf("pym_img_info:frameid %d time_stamp %d ms\n",*/
						/*out_pym_buf.pym_img_info.frame_id,*/
						/*out_pym_buf.pym_img_info.time_stamp/HW_TIMER);*/
				/*printf("Get Frame ok grpid=%d pym_chn=%d bufindex=%d \n",*/
					/*info->group_id, info->data_type, out_pym_buf.pym_img_info.buf_index);*/
				if (vps_dump) {
					for (i = 0; i < 6; i++) {
						snprintf(file_name, sizeof(file_name),
							"grp%d_pym_out_frmid%03d_basic_layer_DS%d_%d_%d.yuv",
							grp_id, out_pym_buf.pym_img_info.frame_id, i * 4,
							out_pym_buf.pym[i].width,
							out_pym_buf.pym[i].height);
						if (out_pym_buf.pym[i].width != 0)
						dumpToFile2planeStride(file_name, out_pym_buf.pym[i].addr[0],
											out_pym_buf.pym[i].addr[1],
											out_pym_buf.pym[i].width *
											out_pym_buf.pym[i].height,
											out_pym_buf.pym[i].width *
											out_pym_buf.pym[i].height / 2,
											out_pym_buf.pym[i].width,
											out_pym_buf.pym[i].height,
											out_pym_buf.pym[i].stride_size);
						for (int j = 0; j < 3; j++) {
							snprintf(file_name, sizeof(file_name),
								"grp%d_pym_out_roi_layer_DS%d_%d_%d.yuv",
								grp_id, i * 4 + j + 1,
								out_pym_buf.pym_roi[i][j].width,
								out_pym_buf.pym_roi[i][j].height);
							if (out_pym_buf.pym_roi[i][j].width != 0)
								dumpToFile2planeStride(file_name,
								out_pym_buf.pym_roi[i][j].addr[0],
								out_pym_buf.pym_roi[i][j].addr[1],
								out_pym_buf.pym_roi[i][j].width *
								out_pym_buf.pym_roi[i][j].height,
								out_pym_buf.pym_roi[i][j].width *
								out_pym_buf.pym_roi[i][j].height / 2,
								out_pym_buf.pym_roi[i][j].width,
								out_pym_buf.pym_roi[i][j].height,
								out_pym_buf.pym_roi[i][j].stride_size);
						}
						snprintf(file_name, sizeof(file_name),
							"grp%d_pym_out_us_layer_US%d_%d_%d.yuv", grp_id,
							i, out_pym_buf.us[i].width, out_pym_buf.us[i].height);
						if (out_pym_buf.us[i].width != 0)
							dumpToFile2planeStride(file_name, out_pym_buf.us[i].addr[0],
											out_pym_buf.us[i].addr[1],
											out_pym_buf.us[i].width *
											out_pym_buf.us[i].height,
											out_pym_buf.us[i].width *
											out_pym_buf.us[i].height / 2,
											out_pym_buf.us[i].width,
											out_pym_buf.us[i].height,
											out_pym_buf.us[i].stride_size);
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

int feedback_read_file(hb_vio_buffer_t *feedback_buf)
{
	struct stat statbuf;
	int img_in_fd, img_out_fd;
	uint32_t sizeraw;

	memset(feedback_buf, 0, sizeof(hb_vio_buffer_t));
	 if(stat("1952.raw",&statbuf)==0)
		printf("statbuf.st_size %d\n", statbuf.st_size);
	sizeraw = statbuf.st_size;
	prepare_user_buf(feedback_buf, sizeraw);
	feedback_buf->img_info.planeCount = 1;
	feedback_buf->img_info.img_format = 0;
	feedback_buf->img_addr.width = 1952;
	feedback_buf->img_addr.height = 1907;
	feedback_buf->img_addr.stride_size = 2928;

	img_in_fd = open("1952.raw", O_RDWR | O_CREAT, 0644);
	if (img_in_fd < 0) {
		printf("open image failed !\n");
		return -1;
	}
	read(img_in_fd, feedback_buf->img_addr.addr[0], sizeraw);
	close(img_in_fd);
	dumpToFile("123.raw", feedback_buf->img_addr.addr[0], sizeraw);
	return 0;
}

int raw_feed_back_func(work_info_t * info, hb_vio_buffer_t *feedback_buf)
{
	hb_vio_buffer_t *isp_yuv = NULL;
	struct timeval time_now = { 0 };
	struct timeval time_next = { 0 };
	int size = -1, ret = 0;
	char file_name[100] = { 0 };
	int time_ms = 0;
	struct timeval select_timeout = {0};
	int pipeId = info->group_id;
	fd_set readfd;

	FD_ZERO(&readfd);

	if (g_vin_fd[pipeId] != 0) {
		FD_SET(g_vin_fd[pipeId], &readfd);
		printf("raw_feed_back_func g_vin_fd[pipeId] %d chn0 need listen\n", g_vin_fd[pipeId]);
	}
	select_timeout.tv_usec = 500 * 1000;
	isp_yuv = (hb_vio_buffer_t *) malloc(sizeof(hb_vio_buffer_t));
	if (isp_yuv == NULL)
		return -1;

	memset(isp_yuv, 0, sizeof(hb_vio_buffer_t));

	ret = HB_VIN_SendPipeRaw(pipeId, feedback_buf, 1000);
	if (ret < 0) {
		printf("HB_VIN_SendPipeRaw error!!!\n");
		free(isp_yuv);
		isp_yuv = NULL;
		return -1;
	}
	printf("raw_feed_back_func begin===\n");

	ret = select(g_vin_fd[pipeId] + 1, &readfd, NULL, NULL, &select_timeout);
	if (ret == -1) {
			printf("raw_feed_back_func select error\n");
	} else if (ret) {
		printf("raw_feed_back_func select return\n");
		if (g_vin_fd[pipeId] != 0) {
			if (FD_ISSET(g_vin_fd[pipeId], &readfd)) {
					gettimeofday(&time_now, NULL);
					time_ms = time_now.tv_sec * 1000 + time_now.tv_usec /1000;
					ret = HB_VIN_GetChnFrame(pipeId, 0, isp_yuv, 2000);
					if (ret < 0) {
						printf("HB_VIN_GetPipeFrame error!!!\n");
					} else {
						if(yuv_dump) {
							normal_buf_info_print(isp_yuv);
							size = isp_yuv->img_addr.stride_size * isp_yuv->img_addr.height;
							printf("yuv stride_size(%u) w x h%u x %u, size %d\n",
										isp_yuv->img_addr.stride_size,
										isp_yuv->img_addr.width, isp_yuv->img_addr.height, size);
							snprintf(file_name, sizeof(file_name),
										"./isp_yuv_%d_index%d.yuv", time_ms,
																	isp_yuv->img_info.buf_index);
							gettimeofday(&time_now, NULL);
							dumpToFile2plane(file_name, isp_yuv->img_addr.addr[0],
											isp_yuv->img_addr.addr[1], size, size/2);
							gettimeofday(&time_next, NULL);
							int time_cost = time_cost_ms(&time_now, &time_next);
							printf("dumpToFile yuv cost time %d ms", time_cost);
						}
						ret = HB_VIN_ReleaseChnFrame(pipeId, 0, isp_yuv);
						if (ret < 0) {
							printf("HB_VIN_ReleaseChnFrame error!!!\n");
					        }
				}
			}
		}
	}

	free(isp_yuv);
	isp_yuv = NULL;
}

void  *work_md_func_thread(void* arg)
{
	printf("work_md_func_thread begin\n");
	while (check_end()) {
		md_func(arg);
	}
	printf("work_md_func_thread end\n");

	return NULL;
}

void  *work_sif_raw_dump_thread(void* arg)
{
	printf("work_sif_raw_dump_thread begin\n");
	while (check_end()) {
		sif_raw_dump_func(arg);
	}
	printf("work_sif_raw_dump_thread end\n");

	return NULL;
}
void  *work_isp_yuv_dump_thread(void* arg)
{
	printf("work_isp_yuv_dump_thread begin\n");
	while (check_end()) {
		isp_yuv_dump_func(arg);
	}
	printf("work_isp_yuv_dump_thread end\n");

	return NULL;
}
void  *work_isp_yuv_getchnfd_dump_thread(void* arg)
{
	printf("work_isp_yuv_getchnfd_dump_thread begin\n");
	while (check_end()) {
		isp_yuv_dump_getchnfd_func(arg);
	}
	printf("work_isp_yuv_getchnfd_dump_thread end\n");

	return NULL;
}

void  *work_raw_feedback_thread(void* arg)
{
	hb_vio_buffer_t feedback_buf;

	printf("work_raw_feedback_thread begin\n");
	memset(&feedback_buf, 0, sizeof(hb_vio_buffer_t));
	feedback_read_file(&feedback_buf);
	while (check_end()) {
		raw_feed_back_func(arg, &feedback_buf);
	}
	printf("work_raw_feedback_thread end\n");

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

void *mirror_ctrl_thread(void *arg)
{
	int ret = 0;
	int pipeid, val;
	char str[4];

	while (check_end()) {

		printf("input mirror ctrl cmd (pipeid, vlaue), like 01>\n");

		scanf("%s", str);
        pipeid = str[0] - 48;
		val = str[1] - 48;

		if (pipeid < 0 || pipeid > 8) {
			printf("pipeid %d is invalid.\n", pipeid);
			continue;
		}

		if (val != 0 && val != 1) {
			printf("mirror value set %d is invalid.\n", val);
			continue;
		}

		printf("pipe id %d, mirror set %d\n", pipeid, val);

		HB_VIN_CtrlPipeMirror(pipeid, val);

		usleep(50 * 1000);
	}

	return NULL;
}

pthread_t vot_pid;

void vot_pause_thread()
{
	int pause = 0, ret = 0;

	sleep(5);
	while (check_end() || (g_exit == 1)) {
		ret = HB_VOT_SetPause(pause);
		printf("HB_VOT_SetPause set:%d ret:%d\n", pause, ret);

		pause = !pause;
		sleep(5);
	}
}

int hb_vot_init()
{
	int ret = 0;
    VOT_PUB_ATTR_S devAttr = {0};
    VOT_VIDEO_LAYER_ATTR_S stLayerAttr = {0};
    VOT_CHN_ATTR_S stChnAttr = {0};
    VOT_CROP_INFO_S cropAttrs = {0};
	uint32_t width, height;
	int chn = 0;
	struct HB_SYS_MOD_S src_mod, dst_mod;

	width = 1920;
	height = 1080;

	devAttr.enIntfSync = VOT_OUTPUT_1920x1080;
	devAttr.u32BgColor = 0x8080;
	devAttr.enOutputMode = HB_VOT_OUTPUT_BT1120;

	ret = HB_VOT_SetPubAttr(0, &devAttr);
	if (ret) {
		printf("HB_VOT_SetPubAttr failed\n");
		goto err;
	}

	ret = HB_VOT_Enable(0);
	if (ret) {
		printf("HB_VOT_Enable failed.\n");
		goto err;
	}

	ret = HB_VOT_GetVideoLayerAttr(0, &stLayerAttr);
	if (ret) {
		printf("HB_VOT_GetVideoLayerAttr failed.\n");
		goto err;
	}

	stLayerAttr.stImageSize.u32Width = width;
	stLayerAttr.stImageSize.u32Height = height;

	stLayerAttr.panel_type = 0;
	stLayerAttr.rotate = 0;
	stLayerAttr.dithering_flag = 0;
	stLayerAttr.dithering_en = 0;
	stLayerAttr.gamma_en = 0;
	stLayerAttr.hue_en = 0;
	stLayerAttr.sat_en = 0;
	stLayerAttr.con_en = 0;
	stLayerAttr.bright_en = 0;
	stLayerAttr.theta_sign = 0;
	stLayerAttr.contrast = 0;
	stLayerAttr.theta_abs = 0;
	stLayerAttr.saturation = 0;
	stLayerAttr.off_contrast = 0;
	stLayerAttr.off_bright = 0;
	stLayerAttr.user_control_disp = 1;
	stLayerAttr.user_control_disp_layer1 = 1;
	stLayerAttr.big_endian = 0;
	ret = HB_VOT_SetVideoLayerAttr(0, &stLayerAttr);
	if (ret) {
		printf("HB_VOT_SetVideoLayerAttr failed.\n");
		goto err;
	}

	ret = HB_VOT_EnableVideoLayer(0);
	if (ret) {
		printf("HB_VOT_EnableVideoLayer failed.\n");
		// goto err;
	}

    // 0 is highest priority, graph layer need higher than video layer
    stChnAttr.u32Priority = (chn >= 2) ? (chn - 2) : (chn + 2);
    stChnAttr.s32X = 0;
    stChnAttr.s32Y = 0;
    stChnAttr.u32SrcWidth = width;
    stChnAttr.u32SrcHeight = height;
    stChnAttr.u32DstWidth = width;
    stChnAttr.u32DstHeight = height;
    ret = HB_VOT_SetChnAttr(0, chn, &stChnAttr);
    if (ret) {
        printf("HB_VOT_SetChnAttr failed: %d\n", ret);
        goto err;
    }

    cropAttrs.u32Width = stChnAttr.u32DstWidth;   // - stChnAttr.s32X;
    cropAttrs.u32Height = stChnAttr.u32DstHeight;  //- stChnAttr.s32Y;
    ret = HB_VOT_SetChnCrop(0, chn, &cropAttrs);
    if (ret < 0) {
        printf("HB_VOT_SetChnCrop failed: %d\n", ret);
    }

    ret = HB_VOT_EnableChn(0, chn);
    if (ret) {
        printf("HB_VOT_EnableChn failed: %d\n", ret);
        goto err;
    }

	src_mod.enModId = HB_ID_VPS;
	src_mod.s32DevId = 0;
	src_mod.s32ChnId = 2;
	dst_mod.enModId = HB_ID_VOT;
	dst_mod.s32DevId = 0;
	dst_mod.s32ChnId = chn;
	ret = HB_SYS_Bind(&src_mod, &dst_mod);
	if (ret != 0)
		printf("HB_SYS_Bind failed\n");

	ret = pthread_create(&vot_pid, NULL, (void *)vot_pause_thread, NULL);

	return ret;

err:
    HB_VOT_DisableVideoLayer(0);
    HB_VOT_Disable(0);

	return ret;
}

void hb_vot_deinit()
{
	int chn = 0;
	struct HB_SYS_MOD_S src_mod, dst_mod;

	pthread_join(vot_pid, NULL);

	src_mod.enModId = HB_ID_VPS;
	src_mod.s32DevId = 0;
	src_mod.s32ChnId = 2;
	dst_mod.enModId = HB_ID_VOT;
	dst_mod.s32DevId = 0;
	dst_mod.s32ChnId = chn;

	HB_SYS_UnBind(&src_mod, &dst_mod);
	HB_VOT_DisableChn(0, chn);
	HB_VOT_DisableVideoLayer(0);
	HB_VOT_Disable(0);
}

int main(int argc, char *argv[])
{
	int ret = 0, i;

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
			printf("%d===groupMask %d ====sensorId[i] %d ===mipiIdx[i] %d=\n", i, groupMask, sensorId[i], mipiIdx[i]);
			ret = hb_vin_vps_init(i, sensorId[i], mipiIdx[i], serdes_port[i], vin_vps_mode[i], virtual[i]);
			if(ret < 0) {
				printf("hb_vin_init error!\n");
				return ret;
			}
		}
	}
	if(need_cam == 1) {
		for(i = 0; i < MAX_ID_NUM; i++) {
			if (BIT(i) & groupMask) {
			    printf("%d===groupMask %d ====sensorId[i] %d ==mipiIdx[i] %d===\n", (BIT(i) & groupMask), groupMask, sensorId[i], mipiIdx[i]);
				if(need_clk == 1) {
					// HB_MIPI_SetSensorClock(mipiIdx[i], 24000000);
			  		 HB_MIPI_EnableSensorClock(mipiIdx[i]);
				}
	 			ret = hb_sensor_init(i, sensorId[i], bus[i], port[i], mipiIdx[i], serdes_index[i], serdes_port[i], extra_mode[i]);
				if(ret < 0) {
					printf("hb_sensor_init error! do vio deinit\n");
					return ret;
				}
				ret = hb_mipi_init(sensorId[i], mipiIdx[i]);
				if(ret < 0) {
					printf("hb_mipi_init error! do vio deinit\n");
					hb_vin_vps_deinit(i);
					return ret;
				}
			}
		}
	}
	if(need_cam == 2) {  // testpattern
		for(i = 0; i < MAX_ID_NUM; i++) {
			if (BIT(i) & groupMask) {
			    printf("%d===groupMask %d ====sensorId[i] %d ==mipiIdx[i] %d===\n", (BIT(i) & groupMask), groupMask, sensorId[i], mipiIdx[i]);

	 			ret = hb_sensor_init(i, sensorId[i], bus[i], port[i], mipiIdx[i], serdes_index[i], serdes_port[i], extra_mode[i]);
				if(ret < 0) {
					printf("hb_sensor_init error! do vio deinit\n");
					return ret;
				}
				ret = HB_MIPI_SetSensorFps(i, testpattern_fps);
				if(ret < 0) {
					printf("HB_MIPI_SetSensorFps error! do vio deinit\n");
					return ret;
				}
			}
		}
	}
	if (need_pym & 0x800) {  // means need pym feedback
		int grp_id = (need_pym & 0x700) >> 8;
		uint32_t size_y, size_uv;
		hb_vio_buffer_t feedback_buf;
		struct feedback_arg_s feedback_arg;
		int img_in_fd;

		hb_pym_feedback_init_start(grp_id);

		memset(&feedback_buf, 0, sizeof(hb_vio_buffer_t));
		size_y = 1280 * 720;
		size_uv = size_y / 2;
		prepare_user_buf_2lane(&feedback_buf, size_y, size_uv);
		feedback_buf.img_info.planeCount = 2;
		feedback_buf.img_info.img_format = 8;
		feedback_buf.img_addr.width = 1280;
		feedback_buf.img_addr.height = 720;
		feedback_buf.img_addr.stride_size = 1280;

		img_in_fd = open("1280720.yuv", O_RDWR | O_CREAT, 0644);
		if (img_in_fd < 0) {
			printf("open image failed !\n");
			return -1;
		}

		read(img_in_fd, feedback_buf.img_addr.addr[0], size_y);
		usleep(10 * 1000);
		read(img_in_fd, feedback_buf.img_addr.addr[1], size_uv);
		usleep(10 * 1000);
		close(img_in_fd);

		feedback_arg.grp_id = grp_id;
		feedback_arg.feedback_buf = &feedback_buf;

		pthread_create(&feedback_arg.thid,
							NULL,
						   (void *)vps_feedback_get_dump_thread,
						   (void *)(&feedback_arg));

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
				hb_vin_vps_deinit(i);
				return ret;
			}
		}
	}
	if(need_cam == 1) {
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
				hb_vin_vps_deinit(i);
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
				hb_vin_vps_deinit(i);
				return ret;
			}
		  }
		}
	}
	if (vot_enable) {
		ret = hb_vot_init();
	}
	signal(SIGINT, intHandler);

	pthread_t mirror_thid;
	if (need_m_thread) {
		if (mirror_ctrl) {
			ret = pthread_create(&mirror_thid, NULL, (void *)mirror_ctrl_thread, NULL);
			if (ret != 0) {
				printf("mirror_ctrl_thread thread start error.\n");
			}
		}
		for (int id = 0; id < MAX_SENSOR_NUM; id++) {
			if (BIT(id) & groupMask) {
				printf("data_type & RAW_ENABLE %d\n", data_type & RAW_ENABLE);
				printf("data_type & YUV_ENABLE %d\n", data_type & YUV_ENABLE);
				printf("data_type & MD_ENABLE %d\n", data_type & MD_ENABLE);
				printf("data_type & RAW_FEEDBACK_ENABLE %d\n", data_type & RAW_FEEDBACK_ENABLE);
				if(data_type & RAW_ENABLE) {
					printf("data_type & RAW_ENABLE %d\n", data_type & RAW_ENABLE);
					ret = pthread_create(&work_info[id][HB_VIO_SIF_RAW_DATA].thid,
								NULL,
							   (void *)work_sif_raw_dump_thread,
							   (void *)(&work_info[id][HB_VIO_SIF_RAW_DATA]));
					printf("pipe(%d)Test work raw_data thread---running.\n", id);
				}
				if(data_type & YUV_ENABLE) {
				printf("data_type & YUV_ENABLE %d\n", data_type & YUV_ENABLE);
				if(need_chnfd) {
					ret = pthread_create(
								&work_info[id][HB_VIO_ISP_YUV_DATA].thid,
								NULL,
							   (void *)work_isp_yuv_getchnfd_dump_thread,
							   (void *)(&work_info[id][HB_VIO_ISP_YUV_DATA]));
					} else {
					ret = pthread_create(
										&work_info[id][HB_VIO_ISP_YUV_DATA].thid,
										NULL,
									   (void *)work_isp_yuv_dump_thread,
									   (void *)(&work_info[id][HB_VIO_ISP_YUV_DATA]));


					}
					printf("pipe(%d)Test work yuv dump thread---running.\n", id);
				}
				if(data_type & RAW_FEEDBACK_ENABLE) {
				printf("data_type & RAW_FEEDBACK_ENABLE %d\n", data_type & RAW_FEEDBACK_ENABLE);
				ret = pthread_create(
							&work_info[id][HB_VIO_SIF_FEEDBACK_SRC_DATA].thid,
							NULL,
						   (void *)work_raw_feedback_thread,
						   (void *)(&work_info[id][HB_VIO_SIF_FEEDBACK_SRC_DATA]));
				   printf("pipe(%d)Test work raw_feedback thread---running.\n", id);
				}
				if(data_type & MD_ENABLE) {
					printf("data_type & MD_ENABLE %d\n", data_type & MD_ENABLE);
					ret = pthread_create(&work_info[id][HB_VIO_MD_DATA].thid,
								NULL,
							   (void *)work_md_func_thread,
							   (void *)(&work_info[id][HB_VIO_MD_DATA]));
					printf("pipe(%d)Test work MD_DATA_FUNC thread---running.\n", id);
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
/*
 * todo delete
 */
	/*
	 *ret = pthread_create(
	 *                        &work_info[1][2].thid,
	 *                        NULL,
	 *                   (void *)work_vps_get_frame_thread,
	 *                   (void *)(&work_info[1][2]));
	 */


	int loop_cnt = 0;
	while (check_end()) {
		loop_cnt++;
		if (g_exit == 1)
			break;
		/*sleep(1);*/
		usleep(30 * 1000);
		if (loop_cnt % 10 == 5 && vps_dump == 5) {
			printf("HB_VPS_SetGrpRotate %d\n", (loop_cnt >> 2) & 0x3);
			ret = HB_VPS_SetGrpRotate(0, (loop_cnt >> 2) & 0x3);
		}

	}

	if (need_m_thread) {

		if (mirror_ctrl) {
			pthread_join(mirror_thid, NULL);
		}

		for (int id = 0; id < 4; id++) {
			if (BIT(id) & groupMask) {
				if(data_type & RAW_ENABLE) {
				   pthread_join(work_info[id][HB_VIO_SIF_RAW_DATA].thid, NULL);
				   printf("pipe(%d)Test work raw_data thread quit done..\n", id);
				}
				if(data_type & YUV_ENABLE) {
				  pthread_join(work_info[id][HB_VIO_ISP_YUV_DATA].thid, NULL);
				  printf("pipe(%d)Test work yuv thread quit done..\n", id);
				}
				if(data_type & RAW_FEEDBACK_ENABLE) {
				   pthread_join(work_info[id][HB_VIO_SIF_FEEDBACK_SRC_DATA].thid,
							NULL);
				   printf("pipe(%d)Test work raw_feedback thread quit done.\n", id);
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
	if(need_cam == 1) {
	 for (i = 0; i < MAX_ID_NUM; i++) {
		if (BIT(i) & groupMask) {
			hb_sensor_stop(i);
			hb_mipi_stop(mipiIdx[i]);
		  }
	  }
	}
	for (i = 0; i < MAX_ID_NUM; i++) {
		if (BIT(i) & groupMask) {
			hb_vin_vps_stop(i);
		}
	}

	if (vot_enable) {
		hb_vot_deinit();
	}
	if (need_cam == 1) {
	  for (i = 0; i < MAX_ID_NUM; i++) {
		if (BIT(i) & groupMask) {
			hb_mipi_deinit(mipiIdx[i]);
			hb_sensor_deinit(i);
			if(need_clk == 1) {
				HB_MIPI_DisableSensorClock(mipiIdx[i]);
			}
		 }
	   }
	}
	for (i = 0; i < MAX_ID_NUM; i++) {
		if (BIT(i) & groupMask) {
			hb_vin_vps_deinit(i);
		}
	}
	printf("-----------------Test done success-----------------\n");
	return 0;
}
