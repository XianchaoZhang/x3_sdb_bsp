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
#include "sample.h"

// #define MAX_SENSOR_NUM  6
// #define MAX_MIPIID_NUM  4
// #define MAX_ID_NUM 32

// #define MD_DATA_FUNC  15
// #define BIT(n)  (1UL << (n))

// #define RAW_ENABLE BIT(HB_VIO_SIF_RAW_DATA)  //  for raw dump/fb
// #define YUV_ENABLE BIT(HB_VIO_ISP_YUV_DATA)   // for yuv dump
// #define RAW_FEEDBACK_ENABLE BIT(HB_VIO_SIF_FEEDBACK_SRC_DATA)  // feedback
// #define MD_ENABLE BIT(MD_DATA_FUNC)  // MD  15

// #define VPS_CHN0_ENABLE BIT(HB_VIO_IPU_DS0_DATA)
// #define VPS_CHN1_ENABLE BIT(HB_VIO_IPU_DS1_DATA)
// #define VPS_CHN2_ENABLE BIT(HB_VIO_IPU_DS2_DATA)
// #define VPS_CHN3_ENABLE BIT(HB_VIO_IPU_DS3_DATA)
// #define VPS_CHN4_ENABLE BIT(HB_VIO_IPU_DS4_DATA)
// #define VPS_CHN5_ENABLE BIT(HB_VIO_IPU_US_DATA)
// #define VPS_CHN6_ENABLE BIT(HB_VIO_PYM_FEEDBACK_SRC_DATA)

// #define BIT2CHN(chns, chn) (chns & (1 << chn))

// time_t start_time, now;
// time_t run_time = 0;
// time_t end_time = 0;
// int g_vin_fd[MAX_SENSOR_NUM];
// int sensorId[MAX_SENSOR_NUM];
// int mipiIdx[MAX_MIPIID_NUM];
// int bus[MAX_SENSOR_NUM];
// int port[MAX_SENSOR_NUM];
// int serdes_index[MAX_SENSOR_NUM];
// int serdes_port[MAX_SENSOR_NUM];
// int groupMask;
// int need_clk;
// int raw_dump;
// int yuv_dump;
// int need_cam;
// int need_m_thread;
// int data_type;
// int vin_vps_mode = VIN_ONLINE_VPS_ONLINE;
// int need_grp_rotate;
// int need_chn_rotate;
// int need_ipu;
// int need_pym;
// int vps_dump;

/*temp bind need this, use sysbind do not use this*/
// #include "vps_group.h"
// #include "vin_group.h"
// extern struct hb_vin_group_s *g_vin[3];

// typedef enum group_id {
// 	GROUP_0 = 0,
// 	GROUP_1,
// 	GROUP_2,
// 	GROUP_3,
// 	GROUP_MAX
// } group_id_e;

// typedef struct work_info_s {
// 	uint32_t group_id;
// 	VIO_DATA_TYPE_E data_type;
// 	pthread_t thid;
// 	int running;
// } work_info_t;
// work_info_t work_info[GROUP_MAX][20];

// struct feedback_arg_s {
// 	int grp_id;
// 	pthread_t thid;
// 	hb_vio_buffer_t *feedback_buf;
// };

// typedef enum HB_MIPI_SNS_TYPE_E
// {
// 	SENSOR_ID_INVALID,
// 	IMX327_30FPS_1952P_RAW12_LINEAR,  // 1
// 	IMX327_30FPS_2228P_RAW12_DOL2,    // 2
// 	AR0233_30FPS_1080P_RAW12_954_PWL,	 // 3
// 	AR0233_30FPS_1080P_RAW12_960_PWL,	// 4
// 	OS8A10_25FPS_3840P_RAW10_LINEAR,	// 5
// 	OS8A10_25FPS_3840P_RAW10_DOL2,	 // 6
// 	OV10635_30FPS_720p_954_YUV,   // 7
// 	OV10635_30FPS_720p_960_YUV,   // 8
// 	SIF_TEST_PATTERN0_1080P,   // 9
// 	SIF_TEST_PATTERN0_1520P,   // 10
// 	SIF_TEST_PATTERN0_2160P,   // 11
// 	FEED_BACK_RAW12_1952P,    // 12
// 	SAMPLE_SENOSR_ID_MAX,
// } MIPI_SNS_TYPE_E;

MIPI_SENSOR_INFO_S SENSOR_4LANE_IMX327_30FPS_12BIT_LINEAR_INFO =
{
	.deseEnable = 0,
	.inputMode = INPUT_MODE_MIPI,
	.sensorInfo = {
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
		.fps = 30,
		.resolution = 2228,
		.sensor_addr = 0x36,
		.entry_index = 1,
		.sensor_mode = 2,
		.reg_width = 16,
		.sensor_name = "imx327"
	}
};

MIPI_SENSOR_INFO_S SENSOR_OS8A10_25FPS_10BIT_LINEAR_INFO =
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

MIPI_SENSOR_INFO_S SENSOR_OS8A10_25FPS_10BIT_DOL2_INFO =
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

MIPI_SENSOR_INFO_S SENSOR_S5KGM1SP_30FPS_10BIT_LINEAR_INFO =
{
	.deseEnable = 0,
	.inputMode = INPUT_MODE_MIPI,
	.sensorInfo = {
	    .bus_num = 4,
		.fps = 30,
		.resolution = 3000,
		.sensor_addr = 0x10,
		.entry_index = 1,
		.sensor_mode = 1,
		.reg_width = 16,
		.sensor_name = "s5kgm1sp"
	}
};

MIPI_SENSOR_INFO_S SENSOR_4LANE_AR0233_30FPS_12BIT_1080P_954_INFO =
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
		.deserial_port = 0,
		.extra_mode = 0
	}
};

MIPI_SENSOR_INFO_S SENSOR_4LANE_AR0233_30FPS_12BIT_1080P_960_INFO =
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
		.deserial_port = 0,
		.extra_mode = 0
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
		.deserial_port = 0,
		.extra_mode = 0
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
		.deserial_port = 0,
		.extra_mode = 0
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

MIPI_ATTR_S MIPI_SENSOR_S5KGM1SP_30FPS_10BIT_LINEAR_ATTR =
{
	.mipi_host_cfg =
	{
		4,			  /* lane */
		0x2b,		  /* datatype */
		24, 		  /* mclk	 */
		4600,		  /* mipiclk */
		30, 		  /* fps */
		4000,		  /* width	*/
		3000,		  /*height */
		5024,		  /* linlength */
		3194,		  /* framelength */
		30, 		  /* settle */
		 2, 		  /*chnnal_num*/
		{0, 1}	  /*vc */
	},
	.dev_enable = 0  /*  mipi dev enable */
};

MIPI_ATTR_S MIPI_SENSOR_OS8A10_25FPS_10BIT_LINEAR_ATTR =
{
	.mipi_host_cfg =
	{
		4,			  /* lane */
		0x2b,		  /* datatype */
		2400, 		  /* mclk	 */
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

 MIPI_ATTR_S MIPI_SENSOR_OS8A10_25FPS_10BIT_LINEAR_SENSOR_CLK_ATTR =
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

MIPI_ATTR_S MIPI_SENSOR_OS8A10_25FPS_10BIT_DOL2_ATTR =
{
	.mipi_host_cfg =
	{
		4,			  /* lane */
		0x2b,		  /* datatype */
		2400, 		  /* mclk	 */
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

VIN_DEV_ATTR_S DEV_ATTR_S5KGM1SP_LINEAR_BASE = {
	.stSize = { 0,		/*format*/
				4000,	/*width*/
				3000,	 /*height*/
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
			.width = 4000,
			.height = 3000,
			.pix_length = 1,
		}
	},
	.outDdrAttr = {
		.stride = 5000,
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
		.buffer_num = 3,
		.frameDepth = 0,
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
	.calib = {
		.mode = 0,
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
	.calib = {
		.mode = 0,
		.lname = "libimx327_linear.so",
	}
};

VIN_PIPE_ATTR_S PIPE_ATTR_AR0233_1080P_BASE = {
	.ddrOutBufNum = 6,
	.snsMode = SENSOR_PWL_MODE,
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

VIN_PIPE_ATTR_S PIPE_ATTR_S5KGM1SP_LINEAR_BASE = {
	.ddrOutBufNum = 6,
	.snsMode = SENSOR_NORMAL_MODE,
	.stSize = {
		.format = 0,
		.width = 4000,
		.height = 3000,
	},
	.cfaPattern = 1,
	.temperMode = 2,
	.ispBypassEn = 0,
	.ispAlgoState = 1,
	.bitwidth = 10,
	.calib = {
		.mode = 0,
		.lname = "lib_imx327.so",
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
	.ddrOutBufNum = 2,
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
	},
	.disBufNum = 3,
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
	},
	.disBufNum = 3,
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
	},
	.disBufNum = 3,
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

VIN_DIS_ATTR_S DIS_ATTR_1520P_BASE = {
	.picSize = {
		.pic_w = 2687,
		.pic_h = 1519,
	},
	.disPath = {
		.rg_dis_enable = 0,
		.rg_dis_path_sel = 1,
	},
	.disHratio = 65536,
	.disVratio = 65536,
	.xCrop = {
		.rg_dis_start = 0,
		.rg_dis_end = 2687,
	},
	.yCrop = {
		.rg_dis_start = 0,
		.rg_dis_end = 1519,
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

VIN_LDC_ATTR_S LDC_ATTR_1520P_BASE = {
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
	 .pic_w = 2687,
	 .pic_h = 1519,
  },
  .lineBuf = 99,
  .xParam = {
	 .rg_algo_param_b = 3,
	 .rg_algo_param_a = 2,
  },
  .yParam = {
	 .rg_algo_param_b = 5,
	 .rg_algo_param_a = 4,
  },
  .offShift = {
	 .rg_center_xoff = 0,
	 .rg_center_yoff = 0,
  },
  .xWoi = {
	 .rg_start = 0,
	 .rg_length = 2687,
  },
  .yWoi = {
	 .rg_start = 0,
	 .rg_length = 1519,
  }
};

VIN_LDC_ATTR_S LDC_ATTR_OS8A10_BASE = {
  .ldcEnable = 0,
  .ldcPath = {
	 .rg_y_only = 0,
	 .rg_uv_mode = 0,
	 .rg_uv_interpo = 0,
	 .rg_h_blank_cyc = 32,
  },
  .yStartAddr = 1134592,
  .cStartAddr = 1153072,
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
	.buf_num = 8,
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

VIN_DEV_ATTR_S DEV_ATTR_TEST_PATTERN_1520P_BASE = {
  .stSize = { 0,        /*format*/
              2688,        /*width*/
              1520,         /*height*/
              1                 /*pix_length*/
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
                .width = 2688,
                .height = 1520,
                .pix_length = 1,
        }
  },
  .outDdrAttr = {
        .stride = 3360,
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

VIN_PIPE_ATTR_S PIPE_ATTR_TEST_PATTERN_1080P_BASE = {
	  .ddrOutBufNum = 6,
	  .snsMode = SENSOR_NORMAL_MODE,
	  .stSize = {
		 .format = 0,
		 .width = 1920,
		 .height = 1080,
	  },
	  .temperMode = 3,
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

VIN_PIPE_ATTR_S PIPE_ATTR_TEST_PATTERN_1520P_BASE = {
           .ddrOutBufNum = 6,
           .snsMode = SENSOR_NORMAL_MODE,
           .stSize = {
                  .format = 0,
                  .width = 2688,
                  .height = 1520,
           },
           .ispBypassEn = 0,
           .ispAlgoState = 0,
           .bitwidth = 10,
 };

VIN_PIPE_ATTR_S PIPE_ATTR_TEST_PATTERN_4K_BASE = {
   .ddrOutBufNum = 6,
   .snsMode = SENSOR_NORMAL_MODE,
   .stSize = {
	  .format = 0,
	  .width = 3840,
	  .height = 2160,
   },
   .temperMode = 2,
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
			 memcpy(pstSnsAttr, &SENSOR_4LANE_IMX327_30FPS_12BIT_DOL2_INFO, sizeof(MIPI_SENSOR_INFO_S));
			 break;
		 case AR0233_30FPS_1080P_RAW12_954_PWL:
			 memcpy(pstSnsAttr, &SENSOR_4LANE_AR0233_30FPS_12BIT_1080P_954_INFO, sizeof(MIPI_SENSOR_INFO_S));
			 break;
		 case AR0233_30FPS_1080P_RAW12_960_PWL:
			 memcpy(pstSnsAttr, &SENSOR_4LANE_AR0233_30FPS_12BIT_1080P_960_INFO, sizeof(MIPI_SENSOR_INFO_S));
			 break;
		 case OS8A10_25FPS_3840P_RAW10_LINEAR:
			 memcpy(pstSnsAttr, &SENSOR_OS8A10_25FPS_10BIT_LINEAR_INFO, sizeof(MIPI_SENSOR_INFO_S));
			 break;
		 case OS8A10_25FPS_3840P_RAW10_DOL2:
			 memcpy(pstSnsAttr, &SENSOR_OS8A10_25FPS_10BIT_DOL2_INFO, sizeof(MIPI_SENSOR_INFO_S));
			 break;
		 case OV10635_30FPS_720p_954_YUV:
			 memcpy(pstSnsAttr, &SENSOR_2LANE_OV10635_30FPS_YUV_720P_954_INFO, sizeof(MIPI_SENSOR_INFO_S));
			 break;
		 case OV10635_30FPS_720p_960_YUV:
		 	 memcpy(pstSnsAttr, &SENSOR_2LANE_OV10635_30FPS_YUV_720P_960_INFO, sizeof(MIPI_SENSOR_INFO_S));
			 break;
		 case SIF_TEST_PATTERN0_1080P:
		 case SIF_TEST_PATTERN0_1520P:
		 case SIF_TEST_PATTERN0_2160P:
		 case SIF_TEST_PATTERN_YUV_720P:
		 case SIF_TEST_PATTERN_12M_RAW12:
		 	 memcpy(pstSnsAttr, &SENSOR_TESTPATTERN_INFO, sizeof(MIPI_SENSOR_INFO_S));
			 break;
		 case S5KGM1SP_30FPS_4000x3000_RAW10:
		 	 memcpy(pstSnsAttr, &SENSOR_S5KGM1SP_30FPS_10BIT_LINEAR_INFO, sizeof(MIPI_SENSOR_INFO_S));
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
			 memcpy(pstMipiAttr, &MIPI_4LANE_SENSOR_IMX327_30FPS_12BIT_DOL2_ATTR, sizeof(MIPI_ATTR_S));
			 break;
		 case AR0233_30FPS_1080P_RAW12_954_PWL:
			 memcpy(pstMipiAttr, &MIPI_4LANE_SENSOR_AR0233_30FPS_12BIT_1080P_954_ATTR, sizeof(MIPI_ATTR_S));
			 break;
		 case AR0233_30FPS_1080P_RAW12_960_PWL:
			 memcpy(pstMipiAttr, &MIPI_4LANE_SENSOR_AR0233_30FPS_12BIT_1080P_960_ATTR, sizeof(MIPI_ATTR_S));
			 break;
		 case OS8A10_25FPS_3840P_RAW10_LINEAR:
			 memcpy(pstMipiAttr, &MIPI_SENSOR_OS8A10_25FPS_10BIT_LINEAR_ATTR, sizeof(MIPI_ATTR_S));
			 break;
		 case OS8A10_25FPS_3840P_RAW10_DOL2:
			 memcpy(pstMipiAttr, &MIPI_SENSOR_OS8A10_25FPS_10BIT_DOL2_ATTR, sizeof(MIPI_ATTR_S));
			 break;
		 case OV10635_30FPS_720p_954_YUV:
			 memcpy(pstMipiAttr, &MIPI_2LANE_OV10635_30FPS_YUV_720P_954_ATTR, sizeof(MIPI_ATTR_S));
			 break;
		 case OV10635_30FPS_720p_960_YUV:
			 memcpy(pstMipiAttr, &MIPI_2LANE_OV10635_30FPS_YUV_720P_960_ATTR, sizeof(MIPI_ATTR_S));
			 break;
		 case S5KGM1SP_30FPS_4000x3000_RAW10:
		 	 memcpy(pstMipiAttr, &MIPI_SENSOR_S5KGM1SP_30FPS_10BIT_LINEAR_ATTR, sizeof(MIPI_ATTR_S));
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
		 case AR0233_30FPS_1080P_RAW12_954_PWL:
		 case AR0233_30FPS_1080P_RAW12_960_PWL:
			 memcpy(pstDevAttr, &DEV_ATTR_AR0233_1080P_BASE, sizeof(VIN_DEV_ATTR_S));
			 break;
		 case OS8A10_25FPS_3840P_RAW10_LINEAR:
			 memcpy(pstDevAttr, &DEV_ATTR_OS8A10_LINEAR_BASE, sizeof(VIN_DEV_ATTR_S));
			 break;
		 case OS8A10_25FPS_3840P_RAW10_DOL2:
			 memcpy(pstDevAttr, &DEV_ATTR_OS8A10_DOL2_BASE, sizeof(VIN_DEV_ATTR_S));
			 break;
		 case OV10635_30FPS_720p_954_YUV:
		 case OV10635_30FPS_720p_960_YUV:
			 memcpy(pstDevAttr, &DEV_ATTR_OV10635_YUV_BASE, sizeof(VIN_DEV_ATTR_S));
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
		case SIF_TEST_PATTERN_12M_RAW12:
			 memcpy(pstDevAttr, &DEV_ATTR_TEST_PATTERN_12M_BASE, sizeof(VIN_DEV_ATTR_S));
			 break;
		case SIF_TEST_PATTERN0_1520P:
			 memcpy(pstDevAttr, &DEV_ATTR_TEST_PATTERN_1520P_BASE, sizeof(VIN_DEV_ATTR_S));
			 break;
		case SIF_TEST_PATTERN0_2160P:
			 memcpy(pstDevAttr, &DEV_ATTR_TEST_PATTERN_4K_BASE, sizeof(VIN_DEV_ATTR_S));
			 break;
		case S5KGM1SP_30FPS_4000x3000_RAW10:
			 memcpy(pstDevAttr, &DEV_ATTR_S5KGM1SP_LINEAR_BASE, sizeof(VIN_DEV_ATTR_S));
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
		 case FEED_BACK_RAW12_1952P:
			 memcpy(pstPipeAttr, &PIPE_ATTR_IMX327_LINEAR_BASE, sizeof(VIN_PIPE_ATTR_S));
			 break;
		 case IMX327_30FPS_2228P_RAW12_DOL2:
			 memcpy(pstPipeAttr, &PIPE_ATTR_IMX327_DOL2_BASE, sizeof(VIN_PIPE_ATTR_S));
			 break;
		 case AR0233_30FPS_1080P_RAW12_954_PWL:
		 case AR0233_30FPS_1080P_RAW12_960_PWL:
			 memcpy(pstPipeAttr, &PIPE_ATTR_AR0233_1080P_BASE, sizeof(VIN_PIPE_ATTR_S));
			 break;
		 case OS8A10_25FPS_3840P_RAW10_LINEAR:
			 memcpy(pstPipeAttr, &PIPE_ATTR_OS8A10_LINEAR_BASE, sizeof(VIN_PIPE_ATTR_S));
			 break;
		 case OS8A10_25FPS_3840P_RAW10_DOL2:
			 memcpy(pstPipeAttr, &PIPE_ATTR_OS8A10_DOL2_BASE, sizeof(VIN_PIPE_ATTR_S));
			 break;
		 case OV10635_30FPS_720p_954_YUV:
		 case OV10635_30FPS_720p_960_YUV:
		 case SIF_TEST_PATTERN_YUV_720P:
			 memcpy(pstPipeAttr, &PIPE_ATTR_OV10635_YUV_BASE, sizeof(VIN_PIPE_ATTR_S));
			 break;
		 case SIF_TEST_PATTERN0_1080P:
			 memcpy(pstPipeAttr, &PIPE_ATTR_TEST_PATTERN_1080P_BASE, sizeof(VIN_PIPE_ATTR_S));
			 break;
		 case SIF_TEST_PATTERN_12M_RAW12:
			 memcpy(pstPipeAttr, &PIPE_ATTR_TEST_PATTERN_12M_BASE, sizeof(VIN_PIPE_ATTR_S));
			 break;
		 case SIF_TEST_PATTERN0_1520P:
			 memcpy(pstPipeAttr, &PIPE_ATTR_TEST_PATTERN_1520P_BASE, sizeof(VIN_PIPE_ATTR_S));
			 break;
		 case SIF_TEST_PATTERN0_2160P:
			 memcpy(pstPipeAttr, &PIPE_ATTR_TEST_PATTERN_4K_BASE, sizeof(VIN_PIPE_ATTR_S));
			 break;
		 case S5KGM1SP_30FPS_4000x3000_RAW10:
			 memcpy(pstPipeAttr, &PIPE_ATTR_S5KGM1SP_LINEAR_BASE, sizeof(VIN_PIPE_ATTR_S));
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
		 case AR0233_30FPS_1080P_RAW12_954_PWL:
		 case AR0233_30FPS_1080P_RAW12_960_PWL:
		 case SIF_TEST_PATTERN0_1080P:
		 case FEED_BACK_RAW12_1952P:
			 memcpy(pstDisAttr, &DIS_ATTR_BASE, sizeof(VIN_DIS_ATTR_S));
			 break;
		 case SIF_TEST_PATTERN0_1520P:
		 	 memcpy(pstDisAttr, &DIS_ATTR_1520P_BASE, sizeof(VIN_DIS_ATTR_S));
			 break;
		 case OS8A10_25FPS_3840P_RAW10_LINEAR:
		 case OS8A10_25FPS_3840P_RAW10_DOL2:
		 case SIF_TEST_PATTERN0_2160P:
			 memcpy(pstDisAttr, &DIS_ATTR_OS8A10_BASE, sizeof(VIN_DIS_ATTR_S));
			 break;
		 case OV10635_30FPS_720p_954_YUV:
		 case OV10635_30FPS_720p_960_YUV:
		 case SIF_TEST_PATTERN_YUV_720P:
			 memcpy(pstDisAttr, &DIS_ATTR_OV10635_BASE, sizeof(VIN_DIS_ATTR_S));
			 break;
		case SIF_TEST_PATTERN_12M_RAW12:
		case S5KGM1SP_30FPS_4000x3000_RAW10:
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
		 case AR0233_30FPS_1080P_RAW12_954_PWL:
		 case AR0233_30FPS_1080P_RAW12_960_PWL:
		 case SIF_TEST_PATTERN0_1080P:
		 case FEED_BACK_RAW12_1952P:
			 memcpy(pstLdcAttr, &LDC_ATTR_BASE, sizeof(VIN_LDC_ATTR_S));
			 break;
		 case SIF_TEST_PATTERN0_1520P:
			 memcpy(pstLdcAttr, &LDC_ATTR_1520P_BASE, sizeof(VIN_LDC_ATTR_S));
			 break;
		 case OS8A10_25FPS_3840P_RAW10_LINEAR:
		 case OS8A10_25FPS_3840P_RAW10_DOL2:
		 case SIF_TEST_PATTERN0_2160P:
			 memcpy(pstLdcAttr, &LDC_ATTR_OS8A10_BASE, sizeof(VIN_LDC_ATTR_S));
			 break;
		 case OV10635_30FPS_720p_954_YUV:
		 case OV10635_30FPS_720p_960_YUV:
		 case SIF_TEST_PATTERN_YUV_720P:
			 memcpy(pstLdcAttr, &LDC_ATTR_OV10635_BASE, sizeof(VIN_LDC_ATTR_S));
			 break;
	     case SIF_TEST_PATTERN_12M_RAW12:
		 case S5KGM1SP_30FPS_4000x3000_RAW10:
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

// static int check_end(void)
// {
// 	time_t now = time(NULL);
// 	//printf("Time info :: now(%ld), end_time(%ld)
// 	// run time(%ld)!\n",now, end_time, run_time);
// 	return !(now > end_time && run_time > 0);
// }

// void parse_sensorid_func(char *optarg)
// {
// 	char *p = optarg, *d;
// 	int i = 0;
// 	printf("optarg %s\n", optarg);
// 	while (p && *p && i < MAX_SENSOR_NUM) {
// 		d = strchr(p, ',');
// 		if (d)
// 			*d = '\0';
// 		sensorId[i] = atoi(p);
// 		printf("i %d sensorId[i] %d===========\n", i, sensorId[i]);
// 		i++;
// 		p = (d) ? (d + 1) : NULL;
// 	}
// 	return;
// }

// void parse_mipiIdx_func(char *optarg)
// {
// 	char *p = optarg, *d;
// 	int i = 0;
// 	printf("optarg %s\n", optarg);
// 	while (p && *p && i < MAX_MIPIID_NUM) {
// 		d = strchr(p, ',');
// 		if (d)
// 			*d = '\0';
// 		mipiIdx[i] = atoi(p);
// 		printf("i %d mipiIdx[i] %d===========\n", i, mipiIdx[i]);
// 		i++;
// 		p = (d) ? (d + 1) : NULL;
// 	}
// 	return;
// }

// int prepare_user_buf(void *buf, uint32_t size)
// {
// 	int ret;
// 	hb_vio_buffer_t *buffer = (hb_vio_buffer_t*)buf;

// 	if (buffer == NULL)
// 		return -1;

// 	buffer->img_info.fd[0] = ion_open();

// 	ret  = ion_alloc_phy(size, &buffer->img_info.fd[0],
// 							&buffer->img_addr.addr[0], &buffer->img_addr.paddr[0]);
// 	if (ret) {
// 		printf("prepare user buf error\n");
// 		return ret;
// 	}
// 	printf("prepare user buf  vaddr = 0x%x paddr = 0x%x \n",
// 				buffer->img_addr.addr[0], buffer->img_addr.paddr[0]);
// 	return 0;
// }

// static void normal_buf_info_print(hb_vio_buffer_t * buf)
// {
// 	printf("normal pipe_id (%d)type(%d)frame_id(%d)buf_index(%d)w x h(%dx%d) data_type %d img_format %d\n",
// 		buf->img_info.pipeline_id,
// 		buf->img_info.data_type,
// 		buf->img_info.frame_id,
// 		buf->img_info.buf_index,
// 		buf->img_addr.width,
// 		buf->img_addr.height,
// 		buf->img_info.data_type,
// 		buf->img_info.img_format);
// }
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

VPS_PYM_CHN_ATTR_S g_pym_all_layer_attr = {
        .frame_id = 0,
        .ds_uv_bypass = 0,
        .ds_layer_en = 23,
        .us_layer_en = 0,
        .us_uv_bypass = 0,
        .timeout = 2000,
        .frameDepth = 6,
        .ds_info[1].factor = 0,
        .ds_info[1].roi_x = 0,
        .ds_info[1].roi_y = 0,
        .ds_info[1].roi_width = 3840,
        .ds_info[1].roi_height = 2160,
        .ds_info[2].factor = 0,
        .ds_info[2].roi_x = 0,
        .ds_info[2].roi_y = 0,
        .ds_info[2].roi_width = 0,
        .ds_info[2].roi_height = 0,
        .ds_info[3].factor = 0,
        .ds_info[3].roi_x = 0,
        .ds_info[3].roi_y = 0,
        .ds_info[3].roi_width = 0,
        .ds_info[3].roi_height = 0,
        .ds_info[5].factor = 32,		// 32
        .ds_info[5].roi_x = 0,
        .ds_info[5].roi_y = 0,
        .ds_info[5].roi_width = 1920,
        .ds_info[5].roi_height = 1080,
        .ds_info[6].factor = 56,
        .ds_info[6].roi_x = 300,
        .ds_info[6].roi_y = 0,
        .ds_info[6].roi_width = 1320,
        .ds_info[6].roi_height = 1080,
        .ds_info[7].factor = 56,
        .ds_info[7].roi_x = 300,
        .ds_info[7].roi_y = 0,
        .ds_info[7].roi_width = 1320,
        .ds_info[7].roi_height = 1080,
};

// static int sample_vps_gdc_init(uint32_t pipe_id, uint32_t channel)
// {
// 	int ret = 0;
// 	int bin_fd;
// 	int file_len = 0;
// 	char *gdc_bin = "./gdc_1080p.bin";
// 	int rotate_degree = 0;
// 	bin_fd = open(gdc_bin, O_RDWR | O_CREAT, 0644);
// 	if (bin_fd < 0) {
// 		printf("open %s failed !\n", gdc_bin);
// 		return bin_fd;
// 	}
// 	char *buf = NULL;
// 	file_len = lseek(bin_fd, 0, SEEK_END);
// 	lseek(bin_fd, 0, SEEK_SET);
// 	buf = malloc(file_len);
// 	if (buf == NULL) {
// 		printf("gdc bin buf malloc failed\n");
// 		return -1;
// 	}
// 	read(bin_fd, buf, file_len);
// 	usleep(10 * 1000);
// 	ret = HB_VPS_SetChnGdc(pipe_id, channel, buf, file_len, rotate_degree);
// 	if (ret) {
// 		printf("HB_VPS_SetChnGdc error!!!\n");
// 	} else {
// 		printf("HB_VPS_SetChnGdc ok: grp_id = %d\n", pipe_id);
// 	}
// 	// ret = HB_VPS_UpdateGdcSize(pipe_id, channel, 1920, 1080);
// 	// if (ret) {
// 	// 	printf("HB_VPS_UpdateGdcSize error!!!\n");
// 	// } else {
// 	// 	printf("HB_VPS_UpdateGdcSize ok: grp_id = %d\n", pipe_id);
// 	// }
// 	free(buf);
// 	close(bin_fd);
// 	return ret;
// }

int hb_vin_vps_init(int pipeId, int sensorId, int mipiIdx, uint32_t deseri_port, vencParam* vparam)
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
		goto devinfo_err;
	}
	pipeinfo = malloc(sizeof(VIN_PIPE_ATTR_S));
	if(pipeinfo == NULL) {
		printf("malloc error\n");
		goto devexinfo_err;
	}
	disinfo = malloc(sizeof(VIN_DIS_ATTR_S));
	if(disinfo == NULL) {
		printf("malloc error\n");
		goto pipeinfo_err;
	}
	ldcinfo = malloc(sizeof(VIN_LDC_ATTR_S));
	if(ldcinfo == NULL) {
		printf("malloc error\n");
		goto disinfo_err;
	}
	memset(devinfo, 0, sizeof(VIN_DEV_ATTR_S));
	memset(pipeinfo, 0, sizeof(VIN_PIPE_ATTR_S));
	memset(disinfo, 0, sizeof(VIN_DIS_ATTR_S));
	memset(ldcinfo, 0, sizeof(VIN_LDC_ATTR_S));
	SAMPLE_VIN_GetDevAttrBySns(sensorId, devinfo);
	SAMPLE_VIN_GetPipeAttrBySns(sensorId, pipeinfo);
	SAMPLE_VIN_GetDisAttrBySns(sensorId, disinfo);
	SAMPLE_VIN_GetLdcAttrBySns(sensorId, ldcinfo);
	SAMPLE_VIN_GetDevAttrExBySns(sensorId, devexinfo);
	print_sensor_dev_info(devinfo);
	print_sensor_pipe_info(pipeinfo);

	if (vin_vps_mode == VIN_ONLINE_VPS_OFFLINE) {
		pipeinfo->ddrOutBufNum = 3;
	}
	devinfo->outDdrAttr.buffer_num = 4;
	pipeinfo->ddrOutBufNum = 4;

	pipeinfo->calib.mode = g_calib > 0 ? 1 : 0;
	pipeinfo->temperMode = g_temperMode;
	if (g_frame_rate == 25)
		disinfo->disPath.rg_dis_enable = 1;
	else 
		disinfo->disPath.rg_dis_enable = 0;
	disinfo->disBufNum = 3;
	ret = HB_SYS_SetVINVPSMode(pipeId, vin_vps_mode);
	if(ret < 0) {
		printf("HB_SYS_SetVINVPSMode%d error!\n", vin_vps_mode);
		goto ldcinfo_err;
	}
	ret = HB_VIN_CreatePipe(pipeId, pipeinfo);   // isp init
	if(ret < 0) {
		printf("HB_MIPI_InitSensor error!\n");
		goto ldcinfo_err;
	}
	ret = HB_VIN_SetMipiBindDev(pipeId, mipiIdx);
	if(ret < 0) {
		printf("HB_VIN_SetMipiBindDev error!\n");
		goto pipe_err;
	}
	ret = HB_VIN_SetDevVCNumber(pipeId, deseri_port);
	if(ret < 0) {
		printf("HB_VIN_SetDevVCNumber error!\n");
		goto pipe_err;
	}
	if((sensorId == OS8A10_25FPS_3840P_RAW10_DOL2) ||
	   (sensorId == IMX327_30FPS_2228P_RAW12_DOL2)) {
		ret = HB_VIN_AddDevVCNumber(pipeId, vc_num);
		if(ret < 0) {
			printf("HB_VIN_AddDevVCNumber error!\n");
			goto pipe_err;
		}
	}
	ret = HB_VIN_SetDevAttr(pipeId, devinfo);     // sif init
	if(ret < 0) {
		printf("HB_MIPI_InitSensor error!\n");
		goto pipe_err;
	}
	// if(need_md) {
	// 	 ret = HB_VIN_SetDevAttrEx(pipeId, devexinfo);
	// 	if(ret < 0) {
	// 		printf("HB_VIN_SetDevAttrEx error!\n");
	// 		return ret;
	// 	}
	// }
	ret = HB_VIN_SetPipeAttr(pipeId, pipeinfo);     // isp init
	if(ret < 0) {
		printf("HB_VIN_SetPipeAttr error!\n");
		goto pipe_err;
	}
	ret = HB_VIN_SetChnDISAttr(pipeId, 1, disinfo);  //  dis init
	if(ret < 0) {
		printf("HB_VIN_SetChnDISAttr error!\n");
		goto pipe_err;
	}
	if(disinfo->disPath.rg_dis_enable == 1) {
		HB_VIN_RegisterDisCallback(pipeId, &pstDISCallback);
	}
	ret = HB_VIN_SetChnLDCAttr(pipeId, 1, ldcinfo);   //  ldc init
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
	grp_attr.maxW = pipeinfo->stSize.width;
	grp_attr.maxH = pipeinfo->stSize.height;
	grp_attr.frameDepth = g_vps_frame_depth;

	ret = HB_VPS_CreateGrp(pipeId, &grp_attr);
	if (ret) {
		printf("HB_VPS_CreateGrp error!!!\n");
	} else {
		printf("created a group ok:GrpId = %d, w:%d, h:%d\n",
			pipeId, grp_attr.maxW, grp_attr.maxH);
	}

	if (need_grp_rotate) {
		if ((sensorId == OS8A10_25FPS_3840P_RAW10_LINEAR) ||
            (sensorId == OS8A10_25FPS_3840P_RAW10_DOL2) ||
			(sensorId == SIF_TEST_PATTERN0_2160P)) {
			int img_in_fd = open("os8a10.bin", O_RDWR | O_CREAT, 0644);
			if (img_in_fd < 0) {
				printf("open image failed !\n");
				goto pipe_err;
			}
			char *buf = NULL;
			buf = malloc(200000);
			if (buf == NULL) {
				printf("gdc bin buf malloc failed\n");
				close(img_in_fd);
				goto pipe_err;
			}
			int binlen = read(img_in_fd, buf, 200000);
			usleep(10 * 1000);
			ret = HB_VPS_SetGrpGdc(pipeId, buf, binlen, ROTATION_0);
			if (ret) {
				printf("HB_VPS_SetGrpGdc error!!!\n");
			} else {
				printf("HB_VPS_SetGrpGdc ok: pipeId = %d\n", pipeId);
			}
			free(buf);
			close(img_in_fd);
		} else if ((sensorId == IMX327_30FPS_1952P_RAW12_LINEAR) ||
				   (sensorId == IMX327_30FPS_2228P_RAW12_DOL2) ||
				   (sensorId == SIF_TEST_PATTERN0_1080P)) {
			ret = HB_VPS_SetGrpRotate(pipeId, ROTATION_0);
			if (ret) {
				printf("HB_VPS_SetGrpRotate error!!!\n");
			} else {
				printf("HB_VPS_SetGrpRotate ok:GrpId = %d\n", pipeId);
			}
		}
	}

	if (g_use_ipu) {
		memset(&chn_attr, 0, sizeof(VPS_CHN_ATTR_S));
		chn_attr.enScale = 1;
		// chn_attr.frameDepth = 2;
		// if (vin_vps_mode == VIN_OFFLINE_VPS_OFFINE) {
		// 	chn_attr.frameDepth = 1;
		// } else {
		// 	chn_attr.frameDepth = 1;
		// }
		chn_attr.frameDepth = g_vps_frame_depth;

		for (int i=0; i<5; i++) {
			if (vparam[i].type == 0) continue;
			chn_attr.width = vparam[i].width;
			chn_attr.height = vparam[i].height;
			ret = HB_VPS_SetChnAttr(pipeId, vparam[i].vpsChn, &chn_attr);
			if (ret) {
				printf("HB_VPS_SetChnAttr error!!!\n");
			} else {
				printf("set chn Attr ok: GrpId = %d, chn_id = %d\n",
										pipeId, vparam[i].vpsChn);
			}
			HB_VPS_EnableChn(pipeId, vparam[i].vpsChn);

			if (vparam[i].vpsChn == 5 && (vparam[i].width == pipeinfo->stSize.width)) {
				VPS_CROP_INFO_S chn_crop_info;
				memset(&chn_crop_info, 0, sizeof(VPS_CROP_INFO_S));
				chn_crop_info.en = 1;
				chn_crop_info.cropRect.x = 0;
				chn_crop_info.cropRect.y = 0;
				chn_crop_info.cropRect.width = vparam[i].width;
				chn_crop_info.cropRect.height = vparam[i].height;
				ret = HB_VPS_SetChnCrop(pipeId, vparam[i].vpsChn, &chn_crop_info);
				if (ret) {
					printf("HB_VPS_SetChnCrop error!!!\n");
				} else {
					printf("HB_VPS_SetChnCrop ok: GrpId = %d, chn_id = %d\n",
						pipeId, vparam[i].vpsChn);
				}
			}
		}

		// sample_vps_gdc_init(pipeId, 1);
		// HB_VPS_SetChnRotate(pipeId, 1, ROTATE_180);
		// HB_VPS_EnableChn(pipeId, 1);

		chn_attr.width = 1920;
		chn_attr.height = 1080;
		chn_attr.frameDepth = g_vps_frame_depth;
		ret = HB_VPS_SetChnAttr(pipeId, 2, &chn_attr);
		if (ret) {
			printf("HB_VPS_SetChnAttr error!!!\n");
		} else {
			printf("set chn Attr ok: GrpId = %d, chn_id = %d\n",
									pipeId, 2);
		}
		HB_VPS_EnableChn(pipeId, 2);

		ret = HB_VPS_SetChnAttr(pipeId, 6, &chn_attr);
		if (ret) {
			printf("HB_VPS_SetChnAttr error!!!\n");
		} else {
			printf("set chn Attr ok: GrpId = %d, chn_id = %d\n",
									pipeId, 6);
		}
		// HB_VPS_EnableChn(pipeId, 6);
		memset(&pym_chn_attr, 0, sizeof(VPS_PYM_CHN_ATTR_S));
		pym_chn_attr.timeout = 2000;
		pym_chn_attr.ds_layer_en = 4;
		pym_chn_attr.us_layer_en = 0;
		pym_chn_attr.frame_id = 0;
		pym_chn_attr.frameDepth = g_vps_frame_depth;

		pym_chn_attr.ds_info[1].factor = 32,
		pym_chn_attr.ds_info[1].roi_x = 0,
		pym_chn_attr.ds_info[1].roi_y = 0,
		pym_chn_attr.ds_info[1].roi_width = 1920,
		pym_chn_attr.ds_info[1].roi_height = 1080,

		ret = HB_VPS_SetPymChnAttr(pipeId, 6, &pym_chn_attr);
		if (ret) {
			printf("HB_VPS_SetPymChnAttr error!!!\n");
		} else {
			printf("HB_VPS_SetPymChnAttr ok: grp_id = %d g_pym_chn = %d\n",
									pipeId, 6);
		}
		HB_VPS_EnableChn(pipeId, 6);
	} else {
		memset(&chn_attr, 0, sizeof(VPS_CHN_ATTR_S));
		chn_attr.enScale = 1;
		chn_attr.frameDepth = 6;

		// for (int i=0; i<5; i++) {
		// 	// if (vparam[i].type == 0) continue;
		// 	chn_attr.width = vparam[i].width;
		// 	chn_attr.height = vparam[i].height;
		// 	ret = HB_VPS_SetChnAttr(pipeId, vparam[i].vpsChn, &chn_attr);
		// 	if (ret) {
		// 		printf("HB_VPS_SetChnAttr error!!!\n");
		// 	} else {
		// 		printf("set chn Attr ok: GrpId = %d, chn_id = %d\n",
		// 								pipeId, vparam[i].vpsChn);
		// 	}
		// 	HB_VPS_EnableChn(pipeId, vparam[i].vpsChn);

		// 	if (vparam[i].vpsChn == 5 && (vparam[i].width == pipeinfo->stSize.width)) {
		// 		VPS_CROP_INFO_S chn_crop_info;
		// 		memset(&chn_crop_info, 0, sizeof(VPS_CROP_INFO_S));
		// 		chn_crop_info.en = 1;
		// 		chn_crop_info.cropRect.x = 0;
		// 		chn_crop_info.cropRect.y = 0;
		// 		chn_crop_info.cropRect.width = vparam[i].width;
		// 		chn_crop_info.cropRect.height = vparam[i].height;
		// 		ret = HB_VPS_SetChnCrop(pipeId, vparam[i].vpsChn, &chn_crop_info);
		// 		if (ret) {
		// 			printf("HB_VPS_SetChnCrop error!!!\n");
		// 		} else {
		// 			printf("HB_VPS_SetChnCrop ok: GrpId = %d, chn_id = %d\n",
		// 				pipeId, vparam[i].vpsChn);
		// 		}
		// 	}
		// }

		chn_attr.width = 3840;
		chn_attr.height = 2160;

		// ret = HB_VPS_SetChnAttr(pipeId, 2, &chn_attr);
		// if (ret) {
		// 	printf("HB_VPS_SetChnAttr error!!!\n");
		// } else {
		// 	printf("set chn Attr ok: GrpId = %d, chn_id = %d\n",
		// 							pipeId, 2);
		// }
		// HB_VPS_EnableChn(pipeId, 2);

		ret = HB_VPS_SetChnAttr(pipeId, 6, &chn_attr);
		if (ret) {
			printf("HB_VPS_SetChnAttr error!!!\n");
		} else {
			printf("set chn Attr ok: GrpId = %d, chn_id = %d\n",
									pipeId, 6);
		}
		// // HB_VPS_EnableChn(pipeId, 6);
		// memset(&pym_chn_attr, 0, sizeof(VPS_PYM_CHN_ATTR_S));
		// pym_chn_attr.timeout = 2000;
		// pym_chn_attr.ds_layer_en = 23;
		// pym_chn_attr.us_layer_en = 0;
		// pym_chn_attr.frame_id = 0;
		// pym_chn_attr.frameDepth = 6;
		// pym_chn_attr.ds_info[1].factor = 32,
		// pym_chn_attr.ds_info[1].roi_x = 0,
		// pym_chn_attr.ds_info[1].roi_y = 0,
		// pym_chn_attr.ds_info[1].roi_width = 3840,
		// pym_chn_attr.ds_info[1].roi_height = 2160,

		ret = HB_VPS_SetPymChnAttr(pipeId, 6, &g_pym_all_layer_attr);
		if (ret) {
			printf("HB_VPS_SetPymChnAttr error!!!\n");
		} else {
			printf("HB_VPS_SetPymChnAttr ok: grp_id = %d g_pym_chn = %d\n",
									pipeId, 6);
		}
		HB_VPS_EnableChn(pipeId, 6);
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
	goto ldcinfo_err;

pipe_err:
	HB_VIN_DestroyDev(pipeId);  // sif deinit
chn_err:
	HB_VIN_DestroyPipe(pipeId);  // isp && dwe deinit

ldcinfo_err:
	free(ldcinfo);
disinfo_err:
	free(disinfo);
pipeinfo_err:
	free(pipeinfo);
devexinfo_err:
	free(devexinfo);
devinfo_err:
	free(devinfo);
	return ret;
}

void hb_vin_vps_deinit(int pipeId, int sensorId)
{
	HB_VIN_DestroyDev(pipeId);  // sif deinit && destroy
	HB_VIN_DestroyChn(pipeId, 1);  // dwe deinit
	HB_VIN_DestroyPipe(pipeId);  // isp deinit && destroy
	HB_VPS_DestroyGrp(pipeId);
}

int hb_mode_init(int pipeId, uint32_t sensorId, uint32_t mipiIdx, uint32_t deseri_port, uint32_t vin_vps_mode)
{
	int ret = 0;
	VIN_DEV_ATTR_S *devinfo = NULL;
	VIN_PIPE_ATTR_S *pipeinfo = NULL;
	VIN_DIS_ATTR_S *disinfo = NULL;
	VIN_LDC_ATTR_S * ldcinfo = NULL;
	VIN_DEV_ATTR_EX_S *devexinfo = NULL;
	// VIN_DIS_CALLBACK_S pstDISCallback;
	// pstDISCallback.VIN_DIS_DATA_CB = dis_crop_set;

	devinfo = malloc(sizeof(VIN_DEV_ATTR_S));
	if(devinfo == NULL) {
		printf("malloc error\n");
		return -1;
	}
	devexinfo = malloc(sizeof(VIN_DEV_ATTR_EX_S));
	if(devexinfo == NULL) {
		printf("malloc error\n");
		goto devinfo_err;
	}
	pipeinfo = malloc(sizeof(VIN_PIPE_ATTR_S));
	if(pipeinfo == NULL) {
		printf("malloc error\n");
		goto devexinfo_err;
	}
	disinfo = malloc(sizeof(VIN_DIS_ATTR_S));
	if(disinfo == NULL) {
		printf("malloc error\n");
		goto pipeinfo_err;
	}
	ldcinfo = malloc(sizeof(VIN_LDC_ATTR_S));
	if(ldcinfo == NULL) {
		printf("malloc error\n");
		goto disinfo_err;
	}
	memset(devinfo, 0, sizeof(VIN_DEV_ATTR_S));
	memset(pipeinfo, 0, sizeof(VIN_PIPE_ATTR_S));
	memset(disinfo, 0, sizeof(VIN_DIS_ATTR_S));
	memset(ldcinfo, 0, sizeof(VIN_LDC_ATTR_S));
	SAMPLE_VIN_GetDevAttrBySns(sensorId, devinfo);
	SAMPLE_VIN_GetPipeAttrBySns(sensorId, pipeinfo);
	SAMPLE_VIN_GetDisAttrBySns(sensorId, disinfo);
	SAMPLE_VIN_GetLdcAttrBySns(sensorId, ldcinfo);
	SAMPLE_VIN_GetDevAttrExBySns(sensorId, devexinfo);
	print_sensor_dev_info(devinfo);
	print_sensor_pipe_info(pipeinfo);

	if (vin_vps_mode == VIN_ONLINE_VPS_OFFLINE) {
		pipeinfo->ddrOutBufNum = 3;
	}
	devinfo->outDdrAttr.buffer_num = 4;
	pipeinfo->ddrOutBufNum = 4;

	pipeinfo->calib.mode = g_calib > 0 ? 1 : 0;
	pipeinfo->temperMode = g_temperMode;

	ret = HB_SYS_SetVINVPSMode(pipeId, vin_vps_mode);
	if(ret < 0) {
		printf("HB_SYS_SetVINVPSMode%d error!\n", vin_vps_mode);
		goto ldcinfo_err;
	}
	ret = HB_VIN_CreatePipe(pipeId, pipeinfo);   // isp init
	if(ret < 0) {
		printf("HB_MIPI_InitSensor error!\n");
		goto ldcinfo_err;
	}
	// if(need_devclk) {
	// 	ret = HB_VIN_SetDevMclk(pipeId, devclk, vpuclk);   // isp init
	// 	if(ret < 0) {
	// 		printf("HB_MIPI_InitSensor error!\n");
	// 		return ret;
	// 	}
	// }
	ret = HB_VIN_SetMipiBindDev(pipeId, mipiIdx);
	if(ret < 0) {
		printf("HB_VIN_SetMipiBindDev error!\n");
		goto pipe_err;
	}
	ret = HB_VIN_SetDevVCNumber(pipeId, deseri_port);
	if(ret < 0) {
		printf("HB_VIN_SetDevVCNumber error!\n");
		goto pipe_err;
	}
	if((sensorId == OS8A10_25FPS_3840P_RAW10_DOL2) ||
	   (sensorId == IMX327_30FPS_2228P_RAW12_DOL2)) {
		printf("HB_VIN_AddDevVCNumber: %d\n", vc_num);
		ret = HB_VIN_AddDevVCNumber(pipeId, vc_num);
		if(ret < 0) {
			printf("HB_VIN_AddDevVCNumber error!\n");
			goto pipe_err;
		}
	}
	ret = HB_VIN_SetDevAttr(pipeId, devinfo);     // sif init
	if(ret < 0) {
		printf("HB_MIPI_InitSensor error!\n");
		goto pipe_err;
	}
	ret = HB_VIN_SetPipeAttr(pipeId, pipeinfo);     // isp init
	if(ret < 0) {
		printf("HB_VIN_SetPipeAttr error!\n");
		goto pipe_err;
	}
	ret = HB_VIN_SetChnDISAttr(pipeId, 1, disinfo);  //  dis init
	if(ret < 0) {
		printf("HB_VIN_SetChnDISAttr error!\n");
		goto pipe_err;
	}
	// if(need_dis) {
	// 	HB_VIN_RegisterDisCallback(pipeId, &pstDISCallback);
	// }
	ret = HB_VIN_SetChnLDCAttr(pipeId, 1, ldcinfo);   //  ldc init
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
	goto ldcinfo_err;

pipe_err:
	HB_VIN_DestroyDev(pipeId);  // sif deinit
chn_err:
	HB_VIN_DestroyPipe(pipeId);  // isp && dwe deinit
ldcinfo_err:
	free(ldcinfo);
disinfo_err:
	free(disinfo);
pipeinfo_err:
	free(pipeinfo);
devexinfo_err:
	free(devexinfo);
devinfo_err:
	free(devinfo);
	return ret;
}

int hb_mode_start(int pipeId)
{
	int ret = 0;

	ret = HB_VIN_EnableChn(pipeId,  0);  // dwe start
	if(ret < 0) {
		printf("HB_VIN_EnableChn error!\n");
		return ret;
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

void hb_mode_stop(int pipeId)
{
	HB_VIN_DisableDev(pipeId);    // thread stop && sif stop
	HB_VIN_StopPipe(pipeId);    // isp stop
	HB_VIN_DisableChn(pipeId, 1);   // dwe stop
}
void hb_mode_destroy(int pipeId)
{
	HB_VIN_DestroyDev(pipeId);  // sif deinit && destroy
	HB_VIN_DestroyChn(pipeId, 1);  // dwe deinit	
	HB_VIN_DestroyPipe(pipeId);  // isp deinit && destroy
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

    HB_MIPI_SetBus(snsinfo, bus);
    HB_MIPI_SetPort(snsinfo, port);
    HB_MIPI_SensorBindSerdes(snsinfo, sedres_index, sedres_port);
    HB_MIPI_SensorBindMipi(snsinfo,  mipiIdx);

	ret = HB_MIPI_InitSensor(devId, snsinfo);
	if(ret < 0) {
		printf("HB_MIPI_InitSensor error!\n");
	}
	free(snsinfo);
	printf("HB_MIPI_InitSensor end\n");
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

void hb_vps_stop(int pipeId)
{
	HB_VPS_StopGrp(pipeId);
}

int hb_vps_init(int pipeId)
{
	int ret = 0;

	VPS_GRP_ATTR_S grp_attr;
	VPS_CHN_ATTR_S chn_attr;

	memset(&grp_attr, 0, sizeof(VPS_GRP_ATTR_S));
	grp_attr.maxW = 3840;
	grp_attr.maxH = 2160;
	ret = HB_VPS_CreateGrp(pipeId, &grp_attr);
	if (ret) {
		printf("HB_VPS_CreateGrp error!!!\n");
	} else {
		printf("created a group ok:GrpId = %d\n", pipeId);
	}

	ret = HB_SYS_SetVINVPSMode(pipeId, VIN_OFFLINE_VPS_OFFINE);
	if(ret < 0) {
		printf("HB_SYS_SetVINVPSMode%d error!\n", VIN_OFFLINE_VPS_OFFINE);
		return ret;
	}

	memset(&chn_attr, 0, sizeof(VPS_CHN_ATTR_S));
	chn_attr.enScale = 1;
	chn_attr.width = 704;
	chn_attr.height = 576;
	chn_attr.frameDepth = 6;

	ret = HB_VPS_SetChnAttr(pipeId, 0, &chn_attr);
	if (ret) {
		printf("HB_VPS_SetChnAttr error!!!\n");
	} else {
		printf("set chn Attr ok: GrpId = %d, chn_id = %d\n",
								pipeId, 0);
	}
	HB_VPS_EnableChn(pipeId, 0);

	chn_attr.width = 3840;
	chn_attr.height = 2160;

	ret = HB_VPS_SetChnAttr(pipeId, 6, &chn_attr);
	if (ret) {
		printf("HB_VPS_SetChnAttr error!!!\n");
	} else {
		printf("set chn Attr ok: GrpId = %d, chn_id = %d\n", pipeId, 6);
	}

	ret = HB_VPS_SetPymChnAttr(pipeId, 6, &g_pym_all_layer_attr);
	if (ret) {
		printf("HB_VPS_SetPymChnAttr error!!!\n");
	} else {
		printf("HB_VPS_SetPymChnAttr ok: grp_id = %d g_pym_chn = %d\n",
								pipeId, 6);
	}
	HB_VPS_EnableChn(pipeId, 6);

	return ret;
}

void hb_vps_deinit(int pipeId)
{
	HB_VPS_DestroyGrp(pipeId);
}

int sample_vps_bind_vps(int vpsGrp, int vpsChn, int dstVpsGrp)
{
    int s32Ret = 0;
    struct HB_SYS_MOD_S src_mod, dst_mod;

	src_mod.enModId = HB_ID_VPS;
	src_mod.s32DevId = vpsGrp;
	src_mod.s32ChnId = vpsChn;
	dst_mod.enModId = HB_ID_VPS;
	dst_mod.s32DevId = dstVpsGrp;
	dst_mod.s32ChnId = 0;
	s32Ret = HB_SYS_Bind(&src_mod, &dst_mod);
	if (s32Ret != 0)
		printf("HB_SYS_Bind failed\n");

    return s32Ret;
}

int sample_vps_bind_vot(int vpsGrp, int vpsChn, int votChn)
{
    int s32Ret = 0;
    struct HB_SYS_MOD_S src_mod, dst_mod;

	src_mod.enModId = HB_ID_VPS;
	src_mod.s32DevId = vpsGrp;
	src_mod.s32ChnId = vpsChn;
	dst_mod.enModId = HB_ID_VOT;
	dst_mod.s32DevId = 0;
	dst_mod.s32ChnId = votChn;
	s32Ret = HB_SYS_Bind(&src_mod, &dst_mod);
	if (s32Ret != 0)
		printf("HB_SYS_Bind failed\n");

    return s32Ret;
}

int sample_vps_bind_venc(int vpsGrp, int vpsChn, int vencChn)
{
    int s32Ret = 0;
    struct HB_SYS_MOD_S src_mod, dst_mod;

	src_mod.enModId = HB_ID_VPS;
	src_mod.s32DevId = vpsGrp;
	src_mod.s32ChnId = vpsChn;
	dst_mod.enModId = HB_ID_VENC;
	dst_mod.s32DevId = vencChn;
	dst_mod.s32ChnId = 0;
	s32Ret = HB_SYS_Bind(&src_mod, &dst_mod);
	if (s32Ret != 0)
		printf("HB_SYS_Bind failed\n");

    return s32Ret;
}

int sample_vin_bind_vps(int pipeId, int vin_vps_mode)
{
	int ret = 0;
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

	return ret;
}

int sample_vps_unbind_vps(int vpsGrp, int vpsChn, int dstVpsGrp)
{
    int s32Ret = 0;
    struct HB_SYS_MOD_S src_mod, dst_mod;

	src_mod.enModId = HB_ID_VPS;
	src_mod.s32DevId = vpsGrp;
	src_mod.s32ChnId = vpsChn;
	dst_mod.enModId = HB_ID_VPS;
	dst_mod.s32DevId = dstVpsGrp;
	dst_mod.s32ChnId = 0;
	s32Ret = HB_SYS_UnBind(&src_mod, &dst_mod);
	if (s32Ret != 0)
		printf("HB_SYS_UnBind failed\n");

    return s32Ret;
}

int sample_vps_unbind_vot(int vpsGrp, int vpsChn, int votChn)
{
    int s32Ret = 0;
    struct HB_SYS_MOD_S src_mod, dst_mod;

	src_mod.enModId = HB_ID_VPS;
	src_mod.s32DevId = vpsGrp;
	src_mod.s32ChnId = vpsChn;
	dst_mod.enModId = HB_ID_VOT;
	dst_mod.s32DevId = 0;
	dst_mod.s32ChnId = votChn;
	s32Ret = HB_SYS_UnBind(&src_mod, &dst_mod);
	if (s32Ret != 0)
		printf("HB_SYS_UnBind failed\n");

    return s32Ret;
}

int sample_vps_unbind_venc(int vpsGrp, int vpsChn, int vencChn)
{
    int s32Ret = 0;
    struct HB_SYS_MOD_S src_mod, dst_mod;

	src_mod.enModId = HB_ID_VPS;
	src_mod.s32DevId = vpsGrp;
	src_mod.s32ChnId = vpsChn;
	dst_mod.enModId = HB_ID_VENC;
	dst_mod.s32DevId = vencChn;
	dst_mod.s32ChnId = 0;
	s32Ret = HB_SYS_UnBind(&src_mod, &dst_mod);
	if (s32Ret != 0)
		printf("HB_SYS_UnBind failed\n");

    return s32Ret;
}

int sample_vin_unbind_vps(int pipeId, int vin_vps_mode)
{
	int ret = 0;
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
	ret = HB_SYS_UnBind(&src_mod, &dst_mod);
	if (ret != 0)
		printf("HB_SYS_UnBind failed\n");

	return ret;
}

int hb_vps_init_param(int pipeId, int grpwidth, int grpheight, vencParam* vparam)
{
	int ret = 0;

	VPS_GRP_ATTR_S grp_attr;
	VPS_CHN_ATTR_S chn_attr;
	VPS_PYM_CHN_ATTR_S pym_chn_attr;

	memset(&grp_attr, 0, sizeof(VPS_GRP_ATTR_S));
	grp_attr.maxW = grpwidth;
	grp_attr.maxH = grpheight;
	ret = HB_VPS_CreateGrp(pipeId, &grp_attr);
	if (ret) {
		printf("HB_VPS_CreateGrp error!!!\n");
	} else {
		printf("created a group ok:GrpId = %d, w:%d, h:%d\n",
			pipeId, grp_attr.maxW, grp_attr.maxH);
	}

	memset(&chn_attr, 0, sizeof(VPS_CHN_ATTR_S));
	chn_attr.enScale = 1;
	chn_attr.frameDepth = 6;

	for (int i=0; i<5; i++) {
		// if (vparam[i].type == 0) continue;
		chn_attr.width = vparam[i].width;
		chn_attr.height = vparam[i].height;
		ret = HB_VPS_SetChnAttr(pipeId, vparam[i].vpsChn, &chn_attr);
		if (ret) {
			printf("HB_VPS_SetChnAttr error!!!\n");
		} else {
			printf("set chn Attr ok: GrpId = %d, chn_id = %d, %dx%d\n",
				pipeId, vparam[i].vpsChn, chn_attr.width, chn_attr.height);
		}
		HB_VPS_EnableChn(pipeId, vparam[i].vpsChn);

		if (vparam[i].vpsChn == 5 && (vparam[i].width == grpwidth)) {
			VPS_CROP_INFO_S chn_crop_info;
			memset(&chn_crop_info, 0, sizeof(VPS_CROP_INFO_S));
			chn_crop_info.en = 1;
			chn_crop_info.cropRect.x = 0;
			chn_crop_info.cropRect.y = 0;
			chn_crop_info.cropRect.width = vparam[i].width;
			chn_crop_info.cropRect.height = vparam[i].height;
			ret = HB_VPS_SetChnCrop(pipeId, vparam[i].vpsChn, &chn_crop_info);
			if (ret) {
				printf("HB_VPS_SetChnCrop error!!!\n");
			} else {
				printf("HB_VPS_SetChnCrop ok: GrpId = %d, chn_id = %d\n",
					pipeId, vparam[i].vpsChn);
			}
		}
	}

	chn_attr.width = 1920;
	chn_attr.height = 1080;

	ret = HB_VPS_SetChnAttr(pipeId, 2, &chn_attr);
	if (ret) {
		printf("HB_VPS_SetChnAttr error!!!\n");
	} else {
		printf("set chn Attr ok: GrpId = %d, chn_id = %d\n",
								pipeId, 2);
	}
	HB_VPS_EnableChn(pipeId, 2);

	ret = HB_VPS_SetChnAttr(pipeId, 6, &chn_attr);
	if (ret) {
		printf("HB_VPS_SetChnAttr error!!!\n");
	} else {
		printf("set chn Attr ok: GrpId = %d, chn_id = %d\n",
								pipeId, 6);
	}
	// // HB_VPS_EnableChn(pipeId, 6);
	memset(&pym_chn_attr, 0, sizeof(VPS_PYM_CHN_ATTR_S));
	pym_chn_attr.timeout = 2000;
	pym_chn_attr.ds_layer_en = 23;
	pym_chn_attr.us_layer_en = 0;
	pym_chn_attr.frame_id = 0;
	pym_chn_attr.frameDepth = 6;
	pym_chn_attr.ds_info[1].factor = 32,
    pym_chn_attr.ds_info[1].roi_x = 0,
    pym_chn_attr.ds_info[1].roi_y = 0,
    pym_chn_attr.ds_info[1].roi_width = 1920,
    pym_chn_attr.ds_info[1].roi_height = 1080,

	ret = HB_VPS_SetPymChnAttr(pipeId, 6, &pym_chn_attr);
	if (ret) {
		printf("HB_VPS_SetPymChnAttr error!!!\n");
	} else {
		printf("HB_VPS_SetPymChnAttr ok: grp_id = %d g_pym_chn = %d\n",
								pipeId, 6);
	}
	HB_VPS_EnableChn(pipeId, 6);


#if 0
	common_bind_input(g_vps[GrpId]->input_channel[0],
							&g_vin[GrpId]->sif_isp.base, 0);
	common_bind_output(g_vin[GrpId]->sif_isp.base.output_channel[0],
							&g_vps[GrpId]->ipu.base, 0);
#else
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

#endif

	return ret;
}

int sample_vps_chn_reconfig(int vpsgrp, int vpschn, int width, int height)
{
	int ret;
	VPS_CHN_ATTR_S chn_attr;

	ret = HB_VPS_GetChnAttr(vpsgrp, vpschn, &chn_attr);
	if (ret) {
		printf("HB_VPS_GetChnAttr error %x!!!\n", ret);
		return ret;
	}

	printf("w: %d, h: %d\n", chn_attr.width, chn_attr.height);

	chn_attr.width = width;
	chn_attr.height = height;
	ret = HB_VPS_SetChnAttr(vpsgrp, vpschn, &chn_attr);
	if (ret) {
		printf("HB_VPS_SetChnAttr error!!!\n");
	} else {
		printf("set chn Attr ok: GrpId = %d, chn_id = %d\n",
								vpsgrp, vpschn);
	}

	return ret;
}
