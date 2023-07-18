/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * hb_video.c
 *	hobot video module functions(sensor->isp->vio->venc)
 *
 * Copyright (C) 2019 Horizon Robotics, Inc.
 *
 * Contact: jianghe xu<jianghe.xu@horizon.ai>
 */

#include <unistd.h>
#include <stdlib.h>
#include <string.h>

#include <linux/videodev2.h>

#include "hb_video.h"
#include "utils.h"
#include "config.h"

/******************************************
 *      Define Implementation             *
 *****************************************/
#define BIT2CHN(chns, chn) (chns & (1 << chn))
#define BIT(n)  (1UL << (n))

#define result_check(r) \
	if (r < 0) { \
		printf("error: %s - %d, ret(%d)\n", __func__, __LINE__, r); \
		return r; \
	}

/* below h265 definition in kernel include/uapi/linux/videodev2.h,
 * but user app use toolchain's include file
 */
#define V4L2_PIX_FMT_H265     v4l2_fourcc('H', '2', '6', '5') /* H265 with start codes */

/******************************************
 *        Global Variable Definition      *
 *****************************************/
/*#################### VIN_DEV_ATTR_S ##################*/
/* test pattern 1080p */
static const VIN_DEV_ATTR_S vindev_attr_test_pattern_1080p = {
	.stSize = {0,		/* format */
		   1920,	/* width */
		   1080,	/* height */
		   2		/* pix_length */
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

/* test pattern 12m8m */
static const VIN_DEV_ATTR_S vindev_attr_test_pattern_12m8m = {
	.stSize = {0,		/* format */
		   4000,	/* width */
		   3000,	/* height */
		   2		/* pix_length */
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

/* test pattern 8m(4k) */
static const VIN_DEV_ATTR_S vindev_attr_test_pattern_8m = {
	.stSize = {0,		/* format */
		   3840,	/* width */
		   2160,	/* height */
		   1		/* pix_length */
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

/* imx327 1952@30p raw12 linear */
static const VIN_DEV_ATTR_S vindev_attr_imx327_linear = {
	.stSize = {0,		/* format */
		   1952,	/* width */
		   1097,	/* height */
		   2		/* pix_length */
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

/* ov10635 720@30p 954/960 yuv */
static const VIN_DEV_ATTR_S vindev_attr_ov10635_yuv = {
	.stSize = {8,		/* format */
		   1280,	/* width */
		   720,		/* height */
		   0		/* pix_length */
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

/* os8a10 3840@30p raw10 linear */
static const VIN_DEV_ATTR_S vindev_attr_os8a10_linear = {
	.stSize = {0,		/* format */
		   3840,	/* width */
		   2160,	/* height */
		   1		/* pix_length */
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

/* ar0144 720@30fps raw12 mono */
static const VIN_DEV_ATTR_S vindev_attr_ar0144_720p = {
	.stSize = {0,		/* format */
		   1280,	/* width */
		   720,		/* height */
		   2		/* pix_length */
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
				.width = 1280,
				.height = 720,
				.pix_length = 2,
				}
		       },
	.outDdrAttr = {
		       .stride = 1920,
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

/* s5kgm1sp 4000x3000@30p raw 19 */
static const VIN_DEV_ATTR_S vindev_attr_s5kgm1sp_linear = {
	.stSize = {0,		/* format */
		   4000,	/* width */
		   3000,	/* height */
		   1		/* pix_length */
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
		       .stride = 5008,
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

/* gc02m1b 1600x1200@25p raw8 */
static const VIN_DEV_ATTR_S vindev_attr_gc02m1b_linear = {
	.stSize = {0,		/* format */
		   1600,	/* width */
		   1200,	/* height */
		   0		/* pix_length */
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
				.width = 1600,
				.height = 1200,
				.pix_length = 0,
				}
		       },
	.outDdrAttr = {
		       .stride = 1600,
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

/* f37 1920x1080@30p raw10 */
static const VIN_DEV_ATTR_S vindev_attr_f37_linear = {
	.stSize = {0,		/* format */
		   1920,	/* width */
		   1080,	/* height */
		   0		/* pix_length */
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
				.pix_length = 1,
				}
		       },
	.outDdrAttr = {
		       .stride = 2400,
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

/* imx307 1080@30p raw12 linear */
static const VIN_DEV_ATTR_S vindev_attr_imx307_linear = {
	.stSize = {0,		/* format */
		   1920,	/* width */
		   1080,	/* height */
		   2		/* pix_length */
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
		       .buffer_num = 10,
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

/* ar0233 1080@30p raw12 1080p */
static const VIN_DEV_ATTR_S vindev_attr_ar0233_1080p_base = {
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

/*#################### VIN_PIPE_ATR_S ##################*/
/* test pattern 1080p for pipe attr */
static const VIN_PIPE_ATTR_S vinpipe_attr_test_pattern_1080p = {
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
};

/* test pattern 12m for pipe attr */
static const VIN_PIPE_ATTR_S vinpipe_attr_test_pattern_12m = {
	.ddrOutBufNum = 6,
	.snsMode = SENSOR_NORMAL_MODE,
	.stSize = {
		   .format = 0,
		   .width = 4000,
		   .height = 3000,
		   },
	.temperMode = 2,
	.ispBypassEn = 0,
	.ispAlgoState = 1,
	.bitwidth = 12,
};

/* test pattern 8m for pipe attr */
static const VIN_PIPE_ATTR_S vinpipe_attr_test_pattern_8m = {
	.ddrOutBufNum = 6,
	.snsMode = SENSOR_NORMAL_MODE,
	.stSize = {
		   .format = 0,
		   .width = 3840,
		   .height = 2160,
		   },
	.temperMode = 2,
	.ispBypassEn = 0,
	.ispAlgoState = 1,
	.bitwidth = 12,
};

/* test pattern 4k for pipe attr */
static const VIN_PIPE_ATTR_S vinpipe_attr_test_pattern_4k = {
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

/* imx327 1952@30p raw12 linear for pipe attr */
static const VIN_PIPE_ATTR_S vinpipe_attr_imx327_linear = {
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

/* ov10635 720@30p 954/960 yuv pipe attr */
static const VIN_PIPE_ATTR_S vinpipe_attr_ov10635_yuv = {
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

/* os8a10 3840@30p raw10 linear for pipe attr */
static const VIN_PIPE_ATTR_S vinpipe_attr_os8a10_linear = {
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


/* s5kgm1sp 4000x3000@30p raw 19 for pipe attr */
static const VIN_PIPE_ATTR_S vinpipe_attr_s5kgm1sp_linear = {
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

/* s5kgm1sp 4000x3000@30p raw 19 for pipe attr */
static const VIN_PIPE_ATTR_S vinpipe_attr_gc02m1b_linear = {
	.ddrOutBufNum = 6,
	.snsMode = SENSOR_NORMAL_MODE,
	.stSize = {
		   .format = 0,
		   .width = 1600,
		   .height = 1200,
		   },
	.cfaPattern = 3,
	.temperMode = 2,
	.ispBypassEn = 0,
	.ispAlgoState = 1,
	.bitwidth = 8,
	.calib = {
		  .mode = 0,
		  .lname = "lib_imx327.so",
		  }
};

/* f37 1920x1080@30p raw 10 for pipe attr */
static const VIN_PIPE_ATTR_S vinpipe_attr_f37_linear = {
	.ddrOutBufNum = 8,
	.snsMode = SENSOR_NORMAL_MODE,
	.stSize = {
		   .format = 0,
		   .width = 1920,
		   .height = 1080,
		   },
	.cfaPattern = 1,	// PIPE_BAYER_GRBG
	.temperMode = 2,
	.ispBypassEn = 0,
	.ispAlgoState = 1,
	.bitwidth = 10,
	.calib = {
		  .mode = 1,
		  .lname = "libjxf37_linear.so",
		  }
};

/* imx307 1080@30p raw12 linear for pipe attr */
static const VIN_PIPE_ATTR_S vinpipe_attr_imx307_linear = {
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
		  .lname = "libimx307_linear.so",
		  }
};

/* ar0233 1080@30p raw12 pwl for pipe attr */
static const VIN_PIPE_ATTR_S vinpipe_attr_ar0233_1080p_pwl = {
	.ddrOutBufNum = 6,
	.snsMode = NORMAL_M,
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
		.lname = "lib_ar0233_linear.so",
	}
};

/*#################### VIN_DIS_ATTR_S ##################*/
static const VIN_DIS_ATTR_S vindis_attr_2m = {
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

static const VIN_DIS_ATTR_S vindis_attr_1k = {
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

static const VIN_DIS_ATTR_S vindis_attr_12m = {
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

static const VIN_DIS_ATTR_S vindis_attr_8m = {
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

static const VIN_DIS_ATTR_S vindis_attr_ov10635 = {
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

static const VIN_DIS_ATTR_S vindis_attr_os8a10 = {
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

static const VIN_DIS_ATTR_S vindis_attr_gc02m1b = {
	.picSize = {
		    .pic_w = 1599,
		    .pic_h = 1199,
		    },
	.disPath = {
		    .rg_dis_enable = 0,
		    .rg_dis_path_sel = 1,
		    },
	.disHratio = 65536,
	.disVratio = 65536,
	.xCrop = {
		  .rg_dis_start = 0,
		  .rg_dis_end = 1599,
		  },
	.yCrop = {
		  .rg_dis_start = 0,
		  .rg_dis_end = 1199,
		  }
};

static const VIN_DIS_ATTR_S vindis_attr_f37 = {
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

/*#################### VIN_LDC_ATTR_S ##################*/
static const VIN_LDC_ATTR_S ldc_attr_1k = {
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

static const VIN_LDC_ATTR_S ldc_attr_12m = {
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

static const VIN_LDC_ATTR_S ldc_attr_os8a10 = {
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
		 .rg_length = 3839,
		 },
	.yWoi = {
		 .rg_start = 0,
		 .rg_length = 2159,
		 }
};

static const VIN_LDC_ATTR_S ldc_attr_2m = {
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

static const VIN_LDC_ATTR_S ldc_attr_8m = {
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
		 .rg_length = 3839,
		 },
	.yWoi = {
		 .rg_start = 0,
		 .rg_length = 2159,
		 }
};

static const VIN_LDC_ATTR_S ldc_attr_ov10635 = {
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

static const VIN_LDC_ATTR_S ldc_attr_gc02m1b = {
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
		    .pic_w = 1599,
		    .pic_h = 1199,
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
		 .rg_length = 1599,
		 },
	.yWoi = {
		 .rg_start = 0,
		 .rg_length = 1199,
		 }
};

static const VIN_LDC_ATTR_S ldc_attr_f37 = {
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

/*#################### MIPI_ATTR_S ##################*/
static const MIPI_ATTR_S mipi_4lane_sensor_imx327_30fps_12bit_normal_attr = {
	.mipi_host_cfg = {
			  4,	/* lane */
			  0x2c,	/* datatype */
			  24,	/* mclk        */
			  891,	/* mipiclk */
			  30,	/* fps */
			  1952,	/* width      */
			  1097,	/* height */
			  2152,	/* linlength */
			  1150,	/* framelength */
			  20	/* settle */
			  },
	.dev_enable = 0		/*  mipi dev enable */
};

static const MIPI_ATTR_S mipi_sensor_os8a10_30fps_10bit_linear_attr = {
	.mipi_host_cfg = {
			  4,	/* lane */
			  0x2b,	/* datatype */
			  24,	/* mclk        */
			  1440,	/* mipiclk */
			  30,	/* fps */
			  3840,	/* width      */
			  2160,	/* height */
			  6326,	/* linlength */
			  4474,	/* framelength */
			  50,	/* settle */
			  4,	/*chnnal_num */
			  {0, 1, 2, 3}	/*vc */
			  },
	.dev_enable = 0		/*  mipi dev enable */
};

#if 0
static const MIPI_ATTR_S mipi_sensor_os8a10_30fps_10bit_linear_sensor_clk_attr = {
	.mipi_host_cfg = {
			  4,	/* lane */
			  0x2b,	/* datatype */
			  2400,	/* mclk        */
			  1440,	/* mipiclk */
			  30,	/* fps */
			  3840,	/* width  */
			  2160,	/* height */
			  6326,	/* linlength */
			  4474,	/* framelength */
			  50,	/* settle */
			  4,	/*chnnal_num */
			  {0, 1, 2, 3}	/*vc */
			  },
	.dev_enable = 0		/*  mipi dev enable */
};
#endif

static const MIPI_ATTR_S mipi_2lane_ov10635_30fps_yuv_720p_954_attr = {
	.mipi_host_cfg = {
			  2,	/* lane */
			  0x1e,	/* datatype */
			  24,	/* mclk        */
			  1600,	/* mipiclk */
			  30,	/* fps */
			  1280,	/* width  */
			  720,	/* height */
			  3207,	/* linlength */
			  748,	/* framelength */
			  30,	/* settle */
			  4,
			  {0, 1, 2, 3}
			  },
	.dev_enable = 0		/*  mipi dev enable */
};

static const MIPI_ATTR_S mipi_2lane_ov10635_30fps_yuv_720p_960_attr = {
	.mipi_host_cfg = {
			  2,	/* lane */
			  0x1e,	/* datatype */
			  24,	/* mclk    */
			  3200,	/* mipiclk */
			  30,	/* fps */
			  1280,	/* width  */
			  720,	/* height */
			  3207,	/* linlength */
			  748,	/* framelength */
			  30,	/* settle */
			  4,
			  {0, 1, 2, 3}
			  },
	.dev_enable = 0		/*  mipi dev enable */
};

static const MIPI_ATTR_S mipi_4lane_sensor_ar0144_30fps_12bit_720p_954_attr = {
	.mipi_host_cfg = {
			  4,	/* lane */
			  0x2c,	/* datatype */
			  24,	/* mclk        */
			  1600,	/* mipiclk */
			  30,	/* fps */
			  1280,	/* width      */
			  720,	/* height */
			  1488,	/* linlength */
			  1600,	/* framelength */
			  30,	/* settle */
			  4,
			  {0, 1, 2, 3}
			  },
	.dev_enable = 0		/*  mipi dev enable */
};

static const MIPI_ATTR_S mipi_sensor_s5kgm1sp_30fps_10bit_linear_attr = {
	.mipi_host_cfg = {
			  4,	/* lane */
			  0x2b,	/* datatype */
			  24,	/* mclk        */
			  4600,	/* mipiclk */
			  30,	/* fps */
			  4000,	/* width      */
			  3000,	/* height */
			  5024,	/* linlength */
			  3194,	/* framelength */
			  30,	/* settle */
			  2,	/*chnnal_num */
			  {0, 1}	/*vc */
			  },
	.dev_enable = 0		/*  mipi dev enable */
};

static const MIPI_ATTR_S mipi_sensor_gc02m1b_25fps_8bit_linear_attr = {
	.mipi_host_cfg = {
			  1,	/* lane */
			  0x2a,	/* datatype */
			  24,	/* mclk        */
			  672,	/* mipiclk */
			  25,	/* fps */
			  1600,	/* width      */
			  1200,	/* height */
			  1700,	/* linlength */
			  1300,	/* framelength */
			  20,	/* settle */
			  1,	/*chnnal_num */
			  {0, 1}	/*vc */
			  },
	.dev_enable = 0		/*  mipi dev enable */
};

static const MIPI_ATTR_S mipi_sensor_f37_30fps_10bit_linear_attr = {
	.mipi_host_cfg = {
			  1,	/* lane */
			  0x2b,	/* datatype */
			  24,	/* mclk        */
			  864,	/* mipiclk */
			  30,	/* fps */
			  1920,	/* width      */
			  1080,	/* height */
			  2560,	/* linlength */
			  1125,	/* framelength */
			  20,	/* settle */
			  4,	/*chnnal_num */
			  {0, 1, 2, 3}	/*vc */
			  },
	.dev_enable = 0		/*  mipi dev enable */
};

static const MIPI_ATTR_S mipi_sensor_f37_30fps_10bit_linear_clk_attr = {
	.mipi_host_cfg = {
			  1,	/* lane */
			  0x2b,	/* datatype */
			  2400,	/* mclk        */
			  864,	/* mipiclk */
			  30,	/* fps */
			  1920,	/* width      */
			  1080,	/* height */
			  2560,	/* linlength */
			  1125,	/* framelength */
			  20,	/* settle */
			  4,	/*chnnal_num */
			  {0, 1, 2, 3}	/*vc */
			  },
	.dev_enable = 0		/*  mipi dev enable */
};

static const MIPI_ATTR_S mipi_4lane_sensor_imx307_30fps_12bit_normal_attr = {
	.mipi_host_cfg = {
			  2,	/* lane */
			  0x2c,	/* datatype */
			  3715,	/* mclk        */
			  446,	/* mipiclk */
			  30,	/* fps */
			  1920,	/* width      */
			  1080,	/* height */
			  2200,	/* linlength */
			  1125,	/* framelength */
			  20	/* settle */
			  },
	.dev_enable = 0		/*  mipi dev enable */
};

static const MIPI_ATTR_S mipi_4lane_sensor_ar0233_30fps_12bit_1080p_954_attr = {
	.mipi_host_cfg =
	{
		4,		  /* lane */
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
		{0, 1, 2, 3},
	},
	.dev_enable = 0  /*  mipi dev enable */
};

/*#################### MIPI_SENSOR_INFO_S ##################*/
static const MIPI_SENSOR_INFO_S sensor_testpattern_info = {
	.sensorInfo = {
		       .sensor_name = "virtual",
		       }
};

static  const MIPI_SENSOR_INFO_S sensor_os8a10_30fps_10bit_linear_info =
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

static const MIPI_SENSOR_INFO_S sensor_4lane_imx327_30fps_12bit_linear_info =
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

static const MIPI_SENSOR_INFO_S sensor_2lane_ov10635_30fps_yuv_720p_954_info = {
	.deseEnable = 1,
	.inputMode = INPUT_MODE_MIPI,
	.deserialInfo = {
			 .bus_type = 0,
			 .bus_num = 4,
			 .deserial_addr = 0x3d,
			 .deserial_name = "s954"},
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
		       .deserial_port = 0}
};

static const MIPI_SENSOR_INFO_S sensor_2lane_ov10635_30fps_yuv_720p_960_info = {
	.deseEnable = 1,
	.inputMode = INPUT_MODE_MIPI,
	.deserialInfo = {
			 .bus_type = 0,
			 .bus_num = 4,
			 .deserial_addr = 0x30,
			 .deserial_name = "s960"},
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
		       .deserial_port = 0}
};

static const MIPI_SENSOR_INFO_S sensor_4lane_ar0144_30fps_12bit_720p_954_info = {
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
		       .resolution = 720,
		       .sensor_addr = 0x10,
		       .serial_addr = 0x18,
		       .entry_index = 1,
		       .sensor_mode = NORMAL_M,
		       .reg_width = 16,
		       .sensor_name = "ar0144AT",
		       .deserial_index = 0,
		       .deserial_port = 0}
};

static const MIPI_SENSOR_INFO_S sensor_s5kgm1sp_30fps_10bit_linear_info = {
	.deseEnable = 0,
	.inputMode = INPUT_MODE_MIPI,
	.sensorInfo = {
		       .port = 0,
		       .dev_port = 0,
		       .bus_type = 0,
		       .bus_num = 4,
		       .fps = 30,
		       .resolution = 3000,
		       .sensor_addr = 0x10,
		       .serial_addr = 0,
		       .entry_index = 1,
		       .sensor_mode = NORMAL_M,
		       .reg_width = 16,
		       .sensor_name = "s5kgm1sp",
		       .deserial_index = -1,
		       .deserial_port = 0}
};

static const MIPI_SENSOR_INFO_S sensor_gc02m1b_25fps_8bit_linear_info = {
	.deseEnable = 0,
	.inputMode = INPUT_MODE_MIPI,
	.sensorInfo = {
		       .port = 0,
		       .dev_port = 0,
		       .bus_type = 0,
		       .bus_num = 5,
		       .fps = 25,
		       .resolution = 1200,
		       .sensor_addr = 0x37,
		       .serial_addr = 0,
		       .entry_index = 0,
		       .sensor_mode = NORMAL_M,
		       .reg_width = 8,
		       .sensor_name = "gc02m1b",
		       .deserial_index = -1,
		       .deserial_port = 0}
};

static const MIPI_SENSOR_INFO_S sensor_f37_30fps_10bit_linear_info = {
	.deseEnable = 0,
	.inputMode = INPUT_MODE_MIPI,
	.sensorInfo = {
		       /* port,dev_port,bus_type, bus_num, fps, resolution, sensor_addr, serial_addr */
		       0, 0, 0, 5, 30, 1080, 0x40, 0,
		       /* entry_index, mode, reg_width, sensor_name, extra_mode */
		       0, 1, 8, "f37", 0,
		       /* deserial_index deserial_port */
		       -1, 0,
		       0, 0, 0	/* spi_info */
		       }
};

static const MIPI_SENSOR_INFO_S sensor_4lane_imx307_30fps_12bit_linear_info = {
	.deseEnable = 0,
	.inputMode = INPUT_MODE_MIPI,
	.sensorInfo = {
		.bus_num = 5,
		.fps = 30,
		.resolution = 1080,
		.sensor_addr = 0x1a,
		.entry_index = 1,
		.sensor_mode = 1,
		.reg_width = 16,
		.sensor_name = "imx307",
		.deserial_index = -1,
		.deserial_port = 0
	}
};

static const MIPI_SENSOR_INFO_S sensor_4lane_ar0233_30fps_12bit_1080p_954_pwl_info = {
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

/* sensor struct list */
static const vin_attr_info vin_attrs[SENOSR_MAX] = {
	{SENSOR_SIF_TEST_PATTERN0_1080P,		// 0
		&vindev_attr_test_pattern_1080p,
		&vinpipe_attr_test_pattern_1080p,
		&vindis_attr_2m,
		&ldc_attr_2m,
		NULL,
		&sensor_testpattern_info},
	{SENSOR_SIF_TEST_PATTERN_12M_RAW12_8M,		// 1
		&vindev_attr_test_pattern_12m8m,
		&vinpipe_attr_test_pattern_8m,
		&vindis_attr_8m,
		&ldc_attr_8m,
		NULL,
		&sensor_testpattern_info},
	{SENSOR_SIF_TEST_PATTERN_12M_RAW12_12M,		// 2
		&vindev_attr_test_pattern_12m8m,
		&vinpipe_attr_test_pattern_12m,
		&vindis_attr_12m,
		&ldc_attr_12m,
		NULL,
		&sensor_testpattern_info},
	{SENSOR_SIF_TEST_PATTERN_8M_RAW12,		// 3
		&vindev_attr_test_pattern_8m,
		&vinpipe_attr_test_pattern_4k,
		&vindis_attr_8m,
		&ldc_attr_8m,
		NULL,
		&sensor_testpattern_info},
	{SENSOR_IMX327_30FPS_1952P_RAW12_LINEAR,	// 4
		&vindev_attr_imx327_linear,
		&vinpipe_attr_imx327_linear,
		&vindis_attr_1k,
		&ldc_attr_1k,
		&mipi_4lane_sensor_imx327_30fps_12bit_normal_attr,
		&sensor_4lane_imx327_30fps_12bit_linear_info},
	{SENSOR_OS8A10_30FPS_3840P_RAW10_LINEAR,	// 5
		&vindev_attr_os8a10_linear,
		&vinpipe_attr_os8a10_linear,
		&vindis_attr_os8a10,
		&ldc_attr_os8a10,
		&mipi_sensor_os8a10_30fps_10bit_linear_attr,
		&sensor_os8a10_30fps_10bit_linear_info},
	{SENSOR_OV10635_30FPS_720p_954_YUV,		// 6
		&vindev_attr_ov10635_yuv,
		&vinpipe_attr_ov10635_yuv,
		&vindis_attr_ov10635,
		&ldc_attr_ov10635,
		&mipi_2lane_ov10635_30fps_yuv_720p_954_attr,
		&sensor_2lane_ov10635_30fps_yuv_720p_954_info},
	{SENSOR_OV10635_30FPS_720p_960_YUV,		// 7
		&vindev_attr_ov10635_yuv,
		&vinpipe_attr_ov10635_yuv,
		&vindis_attr_ov10635,
		&ldc_attr_ov10635,
		&mipi_2lane_ov10635_30fps_yuv_720p_960_attr,
		&sensor_2lane_ov10635_30fps_yuv_720p_960_info},
	{SENSOR_AR0144_30FPS_720P_RAW12_MONO,		// 8
		&vindev_attr_ar0144_720p,
		NULL,
		NULL,
		NULL,
		&mipi_4lane_sensor_ar0144_30fps_12bit_720p_954_attr,
		&sensor_4lane_ar0144_30fps_12bit_720p_954_info},
	{SENSOR_S5KGM1SP_30FPS_4000x3000_RAW19,		// 9
		&vindev_attr_s5kgm1sp_linear,
		&vinpipe_attr_s5kgm1sp_linear,
		&vindis_attr_12m,
		NULL,
		&mipi_sensor_s5kgm1sp_30fps_10bit_linear_attr,
		&sensor_s5kgm1sp_30fps_10bit_linear_info},
	{SENSOR_GC02M1B_25FPS_1600x1200_RAW8,		// 10
		&vindev_attr_gc02m1b_linear,
		&vinpipe_attr_gc02m1b_linear,
		&vindis_attr_gc02m1b,
		&ldc_attr_gc02m1b,
		&mipi_sensor_gc02m1b_25fps_8bit_linear_attr,
		&sensor_gc02m1b_25fps_8bit_linear_info},
	{SENSOR_F37_30FPS_1920x1080_RAW10,		// 11
		&vindev_attr_f37_linear,
		&vinpipe_attr_f37_linear,
		&vindis_attr_f37,
		&ldc_attr_f37,
		&mipi_sensor_f37_30fps_10bit_linear_attr,
		&sensor_f37_30fps_10bit_linear_info},
	{SENSOR_IMX307_30FPS_1080P_RAW12_LINEAR,	// 12
		&vindev_attr_imx307_linear,
		&vinpipe_attr_imx307_linear,
		&vindis_attr_2m,
		&ldc_attr_2m,
		&mipi_4lane_sensor_imx307_30fps_12bit_normal_attr,
		&sensor_4lane_imx307_30fps_12bit_linear_info},
	{SENSOR_AR0233_30FPS_1080P_RAW12_954_PWL,	// 13
		&vindev_attr_ar0233_1080p_base,
		&vinpipe_attr_ar0233_1080p_pwl,
		&vindis_attr_2m,
		&ldc_attr_2m,
		&mipi_4lane_sensor_ar0233_30fps_12bit_1080p_954_attr,
		&sensor_4lane_ar0233_30fps_12bit_1080p_954_pwl_info},
};

/******************************************
 *      Function Declaration              *
 *****************************************/
static video_format format_string_to_enum(const char *str);
static char *sensor_type_to_string(mipi_sensor_type sensor_type);

/******************************************
 *      Function Implementation           *
 *****************************************/
static int vin_get_dev_attr(mipi_sensor_type type, VIN_DEV_ATTR_S *attr)
{
	if (type >= SENOSR_MAX || !vin_attrs[type].dev_attr)
		return -1;

	if (type != vin_attrs[type].sensor_type)
		return -1;

	*attr = *(vin_attrs[type].dev_attr);

	return 0;
}

static int vin_get_pipe_attr(mipi_sensor_type type, VIN_PIPE_ATTR_S *attr)
{
	if (type >= SENOSR_MAX || !vin_attrs[type].pipe_attr)
		return -1;

	if (type != vin_attrs[type].sensor_type)
		return -1;

	*attr = *(vin_attrs[type].pipe_attr);

	return 0;
}

static int vin_get_dis_attr(mipi_sensor_type type, VIN_DIS_ATTR_S *attr)
{
	if (type >= SENOSR_MAX || !vin_attrs[type].dis_attr)
		return -1;

	if (type != vin_attrs[type].sensor_type)
		return -1;

	*attr = *(vin_attrs[type].dis_attr);

	return 0;
}

static int vin_get_ldc_attr(mipi_sensor_type type, VIN_LDC_ATTR_S *attr)
{
	if (type >= SENOSR_MAX || !vin_attrs[type].ldc_attr)
		return -1;

	if (type != vin_attrs[type].sensor_type)
		return -1;

	*attr = *(vin_attrs[type].ldc_attr);

	return 0;
}

static int vin_get_mipi_attr(mipi_sensor_type type, MIPI_ATTR_S *attr)
{
	if (type >= SENOSR_MAX || !vin_attrs[type].mipi_attr)
		return -1;

	if (type != vin_attrs[type].sensor_type)
		return -1;

	*attr = *(vin_attrs[type].mipi_attr);

	return 0;
}

static int vin_get_sensor_info(mipi_sensor_type type,
				MIPI_SENSOR_INFO_S *info)
{
	if (type >= SENOSR_MAX || !vin_attrs[type].sensor_info)
		return -1;

	if (type != vin_attrs[type].sensor_type)
		return -1;

	*info = *(vin_attrs[type].sensor_info);

	return 0;
}

int hb_vin_vps_init_2M(int pipe_id, uint32_t sensor_id, uint32_t mipi_idx,
		       uint32_t deseri_port, uint32_t vin_vps_mode)
{
	VIN_DEV_ATTR_S dev_attr = {0, };
	VIN_PIPE_ATTR_S pipe_attr = {0, };
	int ret = 0;

	trace_in();

	ret = vin_get_dev_attr(sensor_id, &dev_attr);
	result_check(ret);

	ret = HB_SYS_SetVINVPSMode(pipe_id, vin_vps_mode);
	result_check(ret);

	ret = HB_VIN_CreatePipe(pipe_id, &pipe_attr);	// isp init
	result_check(ret);

	ret = HB_VIN_SetMipiBindDev(pipe_id, mipi_idx);
	result_check(ret);

	ret = HB_VIN_SetDevVCNumber(pipe_id, deseri_port);
	result_check(ret);

	ret = HB_VIN_SetDevAttr(pipe_id, &dev_attr);	// sif init
	result_check(ret);

	ret = HB_VIN_SharePipeAE(0, 1);
	result_check(ret);

	trace_out();

	return ret;
}

static int hb_vin_vps_start_2M(int pipe_id)
{
	int ret = 0;

	trace_in();

	ret = HB_VIN_EnableDev(pipe_id);	// sif start && start thread
	if (ret < 0) {
		printf("HB_VIN_EnableDev error!\n");
		return ret;
	}

	trace_out();

	return ret;
}

static void hb_vin_vps_deinit_2M(int pipe_id, int sensor_id)
{
	trace_in();

	HB_VIN_DestroyDev(pipe_id);	// sif deinit && destroy

	trace_out();
}

static int chns2chn(int chns)
{
	int chn = 0;
	if (chns == 0)
		return 0xffffffff;
	while ((chns & 1) != 1) {
		chns >>= 1;
		chn++;
	}
	return chn;
}

int hb_vin_vps_init_12M(int pipe_id, video_context *video_ctx)
{
	VIN_DEV_ATTR_S dev_attr = {0, };
	VIN_PIPE_ATTR_S pipe_attr = {0, };
	VIN_DIS_ATTR_S dis_attr = {0, };
	VIN_LDC_ATTR_S ldc_attr = {0, };

	VPS_GRP_ATTR_S grp_attr = {0, };
	VPS_CHN_ATTR_S chn_attr = {0, };
	VPS_PYM_CHN_ATTR_S pym_chn_attr = {0, };
	int ipu_online_12M_8M_to_pym = 0; // 1: ipu online 12M to pym, 2: ipu online 8M to pym
	int ret = 0;

	trace_in();

	if (!video_ctx)
		return -1;

	uint32_t sensor_id = video_ctx->sensor_id[pipe_id];
	uint32_t mipi_idx = video_ctx->mipi_idx[pipe_id];
	uint32_t deseri_port = video_ctx->serdes_port[pipe_id];
	uint32_t vin_vps_mode = video_ctx->vin_vps_mode[pipe_id];
	uint32_t width = video_ctx->width;
	uint32_t height = video_ctx->height;

	printf("function <%s>, sensor_id(%d), mipi_idx(%d), deseri_port(%d),"
			"vin_vps_mode(%d), width(%d), height(%d)\n",
			__func__, sensor_id, mipi_idx, deseri_port,
			vin_vps_mode, width, height);

	ret = vin_get_dev_attr(sensor_id, &dev_attr);
	result_check(ret);

	ret = vin_get_pipe_attr(sensor_id, &pipe_attr);
	result_check(ret);

	ret = vin_get_dis_attr(sensor_id, &dis_attr);
	result_check(ret);

	ret = vin_get_ldc_attr(sensor_id, &ldc_attr);
	result_check(ret);

	ret = HB_SYS_SetVINVPSMode(pipe_id, vin_vps_mode);
	result_check(ret);

	ret = HB_VIN_CreatePipe(pipe_id, &pipe_attr);		// isp init
	result_check(ret);

	ret = HB_VIN_SetMipiBindDev(pipe_id, mipi_idx);
	result_check(ret);

	ret = HB_VIN_SetDevVCNumber(pipe_id, deseri_port);
	result_check(ret);

	ret = HB_VIN_SetDevAttr(pipe_id, &dev_attr);		// sif init
	result_check(ret);

	ret = HB_VIN_SetPipeAttr(pipe_id, &pipe_attr);		// isp init
	if (ret < 0) {
		printf("HB_VIN_SetPipeAttr error!\n");
		goto pipe_err;
	}

	ret = HB_VIN_SetChnDISAttr(pipe_id, 1, &dis_attr);	//  dis init
	if (ret < 0) {
		printf("HB_VIN_SetChnDISAttr error!\n");
		goto pipe_err;
	}

	ret = HB_VIN_SetChnLDCAttr(pipe_id, 1, &ldc_attr);	//  ldc init
	if (ret < 0) {
		printf("HB_VIN_SetChnLDCAttr error!\n");
		goto pipe_err;
	}

	ret = HB_VIN_SetChnAttr(pipe_id, 1);			//  dwe init
	if (ret < 0) {
		printf("HB_VIN_SetChnAttr error!\n");
		goto chn_err;
	}

	ret = HB_VIN_SetDevBindPipe(pipe_id, pipe_id);		//  bind init
	result_check(ret);

	grp_attr.maxW = pipe_attr.stSize.width;
	grp_attr.maxH = pipe_attr.stSize.height;

	printf("function <%s> grp_attr.maxW(%u), grp_attr.maxH(%u)\n",
			__func__, grp_attr.maxW, grp_attr.maxH);
	ret = HB_VPS_CreateGrp(pipe_id, &grp_attr);
	result_check(ret);

	printf("created a group ok:GrpId = %d\n", pipe_id);

	chn_attr.enScale = 1;
	chn_attr.width = 1920;
	chn_attr.height = 1080;
	chn_attr.frameDepth = 6;

	if (BIT2CHN(video_ctx->ipu_mask, 0)) {
		ret = HB_VPS_SetChnAttr(pipe_id, 0, &chn_attr);
		if (ret) {
			printf("HB_VPS_SetChnAttr error!!!\n");
		} else {
			printf("set chn Attr ok: GrpId = %d, chn_id = %d\n",
			       pipe_id, 0);
		}
		HB_VPS_EnableChn(pipe_id, 0);
	}

	chn_attr.width = 2688;	//1920;
	chn_attr.height = 1520;	//1080;

	if (BIT2CHN(video_ctx->ipu_mask, 5)) {
		ret = HB_VPS_SetChnAttr(pipe_id, 5, &chn_attr);
		if (ret) {
			printf("HB_VPS_SetChnAttr error!!!\n");
		} else {
			printf("set chn Attr ok: GrpId = %d, chn_id = %d\n",
			       pipe_id, 5);
		}
		HB_VPS_EnableChn(pipe_id, 5);
	}

	chn_attr.width = 1920;
	chn_attr.height = 1080;
	if (BIT2CHN(video_ctx->ipu_mask, 1)) {
		ret = HB_VPS_SetChnAttr(pipe_id, 1, &chn_attr);
		if (ret) {
			printf("HB_VPS_SetChnAttr error!!!\n");
		} else {
			printf("set chn Attr ok: GrpId = %d, chn_id = %d\n",
			       pipe_id, 1);
		}
		HB_VPS_EnableChn(pipe_id, 1);
	}

	// chn_attr.width = 1920;
	// chn_attr.height = 1080;
	if (BIT2CHN(video_ctx->ipu_mask, 2)) {
		ret = HB_VPS_SetChnAttr(pipe_id, 2, &chn_attr);
		if (ret) {
			printf("HB_VPS_SetChnAttr error!!!\n");
		} else {
			printf("set chn Attr ok: pipe_id = %d, chn_id = %d\n",
			       pipe_id, 2);
		}
		HB_VPS_EnableChn(pipe_id, 2);
	}

	if (pipe_id == video_ctx->pipe_id_using) {
		chn_attr.width = width;
		chn_attr.height = height;
		if ((width == 4000) && (height == 3000)) {
			ipu_online_12M_8M_to_pym = 1;	// 12M online to pym
		} else if ((width == 3840) && (height == 2160)) {
			ipu_online_12M_8M_to_pym = 2;	// 8M online to pym
		} else {
			ipu_online_12M_8M_to_pym = 0;
		}

		if (BIT2CHN(video_ctx->ipu_mask, 6)) {
			ret = HB_VPS_SetChnAttr(pipe_id, 6, &chn_attr);
			if (ret) {
				printf("HB_VPS_SetChnAttr error!!!\n");
			} else {
				printf
				    ("set chn Attr ok: GrpId = %d, chn_id = %d\n",
				     pipe_id, 6);
			}
			HB_VPS_EnableChn(pipe_id, 6);
		}
	}

	chn_attr.width = 704;
	chn_attr.height = 576;

	if (BIT2CHN(video_ctx->ipu_mask, 3)) {
		ret = HB_VPS_SetChnAttr(pipe_id, 3, &chn_attr);
		if (ret) {
			printf("HB_VPS_SetChnAttr error!!!\n");
		} else {
			printf("set chn Attr ok: GrpId = %d, chn_id = %d\n",
			       pipe_id, 3);
		}
		HB_VPS_EnableChn(pipe_id, 3);
	}

	if (BIT2CHN(video_ctx->ipu_mask, 4)) {
		ret = HB_VPS_SetChnAttr(pipe_id, 4, &chn_attr);
		if (ret) {
			printf("HB_VPS_SetChnAttr error!!!\n");
		} else {
			printf("set chn Attr ok: GrpId = %d, chn_id = %d\n",
			       pipe_id, 4);
		}
		HB_VPS_EnableChn(pipe_id, 4);
	}

	if (video_ctx->pym_mask) {
		memset(&pym_chn_attr, 0, sizeof(VPS_PYM_CHN_ATTR_S));
		pym_chn_attr.timeout = 2000;
		pym_chn_attr.ds_layer_en = 23;
		pym_chn_attr.us_layer_en = 0;
		pym_chn_attr.frame_id = 0;
		pym_chn_attr.frameDepth = 6;
		if (ipu_online_12M_8M_to_pym == 1) {
			// if resolution is 12M ,960x540 at ds_info[5]
			// baselayer is ds[4] 2000x1500
			pym_chn_attr.ds_info[5].factor = 2;
			pym_chn_attr.ds_info[5].roi_x = 40;
			pym_chn_attr.ds_info[5].roi_y = 210;
			pym_chn_attr.ds_info[5].roi_width = 990;
			pym_chn_attr.ds_info[5].roi_height = 558;
		}

		ret = HB_VPS_SetPymChnAttr(pipe_id, chns2chn(video_ctx->pym_mask),
					 &pym_chn_attr);
		if (ret) {
			printf("HB_VPS_SetPymChnAttr error!!!\n");
		} else {
			printf
			    ("HB_VPS_SetPymChnAttr ok: grp_id = %d g_pym_chn = %d\n",
			     pipe_id, chns2chn(video_ctx->pym_mask));
		}
		HB_VPS_EnableChn(pipe_id, chns2chn(video_ctx->pym_mask));
	}

	struct HB_SYS_MOD_S src_mod, dst_mod;
	src_mod.enModId = HB_ID_VIN;
	src_mod.s32DevId = pipe_id;
	if (vin_vps_mode == VIN_ONLINE_VPS_ONLINE ||
	    vin_vps_mode == VIN_OFFLINE_VPS_ONLINE ||
	    vin_vps_mode == VIN_SIF_ONLINE_DDR_ISP_ONLINE_VPS_ONLINE ||
	    vin_vps_mode == VIN_SIF_OFFLINE_ISP_OFFLINE_VPS_ONLINE ||
	    vin_vps_mode == VIN_FEEDBACK_ISP_ONLINE_VPS_ONLINE ||
	    vin_vps_mode == VIN_SIF_VPS_ONLINE)
		src_mod.s32ChnId = 1;
	else
		src_mod.s32ChnId = 0;
	dst_mod.enModId = HB_ID_VPS;
	dst_mod.s32DevId = pipe_id;
	dst_mod.s32ChnId = 0;
	ret = HB_SYS_Bind(&src_mod, &dst_mod);
	if (ret != 0)
		printf("HB_SYS_Bind failed\n");

	trace_out();

	return ret;
pipe_err:
	HB_VIN_DestroyDev(pipe_id);	// sif deinit
chn_err:
	HB_VIN_DestroyPipe(pipe_id);	// isp && dwe deinit
	return ret;
}

static int hb_vin_vps_start_12M(int pipe_id)
{
	int ret = 0;

	trace_in();

	ret = HB_VIN_EnableChn(pipe_id, 0);	// dwe start
	if (ret < 0) {
		printf("HB_VIN_EnableChn error!\n");
		return ret;
	}

	ret = HB_VPS_StartGrp(pipe_id);
	if (ret)
		printf("HB_VPS_StartGrp error!!!\n");
	else
		printf("start grp ok: grp_id = %d\n", pipe_id);

	ret = HB_VIN_StartPipe(pipe_id);	// isp start
	if (ret < 0) {
		printf("HB_VIN_StartPipe error!\n");
		return ret;
	}
	ret = HB_VIN_EnableDev(pipe_id);	// sif start && start thread
	if (ret < 0) {
		printf("HB_VIN_EnableDev error!\n");
		return ret;
	}

	trace_out();

	return ret;
}

static void hb_vin_vps_stop_2M(int pipe_id)
{
	trace_in();

	HB_VIN_DisableDev(pipe_id);	// thread stop && sif stop

	trace_out();
}

static void hb_vin_vps_stop_12M(int pipe_id)
{
	trace_in();

	HB_VIN_DisableDev(pipe_id);	// thread stop && sif stop
	HB_VIN_StopPipe(pipe_id);	// isp stop
	HB_VIN_DisableChn(pipe_id, 1);	// dwe stop
	HB_VPS_StopGrp(pipe_id);

	trace_out();
}

static void hb_vin_vps_deinit_12M(int pipe_id, int sensor_id)
{
	trace_in();

	HB_VIN_DestroyDev(pipe_id);	// sif deinit && destroy
	HB_VIN_DestroyChn(pipe_id, 1);	// dwe deinit
	HB_VIN_DestroyPipe(pipe_id);	// isp deinit && destroy
	HB_VPS_DestroyGrp(pipe_id);

	trace_out();
}

static int hb_sensor_init(int dev_id, int sensor_id, int bus_idx, int port_idx,
		int mipi_idx, int sedres_idx, int sedres_port)
{
	MIPI_SENSOR_INFO_S sensor_info = {0, };
	int ret = 0;

	trace_in();

	ret = vin_get_sensor_info(sensor_id, &sensor_info);
	result_check(ret);

	if (sensor_id == SENSOR_IMX327_30FPS_1952P_RAW12_LINEAR) {
		if (bus_idx == 0)
			system("echo 1 > /sys/class/vps/"
					"mipi_host0/""param/stop_check_instart");
		else if (bus_idx == 5)
			system("echo 1 > /sys/class/vps/"
					"mipi_host1/param/stop_check_instart");
	}

	if (sensor_id == SENSOR_IMX307_30FPS_1080P_RAW12_LINEAR) {
		if (bus_idx == 2) {
			printf("%s mipi_host1 stop_check_instart\n", __func__);
			system("echo 1 > /sys/class/vps/"
					"mipi_host1/param/stop_check_instart");
		}
	}

	HB_MIPI_SetBus(&sensor_info, bus_idx);
	HB_MIPI_SetPort(&sensor_info, port_idx);
	HB_MIPI_SensorBindSerdes(&sensor_info, sedres_idx, sedres_port);
	HB_MIPI_SensorBindMipi(&sensor_info, mipi_idx);

	ret = HB_MIPI_InitSensor(dev_id, &sensor_info);
	if (ret < 0) {
		printf("HB_MIPI_InitSensor error!\n");
		return ret;
	}

	trace_out();

	return ret;
}

static int hb_sensor_start(int dev_id, int sensor_id)
{
	int ret = 0;

	trace_in();

	ret = HB_MIPI_ResetSensor(dev_id);
	if (ret < 0) {
		printf("HB_MIPI_ResetSensor error!\n");
		return ret;
	}

	/* f37 register sys[5] for mirror feature. 1 - mirror, 0 - normal */
	if (sensor_id == SENSOR_F37_30FPS_1920x1080_RAW10) {
		// "i2cset -y -f 2 0x40 0x12 0x00"
		HB_MIPI_WriteSensor(dev_id, 0x12, "0x0", 1);
	}

	trace_out();

	return ret;
}

static int hb_sensor_stop(int dev_id)
{
	int ret = 0;

	ret = HB_MIPI_UnresetSensor(dev_id);
	if (ret < 0) {
		printf("HB_MIPI_UnresetSensor error!\n");
		return ret;
	}

	return ret;
}

static int hb_sensor_deinit(int dev_id)
{
	int ret = 0;

	trace_in();

	ret = HB_MIPI_DeinitSensor(dev_id);
	if (ret < 0) {
		printf("HB_MIPI_DeinitSensor error! ret(0x%x)\n", ret);
		return ret;
	}

	trace_out();

	return ret;
}

static int hb_mipi_init(int sensor_id, int mipi_idx, int need_clk)
{
	int ret = 0;
	MIPI_ATTR_S mipi_attr = {0, };

	ret = vin_get_mipi_attr(sensor_id, &mipi_attr);
	result_check(ret);

	/* need supply clk for os8a10 and f37 sensor */
	if (need_clk && (sensor_id == SENSOR_OS8A10_30FPS_3840P_RAW10_LINEAR
			|| sensor_id == SENSOR_F37_30FPS_1920x1080_RAW10)) {
		printf("supply clock for sensor(eg. os8a10, f37, imx307)\n");
		mipi_attr.mipi_host_cfg.mclk = 2400;
	}

	if (need_clk && (sensor_id == SENSOR_IMX307_30FPS_1080P_RAW12_LINEAR)) {
		mipi_attr.mipi_host_cfg.mclk = 3715;
	}

	ret = HB_MIPI_SetMipiAttr(mipi_idx, &mipi_attr);
	if (ret < 0) {
		printf("HB_MIPI_SetDevAttr error!\n");
		return ret;
	}

	return ret;
}

static int hb_mipi_start(int mipi_id)
{
	int ret = 0;

	trace_in();

	ret = HB_MIPI_ResetMipi(mipi_id);
	if (ret < 0) {
		printf("HB_MIPI_ResetMipi error!\n");
		return ret;
	}

	trace_out();

	return ret;
}

static int hb_mipi_stop(int mipi_id)
{
	int ret = 0;

	trace_in();

	ret = HB_MIPI_UnresetMipi(mipi_id);
	if (ret < 0) {
		printf("HB_MIPI_UnresetMipi error!\n");
		return ret;
	}

	trace_out();

	return ret;
}

static int hb_mipi_deinit(int mipi_idx)
{
	int ret = 0;

	trace_in();

	ret = HB_MIPI_Clear(mipi_idx);
	if (ret < 0) {
		printf("HB_MIPI_Clear error!\n");
		return ret;
	}

	trace_out();

	return ret;
}

static int get_config_list_val(char *str, uint8_t *list, int len)
{
	char *s;
	int tmp, i = 0;
	int r = 0;

	if (!list)
		return -1;

	// get first token
	s = strtok(str, ",");

	while (s != NULL) {
		tmp = atoi(s);
		if (tmp > UINT8_MAX) {
			printf("%s value(%d) too big. need to be [0 ~ 255]\n",
					__func__, tmp);

			r = -1;
			break;
		}

		list[i++] = (uint8_t)tmp;

		if (i >= len) {
			printf("get config list val out of range, "
					"i(%d), len(%d)\n", i, len);

			r = -1;
			break;
		}

		// get following token
		s = strtok(NULL, ",");
	}

	return r;
}

static int load_context_from_file(video_context *video_ctx,
		mipi_sensor_type sensor_type)
{
	struct config_group *grp;
	struct config_key *k;
	char *sensor_name;

	if (sensor_type <= SENSOR_INVALID || sensor_type >= SENOSR_MAX) {
		fprintf(stderr, "load_context_from_file. "
				"not invalid sensor_type(%d)\n", sensor_type);
		return -1;
	}

	if (load_config_file("/etc/usb_webcam.conf")) {
		fprintf(stderr, "load config file \"/etc/usb_webcam.conf\" failed");
		return -1;
	}

	printf("using /etc/usb_webcam.conf as option config file\n");

	sensor_name = sensor_type_to_string(sensor_type);
	grp = find_config_group(sensor_name);
	if (!grp) {
		fprintf(stderr, "find group %s failed in usb_webcam.conf",
				sensor_name);
		return -1;
	}

	printf("find %s group in config file succeed\n", sensor_name);

	config_key_for_each(k, grp->keylist)  {
		if (!strcasecmp(k->name, "format"))
			video_ctx->format = format_string_to_enum(k->val);
		else if (!strcasecmp(k->name, "width"))
			video_ctx->width = atoi(k->val);
		else if (!strcasecmp(k->name, "height"))
			video_ctx->height = atoi(k->val);
		else if (!strcasecmp(k->name, "req_width"))
			video_ctx->req_width = atoi(k->val);
		else if (!strcasecmp(k->name, "req_height"))
			video_ctx->req_height = atoi(k->val);
		else if (!strcasecmp(k->name, "pipe_id_mask"))
			video_ctx->pipe_id_mask = (uint8_t)atoi(k->val);
		else if (!strcasecmp(k->name, "pipe_id_using"))
			video_ctx->pipe_id_using = (uint8_t)atoi(k->val);
		else if (!strcasecmp(k->name, "need_clk"))
			video_ctx->need_clk = (uint8_t)atoi(k->val);
		else if (!strcasecmp(k->name, "use_pattern"))
			video_ctx->use_pattern = (uint8_t)atoi(k->val);
		else if (!strcasecmp(k->name, "pattern_fps"))
			video_ctx->pattern_fps = (uint8_t)atoi(k->val);
		else if (!strcasecmp(k->name, "pym_mask"))
			video_ctx->pym_mask = (uint8_t)atoi(k->val);
		else if (!strcasecmp(k->name, "ipu_mask"))
			video_ctx->ipu_mask = (uint8_t)atoi(k->val);
		else if (!strcasecmp(k->name, "has_venc"))
			video_ctx->has_venc = (uint8_t)atoi(k->val);
		else if (!strcasecmp(k->name, "sensor_id"))
			get_config_list_val(k->val, video_ctx->sensor_id,
					MAX_SENSOR_NUM);
		else if (!strcasecmp(k->name, "mipi_idx"))
			get_config_list_val(k->val, video_ctx->mipi_idx,
					MAX_MIPI_NUM);
		else if (!strcasecmp(k->name, "bus_idx"))
			get_config_list_val(k->val, video_ctx->bus_idx,
					MAX_SENSOR_NUM);
		else if (!strcasecmp(k->name, "port_idx"))
			get_config_list_val(k->val, video_ctx->port_idx,
					MAX_SENSOR_NUM);
		else if (!strcasecmp(k->name, "serdes_idx"))
			get_config_list_val(k->val, video_ctx->serdes_idx,
					MAX_SENSOR_NUM);
		else if (!strcasecmp(k->name, "serdes_port"))
			get_config_list_val(k->val, video_ctx->serdes_port,
					MAX_SENSOR_NUM);
		else if (!strcasecmp(k->name, "vin_vps_mode"))
			get_config_list_val(k->val, video_ctx->vin_vps_mode,
					MAX_SENSOR_NUM);
		else
			printf("other key not support yet. key(%s)\n", k->name);
	}

	return 0;
}

static void show_video_context_info(video_context *video_ctx)
{
	printf("#### video context ####\n");
	printf("venc_ctx =	 %p\n", video_ctx->venc_ctx);
	printf("format =         %u\n", video_ctx->format);
	printf("width  =         %u\n", video_ctx->width);
	printf("height =         %u\n", video_ctx->height);
	printf("pipe_id_mask =       %u\n", video_ctx->pipe_id_mask);
	printf("need_clk =       %u\n", video_ctx->need_clk);
	printf("use_pattern =    %u\n", video_ctx->use_pattern);
	printf("pattern_fps =    %u\n", video_ctx->pattern_fps);
	printf("pym_mask =        %u\n", video_ctx->pym_mask);
	printf("has_venc =       %u\n", video_ctx->has_venc);
	printf("sensor_id[0] =   %u\n", video_ctx->sensor_id[0]);
	printf("mipi_idx[0] =    %u\n", video_ctx->mipi_idx[0]);
	printf("bus_idx[0] =     %u\n", video_ctx->bus_idx[0]);
	printf("port_idx[0] =    %u\n", video_ctx->port_idx[0]);
	printf("serdes_idx[0] =  %u\n", video_ctx->serdes_idx[0]);
	printf("serdes_port[0] = %u\n", video_ctx->serdes_port[0]);
	printf("vin_vps_mode[0] =   %u\n", video_ctx->vin_vps_mode[0]);
}

video_context *hb_video_alloc_context(mipi_sensor_type sensor_type)
{
	video_context *video_ctx = calloc(1, sizeof(video_context));

	/* default video params (12M test pattern, 1080p, mjpeg)*/
	video_ctx->format = VIDEO_FMT_MJPEG;
	video_ctx->venc_ctx = NULL;
	video_ctx->mono_raw_q = NULL;
	video_ctx->width = 1920;
	video_ctx->height = 1080;
	video_ctx->req_width = 1920;
	video_ctx->req_height = 1080;
	video_ctx->pipe_id_mask = 1;
	video_ctx->pipe_id_using = 0;
	video_ctx->need_clk = 1;
	video_ctx->use_pattern = 1;
	video_ctx->pattern_fps = 30;
	video_ctx->ipu_mask = 64;
	video_ctx->pym_mask = 64;
	video_ctx->has_venc = 0;		/* don't need venc as default */
	video_ctx->sensor_id[0] = 2;		/* (12m test pattern) SENSOR_SIF_TEST_PATTERN_12M_RAW12_12M */
	video_ctx->mipi_idx[0] = 1;
	video_ctx->bus_idx[0] = 5;
	video_ctx->port_idx[0] = 0;
	video_ctx->serdes_idx[0] = 0;
	video_ctx->serdes_port[0] = 0;
	video_ctx->vin_vps_mode[0] = 0;
	video_ctx->quit = 0;

	video_ctx->tmp_buffer = NULL;
	video_ctx->buffer_size = 0;

	if (load_context_from_file(video_ctx, sensor_type))
		printf("load context from file failed\n");

	show_video_context_info(video_ctx);

	return video_ctx;
}

int hb_video_init(video_context *video_ctx)
{
	int ret = -1, i;
	int vin_vps_init_num = 0, sensor_init_num = 0, mipi_init_num = 0;

	trace_in();

	show_video_context_info(video_ctx);

	for (i = 0; i < MAX_ID_NUM; i++) {
		if (BIT(i) & video_ctx->pipe_id_mask) {
			if (i == 0) {
				ret = hb_vin_vps_init_12M(i, video_ctx);
				if (ret < 0) {
					printf("pipe 0 hb_vin_init error!\n");
					return ret;
				}

				vin_vps_init_num = i + 1;
			}

			if (i == 1) {
				ret = hb_vin_vps_init_2M(i, video_ctx->sensor_id[i],
						       video_ctx->mipi_idx[i],
						       video_ctx->serdes_port[i],
						       video_ctx->vin_vps_mode[i]);
				if (ret < 0) {
					printf("pipe 1 hb_vin_init error!\n");
					vin_vps_init_num = i;
					goto error2;
				}

				vin_vps_init_num = i + 1;
			}
		}
	}

	if (video_ctx->use_pattern) {
		/* use test pattern */
		for (i = 0; i < MAX_ID_NUM; i++) {
			if (BIT(i) & video_ctx->pipe_id_mask) {
				ret = hb_sensor_init(i, video_ctx->sensor_id[i],
					video_ctx->bus_idx[i], video_ctx->port_idx[i],
					video_ctx->mipi_idx[i],
					video_ctx->serdes_idx[i],
					video_ctx->serdes_port[i]);

				if (ret < 0) {
					printf("hb_sensor_init error! "
							"do vio deinit\n");
					sensor_init_num = i;
					goto error1;
				}

				ret = HB_MIPI_SetSensorFps(i, video_ctx->pattern_fps);
				if (ret < 0) {
					printf("HB_MIPI_SetSensorFps error! "
							"do vio deinit\n");
					sensor_init_num = i;
					goto error1;
				}

				sensor_init_num = i + 1;
			}
		}
	} else {
		/* use real camera */
		for (i = 0; i < MAX_ID_NUM; i++) {
			if (BIT(i) & video_ctx->pipe_id_mask) {
				if (video_ctx->need_clk)
					HB_MIPI_EnableSensorClock(video_ctx->mipi_idx[i]);

				ret = hb_sensor_init(i, video_ctx->sensor_id[i],
					video_ctx->bus_idx[i], video_ctx->port_idx[i],
					video_ctx->mipi_idx[i],
					video_ctx->serdes_idx[i],
					video_ctx->serdes_port[i]);

				if (ret < 0) {
					printf("hb_sensor_init error! "
						"do vio deinit\n");
					sensor_init_num = i;
					goto error1;
				}

				sensor_init_num = i + 1;

				ret = hb_mipi_init(video_ctx->sensor_id[i],
						video_ctx->mipi_idx[i],
						video_ctx->need_clk);
				if (ret < 0) {
					printf("hb_mipi_init error! "
							"do vio deinit\n");

					mipi_init_num = i;
					goto error1;
				}

				mipi_init_num = i + 1;
			}
		}
	}

	trace_out();

	return ret;

error1:
	if (video_ctx->use_pattern) {
		for (i = 0; i < sensor_init_num; i++) {
			if (BIT(i) & video_ctx->pipe_id_mask) {
				hb_sensor_deinit(i);
			}
		}
	} else {
		for (i = 0; i < sensor_init_num; i++) {
			if (BIT(i) & video_ctx->pipe_id_mask) {
				if (hb_sensor_deinit(i) < 0)
					fprintf(stderr, "[%d]hb_sensor_deinit "
							"failed\n", i);
			}
		}

		for (i = 0; i < mipi_init_num; i++) {
			if (BIT(i) & video_ctx->pipe_id_mask) {
				if (hb_mipi_deinit(video_ctx->mipi_idx[i]) < 0)
					fprintf(stderr, "[%d]hb_mipi_deinit"
							"failed. mipi_idx[%d]\n",
							i, video_ctx->mipi_idx[i]);
			}
		}

		/* disable sensor clock */
		if (video_ctx->need_clk)
			HB_MIPI_DisableSensorClock(video_ctx->mipi_idx[i]);
	}

error2:

	for (i = 0; i < vin_vps_init_num; i++) {
		if (BIT(i) & video_ctx->pipe_id_mask) {
			if (i == 0)
				hb_vin_vps_deinit_12M(i, video_ctx->sensor_id[i]);

			if (i == 1)
				hb_vin_vps_deinit_2M(i, video_ctx->sensor_id[i]);
		}
	}

	return ret;
}

int hb_video_prepare(video_context *video_ctx)
{
	if (!video_ctx)
		return -1;

	trace_in();

	venc_context *venc_ctx = video_ctx->venc_ctx;
	if (venc_ctx) {
		printf("already allocate video encoder context(%p)\n",
				venc_ctx);
		return -1;
	}

	if (video_ctx->format == VIDEO_FMT_H264 || video_ctx->format == VIDEO_FMT_H265
			|| video_ctx->format == VIDEO_FMT_MJPEG) {
		/* need video encoder */
		printf("need video encoder, prepare video encoder\n");
		video_ctx->has_venc = 1;

		venc_ctx = hb_video_encoder_alloc_context();
		if (!venc_ctx) {
			printf("hb_video_encoder_alloc_context failed\n");
			return -1;
		}

		switch (video_ctx->format) {
		case VIDEO_FMT_H264:
			venc_ctx->type = VENC_H264;
			break;
		case VIDEO_FMT_H265:
			venc_ctx->type = VENC_H265;
			break;
		case VIDEO_FMT_MJPEG:
			venc_ctx->type = VENC_MJPEG;
			break;
		default:
			break;
		}

		venc_ctx->width = video_ctx->req_width;
		venc_ctx->height = video_ctx->req_height;

		video_ctx->venc_ctx = venc_ctx;

		if (hb_video_encoder_init(venc_ctx)){
			printf("video encoder init failed\n");
			goto error;
		}
	} else {
		/* don't need video encoder */
		printf("yuv (nv12 or yuy2 data). no need video encoder\n");
		video_ctx->has_venc = 0;
	}

	trace_out();

	return 0;

error:
	free(venc_ctx);
	video_ctx->venc_ctx = NULL;

	return -1;
}

static void *hb_video_work_routine(void *param)
{
	video_context *vctx = (video_context *)param;
	int ret;

	trace_in();

	if (!vctx)
		return (void *)0;

	if (vctx->mono_raw_q) {
		printf("error happen. mono_raw_q already create...\n");
		return (void *)0;
	}

	vctx->mono_raw_q = calloc(1, sizeof(raw_single_buffer));

	if (!vctx->mono_raw_q)
		return (void *)0;

	raw_single_buffer *mono_raw_q = vctx->mono_raw_q;
	mono_raw_q->used = 0;
	memset(&mono_raw_q->pym_buffer, 0, sizeof(pym_buffer_t));
	pthread_mutex_init(&mono_raw_q->mutex, NULL);

	while (!vctx->quit) {
		if (!mono_raw_q)
			break;

		/* check uvc single buffer */
		pthread_mutex_lock(&mono_raw_q->mutex);
		if (mono_raw_q->used) {
			pthread_mutex_unlock(&mono_raw_q->mutex);
			usleep(5*1000);		/* sleep 5ms and re-check mono_raw_q */
			continue;
		}

		ret = HB_VPS_GetChnFrame(0, 6, &mono_raw_q->pym_buffer, 2000);
		if (ret < 0) {
			pthread_mutex_unlock(&mono_raw_q->mutex);
			usleep(3*1000);		/* sleep 3ms and re-encode */
			continue;
		}

#if 0
		printf("HB_VPS_GetChnFrame Succeed, data(%p), width(%d), height(%d)\n",
				mono_raw_q->pym_buffer.pym[0].addr[0],
				mono_raw_q->pym_buffer.pym[0].width,
				mono_raw_q->pym_buffer.pym[0].height);
#endif

		mono_raw_q->used = 1;
		pthread_mutex_unlock(&mono_raw_q->mutex);
	}

	trace_out();

	return (void *)0;

}

static int hb_video_start_routine(video_context *video_ctx)
{
	int ret;

	trace_in();

	if (!video_ctx)
		return -1;

	video_ctx->quit = 0;
	ret = pthread_create(&video_ctx->routine_id, NULL,
			hb_video_work_routine, video_ctx);

	if (ret) {
		printf("start hb_video_work_routine thread failed\n");
		return -1;
	}

	trace_out();

	return 0;
}

static int hb_video_stop_routine(video_context *video_ctx)
{
	trace_in();

	if (!video_ctx)
		return -1;

	video_ctx->quit = 1;
	pthread_join(video_ctx->routine_id, NULL);

	if (video_ctx->mono_raw_q) {
		free(video_ctx->mono_raw_q);
		video_ctx->mono_raw_q = NULL;
	}

	trace_out();

	return 0;
}

int hb_video_start(video_context *video_ctx)
{
	int has_2m = 0, has_12m = 0;
	int pipe_2m_id = 0, pipe_12m_id = 0;
	int sensor_cur_num = 0;
	int mipi_cur_num = 0;
	int ret = 0, i;

	trace_in();

	if (!video_ctx)
		return -1;

	venc_context *venc_ctx = video_ctx->venc_ctx;

	/* vin vps start */
	for (i = 0; i < MAX_ID_NUM; i++) {
		if (BIT(i) & video_ctx->pipe_id_mask) {
			if (i == 0) {
				ret = hb_vin_vps_start_12M(i);
				has_12m = 1;
				pipe_12m_id = i;

				if (ret < 0) {
					printf("hb_vin_vps_start_12M error! "
						"do cam && vio deinit\n");
					goto error1;
				}
			}

			if (i == 1) {
				ret = hb_vin_vps_start_2M(i);
				has_2m = 1;
				pipe_2m_id = i;

				if (ret < 0) {
					printf("hb_vin_vps_start_2M error! "
						"do cam && vio deinit\n");
					goto error1;
				}
			}
		}
	}

	/* video encoder start */
	if (video_ctx->has_venc) {
		if (hb_video_encoder_start(venc_ctx)) {
			printf("hb_video_encoder_start failed\n");
			goto error1;
		}
	} else {
		/* start yuv(nv12 or yuy2) work routine */
		if (hb_video_start_routine(video_ctx)) {
			printf("hb_video_start_routine failed\n");
			goto error1;
		}
	}

	/* sensor(or test pattern), mipi strat */
	if (!video_ctx->use_pattern) {
		/* not use pattern, but use real camera */
		for (i = 0; i < MAX_ID_NUM; i++) {
			if (BIT(i) & video_ctx->pipe_id_mask) {
				ret = hb_sensor_start(i,
						video_ctx->sensor_id[i]);

				if (ret < 0) {
					printf("hb_sensor_start error! "
						"do cam && vio deinit\n");
					sensor_cur_num = i;
					goto error2;
				}

				ret = hb_mipi_start(video_ctx->mipi_idx[i]);
				if (ret < 0) {
					printf("hb_mipi_start error! "
						"do cam && vio deinit\n");
					mipi_cur_num = i;
					goto error3;
				}
			}
		}
	}

	trace_out();

	return 0;

error3:
	if (pipe_2m_id <= mipi_cur_num)
		hb_mipi_stop(pipe_2m_id);

	if (pipe_12m_id <= mipi_cur_num)
		hb_mipi_stop(pipe_12m_id);

error2:
	if (pipe_2m_id <= sensor_cur_num)
		hb_sensor_stop(pipe_2m_id);

	if (pipe_12m_id <= sensor_cur_num)
		hb_sensor_stop(pipe_12m_id);

error1:
	if (has_2m) {
		hb_sensor_deinit(pipe_2m_id);
		hb_mipi_deinit(video_ctx->mipi_idx[pipe_2m_id]);
		hb_vin_vps_stop_2M(pipe_2m_id);
		hb_vin_vps_deinit_2M(pipe_2m_id, video_ctx->sensor_id[pipe_2m_id]);
	}

	if (has_12m) {
		hb_sensor_deinit(pipe_12m_id);
		hb_mipi_deinit(video_ctx->mipi_idx[pipe_12m_id]);
		hb_vin_vps_stop_12M(pipe_12m_id);
		hb_vin_vps_deinit_12M(pipe_12m_id, video_ctx->sensor_id[pipe_12m_id]);
	}

	return ret;
}

int hb_video_stop(video_context *video_ctx)
{
	int i;

	if (!video_ctx)
		return -1;

	trace_in();

	venc_context *venc_ctx = video_ctx->venc_ctx;

	/* 1. stop video encoder first */
	if (video_ctx->has_venc) {
		if (hb_video_encoder_stop(venc_ctx))
			printf("hb_video_encoder_stopfailed\n");
	} else {
		/* stop yuv(nv12 or yuy2) work routine */
		if (hb_video_stop_routine(video_ctx))
			printf("hb_video_stop_routine failed\n");
	}

	/* 2. stop vin vps */
	for (i = 0; i < MAX_ID_NUM; i++) {
		if (BIT(i) & video_ctx->pipe_id_mask) {
			if (i == 0)
				hb_vin_vps_stop_12M(i);

			if (i == 1)
				hb_vin_vps_stop_2M(i);
		}
	}

	/* 3. stop mipi sensor(for real camera case, not test pattern) */
	if (!video_ctx->use_pattern) {
		for (i = 0; i < MAX_ID_NUM; i++) {
			if (BIT(i) & video_ctx->pipe_id_mask) {
				if (hb_mipi_stop(video_ctx->mipi_idx[i]))
					fprintf(stderr, "[%d]hb_mipi_stop failed."
							" mipi_idx(%d)\n",
							i, video_ctx->mipi_idx[i]);

				if (hb_sensor_stop(i) < 0)
					fprintf(stderr, "[%d]hb_sensor_stop "
							"failed.\n", i);
			}
		}
	}

	trace_out();

	return 0;
}

void hb_video_finalize(video_context *video_ctx)
{
	if (!video_ctx)
		return;

	trace_in();

	venc_context *venc_ctx = video_ctx->venc_ctx;
	if (!venc_ctx) {
		printf("no venc_ctx. no need to release video encoder\n");
		return;
	}

	if (video_ctx->has_venc) {
		printf("hb_video_encoder_deinit\n");
		hb_video_encoder_deinit(venc_ctx);

		video_ctx->venc_ctx = NULL;
	} else {
		/* don't need video encoder */
		printf("no video encoder, do nothing\n");
	}

	trace_out();
}

void hb_video_deinit(video_context *video_ctx)
{
	int i;

	trace_in();

	/* 1. deinit vin & vps */
	for (i = 0; i < MAX_ID_NUM; i++) {
		if (BIT(i) & video_ctx->pipe_id_mask) {
			if (i == 0)
				hb_vin_vps_deinit_12M(i, video_ctx->sensor_id[i]);

			if (i == 1)
				hb_vin_vps_deinit_2M(i, video_ctx->sensor_id[i]);
		}
	}

	/* 2. deinit sensor & mipi */
	if (video_ctx->use_pattern) {
		/* test pattern */
		for (i = 0; i < MAX_ID_NUM; i++) {
			if (BIT(i) & video_ctx->pipe_id_mask) {
				if (hb_sensor_deinit(i) < 0)
					fprintf(stderr, "[%d]hb_sensor_deinit(%d) "
							"failed\n", i, __LINE__);
			}
		}
	} else {
		/* real camera */
		for (i = 0; i < MAX_ID_NUM; i++) {
			if (BIT(i) & video_ctx->pipe_id_mask) {
				if (hb_sensor_deinit(i) < 0)
					fprintf(stderr, "[%d]hb_sensor_deinit(%d) "
							"failed\n", i, __LINE__);
			}
		}

		for (i = 0; i < MAX_ID_NUM; i++) {
			if (BIT(i) & video_ctx->pipe_id_mask) {
				if (hb_mipi_deinit(video_ctx->mipi_idx[i]) < 0)
					fprintf(stderr, "[%d]hb_mipi_deinit"
							"failed. mipi_idx[%d]\n",
							i, video_ctx->mipi_idx[i]);
			}
		}

		/* disable sensor clock */
		for (i = 0; i < MAX_ID_NUM; i++) {
			if (BIT(i) & video_ctx->pipe_id_mask)
				if (video_ctx->need_clk)
					HB_MIPI_DisableSensorClock(video_ctx->mipi_idx[i]);
		}
	}

#if 0
	if (video_ctx)
		free(video_ctx);
#endif

	trace_out();
}

void hb_video_free_context(video_context *video_ctx)
{
	if (video_ctx)
		free(video_ctx);
}

video_format fcc_to_video_format(unsigned int fcc)
{
	video_format format = VIDEO_FMT_INVALID;

	switch (fcc) {
	case V4L2_PIX_FMT_YUYV:
		format = VIDEO_FMT_YUY2;
		break;
	case V4L2_PIX_FMT_NV12:
		format = VIDEO_FMT_NV12;
		break;
	case V4L2_PIX_FMT_MJPEG:
		format = VIDEO_FMT_MJPEG;
		break;
	case V4L2_PIX_FMT_H264:
		format = VIDEO_FMT_H264;
		break;
	case V4L2_PIX_FMT_H265:
		format = VIDEO_FMT_H265;
		break;
	default:
		break;
	}

	return format;
}

static video_format format_string_to_enum(const char *str)
{
	video_format format = VIDEO_FMT_INVALID;

	if (!str)
		return format;

	if (strcasecmp(str, "yuy2"))
		format = VIDEO_FMT_YUY2;
	else if (strcasecmp(str, "nv12"))
		format = VIDEO_FMT_NV12;
	else if (strcasecmp(str, "mjpeg"))
		format = VIDEO_FMT_MJPEG;
	else if (strcasecmp(str, "h264"))
		format = VIDEO_FMT_H264;
	else if (strcasecmp(str, "h265"))
		format = VIDEO_FMT_H265;
	else
		format = VIDEO_FMT_INVALID;

	return format;
}

static char *sensor_type_to_string(mipi_sensor_type sensor_type)
{
	switch (sensor_type) {
	case SENSOR_SIF_TEST_PATTERN0_1080P:
		return "pattern@1080p";
	case SENSOR_SIF_TEST_PATTERN_12M_RAW12_8M:
		return "pattern@12m_8m";
	case SENSOR_SIF_TEST_PATTERN_12M_RAW12_12M:
		return "pattern@12m";
	case SENSOR_SIF_TEST_PATTERN_8M_RAW12:
		return "pattern@8m";
	case SENSOR_IMX327_30FPS_1952P_RAW12_LINEAR:
		return "imx327";
	case SENSOR_OS8A10_30FPS_3840P_RAW10_LINEAR:
		return "os8a10";
	case SENSOR_OV10635_30FPS_720p_954_YUV:
		return "ov10635";
	case SENSOR_OV10635_30FPS_720p_960_YUV:
		return "ov10635";
	case SENSOR_AR0144_30FPS_720P_RAW12_MONO:
		return "ar0144";
	case SENSOR_S5KGM1SP_30FPS_4000x3000_RAW19:
		return "s5kgm1sp";
	case SENSOR_GC02M1B_25FPS_1600x1200_RAW8:
		return "gc02m1b";
	case SENSOR_F37_30FPS_1920x1080_RAW10:
		return "f37";
	case SENSOR_IMX307_30FPS_1080P_RAW12_LINEAR:
		return "imx307";
	case SENSOR_AR0233_30FPS_1080P_RAW12_954_PWL:
		return "ar0233";
	default:
		return "unknown";
	}
}
