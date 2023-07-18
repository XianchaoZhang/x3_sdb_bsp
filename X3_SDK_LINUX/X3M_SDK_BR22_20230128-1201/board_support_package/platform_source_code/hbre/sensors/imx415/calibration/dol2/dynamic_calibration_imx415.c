/**
 *  The confidential and proprietary information contained in this file may
 *   only be used by a person authorised under and to the extent permitted
 *  by a subsisting licensing agreement from ARM Limited or its affiliates.
 *
 *        (C) COPYRIGHT 2020 ARM Limited or its affiliates
 *                       ALL RIGHTS RESERVED
 /
 *       This entire notice must be reproduced on all copies of this file
 *   and copies of this file may only be made by a person if such person is
 *    permitted to do so under the terms of a subsisting license agreement
 *                  from ARM Limited or its affiliates.
 *
 */ 

#include "acamera_command_define.h"
//#include "apical_types.h"
//#include "apical_command_api.h"
//#include "apical_firmware_api.h"

// created from 2020-01-02T11:46:38.724Z UTCdynamic-calibrations (14).json

// CALIBRATION_AE_CONTROL (1x9 4 bytes)
static uint32_t _calibration_ae_control[]
 = {15,200,5,50,15,85,95,1,8};

// CALIBRATION_AE_CONTROL_HDR_TARGET (8x2 2 bytes)
static uint16_t _calibration_ae_control_hdr_target[][2]
 =  {
  { 0, 200 },
  { 256, 180 },
  { 512, 145 },
  { 768, 120 },
  { 1024, 100 },
  { 1280, 90 }
};

// CALIBRATION_AE_CORRECTION (1x16 1 bytes)
static uint8_t _calibration_ae_correction[]
 = {128,128,128,128,128,110,107,90,70,65,55,50,45,45,40,40};

// CALIBRATION_AE_EXPOSURE_CORRECTION (1x16 4 bytes)
static uint32_t _calibration_ae_exposure_correction[]
 = {6030,9989,14861,18360,24722,31760,53480,81060,128600,176500,221560,296400,371117,466000,568000,656000};
 //= {1920,2715,4840,6430,9680,13861,17360,23722,30720,53444,71440,96400,131117,196000,298000,416000};

// CALIBRATION_AE_ZONE_WGHT_HOR (1x32 2 bytes)
static uint16_t _calibration_ae_zone_wght_hor[]
 = {16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16};

// CALIBRATION_AE_ZONE_WGHT_VER (1x32 2 bytes)
static uint16_t _calibration_ae_zone_wght_ver[]
 = {16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16};

// CALIBRATION_EXPOSURE_RATIO_ADJUSTMENT (4x2 2 bytes)
static uint16_t _calibration_exposure_ratio_adjustment[][2]
 =  {
  { 2560, 200 },
  { 2800, 340},
  { 7680, 340 },
  { 10240, 340 },
  { 15360, 768 },
  { 65535, 768 }
};

// CALIBRATION_CMOS_CONTROL (1x18 4 bytes)
static uint32_t _calibration_cmos_control[]
 = {1,50,0,0,0,0,1,431,160,0,32,10,431,5,0,8,0,4,120,750,0,420,336,5,120,200};

// CALIBRATION_CMOS_EXPOSURE_PARTITION_LUTS (2x10 2 bytes)
static uint16_t _calibration_cmos_exposure_partition_luts[][10]
 =  {
  { 1, 1, 1, 1, 2, 1, 3, 1, 4, 15 },
  { 10, 1, 10, 2, 20, 2, 20, 4, 40, 16 }
};

// CALIBRATION_AF_LMS (1x21 4 bytes)
static uint32_t _calibration_af_lms[]
 = {14416,14416,14416,14416,14416,14416,15184,15184,15184,15184,15184,15184,8,20,2,30,131072,131072,409600,65536,0};

// CALIBRATION_AF_ZONE_WGHT_HOR (1x17 2 bytes)
static uint16_t _calibration_af_zone_wght_hor[]
 = {0,0,16,16,16,16,16,16,16,16,16,16,16,16,16,0,0};

// CALIBRATION_AF_ZONE_WGHT_VER (1x17 2 bytes)
static uint16_t _calibration_af_zone_wght_ver[]
 = {0,0,16,16,16,16,16,16,16,16,16,16,16,16,16,0,0};

// CALIBRATION_AWB_AVG_COEF (1x1 1 bytes)
static uint8_t _calibration_awb_avg_coef[]
 = {15};

// CALIBRATION_AWB_BG_MAX_GAIN (3x2 2 bytes)
static uint16_t _calibration_awb_bg_max_gain[][2]
 =  {
  { 0, 100 },
  { 256, 100 },
  { 1792, 200 }
};

// AWB_COLOUR_PREFERENCE (1x4 2 bytes)
static uint16_t _awb_colour_preference[]
 = {7500,6000,4700,2800};

// CALIBRATION_AWB_MIX_LIGHT_PARAMETERS (1x8 4 bytes)
static uint32_t _calibration_awb_mix_light_parameters[]
 = {0,600,2500,40000,500,0,300,256};

// CALIBRATION_AWB_ZONE_WGHT_HOR (1x32 2 bytes)
static uint16_t _calibration_awb_zone_wght_hor[]
 = {16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16};

// CALIBRATION_AWB_ZONE_WGHT_VER (1x32 2 bytes)
static uint16_t _calibration_awb_zone_wght_ver[]
 = {16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16};

// CALIBRATION_EVTOLUX_PROBABILITY_ENABLE (1x1 1 bytes)
static uint8_t _calibration_evtolux_probability_enable[]
 = {0};

// CALIBRATION_AUTO_LEVEL_CONTROL (1x7 4 bytes)
static uint32_t _calibration_auto_level_control[]
 = {6,95,5,15,99,20,1};

// CALIBRATION_CCM_ONE_GAIN_THRESHOLD (1x1 2 bytes)
static uint16_t _calibration_ccm_one_gain_threshold[]
 = {1920};

// CALIBRATION_DEMOSAIC_NP_OFFSET (9x2 2 bytes)
static uint16_t _calibration_demosaic_np_offset[][2]
 =  {
  { 0, 1 },
  { 256, 1 },
  { 512, 1 },
  { 768, 1 },
  { 1024, 1 },
  { 1280, 3 },
  { 1536, 3 },
  { 1790, 3 },
  { 1920, 3 },
  { 2048, 3 },

};

// CALIBRATION_SHARP_ALT_D (11x2 2 bytes)
static uint16_t _calibration_sharp_alt_d[][2]
 =  {
  { 0, 120 },
  { 256, 120 },
  { 512, 115 },
  { 768, 100 },
  { 1024, 96 },
  { 1280, 96 }
};

// CALIBRATION_SHARP_ALT_DU (11x2 2 bytes)
static uint16_t _calibration_sharp_alt_du[][2]
 =  {
  { 0, 80 },
  { 256, 80 },
  { 512, 80 },
  { 768, 80 },
  { 1024, 80 },
  { 1280, 80 }
};

// CALIBRATION_SHARP_ALT_UD (11x2 2 bytes)
static uint16_t _calibration_sharp_alt_ud[][2]
 =  {
  { 0, 5 },
  { 256, 5 },
  { 512, 3 },
  { 768, 0 },
  { 1024, 0 },
  { 1280, 0 }
};

// CALIBRATION_DP_SLOPE (7x2 2 bytes)
static uint16_t _calibration_dp_slope[][2]
 =  {
  { 0, 170 },
  { 256, 360 },
  { 512, 2048 },
  { 768, 2048 },
  { 1024, 2048 },
  { 1280, 2048 }
};

// CALIBRATION_DP_THRESHOLD (10x2 2 bytes)
static uint16_t _calibration_dp_threshold[][2]
 =  {
  { 0, 256 },
  { 256, 256 },
  { 512, 25 },
  { 768, 25 },
  { 1024, 25 },
  { 1280, 25 }
};

// CALIBRATION_IRIDIX_AVG_COEF (1x1 1 bytes)
static uint8_t _calibration_iridix_avg_coef[]
 = {15};

// CALIBRATION_IRIDIX_EV_LIM_FULL_STR (1x1 4 bytes)
static uint32_t _calibration_iridix_ev_lim_full_str[]
 = {1557570};

// CALIBRATION_IRIDIX_EV_LIM_NO_STR (1x2 4 bytes)
static uint32_t _calibration_iridix_ev_lim_no_str[]
 = {3796440,4036440};

// CALIBRATION_IRIDIX_MIN_MAX_STR (1x1 2 bytes)
static uint16_t _calibration_iridix_min_max_str[]
 = {0};

// CALIBRATION_IRIDIX_STRENGTH_MAXIMUM (1x1 1 bytes)
static uint8_t _calibration_iridix_strength_maximum[]
 = {255};

// CALIBRATION_IRIDIX8_STRENGTH_DK_ENH_CONTROL (1x15 4 bytes)
static uint32_t _calibration_iridix8_strength_dk_enh_control[]
 = {15,95,850,1850,0,30,4096,14080,0,22,30,2048,10480,32,0};

// CALIBRATION_MESH_SHADING_STRENGTH (1x2 2 bytes)

static uint16_t _calibration_mesh_shading_strength[][2]
 =  {
  { 0, 4096 },
  { 256, 4096 },
  { 512, 3196 },
  { 768, 1024 },
  { 1024, 256 },
  { 1280, 0 }
};

// CALIBRATION_PF_RADIAL_LUT (1x33 1 bytes)
static uint8_t _calibration_pf_radial_lut[]
 = {255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255};

// CALIBRATION_PF_RADIAL_PARAMS (1x3 2 bytes)
static uint16_t _calibration_pf_radial_params[]
 = {1920,1080,442};

// CALIBRATION_SATURATION_STRENGTH (11x2 2 bytes)
static uint16_t _calibration_saturation_strength[][2]
 =  {
  { 0, 128 },
  { 256, 128 },
  { 512, 128},
  { 768, 128},
  { 1024, 128},
  { 1280, 100},
  { 1536, 95},
  { 1790,90},
  { 1920, 90},
  { 2048, 85}
};

// CALIBRATION_SCALER_H_FILTER (1x256 4 bytes)
static uint32_t _calibration_scaler_h_filter[]
 = {670499328,194343,704053760,194596,754385408,194593,787939840,194846,821494272,194844,871825920,129561,905380352,129559,938934784,129812,972489216,130065,989331968,130063,1006174720,130316,1023017472,130314,1056637184,65032,1056702720,65286,1073545472,65284,1090387968,2,1073741824,0,1073872896,254,1057292032,509,1040645888,508,1040776704,507,1007353089,762,990706945,761,974126081,760,957479937,759,924121857,759,890763777,759,857340417,759,807205122,759,773781762,759,740423682,759,690288642,759,637271301,16644902,670891013,16710180,687733765,16709923,704576261,16775457,704641798,16775200,704707078,63518,721549830,63261,721615110,128796,738457862,128794,755300358,194073,755431430,194071,755496966,259605,789116677,259604,789182213,325138,789313285,325136,789444101,325135,789444101,390670,789575172,390669,789640708,390923,789771780,390921,789902851,390920,756413955,456711,756545026,456710,756676098,456708,739964673,456963,723318529,456962,723384064,457217,706672640,457216,690026751,457727,690092543,392190,673446398,392445,656734974,392444,504036092,16386590,504101628,16386334,504167419,16386333,520944635,16386077,537787387,16320285,537852923,16320284,537853179,16320028,554695930,16319772,554761466,16319771,554827258,16319514,571604474,16319258,571670266,16319256,571735802,16319000,571736057,16319000,571867129,16318743,571867129,16318743,571867385,16318486,571933177,16383765,571933177,16383765,571998969,16383763,571998969,16449043,571999225,16449042,572130297,16448785,555353337,16448785,555419129,16448528,555484665,16448527,538707705,16514062,538707961,16513806,538773497,16513805,521996538,16513804,505219578,16513804,505285114,16579083,235603458,396046,235603714,396045,235603714,396045,235603714,396045,235669250,395789,235669250,395789,235669251,330253,235669251,330253,235669251,330253,235669507,329997,235669507,329997,235669507,329997,235669507,329997,235669508,264461,235669508,264461,235669764,264460,235669764,264460,235669764,264460,235735300,264204,235735300,264204,235735301,198668,235735301,198668,235735301,198668,235735301,198668,235735557,198412,235735557,198412,235735557,198412,235735558,132876,235735558,132876,235735814,132875,235735814,132875,235735814,132875};

// CALIBRATION_SCALER_V_FILTER (1x256 4 bytes)
static uint32_t _calibration_scaler_v_filter[]
 = {4194304,0,37813760,0,54590721,255,88079361,255,121633537,254,155122177,254,205322497,254,238811393,253,288946177,509,322435073,508,372635649,507,422770689,507,456259585,506,506394625,506,556595201,505,606730241,505,640088321,505,690288897,504,740423937,504,773782017,504,823917057,504,840563457,504,890698497,504,924056577,504,940702977,504,990838016,505,1007484416,505,1040842240,506,1057488384,507,1057357568,508,1074003712,509,1073872896,254,623311869,392699,656734974,392444,673446398,392445,690092543,392190,690026751,457727,706672640,457216,723384064,457217,723318529,456962,739964673,456963,756676098,456708,756545026,456710,756413955,456711,789902851,390920,789771780,390921,789640708,390923,789575172,390669,789509637,390669,789444101,325135,789313285,325136,789182213,325138,789116677,259604,755496966,259605,755431430,194071,755300358,194073,738457862,128794,721615110,128796,721549830,63261,704707078,63518,704641798,16775200,704576261,16775457,687733765,16709923,670891013,16710180,505285370,16579082,505285114,16579083,505219578,16513804,521996538,16513804,538773497,16513805,538707961,16513806,538707705,16514062,555484665,16448527,555419129,16448528,555353337,16448785,572130297,16448785,571999225,16449042,571998969,16449043,571998969,16383763,571933177,16383765,571933177,16383765,571867385,16318486,571867129,16318743,571867129,16318743,571736057,16319000,571735802,16319000,571670266,16319256,571604474,16319258,554827258,16319514,554761466,16319771,554695930,16319772,537853179,16320028,537852923,16320284,537787387,16320285,520944635,16386077,504167419,16386333,504101628,16386334,235801350,132619,235735814,132875,235735814,132875,235735814,132875,235735558,132876,235735558,132876,235735557,198412,235735557,198412,235735557,198412,235735301,198668,235735301,198668,235735301,198668,235735301,198668,235735300,264204,235735300,264204,235669764,264460,235669764,264460,235669764,264460,235669508,264461,235669508,264461,235669507,329997,235669507,329997,235669507,329997,235669507,329997,235669251,330253,235669251,330253,235669251,330253,235669250,395789,235669250,395789,235603714,396045,235603714,396045,235603714,396045};

// CALIBRATION_SHARPEN_DS1 (9x2 2 bytes)
static uint16_t _calibration_sharpen_ds1[][2]
 =  {
  { 0, 70 },
  { 256, 70 },
  { 512, 70 },
  { 768, 70 },
  { 1024, 70 },
  { 1280, 50 },
};

// CALIBRATION_SHARPEN_FR (10x2 2 bytes)
static uint16_t _calibration_sharpen_fr[][2]
 =  {
  { 0, 32 },
  { 256, 32 },
  { 512, 16 },
  { 768, 16 },
  { 1024, 16 },
  { 1280, 24 }
};

// CALIBRATION_SINTER_INTCONFIG (7x2 2 bytes)
static uint16_t _calibration_sinter_intconfig[][2]
 =  {
  { 0, 7 },
  { 256, 7 },
  { 512, 5 },
  { 768, 2 },
  { 1024, 0 },
  { 1280, 0 },
  { 1536, 0 },
  { 1790, 0 },
  { 1920, 0 },
  { 2048, 0 }
};

// CALIBRATION_SINTER_RADIAL_LUT (1x33 1 bytes)
static uint8_t _calibration_sinter_radial_lut[]
 = {0,0,0,0,0,0,1,3,4,6,7,9,10,12,13,15,16,18,19,21,22,24,24,24,24,24,24,24,24,24,24,24,24};

// CALIBRATION_SINTER_RADIAL_PARAMS (1x4 2 bytes)
static uint16_t _calibration_sinter_radial_params[]
 = {0,1920,1080,1770};

// CALIBRATION_SINTER_SAD (7x2 2 bytes)
static uint16_t _calibration_sinter_sad[][2]
 =  {
  { 0, 0 },
  { 256, 0 },
  { 512, 0 },
  { 768, 5 },
  { 1024, 9 },
  { 1280, 11 },
  { 1536, 16 }, 
  { 1790, 16 },
  { 1920, 16 },
  { 2048, 16 },
};

// CALIBRATION_SINTER_STRENGTH (10x2 2 bytes)
static uint16_t _calibration_sinter_strength[][2]
 =  {
  { 0, 40 },
  { 256, 50 },
  { 512, 50 },
  { 768, 65 },
  { 1024, 65 },
  { 1280, 65 }
};

// CALIBRATION_SINTER_STRENGTH_MC_CONTRAST (1x2 2 bytes)
static uint16_t _calibration_sinter_strength_mc_contrast[][2]
 = { 
   {0,0}, 
   {0,0}, 
   {0,0},
   {0,0},
   {0,0},
 } ;

// CALIBRATION_SINTER_STRENGTH1 (11x2 2 bytes)
static uint16_t _calibration_sinter_strength1[][2]
 =  {
  { 0, 55 },
  { 256, 96 },
  { 512, 110 },
  { 768, 125 },
  { 1024, 140 },
  { 1280, 155 }
};

// CALIBRATION_SINTER_THRESH1 (7x2 2 bytes)
static uint16_t _calibration_sinter_thresh1[][2]
 =  {
  { 0, 32 },
  { 256, 64 },
  { 512, 96 },
  { 768, 116 },
  { 1024, 145 },
  { 1280, 145 }
};

// CALIBRATION_SINTER_THRESH4 (10x2 2 bytes)
static uint16_t _calibration_sinter_thresh4[][2]
 =  {
  { 0, 10 },
  { 256, 14 },
  { 512, 16 },
  { 768, 16 },
  { 1024, 16 },
  { 1280, 16 }
};

// CALIBRATION_CNR_UV_DELTA12_SLOPE (8x2 2 bytes)
static uint16_t _calibration_cnr_uv_delta12_slope[][2]
 =  {
  { 0, 1500 },
  { 256, 2000 },
  { 512, 2100 },
  { 768, 2100 },
  { 1024, 3500 },
  { 1280, 4100 },
};

// CALIBRATION_STITCHING_LM_MED_NOISE_INTENSITY (3x2 2 bytes)
static uint16_t _calibration_stitching_lm_med_noise_intensity[][2]
 =  {
  { 0, 32 },
  { 1536, 32 },
  { 2048, 4095 }
};

// CALIBRATION_STITCHING_LM_MOV_MULT (10x2 2 bytes)
static uint16_t _calibration_stitching_lm_mov_mult[][2]
 =  {
  { 0, 2048 },
  { 256, 2048 },
  { 512, 2048 },
  { 768, 2048},
  { 1024, 2048},
  { 1280, 2048},
};

// CALIBRATION_STITCHING_LM_NP (9x2 2 bytes)
static uint16_t _calibration_stitching_lm_np[][2]
 =  {
  { 0, 1536 },
  { 256, 1536 },
  { 512, 1536 },
  { 768, 1536 },
  { 1024, 1536 },
  { 1280, 1536 },
};

// CALIBRATION_STITCHING_MS_MOV_MULT (3x2 2 bytes)
static uint16_t _calibration_stitching_ms_mov_mult[][2]
 =  {
  { 0, 128 },
  { 256, 128 },
  { 512, 100 }
};

// CALIBRATION_STITCHING_MS_NP (3x2 2 bytes)
static uint16_t _calibration_stitching_ms_np[][2]
 =  {
  { 0, 1 },
  { 256, 1 },
  { 512, 1 }
};

// CALIBRATION_STITCHING_SVS_MOV_MULT (3x2 2 bytes)
static uint16_t _calibration_stitching_svs_mov_mult[][2]
 =  {
  { 0, 128 },
  { 256, 128 },
  { 512, 128 }
};

// CALIBRATION_STITCHING_SVS_NP (3x2 2 bytes)
static uint16_t _calibration_stitching_svs_np[][2]
 =  {
  { 0, 3680 },
  { 256, 3680 },
  { 512, 2680 }
};

// CALIBRATION_FS_MC_OFF (1x1 2 bytes)
static uint16_t _calibration_fs_mc_off[]
 = {2048};

// CALIBRATION_TEMPER_STRENGTH (8x2 2 bytes)
static uint16_t _calibration_temper_strength[][2]
 =  {
  { 0, 50 },
  { 256, 60 },
  { 512, 70 },
  { 768, 85 },
  { 1024, 90 },
  { 1280, 95 },
  { 1536, 85 },
  { 1790, 90 },
  { 1920, 90 },
  { 2048, 90 }
};

// CALIBRATION_RGB2YUV_CONVERSION (1x12 2 bytes)
static uint16_t _calibration_rgb2yuv_conversion[]
 = {76,150,29,32811,32852,128,128,32875,32788,0,512,512};
//H264
 //= {76,150,29,32811,32852,128,128,32875,32788,96,512,512};

// CALIBRATION_CUSTOM_SETTINGS_CONTEXT (1x4 4 bytes)
//static uint32_t _calibration_custom_settings_context[]
// = {0,0,0,0};


static uint32_t _calibration_custom_settings_context[][4]
 = {
	//{0x18e8c, 0x00000303, 0xFFFFFFFF, 4},    
	{0x1ae8c, 0x00960171, 0xFFFFFFFF, 4},
	{0x1aeb4, 0x00001EC3, 0xFFFFFFFF, 4},
	{0x1aeb8, 0x00001DFB, 0xFFFFFFFF, 4},
	{0x1aec0, 0x000808BE, 0xFFFFFFFF, 4},
	// for 1lux demosaic
	//0x1aec4, 0x000000C8,
	{0x1aec4, 0x00000014, 0xFFFFFFFF, 4}, 
	{0x1ac30, 0x00000000, 0xFFFFFFFF, 4},
	{0x1ac34, 0x000D5CE7, 0xFFFFFFFF, 4},
	{0x1ac3c, 0x00A30202, 0xFFFFFFFF, 4},
	{0x1ac4c, 0xB4FA0101, 0xFFFFFFFF, 4},//contrast;bright_pr;svariance;filter mux；
	{0x1ac54, 0x000000A4, 0xFFFFFFFF, 4},
	{0x1ac58, 0x00001000, 0xFFFFFFFF, 4},
	{0x19368, 0x00001002, 0xFFFFFFFF, 4},
	{0x19370, 0x00000000, 0xFFFFFFFF, 4},
	{0x19374, 0x00000000, 0xFFFFFFFF, 4},
	{0x19378, 0x00000000, 0xFFFFFFFF, 4},
	{0x1937c, 0x00000000, 0xFFFFFFFF, 4},
	{0x1aa28, 0x00007802, 0xFFFFFFFF, 4},
	{0x1aa30, 0x00000000, 0xFFFFFFFF, 4},
	{0x1aa34, 0x00000000, 0xFFFFFFFF, 4},
	{0x1aa38, 0x00000000, 0xFFFFFFFF, 4},
	{0x18eac, 0x00000004, 0xFFFFFFFF, 4},
	{0x18eb0, 0x00000016, 0xFFFFFFFF, 4},
	{0x18eb8, 0x00000000, 0xFFFFFFFF, 4},
	// enable iridix & iridix_gain
	{0x18ebc, 0x00000025, 0xFFFFFFFF, 4},
	//disable irdix gain and iridix
	//{0x18ebc, 0x00000045, 0xFFFFFFFF, 4},
	// disable CNR
	//{0x18ec0, 0x000000E5, 0xFFFFFFFF, 4},
	// enable CNR
	{0x18ec0, 0x000000C5, 0xFFFFFFFF, 4}, 
	{0x18ec4, 0x00000000, 0xFFFFFFFF, 4},
	{0x18ec8, 0x00000000, 0xFFFFFFFF, 4},
	{0x19364, 0x96005C00, 0xFFFFFFFF, 4},
	// Fr sharpen 580/580
	//{0x1c090, 0x02440244, 0xFFFFFFFF, 4},
	// Fr sharpen 2280/280
	{0x1c090, 0x01180118, 0xFFFFFFFF, 4}, 
	{0x1aa1c, 0x00000003, 0xFFFFFFFF, 4},
	// Temper 3 Delta = 4
	//{0x1aa24, 0x00000402, 0xFFFFFFFF, 4},
	// Temper 3 Delta = 4
	{0x1aa24, 0x00000002, 0xFFFFFFFF, 4},
	// Demosaic
	{0x1ae7c, 0x94bcc3be, 0xFFFFFFFF, 4},
	{0x1ae80, 0x00000080, 0xFFFFFFFF, 4},
	{0x1ae84, 0x001e00b4, 0xFFFFFFFF, 4},
	{0x1ae88, 0x0000018b, 0xFFFFFFFF, 4},
	{0x1ae8c, 0x00960171, 0xFFFFFFFF, 4},
	{0x1ae90, 0x08000800, 0xFFFFFFFF, 4},
	{0x1ae94, 0x00000800, 0xFFFFFFFF, 4},
	{0x1ae9c, 0x0001004b, 0xFFFFFFFF, 4},
	{0x1aea0, 0x00000000, 0xFFFFFFFF, 4},
	{0x1aea4, 0x00cf01b3, 0xFFFFFFFF, 4},
	//{0x1aea8, 0x000041A5, 0xFFFFFFFF, 4},
	{0x1aea8, 0x00005587, 0xFFFFFFFF, 4},
	{0x1aeac, 0x00000000, 0xFFFFFFFF, 4},
	{0x1aeb0, 0x10106100, 0xFFFFFFFF, 4},
	{0x1aeb4, 0x00001f59, 0xFFFFFFFF, 4},
	{0x1aeb8, 0x00001fff, 0xFFFFFFFF, 4},
	{0x1aebc, 0x00000001, 0xFFFFFFFF, 4},
	{0x1aec0, 0x0014b4c8, 0xFFFFFFFF, 4},
	{0x1aec4, 0x00000014, 0xFFFFFFFF, 4},
	{0x1aec8, 0xffff9085, 0xFFFFFFFF, 4},
	{0x1aecc, 0x01000100, 0xFFFFFFFF, 4},//max_d_strength [12:0]
	{0x1aed0, 0x00000010, 0xFFFFFFFF, 4},//luma_thresh_low_d
	{0x1aed4, 0x00004000, 0xFFFFFFFF, 4},
	{0x1aed8, 0x0fa00000, 0xFFFFFFFF, 4},
	{0x1aedc, 0x00004000, 0xFFFFFFFF, 4},
	{0x1aee0, 0x00000010, 0xFFFFFFFF, 4},//luma_thresh_low_ud [11:0];luma_offset_low_ud [7:0] 
	{0x1aee4, 0x00004000, 0xFFFFFFFF, 4},
	{0x1aee8, 0x00000fa0, 0xFFFFFFFF, 4},
	{0x1aeec, 0x00004000, 0xFFFFFFFF, 4},
	// CNR
	{0x1b0d0, 0x00000001, 0xFFFFFFFF, 4},
	{0x1b0d4, 0x00000001, 0xFFFFFFFF, 4},
	{0x1b0d8, 0x00000000, 0xFFFFFFFF, 4},
	{0x1b0dc, 0x00000000, 0xFFFFFFFF, 4},
	{0x1b0e0, 0x000000FF, 0xFFFFFFFF, 4},
	{0x1b0e4, 0x0000003F, 0xFFFFFFFF, 4},
	{0x1b0e8, 0x00000800, 0xFFFFFFFF, 4},
	{0x1b0ec, 0x00000800, 0xFFFFFFFF, 4},
	{0x1b0f0, 0x00000332, 0xFFFFFFFF, 4},
	{0x1b0f4, 0x000000CD, 0xFFFFFFFF, 4},
	{0x1b0f8, 0x00000000, 0xFFFFFFFF, 4},
	{0x1b0fc, 0x000002B2, 0xFFFFFFFF, 4},
	{0x1b100, 0x0000FD69, 0xFFFFFFFF, 4},
	{0x1b104, 0x00000000, 0xFFFFFFFF, 4},
	{0x1b108, 0x000000DC, 0xFFFFFFFF, 4},
	{0x1b10c, 0x0000E4E1, 0xFFFFFFFF, 4},
	{0x1b110, 0x00000000, 0xFFFFFFFF, 4},
	{0x1b114, 0x000000DC, 0xFFFFFFFF, 4},
	{0x1b118, 0x0000E4E1, 0xFFFFFFFF, 4},
	{0x1b11c, 0x00000000, 0xFFFFFFFF, 4},
	{0x1b120, 0x000000DC, 0xFFFFFFFF, 4},
	{0x1b124, 0x0000E4E1, 0xFFFFFFFF, 4},
	{0x1b128, 0x00000000, 0xFFFFFFFF, 4},
	{0x1b12c, 0x000000DC, 0xFFFFFFFF, 4},
	{0x1b130, 0x0000E4E1, 0xFFFFFFFF, 4},
	{0x1b134, 0x00000000, 0xFFFFFFFF, 4},
	{0x1b138, 0x000003FF, 0xFFFFFFFF, 4},
	{0x1b13c, 0x0000FF00, 0xFFFFFFFF, 4},
	{0x1b140, 0x00000000, 0xFFFFFFFF, 4},
	{0x1b144, 0x000003FF, 0xFFFFFFFF, 4},
	{0x1b148, 0x0000FF00, 0xFFFFFFFF, 4},
	{0x1b14c, 0x00001010, 0xFFFFFFFF, 4},
	{0x1b150, 0x00000000, 0xFFFFFFFF, 4},
	{0x1b154, 0x000000F0, 0xFFFFFFFF, 4},
	{0x1b158, 0x0000FFFF, 0xFFFFFFFF, 4},
	{0x1b15c, 0x00000000, 0xFFFFFFFF, 4},
	{0x1b160, 0x000000F0, 0xFFFFFFFF, 4},
	{0x1b164, 0x0000FFFF, 0xFFFFFFFF, 4},
	// pf correction
	{0x1afd4, 0x00000FFF, 0xFFFFFFFF, 4},
	// DOL2 Frame Stitch
	{0x18ff0, 0x0DF40F28, 0xFFFFFFFF, 4},
	{0x1904c, 0x00010002, 0xFFFFFFFF, 4},
	// iridix contrast and bright_pr
	//	{0x1ac4c, 0xB4FF0001, 0xFFFFFFFF, 4},
	//	{0x1aec0, 0x0050B4BE, 0xFFFFFFFF, 4},
	//	{0x1aec4, 0x00000000, 0xFFFFFFFF, 4},

	{0,0,0,0},
};

static uint16_t _calibration_gamma_ev1[]
 = {0,305,509,634,807,964,1112,1261,1387,1508,1600,1670,1734,1799,1854,1911,1962,2001,2035,2070,2105,2144,2182,2221,2266,2300,2340,2372,2398,2424,2450,2474,2502,2526,2550,2573,2598,2622,2647,2673,2696,2719,2748,2771,2796,2820,2844,2869,2888,2912,2930,2952,2971,2988,3008,3024,3041,3056,3072,3087,3098,3114,3128,3139,3154,3164,3178,3188,3201,3214,3224,3236,3248,3256,3268,3279,3287,3298,3307,3319,3327,3340,3349,3361,3371,3384,3395,3408,3419,3430,3445,3456,3471,3482,3494,3510,3522,3534,3546,3556,3568,3581,3595,3608,3620,3633,3645,3658,3672,3688,3697,3713,3729,3747,3765,3785,3804,3819,3840,3864,3887,3903,3929,3956,3974,4002,4020,4048,4095};
 //= {0,158,367,557,719,847,973,1103,1217,1335,1435,1517,1596,1679,1749,1821,1882,1930,1969,2010,2051,2097,2141,2185,2237,2275,2322,2357,2387,2416,2446,2473,2504,2531,2558,2584,2611,2638,2666,2695,2720,2746,2778,2804,2832,2858,2885,2913,2934,2960,2981,3006,3027,3046,3069,3088,3107,3125,3143,3160,3173,3191,3207,3220,3237,3249,3265,3276,3291,3305,3316,3330,3343,3352,3365,3377,3386,3398,3408,3421,3429,3443,3453,3466,3477,3490,3502,3515,3526,3537,3552,3563,3577,3588,3599,3614,3625,3637,3648,3657,3668,3680,3693,3704,3715,3727,3738,3749,3761,3775,3783,3797,3811,3826,3842,3858,3874,3886,3904,3923,3941,3954,3974,3994,4008,4029,4042,4062,4095};

// Gamma Rec.709
 //= {0, 144, 289, 426, 541, 641, 730, 812, 887, 957, 1024, 1086, 1146, 1203, 1257, 1310, 1360, 1409, 1456, 1502, 1547, 1590, 1632, 1673, 1714, 1753, 1791, 1829, 1866, 1902, 1938, 1972, 2007, 2040, 2073, 2106, 2138, 2169, 2200, 2231, 2261, 2291, 2321, 2350, 2378, 2406, 2434, 2462, 2489, 2516, 2543, 2569, 2596, 2621, 2647, 2672, 2697, 2722, 2747, 2771, 2795, 2819, 2843, 2866, 2889, 2913, 2935, 2958, 2981, 3003, 3025, 3047, 3069, 3090, 3112, 3133, 3154, 3175, 3196, 3217, 3237, 3258, 3278, 3298, 3318, 3338, 3358, 3378, 3397, 3417, 3436, 3455, 3474, 3493, 3512, 3530, 3549, 3567, 3586, 3604, 3622, 3640, 3658, 3676, 3694, 3712, 3729, 3747, 3764, 3782, 3799, 3816, 3833, 3850, 3867, 3884, 3900, 3917, 3934, 3950, 3967, 3983, 3999, 4016, 4032, 4048, 4064, 4080, 4095};


static uint16_t _calibration_gamma_ev2[] 
 = {0,233,452,698,888,1045,1216,1358,1457,1556,1637,1702,1764,1829,1885,1943,1994,2033,2068,2102,2136,2174,2211,2248,2292,2324,2363,2393,2418,2443,2468,2491,2518,2541,2564,2586,2610,2633,2657,2682,2704,2726,2754,2777,2801,2824,2848,2872,2891,2914,2932,2954,2972,2989,3009,3025,3042,3057,3073,3088,3099,3115,3129,3140,3155,3165,3179,3189,3202,3215,3225,3237,3249,3257,3269,3280,3288,3299,3308,3320,3328,3341,3350,3363,3373,3386,3397,3410,3421,3432,3447,3458,3473,3484,3496,3512,3524,3536,3548,3558,3570,3583,3597,3610,3622,3635,3647,3660,3674,3690,3699,3715,3731,3749,3767,3787,3806,3821,3842,3866,3889,3905,3931,3958,3976,4004,4022,4050,4095};
// =  {0, 400, 627, 760, 937, 1079, 1234, 1386, 1521, 1647, 1759, 1855, 1942, 2022, 2094, 2164, 2231, 2291, 2351, 2404, 2456, 2506, 2553, 2597, 2639, 2680, 2720, 2758, 2795, 2831, 2866, 2899, 2931, 2963, 2994, 3024, 3053, 3082, 3111, 3139, 3167, 3194, 3221, 3246, 3270, 3292, 3315, 3338, 3359, 3380, 3402, 3424, 3446, 3466, 3488, 3509, 3530, 3551, 3571, 3591, 3608, 3626, 3644, 3659, 3674, 3690, 3703, 3718, 3730, 3744, 3757, 3769, 3781, 3792, 3804, 3815, 3826, 3837, 3848, 3858, 3868, 3878, 3888, 3896, 3904, 3912, 3920, 3927, 3933, 3940, 3946, 3951, 3957, 3962, 3967, 3971, 3975, 3979, 3983, 3987, 3990, 3993, 3997, 4000, 4003, 4007, 4011, 4014, 4018, 4022, 4026, 4030, 4034, 4038, 4042, 4047, 4051, 4055, 4059, 4063, 4067, 4071, 4074, 4078, 4082, 4085, 4089, 4092, 4095}; 
//{0, 259, 440, 590, 719, 832, 936, 1032, 1123, 1208, 1290, 1367, 1440, 1511, 1577, 1641, 1704, 1765, 1825, 1884, 1943, 2001, 2060, 2116, 2170, 2221, 2268, 2314, 2358, 2401, 2442, 2483, 2522, 2559, 2596, 2630, 2665, 2697, 2729, 2760, 2790, 2819, 2848, 2875, 2903, 2929, 2954, 2979, 3003, 3027, 3049, 3071, 3092, 3114, 3134, 3155, 3173, 3192, 3211, 3229, 3246, 3263, 3279, 3295, 3310, 3325, 3339, 3353, 3366, 3379, 3392, 3405, 3417, 3430, 3442, 3456, 3468, 3481, 3495, 3507, 3521, 3534, 3547, 3560, 3573, 3587, 3600, 3612, 3625, 3638, 3651, 3663, 3675, 3687, 3699, 3711, 3722, 3733, 3744, 3755, 3765, 3775, 3785, 3795, 3805, 3815, 3826, 3838, 3848, 3860, 3871, 3883, 3895, 3907, 3918, 3930, 3943, 3955, 3967, 3979, 3992, 4004, 4017, 4030, 4043, 4056, 4069, 4082, 4095};


static int32_t _calibration_gamma_threshold[] 
 = {3950000,4404885,1};



// CALIBRATION_STATUS_INFO (1x5 4 bytes)
static uint32_t _calibration_status_info[]
 = {262144,3928235,2304,14966,0};

static uint32_t _calibration_zoom_lms[] = {
13, 0, 180, 361, 541, 722, 902, 1082, 1263, 1443, 1624, 1804, 1985, 2165, 2275
};

static uint32_t _calibration_zoom_af_lms[][21] = {
{166400/4, 166400/4, 166400/4, 167680/4, 167680/4, 167680/4,176000/4,176000/4,176000/4,177920/4,177920/4,177920/4,11,6,2,30,131072,131072,262144,65536, 0},
{136320/4, 136320/4, 136320/4, 137600/4, 137600/4, 137600/4,147200/4,147200/4,147200/4,149120/4,149120/4,149120/4,11,6,2,30,131072,131072,262144,65536, 0},
{110720/4, 110720/4, 110720/4, 112000/4, 112000/4, 112000/4,120320/4,120320/4,120320/4,122240/4,122240/4,122240/4,11,6,2,30,131072,131072,262144,65536, 0},
{88960/4, 88960/4, 88960/4, 90240/4, 90240/4, 90240/4, 99200/4, 99200/4, 99200/4,101120/4,101120/4,101120/4,11,6,2,30,131072,131072,262144,65536, 0},
{72320/4, 72320/4, 72320/4, 73600/4, 73600/4, 73600/4, 81920/4, 81920/4, 81920/4, 83840/4, 83840/4, 83840/4,11,6,2,30,131072,131072,262144,65536, 0},
{58240/4, 58240/4, 58240/4, 59520/4, 59520/4, 59520/4, 67840/4, 67840/4, 67840/4, 69760/4, 69760/4, 69760/4,11,6,2,30,131072,131072,262144,65536, 0},
{46080/4, 46080/4, 46080/4, 47360/4, 47360/4, 47360/4, 56960/4, 56960/4, 56960/4, 58880/4, 58880/4, 58880/4,11,6,2,30,131072,131072,262144,65536, 0},
{36480/4, 36480/4, 36480/4, 37760/4, 37760/4, 37760/4, 46720/4, 46720/4, 46720/4, 48640/4, 48640/4, 48640/4,11,6,2,30,131072,131072,262144,65536, 0},
{28800/4, 28800/4, 28800/4, 30080/4, 30080/4, 30080/4, 39040/4, 39040/4, 39040/4, 40960/4, 40960/4, 40960/4,11,6,2,30,131072,131072,262144,65536, 0},
{23040/4, 23040/4, 23040/4, 24320/4, 24320/4, 24320/4, 33280/4, 33280/4, 33280/4, 35200/4, 35200/4, 35200/4,11,6,2,30,131072,131072,262144,65536, 0},
{18560/4, 18560/4, 18560/4, 19840/4, 19840/4, 19840/4, 28800/4, 28800/4, 28800/4, 30720/4, 30720/4, 30720/4,11,6,2,30,131072,131072,262144,65536, 0},
{14720/4, 14720/4, 14720/4, 16000/4, 16000/4, 16000/4, 24960/4, 24960/4, 24960/4, 26880/4, 26880/4, 26880/4,11,6,2,30,131072,131072,262144,65536, 0},
{11520/4, 11520/4, 11520/4, 12800/4, 12800/4, 12800/4, 22400/4, 22400/4, 22400/4, 24320/4, 24320/4, 24320/4,11,6,2,30,131072,131072,262144,65536, 0}
};


static uint16_t _calibration_demosaic_uu_slope[][2] = {
    {0, 130},     //255
    {256, 130},     //255
    {512, 130},    //255
    {768, 130},    //255
    {1024, 130},    //255 4 int
    {1280, 130},    //255 4 int
    {1536, 130},    //255 4 int
    {1790, 130},    //255 4 int
    {1920, 130},    //255 4 int
    {2048, 130},    //255 4 int
};

static int16_t _calibration_bypass_control[] = {1, 0, 1200, 1024};



static LookupTable calibration_ae_control = { .ptr = _calibration_ae_control, .rows = 1, .cols = sizeof(_calibration_ae_control) / sizeof(_calibration_ae_control[0]), .width = sizeof(_calibration_ae_control[0] ) };
static LookupTable calibration_ae_control_hdr_target = { .ptr = _calibration_ae_control_hdr_target, .cols = 2, .rows = sizeof(_calibration_ae_control_hdr_target) / sizeof(_calibration_ae_control_hdr_target[0]), .width = sizeof(_calibration_ae_control_hdr_target[0][0] ) };
static LookupTable calibration_ae_correction = { .ptr = _calibration_ae_correction, .rows = 1, .cols = sizeof(_calibration_ae_correction) / sizeof(_calibration_ae_correction[0]), .width = sizeof(_calibration_ae_correction[0] ) };
static LookupTable calibration_ae_exposure_correction = { .ptr = _calibration_ae_exposure_correction, .rows = 1, .cols = sizeof(_calibration_ae_exposure_correction) / sizeof(_calibration_ae_exposure_correction[0]), .width = sizeof(_calibration_ae_exposure_correction[0] ) };
static LookupTable calibration_ae_zone_wght_hor = { .ptr = _calibration_ae_zone_wght_hor, .rows = 1, .cols = sizeof(_calibration_ae_zone_wght_hor) / sizeof(_calibration_ae_zone_wght_hor[0]), .width = sizeof(_calibration_ae_zone_wght_hor[0] ) };
static LookupTable calibration_ae_zone_wght_ver = { .ptr = _calibration_ae_zone_wght_ver, .rows = 1, .cols = sizeof(_calibration_ae_zone_wght_ver) / sizeof(_calibration_ae_zone_wght_ver[0]), .width = sizeof(_calibration_ae_zone_wght_ver[0] ) };
static LookupTable calibration_exposure_ratio_adjustment = { .ptr = _calibration_exposure_ratio_adjustment, .cols = 2, .rows = sizeof(_calibration_exposure_ratio_adjustment) / sizeof(_calibration_exposure_ratio_adjustment[0]), .width = sizeof(_calibration_exposure_ratio_adjustment[0][0] ) };
static LookupTable calibration_cmos_control = { .ptr = _calibration_cmos_control, .rows = 1, .cols = sizeof(_calibration_cmos_control) / sizeof(_calibration_cmos_control[0]), .width = sizeof(_calibration_cmos_control[0] ) };
static LookupTable calibration_cmos_exposure_partition_luts = { .ptr = _calibration_cmos_exposure_partition_luts, .cols = 10, .rows = sizeof(_calibration_cmos_exposure_partition_luts) / sizeof(_calibration_cmos_exposure_partition_luts[0]), .width = sizeof(_calibration_cmos_exposure_partition_luts[0][0] ) };
static LookupTable calibration_af_lms = { .ptr = _calibration_af_lms, .rows = 1, .cols = sizeof(_calibration_af_lms) / sizeof(_calibration_af_lms[0]), .width = sizeof(_calibration_af_lms[0] ) };
static LookupTable calibration_af_zone_wght_hor = { .ptr = _calibration_af_zone_wght_hor, .rows = 1, .cols = sizeof(_calibration_af_zone_wght_hor) / sizeof(_calibration_af_zone_wght_hor[0]), .width = sizeof(_calibration_af_zone_wght_hor[0] ) };
static LookupTable calibration_af_zone_wght_ver = { .ptr = _calibration_af_zone_wght_ver, .rows = 1, .cols = sizeof(_calibration_af_zone_wght_ver) / sizeof(_calibration_af_zone_wght_ver[0]), .width = sizeof(_calibration_af_zone_wght_ver[0] ) };
static LookupTable calibration_awb_avg_coef = { .ptr = _calibration_awb_avg_coef, .rows = 1, .cols = sizeof(_calibration_awb_avg_coef) / sizeof(_calibration_awb_avg_coef[0]), .width = sizeof(_calibration_awb_avg_coef[0] ) };
static LookupTable calibration_awb_bg_max_gain = { .ptr = _calibration_awb_bg_max_gain, .cols = 2, .rows = sizeof(_calibration_awb_bg_max_gain) / sizeof(_calibration_awb_bg_max_gain[0]), .width = sizeof(_calibration_awb_bg_max_gain[0][0] ) };
static LookupTable awb_colour_preference = { .ptr = _awb_colour_preference, .rows = 1, .cols = sizeof(_awb_colour_preference) / sizeof(_awb_colour_preference[0]), .width = sizeof(_awb_colour_preference[0] ) };
static LookupTable calibration_awb_mix_light_parameters = { .ptr = _calibration_awb_mix_light_parameters, .rows = 1, .cols = sizeof(_calibration_awb_mix_light_parameters) / sizeof(_calibration_awb_mix_light_parameters[0]), .width = sizeof(_calibration_awb_mix_light_parameters[0] ) };
static LookupTable calibration_awb_zone_wght_hor = { .ptr = _calibration_awb_zone_wght_hor, .rows = 1, .cols = sizeof(_calibration_awb_zone_wght_hor) / sizeof(_calibration_awb_zone_wght_hor[0]), .width = sizeof(_calibration_awb_zone_wght_hor[0] ) };
static LookupTable calibration_awb_zone_wght_ver = { .ptr = _calibration_awb_zone_wght_ver, .rows = 1, .cols = sizeof(_calibration_awb_zone_wght_ver) / sizeof(_calibration_awb_zone_wght_ver[0]), .width = sizeof(_calibration_awb_zone_wght_ver[0] ) };
static LookupTable calibration_evtolux_probability_enable = { .ptr = _calibration_evtolux_probability_enable, .rows = 1, .cols = sizeof(_calibration_evtolux_probability_enable) / sizeof(_calibration_evtolux_probability_enable[0]), .width = sizeof(_calibration_evtolux_probability_enable[0] ) };
static LookupTable calibration_auto_level_control = { .ptr = _calibration_auto_level_control, .rows = 1, .cols = sizeof(_calibration_auto_level_control) / sizeof(_calibration_auto_level_control[0]), .width = sizeof(_calibration_auto_level_control[0] ) };
static LookupTable calibration_ccm_one_gain_threshold = { .ptr = _calibration_ccm_one_gain_threshold, .rows = 1, .cols = sizeof(_calibration_ccm_one_gain_threshold) / sizeof(_calibration_ccm_one_gain_threshold[0]), .width = sizeof(_calibration_ccm_one_gain_threshold[0] ) };
static LookupTable calibration_demosaic_np_offset = { .ptr = _calibration_demosaic_np_offset, .cols = 2, .rows = sizeof(_calibration_demosaic_np_offset) / sizeof(_calibration_demosaic_np_offset[0]), .width = sizeof(_calibration_demosaic_np_offset[0][0] ) };
static LookupTable calibration_sharp_alt_d = { .ptr = _calibration_sharp_alt_d, .cols = 2, .rows = sizeof(_calibration_sharp_alt_d) / sizeof(_calibration_sharp_alt_d[0]), .width = sizeof(_calibration_sharp_alt_d[0][0] ) };
static LookupTable calibration_sharp_alt_du = { .ptr = _calibration_sharp_alt_du, .cols = 2, .rows = sizeof(_calibration_sharp_alt_du) / sizeof(_calibration_sharp_alt_du[0]), .width = sizeof(_calibration_sharp_alt_du[0][0] ) };
static LookupTable calibration_sharp_alt_ud = { .ptr = _calibration_sharp_alt_ud, .cols = 2, .rows = sizeof(_calibration_sharp_alt_ud) / sizeof(_calibration_sharp_alt_ud[0]), .width = sizeof(_calibration_sharp_alt_ud[0][0] ) };
static LookupTable calibration_dp_slope = { .ptr = _calibration_dp_slope, .cols = 2, .rows = sizeof(_calibration_dp_slope) / sizeof(_calibration_dp_slope[0]), .width = sizeof(_calibration_dp_slope[0][0] ) };
static LookupTable calibration_dp_threshold = { .ptr = _calibration_dp_threshold, .cols = 2, .rows = sizeof(_calibration_dp_threshold) / sizeof(_calibration_dp_threshold[0]), .width = sizeof(_calibration_dp_threshold[0][0] ) };
static LookupTable calibration_iridix_avg_coef = { .ptr = _calibration_iridix_avg_coef, .rows = 1, .cols = sizeof(_calibration_iridix_avg_coef) / sizeof(_calibration_iridix_avg_coef[0]), .width = sizeof(_calibration_iridix_avg_coef[0] ) };
static LookupTable calibration_iridix_ev_lim_full_str = { .ptr = _calibration_iridix_ev_lim_full_str, .rows = 1, .cols = sizeof(_calibration_iridix_ev_lim_full_str) / sizeof(_calibration_iridix_ev_lim_full_str[0]), .width = sizeof(_calibration_iridix_ev_lim_full_str[0] ) };
static LookupTable calibration_iridix_ev_lim_no_str = { .ptr = _calibration_iridix_ev_lim_no_str, .rows = 1, .cols = sizeof(_calibration_iridix_ev_lim_no_str) / sizeof(_calibration_iridix_ev_lim_no_str[0]), .width = sizeof(_calibration_iridix_ev_lim_no_str[0] ) };
static LookupTable calibration_iridix_min_max_str = { .ptr = _calibration_iridix_min_max_str, .rows = 1, .cols = sizeof(_calibration_iridix_min_max_str) / sizeof(_calibration_iridix_min_max_str[0]), .width = sizeof(_calibration_iridix_min_max_str[0] ) };
static LookupTable calibration_iridix_strength_maximum = { .ptr = _calibration_iridix_strength_maximum, .rows = 1, .cols = sizeof(_calibration_iridix_strength_maximum) / sizeof(_calibration_iridix_strength_maximum[0]), .width = sizeof(_calibration_iridix_strength_maximum[0] ) };
static LookupTable calibration_iridix8_strength_dk_enh_control = { .ptr = _calibration_iridix8_strength_dk_enh_control, .rows = 1, .cols = sizeof(_calibration_iridix8_strength_dk_enh_control) / sizeof(_calibration_iridix8_strength_dk_enh_control[0]), .width = sizeof(_calibration_iridix8_strength_dk_enh_control[0] ) };

static LookupTable calibration_mesh_shading_strength = { .ptr = _calibration_mesh_shading_strength, .cols = 2, .rows= sizeof(_calibration_mesh_shading_strength) / sizeof(_calibration_mesh_shading_strength[0]), .width = sizeof(_calibration_mesh_shading_strength[0][0] ) };

static LookupTable calibration_pf_radial_lut = { .ptr = _calibration_pf_radial_lut, .rows = 1, .cols = sizeof(_calibration_pf_radial_lut) / sizeof(_calibration_pf_radial_lut[0]), .width = sizeof(_calibration_pf_radial_lut[0] ) };
static LookupTable calibration_pf_radial_params = { .ptr = _calibration_pf_radial_params, .rows = 1, .cols = sizeof(_calibration_pf_radial_params) / sizeof(_calibration_pf_radial_params[0]), .width = sizeof(_calibration_pf_radial_params[0] ) };
static LookupTable calibration_saturation_strength = { .ptr = _calibration_saturation_strength, .cols = 2, .rows = sizeof(_calibration_saturation_strength) / sizeof(_calibration_saturation_strength[0]), .width = sizeof(_calibration_saturation_strength[0][0] ) };
static LookupTable calibration_scaler_h_filter = { .ptr = _calibration_scaler_h_filter, .rows = 1, .cols = sizeof(_calibration_scaler_h_filter) / sizeof(_calibration_scaler_h_filter[0]), .width = sizeof(_calibration_scaler_h_filter[0] ) };
static LookupTable calibration_scaler_v_filter = { .ptr = _calibration_scaler_v_filter, .rows = 1, .cols = sizeof(_calibration_scaler_v_filter) / sizeof(_calibration_scaler_v_filter[0]), .width = sizeof(_calibration_scaler_v_filter[0] ) };
static LookupTable calibration_sharpen_ds1 = { .ptr = _calibration_sharpen_ds1, .cols = 2, .rows = sizeof(_calibration_sharpen_ds1) / sizeof(_calibration_sharpen_ds1[0]), .width = sizeof(_calibration_sharpen_ds1[0][0] ) };
static LookupTable calibration_sharpen_fr = { .ptr = _calibration_sharpen_fr, .cols = 2, .rows = sizeof(_calibration_sharpen_fr) / sizeof(_calibration_sharpen_fr[0]), .width = sizeof(_calibration_sharpen_fr[0][0] ) };
static LookupTable calibration_sinter_intconfig = { .ptr = _calibration_sinter_intconfig, .cols = 2, .rows = sizeof(_calibration_sinter_intconfig) / sizeof(_calibration_sinter_intconfig[0]), .width = sizeof(_calibration_sinter_intconfig[0][0] ) };
static LookupTable calibration_sinter_radial_lut = { .ptr = _calibration_sinter_radial_lut, .rows = 1, .cols = sizeof(_calibration_sinter_radial_lut) / sizeof(_calibration_sinter_radial_lut[0]), .width = sizeof(_calibration_sinter_radial_lut[0] ) };
static LookupTable calibration_sinter_radial_params = { .ptr = _calibration_sinter_radial_params, .rows = 1, .cols = sizeof(_calibration_sinter_radial_params) / sizeof(_calibration_sinter_radial_params[0]), .width = sizeof(_calibration_sinter_radial_params[0] ) };
static LookupTable calibration_sinter_sad = { .ptr = _calibration_sinter_sad, .cols = 2, .rows = sizeof(_calibration_sinter_sad) / sizeof(_calibration_sinter_sad[0]), .width = sizeof(_calibration_sinter_sad[0][0] ) };
static LookupTable calibration_sinter_strength = { .ptr = _calibration_sinter_strength, .cols = 2, .rows = sizeof(_calibration_sinter_strength) / sizeof(_calibration_sinter_strength[0]), .width = sizeof(_calibration_sinter_strength[0][0] ) };
static LookupTable calibration_sinter_strength_mc_contrast = {.ptr =_calibration_sinter_strength_mc_contrast, .rows = sizeof(_calibration_sinter_strength_mc_contrast)/sizeof(_calibration_sinter_strength_mc_contrast[0]), .cols = 2, .width = sizeof(_calibration_sinter_strength_mc_contrast[0][0])};
static LookupTable calibration_sinter_strength1 = { .ptr = _calibration_sinter_strength1, .cols = 2, .rows = sizeof(_calibration_sinter_strength1) / sizeof(_calibration_sinter_strength1[0]), .width = sizeof(_calibration_sinter_strength1[0][0] ) };
static LookupTable calibration_sinter_thresh1 = { .ptr = _calibration_sinter_thresh1, .cols = 2, .rows = sizeof(_calibration_sinter_thresh1) / sizeof(_calibration_sinter_thresh1[0]), .width = sizeof(_calibration_sinter_thresh1[0][0] ) };
static LookupTable calibration_sinter_thresh4 = { .ptr = _calibration_sinter_thresh4, .cols = 2, .rows = sizeof(_calibration_sinter_thresh4) / sizeof(_calibration_sinter_thresh4[0]), .width = sizeof(_calibration_sinter_thresh4[0][0] ) };
static LookupTable calibration_cnr_uv_delta12_slope = { .ptr = _calibration_cnr_uv_delta12_slope, .cols = 2, .rows = sizeof(_calibration_cnr_uv_delta12_slope) / sizeof(_calibration_cnr_uv_delta12_slope[0]), .width = sizeof(_calibration_cnr_uv_delta12_slope[0][0] ) };
static LookupTable calibration_stitching_lm_med_noise_intensity = { .ptr = _calibration_stitching_lm_med_noise_intensity, .cols = 2, .rows = sizeof(_calibration_stitching_lm_med_noise_intensity) / sizeof(_calibration_stitching_lm_med_noise_intensity[0]), .width = sizeof(_calibration_stitching_lm_med_noise_intensity[0][0] ) };
static LookupTable calibration_stitching_lm_mov_mult = { .ptr = _calibration_stitching_lm_mov_mult, .cols = 2, .rows = sizeof(_calibration_stitching_lm_mov_mult) / sizeof(_calibration_stitching_lm_mov_mult[0]), .width = sizeof(_calibration_stitching_lm_mov_mult[0][0] ) };
static LookupTable calibration_stitching_lm_np = { .ptr = _calibration_stitching_lm_np, .cols = 2, .rows = sizeof(_calibration_stitching_lm_np) / sizeof(_calibration_stitching_lm_np[0]), .width = sizeof(_calibration_stitching_lm_np[0][0] ) };
static LookupTable calibration_stitching_ms_mov_mult = { .ptr = _calibration_stitching_ms_mov_mult, .cols = 2, .rows = sizeof(_calibration_stitching_ms_mov_mult) / sizeof(_calibration_stitching_ms_mov_mult[0]), .width = sizeof(_calibration_stitching_ms_mov_mult[0][0] ) };
static LookupTable calibration_stitching_ms_np = { .ptr = _calibration_stitching_ms_np, .cols = 2, .rows = sizeof(_calibration_stitching_ms_np) / sizeof(_calibration_stitching_ms_np[0]), .width = sizeof(_calibration_stitching_ms_np[0][0] ) };
static LookupTable calibration_stitching_svs_mov_mult = { .ptr = _calibration_stitching_svs_mov_mult, .cols = 2, .rows = sizeof(_calibration_stitching_svs_mov_mult) / sizeof(_calibration_stitching_svs_mov_mult[0]), .width = sizeof(_calibration_stitching_svs_mov_mult[0][0] ) };
static LookupTable calibration_stitching_svs_np = { .ptr = _calibration_stitching_svs_np, .cols = 2, .rows = sizeof(_calibration_stitching_svs_np) / sizeof(_calibration_stitching_svs_np[0]), .width = sizeof(_calibration_stitching_svs_np[0][0] ) };
static LookupTable calibration_fs_mc_off = { .ptr = _calibration_fs_mc_off, .rows = 1, .cols = sizeof(_calibration_fs_mc_off) / sizeof(_calibration_fs_mc_off[0]), .width = sizeof(_calibration_fs_mc_off[0] ) };
static LookupTable calibration_temper_strength = { .ptr = _calibration_temper_strength, .cols = 2, .rows = sizeof(_calibration_temper_strength) / sizeof(_calibration_temper_strength[0]), .width = sizeof(_calibration_temper_strength[0][0] ) };
static LookupTable calibration_rgb2yuv_conversion = { .ptr = _calibration_rgb2yuv_conversion, .rows = 1, .cols = sizeof(_calibration_rgb2yuv_conversion) / sizeof(_calibration_rgb2yuv_conversion[0]), .width = sizeof(_calibration_rgb2yuv_conversion[0] ) };
static LookupTable calibration_custom_settings_context = { .ptr = _calibration_custom_settings_context, .rows = 1, .cols = sizeof(_calibration_custom_settings_context) / sizeof(_calibration_custom_settings_context[0]), .width = sizeof(_calibration_custom_settings_context[0] ) };
//static LookupTable calibration_custom_settings_context = { .ptr = _calibration_custom_settings_context, .cols = 4, .rows= sizeof(_calibration_custom_settings_context) / sizeof(_calibration_custom_settings_context[0]), .width = sizeof(_calibration_custom_settings_context[0][0] ) };
//static LookupTable calibration_custom_settings_context = {.ptr = _calibration_custom_settings_context, .rows = sizeof( _calibration_custom_settings_context ) / sizeof( _calibration_custom_settings_context[0] ), .cols = 4, .width = sizeof( _calibration_custom_settings_context[0][0] )};
static LookupTable calibration_status_info = { .ptr = _calibration_status_info, .rows = 1, .cols = sizeof(_calibration_status_info) / sizeof(_calibration_status_info[0]), .width = sizeof(_calibration_status_info[0] ) };
static LookupTable calibration_zoom_lms = {.ptr = _calibration_zoom_lms, .rows = 1, .cols = sizeof(_calibration_zoom_lms)/sizeof(_calibration_zoom_lms[0]), .width = sizeof( _calibration_zoom_lms[0])};
static LookupTable calibration_zoom_af_lms = {.ptr = _calibration_zoom_af_lms, .rows = sizeof( _calibration_zoom_af_lms ) / sizeof( _calibration_zoom_af_lms[0] ), .cols = 21, .width = sizeof( _calibration_zoom_af_lms[0][0] )};


static LookupTable calibration_gamma_ev1 = {.ptr = _calibration_gamma_ev1, .rows = 1, .cols = sizeof( _calibration_gamma_ev1 ) / sizeof( _calibration_gamma_ev1[0] ), .width = sizeof( _calibration_gamma_ev1[0] )};
static LookupTable calibration_gamma_ev2 = {.ptr = _calibration_gamma_ev2, .rows = 1, .cols = sizeof( _calibration_gamma_ev2 ) / sizeof( _calibration_gamma_ev2[0] ), .width = sizeof( _calibration_gamma_ev2[0] )};

static LookupTable calibration_gamma_threshold = {.ptr = _calibration_gamma_threshold, .rows = 1, .cols = sizeof( _calibration_gamma_threshold ) / sizeof( _calibration_gamma_threshold[0] ), .width = sizeof( _calibration_gamma_threshold[0] )};


//static LookupTable calibration_demosaic_uu_slope = {.ptr = _calibration_demosaic_uu_slope, .cols=2, .rows = sizeof(_calibration_demosaic_uu_slope) / sizeof(_calibration_demosaic_uu_slope[0]), .width = sizeof(_calibration_demosaic_uu_slope[0][0])};

//static LookupTable calibration_bypass_control = {.ptr = _calibration_bypass_control, .rows = 1, .cols = sizeof(_calibration_bypass_control) / sizeof(_calibration_bypass_control[0]), .width = sizeof( _calibration_bypass_control[0])};

//static LookupTable calibration_sinter_strength4 = {.ptr = _calibration_sinter_strength4, .cols=2, .rows = sizeof(_calibration_sinter_strength4) / sizeof(_calibration_sinter_strength4[0]), .width = sizeof(_calibration_sinter_strength4[0][0])};


uint32_t get_dynamic_calibrations( ACameraCalibrations * c ) {
    uint32_t result = 0;
    if (c != 0) {
        c->calibrations[CALIBRATION_AE_CONTROL] = &calibration_ae_control;
        c->calibrations[CALIBRATION_AE_CONTROL_HDR_TARGET] = &calibration_ae_control_hdr_target;
        c->calibrations[CALIBRATION_AE_CORRECTION] = &calibration_ae_correction;
        c->calibrations[CALIBRATION_AE_EXPOSURE_CORRECTION] = &calibration_ae_exposure_correction;
        c->calibrations[CALIBRATION_AE_ZONE_WGHT_HOR] = &calibration_ae_zone_wght_hor;
        c->calibrations[CALIBRATION_AE_ZONE_WGHT_VER] = &calibration_ae_zone_wght_ver;
        c->calibrations[CALIBRATION_EXPOSURE_RATIO_ADJUSTMENT] = &calibration_exposure_ratio_adjustment;
        c->calibrations[CALIBRATION_CMOS_CONTROL] = &calibration_cmos_control;
        c->calibrations[CALIBRATION_CMOS_EXPOSURE_PARTITION_LUTS] = &calibration_cmos_exposure_partition_luts;
        c->calibrations[CALIBRATION_AF_LMS] = &calibration_af_lms;
        c->calibrations[CALIBRATION_AF_ZONE_WGHT_HOR] = &calibration_af_zone_wght_hor;
        c->calibrations[CALIBRATION_AF_ZONE_WGHT_VER] = &calibration_af_zone_wght_ver;
        c->calibrations[CALIBRATION_AWB_AVG_COEF] = &calibration_awb_avg_coef;
        c->calibrations[CALIBRATION_AWB_BG_MAX_GAIN] = &calibration_awb_bg_max_gain;
        c->calibrations[AWB_COLOUR_PREFERENCE] = &awb_colour_preference;
        c->calibrations[CALIBRATION_AWB_MIX_LIGHT_PARAMETERS] = &calibration_awb_mix_light_parameters;
        c->calibrations[CALIBRATION_AWB_ZONE_WGHT_HOR] = &calibration_awb_zone_wght_hor;
        c->calibrations[CALIBRATION_AWB_ZONE_WGHT_VER] = &calibration_awb_zone_wght_ver;
        c->calibrations[CALIBRATION_EVTOLUX_PROBABILITY_ENABLE] = &calibration_evtolux_probability_enable;
        c->calibrations[CALIBRATION_AUTO_LEVEL_CONTROL] = &calibration_auto_level_control;
        c->calibrations[CALIBRATION_CCM_ONE_GAIN_THRESHOLD] = &calibration_ccm_one_gain_threshold;
        c->calibrations[CALIBRATION_DEMOSAIC_NP_OFFSET] = &calibration_demosaic_np_offset;
        c->calibrations[CALIBRATION_SHARP_ALT_D] = &calibration_sharp_alt_d;
        c->calibrations[CALIBRATION_SHARP_ALT_DU] = &calibration_sharp_alt_du;
        c->calibrations[CALIBRATION_SHARP_ALT_UD] = &calibration_sharp_alt_ud;
        c->calibrations[CALIBRATION_DP_SLOPE] = &calibration_dp_slope;
        c->calibrations[CALIBRATION_DP_THRESHOLD] = &calibration_dp_threshold;
        c->calibrations[CALIBRATION_IRIDIX_AVG_COEF] = &calibration_iridix_avg_coef;
        c->calibrations[CALIBRATION_IRIDIX_EV_LIM_FULL_STR] = &calibration_iridix_ev_lim_full_str;
        c->calibrations[CALIBRATION_IRIDIX_EV_LIM_NO_STR] = &calibration_iridix_ev_lim_no_str;
        c->calibrations[CALIBRATION_IRIDIX_MIN_MAX_STR] = &calibration_iridix_min_max_str;
        c->calibrations[CALIBRATION_IRIDIX_STRENGTH_MAXIMUM] = &calibration_iridix_strength_maximum;
        c->calibrations[CALIBRATION_IRIDIX8_STRENGTH_DK_ENH_CONTROL] = &calibration_iridix8_strength_dk_enh_control;
        c->calibrations[CALIBRATION_MESH_SHADING_STRENGTH] = &calibration_mesh_shading_strength;
        c->calibrations[CALIBRATION_PF_RADIAL_LUT] = &calibration_pf_radial_lut;
        c->calibrations[CALIBRATION_PF_RADIAL_PARAMS] = &calibration_pf_radial_params;
        c->calibrations[CALIBRATION_SATURATION_STRENGTH] = &calibration_saturation_strength;
        c->calibrations[CALIBRATION_SCALER_H_FILTER] = &calibration_scaler_h_filter;
        c->calibrations[CALIBRATION_SCALER_V_FILTER] = &calibration_scaler_v_filter;
        c->calibrations[CALIBRATION_SHARPEN_DS1] = &calibration_sharpen_ds1;
        c->calibrations[CALIBRATION_SHARPEN_FR] = &calibration_sharpen_fr;
        c->calibrations[CALIBRATION_SINTER_INTCONFIG] = &calibration_sinter_intconfig;
        c->calibrations[CALIBRATION_SINTER_RADIAL_LUT] = &calibration_sinter_radial_lut;
        c->calibrations[CALIBRATION_SINTER_RADIAL_PARAMS] = &calibration_sinter_radial_params;
        c->calibrations[CALIBRATION_SINTER_SAD] = &calibration_sinter_sad;
        c->calibrations[CALIBRATION_SINTER_STRENGTH] = &calibration_sinter_strength;
        c->calibrations[CALIBRATION_SINTER_STRENGTH_MC_CONTRAST] = &calibration_sinter_strength_mc_contrast;
        c->calibrations[CALIBRATION_SINTER_STRENGTH1] = &calibration_sinter_strength1;
        c->calibrations[CALIBRATION_SINTER_THRESH1] = &calibration_sinter_thresh1;
        c->calibrations[CALIBRATION_SINTER_THRESH4] = &calibration_sinter_thresh4;
        c->calibrations[CALIBRATION_CNR_UV_DELTA12_SLOPE] = &calibration_cnr_uv_delta12_slope;
        c->calibrations[CALIBRATION_STITCHING_LM_MED_NOISE_INTENSITY] = &calibration_stitching_lm_med_noise_intensity;
        c->calibrations[CALIBRATION_STITCHING_LM_MOV_MULT] = &calibration_stitching_lm_mov_mult;
        c->calibrations[CALIBRATION_STITCHING_LM_NP] = &calibration_stitching_lm_np;
        c->calibrations[CALIBRATION_STITCHING_MS_MOV_MULT] = &calibration_stitching_ms_mov_mult;
        c->calibrations[CALIBRATION_STITCHING_MS_NP] = &calibration_stitching_ms_np;
        c->calibrations[CALIBRATION_STITCHING_SVS_MOV_MULT] = &calibration_stitching_svs_mov_mult;
        c->calibrations[CALIBRATION_STITCHING_SVS_NP] = &calibration_stitching_svs_np;
        c->calibrations[CALIBRATION_FS_MC_OFF] = &calibration_fs_mc_off;
        c->calibrations[CALIBRATION_TEMPER_STRENGTH] = &calibration_temper_strength;
        c->calibrations[CALIBRATION_RGB2YUV_CONVERSION] = &calibration_rgb2yuv_conversion;
        c->calibrations[CALIBRATION_CUSTOM_SETTINGS_CONTEXT] = &calibration_custom_settings_context;
        c->calibrations[CALIBRATION_STATUS_INFO] = &calibration_status_info;
	c->calibrations[CALIBRATION_ZOOM_LMS] = &calibration_zoom_lms;
	c->calibrations[CALIBRATION_ZOOM_AF_LMS] = &calibration_zoom_af_lms;
	c->calibrations[CALIBRATION_GAMMA_EV1] = &calibration_gamma_ev1;
	c->calibrations[CALIBRATION_GAMMA_EV2] = &calibration_gamma_ev2;
	c->calibrations[CALIBRATION_GAMMA_THRESHOLD] = &calibration_gamma_threshold;
	//c->calibrations[CALIBRATION_DEMOSAIC_UU_SLOPE] = &calibration_demosaic_uu_slope;
	//c->calibrations[CALIBRATION_BYPASS_CONTROL] = &calibration_bypass_control;
        //c->calibrations[CALIBRATION_SINTER_STRENGTH4] = &calibration_sinter_strength4;
    } else {
        result = -1;
    }
    return result;
}
