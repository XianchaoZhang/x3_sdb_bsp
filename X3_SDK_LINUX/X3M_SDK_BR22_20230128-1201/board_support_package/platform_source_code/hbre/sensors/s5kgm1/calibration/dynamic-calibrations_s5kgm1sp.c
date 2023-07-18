/**
 *  The confidential and proprietary information contained in this file may
 *   only be used by a person authorised under and to the extent permitted
 *  by a subsisting licensing agreement from ARM Limited or its affiliates.
 *
 *        (C) COPYRIGHT 2020 ARM Limited or its affiliates
 *                       ALL RIGHTS RESERVED
 *
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

// created from 2020-06-10T12:13:58.522Z UTCdynamic-calibrations (1).json

// CALIBRATION_AE_CONTROL (1x9 4 bytes)
static uint32_t _calibration_ae_control[]
 = {15,245,10,0,0,90,5,1,8};

// CALIBRATION_AE_CONTROL_HDR_TARGET (8x2 2 bytes)
static uint16_t _calibration_ae_control_hdr_target[][2]
 =  {
  { 0, 95 },
  { 256, 100 },
  { 512, 110 },
  { 768, 110 },
  { 1024, 110 },
  { 1280, 110 },
};

// CALIBRATION_AE_CORRECTION (1x12 1 bytes)
static uint8_t _calibration_ae_correction[]
 = {128,128,128,128,128,128,128,128,128,128,128};

// CALIBRATION_AE_EXPOSURE_CORRECTION (1x12 4 bytes)
static uint32_t _calibration_ae_exposure_correction[]
 = {2580880,2580880,2580880,2580880,250880,2580880,2580880,2580880,3854498,3854498,3854498};

// CALIBRATION_AE_ZONE_WGHT_HOR (1x32 2 bytes)
static uint16_t _calibration_ae_zone_wght_hor[]
 = {16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16};

// CALIBRATION_AE_ZONE_WGHT_VER (1x32 2 bytes)
static uint16_t _calibration_ae_zone_wght_ver[]
 = {16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16};

// CALIBRATION_EXPOSURE_RATIO_ADJUSTMENT (4x2 2 bytes)
static uint16_t _calibration_exposure_ratio_adjustment[][2]
 =  {
  { 256, 256 },
  { 4096, 256 },
  { 8192, 256 },
  { 16384, 256 }
};

// CALIBRATION_CMOS_CONTROL (1x18 4 bytes)
static uint32_t _calibration_cmos_control[]
 = {0,50,0,0,0,0,2310,2310,126,0,32,255,2310,52,0,2,1,4};

// CALIBRATION_CMOS_EXPOSURE_PARTITION_LUTS (2x10 2 bytes)
static uint16_t _calibration_cmos_exposure_partition_luts[][10]
 =  {
  { 10, 1, 20, 1, 20, 1, 40, 1, 40, 16 },
  { 10, 1, 10, 2, 20, 2, 20, 4, 40, 16 }
};

// CALIBRATION_AF_LMS (1x21 4 bytes)
static uint32_t _calibration_af_lms[]
 = {6000,6000,6000,6100,6100,6100,15200,15200,15200,15400,15400,15400,16,6,2,30,131072,131072,176144,65536,0};

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

// CALIBRATION_AWB_MIX_LIGHT_PARAMETERS (1x8 2 bytes)
static uint16_t _calibration_awb_mix_light_parameters[]
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
 = {6,99,0,5,99,15,1};

// CALIBRATION_CCM_ONE_GAIN_THRESHOLD (1x1 2 bytes)
static uint16_t _calibration_ccm_one_gain_threshold[]
 = {1000};

// CALIBRATION_DEMOSAIC_NP_OFFSET (7x2 2 bytes)
static uint16_t _calibration_demosaic_np_offset[][2]
 =  {
  { 0, 1 },
  { 256, 1 },
  { 512, 1 },
  { 768, 1 },
  { 1024, 1 },
  { 1280, 3 },

};

// CALIBRATION_SHARP_ALT_D (8x2 2 bytes)
static uint16_t _calibration_sharp_alt_d[][2]
 =  {
  { 0, 40 },
  { 256, 40 },
  { 512, 50 },
  { 768, 60 },
  { 1024, 76 },
  { 1280, 76 }
};

// CALIBRATION_SHARP_ALT_DU (8x2 2 bytes)
static uint16_t _calibration_sharp_alt_du[][2]
 =  {
  { 0, 30 },
  { 256, 30 },
  { 512, 35 },
  { 768, 40 },
  { 1024, 46 },
  { 1280, 46 }
};

// CALIBRATION_SHARP_ALT_UD (8x2 2 bytes)
static uint16_t _calibration_sharp_alt_ud[][2]
 =  {
  { 0, 10 },
  { 256, 5 },
  { 512, 0 },
  { 768, 0 },
  { 1024, 0 },
  { 1280, 0 }
};

// CALIBRATION_DP_SLOPE (7x2 2 bytes)
static uint16_t _calibration_dp_slope[][2]
 =  {
  { 0, 170 },
  { 256, 170 },
  { 512, 2048 },
  { 768, 2048 },
  { 1024, 2048 },
  { 1280, 2048 }
};

// CALIBRATION_DP_THRESHOLD (7x2 2 bytes)
static uint16_t _calibration_dp_threshold[][2]
 =  {
  { 0, 4095 },
  { 256, 4095 },
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
 = {2557570};

// CALIBRATION_IRIDIX_EV_LIM_NO_STR (1x2 4 bytes)
static uint32_t _calibration_iridix_ev_lim_no_str[]
 = {4150000,4274729};

// CALIBRATION_IRIDIX_MIN_MAX_STR (1x1 2 bytes)
static uint16_t _calibration_iridix_min_max_str[]
 = {0};

// CALIBRATION_IRIDIX_STRENGTH_MAXIMUM (1x1 1 bytes)
static uint8_t _calibration_iridix_strength_maximum[]
 = {255};

// CALIBRATION_IRIDIX8_STRENGTH_DK_ENH_CONTROL (1x15 4 bytes)
static uint32_t _calibration_iridix8_strength_dk_enh_control[]
 = {15,95,2000,3500,8,30,4096,13824,0,25,30,5120,13824,32,0};

// CALIBRATION_MESH_SHADING_STRENGTH (1x2 2 bytes)
static uint16_t _calibration_mesh_shading_strength[]
 = {0,4096};

// CALIBRATION_PF_RADIAL_LUT (1x33 1 bytes)
static uint8_t _calibration_pf_radial_lut[]
 = {255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255};

// CALIBRATION_PF_RADIAL_PARAMS (1x3 2 bytes)
static uint16_t _calibration_pf_radial_params[]
 = {1984,1500,348};

// CALIBRATION_SATURATION_STRENGTH (9x2 2 bytes)
static uint16_t _calibration_saturation_strength[][2]
 =  {
  { 0, 118 },
  { 256, 118},
  { 512, 118 },
  { 768, 118 },
  { 1024, 128 },
  { 1280, 128 },
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

// CALIBRATION_SHARPEN_FR (7x2 2 bytes)
static uint16_t _calibration_sharpen_fr[][2]
 =  {
  { 0, 20 },
  { 256, 20 },
  { 512, 20 },
  { 768, 20 },
  { 1024, 20 },
  { 1280, 20 }
};

// CALIBRATION_SINTER_INTCONFIG (7x2 2 bytes)
static uint16_t _calibration_sinter_intconfig[][2]
 =  {
  { 0, 100 },
  { 256, 120 },
  { 512, 135 },
  { 768, 135 },
  { 1024, 135 },
  { 1280, 145 }
};

// CALIBRATION_SINTER_RADIAL_LUT (1x33 1 bytes)
static uint8_t _calibration_sinter_radial_lut[]
 = {0,0,0,0,0,0,1,3,4,6,7,9,10,12,13,15,16,18,19,21,22,24,24,24,24,24,24,24,24,24,24,24,24};

// CALIBRATION_SINTER_RADIAL_PARAMS (1x4 2 bytes)
static uint16_t _calibration_sinter_radial_params[]
 = {0,960,540,1770};

// CALIBRATION_SINTER_SAD (7x2 2 bytes)
static uint16_t _calibration_sinter_sad[][2]
 =  {
  { 0, 0 },
  { 256, 0 },
  { 512, 0 },
  { 768, 5 },
  { 1024, 9 },
  { 1280, 11 },
  { 1536, 13 }
};

// CALIBRATION_SINTER_STRENGTH (8x2 2 bytes)
static uint16_t _calibration_sinter_strength[][2]
 =  {
  { 0, 20 },
  { 256, 35 },
  { 512, 45 },
  { 768, 60 },
  { 1024, 95 },
  { 1280, 100 }
};

// CALIBRATION_SINTER_STRENGTH_MC_CONTRAST (1x2 2 bytes)
static uint16_t _calibration_sinter_strength_mc_contrast[]
 = {10,0};

// CALIBRATION_SINTER_STRENGTH1 (8x2 2 bytes)
static uint16_t _calibration_sinter_strength1[][2]
 =  {
  { 0, 20 },
  { 256, 60 },
  { 512, 86 },
  { 768, 95 },
  { 1024, 100 },
  { 1280, 120},
};

// CALIBRATION_SINTER_THRESH1 (7x2 2 bytes)
static uint16_t _calibration_sinter_thresh1[][2]
 =  {
  { 0, 10 },
  { 256, 20 },
  { 512, 20 },
  { 768, 32 },
  { 1024, 48 },
  { 1280, 64 }
};

// CALIBRATION_SINTER_THRESH4 (7x2 2 bytes)
static uint16_t _calibration_sinter_thresh4[][2]
 =  {
  { 0, 10 },
  { 256, 20 },
  { 512, 30 },
  { 768, 30 },
  { 1024, 30 },
  { 1280, 96 }
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

// CALIBRATION_STITCHING_LM_MOV_MULT (3x2 2 bytes)
static uint16_t _calibration_stitching_lm_mov_mult[][2]
 =  {
  { 0, 450 },
  { 256, 500 },
  { 512, 550 },
  { 768, 600 },
  { 1024, 550 },
  { 1280, 550 },
};

// CALIBRATION_STITCHING_LM_NP (4x2 2 bytes)
static uint16_t _calibration_stitching_lm_np[][2]
 =  {
  { 0, 16 },
  { 256, 16 },
  { 512, 16 },
  { 768, 40 },
  { 1024, 40 },
  { 1280, 55 },
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
  { 0, 155 },
  { 256, 165 },
  { 512, 165 },
  { 768, 170 },
  { 1024, 170 },
  { 1280, 170 }
};

// CALIBRATION_RGB2YUV_CONVERSION (1x12 2 bytes)
static uint16_t _calibration_rgb2yuv_conversion[]
 = {76,150,29,32811,32852,128,128,32875,32788,0,512,512};

// CALIBRATION_CUSTOM_SETTINGS_CONTEXT (1x4 4 bytes)
//static uint32_t _calibration_custom_settings_context[]
// = {0,0,0,0};


static uint32_t _calibration_custom_settings_context[][4]
 = {
	//  {0x18e8c, 0x00000303, 0xFFFFFFFF, 4},
	{0x1ae8c, 0x00960171, 0xFFFFFFFF, 4},
	{0x1aeb4, 0x00001EC3, 0xFFFFFFFF, 4},
	{0x1aeb8, 0x00001DFB, 0xFFFFFFFF, 4},
	{0x1aec0, 0x000808BE, 0xFFFFFFFF, 4},
	// for 1lux demosaic
	//0x1aec4, 0x000000C8,
	{0x1aec4, 0x00000014, 0xFFFFFFFF, 4},
	{0x1ac30, 0x00004002, 0xFFFFFFFF, 4},
	{0x1ac34, 0x000CCCCC, 0xFFFFFFFF, 4},
	{0x1ac3c, 0x00A30000, 0xFFFFFFFF, 4},
	{0x1ac4c, 0xB4A00401, 0xFFFFFFFF, 4},
	{0x19368, 0x00001001, 0xFFFFFFFF, 4},
	{0x19370, 0x00000000, 0xFFFFFFFF, 4},
	{0x19374, 0x00000000, 0xFFFFFFFF, 4},
	{0x19378, 0x00000000, 0xFFFFFFFF, 4},
	{0x1937c, 0x00000000, 0xFFFFFFFF, 4},
	{0x1aa28, 0x00007801, 0xFFFFFFFF, 4},
	{0x1aa30, 0x00000000, 0xFFFFFFFF, 4},
	{0x1aa34, 0x00000000, 0xFFFFFFFF, 4},
	{0x1aa38, 0x00000000, 0xFFFFFFFF, 4},
	{0x18eac, 0x0000003C, 0xFFFFFFFF, 4},
	{0x18eb0, 0x00000016, 0xFFFFFFFF, 4},
	//{0x18eb8, 0x00000002, 0xFFFFFFFF, 4},
	// enable iridix & iridix_gain
	{0x18ebc, 0x00000005, 0xFFFFFFFF, 4},
	//disable irdix gain and iridix
	//{0x18ebc, 0x00000045, 0xFFFFFFFF, 4},
	// disable CNR
	//{0x18ec0, 0x000000E5, 0xFFFFFFFF, 4},
	// enable CNR
	{0x18ec0, 0x000000C5, 0xFFFFFFFF, 4},
	{0x18ec4, 0x00000000, 0xFFFFFFFF, 4},
	{0x18ec8, 0x00000000, 0xFFFFFFFF, 4},
	{0x19364, 0xFF005C00, 0xFFFFFFFF, 4},
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

	{0x1ae7c, 0x82B4C3C6, 0xFFFFFFFF, 4},
	{0x1ae80, 0x0000005D, 0xFFFFFFFF, 4},
	{0x1ae84, 0x004100DC, 0xFFFFFFFF, 4},
	{0x1ae88, 0x000000AF, 0xFFFFFFFF, 4},
	{0x1ae8c, 0x00960171, 0xFFFFFFFF, 4},
	{0x1ae90, 0x08000800, 0xFFFFFFFF, 4},
	{0x1ae94, 0x00000800, 0xFFFFFFFF, 4},
	{0x1ae9c, 0x0001006D, 0xFFFFFFFF, 4},
	{0x1aea0, 0x00000000, 0xFFFFFFFF, 4},
	{0x1aea4, 0x00CF01B3, 0xFFFFFFFF, 4},
	{0x1aea8, 0x000041A5, 0xFFFFFFFF, 4},
	{0x1aeac, 0x00000000, 0xFFFFFFFF, 4},
	{0x1aeb0, 0x08006D6D, 0xFFFFFFFF, 4},
	{0x1aeb4, 0x00001F8B, 0xFFFFFFFF, 4},
	{0x1aeb8, 0x00001FA4, 0xFFFFFFFF, 4},
	{0x1aebc, 0x00000001, 0xFFFFFFFF, 4},
	{0x1aec0, 0x0050B4B4, 0xFFFFFFFF, 4},
	{0x1aec4, 0x009B0050, 0xFFFFFFFF, 4},
	{0x1aec8, 0xFFFF96A1, 0xFFFFFFFF, 4},
	{0x1aecc, 0x03330333, 0xFFFFFFFF, 4},
	{0x1aed0, 0x00000008, 0xFFFFFFFF, 4},
	{0x1aed4, 0x00004000, 0xFFFFFFFF, 4},
	{0x1aed8, 0x0FA00000, 0xFFFFFFFF, 4},
	{0x1aedc, 0x00004000, 0xFFFFFFFF, 4},
	{0x1aee0, 0x0000004E, 0xFFFFFFFF, 4},
	{0x1aee4, 0x00004000, 0xFFFFFFFF, 4},
	{0x1aee8, 0x00000FA0, 0xFFFFFFFF, 4},
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
	{0,0,0,0},
};
// CALIBRATION_STATUS_INFO (1x5 4 bytes)
static uint32_t _calibration_status_info[]
 = {450317,3379427,6997,6100,0};

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
static LookupTable calibration_mesh_shading_strength = { .ptr = _calibration_mesh_shading_strength, .rows = 1, .cols = sizeof(_calibration_mesh_shading_strength) / sizeof(_calibration_mesh_shading_strength[0]), .width = sizeof(_calibration_mesh_shading_strength[0] ) };
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
static LookupTable calibration_sinter_strength_mc_contrast = { .ptr = _calibration_sinter_strength_mc_contrast, .rows = 1, .cols = sizeof(_calibration_sinter_strength_mc_contrast) / sizeof(_calibration_sinter_strength_mc_contrast[0]), .width = sizeof(_calibration_sinter_strength_mc_contrast[0] ) };
static LookupTable calibration_sinter_strength1 = { .ptr = _calibration_sinter_strength1, .cols = 2, .rows = sizeof(_calibration_sinter_strength1) / sizeof(_calibration_sinter_strength1[0]), .width = sizeof(_calibration_sinter_strength1[0][0] ) };
static LookupTable calibration_sinter_thresh1 = { .ptr = _calibration_sinter_thresh1, .cols = 2, .rows = sizeof(_calibration_sinter_thresh1) / sizeof(_calibration_sinter_thresh1[0]), .width = sizeof(_calibration_sinter_thresh1[0][0] ) };
static LookupTable calibration_sinter_thresh4 = { .ptr = _calibration_sinter_thresh4, .cols = 2, .rows = sizeof(_calibration_sinter_thresh4) / sizeof(_calibration_sinter_thresh4[0]), .width = sizeof(_calibration_sinter_thresh4[0][0] ) };
static LookupTable calibration_cnr_uv_delta12_slope = { .ptr = _calibration_cnr_uv_delta12_slope, .cols = 2, .rows = sizeof(_calibration_cnr_uv_delta12_slope) / sizeof(_calibration_cnr_uv_delta12_slope[0]), .width = sizeof(_calibration_cnr_uv_delta12_slope[0][0] ) };
static LookupTable calibration_stitching_lm_med_noise_intensity = { .ptr = _calibration_stitching_lm_med_noise_intensity, .cols = 2, .rows = sizeof(_calibration_stitching_lm_med_noise_intensity) / sizeof(_calibration_stitching_lm_med_noise_intensity[0]), .width = sizeof(_calibration_stitching_lm_med_noise_intensity[0][0] ) };
static LookupTable calibration_stitching_lm_mov_mult = { .ptr = _calibration_stitching_lm_mov_mult, .cols = 2, .rows = sizeof(_calibration_stitching_lm_mov_mult) / sizeof(_calibration_stitching_lm_mov_mult[0]), .width = sizeof(_calibration_stitching_lm_mov_mult[0][0] ) };
static LookupTable calibration_stitching_lm_np = { .ptr = _calibration_stitching_lm_np, .cols = 2, .rows = sizeof(_calibration_stitching_lm_np) / sizeof(_calibration_stitching_lm_np[0]), .width = sizeof(_calibration_stitching_lm_np[0][0] ) };
static LookupTable calibration_stitching_ms_mov_mult = { .ptr = _calibration_stitching_ms_mov_mult, .cols = 2, .rows = sizeof(_calibration_stitching_ms_mov_mult) / sizeof(_calibration_stitching_ms_mov_mult[0]), .width = sizeof(_calibration_stitching_ms_mov_mult[0][0] ) };
static LookupTable calibration_stitching_ms_np = { .ptr = _calibration_stitching_ms_np, .cols = 2, .rows = sizeof(_calibration_stitching_ms_np) / sizeof(_calibration_stitching_ms_np[0]), .width = sizeof(_calibration_stitching_ms_np[0][0] ) };
static LookupTable calibration_fs_mc_off = { .ptr = _calibration_fs_mc_off, .rows = 1, .cols = sizeof(_calibration_fs_mc_off) / sizeof(_calibration_fs_mc_off[0]), .width = sizeof(_calibration_fs_mc_off[0] ) };
static LookupTable calibration_temper_strength = { .ptr = _calibration_temper_strength, .cols = 2, .rows = sizeof(_calibration_temper_strength) / sizeof(_calibration_temper_strength[0]), .width = sizeof(_calibration_temper_strength[0][0] ) };
static LookupTable calibration_rgb2yuv_conversion = { .ptr = _calibration_rgb2yuv_conversion, .rows = 1, .cols = sizeof(_calibration_rgb2yuv_conversion) / sizeof(_calibration_rgb2yuv_conversion[0]), .width = sizeof(_calibration_rgb2yuv_conversion[0] ) };
static LookupTable calibration_custom_settings_context = { .ptr = _calibration_custom_settings_context, .rows = 1, .cols = sizeof(_calibration_custom_settings_context) / sizeof(_calibration_custom_settings_context[0]), .width = sizeof(_calibration_custom_settings_context[0] ) };
static LookupTable calibration_status_info = { .ptr = _calibration_status_info, .rows = 1, .cols = sizeof(_calibration_status_info) / sizeof(_calibration_status_info[0]), .width = sizeof(_calibration_status_info[0] ) };

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
        c->calibrations[CALIBRATION_FS_MC_OFF] = &calibration_fs_mc_off;
        c->calibrations[CALIBRATION_TEMPER_STRENGTH] = &calibration_temper_strength;
        c->calibrations[CALIBRATION_RGB2YUV_CONVERSION] = &calibration_rgb2yuv_conversion;
        c->calibrations[CALIBRATION_CUSTOM_SETTINGS_CONTEXT] = &calibration_custom_settings_context;
        c->calibrations[CALIBRATION_STATUS_INFO] = &calibration_status_info;
    } else {
        result = -1;
    }
    return result;
}
