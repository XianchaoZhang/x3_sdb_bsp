/*
*
* SPDX-License-Identifier: GPL-2.0
*
* Copyright (C) 2011-2018 ARM or its affiliates
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; version 2.
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
* or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
* for more details.
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*
*/

#ifndef _ACAMERA_COMMAND_DEFINE_H_
#define _ACAMERA_COMMAND_DEFINE_H_

#include<stdint.h>



// ------------------------------------------------------------------------------ //
//		STATIC CALIBRATION VALUES
// ------------------------------------------------------------------------------ //
#define CALIBRATION_LIGHT_SRC                             0x00000000
#define CALIBRATION_RG_POS                                0x00000001
#define CALIBRATION_BG_POS                                0x00000002
#define CALIBRATION_MESH_RGBG_WEIGHT                      0x00000003
#define CALIBRATION_MESH_LS_WEIGHT                        0x00000004
#define CALIBRATION_MESH_COLOR_TEMPERATURE                0x00000005
#define CALIBRATION_WB_STRENGTH                           0x00000006
#define CALIBRATION_SKY_LUX_TH                            0x00000007
#define CALIBRATION_CT_RG_POS_CALC                        0x00000008
#define CALIBRATION_CT_BG_POS_CALC                        0x00000009
#define CALIBRATION_COLOR_TEMP                            0x0000000A
#define CALIBRATION_CT65POS                               0x0000000B
#define CALIBRATION_CT40POS                               0x0000000C
#define CALIBRATION_CT30POS                               0x0000000D
#define CALIBRATION_EVTOLUX_EV_LUT                        0x0000000E
#define CALIBRATION_EVTOLUX_LUX_LUT                       0x0000000F
#define CALIBRATION_BLACK_LEVEL_R                         0x00000010
#define CALIBRATION_BLACK_LEVEL_GR                        0x00000011
#define CALIBRATION_BLACK_LEVEL_GB                        0x00000012
#define CALIBRATION_BLACK_LEVEL_B                         0x00000013
#define CALIBRATION_STATIC_WB                             0x00000014
#define CALIBRATION_MT_ABSOLUTE_LS_A_CCM                  0x00000015
#define CALIBRATION_MT_ABSOLUTE_LS_D40_CCM                0x00000016
#define CALIBRATION_MT_ABSOLUTE_LS_D50_CCM                0x00000017
#define CALIBRATION_SHADING_LS_A_R                        0x00000018
#define CALIBRATION_SHADING_LS_A_G                        0x00000019
#define CALIBRATION_SHADING_LS_A_B                        0x0000001A
#define CALIBRATION_SHADING_LS_TL84_R                     0x0000001B
#define CALIBRATION_SHADING_LS_TL84_G                     0x0000001C
#define CALIBRATION_SHADING_LS_TL84_B                     0x0000001D
#define CALIBRATION_SHADING_LS_D65_R                      0x0000001E
#define CALIBRATION_SHADING_LS_D65_G                      0x0000001F
#define CALIBRATION_SHADING_LS_D65_B                      0x00000020
#define CALIBRATION_AWB_WARMING_LS_A                      0x00000021
#define CALIBRATION_AWB_WARMING_LS_D50                    0x00000022
#define CALIBRATION_AWB_WARMING_LS_D75                    0x00000023
#define CALIBRATION_NOISE_PROFILE                         0x00000024
#define CALIBRATION_DEMOSAIC                              0x00000025
#define CALIBRATION_GAMMA                                 0x00000026
#define CALIBRATION_IRIDIX_ASYMMETRY                      0x00000027
#define CALIBRATION_AWB_SCENE_PRESETS                     0x00000028
#define CALIBRATION_WDR_NP_LUT                            0x00000029
#define CALIBRATION_CA_FILTER_MEM                         0x0000002A
#define CALIBRATION_CA_CORRECTION                         0x0000002B
#define CALIBRATION_CA_CORRECTION_MEM                     0x0000002C
#define CALIBRATION_LUT3D_MEM                             0x0000002D
#define CALIBRATION_DECOMPANDER0_MEM                      0x0000002E
#define CALIBRATION_DECOMPANDER1_MEM                      0x0000002F
#define CALIBRATION_SHADING_RADIAL_R                      0x00000030
#define CALIBRATION_SHADING_RADIAL_G                      0x00000031
#define CALIBRATION_SHADING_RADIAL_B                      0x00000032


// ------------------------------------------------------------------------------ //
//		DYNAMIC CALIBRATION VALUES
// ------------------------------------------------------------------------------ //
#define CALIBRATION_STITCHING_LM_MED_NOISE_INTENSITY      0x00000033
#define AWB_COLOUR_PREFERENCE                             0x00000034
#define CALIBRATION_AWB_MIX_LIGHT_PARAMETERS              0x00000035
#define CALIBRATION_PF_RADIAL_LUT                         0x00000036
#define CALIBRATION_PF_RADIAL_PARAMS                      0x00000037
#define CALIBRATION_SINTER_RADIAL_LUT                     0x00000038
#define CALIBRATION_SINTER_RADIAL_PARAMS                  0x00000039
#define CALIBRATION_AWB_BG_MAX_GAIN                       0x0000003A
#define CALIBRATION_IRIDIX8_STRENGTH_DK_ENH_CONTROL       0x0000003B
#define CALIBRATION_CMOS_CONTROL                          0x0000003C
#define CALIBRATION_CMOS_EXPOSURE_PARTITION_LUTS          0x0000003D
#define CALIBRATION_STATUS_INFO                           0x0000003E
#define CALIBRATION_AUTO_LEVEL_CONTROL                    0x0000003F
#define CALIBRATION_DP_SLOPE                              0x00000040
#define CALIBRATION_DP_THRESHOLD                          0x00000041
#define CALIBRATION_STITCHING_LM_MOV_MULT                 0x00000042
#define CALIBRATION_STITCHING_LM_NP                       0x00000043
#define CALIBRATION_STITCHING_MS_MOV_MULT                 0x00000044
#define CALIBRATION_STITCHING_MS_NP                       0x00000045
#define CALIBRATION_STITCHING_SVS_MOV_MULT                0x00000046
#define CALIBRATION_STITCHING_SVS_NP                      0x00000047
#define CALIBRATION_EVTOLUX_PROBABILITY_ENABLE            0x00000048
#define CALIBRATION_AWB_AVG_COEF                          0x00000049
#define CALIBRATION_IRIDIX_AVG_COEF                       0x0000004A
#define CALIBRATION_IRIDIX_STRENGTH_MAXIMUM               0x0000004B
#define CALIBRATION_IRIDIX_MIN_MAX_STR                    0x0000004C
#define CALIBRATION_IRIDIX_EV_LIM_FULL_STR                0x0000004D
#define CALIBRATION_IRIDIX_EV_LIM_NO_STR                  0x0000004E
#define CALIBRATION_AE_CORRECTION                         0x0000004F
#define CALIBRATION_AE_EXPOSURE_CORRECTION                0x00000050
#define CALIBRATION_SINTER_STRENGTH                       0x00000051
#define CALIBRATION_SINTER_STRENGTH1                      0x00000052
#define CALIBRATION_SINTER_THRESH1                        0x00000053
#define CALIBRATION_SINTER_THRESH4                        0x00000054
#define CALIBRATION_SINTER_INTCONFIG                      0x00000055
#define CALIBRATION_SHARP_ALT_D                           0x00000056
#define CALIBRATION_SHARP_ALT_UD                          0x00000057
#define CALIBRATION_SHARP_ALT_DU                          0x00000058
#define CALIBRATION_DEMOSAIC_NP_OFFSET                    0x00000059
#define CALIBRATION_MESH_SHADING_STRENGTH                 0x0000005A
#define CALIBRATION_SATURATION_STRENGTH                   0x0000005B
#define CALIBRATION_CCM_ONE_GAIN_THRESHOLD                0x0000005C
#define CALIBRATION_RGB2YUV_CONVERSION                    0x0000005D
#define CALIBRATION_AE_ZONE_WGHT_HOR                      0x0000005E
#define CALIBRATION_AE_ZONE_WGHT_VER                      0x0000005F
#define CALIBRATION_AWB_ZONE_WGHT_HOR                     0x00000060
#define CALIBRATION_AWB_ZONE_WGHT_VER                     0x00000061
#define CALIBRATION_SHARPEN_FR                            0x00000062
#define CALIBRATION_SHARPEN_DS1                           0x00000063
#define CALIBRATION_TEMPER_STRENGTH                       0x00000064
#define CALIBRATION_SCALER_H_FILTER                       0x00000065
#define CALIBRATION_SCALER_V_FILTER                       0x00000066
#define CALIBRATION_SINTER_STRENGTH_MC_CONTRAST           0x00000067
#define CALIBRATION_EXPOSURE_RATIO_ADJUSTMENT             0x00000068
#define CALIBRATION_CNR_UV_DELTA12_SLOPE                  0x00000069
#define CALIBRATION_FS_MC_OFF                             0x0000006A
#define CALIBRATION_SINTER_SAD                            0x0000006B
#define CALIBRATION_AF_LMS                                0x0000006C
#define CALIBRATION_AF_ZONE_WGHT_HOR                      0x0000006D
#define CALIBRATION_AF_ZONE_WGHT_VER                      0x0000006E
#define CALIBRATION_AE_CONTROL_HDR_TARGET                 0x0000006F
#define CALIBRATION_AE_CONTROL                            0x00000070
#define CALIBRATION_CUSTOM_SETTINGS_CONTEXT               0x00000071
#define CALIBRATION_ZOOM_LMS                              0x00000072
#define CALIBRATION_ZOOM_AF_LMS                           0x00000073


// ------------------------------------------------------------ //
//              r2p0_new func
// ------------------------------------------------------------ //
#define CALIBRATION_AWB_WARMING_CCT                       0x00000074
#define CALIBRATION_SHADING_RADIAL_IR                     0x00000075
#define CALIBRATION_SHADING_RADIAL_CENTRE_AND_MULT        0x00000076
#define CALIBRATION_GAMMA_EV1                             0x00000077
#define CALIBRATION_GAMMA_EV2                             0x00000078
#define CALIBRATION_GAMMA_THRESHOLD                       0x00000079

#define CALIBRATION_SINTER_STRENGTH4                      0x0000007a
#define CALIBRATION_BYPASS_CONTROL                        0x0000007b

// dynamic
#define CALIBRATION_IRIDIX_BRIGHT_PR                      0x0000007c
#define CALIBRATION_IRIDIX_SVARIANCE                      0x0000007d
// static
#define CALIBRATION_LUT3D_MEM_A                           0x0000007e
#define CALIBRATION_LUT3D_MEM_D40                         0x0000007f
#define CALIBRATION_LUT3D_MEM_D50                         0x00000080
#define CALIBRATION_MT_ABSOLUTE_LS_U30_CCM                0x00000081
// dynamic
#define CALIBRATION_SHADING_TEMPER_THRESHOLD              0x00000082
#define CALIBRATION_CCM_TEMPER_THRESHOLD                  0x00000083
// static
#define CALIBRATION_SHADING_LS_D50_R                      0x00000084
#define CALIBRATION_SHADING_LS_D50_G                      0x00000085
#define CALIBRATION_SHADING_LS_D50_B                      0x00000086

// ------------------------------------------------------------------------------ //
//		DYNAMIC STATE VALUES
// ------------------------------------------------------------------------------ //

#define CALIBRATION_TOTAL_SIZE 135
//------------------FILE TRANSFER-------------------


// add r2p0 func
#define SYSTEM_MANUAL_CCM                                 0x00000083
#define SYSTEM_CCM_MATRIX_RR                              0x00000084
#define SYSTEM_CCM_MATRIX_RG                              0x00000085
#define SYSTEM_CCM_MATRIX_RB                              0x00000086
#define SYSTEM_CCM_MATRIX_GR                              0x00000087
#define SYSTEM_CCM_MATRIX_GG                              0x00000088
#define SYSTEM_CCM_MATRIX_GB                              0x00000089
#define SYSTEM_CCM_MATRIX_BR                              0x0000008a
#define SYSTEM_CCM_MATRIX_BG                              0x0000008b
#define SYSTEM_CCM_MATRIX_BB                              0x0000008c
#define SYSTEM_AWB_GREEN_EVEN_GAIN                        0x0000008d
#define SYSTEM_AWB_GREEN_ODD_GAIN                         0x0000008e
#define NOISE_REDUCTION_MODE_ID                           0x0000008f
#define SHADING_STRENGTH_ID                               0x00000090
#define HUE_THETA_ID                                      0x00000091

// r2p0 func
#define NOISE_REDUCTION_OFF                               0x00000046
#define NOISE_REDUCTION_ON                                0x00000047


typedef struct LookupTable {
    void *ptr;
    uint16_t rows;
    uint16_t cols;
    uint16_t width;
} LookupTable;

typedef struct _ACameraCalibrations {
    LookupTable* calibrations[ CALIBRATION_TOTAL_SIZE ] ;
} ACameraCalibrations ;

typedef struct calib_module_s {
        const char module[20];
        uint32_t (*get_calib_dynamic)( ACameraCalibrations *c );
        uint32_t (*get_calib_static)( ACameraCalibrations *c );
} calib_module_t;


#endif//_ACAMERA_COMMAND_API_H_
