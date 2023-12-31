/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef INCLUDE_CAMERA_H_
#define INCLUDE_CAMERA_H_

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* __cplusplus */

#include "hb_mipi_api.h"
#include "hb_vin_api.h"

extern VIN_LDC_ATTR_S LDC_ATTR_BASE;
extern VIN_DIS_ATTR_S DIS_ATTR_BASE;

extern MIPI_SENSOR_INFO_S SENSOR_F37_30FPS_10BIT_LINEAR_INFO;
extern MIPI_ATTR_S MIPI_SENSOR_F37_30FPS_10BIT_LINEAR_ATTR;
extern MIPI_ATTR_S MIPI_SENSOR_F37_30FPS_10BIT_LINEAR_SENSOR_CLK_ATTR;
extern VIN_DEV_ATTR_S DEV_ATTR_F37_LINEAR_BASE;
extern VIN_DEV_ATTR_EX_S DEV_ATTR_F37_MD_BASE;
extern VIN_PIPE_ATTR_S PIPE_ATTR_F37_LINEAR_BASE;
extern VIN_DEV_ATTR_S DEV_ATTR_F37_LINEAR_FEEDBACK;

extern MIPI_SENSOR_INFO_S SENSOR_4LANE_IMX327_30FPS_12BIT_LINEAR_INFO;
extern MIPI_SENSOR_INFO_S SENSOR_4LANE_IMX327_30FPS_12BIT_DOL2_INFO;
extern MIPI_SENSOR_INFO_S SENSOR_4LANE_IMX327_15FPS_12BIT_DOL3_INFO;
extern MIPI_ATTR_S MIPI_4LANE_SENSOR_IMX327_30FPS_12BIT_NORMAL_ATTR;
extern MIPI_ATTR_S MIPI_4LANE_SENSOR_IMX327_30FPS_12BIT_NORMAL_SENSOR_CLK_ATTR;
extern MIPI_ATTR_S MIPI_4LANE_SENSOR_IMX327_30FPS_12BIT_DOL2_ATTR;
extern MIPI_ATTR_S MIPI_4LANE_SENSOR_IMX327_15FPS_12BIT_DOL3_ATTR;
extern VIN_DEV_ATTR_S DEV_ATTR_IMX327_LINEAR_BASE;
extern VIN_DEV_ATTR_S DEV_ATTR_IMX327_DOL2_BASE;
extern VIN_DEV_ATTR_S DEV_ATTR_IMX327_DOL3_BASE;
extern VIN_DEV_ATTR_S DEV_ATTR_IMX327_DOL2_TWO_LINEAR_BASE;
extern VIN_DEV_ATTR_S DEV_ATTR_IMX327_DOL3_LEF_SEF1_BASE;
extern VIN_DEV_ATTR_S DEV_ATTR_IMX327_DOL3_THREE_LINEAR_BASE;
extern VIN_DEV_ATTR_EX_S DEV_ATTR_IMX327_MD_BASE;
extern VIN_PIPE_ATTR_S PIPE_ATTR_IMX327_DOL2_BASE;
extern VIN_PIPE_ATTR_S PIPE_ATTR_IMX327_DOL3_BASE;
extern VIN_PIPE_ATTR_S PIPE_ATTR_IMX327_LINEAR_BASE;

extern MIPI_SENSOR_INFO_S SENSOR_4LANE_AR0233_30FPS_12BIT_1080P_954_PWL_INFO;
extern MIPI_SENSOR_INFO_S SENSOR_4LANE_AR0233_30FPS_12BIT_1080P_954_LINEAR_INFO;
extern MIPI_SENSOR_INFO_S SENSOR_4LANE_AR0233_30FPS_12BIT_1080P_960_PWL_INFO;
extern MIPI_SENSOR_INFO_S SENSOR_4LANE_AR0233_30FPS_12BIT_1080P_960_LINEAR_INFO;
extern MIPI_ATTR_S MIPI_4LANE_SENSOR_AR0233_30FPS_12BIT_1080P_954_ATTR;
extern MIPI_ATTR_S MIPI_4LANE_SENSOR_AR0233_30FPS_12BIT_1080P_960_ATTR;
extern VIN_DEV_ATTR_S DEV_ATTR_AR0233_1080P_BASE;
extern VIN_PIPE_ATTR_S PIPE_ATTR_AR0233_1080P_PWL_BASE;
extern VIN_PIPE_ATTR_S PIPE_ATTR_AR0233_1080P_LINEAR_BASE;

extern MIPI_SENSOR_INFO_S SENSOR_OS8A10_30FPS_10BIT_LINEAR_INFO;
extern MIPI_SENSOR_INFO_S SENSOR_OS8A10_30FPS_10BIT_DOL2_INFO;
extern MIPI_ATTR_S MIPI_SENSOR_OS8A10_30FPS_10BIT_LINEAR_ATTR;
extern MIPI_ATTR_S MIPI_SENSOR_OS8A10_30FPS_10BIT_LINEAR_SENSOR_CLK_ATTR;
extern MIPI_ATTR_S MIPI_SENSOR_OS8A10_30FPS_10BIT_DOL2_ATTR;
extern MIPI_ATTR_S MIPI_SENSOR_OS8A10_30FPS_10BIT_DOL2_SENSOR_CLK_ATTR;
extern VIN_DEV_ATTR_S DEV_ATTR_OS8A10_LINEAR_BASE;
extern VIN_DEV_ATTR_S DEV_ATTR_OS8A10_DOL2_BASE;
extern VIN_PIPE_ATTR_S PIPE_ATTR_OS8A10_LINEAR_BASE;
extern VIN_PIPE_ATTR_S PIPE_ATTR_OS8A10_DOL2_BASE;
extern VIN_DIS_ATTR_S DIS_ATTR_OS8A10_BASE;
extern VIN_LDC_ATTR_S LDC_ATTR_OS8A10_BASE;

extern MIPI_SENSOR_INFO_S SENSOR_S5KGM1_30FPS_10BIT_LINEAR_INFO;
extern MIPI_ATTR_S MIPI_SENSOR_S5KGM1_30FPS_10BIT_LINEAR_ATTR;
extern MIPI_ATTR_S MIPI_SENSOR_S5KGM1_30FPS_10BIT_LINEAR_SENSOR_CLK_ATTR;
extern VIN_DEV_ATTR_S DEV_ATTR_S5KGM1_LINEAR_BASE;
extern VIN_PIPE_ATTR_S PIPE_ATTR_S5KGM1_LINEAR_BASE;
extern VIN_DIS_ATTR_S DIS_ATTR_S5KGM1_BASE;
extern VIN_LDC_ATTR_S LDC_ATTR_S5KGM1_BASE;

extern MIPI_SENSOR_INFO_S SENSOR_SC8238_30FPS_10BIT_LINEAR_INFO;
extern MIPI_ATTR_S MIPI_SENSOR_SC8238_30FPS_10BIT_LINEAR_ATTR;
extern MIPI_ATTR_S MIPI_SENSOR_SC8238_30FPS_10BIT_LINEAR_SENSOR_CLK_ATTR;
extern VIN_DEV_ATTR_S DEV_ATTR_SC8238_LINEAR_BASE;
extern VIN_PIPE_ATTR_S PIPE_ATTR_SC8238_LINEAR_BASE;
extern VIN_DIS_ATTR_S DIS_ATTR_SC8238_BASE;
extern VIN_LDC_ATTR_S LDC_ATTR_SC8238_BASE;

extern MIPI_SENSOR_INFO_S SENSOR_2LANE_OV10635_30FPS_YUV_720P_954_INFO;
extern MIPI_SENSOR_INFO_S SENSOR_2LANE_OV10635_30FPS_YUV_720P_960_INFO;
extern MIPI_ATTR_S MIPI_2LANE_OV10635_30FPS_YUV_720P_954_ATTR;
extern MIPI_ATTR_S MIPI_2LANE_OV10635_30FPS_YUV_720P_960_ATTR;
extern VIN_DEV_ATTR_S DEV_ATTR_OV10635_YUV_BASE;
extern VIN_PIPE_ATTR_S PIPE_ATTR_OV10635_YUV_BASE;
extern VIN_DIS_ATTR_S DIS_ATTR_OV10635_BASE;
extern VIN_LDC_ATTR_S LDC_ATTR_OV10635_BASE;

extern MIPI_SENSOR_INFO_S SENSOR_IMX415_30FPS_10BIT_LINEAR_INFO;
extern MIPI_ATTR_S MIPI_SENSOR_IMX415_30FPS_10BIT_LINEAR_ATTR;
extern MIPI_ATTR_S MIPI_SENSOR_IMX415_30FPS_10BIT_LINEAR_SENSOR_CLK_ATTR;
extern VIN_DEV_ATTR_S DEV_ATTR_IMX415_LINEAR_BASE;
extern VIN_PIPE_ATTR_S PIPE_ATTR_IMX415_LINEAR_BASE;
extern VIN_DIS_ATTR_S DIS_ATTR_IMX415_BASE;
extern VIN_LDC_ATTR_S LDC_ATTR_OS8A10_BASE;

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */
#endif // INCLUDE_CAMERA_H_