/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef SENSOR_OS8A10_CONFIG_H_
#define SENSOR_OS8A10_CONFIG_H_

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* __cplusplus */

#include "vio/hb_mipi_api.h"
#include "vio/hb_vin_api.h"

extern MIPI_SENSOR_INFO_S SENSOR_OS8A10_30FPS_10BIT_LINEAR_INFO;
extern MIPI_ATTR_S MIPI_SENSOR_OS8A10_30FPS_10BIT_LINEAR_ATTR;
extern MIPI_ATTR_S MIPI_SENSOR_OS8A10_30FPS_10BIT_LINEAR_SENSOR_CLK_ATTR;
extern VIN_DEV_ATTR_S DEV_ATTR_OS8A10_LINEAR_BASE;
extern VIN_PIPE_ATTR_S PIPE_ATTR_OS8A10_LINEAR_BASE;

extern VIN_DIS_ATTR_S DIS_ATTR_OS8A10_BASE;
extern VIN_LDC_ATTR_S LDC_ATTR_OS8A10_BASE;

extern MIPI_SENSOR_INFO_S SENSOR_OS8A10_30FPS_10BIT_DOL2_INFO;
extern MIPI_ATTR_S MIPI_SENSOR_OS8A10_30FPS_10BIT_DOL2_ATTR;
extern MIPI_ATTR_S MIPI_SENSOR_OS8A10_30FPS_10BIT_DOL2_SENSOR_CLK_ATTR;
extern VIN_DEV_ATTR_S DEV_ATTR_OS8A10_DOL2_BASE;
extern VIN_PIPE_ATTR_S PIPE_ATTR_OS8A10_DOL2_BASE;

extern MIPI_SENSOR_INFO_S SENSOR_OS8A10_2K_25FPS_10BIT_LINEAR_INFO;
extern MIPI_SENSOR_INFO_S SENSOR_OS8A10_2K_25FPS_10BIT_DOL2_INFO;
extern MIPI_ATTR_S MIPI_SENSOR_OS8A10_2K_25FPS_10BIT_LINEAR_ATTR;
extern MIPI_ATTR_S MIPI_SENSOR_OS8A10_2K_25FPS_10BIT_DOL2_ATTR;
extern VIN_DEV_ATTR_S DEV_ATTR_OS8A10_2K_LINEAR_BASE;
extern VIN_DEV_ATTR_S DEV_ATTR_OS8A10_2K_DOL2_BASE;
extern VIN_PIPE_ATTR_S PIPE_ATTR_OS8A10_2K_LINEAR_BASE;
extern VIN_PIPE_ATTR_S PIPE_ATTR_OS8A10_2K_DOL2_BASE;
extern VIN_DIS_ATTR_S DIS_ATTR_OS8A10_2K_BASE;
extern VIN_LDC_ATTR_S LDC_ATTR_OS8A10_2K_BASE;

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */
#endif // SENSOR_OS8A10_CONFIG_H_