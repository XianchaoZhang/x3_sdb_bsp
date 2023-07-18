/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef X3_USB_CAM_IMPL_H_
#define X3_USB_CAM_IMPL_H_

#include "camera_struct_define.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

int x3_usb_cam_init_param(void);
int x3_usb_cam_init(void);
int x3_usb_cam_uninit(void);
int x3_usb_cam_start(void);
int x3_usb_cam_stop(void);
int x3_usb_cam_param_set(CAM_PARAM_E type, char* val, unsigned int length);
int x3_usb_cam_param_get(CAM_PARAM_E type, char* val, unsigned int* length);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif // X3_USB_CAM_IMPL_H_