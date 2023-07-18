/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef X3_BOX_IMPL_H_
#define X3_BOX_IMPL_H_

#include "camera_struct_define.h"

int x3_box_init_param(void);

int x3_box_init(void);
int x3_box_uninit(void);
int x3_box_start(void);
int x3_box_stop(void);

int x3_box_param_set(CAM_PARAM_E type, char* val, unsigned int length);
int x3_box_param_get(CAM_PARAM_E type, char* val, unsigned int* length);

#endif // X3_BOX_IMPL_H_