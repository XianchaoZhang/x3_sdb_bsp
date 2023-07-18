/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef X3_IPC_IMPL_H_
#define X3_IPC_IMPL_H_

#include "camera_struct_define.h"

#include "x3_sdk_wrap.h"
#include "x3_vio_venc.h"

#if 0
int x3_sdk_f37_linear_param_init(void);
int x3_sdk_f37_dol2_param_init(void);
int x3_sdk_os8a10_linear_param_init(void);
int x3_sdk_os8a10_dol2_param_init(void);
int x3_sdk_os8a10_linear_f37_liear_param_init(void);
int x3_sdk_os8a10_2k_linear_param_init(void);
int x3_sdk_os8a10_2k_linear_f37_liear_param_init(void);
int x3_sdk_imx415_linear_param_init(void);
int x3_sdk_imx415_linear_f37_liear_param_init(void);
#endif

int x3_ipc_init_param(void);
int x3_ipc_init(void);
int x3_ipc_uninit(void);
int x3_ipc_start(void);
int x3_ipc_stop(void);
int x3_ipc_param_set(CAM_PARAM_E type, char* val, unsigned int length);
int x3_ipc_param_get(CAM_PARAM_E type, char* val, unsigned int* length);

#endif // X3_IPC_IMPL_H_