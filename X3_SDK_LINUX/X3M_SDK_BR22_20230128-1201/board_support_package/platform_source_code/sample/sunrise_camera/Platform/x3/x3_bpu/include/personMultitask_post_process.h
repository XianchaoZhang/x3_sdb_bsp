// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef _PERSON_POST_PROCESS_H_
#define _PERSON_POST_PROCESS_H_

#include "dnn/hb_dnn.h"

#ifdef __cplusplus
  extern "C"{
#endif

typedef struct {
	int height;
	int width;
	int ori_height;
	int ori_width;
	hbDNNTensor *output_tensor;
	hbDNNHandle_t m_dnn_handle;
} PersonPostProcessInfo_t;

char* PersonPostProcess(PersonPostProcessInfo_t *post_info);

#ifdef __cplusplus
}
#endif

#endif  // _PERSON_POST_PROCESS_H_