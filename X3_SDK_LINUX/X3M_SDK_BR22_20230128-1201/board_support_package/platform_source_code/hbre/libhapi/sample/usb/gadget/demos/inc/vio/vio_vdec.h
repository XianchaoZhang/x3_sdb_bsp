/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef INCLUDE_VIO_VDEC_H_
#define INCLUDE_VIO_VDEC_H_
#include "hb_comm_vdec.h"
#include "hb_comm_video.h"
#include "hb_vdec.h"
#include "utils/utils.h"

typedef struct {
    int chn;
    vp_param_t vp_param;
    av_param_t av_param;
} vdecParam_t;

int hb_vdec_init(VDEC_CHN_ATTR_S vdecChnAttr, int chn);
int hb_vdec_start(int chn);
int hb_vdec_stop(int chn);
int hb_vps_unbind_venc(int vpsGrp, int vpsChn, int vencChn);
int hb_vdec_input(int chn, VIDEO_STREAM_S *pstStream);
int hb_vdec_output_release(int chn, VIDEO_FRAME_S *pstStream);
int hb_vdec_get_output(int chn, VIDEO_FRAME_S *pstStream);
int hb_vdec_deinit(int chn);
#endif // INCLUDE_VIO_VDEC_H_
