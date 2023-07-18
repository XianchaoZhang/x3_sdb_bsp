/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#include <stdio.h>
#include <string.h>
#include "vio_vdec.h"

int hb_vdec_init(VDEC_CHN_ATTR_S vdecChnAttr, int chn)
{
	auto ret = HB_VDEC_CreateChn(chn, &vdecChnAttr);
	if (ret) {
		printf("HB_VDEC_CreateChn failed %x\n", ret);
		return ret;
	}

	ret = HB_VDEC_SetChnAttr(chn, &vdecChnAttr);	// config
	if (ret) {
		printf("HB_VDEC_SetChnAttr failed %x\n", ret);
		return ret;
	}

	return 0;
}

int hb_vdec_start(int chn)
{
	int ret = 0;

	ret = HB_VDEC_StartRecvStream(chn);
	if (ret != 0) {
		printf("HB_VDEC_StartRecvStream failed %x\n", ret);
	}
	return ret;
}

int hb_vdec_input(int chn, VIDEO_STREAM_S * pstStream)
{
	auto ret = HB_VDEC_SendStream(chn, pstStream, 3000);
	if (ret) {
		printf("HB_VDEC_SendStream error!!!\n");
	}
	return ret;
}

int hb_vdec_get_output(int chn, VIDEO_FRAME_S * stFrameInfo)
{
	auto ret = HB_VDEC_GetFrame(chn, stFrameInfo, 2000);
	if (ret) {
		printf("HB_VDEC_GetFrame error!!!\n");
	}
	return ret;
}

int hb_vdec_output_release(int chn, VIDEO_FRAME_S * stFrameInfo)
{
	auto ret = HB_VDEC_ReleaseFrame(chn, stFrameInfo);
	if (ret) {
		printf("HB_VDEC_ReleaseFrame failed\n");
	}
	return ret;
}

int hb_vdec_stop(int chn)
{
	auto ret = HB_VDEC_StopRecvStream(chn);
	if (ret) {
		printf("HB_VDEC_StopRecvStream failed %x\n", ret);
	}

	return ret;
}

int hb_vdec_deinit(int chn)
{
	auto ret = HB_VDEC_DestroyChn(chn);
	if (ret) {
		printf("HB_VDEC_ReleaseFrame failed %d\n", ret);
	}

	return ret;
}
