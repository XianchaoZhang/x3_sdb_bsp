/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>
#include <pthread.h>
#include <sys/stat.h>
#include <time.h>
#include <sys/time.h>
#include <signal.h>
#include <fcntl.h>
#include <linux/fb.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include "hb_vot.h"
#include "dual_common.h"

int hb_vot_init(void)
{
	int ret = 0;
	VOT_PUB_ATTR_S devAttr = {0};
	VOT_VIDEO_LAYER_ATTR_S stLayerAttr = {0};
	VOT_CHN_ATTR_S stChnAttr = {0};
	// VOT_WB_ATTR_S stWbAttr;
	VOT_CROP_INFO_S cropAttrs = {0};

	devAttr.enIntfSync = VOT_OUTPUT_1920x1080;
	devAttr.u32BgColor = 0x8080;
	devAttr.enOutputMode = HB_VOT_OUTPUT_BT1120;

	ret = HB_VOT_SetPubAttr(0, &devAttr);
	if (ret) {
		printf("HB_VOT_SetPubAttr failed\n");
		goto err;
	}

	ret = HB_VOT_Enable(0);
	if (ret) {
		printf("HB_VOT_Enable failed.\n");
		goto err;
	}

	ret = HB_VOT_GetVideoLayerAttr(0, &stLayerAttr);
	if (ret) {
		printf("HB_VOT_GetVideoLayerAttr failed.\n");
		goto err;
	}

	stLayerAttr.stImageSize.u32Width  = 1920;
	stLayerAttr.stImageSize.u32Height = 1080;

	stLayerAttr.panel_type = 0;
	stLayerAttr.rotate = 0;
	stLayerAttr.dithering_flag = 0;
	stLayerAttr.dithering_en = 0;
	stLayerAttr.gamma_en = 0;
	stLayerAttr.hue_en = 0;
	stLayerAttr.sat_en = 0;
	stLayerAttr.con_en = 0;
	stLayerAttr.bright_en = 0;
	stLayerAttr.theta_sign = 0;
	stLayerAttr.contrast = 0;
	stLayerAttr.theta_abs = 0;
	stLayerAttr.saturation = 0;
	stLayerAttr.off_contrast = 0;
	stLayerAttr.off_bright = 0;
	stLayerAttr.user_control_disp = 0;
	stLayerAttr.user_control_disp_layer1 = 0;
	stLayerAttr.big_endian = 0;
	ret = HB_VOT_SetVideoLayerAttr(0, &stLayerAttr);
	if (ret) {
		printf("HB_VOT_SetVideoLayerAttr failed.\n");
		goto err;
	}

	ret = HB_VOT_EnableVideoLayer(0);
	if (ret) {
		printf("HB_VOT_EnableVideoLayer failed.\n");
		HB_VOT_Disable(0);
		goto err;
	}

	stChnAttr.u32Priority = 2;
	stChnAttr.s32X = 0;
	stChnAttr.s32Y = 0;
	stChnAttr.u32SrcWidth = 1920;
	stChnAttr.u32SrcHeight = 1080;
	stChnAttr.u32DstWidth = 1920;
	stChnAttr.u32DstHeight = 1080;
	ret = HB_VOT_SetChnAttr(0, 0, &stChnAttr);
	if (ret) {
		printf("HB_VOT_SetChnAttr 0: %d\n", ret);
		HB_VOT_DisableVideoLayer(0);
		HB_VOT_Disable(0);
		goto err;
	}

	cropAttrs.u32Width = stChnAttr.u32DstWidth;  // - stChnAttr.s32X;
	cropAttrs.u32Height = stChnAttr.u32DstHeight;  //- stChnAttr.s32Y;
	ret = HB_VOT_SetChnCrop(0, 0, &cropAttrs);
	printf("HB_VOT_SetChnCrop: %d\n", ret);

	ret = HB_VOT_EnableChn(0, 0);
	if (ret) {
		printf("HB_VOT_EnableChn: %d\n", ret);
		HB_VOT_DisableVideoLayer(0);
		HB_VOT_Disable(0);
		goto err;
	}

	ret = HB_VOT_BindVps(0, 7, 0, 0);  // 37  pym ds0
	if (ret) {
		printf("HB_VOT_BindVps: %d\n", ret);
		HB_VOT_DisableChn(0, 1);
		HB_VOT_DisableVideoLayer(0);
		HB_VOT_Disable(0);
	}

err:
	return ret;
}



int hb_vot_deinit(void)
{
	int ret = 0;

	ret = HB_VOT_DisableChn(0, 0);
	if (ret) {
		printf("HB_VOT_DisableChn0 failed.\n");
	}
	ret = HB_VOT_DisableVideoLayer(0);
	if (ret) {
		printf("HB_VOT_DisableVideoLayer failed.\n");
	}

	ret = HB_VOT_Disable(0);
	if (ret) {
		printf("HB_VOT_Disable failed.\n");
	}
	return 0;
}


