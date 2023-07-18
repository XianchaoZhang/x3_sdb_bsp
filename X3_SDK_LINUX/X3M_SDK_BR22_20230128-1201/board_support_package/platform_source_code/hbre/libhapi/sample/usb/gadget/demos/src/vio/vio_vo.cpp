/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#include <memory>
#include <stdio.h>
#include <sys/stat.h>
#ifdef __cplusplus
extern "C" {
#endif
#include "hb_sys.h"
#ifdef __cplusplus
}
#endif
#include "vio/vio_log.h"
#include "hb_vot.h"
#include "vio_vo.h"
char *buffer_;
int y_offset = 1920 * 1080;
int uv_offset = 1920 * 1080 / 2;

int hb_vo_init()
{
	int ret = 0;
	VOT_VIDEO_LAYER_ATTR_S stLayerAttr;
	VOT_CHN_ATTR_S stChnAttr;
	// VOT_WB_ATTR_S stWbAttr;
	VOT_CROP_INFO_S cropAttrs;
	// hb_vio_buffer_t iar_buf = {0};
	VOT_PUB_ATTR_S devAttr;
	// iar_mmap_channel0();

	devAttr.enIntfSync = VOT_OUTPUT_1920x1080;
	devAttr.u32BgColor = 0x108080;
	devAttr.enOutputMode = HB_VOT_OUTPUT_BT1120;
	ret = HB_VOT_SetPubAttr(0, &devAttr);
	if (ret) {
		printf("HB_VOT_SetPubAttr failed\n");
		return -1;
	}
	ret = HB_VOT_Enable(0);
	if (ret) {
		printf("HB_VOT_Enable failed.\n");
		return -1;
	}

	ret = HB_VOT_GetVideoLayerAttr(0, &stLayerAttr);
	if (ret) {
		printf("HB_VOT_GetVideoLayerAttr failed.\n");
		return -1;
	}
	// memset(&stLayerAttr, 0, sizeof(stLayerAttr));
	stLayerAttr.stImageSize.u32Width = 1920;
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
	stLayerAttr.big_endian = 0;
	stLayerAttr.display_addr_type = 2;
	stLayerAttr.display_addr_type_layer1 = 2;
	// stLayerAttr.display_addr_type = 0;
	// stLayerAttr.display_addr_type_layer1 = 0;
	ret = HB_VOT_SetVideoLayerAttr(0, &stLayerAttr);
	if (ret) {
		printf("HB_VOT_SetVideoLayerAttr failed.\n");
		return -1;
	}

	ret = HB_VOT_EnableVideoLayer(0);
	if (ret) {
		printf("HB_VOT_EnableVideoLayer failed.\n");
		return -1;
	}

	stChnAttr.u32Priority = 2;
	stChnAttr.s32X = 0;
	stChnAttr.s32Y = 0;
	stChnAttr.u32SrcWidth = 1920;
	stChnAttr.u32SrcHeight = 1080;
	stChnAttr.u32DstWidth = 1920;
	stChnAttr.u32DstHeight = 1080;
	ret = HB_VOT_SetChnAttr(0, 0, &stChnAttr);
	printf("HB_VOT_SetChnAttr 0: %d\n", ret);

	cropAttrs.u32Width = stChnAttr.u32DstWidth;
	cropAttrs.u32Height = stChnAttr.u32DstHeight;
	ret = HB_VOT_SetChnCrop(0, 0, &cropAttrs);
	printf("HB_VOT_EnableChn: %d\n", ret);

	ret = HB_VOT_EnableChn(0, 0);
	printf("HB_VOT_EnableChn: %d\n", ret);

	buffer_ = reinterpret_cast < char *>(malloc(y_offset + uv_offset));
	return ret;
}

int hb_vo_deinit()
{
	int ret = 0;
	ret = HB_VOT_DisableChn(0, 0);
	if (ret)
		printf("HB_VOT_DisableChn failed.\n");

	ret = HB_VOT_DisableVideoLayer(0);
	if (ret)
		printf("HB_VOT_DisableVideoLayer failed.\n");

	ret = HB_VOT_Disable(0);
	if (ret)
		printf("HB_VOT_Disable failed.\n");

	free(buffer_);
	buffer_ = NULL;
	return ret;
}

int hb_vo_send_frame(address_info_t addr)
{
	VOT_FRAME_INFO_S stFrame = { };
	stFrame.addr = buffer_;
	stFrame.size = 1920 * 1080 * 3 / 2;

	memcpy(buffer_, addr.addr[0], y_offset);
	memcpy(buffer_ + y_offset, addr.addr[1], uv_offset);
	HB_VOT_SendFrame(0, 0, &stFrame, -1);
}
