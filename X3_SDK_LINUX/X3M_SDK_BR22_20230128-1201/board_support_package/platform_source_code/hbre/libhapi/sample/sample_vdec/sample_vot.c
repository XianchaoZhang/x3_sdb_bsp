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

// #include "iar_interface.h"
#include "hb_vot.h"
#include "hb_vio_interface.h"
// #include "common_group.h"
#include "hb_sys.h"
#include "sample.h"

int sample_vot_sendframe(hb_vio_buffer_t *out_buf)
{
    int ret = 0;
    VOT_FRAME_INFO_S stFrame = {};

    stFrame.addr = out_buf->img_addr.addr[0];
    stFrame.size = out_buf->img_addr.width * out_buf->img_addr.height * 3/2;
    ret = HB_VOT_SendFrame(0, 0, &stFrame, -1);

    return ret;
}

int sample_vot_init(int layer)
{
    int ret = 0;
    VOT_PUB_ATTR_S devAttr;
    VOT_VIDEO_LAYER_ATTR_S stLayerAttr;
    VOT_CHN_ATTR_S stChnAttr;
    // VOT_WB_ATTR_S stWbAttr;
    VOT_CROP_INFO_S cropAttrs;

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
	stLayerAttr.user_control_disp = 1;
    stLayerAttr.user_control_disp_layer1 = 1;
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

    // stChnAttr.u32Priority = 3;
    // stChnAttr.s32X = 0;
    // stChnAttr.s32Y = 640;
    // stChnAttr.u32SrcWidth = 704;
    // stChnAttr.u32SrcHeight = 576;
    // stChnAttr.u32DstWidth = 704;
    // stChnAttr.u32DstHeight = 576;
    // ret = HB_VOT_SetChnAttr(0, 1, &stChnAttr);
    // if (ret) {
    //     printf("HB_VOT_SetChnAttr 0: %d\n", ret);
    //     HB_VOT_DisableVideoLayer(0);
    //     HB_VOT_Disable(0);
    //     goto err;
    // }

    // cropAttrs.u32Width = stChnAttr.u32DstWidth;  // - stChnAttr.s32X;
    // cropAttrs.u32Height = stChnAttr.u32DstHeight;  //- stChnAttr.s32Y;
    // ret = HB_VOT_SetChnCrop(0, 1, &cropAttrs);
    // printf("HB_VOT_SetChnCrop: %d\n", ret);

    // ret = HB_VOT_EnableChn(0, 1);
    // if (ret) {
    //     printf("HB_VOT_EnableChn: %d\n", ret);
    //     HB_VOT_DisableVideoLayer(0);
    //     HB_VOT_Disable(0);
    //     goto err;
    // }
    
    // ret = HB_VOT_BindVps(0, layer, 0, 0);  // 37 gdc0
    
    // if (ret) {
    //     printf("HB_VOT_BindVps: %d\n", ret);
    //     HB_VOT_DisableChn(0, 1);
    //     HB_VOT_DisableVideoLayer(0);
    //     HB_VOT_Disable(0);
    // }

    // ret = HB_VOT_BindVps(0, 6, 0, 1);  // 37 gdc0
    // if (ret) {
    //     printf("HB_VOT_BindVps: %d\n", ret);
    //     HB_VOT_DisableChn(0, 1);
    //     HB_VOT_DisableVideoLayer(0);
    //     HB_VOT_Disable(0);
    // }

    // struct HB_SYS_MOD_S src_mod, dst_mod;
	// src_mod.enModId = HB_ID_VPS;
	// src_mod.s32DevId = 0;
	// src_mod.s32ChnId = 3;
	// dst_mod.enModId = HB_ID_VOT;
	// dst_mod.s32DevId = 0;
	// dst_mod.s32ChnId = 0;
	// ret = HB_SYS_Bind(&src_mod, &dst_mod);
	// if (ret != 0)
	// 	printf("HB_SYS_Bind failed\n");

	// src_mod.enModId = HB_ID_VPS;
	// src_mod.s32DevId = 0;
	// src_mod.s32ChnId = 4;
	// dst_mod.enModId = HB_ID_VOT;
	// dst_mod.s32DevId = 0;
	// dst_mod.s32ChnId = 1;
	// ret = HB_SYS_Bind(&src_mod, &dst_mod);
	// if (ret != 0)
	// 	printf("HB_SYS_Bind failed\n");

err:
    return ret;
}

int sample_vot_deinit()
{
    int ret = 0;

    // struct HB_SYS_MOD_S src_mod, dst_mod;
	// src_mod.enModId = HB_ID_VPS;
	// src_mod.s32DevId = 0;
	// src_mod.s32ChnId = 3;
	// dst_mod.enModId = HB_ID_VOT;
	// dst_mod.s32DevId = 0;
	// dst_mod.s32ChnId = 0;
	// ret = HB_SYS_UnBind(&src_mod, &dst_mod);
	// if (ret != 0)
	// 	printf("HB_SYS_Bind failed\n");

	// src_mod.enModId = HB_ID_VPS;
	// src_mod.s32DevId = 0;
	// src_mod.s32ChnId = 4;
	// dst_mod.enModId = HB_ID_VOT;
	// dst_mod.s32DevId = 0;
	// dst_mod.s32ChnId = 1;
	// ret = HB_SYS_UnBind(&src_mod, &dst_mod);
	// if (ret != 0)
	// 	printf("HB_SYS_Bind failed\n");

    ret = HB_VOT_DisableChn(0, 0);
    if (ret) {
        printf("HB_VOT_DisableChn failed.\n");
    }

    // ret = HB_VOT_DisableChn(0, 1);
    // if (ret) {
    //     printf("HB_VOT_DisableChn failed.\n");
    // }

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

int sample_vot_wb_init(int wb_src, int wb_format)
{
    int ret = 0;
    VOT_WB_ATTR_S stWbAttr;

    stWbAttr.wb_src = wb_src;
    stWbAttr.wb_format = wb_format;
    HB_VOT_SetWBAttr(0, &stWbAttr);

    ret = HB_VOT_EnableWB(0);
    if (ret) {
        printf("HB_VOT_EnableWB failed.\n");
        return -1;
    }

    // struct HB_SYS_MOD_S src_mod, dst_mod;
    // src_mod.enModId = HB_ID_VOT;
    // src_mod.s32DevId = 0;
    // src_mod.s32ChnId = 0;
    // dst_mod.enModId = HB_ID_VENC;
    // dst_mod.s32DevId = 0;
    // dst_mod.s32ChnId = 0;
    // ret = HB_SYS_Bind(&src_mod, &dst_mod);
    // if (ret != 0) {
    //   printf("HB_SYS_Bind failed\n");
    //   return -1;
    // }

    return 0;
}

int sample_votwb_bind_vps(int vps_grp, int vps_chn)
{
    int ret = 0;

    struct HB_SYS_MOD_S src_mod, dst_mod;
	src_mod.enModId = HB_ID_VOT;
	src_mod.s32DevId = 0;
	src_mod.s32ChnId = 0;
	dst_mod.enModId = HB_ID_VPS;
	dst_mod.s32DevId = vps_grp;
	dst_mod.s32ChnId = vps_chn;
	ret = HB_SYS_Bind(&src_mod, &dst_mod);
	if (ret != 0)
		printf("HB_SYS_Bind failed\n");

    return ret;
}

int sample_votwb_bind_venc(int venc_chn)
{
    int ret = 0;

    struct HB_SYS_MOD_S src_mod, dst_mod;
    src_mod.enModId = HB_ID_VOT;
    src_mod.s32DevId = 0;
    src_mod.s32ChnId = 0;
    dst_mod.enModId = HB_ID_VENC;
    dst_mod.s32DevId = venc_chn;
    dst_mod.s32ChnId = venc_chn;
    ret = HB_SYS_Bind(&src_mod, &dst_mod);
    if (ret != 0) {
      printf("HB_SYS_Bind failed\n");
      return -1;
    }

    return 0;
}

int sample_votwb_unbind_vps(int vps_grp, int vps_chn)
{
    int ret = 0;

    struct HB_SYS_MOD_S src_mod, dst_mod;
	src_mod.enModId = HB_ID_VOT;
	src_mod.s32DevId = 0;
	src_mod.s32ChnId = 0;
	dst_mod.enModId = HB_ID_VPS;
	dst_mod.s32DevId = vps_grp;
	dst_mod.s32ChnId = vps_chn;
	ret = HB_SYS_UnBind(&src_mod, &dst_mod);
	if (ret != 0)
		printf("HB_SYS_Bind failed\n");

    return ret;
}

int sample_votwb_unbind_venc(int venc_chn)
{
    int ret = 0;

    struct HB_SYS_MOD_S src_mod, dst_mod;
    src_mod.enModId = HB_ID_VOT;
    src_mod.s32DevId = 0;
    src_mod.s32ChnId = 0;
    dst_mod.enModId = HB_ID_VENC;
    dst_mod.s32DevId = venc_chn;
    dst_mod.s32ChnId = venc_chn;
    ret = HB_SYS_UnBind(&src_mod, &dst_mod);
    if (ret != 0) {
      printf("HB_SYS_Bind failed\n");
      return -1;
    }

    return 0;
}

int sample_vot_wb_deinit()
{
    int ret = 0;

    ret = HB_VOT_DisableWB(0);
    if (ret) {
        printf("HB_VOT_DisableWB failed.\n");
        return -1;
    }
    return 0;
}
