/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "hb_comm_venc.h"
#include "hb_vdec.h"
#include "hb_venc.h"
#include "hb_vp_api.h"
// #define VENC_BIND
int VencChnAttrInit(VENC_CHN_ATTR_S *pVencChnAttr, PAYLOAD_TYPE_E p_enType,
            int p_Width, int p_Height, PIXEL_FORMAT_E pixFmt) {
    int streambuf = 2*1024*1024;

    memset(pVencChnAttr, 0, sizeof(VENC_CHN_ATTR_S));
    pVencChnAttr->stVencAttr.enType = p_enType;

    pVencChnAttr->stVencAttr.u32PicWidth = p_Width;
    pVencChnAttr->stVencAttr.u32PicHeight = p_Height;

    pVencChnAttr->stVencAttr.enMirrorFlip = DIRECTION_NONE;
    pVencChnAttr->stVencAttr.enRotation = CODEC_ROTATION_0;
    pVencChnAttr->stVencAttr.stCropCfg.bEnable = HB_FALSE;

    if (p_Width * p_Height > 2688 * 1522) {
        streambuf = 3 * 1024 * 1024;
        pVencChnAttr->stVencAttr.vlc_buf_size = 7900*1024;
    } else if (p_Width * p_Height > 1920 * 1080) {
        streambuf = 1024 * 1024;
        pVencChnAttr->stVencAttr.vlc_buf_size = 4*1024*1024;
    } else if (p_Width * p_Height > 1280 * 720) {
        streambuf = 768 * 1024;
        pVencChnAttr->stVencAttr.vlc_buf_size = 2100*1024;
    } else {
        streambuf = 256 * 1024;
        pVencChnAttr->stVencAttr.vlc_buf_size = 2048*1024;
    }
    // pVencChnAttr->stVencAttr.vlc_buf_size = 0;
    //     ((((p_Width * p_Height) * 9) >> 3) * 3) >> 1;
    if (p_enType == PT_JPEG || p_enType == PT_MJPEG) {
        pVencChnAttr->stVencAttr.enPixelFormat = pixFmt;
        pVencChnAttr->stVencAttr.u32BitStreamBufferCount = 1;
        pVencChnAttr->stVencAttr.u32FrameBufferCount = 2;
        pVencChnAttr->stVencAttr.bExternalFreamBuffer = HB_TRUE;
        pVencChnAttr->stVencAttr.stAttrJpeg.dcf_enable = HB_FALSE;
        pVencChnAttr->stVencAttr.stAttrJpeg.quality_factor = 0;
        pVencChnAttr->stVencAttr.stAttrJpeg.restart_interval = 0;
        pVencChnAttr->stVencAttr.u32BitStreamBufSize = streambuf;
    } else {
        pVencChnAttr->stVencAttr.enPixelFormat = pixFmt;
        pVencChnAttr->stVencAttr.u32BitStreamBufferCount = 3;
        pVencChnAttr->stVencAttr.u32FrameBufferCount = 3;
        pVencChnAttr->stVencAttr.bExternalFreamBuffer = HB_TRUE;
        pVencChnAttr->stVencAttr.u32BitStreamBufSize = streambuf;
    }

    if (p_enType == PT_H265) {
        pVencChnAttr->stRcAttr.enRcMode = VENC_RC_MODE_H265VBR;
        pVencChnAttr->stRcAttr.stH265Vbr.bQpMapEnable = HB_TRUE;
        pVencChnAttr->stRcAttr.stH265Vbr.u32IntraQp = 20;
        pVencChnAttr->stRcAttr.stH265Vbr.u32IntraPeriod = 60;
        pVencChnAttr->stRcAttr.stH265Vbr.u32FrameRate = 30;
    }
    if (p_enType == PT_H264) {
        pVencChnAttr->stRcAttr.enRcMode = VENC_RC_MODE_H264VBR;
        pVencChnAttr->stRcAttr.stH264Vbr.bQpMapEnable = HB_TRUE;
        pVencChnAttr->stRcAttr.stH264Vbr.u32IntraQp = 20;
        pVencChnAttr->stRcAttr.stH264Vbr.u32IntraPeriod = 60;
        pVencChnAttr->stRcAttr.stH264Vbr.u32FrameRate = 30;
        pVencChnAttr->stVencAttr.stAttrH264.h264_profile = HB_H264_PROFILE_MP;
        pVencChnAttr->stVencAttr.stAttrH264.h264_level = HB_H264_LEVEL1;
    }

    pVencChnAttr->stGopAttr.u32GopPresetIdx = 3;
    pVencChnAttr->stGopAttr.s32DecodingRefreshType = 2;


    return 0;
}

int sample_venc_common_init()
{
    int s32Ret;
    
    s32Ret = HB_VENC_Module_Init();
    if (s32Ret) {
        printf("HB_VENC_Module_Init: %d\n", s32Ret);
    }

    return s32Ret;
}

int sample_venc_common_deinit()
{
    int s32Ret;

    s32Ret = HB_VENC_Module_Uninit();
    if (s32Ret) {
        printf("HB_VENC_Module_Init: %d\n", s32Ret);
    }

    return s32Ret;
}

int sample_venc_init(int VeChn, int type, int width, int height, int bits)
{
    int s32Ret;
    VENC_CHN_ATTR_S vencChnAttr;
    VENC_RC_ATTR_S *pstRcParam;
    PAYLOAD_TYPE_E ptype;

    if (type == 1) {
        ptype = PT_H264;
    } else if (type == 2) {
        ptype = PT_H265;
    } else {
        ptype = PT_JPEG;
    }

    VencChnAttrInit(&vencChnAttr, ptype, width, height, HB_PIXEL_FORMAT_NV12);

    s32Ret = HB_VENC_CreateChn(VeChn, &vencChnAttr);
    if (s32Ret != 0) {
        printf("HB_VENC_CreateChn %d failed, %x.\n", VeChn, s32Ret);
        return -1;
    }
    // HB_VENC_Module_Uninit();
    if (ptype == PT_H264) {
        pstRcParam = &(vencChnAttr.stRcAttr);
        vencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
        s32Ret = HB_VENC_GetRcParam(VeChn, pstRcParam);
        if (s32Ret != 0) {
            printf("HB_VENC_GetRcParam failed.\n");
            return -1;
        }

        printf(" vencChnAttr.stRcAttr.enRcMode = %d mmmmmmmmmmmmmmmmmm   \n",
                vencChnAttr.stRcAttr.enRcMode);
        printf(" u32VbvBufferSize = %d mmmmmmmmmmmmmmmmmm   \n",
                vencChnAttr.stRcAttr.stH264Cbr.u32VbvBufferSize);

        pstRcParam->stH264Cbr.u32BitRate = bits;
        pstRcParam->stH264Cbr.u32FrameRate = 30;
        pstRcParam->stH264Cbr.u32IntraPeriod = 60;
        pstRcParam->stH264Cbr.u32VbvBufferSize = 3000;
    } else if (ptype == PT_H265) {
        pstRcParam = &(vencChnAttr.stRcAttr);
        vencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H265CBR;
        s32Ret = HB_VENC_GetRcParam(VeChn, pstRcParam);
        if (s32Ret != 0) {
            printf("HB_VENC_GetRcParam failed.\n");
            return -1;
        }
        printf(" m_VencChnAttr.stRcAttr.enRcMode = %d mmmmmmmmmmmmmmmmmm   \n",
                vencChnAttr.stRcAttr.enRcMode);
        printf(" u32VbvBufferSize = %d mmmmmmmmmmmmmmmmmm   \n",
                vencChnAttr.stRcAttr.stH265Cbr.u32VbvBufferSize);

        pstRcParam->stH265Cbr.u32BitRate = bits;
        pstRcParam->stH265Cbr.u32FrameRate = 30;
        pstRcParam->stH265Cbr.u32IntraPeriod = 30;
        pstRcParam->stH265Cbr.u32VbvBufferSize = 3000;
    }

    s32Ret = HB_VENC_SetChnAttr(VeChn, &vencChnAttr);  // config
    if (s32Ret != 0) {
        printf("HB_VENC_SetChnAttr failed\n");
        return -1;
    }

#ifdef VENC_BIND
    if (VeChn == 1) {
      struct HB_SYS_MOD_S src_mod, dst_mod;
      src_mod.enModId = HB_ID_VPS;
      src_mod.s32DevId = 0;
      src_mod.s32ChnId = 5;
      dst_mod.enModId = HB_ID_VENC;
      dst_mod.s32DevId = 0;
      dst_mod.s32ChnId = VeChn;
      s32Ret = HB_SYS_Bind(&src_mod, &dst_mod);
      if (s32Ret != 0) {
        printf("HB_SYS_Bind failed\n");
        return -1;
      }
    } else {
      struct HB_SYS_MOD_S src_mod, dst_mod;
      src_mod.enModId = HB_ID_VPS;
      src_mod.s32DevId = 0;
      src_mod.s32ChnId = VeChn;
      dst_mod.enModId = HB_ID_VENC;
      dst_mod.s32DevId = 0;
      dst_mod.s32ChnId = VeChn;
      s32Ret = HB_SYS_Bind(&src_mod, &dst_mod);
      if (s32Ret != 0) {
        printf("HB_SYS_Bind failed\n");
        return -1;
      }
    }
#endif

    return 0;
}

int sample_venc_start(int VeChn)
{
    int s32Ret = 0;

    VENC_RECV_PIC_PARAM_S pstRecvParam;
    pstRecvParam.s32RecvPicNum = 0;  // unchangable

    s32Ret = HB_VENC_StartRecvFrame(VeChn, &pstRecvParam);
    if (s32Ret != 0) {
        printf("HB_VENC_StartRecvFrame failed\n");
        return -1;
    }

    return 0;
}

int sample_venc_stop(int VeChn)
{
    int s32Ret = 0;

    s32Ret = HB_VENC_StopRecvFrame(VeChn);
    if (s32Ret != 0) {
        printf("HB_VENC_StopRecvFrame failed\n");
        return -1;
    }

    return 0;
}

int sample_venc_deinit(int VeChn)
{
    int s32Ret = 0;

    s32Ret = HB_VENC_DestroyChn(VeChn);
    if (s32Ret != 0) {
        printf("HB_VENC_DestroyChn failed\n");
        return -1;
    }

    return 0;
}

#if 0
//TEST_F(HapiVideoTest, test_encoding_case_h264_640x480_420p) {
int sample_venc_test() 
{
    int32_t ret = 0;
    int fd;
    FILE *inFile;
    FILE *outFile;
    FILE *inFileBak;
    int noMoreInput = 0;
    int lastStream = 0;
    int step = 0;
    int i = 0;

    VENC_CHN VeChn = 0;
    int Width = 640;
    int Height = 480;

    VencChnAttrInit(PT_H264, Width, Height);

    char *inputFileName = "./venc/yuv/input_640x480_yuv420p.yuv";
    char *inputFileNameBak = "./venc/video/output_640x480_yuv420p.yuv";
    char *outputFileName = "./venc/video/output_strame_640x480.h264";

    ASSERT_NE(inputFileName, nullptr);
    ASSERT_NE(inputFileNameBak, nullptr);
    ASSERT_NE(outputFileName, nullptr);

    inFile = fopen(inputFileName, "rb");
    ASSERT_NE(inFile, nullptr);

    inFileBak = fopen(inputFileNameBak, "wb");
    ASSERT_NE(inFileBak, nullptr);

    outFile = fopen(outputFileName, "wb");
    ASSERT_NE(outFile, nullptr);

    char* mmz_vaddr[10];
    for (i=0;i<10;i++)
    {
        mmz_vaddr[i] = NULL;
    }
    uint64_t mmz_paddr[10];
    memset(mmz_paddr, 0, sizeof(mmz_paddr));

    int mmz_size = Width * Height * 3 / 2;

    int s32Ret = HB_VP_Init();
    if (s32Ret != 0) {
        printf("vp_init fail s32Ret = %d !\n",s32Ret);
    }

    for (i = 0; i < 10; i++) {
        s32Ret = HB_SYS_Alloc(&mmz_paddr[i], (void **)&mmz_vaddr[i], mmz_size);
        if (s32Ret == 0) {
        printf("mmzAlloc paddr = 0x%x, vaddr = 0x%x i = %d \n", mmz_paddr[i], mmz_vaddr[i],i);
        }
    }

    int s32ReadLen = 0;
    for (i = 0; i < 10; i++) {
        s32ReadLen = fread(mmz_vaddr[i], 1, mmz_size, inFile);
        printf("s32ReadLen = %d !!!!!\n", s32ReadLen);
        if (s32ReadLen == 0) {
        printf("read over !!!\n");
        }

        fwrite(mmz_vaddr[i], s32ReadLen, 1, inFileBak);
    }


    s32Ret = HB_VENC_CreateChn(VeChn, &m_VencChnAttr);
    ASSERT_EQ(s32Ret, (int32_t)0);

    VENC_RC_ATTR_S *pstRcParam = &(m_VencChnAttr.stRcAttr);
    m_VencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
    s32Ret = HB_VENC_GetRcParam(VeChn, pstRcParam);
    EXPECT_EQ(s32Ret, (int32_t)0);
    printf(" m_VencChnAttr.stRcAttr.enRcMode = %d mmmmmmmmmmmmmmmmmm   \n",
            m_VencChnAttr.stRcAttr.enRcMode);
    printf(" u32VbvBufferSize = %d mmmmmmmmmmmmmmmmmm   \n",
            m_VencChnAttr.stRcAttr.stH264Cbr.u32VbvBufferSize);


    pstRcParam->stH264Cbr.u32BitRate = 3000;
    pstRcParam->stH264Cbr.u32FrameRate = 30;
    pstRcParam->stH264Cbr.u32IntraPeriod = 30;

    pstRcParam->stH264Cbr.u32VbvBufferSize = 3000;

    
    ASSERT_EQ(HB_VENC_SetChnAttr(VeChn, &m_VencChnAttr), 0);  // config

    VENC_RECV_PIC_PARAM_S pstRecvParam;
    pstRecvParam.s32RecvPicNum = 0;  // unchangable

    s32Ret = HB_VENC_StartRecvFrame(VeChn, &pstRecvParam);
    EXPECT_EQ(s32Ret, (int32_t)0);

    VIDEO_FRAME_S pstFrame;
    VIDEO_STREAM_S pstStream;
    memset(&pstFrame, 0, sizeof(VIDEO_FRAME_S));
    memset(&pstStream, 0, sizeof(VIDEO_STREAM_S));

    pstFrame.stVFrame.width = Width;
    pstFrame.stVFrame.height = Height;
    pstFrame.stVFrame.size = mmz_size;

    int offset = Width * Height;
    for (i=0;i<10;i++)
    {
        pstFrame.stVFrame.phy_ptr[0] = mmz_paddr[i];
        pstFrame.stVFrame.phy_ptr[1] = mmz_paddr[i] + offset;

        pstFrame.stVFrame.vir_ptr[0] = mmz_vaddr[i];
        pstFrame.stVFrame.vir_ptr[1] = mmz_vaddr[i] + offset;

        if (i == 9)
        {
        pstFrame.stVFrame.frame_end = HB_TRUE;
        }

        s32Ret = HB_VENC_SendFrame(VeChn, &pstFrame, 3000);
        EXPECT_EQ(s32Ret, (int32_t)0);

        usleep(300000);

        s32Ret = HB_VENC_GetStream(VeChn, &pstStream, 3000);
        EXPECT_EQ(s32Ret, (int32_t)0);

        fwrite(pstStream.pstPack.vir_ptr, pstStream.pstPack.size, 1, outFile);
        printf("i = %d   pstStream.pstPack.size = %d !!!!!\n", i, pstStream.pstPack.size);
        
    }

    sleep(3);

    s32Ret = HB_VENC_StopRecvFrame(VeChn);
    EXPECT_EQ(s32Ret, (int32_t)0);

    s32Ret = HB_VENC_DestroyChn(VeChn);
    ASSERT_EQ(s32Ret, (int32_t)0);

    for (i = 0; i < 10; i++) {
        s32Ret = HB_SYS_Free(mmz_paddr[i], mmz_vaddr[i]);
        if (s32Ret == 0) {
        printf("mmzFree paddr = 0x%x, vaddr = 0x%x i = %d \n", mmz_paddr[i],
                mmz_vaddr[i], i);
        }
    }

    s32Ret = HB_VP_Exit();
    if (s32Ret == 0) printf("vp exit ok!\n");

    if (inFile) fclose(inFile);

    if (inFileBak) fclose(inFileBak);
    
    if (outFile) fclose(outFile);
}
#endif