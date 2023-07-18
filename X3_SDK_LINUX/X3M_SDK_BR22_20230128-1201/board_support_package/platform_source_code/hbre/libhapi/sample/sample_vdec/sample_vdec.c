/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#include <stdio.h>
#include <stdlib.h>
 #include <unistd.h>
#include "hb_comm_vdec.h"
#include "hb_comm_venc.h"
#include "hb_comm_video.h"
#include "hb_common.h"
#include "hb_type.h"
#include "hb_vdec.h"
#include "hb_venc.h"
#include "hb_vp_api.h"
#ifdef __cplusplus
extern "C" {
#endif				/* __cplusplus */
#include <libavformat/avformat.h>
#include <libavutil/timestamp.h>
#ifdef __cplusplus
}
#endif				/* __cplusplus */
#include "sample.h"

void VdecChnAttrInit(VDEC_CHN_ATTR_S *pVdecChnAttr,
    PAYLOAD_TYPE_E p_enType, int p_Width, int p_Height)
{
    memset(pVdecChnAttr, 0, sizeof(VDEC_CHN_ATTR_S));
  pVdecChnAttr->enType = p_enType;
  pVdecChnAttr->enMode = VIDEO_MODE_FRAME;
  pVdecChnAttr->enPixelFormat = HB_PIXEL_FORMAT_NV12;
  pVdecChnAttr->u32FrameBufCnt = 3;
  pVdecChnAttr->u32StreamBufCnt = 3;
  pVdecChnAttr->u32StreamBufSize = p_Width * p_Height/2;
  pVdecChnAttr->bExternalBitStreamBuf  = HB_TRUE;

  if (p_enType == PT_H265) {
    pVdecChnAttr->stAttrH265.bandwidth_Opt = HB_TRUE;
    pVdecChnAttr->stAttrH265.enDecMode = VIDEO_DEC_MODE_NORMAL;
    pVdecChnAttr->stAttrH265.enOutputOrder = VIDEO_OUTPUT_ORDER_DISP;
    pVdecChnAttr->stAttrH265.cra_as_bla = HB_FALSE;
    pVdecChnAttr->stAttrH265.dec_temporal_id_mode = 0;
    pVdecChnAttr->stAttrH265.target_dec_temporal_id_plus1 = 2;
  }
  if (p_enType == PT_H264) {
    pVdecChnAttr->stAttrH264.bandwidth_Opt = HB_TRUE;
    pVdecChnAttr->stAttrH264.enDecMode = VIDEO_DEC_MODE_NORMAL;
    pVdecChnAttr->stAttrH264.enOutputOrder = VIDEO_OUTPUT_ORDER_DISP;
    // pVdecChnAttr->vlc_buf_size = 2100*1024;
  }
  if (p_enType == PT_JPEG) {
    pVdecChnAttr->bExternalBitStreamBuf  = HB_FALSE;
    pVdecChnAttr->u32StreamBufSize = (p_Width * p_Height * 3/2+1024)&~0x3ff;
    pVdecChnAttr->stAttrJpeg.enMirrorFlip = DIRECTION_NONE;
    pVdecChnAttr->stAttrJpeg.enRotation = CODEC_ROTATION_0;
    pVdecChnAttr->stAttrJpeg.stCropCfg.bEnable = HB_FALSE;
  }
}


#define SET_BYTE(_p, _b) \
    *_p++ = (unsigned char)_b;

#define SET_BUFFER(_p, _buf, _len) \
    memcpy(_p, _buf, _len); \
    (_p) += (_len);

static int build_dec_seq_header(uint8_t *        pbHeader,
    const PAYLOAD_TYPE_E p_enType, const AVStream* st, int* sizelength)
{
    AVCodecParameters* avc = st->codecpar;

    uint8_t* pbMetaData = avc->extradata;
    int nMetaData = avc->extradata_size;
    uint8_t* p =    pbMetaData;
    uint8_t *a =    p + 4 - ((long) p & 3);
    uint8_t* t =    pbHeader;
    int         size;
    int         sps, pps, i, nal;

    size = 0;
    *sizelength = 4;  // default size length(in bytes) = 4
    if (p_enType == PT_H264) {
        if (nMetaData > 1 && pbMetaData && pbMetaData[0] == 0x01) {
            // check mov/mo4 file format stream
            p += 4;
            *sizelength = (*p++ & 0x3) + 1;
            sps = (*p & 0x1f);  // Number of sps
            p++;
            for (i = 0; i < sps; i++) {
                nal = (*p << 8) + *(p + 1) + 2;
                SET_BYTE(t, 0x00);
                SET_BYTE(t, 0x00);
                SET_BYTE(t, 0x00);
                SET_BYTE(t, 0x01);
                SET_BUFFER(t, p+2, nal-2);
                p += nal;
                size += (nal - 2 + 4);  // 4 => length of start code to be inserted
            }

            pps = *(p++);  // number of pps
            for (i = 0; i < pps; i++)
            {
                nal = (*p << 8) + *(p + 1) + 2;
                SET_BYTE(t, 0x00);
                SET_BYTE(t, 0x00);
                SET_BYTE(t, 0x00);
                SET_BYTE(t, 0x01);
                SET_BUFFER(t, p+2, nal-2);
                p += nal;
                size += (nal - 2 + 4);  // 4 => length of start code to be inserted
            }
        } else if(nMetaData > 3) {
            size = -1;  // return to meaning of invalid stream data;
            for (; p < a; p++) {
                if (p[0] == 0 && p[1] == 0 && p[2] == 1)  {
                    // find startcode
                    size = avc->extradata_size;
                    if (!pbHeader || !pbMetaData)
                        return 0;
                    SET_BUFFER(pbHeader, pbMetaData, size);
                    break;
                }
            }
        }
    } else if (p_enType == PT_H265) {
        if (nMetaData > 1 && pbMetaData && pbMetaData[0] == 0x01) {
            static const int8_t nalu_header[4] = { 0, 0, 0, 1 };
            int numOfArrays = 0;
            uint16_t numNalus = 0;
            uint16_t nalUnitLength = 0;
            uint32_t offset = 0;

            p += 21;
            *sizelength = (*p++ & 0x3) + 1;
            numOfArrays = *p++;

            while(numOfArrays--) {
                p++;   // NAL type
                numNalus = (*p << 8) + *(p + 1);
                p+=2;
                for(i = 0;i < numNalus;i++)
                {
                    nalUnitLength = (*p << 8) + *(p + 1);
                    p+=2;
                    // if(i == 0)
                    {
                        memcpy(pbHeader + offset, nalu_header, 4);
                        offset += 4;
                        memcpy(pbHeader + offset, p, nalUnitLength);
                        offset += nalUnitLength;
                    }
                    p += nalUnitLength;
                }
            }

            size = offset;
        } else if(nMetaData > 3) {
            size = -1;  // return to meaning of invalid stream data;

            for (; p < a; p++)
            {
                if (p[0] == 0 && p[1] == 0 && p[2] == 1)  // find startcode
                {
                    size = avc->extradata_size;
                    if (!pbHeader || !pbMetaData)
                        return 0;
                    SET_BUFFER(pbHeader, pbMetaData, size);
                    break;
                }
            }
        }
    } else {
        SET_BUFFER(pbHeader, pbMetaData, nMetaData);
        size = nMetaData;
    }

    return size;
}

int sample_vdec_init(int p_enType, int vdecChn)
{
    int s32Ret = 0;
    VDEC_CHN hb_VDEC_Chn = vdecChn;
    PAYLOAD_TYPE_E hb_CodecType = (PAYLOAD_TYPE_E)p_enType;
    VDEC_CHN_ATTR_S hb_VdecChnAttr;
    int width = 1920;
    int height = 1080;

    VdecChnAttrInit(&hb_VdecChnAttr, hb_CodecType, width, height);

    s32Ret = HB_VDEC_CreateChn(hb_VDEC_Chn, &hb_VdecChnAttr);
    if (s32Ret != 0) {
        printf("HB_VDEC_CreateChn failed %x\n", s32Ret);
        return s32Ret;
    }

    // EXPECT_EQ(HB_VDEC_GetChnAttr(hb_VDEC_Chn, &hb_VencChnAttr), 0);

    s32Ret = HB_VDEC_SetChnAttr(hb_VDEC_Chn, &hb_VdecChnAttr);  // config
    if (s32Ret != 0) {
        printf("HB_VDEC_SetChnAttr failed %x\n", s32Ret);
        return s32Ret;
    }

    s32Ret = HB_VDEC_StartRecvStream(hb_VDEC_Chn);
    if (s32Ret != 0) {
        printf("HB_VDEC_StartRecvStream failed %x\n", s32Ret);
        return s32Ret;
    }

    return 0;
}

int sample_vdec_stop(int hb_VDEC_Chn)
{
    int s32Ret = 0;

    s32Ret = HB_VDEC_StopRecvStream(hb_VDEC_Chn);
    if (s32Ret != 0) {
        printf("HB_VDEC_StopRecvStream failed %x\n", s32Ret);
        return s32Ret;
    }

    return 0;
}

int sample_vdec_deinit(int hb_VDEC_Chn)
{
    int s32Ret = 0;

    // s32Ret = HB_VDEC_StopRecvStream(hb_VDEC_Chn);
    // if (s32Ret != 0) {
    //     printf("HB_VDEC_StopRecvStream failed %d\n", s32Ret);
    //     return s32Ret;
    // }

    s32Ret = HB_VDEC_DestroyChn(hb_VDEC_Chn);
    if (s32Ret != 0) {
        printf("HB_VDEC_ReleaseFrame failed %d\n", s32Ret);
        return s32Ret;
    }

    return 0;
}

int sample_vdec_bind_vot(int vdecChn, int votChn)
{
    int s32Ret = 0;
    struct HB_SYS_MOD_S src_mod, dst_mod;

	src_mod.enModId = HB_ID_VDEC;
	src_mod.s32DevId = vdecChn;
	src_mod.s32ChnId = 0;
	dst_mod.enModId = HB_ID_VOT;
	dst_mod.s32DevId = 0;
	dst_mod.s32ChnId = votChn;
	s32Ret = HB_SYS_Bind(&src_mod, &dst_mod);
	if (s32Ret != 0)
		printf("HB_SYS_Bind failed\n");

    return s32Ret;
}

int sample_vdec_bind_vps(int vdecChn, int vpsGrp, int vpsChn)
{
    int s32Ret = 0;
    struct HB_SYS_MOD_S src_mod, dst_mod;

	src_mod.enModId = HB_ID_VDEC;
	src_mod.s32DevId = vdecChn;
	src_mod.s32ChnId = 0;
	dst_mod.enModId = HB_ID_VPS;
	dst_mod.s32DevId = vpsGrp;
	dst_mod.s32ChnId = vpsChn;
	s32Ret = HB_SYS_Bind(&src_mod, &dst_mod);
	if (s32Ret != 0)
		printf("HB_SYS_Bind failed\n");

    return s32Ret;
}

int sample_vdec_bind_venc(int vdecChn, int vencChn)
{
    int s32Ret = 0;
    struct HB_SYS_MOD_S src_mod, dst_mod;

	src_mod.enModId = HB_ID_VDEC;
	src_mod.s32DevId = vdecChn;
	src_mod.s32ChnId = 0;
	dst_mod.enModId = HB_ID_VENC;
	dst_mod.s32DevId = vencChn;
	dst_mod.s32ChnId = 0;
	s32Ret = HB_SYS_Bind(&src_mod, &dst_mod);
	if (s32Ret != 0)
		printf("HB_SYS_Bind failed\n");

    return s32Ret;
}

int sample_vdec_unbind_vot(int vdecChn, int votChn)
{
    int s32Ret = 0;
    struct HB_SYS_MOD_S src_mod, dst_mod;

	src_mod.enModId = HB_ID_VDEC;
	src_mod.s32DevId = vdecChn;
	src_mod.s32ChnId = 0;
	dst_mod.enModId = HB_ID_VOT;
	dst_mod.s32DevId = 0;
	dst_mod.s32ChnId = votChn;
	s32Ret = HB_SYS_UnBind(&src_mod, &dst_mod);
	if (s32Ret != 0)
		printf("HB_SYS_Bind failed\n");

    return s32Ret;
}

int sample_vdec_unbind_vps(int vdecChn, int vpsGrp, int vpsChn)
{
    int s32Ret = 0;
    struct HB_SYS_MOD_S src_mod, dst_mod;

	src_mod.enModId = HB_ID_VDEC;
	src_mod.s32DevId = vdecChn;
	src_mod.s32ChnId = 0;
	dst_mod.enModId = HB_ID_VPS;
	dst_mod.s32DevId = vpsGrp;
	dst_mod.s32ChnId = vpsChn;
	s32Ret = HB_SYS_UnBind(&src_mod, &dst_mod);
	if (s32Ret != 0)
		printf("HB_SYS_UnBind failed\n");

    return s32Ret;
}

int sample_vdec_unbind_venc(int vdecChn, int vencChn)
{
    int s32Ret = 0;
    struct HB_SYS_MOD_S src_mod, dst_mod;

	src_mod.enModId = HB_ID_VDEC;
	src_mod.s32DevId = vdecChn;
	src_mod.s32ChnId = 0;
	dst_mod.enModId = HB_ID_VENC;
	dst_mod.s32DevId = vencChn;
	dst_mod.s32ChnId = 0;
	s32Ret = HB_SYS_UnBind(&src_mod, &dst_mod);
	if (s32Ret != 0)
		printf("HB_SYS_UnBind failed\n");

    return s32Ret;
}

int sample_jpeg_decoding()
{
    const char *hb_inputFileName = "./rcv.jpeg";
    const char *hb_outputFileName = "./rcv.yuv";
    VDEC_CHN hb_VDEC_Chn = 0;
    PAYLOAD_TYPE_E hb_CodecType = PT_H264;
    int width = 1920;
    int height = 1088;
    int s32Ret;
    int bufSize;
    int fileSize;
    FILE *inFile;
    FILE *outFile;

    inFile = fopen(hb_inputFileName, "rb");
    if (inFile == NULL) {
        printf("fopen %s failed\n", hb_inputFileName);
        return -1;
    }
    outFile = fopen(hb_outputFileName, "wb");
    if (outFile == NULL) {
        printf("fopen %s failed\n", hb_outputFileName);
        return -1;
    }

    fseek(inFile, 0, SEEK_END);
	fileSize = ftell(inFile);
	fseek(inFile, 0, SEEK_SET);

    char* mmz_vaddr[1];
    mmz_vaddr[0] = NULL;

    uint64_t mmz_paddr[1];
    memset(mmz_paddr, 0, sizeof(mmz_paddr));

    int mmz_size = 1.5*width*height;
    int mmz_cnt = 1;

    VP_CONFIG_S struVpConf;
    memset(&struVpConf, 0x00, sizeof(VP_CONFIG_S));
    struVpConf.u32MaxPoolCnt = 32;
    HB_VP_SetConfig(&struVpConf);

    s32Ret = HB_VP_Init();
    if (s32Ret != 0) {
        printf("vp_init fail s32Ret = %d !\n", s32Ret);
    }

    s32Ret = HB_SYS_Alloc(&mmz_paddr[0], (void **)&mmz_vaddr[0], mmz_size);
    if (s32Ret == 0) {
        printf("mmzAlloc paddr = 0x%x, vaddr = 0x%x i = %d \n",
                mmz_paddr[0], mmz_vaddr[0], 0);
    }

    VIDEO_FRAME_S pstFrame;
    VIDEO_STREAM_S pstStream;
    memset(&pstFrame, 0, sizeof(VIDEO_FRAME_S));
    memset(&pstStream, 0, sizeof(VIDEO_STREAM_S));

    bufSize = fread(mmz_vaddr[0], 1, fileSize, inFile);
    if (bufSize != fileSize) {
        printf("read file failed\n");
        return -1;
    }

    pstStream.pstPack.phy_ptr = mmz_paddr[0];
    pstStream.pstPack.vir_ptr = mmz_vaddr[0];
    pstStream.pstPack.pts = 0;
    pstStream.pstPack.src_idx = 0;
    pstStream.pstPack.size = bufSize;
    pstStream.pstPack.stream_end = HB_FALSE;

    printf("[pstStream] pts:%d, vir_ptr:%lld, size:%d\n",
            pstStream.pstPack.pts,
            pstStream.pstPack.vir_ptr,
            pstStream.pstPack.size);

    s32Ret = HB_VDEC_SendStream(hb_VDEC_Chn, &pstStream, 3000);
    if (s32Ret != 0) {
        printf("HB_VDEC_SendStream failed %d\n", s32Ret);
    }

    if (1) {
        s32Ret = HB_VDEC_GetFrame(hb_VDEC_Chn, &pstFrame, 3000);
        if (s32Ret != 0) {
            printf("HB_VDEC_GetFrame failed %d\n", s32Ret);
        }

        fwrite(pstFrame.stVFrame.vir_ptr[0],
                pstFrame.stVFrame.size, 1, outFile);
        printf("[pstFrame] pts:%d, vir_ptr:%lld, frame_size:[%d %d, %d]\n",
                pstStream.pstPack.pts,
                pstFrame.stVFrame.vir_ptr[0],
                pstFrame.stVFrame.width,
                pstFrame.stVFrame.height,
                pstFrame.stVFrame.size);
        s32Ret = HB_VDEC_ReleaseFrame(hb_VDEC_Chn, &pstFrame);
        if (s32Ret != 0) {
            printf("HB_VDEC_ReleaseFrame failed %d\n", s32Ret);
        }
    }

    fclose(inFile);
    fclose(outFile);

    return 0;
}

void do_sync_decoding(void *param)
{
    int i = 0, error = 0;
    int s32Ret = 0;
    FILE *inFile = NULL;
    FILE *outFile = NULL;
    VDEC_CHN_ATTR_S hb_VdecChnAttr;
    vdecParam *vdecparam;
    char *hb_inputFileName;
    // const char *hb_outputFileName = "./4k.yuv";
    VDEC_CHN hb_VDEC_Chn = 0;
    PAYLOAD_TYPE_E hb_CodecType = PT_H264;
    int width = g_width;
    int height = g_height;

    vdecparam = (vdecParam*)param;
    hb_VDEC_Chn = vdecparam->vdecchn;

    // HB_VDEC_Module_Init();
    // VdecChnAttrInit(&hb_VdecChnAttr, PT_H264, width, height);
    // if (g_vdecfilename != NULL) {
    //     hb_inputFileName = g_vdecfilename;
    // } else {
    //     hb_inputFileName = "./rcv.h264";
    // }
    hb_inputFileName = vdecparam->fname;
    inFile = fopen(hb_inputFileName, "rb");
    if (inFile == NULL) {
        printf("fopen %s failed\n", hb_inputFileName);
        return;
    }
    // outFile = fopen(hb_outputFileName, "wb");
    // if (outFile == NULL) {
    //     printf("fopen %s failed\n", hb_outputFileName);
    //     return;
    // }

    char* mmz_vaddr[5];
    for (i = 0; i < 5; i++)
    {
        mmz_vaddr[i] = NULL;
    }
    uint64_t mmz_paddr[5];
    memset(mmz_paddr, 0, sizeof(mmz_paddr));

    int mmz_size = width*height;
    int mmz_cnt = 5;
    static int vpinit = 0;
    if (vpinit == 0) {
        VP_CONFIG_S struVpConf;
        memset(&struVpConf, 0x00, sizeof(VP_CONFIG_S));
        struVpConf.u32MaxPoolCnt = 32;
        HB_VP_SetConfig(&struVpConf);

        s32Ret = HB_VP_Init();
        if (s32Ret != 0) {
            printf("vp_init fail s32Ret = %d !\n",s32Ret);
        }
        vpinit = 1;
    }
    for (i = 0; i < mmz_cnt; i++) {
        s32Ret = HB_SYS_Alloc(&mmz_paddr[i], (void **)&mmz_vaddr[i], mmz_size);
        if (s32Ret == 0) {
            printf("mmzAlloc paddr = 0x%x, vaddr = 0x%x i = %d \n",
                    mmz_paddr[i], mmz_vaddr[i],i);
        }
    }    

    VIDEO_FRAME_S pstFrame;
    VIDEO_STREAM_S pstStream;
    memset(&pstFrame, 0, sizeof(VIDEO_FRAME_S));
    memset(&pstStream, 0, sizeof(VIDEO_STREAM_S));

    AVFormatContext *avContext = NULL;
    AVPacket avpacket = {0};
    int videoIndex;
    uint8_t *seqHeader = NULL;
    int seqHeaderSize = 0;
    uint8_t *picHeader = NULL;
    int picHeaderSize;
    int firstPacket = 1;
    int eos = 0;
    int bufSize = 0;
    int lastFrame = 0;
    int count = 0;
    int mmz_index = 0;

    s32Ret = avformat_open_input(&avContext, hb_inputFileName, 0, 0);
    s32Ret = avformat_find_stream_info(avContext, 0);
    videoIndex = av_find_best_stream(avContext, AVMEDIA_TYPE_VIDEO, -1, -1, NULL, 0);
    av_init_packet(&avpacket);

    do {
        VDEC_CHN_STATUS_S pstStatus;
        HB_VDEC_QueryStatus(hb_VDEC_Chn, &pstStatus);
        if (pstStatus.cur_input_buf_cnt >= (uint32_t)mmz_cnt) {
            usleep(10000);
            continue;
        }
        usleep(30000);
        if (!avpacket.size) {
            error = av_read_frame(avContext, &avpacket);
        }
        if (error < 0) {
            if (error == AVERROR_EOF || avContext->pb->eof_reached == HB_TRUE) {
                printf("There is no more input data, %d!\n", avpacket.size);
            } else {
                printf("Failed to av_read_frame error(0x%08x)\n", error);
            }
            if(vdecparam->loop) {
                if (avContext)
                    avformat_close_input(&avContext);
                avContext = NULL;
                // avpacket = {0};
                avformat_open_input(&avContext, hb_inputFileName, 0, 0);
                avformat_find_stream_info(avContext, 0);
                videoIndex = av_find_best_stream(avContext, AVMEDIA_TYPE_VIDEO, -1, -1, NULL, 0);
                av_init_packet(&avpacket);
                continue;
            } else
                eos = 1;
            break;
        } else {
            seqHeaderSize = 0;
            mmz_index = count % mmz_cnt;
            if (firstPacket) {
                AVCodecParameters* codec;
                int retSize = 0;
                codec = avContext->streams[videoIndex]->codecpar;
                seqHeader = (uint8_t*)malloc(codec->extradata_size + 1024);
                if (seqHeader == NULL) {
                    printf("Failed to mallock seqHeader\n");
                    eos = 1;
                    break;
                }
                memset((void*)seqHeader, 0x00, codec->extradata_size + 1024);

                seqHeaderSize = build_dec_seq_header(seqHeader,
                        hb_CodecType, avContext->streams[videoIndex], &retSize);
                if (seqHeaderSize < 0) {
                    printf("Failed to build seqHeader\n");
                    eos = 1;
                    break;
                }
                firstPacket = 0;
            }
            if (avpacket.size <= mmz_size) {
                if (seqHeaderSize) {
                    memcpy((void*)mmz_vaddr[mmz_index],
                            (void*)seqHeader, seqHeaderSize);
                    bufSize = seqHeaderSize;
                } else {
                    memcpy((void*)mmz_vaddr[mmz_index],
                            (void*)avpacket.data, avpacket.size);
                    bufSize = avpacket.size;
                    av_packet_unref(&avpacket);
                    avpacket.size = 0;
                }
            } else {
                printf("The external stream buffer is too small!"
                        "avpacket.size:%d, mmz_size:%d\n",
                        avpacket.size, mmz_size);
                eos = 1;
                break;
            }
            if (seqHeader) {
                free(seqHeader);
                seqHeader = NULL;
            }
        }
        pstStream.pstPack.phy_ptr = mmz_paddr[mmz_index];
        pstStream.pstPack.vir_ptr = mmz_vaddr[mmz_index];
        pstStream.pstPack.pts = count++;
        pstStream.pstPack.src_idx = mmz_index;
        if (!eos) {
            pstStream.pstPack.size = bufSize;
            pstStream.pstPack.stream_end = HB_FALSE;
        } else {
            pstStream.pstPack.size = 0;
            pstStream.pstPack.stream_end = HB_TRUE;
        }
        // printf("[pstStream] pts:%d, vir_ptr:%lld, size:%d\n",
        //         pstStream.pstPack.pts,
        //         pstStream.pstPack.vir_ptr,
        //         pstStream.pstPack.size);

        s32Ret = HB_VDEC_SendStream(hb_VDEC_Chn, &pstStream, 3000);
        if (s32Ret != 0) {
            printf("HB_VDEC_SendStream failed %d\n", s32Ret);
        }

        // usleep(50000);

        // if (!lastFrame) {
        //     s32Ret = HB_VDEC_GetFrame(hb_VDEC_Chn, &pstFrame, 3000);
        //     if (s32Ret != 0) {
        //         printf("HB_VDEC_GetFrame failed %d\n", s32Ret);
        //     }

        //     fwrite(pstFrame.stVFrame.vir_ptr[0], pstFrame.stVFrame.size, 1, outFile);
        //     printf("[pstFrame] pts:%d, vir_ptr:%lld, frame_size:[%d %d, %d]\n",
        //             pstStream.pstPack.pts,
        //             pstFrame.stVFrame.vir_ptr[0],
        //             pstFrame.stVFrame.width,
        //             pstFrame.stVFrame.height,
        //             pstFrame.stVFrame.size);
        //     s32Ret = HB_VDEC_ReleaseFrame(hb_VDEC_Chn, &pstFrame);
        //     if (s32Ret != 0) {
        //         printf("HB_VDEC_ReleaseFrame failed %d\n", s32Ret);
        //     }
        //     if (pstFrame.stVFrame.frame_end) {
        //         printf("There is no more output data!\n");
        //         lastFrame = 1;
        //         break;
        //     }
        // }

    } while(g_exit == 0);

    for (i = 0; i < mmz_cnt; i++) {
        s32Ret = HB_SYS_Free(mmz_paddr[i], mmz_vaddr[i]);
        if (s32Ret == 0) {
            printf("mmzFree paddr = 0x%x, vaddr = 0x%x i = %d \n", mmz_paddr[i],
                    mmz_vaddr[i], i);
        }
    }

    // s32Ret = HB_VP_Exit();
    // if (s32Ret == 0) {
    //     printf("vp exit ok!\n");
    // }

    if (avContext) avformat_close_input(&avContext);
    if (inFile) fclose(inFile);
    if (outFile) fclose(outFile);

    g_exit = 1;
}
