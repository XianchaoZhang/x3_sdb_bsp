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
#include <ctype.h>
#include "hb_vio_interface.h"
#include "hb_vps_api.h"

#include "hb_comm_venc.h"
#include "hb_venc.h"
#include "hb_vdec.h"
#include "vps_common.h"

#include <arm_neon.h>
struct single_buffer {
	VIDEO_STREAM_S vstream;

	int used;
	pthread_mutex_t mutex;
};

static pthread_t vens_thid[6];
static vencParam vencparam[6] = { 0 };


int VencChnAttrInit(VENC_CHN_ATTR_S * pVencChnAttr, PAYLOAD_TYPE_E p_enType,
		    int p_Width, int p_Height, PIXEL_FORMAT_E pixFmt)
{
	memset(pVencChnAttr, 0, sizeof(VENC_CHN_ATTR_S));
	pVencChnAttr->stVencAttr.enType = p_enType;

	pVencChnAttr->stVencAttr.u32PicWidth = p_Width;
	pVencChnAttr->stVencAttr.u32PicHeight = p_Height;

	pVencChnAttr->stVencAttr.enMirrorFlip = DIRECTION_NONE;
	pVencChnAttr->stVencAttr.enRotation = CODEC_ROTATION_0;
	pVencChnAttr->stVencAttr.stCropCfg.bEnable = HB_FALSE;
	pVencChnAttr->stVencAttr.stAttrH264.h264_profile = 0;

	if (p_enType == PT_JPEG || p_enType == PT_MJPEG) {
		pVencChnAttr->stVencAttr.enPixelFormat = pixFmt;
		pVencChnAttr->stVencAttr.u32BitStreamBufferCount = 1;
		pVencChnAttr->stVencAttr.u32FrameBufferCount = 2;
		pVencChnAttr->stVencAttr.bExternalFreamBuffer = HB_TRUE;
		pVencChnAttr->stVencAttr.stAttrJpeg.dcf_enable = HB_FALSE;
		pVencChnAttr->stVencAttr.stAttrJpeg.quality_factor = 0;
		pVencChnAttr->stVencAttr.stAttrJpeg.restart_interval = 0;
	} else {
		pVencChnAttr->stVencAttr.enPixelFormat = pixFmt;
		pVencChnAttr->stVencAttr.u32BitStreamBufferCount = 5;
		pVencChnAttr->stVencAttr.u32FrameBufferCount = 5;
		pVencChnAttr->stVencAttr.bExternalFreamBuffer = HB_TRUE;
	}

	if (p_enType == PT_H265) {
		pVencChnAttr->stRcAttr.enRcMode = VENC_RC_MODE_H265VBR;
		pVencChnAttr->stRcAttr.stH265Vbr.bQpMapEnable = HB_TRUE;
		pVencChnAttr->stRcAttr.stH265Vbr.u32IntraQp = 20;
		pVencChnAttr->stRcAttr.stH265Vbr.u32IntraPeriod = 20;
		pVencChnAttr->stRcAttr.stH265Vbr.u32FrameRate = 25;
	}
	if (p_enType == PT_H264) {
		pVencChnAttr->stRcAttr.enRcMode = VENC_RC_MODE_H264VBR;
		pVencChnAttr->stRcAttr.stH264Vbr.bQpMapEnable = HB_TRUE;
		pVencChnAttr->stRcAttr.stH264Vbr.u32IntraQp = 20;
		pVencChnAttr->stRcAttr.stH264Vbr.u32IntraPeriod = 20;
		pVencChnAttr->stRcAttr.stH264Vbr.u32FrameRate = 25;
	}

	pVencChnAttr->stGopAttr.u32GopPresetIdx = 6;
	pVencChnAttr->stGopAttr.s32DecodingRefreshType = 2;

	return 0;
}

int dual_venc_common_init()
{
	int s32Ret;
	printf("dual_sample_venc_common_init==begin======\n");
	s32Ret = HB_VENC_Module_Init();
	if (s32Ret) {
		printf("HB_VENC_Module_Init: %d\n", s32Ret);
	}
	printf("dual_sample_venc_common_init=end=======\n");
	return s32Ret;
}

int dual_venc_common_deinit()
{
	int s32Ret;

	s32Ret = HB_VENC_Module_Uninit();
	if (s32Ret) {
		printf("HB_VENC_Module_Init: %d\n", s32Ret);
	}

	return s32Ret;
}

int dual_venc_init(int VeChn, int type, int width, int height, int bits)
{
	int s32Ret;
	VENC_CHN_ATTR_S vencChnAttr;
	VENC_RC_ATTR_S *pstRcParam;
	PAYLOAD_TYPE_E ptype;

	if (type == VENC_H264) {
		ptype = PT_H264;
	} else if (type == VENC_H265) {
		ptype = PT_H265;
	} else if (type == VENC_MJPEG) {
		ptype = PT_MJPEG;
	} else {
		ptype = PT_JPEG;
	}

	VencChnAttrInit(&vencChnAttr, ptype, width, height,
			HB_PIXEL_FORMAT_NV12);

	s32Ret = HB_VENC_CreateChn(VeChn, &vencChnAttr);
	if (s32Ret != 0) {
		printf("HB_VENC_CreateChn %d failed, %x.\n", VeChn, s32Ret);
		return -1;
	}
	if (ptype == PT_H264) {
		pstRcParam = &(vencChnAttr.stRcAttr);
		vencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
		s32Ret = HB_VENC_GetRcParam(VeChn, pstRcParam);
		if (s32Ret != 0) {
			printf("HB_VENC_GetRcParam failed.\n");
			return -1;
		}

		printf
		    (" vencChnAttr.stRcAttr.enRcMode = %d mmmmmmmmmmmmmmmmmm   \n",
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
		printf
		    (" m_VencChnAttr.stRcAttr.enRcMode = %d mmmmmmmmmmmmmmmmmm   \n",
		     vencChnAttr.stRcAttr.enRcMode);
		printf(" u32VbvBufferSize = %d mmmmmmmmmmmmmmmmmm   \n",
		       vencChnAttr.stRcAttr.stH265Cbr.u32VbvBufferSize);

		pstRcParam->stH265Cbr.u32BitRate = bits;
		pstRcParam->stH265Cbr.u32FrameRate = 30;
		pstRcParam->stH265Cbr.u32IntraPeriod = 30;
		pstRcParam->stH265Cbr.u32VbvBufferSize = 3000;
	} else if (ptype == PT_MJPEG) {
		pstRcParam = &(vencChnAttr.stRcAttr);
		vencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_MJPEGFIXQP;
		s32Ret = HB_VENC_GetRcParam(VeChn, pstRcParam);
		if (s32Ret != 0) {
			printf("HB_VENC_GetRcParam failed.\n");
			return -1;
		}

	}

	s32Ret = HB_VENC_SetChnAttr(VeChn, &vencChnAttr);	// config
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

int dual_venc_start(int VeChn)
{
	int s32Ret = 0;

	VENC_RECV_PIC_PARAM_S pstRecvParam;
	pstRecvParam.s32RecvPicNum = 0;	// unchangable

	s32Ret = HB_VENC_StartRecvFrame(VeChn, &pstRecvParam);
	if (s32Ret != 0) {
		printf("HB_VENC_StartRecvFrame failed\n");
		return -1;
	}
	return 0;
}

int dual_venc_stop(int VeChn)
{
	int s32Ret = 0;

	s32Ret = HB_VENC_StopRecvFrame(VeChn);
	if (s32Ret != 0) {
		printf("HB_VENC_StopRecvFrame failed\n");
		return -1;
	}

	return 0;
}

int dual_venc_deinit(int VeChn)
{
	int s32Ret = 0;

	s32Ret = HB_VENC_DestroyChn(VeChn);
	if (s32Ret != 0) {
		printf("HB_VENC_DestroyChn failed\n");
		return -1;
	}

	return 0;
}

void dual_pym_get_thread(void *param)
{
	int ret;
	pym_buffer_t out_pym_buf;
	int width, height;
	struct timeval ts0, ts1, ts2, ts3, ts4, ts5;
	int pts = 0;
	int dumpnum = 10;
	uint32_t color_map[15] =
	{ 0xFFFFFF, 0x000000, 0x808000, 0x00BFFF, 0x00FF00,
	  0xFFFF00, 0x8B4513, 0xFF8C00, 0x800080, 0xFFC0CB,
	  0xFF0000, 0x98F898, 0x00008B, 0x006400, 0x8B0000
	};

	VIDEO_FRAME_S pstFrame;
	printf("pym_get_thread in===========\n");
	time_t start, last = -1;
	int idx = 0;
	while (g_exit == 0) {
		ret = HB_VPS_GetChnFrame(1, 6, &out_pym_buf, 2000);  // ipu chn6
		if (ret != 0) {
			printf("HB_VPS_GetChnFrame error!!!ret %d \n", ret);
			continue;
		}
		start = time(NULL);
		idx++;
		if (start > last) {
			printf("pym fps %d\n", idx);
			last = start;
			idx = 0;
		}

		gettimeofday(&ts0, NULL);
		if (access("./pymdump", F_OK) == 0) {
			system("rm ./pymdump");
			dumpnum = 0;
		}

		if (dumpnum < 3) {
            char fname[32];
            FILE *fw;
            int layer, roi;

            for (layer = 0; layer < 23; layer++) {
                roi = layer%4;
                if (roi == 0) {
                    if (out_pym_buf.pym[layer/4].width == 0)
                        continue;
                } else {
                    if (out_pym_buf.pym_roi[layer/4][roi-1].width == 0)
                        continue;
                }

                if (roi == 0) {
                    sprintf(fname, "pym_%d_%d.yuv", layer/4, dumpnum);
                } else {
                    sprintf(fname, "pymroi_%d_%d_%d.yuv",
                            layer/4, roi-1, dumpnum);
                }
                printf("start dump %s\n", fname);
                fw = fopen(fname, "wb");
                if (fw != NULL) {
                    if (roi == 0) {
                        fwrite(out_pym_buf.pym[layer/4].addr[0],
                            out_pym_buf.pym[layer/4].width * out_pym_buf.pym[layer/4].height, 1, fw);
                        fwrite(out_pym_buf.pym[layer/4].addr[1],
                            out_pym_buf.pym[layer/4].width * out_pym_buf.pym[layer/4].height / 2, 1, fw);
                    } else {
                        fwrite(out_pym_buf.pym_roi[layer/4][roi-1].addr[0],
                            out_pym_buf.pym_roi[layer/4][roi-1].width * out_pym_buf.pym_roi[layer/4][roi-1].height, 1, fw);
                        fwrite(out_pym_buf.pym_roi[layer/4][roi-1].addr[1],
                            out_pym_buf.pym_roi[layer/4][roi-1].width * out_pym_buf.pym_roi[layer/4][roi-1].height / 2, 1, fw);
                    }
                    fclose(fw);
                }
            }
            dumpnum++;
        }
		memset(&pstFrame, 0, sizeof(VIDEO_FRAME_S));
		pstFrame.stVFrame.pix_format = HB_PIXEL_FORMAT_NV12;
		pstFrame.stVFrame.width = out_pym_buf.pym[0].width;
		pstFrame.stVFrame.height = out_pym_buf.pym[0].height;
		pstFrame.stVFrame.size = out_pym_buf.pym[0].width *
		    out_pym_buf.pym[0].height * 3 / 2;
		pstFrame.stVFrame.phy_ptr[0] = out_pym_buf.pym[0].paddr[0];
		pstFrame.stVFrame.phy_ptr[1] = out_pym_buf.pym[0].paddr[1];
		pstFrame.stVFrame.vir_ptr[0] = out_pym_buf.pym[0].addr[0];
		pstFrame.stVFrame.vir_ptr[1] = out_pym_buf.pym[0].addr[1];
		pstFrame.stVFrame.pts = pts;

		ret = HB_VENC_SendFrame(0, &pstFrame, 3000);
		if (ret < 0) {
			printf("HB_VENC_SendFrame 0 error!!!ret %d \n", ret);
		}
		gettimeofday(&ts1, NULL);
		pts++;
		ret = HB_VPS_ReleaseChnFrame(1, 6, &out_pym_buf);
		if (ret) {
			printf("HB_VPS_ReleaseChnFrame error!!!\n");
			break;
		}
	}
	printf("pym_get_thread out===========\n");
	g_exit = 1;

	return;
}
void venc_from_pym_thread(void *vencpram)
{
	int ret;
    int idx = 0;
    hb_vio_buffer_t out_buf;
    int width = 1280, height = 720;
    int mmz_size = width * height * 3 / 2;
    int offset = width * height;
    int veChn = 0;
    int vpsGrp = 0;
    int vpsChn = 0;
    vencParam *vparam = (vencParam*)vencpram;
    FILE *outFile;
    char fname[32];
    int32_t pollFd;
    struct timeval select_timeout = {0};
    fd_set readFds;

    VIDEO_FRAME_S pstFrame;
    VIDEO_STREAM_S pstStream;
    memset(&pstFrame, 0, sizeof(VIDEO_FRAME_S));
    memset(&pstStream, 0, sizeof(VIDEO_STREAM_S));

    veChn = vparam->veChn;
    width = vparam->width;
    height = vparam->height;
    vpsGrp = vparam->vpsGrp;
    vpsChn = vparam->vpsChn;

    mmz_size = width * height * 3 / 2;

    pstFrame.stVFrame.width = width;
    pstFrame.stVFrame.height = height;
    pstFrame.stVFrame.size = mmz_size;
    pstFrame.stVFrame.pix_format = HB_PIXEL_FORMAT_NV12;

    if (g_save_flag == 1) {
        if (vparam->type ==1) {
            sprintf(fname, "./venc%d_stream.h264", veChn);
		} else if (vparam->type ==2) {
            sprintf(fname, "./venc%d_stream.h265", veChn);
		} else {
			sprintf(fname, "./venc%d_stream.jpg", veChn);
		}
		outFile = fopen(fname, "wb");
    }

	printf("venc_from_pym_thread in, %d,%d,%d,%d,%d\n",
        veChn, vpsGrp, vpsChn, width, height);

	while (g_exit == 0) {
        if (vparam->quit == 1) {
            break;
        }

        ret = HB_VENC_GetStream(veChn, &pstStream, 300);
        if (ret < 0) {
        } else {
            struct timeval ts0;
            gettimeofday(&ts0, NULL);
            if (g_save_flag == 1) {
                fwrite(pstStream.pstPack.vir_ptr,
                    pstStream.pstPack.size, 1, outFile);
            }

            HB_VENC_ReleaseStream(veChn, &pstStream);
        }
	}

    if (g_save_flag == 1) {
        fclose(outFile);
    }

	printf("venc_from_pym_thread exit\n");
    return;
}

void dual_venc_pthread_start(void)
{
	int ret;

	ret = pthread_create(&vens_thid[5], NULL, (void *)dual_pym_get_thread, NULL);
	if (vencparam[0].type > 0) {
           ret = pthread_create(&vens_thid[0], NULL,
                (void *)venc_from_pym_thread, &(vencparam[0]));
     }

	return;
}
static int dual_venc_prepare(vencParam *vparam)
{
	int ret;

	if (!vparam)
		return -1;

	ret = dual_venc_common_init();
	if (ret < 0) {
		printf("dual_venc_common_init error!\n");
		return ret;
	}

	printf("%s, veChn: %d, type: %d, width: %d, height: %d, bitrate :%d\n",
			__func__, vparam->veChn, vparam->type, vparam->width,
			vparam->height, vparam->bitrate);

	ret = dual_venc_init(vparam->veChn, vparam->type,
			       vparam->width, vparam->height,
			       vparam->bitrate);
	if (ret < 0) {
		printf("dual_venc_init error!\n");
		return ret;
	}

	ret = dual_venc_start(vparam->veChn);
	if (ret < 0) {
		printf("sample_start error!\n");
		dual_venc_common_deinit();
		return ret;
	}

	printf("dual_venc_prepare succeed\n");

	return ret;
}

int dual_singlepipe_venc_init()
{
	int ret, i, venctype;
	int vdecchn = 0;
	int vdectype = 96;

	venctype = g_venc_flag;
	vencparam[0].veChn = 0;	// ds0
	vencparam[0].type = venctype;
	vencparam[0].width = 1920;
	vencparam[0].height = 1080;
	vencparam[0].vpsGrp = 0;
	vencparam[0].vpsChn = 5;
	vencparam[0].bitrate = 2000;
	vencparam[0].quit = 0;

	if (dual_venc_prepare(&vencparam[0]) < 0) {
		printf("dual_venc_prepare failed\n");
		return -1;
	}

	printf("dual_singlepipe_venc_init end========\n");
	return 0;
}

int dual_singlepipe_venc_deinit()
{
	int ret;

	g_exit = 1;

	ret = pthread_join(vens_thid[5], NULL);
	if (vencparam[0].type > 0) ret = pthread_join(vens_thid[0], NULL);

	if (vencparam[0].type > 0) {
		dual_venc_stop(0);
		dual_venc_deinit(0);
	}
	dual_venc_common_deinit();
	return 0;
}
