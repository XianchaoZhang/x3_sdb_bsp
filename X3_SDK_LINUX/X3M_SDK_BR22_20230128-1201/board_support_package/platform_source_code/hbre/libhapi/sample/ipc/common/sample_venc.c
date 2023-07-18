/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "hb_comm_venc.h"
#include "hb_vdec.h"
#include "hb_venc.h"
#include "hb_vp_api.h"
#include "hb_vio_interface.h"
#include "sample.h"

int sample_venc_h264cbr(VENC_RC_ATTR_S *pstRcParam,
                                        VENC_H264_CBR_S *pstH264Cbr);

int sample_venc_h264vbr(VENC_RC_ATTR_S *pstRcParam,
                                        VENC_H264_VBR_S *pstH264Vbr);

int sample_venc_h264avbr(VENC_RC_ATTR_S *pstRcParam,
                                    VENC_H264_AVBR_S *pstH264AVbr);

int sample_venc_h264fixqp(VENC_RC_ATTR_S *pstRcParam,
                                    VENC_H264_FIXQP_S *pstH264FixQp);

int sample_venc_h264qpmap(VENC_RC_ATTR_S *pstRcParam,
                                    VENC_H264_QPMAP_S *pstH264QpMap);

int sample_venc_h265cbr(VENC_RC_ATTR_S *pstRcParam,
                                        VENC_H265_CBR_S *pstH265Cbr);

int sample_venc_h265vbr(VENC_RC_ATTR_S *pstRcParam,
                                        VENC_H265_VBR_S *pstH265Vbr);

int sample_venc_h265avbr(VENC_RC_ATTR_S *pstRcParam,
                                        VENC_H265_AVBR_S *pstH265AVbr);

int sample_venc_h265fixqp(VENC_RC_ATTR_S *pstRcParam,
                                        VENC_H265_FIXQP_S *pstH265FixQp);

int sample_venc_h265qpmap(VENC_RC_ATTR_S *pstRcParam,
                                        VENC_H265_QPMAP_S *pstH265QpMap);

int sample_venc_mjpgfixqp(VENC_RC_ATTR_S *pstRcParam,
                                    VENC_MJPEG_FIXQP_S *pstMjpegFixQp);

int sample_venc_setgop(VENC_GOP_ATTR_S *pstGopAttr, int presetidx,
                                int refreshtype, int qp, int intraqp);
int sample_venc_setRefParam(int Vechn, uint32_t longterm_pic_period,
                    uint32_t longterm_pic_using_period);

// #define VENC_BIND
int VencChnAttrInit(VENC_CHN_ATTR_S *pVencChnAttr, PAYLOAD_TYPE_E p_enType,
            int p_Width, int p_Height, PIXEL_FORMAT_E pixFmt) {
    int streambuf = 2*1024*1024;

    memset(pVencChnAttr, 0, sizeof(VENC_CHN_ATTR_S));
    pVencChnAttr->stVencAttr.enType = p_enType;

    pVencChnAttr->stVencAttr.u32PicWidth = p_Width;
    pVencChnAttr->stVencAttr.u32PicHeight = p_Height;

    // pVencChnAttr->stVencAttr.u32InputFrameRate = 30;
    // pVencChnAttr->stVencAttr.u32OutputFrameRate = 30;

    pVencChnAttr->stVencAttr.enMirrorFlip = DIRECTION_NONE;
    pVencChnAttr->stVencAttr.enRotation = CODEC_ROTATION_0;
    pVencChnAttr->stVencAttr.stCropCfg.bEnable = HB_FALSE;
    pVencChnAttr->stVencAttr.bEnableUserPts = HB_TRUE;
    pVencChnAttr->stVencAttr.s32BufJoint = 0;
    pVencChnAttr->stVencAttr.s32BufJointSize = 8000000;

    if (p_Width * p_Height > 2688 * 1522) {
        pVencChnAttr->stVencAttr.vlc_buf_size = 7900*1024;
    } else if (p_Width * p_Height > 1920 * 1080) {
        pVencChnAttr->stVencAttr.vlc_buf_size = 4*1024*1024;
    } else if (p_Width * p_Height > 1280 * 720) {
        pVencChnAttr->stVencAttr.vlc_buf_size = 2100*1024;
    } else if (p_Width * p_Height > 704 * 576) {
        pVencChnAttr->stVencAttr.vlc_buf_size = 2100*1024;
    } else {
        pVencChnAttr->stVencAttr.vlc_buf_size = 2048*1024;
    }
    streambuf = (p_Width * p_Height)&0xfffff000;
    // pVencChnAttr->stVencAttr.vlc_buf_size = 0;
    //     ((((p_Width * p_Height) * 9) >> 3) * 3) >> 1;
    if (p_enType == PT_JPEG || p_enType == PT_MJPEG) {
        // streambuf = (p_Width * p_Height * 3/2)&0xfffff000;
        pVencChnAttr->stVencAttr.enPixelFormat = pixFmt;
        pVencChnAttr->stVencAttr.u32BitStreamBufferCount = 1;
        pVencChnAttr->stVencAttr.u32FrameBufferCount = 2;
        pVencChnAttr->stVencAttr.bExternalFreamBuffer = HB_TRUE;
        pVencChnAttr->stVencAttr.stAttrJpeg.dcf_enable = HB_FALSE;
        pVencChnAttr->stVencAttr.stAttrJpeg.quality_factor = 30;
        pVencChnAttr->stVencAttr.stAttrJpeg.restart_interval = 0;
        pVencChnAttr->stVencAttr.u32BitStreamBufSize = streambuf;
        pVencChnAttr->stVencAttr.stCropCfg.bEnable = HB_FALSE;
        pVencChnAttr->stVencAttr.stCropCfg.stRect.s32X = 1920;
        pVencChnAttr->stVencAttr.stCropCfg.stRect.s32Y = 1080;
        pVencChnAttr->stVencAttr.stCropCfg.stRect.u32Width = 1920;
        pVencChnAttr->stVencAttr.stCropCfg.stRect.u32Height = 1080;
    } else {
        pVencChnAttr->stVencAttr.enPixelFormat = pixFmt;
        pVencChnAttr->stVencAttr.u32BitStreamBufferCount = 4;
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
        pVencChnAttr->stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
        // pVencChnAttr->stRcAttr.stH264Vbr.bQpMapEnable = HB_TRUE;
        // pVencChnAttr->stRcAttr.stH264Vbr.u32IntraQp = 20;
        // pVencChnAttr->stRcAttr.stH264Vbr.u32IntraPeriod = 60;
        // pVencChnAttr->stRcAttr.stH264Vbr.u32FrameRate = 30;
        pVencChnAttr->stVencAttr.stAttrH264.h264_profile = 0;
        pVencChnAttr->stVencAttr.stAttrH264.h264_level = 0;
    }

    pVencChnAttr->stGopAttr.u32GopPresetIdx = 6;
    pVencChnAttr->stGopAttr.s32DecodingRefreshType = 2;

    return 0;
}

int compare = 0;
int cmp_param[6] = {0};
uint32_t y_cnt = 0;
int cmp_dump_cnt = 0;
uint32_t uv_cnt = 0;
int cmp_step = 0;

int32_t compare_frames(address_info_t *a, address_info_t *b,
						uint32_t w_step, uint32_t h_step,
						uint32_t y_td, uint32_t uv_td,
						uint32_t *y_cnt, uint32_t *uv_cnt)
{
	uint32_t y_diff_cnt = 0;
	uint32_t uv_diff_cnt = 0;
	uint16_t i = 0, j = 0;
	if (a == NULL || b == NULL) {
		return -1;
	}
	if (w_step == 0 || h_step ==0) {
		return -1;
	}
	char *_a = a->addr[0];
	char *_b = b->addr[0];
	/* compare y value */
	if (y_td != -1) {
		for(i = 0; i < a->height; i+=h_step) {
			for(j = 0; j < a->width; j+=w_step) {
				if (abs(*_a - *_b) > y_td) {
					y_diff_cnt++;
				}
				_a+=w_step;
				_b+=w_step;
			}
			_a = a->addr[0] + a->stride_size;
			_b = b->addr[0] + b->stride_size;
		}
	}
	/* compare uv value */
	_a = a->addr[1];
	_b = b->addr[1];
	if (uv_td != -1) {
		for(i = 0; i < a->height/2; i+=h_step) {
			for(j = 0; j < a->width; j+=w_step) {
				if (abs(*_a - *_b) > uv_td) {
					uv_diff_cnt++;
				}
				_a+=w_step;
				_b+=w_step;
			}
			_a = a->addr[1] + a->stride_size;
			_b = b->addr[1] + b->stride_size;
		}
		printf("compare uv\n");
	}
	*y_cnt = y_diff_cnt;
	*uv_cnt = uv_diff_cnt;
	return 0;
}

void venc_thread(void *vencpram)
{
	int ret;
    hb_vio_buffer_t out_buf;
    int width = 1280, height = 720;
    int mmz_size = width * height * 3 / 2;
    int veChn = 0;
    int vpsGrp = 0;
    int vpsChn = 0;
    vencParam *vparam = (vencParam*)vencpram;
    FILE *outFile;
    char fname[32];
    struct timeval ts0, ts1;
    int pts = 0;
    int dumpnum = 10;
    int errnum = 0;
    int idx = 0;
    time_t start, last = time(NULL);
    static hb_vio_buffer_t prev_buf = {0};

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
        } else if (vparam->type ==3) {
            sprintf(fname, "./venc%d_stream.mjpeg", veChn);
		} else {
			sprintf(fname, "./venc%d_%d.jpg", veChn, pts);
		}
		outFile = fopen(fname, "wb");
    }

	printf("venc_thread in, %d,%d,%d,%d,%d\n",
                            veChn, vpsGrp, vpsChn, width, height);

    // ret = HB_VENC_GetFd(veChn, &pollFd);
    // if (ret < 0) {
    //     printf("venc %d HB_VENC_GetFd failed\n", veChn);
    //     g_exit = 1;
    //     return;
    // }

	while (g_exit == 0) {
        if (vparam->quit == 1) {
            break;
        }
        if (g_bindflag == 0) {
	    /* coverity[overrun-buffer-val] */
            ret = HB_VPS_GetChnFrame(vpsGrp, vpsChn, &out_buf, 2000);
            if (ret != 0) {
                printf("HB_VPS_GetChnFrame error %d!!!\n", errnum);
                errnum++;
                // if (errnum > 30) {
                //     printf("HB_VPS_GetChnFrame error\n");
                //     g_exit = 1;
                //     break;
                // }
                continue;
            }
            gettimeofday(&ts0, NULL);
            errnum = 0;
            // printf("chn:%d, frameid: %d\n",
            //          vpsChn, out_buf.img_info.frame_id);

            // normal_buf_info_print(&out_buf);
            if (access("./vpsdump", F_OK) == 0) {
                system("rm ./vpsdump");
                dumpnum = 0;
            }
            if (dumpnum < 3) {
                char fname[32];
                sprintf(fname, "vps_%d_%d_%d.yuv", vpsGrp, vpsChn, dumpnum);
                printf("start dump %s\n", fname);
                FILE *fw = fopen(fname, "wb");
                if (fw != NULL) {
                    fwrite(out_buf.img_addr.addr[0], width * height, 1, fw);
                    fwrite(out_buf.img_addr.addr[1], width * height / 2, 1, fw);
                    fclose(fw);
                }
                dumpnum++;
            }

            pstFrame.stVFrame.phy_ptr[0] = out_buf.img_addr.paddr[0];
            pstFrame.stVFrame.phy_ptr[1] = out_buf.img_addr.paddr[1];

            pstFrame.stVFrame.vir_ptr[0] = out_buf.img_addr.addr[0];
            pstFrame.stVFrame.vir_ptr[1] = out_buf.img_addr.addr[1];
            pstFrame.stVFrame.pts = out_buf.img_info.frame_id;
            // if (veChn < 1)
            // printf("chn%d: %5d.%06d ms, pts:%d\n", veChn, ts0.tv_sec, ts0.tv_usec, pts);
            // if (veChn == 2) {
            //     printf("[%5d.%06d] vps: %5d.%06d ms, pts:%llu, %llu\n", ts0.tv_sec,
            //             ts0.tv_usec, ts0.tv_sec, ts0.tv_usec, pstFrame.stVFrame.pts, pstFrame.stVFrame.phy_ptr[0]);
            // }
            ret = HB_VENC_SendFrame(veChn, &pstFrame, 3000);
            if (ret < 0) {
                printf("HB_VENC_SendFrame error!!!\n");
                break;
            }
        }

        // select_timeout.tv_sec = 0;
        // select_timeout.tv_usec = 100 * 1000;
        // FD_ZERO(&readFds);
        // FD_SET(pollFd, &readFds);
        // ret = select(pollFd+1, &readFds, NULL, NULL, &select_timeout);
        // if (ret < 0) {
        //     printf("Failed to select fd = %d.\n", pollFd);
        //     break;
        // } else if (ret == 0) {
        //     printf("Time out to select fd = %d.\n", pollFd);
        //     continue;
        // }
        // if (FD_ISSET(pollFd, &readFds)) {
            
            ret = HB_VENC_GetStream(veChn, &pstStream, 1000);
            gettimeofday(&ts1, NULL);
            // printf("\n[%d.%06d][%d.%06d] venc%d %d %d use time: %d\n",
            // ts0.tv_sec, ts0.tv_usec, ts1.tv_sec, ts1.tv_usec, veChn, pts,
            // pts*3000, (ts1.tv_sec-ts0.tv_sec)*1000000 + ts1.tv_usec-ts0.tv_usec);
            if (ret < 0) {
                printf("HB_VENC_GetStream %d error!!!\n", veChn);
                // usleep(50000);
                // errnum++;
                // if (errnum > 3) {
                //     printf("HB_VENC_GetStream error\n");
                //     g_exit = 1;
                // }
                // break;
            } else {
                // if ((veChn == 2)) {
                // int ssize = pstStream.pstPack.size;
                // printf("-- %6d %02x %02x %d %d\n", pstStream.pstPack.size,
                // pstStream.pstPack.vir_ptr[3], pstStream.pstPack.vir_ptr[4],
                // pstStream.stStreamInfo.temporal_id,
                // pstStream.stStreamInfo.longterm_ref_type);
                // printf("pts: %llu\n", pstStream.pstPack.pts);
                // }
                // if (veChn == 2) {
                //     printf("[%5d.%06d] chn: %5d.%06d ms, pts:%llu\n", ts1.tv_sec,
                //          ts1.tv_usec, ts1.tv_sec, ts1.tv_usec, pstStream.pstPack.pts);
                // }
                start = time(NULL);
                idx++;
                if (start > last) {
                    gettimeofday(&ts1, NULL);
                    printf("[%ld.%06ld]venc chn %d fps %d\n",
                                ts1.tv_sec, ts1.tv_usec, veChn, idx);
                    last = start;
                    idx = 0;
                }
                errnum = 0;
                HB_VENC_ReleaseStream(veChn, &pstStream);
                if (g_save_flag == 1) {
                    if (outFile > 0)
                    fwrite(pstStream.pstPack.vir_ptr,
                        pstStream.pstPack.size, 1, outFile);
                    if (vparam->type == 4) {
                        fclose(outFile);
                        sprintf(fname, "./venc%d_%d.jpg", veChn, pts);
		                outFile = fopen(fname, "wb");
                    }
                }
            }
        // }
        // usleep(100000);

        if (g_bindflag == 0) {
            if (compare && (vpsChn == 0)) {
                /* step 0 save prev buf */
                if (cmp_step == 0) {
					memcpy(&prev_buf, &out_buf, sizeof(hb_vio_buffer_t));
					cmp_step = 1;
				} else {/* step 1 compare out_buf with saved prev buf */
                    compare_frames(&prev_buf.img_addr, &out_buf.img_addr,
								cmp_param[0], cmp_param[1], cmp_param[2], cmp_param[3],
								&y_cnt, &uv_cnt);
                    if ((y_cnt > cmp_param[4]) || (uv_cnt > cmp_param[5])) {
                        printf("ydiff cnt %d uv diff cnt %d\n", y_cnt, uv_cnt);
                        char fname[32];
                        int size = width*height;
                        snprintf(fname, sizeof(fname),
                            "/userdata/vps_%d_%d_fid%d_%dx%d.yuv",
                            vpsGrp, vpsChn, out_buf.img_info.frame_id,
                            width, height);
                        printf("start dump %s\n", fname);
                        FILE *fw = fopen(fname, "wb");
                        if (fw != NULL) {
                            fwrite(out_buf.img_addr.addr[0], size, 1, fw);
                            fwrite(out_buf.img_addr.addr[1], size/2, 1, fw);
                            fclose(fw);
                        }
                        char fname1[128];
                        snprintf(fname1, sizeof(fname1),
                            "/userdata/prev_vps_%d_%d_fid%d_%dx%d.yuv",
                            vpsGrp, vpsChn, prev_buf.img_info.frame_id,
                            width, height);
                        printf("start dump %s\n", fname1);
                        FILE *fw1 = fopen(fname1, "wb");
                        if (fw1 != NULL) {
                            fwrite(prev_buf.img_addr.addr[0], size, 1, fw1);
                            fwrite(prev_buf.img_addr.addr[1], size/2, 1, fw1);
                            fclose(fw1);
                        }
                        cmp_dump_cnt++;
                        if (cmp_dump_cnt > 500) {
                            printf("cmp_dump_cnt > 500 exit\n");
                            exit(1);
                        }
                    }
                    /* step 1 release prev_buf */
                    ret = HB_VPS_ReleaseChnFrame(vpsGrp, vpsChn, &prev_buf);
                    if (ret) {
                        printf("HB_VPS_ReleaseChnFrame error!!!\n");
                    }
                    cmp_step = 2;
                }
                /* step 2 release out_buf */
                if (cmp_step == 2) {
                    cmp_step = 0;
                    ret = HB_VPS_ReleaseChnFrame(vpsGrp, vpsChn, &out_buf);
                    if (ret) {
                        printf("HB_VPS_ReleaseChnFrame error!!!\n");
                    }
                }
            } else {
                ret = HB_VPS_ReleaseChnFrame(vpsGrp, vpsChn, &out_buf);
                if (ret) {
                    printf("HB_VPS_ReleaseChnFrame error!!!\n");
                }
            }
        }
        pts++;
	}

    if (g_save_flag == 1) {
        fclose(outFile);
    }

    // ret = HB_VENC_CloseFd(veChn, pollFd);
	printf("venc chn %d thread exit\n", veChn);
}

int sample_venc_setjpegmode(int vechn, HB_VENC_JPEG_ENCODE_MODE_E mode)
{
    return HB_VENC_SetJpegEncodeMode(vechn, mode);
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

int sample_venc_setrcattr(int VeChn)
{
    VENC_RC_ATTR_S stRcParam;
    int s32Ret;

    stRcParam.enRcMode = VENC_RC_MODE_H264AVBR;
    s32Ret = HB_VENC_GetRcParam(VeChn, &stRcParam);
    if (s32Ret != 0) {
        printf("HB_VENC_GetRcParam failed.\n");
        return -1;
    }
    printf("stRcParam.enRcMode: %d\n", stRcParam.enRcMode);
    // stRcParam.stH264Cbr.u32BitRate = 2000;
    // stRcParam.stH264Cbr.u32IntraPeriod = 60;
    // stRcParam.stH264Cbr.u32FrameRate = 25;
    stRcParam.stH264AVbr.u32BitRate = 8192;
    stRcParam.stH264AVbr.u32FrameRate = 25;
    stRcParam.stH264AVbr.u32IntraQp = 30;
    stRcParam.stH264AVbr.u32InitialRcQp = 45;
    stRcParam.stH264AVbr.u32IntraPeriod = 50;
    stRcParam.stH264AVbr.u32VbvBufferSize = 3000;
    stRcParam.stH264AVbr.bMbLevelRcEnable = HB_FALSE;
    stRcParam.stH264AVbr.u32MaxIQp = 51;
    stRcParam.stH264AVbr.u32MinIQp = 28;
    stRcParam.stH264AVbr.u32MaxPQp = 51;
    stRcParam.stH264AVbr.u32MinPQp = 28;
    stRcParam.stH264AVbr.u32MaxBQp = 51;
    stRcParam.stH264AVbr.u32MinBQp = 28;
    stRcParam.stH264AVbr.bHvsQpEnable = HB_TRUE;
    stRcParam.stH264AVbr.s32HvsQpScale = 2;
    stRcParam.stH264AVbr.u32MaxDeltaQp = 3;
    stRcParam.stH264AVbr.bQpMapEnable = HB_FALSE;

    s32Ret = HB_VENC_SetRcParam(VeChn, &stRcParam);
    if (s32Ret != 0) {
        printf("HB_VENC_SetRcParam failed.\n");
        return -1;
    }

    return 0;
}

int sample_venc_initattr(int VeChn, int type, int width, int height, int bits,
                VENC_RC_MODE_E rcmode, VENC_CHN_ATTR_S *vencChnAttr)
{
    int s32Ret;
    // VENC_CHN_ATTR_S vencChnAttr;
    VENC_RC_ATTR_S *pstRcParam;
    PAYLOAD_TYPE_E ptype;

    if (type == 1) {
        ptype = PT_H264;
    } else if (type == 2) {
        ptype = PT_H265;
    } else if (type == 3) {
        ptype = PT_MJPEG;
    } else {
        ptype = PT_JPEG;
    }

    VencChnAttrInit(vencChnAttr, ptype, width, height, HB_PIXEL_FORMAT_NV12);
    // if (VeChn == 0) {
    //     vencChnAttr->stVencAttr.stCropCfg.bEnable = HB_TRUE;
    // }

    s32Ret = HB_VENC_CreateChn(VeChn, vencChnAttr);
    if (s32Ret != 0) {
        printf("HB_VENC_CreateChn %d failed, %d.\n", VeChn, s32Ret);
        return -1;
    }

    // stModParam.u32OneStreamBuffer = 0;
    // s32Ret = HB_VENC_SetModParam(VeChn, &stModParam);
    // if (s32Ret != 0) {
    //     printf("HB_VENC_SetModParam %d failed, %d.\n", VeChn, s32Ret);
    //     return -1;
    // }

    // HB_VENC_Module_Uninit();
    vencChnAttr->stRcAttr.enRcMode = rcmode;
    if (ptype == PT_H264) {
        pstRcParam = &(vencChnAttr->stRcAttr);
        s32Ret = HB_VENC_GetRcParam(VeChn, pstRcParam);
        if (s32Ret != 0) {
            printf("HB_VENC_GetRcParam failed.\n");
            return -1;
        }
        switch (vencChnAttr->stRcAttr.enRcMode) {
        case VENC_RC_MODE_H264CBR:
        {
            VENC_H264_CBR_S stH264Cbr;
            memset(&stH264Cbr, 0, sizeof(stH264Cbr));
            stH264Cbr.u32BitRate = bits;
            stH264Cbr.u32FrameRate = 30;
            stH264Cbr.u32IntraPeriod = 30;
            stH264Cbr.u32VbvBufferSize = 3000;
            stH264Cbr.u32IntraQp = g_intra_qp;
            stH264Cbr.u32InitialRcQp = g_intra_qp;
            stH264Cbr.bMbLevelRcEnable = HB_FALSE;
            stH264Cbr.u32MaxIQp = 51;
            stH264Cbr.u32MinIQp = 10;
            stH264Cbr.u32MaxPQp = 51;
            stH264Cbr.u32MinPQp = 10;
            stH264Cbr.u32MaxBQp = 51;
            stH264Cbr.u32MinBQp = 10;
            stH264Cbr.bHvsQpEnable = HB_FALSE;
            stH264Cbr.s32HvsQpScale = 2;
            stH264Cbr.u32MaxDeltaQp = 3;
            stH264Cbr.bQpMapEnable = HB_FALSE;
            sample_venc_h264cbr(pstRcParam, &stH264Cbr);
            break;
        }
        case VENC_RC_MODE_H264VBR:
        {
            VENC_H264_VBR_S stH264Vbr;
            memset(&stH264Vbr, 0, sizeof(stH264Vbr));
            stH264Vbr.bQpMapEnable = HB_FALSE;
            stH264Vbr.u32FrameRate = 30;
            stH264Vbr.u32IntraPeriod = 30;
            stH264Vbr.u32IntraQp = 35;
            sample_venc_h264vbr(pstRcParam, &stH264Vbr);
            break;
        }
        case VENC_RC_MODE_H264AVBR:
        {
            VENC_H264_AVBR_S stH264Avbr;
            memset(&stH264Avbr, 0, sizeof(stH264Avbr));
            stH264Avbr.u32BitRate = bits;
            stH264Avbr.u32FrameRate = 25;
            stH264Avbr.u32IntraPeriod = 50;
            stH264Avbr.u32VbvBufferSize = 3000;
            stH264Avbr.u32IntraQp = 30;
            stH264Avbr.u32InitialRcQp = 45;
            stH264Avbr.bMbLevelRcEnable = HB_FALSE;
            stH264Avbr.u32MaxIQp = 51;
            stH264Avbr.u32MinIQp = 28;
            stH264Avbr.u32MaxPQp = 51;
            stH264Avbr.u32MinPQp = 28;
            stH264Avbr.u32MaxBQp = 51;
            stH264Avbr.u32MinBQp = 28;
            stH264Avbr.bHvsQpEnable = HB_TRUE;
            stH264Avbr.s32HvsQpScale = 2;
            stH264Avbr.u32MaxDeltaQp = 3;
            stH264Avbr.bQpMapEnable = HB_FALSE;
            sample_venc_h264avbr(pstRcParam, &stH264Avbr);
            break;
        }
        case VENC_RC_MODE_H264FIXQP:
        {
            VENC_H264_FIXQP_S stH264FixQp;
            memset(&stH264FixQp, 0, sizeof(stH264FixQp));
            stH264FixQp.u32FrameRate = 30;
            stH264FixQp.u32IntraPeriod = 30;
            stH264FixQp.u32IQp = 35;
            stH264FixQp.u32PQp = 35;
            stH264FixQp.u32BQp = 35;
            sample_venc_h264fixqp(pstRcParam, &stH264FixQp);
            break;
        }
        case VENC_RC_MODE_H264QPMAP:
        {
            VENC_H264_QPMAP_S stH264QpMap;
            memset(&stH264QpMap, 0, sizeof(stH264QpMap));
            stH264QpMap.u32FrameRate = 30;
            stH264QpMap.u32IntraPeriod = 30;
            stH264QpMap.u32QpMapArrayCount =
                ((width+0x0f)&~0x0f)/16 * ((height+0x0f)&~0x0f)/16;
            stH264QpMap.u32QpMapArray = malloc(stH264QpMap.u32QpMapArrayCount);
            for(int i = 0; i < stH264QpMap.u32QpMapArrayCount; i++) {
                stH264QpMap.u32QpMapArray[i] = 30;
            }
            sample_venc_h264qpmap(pstRcParam, &stH264QpMap);
            printf("sample_venc_h264qpmap 30\n");
            break;
        }
        default:
            break;
        }
        printf(" vencChnAttr->stRcAttr.enRcMode = %d\n",
                vencChnAttr->stRcAttr.enRcMode);
    } else if (ptype == PT_H265) {
        pstRcParam = &(vencChnAttr->stRcAttr);
        s32Ret = HB_VENC_GetRcParam(VeChn, pstRcParam);
        if (s32Ret != 0) {
            printf("HB_VENC_GetRcParam failed.\n");
            return -1;
        }
        switch (vencChnAttr->stRcAttr.enRcMode) {
        case VENC_RC_MODE_H265CBR:
        {
            VENC_H265_CBR_S stH265Cbr;
            memset(&stH265Cbr, 0, sizeof(stH265Cbr));
            stH265Cbr.u32IntraPeriod = 30;
            stH265Cbr.u32IntraQp = 30;
            stH265Cbr.u32BitRate = bits;
            stH265Cbr.u32FrameRate = 30;
            stH265Cbr.u32InitialRcQp = 45;
            stH265Cbr.u32VbvBufferSize = 3000;
            stH265Cbr.bCtuLevelRcEnable = HB_FALSE;
            stH265Cbr.u32MaxIQp = 45;
            stH265Cbr.u32MinIQp = 22;
            stH265Cbr.u32MaxPQp = 45;
            stH265Cbr.u32MinPQp = 22;
            stH265Cbr.u32MaxBQp = 45;
            stH265Cbr.u32MinBQp = 22;
            stH265Cbr.bHvsQpEnable = HB_FALSE;
            stH265Cbr.s32HvsQpScale = 2;
            stH265Cbr.u32MaxDeltaQp = 10;
            stH265Cbr.bQpMapEnable = HB_FALSE;
            sample_venc_h265cbr(pstRcParam, &stH265Cbr);
            break;
        }
        case VENC_RC_MODE_H265VBR:
        {
            VENC_H265_VBR_S stH265Vbr;
            memset(&stH265Vbr, 0, sizeof(stH265Vbr));
            stH265Vbr.u32IntraPeriod = 30;
            stH265Vbr.u32FrameRate = 30;
            stH265Vbr.bQpMapEnable = HB_FALSE;
            stH265Vbr.u32IntraQp = 32;
            sample_venc_h265vbr(pstRcParam, &stH265Vbr);
            break;
        }
        case VENC_RC_MODE_H265AVBR:
        {
            VENC_H265_AVBR_S stH265Avbr;
            memset(&stH265Avbr, 0, sizeof(stH265Avbr));
            stH265Avbr.u32IntraPeriod = 30;
            stH265Avbr.u32IntraQp = 30;
            stH265Avbr.u32BitRate = bits;
            stH265Avbr.u32FrameRate = 30;
            stH265Avbr.u32InitialRcQp = 45;
            stH265Avbr.u32VbvBufferSize = 3000;
            stH265Avbr.bCtuLevelRcEnable = HB_FALSE;
            stH265Avbr.u32MaxIQp = 45;
            stH265Avbr.u32MinIQp = 22;
            stH265Avbr.u32MaxPQp = 45;
            stH265Avbr.u32MinPQp = 22;
            stH265Avbr.u32MaxBQp = 45;
            stH265Avbr.u32MinBQp = 22;
            stH265Avbr.bHvsQpEnable = HB_TRUE;
            stH265Avbr.s32HvsQpScale = 2;
            stH265Avbr.u32MaxDeltaQp = 10;
            stH265Avbr.bQpMapEnable = HB_FALSE;
            sample_venc_h265avbr(pstRcParam, &stH265Avbr);
            break;
        }
        case VENC_RC_MODE_H265FIXQP:
        {
            VENC_H265_FIXQP_S stH265FixQp;
            memset(&stH265FixQp, 0, sizeof(stH265FixQp));
            stH265FixQp.u32FrameRate = 30;
            stH265FixQp.u32IntraPeriod = 30;
            stH265FixQp.u32IQp = 35;
            stH265FixQp.u32PQp = 35;
            stH265FixQp.u32BQp = 35;
            sample_venc_h265fixqp(pstRcParam, &stH265FixQp);
            break;
        }
        case VENC_RC_MODE_H265QPMAP:
        {
            VENC_H265_QPMAP_S stH265QpMap;
            memset(&stH265QpMap, 0, sizeof(stH265QpMap));
            stH265QpMap.u32FrameRate = 30;
            stH265QpMap.u32IntraPeriod = 30;
            stH265QpMap.u32QpMapArrayCount =
                ((width+0x0f)&~0x0f)/16 * ((height+0x0f)&~0x0f)/16;;
            stH265QpMap.u32QpMapArray = malloc(stH265QpMap.u32QpMapArrayCount);
            sample_venc_h265qpmap(pstRcParam, &stH265QpMap);
            break;
        }
        default:
            break;
        }
        printf(" m_VencChnAttr->stRcAttr.enRcMode = %d\n",
                vencChnAttr->stRcAttr.enRcMode);
    } else if (ptype == PT_MJPEG) {
        VENC_MJPEG_FIXQP_S stMjpegFixQp;
        pstRcParam = &(vencChnAttr->stRcAttr);
        vencChnAttr->stRcAttr.enRcMode = VENC_RC_MODE_MJPEGFIXQP;
        s32Ret = HB_VENC_GetRcParam(VeChn, pstRcParam);
        if (s32Ret != 0) {
            printf("HB_VENC_GetRcParam failed.\n");
            return -1;
        }
        memset(&stMjpegFixQp, 0,  sizeof(stMjpegFixQp));
        stMjpegFixQp.u32FrameRate = 30;
        stMjpegFixQp.u32QualityFactort = 50;
        sample_venc_mjpgfixqp(pstRcParam, &stMjpegFixQp);
    }

    return 0;
}

int sample_venc_init(int VeChn, int type, int width, int height, int bits)
{
    int s32Ret;
    VENC_CHN_ATTR_S vencChnAttr;
    VENC_RC_MODE_E rcmode = VENC_RC_MODE_H265AVBR;

    if (type == 1) rcmode = VENC_RC_MODE_H264QPMAP;
    else if (type == 2) rcmode = VENC_RC_MODE_H265AVBR;
    s32Ret = sample_venc_initattr(VeChn, type, width, height, bits,
                    rcmode, &vencChnAttr);
    if (s32Ret != 0) {
        printf("sample_venc_initattr failed\n");
        return -1;
    }

    // setup user gop
    printf("sample_venc_setgop: interqp: %d, intarqp: %d\n",
     g_inter_qp, g_intra_qp);
    // sample_venc_setgop(&vencChnAttr.stGopAttr, 10, 2, g_inter_qp, g_intra_qp);
    // setup refparam
    // sample_venc_setRefParam(VeChn, 4, 2);
    // s32Ret = sample_setroi(VeChn, width, height);
    // if (s32Ret != 0) {
    //     printf("sample_venc_setroi failed\n");
    //     return -1;
    // }

    s32Ret = HB_VENC_SetChnAttr(VeChn, &vencChnAttr);  // config
    if (s32Ret != 0) {
        printf("HB_VENC_SetChnAttr failed\n");
        return -1;
    }

    return 0;
}

int sample_setroi(int VeChn, int width, int height)
{
    VENC_ROI_ATTR_S pstRoiAttr;
    int stroi_map_len;
    int s32Ret;

    stroi_map_len = ((width+15) >> 4) * ((height+15) >> 4);

    memset(&pstRoiAttr, 0, sizeof(VENC_ROI_ATTR_S));
    pstRoiAttr.roi_enable = HB_TRUE;
    pstRoiAttr.roi_map_array_count = stroi_map_len;
    pstRoiAttr.roi_map_array =
        (unsigned char*)malloc(stroi_map_len * sizeof(unsigned char));
    for (int i = 0; i < stroi_map_len; i++) {
        pstRoiAttr.roi_map_array[i] = 51;
    }
    s32Ret = HB_VENC_SetRoiAttr(VeChn, &pstRoiAttr);
    if (s32Ret != 0) {
        printf("HB_VENC_SetRoiAttr %d failed, %dx%d\n", VeChn, width, height);
        return -1;
    }
    return s32Ret;
}

int sample_venc_setroi(int VeChn, int type, int width, int height, int bits)
{
    int s32Ret;
    VENC_CHN_ATTR_S vencChnAttr;

    s32Ret = HB_VENC_StopRecvFrame(VeChn);
    if (s32Ret != 0) {
        printf("HB_VENC_StopRecvFrame %d failed\n", VeChn);
        return -1;
    }

    s32Ret = HB_VENC_DestroyChn(VeChn);
    if (s32Ret != 0) {
        printf("HB_VENC_DestroyChn %d failed\n", VeChn);
        return -1;
    }

    s32Ret = sample_venc_initattr(VeChn, type, width, height,
                    bits, VENC_RC_MODE_H264AVBR, &vencChnAttr);
    if (s32Ret != 0) {
        printf("sample_venc_initattr failed\n");
        return -1;
    }

    // setup user gop
    // sample_venc_setgop(&vencChnAttr.stGopAttr, 10, 2);
    // setup refparam
    // sample_venc_setRefParam(VeChn, 4, 2);
    s32Ret = sample_setroi(VeChn, width, height);
    if (s32Ret != 0) {
        printf("sample_venc_setroi failed\n");
        return -1;
    }

    s32Ret = HB_VENC_SetChnAttr(VeChn, &vencChnAttr);  // config
    if (s32Ret != 0) {
        printf("HB_VENC_SetChnAttr failed\n");
        return -1;
    }

    sample_venc_start(VeChn);

    return 0;
}

int sample_venc_seth264_profile(int VeChn, int type, int width, int height,
        int bits, int profile, int level)
{
    int s32Ret;
    VENC_CHN_ATTR_S vencChnAttr;

    s32Ret = HB_VENC_StopRecvFrame(VeChn);
    if (s32Ret != 0) {
        printf("HB_VENC_StopRecvFrame %d failed\n", VeChn);
        return -1;
    }

    s32Ret = HB_VENC_DestroyChn(VeChn);
    if (s32Ret != 0) {
        printf("HB_VENC_DestroyChn %d failed\n", VeChn);
        return -1;
    }

    s32Ret = sample_venc_initattr(VeChn, type, width, height,
                            bits, VENC_RC_MODE_H264CBR, &vencChnAttr);
    if (s32Ret != 0) {
        printf("sample_venc_initattr failed\n");
        return -1;
    }

    vencChnAttr.stVencAttr.stAttrH264.h264_profile = profile;
    vencChnAttr.stVencAttr.stAttrH264.h264_level = level;

    s32Ret = HB_VENC_SetChnAttr(VeChn, &vencChnAttr);  // config
    if (s32Ret != 0) {
        printf("HB_VENC_SetChnAttr failed\n");
        return -1;
    }

    sample_venc_start(VeChn);

    return 0;
}

int sample_venc_set_rcmodes(int VeChn)
{
    int s32Ret;
    VENC_RC_ATTR_S stRcParam;
    stRcParam.enRcMode = VENC_RC_MODE_H264CBR;
    HB_VENC_GetRcParam(VeChn, &stRcParam);
    if (stRcParam.enRcMode == VENC_RC_MODE_H265CBR) {    
        stRcParam.stH265Cbr.u32IntraPeriod = 15;
        printf("xxxxxxxxxh265xxxxxxxxx\n");
    } else if (stRcParam.enRcMode == VENC_RC_MODE_H264CBR) {  
        stRcParam.stH264Cbr.u32IntraPeriod = 15;
        printf("xxxxxxxxxh264xxxxxxxxx\n");
    }
    s32Ret = HB_VENC_SetRcParam(VeChn, &stRcParam);

    return s32Ret;
}

int sample_venc_set_rcmode(int VeChn, int type, int width,
                int height, int bits, VENC_RC_MODE_E rcmode)
{
    int s32Ret;
    VENC_CHN_ATTR_S vencChnAttr;

    s32Ret = HB_VENC_StopRecvFrame(VeChn);
    if (s32Ret != 0) {
        printf("HB_VENC_StopRecvFrame %d failed\n", VeChn);
        return -1;
    }

    s32Ret = HB_VENC_DestroyChn(VeChn);
    if (s32Ret != 0) {
        printf("HB_VENC_DestroyChn %d failed\n", VeChn);
        return -1;
    }

    s32Ret = sample_venc_initattr(VeChn, type, width, height,
                            bits, rcmode, &vencChnAttr);
    if (s32Ret != 0) {
        printf("sample_venc_initattr failed\n");
        return -1;
    }

    s32Ret = HB_VENC_SetChnAttr(VeChn, &vencChnAttr);  // config
    if (s32Ret != 0) {
        printf("HB_VENC_SetChnAttr failed\n");
        return -1;
    }

    sample_venc_start(VeChn);

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

    // s32Ret = HB_VENC_EnableIDR(VeChn, HB_FALSE);
    // printf("HB_VENC_EnableIDR: %x\n", s32Ret);
    // s32Ret = HB_VENC_EnableIDR(VeChn, HB_TRUE);
    // printf("HB_VENC_EnableIDR %x\n", s32Ret);
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

// int sample_reset(int VeChn, )

int sample_set_vencfps(int VeChn, int InputFps, int OutputFps)
{
    VENC_CHN_PARAM_S chnparam;

    HB_VENC_GetChnParam(VeChn, &chnparam);
    chnparam.stFrameRate.s32InputFrameRate = InputFps;
    chnparam.stFrameRate.s32OutputFrameRate = OutputFps;
    HB_VENC_SetChnParam(VeChn, &chnparam);
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

int sample_venc_reinit(int Vechn, int type, int width, int height, int bits)
{
    int s32Ret = 0;

    s32Ret = HB_VENC_StopRecvFrame(Vechn);
    if (s32Ret != 0) {
        printf("HB_VENC_StopRecvFrame %d failed\n", Vechn);
        return -1;
    }

    s32Ret = HB_VENC_DestroyChn(Vechn);
    if (s32Ret != 0) {
        printf("HB_VENC_DestroyChn %d failed\n", Vechn);
        return -1;
    }

    s32Ret = sample_venc_init(Vechn, type, width, height, bits);
    if (s32Ret != 0) {
        printf("sample_venc_init failed\n");
        return -1;
    }

    s32Ret = sample_venc_start(Vechn);
    if (s32Ret != 0) {
        printf("sample_venc_start failed\n");
        return -1;
    }

    return 0;
}

int sample_venc_setRefParam(int Vechn, uint32_t longterm_pic_period,
                    uint32_t longterm_pic_using_period)
{
    VENC_REF_PARAM_S stRefParam;
    HB_VENC_GetRefParam(Vechn, &stRefParam);
    stRefParam.use_longterm = HB_TRUE;
    stRefParam.longterm_pic_period = longterm_pic_period;
    stRefParam.longterm_pic_using_period = longterm_pic_using_period;
    HB_VENC_SetRefParam(Vechn, &stRefParam);
    return 0;
}

int sample_venc_h264cbr(VENC_RC_ATTR_S *pstRcParam, VENC_H264_CBR_S *pstH264Cbr)
{
    pstRcParam->stH264Cbr.u32BitRate = pstH264Cbr->u32BitRate;
    pstRcParam->stH264Cbr.u32FrameRate = pstH264Cbr->u32FrameRate;
    pstRcParam->stH264Cbr.u32IntraPeriod = pstH264Cbr->u32IntraPeriod;
    pstRcParam->stH264Cbr.u32VbvBufferSize = pstH264Cbr->u32VbvBufferSize;
    pstRcParam->stH264Cbr.u32IntraQp = pstH264Cbr->u32IntraQp;
    pstRcParam->stH264Cbr.u32InitialRcQp = pstH264Cbr->u32InitialRcQp;
    pstRcParam->stH264Cbr.bMbLevelRcEnable = pstH264Cbr->bMbLevelRcEnable;
    pstRcParam->stH264Cbr.u32MaxIQp = pstH264Cbr->u32MaxIQp;
    pstRcParam->stH264Cbr.u32MinIQp = pstH264Cbr->u32MinIQp;
    pstRcParam->stH264Cbr.u32MaxPQp = pstH264Cbr->u32MaxPQp;
    pstRcParam->stH264Cbr.u32MinPQp = pstH264Cbr->u32MinPQp;
    pstRcParam->stH264Cbr.u32MaxBQp = pstH264Cbr->u32MaxBQp;
    pstRcParam->stH264Cbr.u32MinBQp = pstH264Cbr->u32MinBQp;
    pstRcParam->stH264Cbr.bHvsQpEnable = pstH264Cbr->bHvsQpEnable;
    pstRcParam->stH264Cbr.s32HvsQpScale = pstH264Cbr->s32HvsQpScale;
    pstRcParam->stH264Cbr.u32MaxDeltaQp = pstH264Cbr->u32MaxDeltaQp;
    pstRcParam->stH264Cbr.bQpMapEnable = pstH264Cbr->bQpMapEnable;

    return 0;
}

int sample_venc_h264vbr(VENC_RC_ATTR_S *pstRcParam, VENC_H264_VBR_S *pstH264Vbr)
{
    pstRcParam->stH264Vbr.u32IntraPeriod = pstH264Vbr->u32IntraPeriod;
    pstRcParam->stH264Vbr.u32IntraQp = pstH264Vbr->u32IntraQp;
    pstRcParam->stH264Vbr.u32FrameRate = pstH264Vbr->u32FrameRate;
    pstRcParam->stH264Vbr.bQpMapEnable = pstH264Vbr->bQpMapEnable;

    return 0;
}

int sample_venc_h264avbr(VENC_RC_ATTR_S *pstRcParam,
                    VENC_H264_AVBR_S *pstH264AVbr)
{
    pstRcParam->stH264AVbr.u32BitRate = pstH264AVbr->u32BitRate;
    pstRcParam->stH264AVbr.u32FrameRate = pstH264AVbr->u32FrameRate;
    pstRcParam->stH264AVbr.u32IntraPeriod = pstH264AVbr->u32IntraPeriod;
    pstRcParam->stH264AVbr.u32VbvBufferSize = pstH264AVbr->u32VbvBufferSize;
    pstRcParam->stH264AVbr.u32IntraQp = pstH264AVbr->u32IntraQp;
    pstRcParam->stH264AVbr.u32InitialRcQp = pstH264AVbr->u32InitialRcQp;
    pstRcParam->stH264AVbr.bMbLevelRcEnable = pstH264AVbr->bMbLevelRcEnable;
    pstRcParam->stH264AVbr.u32MaxIQp = pstH264AVbr->u32MaxIQp;
    pstRcParam->stH264AVbr.u32MinIQp = pstH264AVbr->u32MinIQp;
    pstRcParam->stH264AVbr.u32MaxPQp = pstH264AVbr->u32MaxPQp;
    pstRcParam->stH264AVbr.u32MinPQp = pstH264AVbr->u32MinPQp;
    pstRcParam->stH264AVbr.u32MaxBQp = pstH264AVbr->u32MaxBQp;
    pstRcParam->stH264AVbr.u32MinBQp = pstH264AVbr->u32MinBQp;
    pstRcParam->stH264AVbr.bHvsQpEnable = pstH264AVbr->bHvsQpEnable;
    pstRcParam->stH264AVbr.s32HvsQpScale = pstH264AVbr->s32HvsQpScale;
    pstRcParam->stH264AVbr.u32MaxDeltaQp = pstH264AVbr->u32MaxDeltaQp;
    pstRcParam->stH264AVbr.bQpMapEnable = pstH264AVbr->bQpMapEnable;

    return 0;
}

int sample_venc_h264fixqp(VENC_RC_ATTR_S *pstRcParam,
                VENC_H264_FIXQP_S *pstH264FixQp)
{
    pstRcParam->stH264FixQp.u32FrameRate = pstH264FixQp->u32FrameRate;
    pstRcParam->stH264FixQp.u32IntraPeriod = pstH264FixQp->u32IntraPeriod;
    pstRcParam->stH264FixQp.u32IQp = pstH264FixQp->u32IQp;
    pstRcParam->stH264FixQp.u32PQp = pstH264FixQp->u32PQp;
    pstRcParam->stH264FixQp.u32BQp = pstH264FixQp->u32BQp;

    return 0;
}

int sample_venc_h264qpmap(VENC_RC_ATTR_S *pstRcParam,
        VENC_H264_QPMAP_S *pstH264QpMap)
{
    pstRcParam->stH264QpMap.u32IntraPeriod = pstH264QpMap->u32IntraPeriod;
    pstRcParam->stH264QpMap.u32FrameRate = pstH264QpMap->u32FrameRate;
    pstRcParam->stH264QpMap.u32QpMapArrayCount =
                                        pstH264QpMap->u32QpMapArrayCount;
    pstRcParam->stH264QpMap.u32QpMapArray = pstH264QpMap->u32QpMapArray;

    return 0;
}

int sample_venc_h265cbr(VENC_RC_ATTR_S *pstRcParam, VENC_H265_CBR_S *pstH265Cbr)
{
    pstRcParam->stH265Cbr.u32IntraPeriod = pstH265Cbr->u32IntraPeriod;
    pstRcParam->stH265Cbr.u32IntraQp = pstH265Cbr->u32IntraQp;
    pstRcParam->stH265Cbr.u32BitRate = pstH265Cbr->u32BitRate;
    pstRcParam->stH265Cbr.u32FrameRate = pstH265Cbr->u32FrameRate;
    pstRcParam->stH265Cbr.u32InitialRcQp = pstH265Cbr->u32InitialRcQp;
    pstRcParam->stH265Cbr.u32VbvBufferSize = pstH265Cbr->u32VbvBufferSize;
    pstRcParam->stH265Cbr.bCtuLevelRcEnable = pstH265Cbr->bCtuLevelRcEnable;
    pstRcParam->stH265Cbr.u32MaxIQp = pstH265Cbr->u32MaxIQp;
    pstRcParam->stH265Cbr.u32MinIQp = pstH265Cbr->u32MinIQp;
    pstRcParam->stH265Cbr.u32MaxPQp = pstH265Cbr->u32MaxPQp;
    pstRcParam->stH265Cbr.u32MinPQp = pstH265Cbr->u32MinPQp;
    pstRcParam->stH265Cbr.u32MaxBQp = pstH265Cbr->u32MaxBQp;
    pstRcParam->stH265Cbr.u32MinBQp = pstH265Cbr->u32MinBQp;
    pstRcParam->stH265Cbr.bHvsQpEnable = pstH265Cbr->bHvsQpEnable;
    pstRcParam->stH265Cbr.s32HvsQpScale = pstH265Cbr->s32HvsQpScale;
    pstRcParam->stH265Cbr.u32MaxDeltaQp = pstH265Cbr->u32MaxDeltaQp;
    pstRcParam->stH265Cbr.bQpMapEnable = pstH265Cbr->bQpMapEnable;

    return 0;
}

int sample_venc_h265vbr(VENC_RC_ATTR_S *pstRcParam, VENC_H265_VBR_S *pstH265Vbr)
{
    pstRcParam->stH265Vbr.u32IntraPeriod = pstH265Vbr->u32IntraPeriod;
    pstRcParam->stH265Vbr.u32IntraQp = pstH265Vbr->u32IntraQp;
    pstRcParam->stH265Vbr.u32FrameRate = pstH265Vbr->u32FrameRate;
    pstRcParam->stH265Vbr.bQpMapEnable = pstH265Vbr->bQpMapEnable;

    // pstRcParam->stH265Vbr.u32IntraPeriod = intraperiod;
    // pstRcParam->stH265Vbr.u32IntraQp = intraqp;
    // pstRcParam->stH265Vbr.u32FrameRate = framerate;
    // pstRcParam->stH265Vbr.bQpMapEnable = HB_FALSE;

    return 0;
}

int sample_venc_h265avbr(VENC_RC_ATTR_S *pstRcParam,
            VENC_H265_AVBR_S *pstH265AVbr)
{
    pstRcParam->stH265AVbr.u32IntraPeriod = pstH265AVbr->u32IntraPeriod;
    pstRcParam->stH265AVbr.u32IntraQp = pstH265AVbr->u32IntraQp;
    pstRcParam->stH265AVbr.u32BitRate = pstH265AVbr->u32BitRate;
    pstRcParam->stH265AVbr.u32FrameRate = pstH265AVbr->u32FrameRate;
    pstRcParam->stH265AVbr.u32InitialRcQp = pstH265AVbr->u32InitialRcQp;
    pstRcParam->stH265AVbr.u32VbvBufferSize = pstH265AVbr->u32VbvBufferSize;
    pstRcParam->stH265AVbr.bCtuLevelRcEnable = pstH265AVbr->bCtuLevelRcEnable;
    pstRcParam->stH265AVbr.u32MaxIQp = pstH265AVbr->u32MaxIQp;
    pstRcParam->stH265AVbr.u32MinIQp = pstH265AVbr->u32MinIQp;
    pstRcParam->stH265AVbr.u32MaxPQp = pstH265AVbr->u32MaxPQp;
    pstRcParam->stH265AVbr.u32MinPQp = pstH265AVbr->u32MinPQp;
    pstRcParam->stH265AVbr.u32MaxBQp = pstH265AVbr->u32MaxBQp;
    pstRcParam->stH265AVbr.u32MinBQp = pstH265AVbr->u32MinBQp;
    pstRcParam->stH265AVbr.bHvsQpEnable = pstH265AVbr->bHvsQpEnable;
    pstRcParam->stH265AVbr.s32HvsQpScale = pstH265AVbr->s32HvsQpScale;
    pstRcParam->stH265AVbr.u32MaxDeltaQp = pstH265AVbr->u32MaxDeltaQp;
    pstRcParam->stH265AVbr.bQpMapEnable = pstH265AVbr->bQpMapEnable;

    return 0;
}

int sample_venc_h265fixqp(VENC_RC_ATTR_S *pstRcParam,
                VENC_H265_FIXQP_S *pstH265FixQp)
{
    pstRcParam->stH265FixQp.u32FrameRate = pstH265FixQp->u32FrameRate;
    pstRcParam->stH265FixQp.u32IntraPeriod = pstH265FixQp->u32IntraPeriod;
    pstRcParam->stH265FixQp.u32IQp = pstH265FixQp->u32IQp;
    pstRcParam->stH265FixQp.u32PQp = pstH265FixQp->u32PQp;
    pstRcParam->stH265FixQp.u32BQp = pstH265FixQp->u32BQp;

    return 0;
}

int sample_venc_h265qpmap(VENC_RC_ATTR_S *pstRcParam,
                    VENC_H265_QPMAP_S *pstH265QpMap)
{
    pstRcParam->stH264QpMap.u32IntraPeriod = pstH265QpMap->u32IntraPeriod;
    pstRcParam->stH264QpMap.u32FrameRate = pstH265QpMap->u32FrameRate;
    pstRcParam->stH264QpMap.u32QpMapArrayCount =
                                pstH265QpMap->u32QpMapArrayCount;
    pstRcParam->stH264QpMap.u32QpMapArray =
                                pstH265QpMap->u32QpMapArray;

    return 0;
}

int sample_venc_mjpgfixqp(VENC_RC_ATTR_S *pstRcParam,
            VENC_MJPEG_FIXQP_S *pstMjpegFixQp)
{
    pstRcParam->stMjpegFixQp.u32FrameRate = pstMjpegFixQp->u32FrameRate;
    pstRcParam->stMjpegFixQp.u32QualityFactort =
                                        pstMjpegFixQp->u32QualityFactort;

    return 0;
}

int sample_venc_setgop(VENC_GOP_ATTR_S *pstGopAttr, int presetidx,
                        int refreshtype, int qp, int intraqp)
{
    pstGopAttr->u32GopPresetIdx = presetidx;
    pstGopAttr->s32DecodingRefreshType = refreshtype;
    if (presetidx == 10) {
        // I-P2-P1-P2-P0-P2-P1-P2-P0
        pstGopAttr->stCustomGopParam.u32CustomGopSize = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[0].u32PictureType = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[0].s32PocOffset = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[0].u32PictureQp = qp;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[0].s32NumRefPictureL0 = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[0].u32TemporalId = 0;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[0].s32RefPocL0 = 0;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[0].s32RefPocL1 = 0;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[1].u32PictureType = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[1].s32PocOffset = 2;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[1].u32PictureQp = qp;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[1].s32NumRefPictureL0 = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[1].u32TemporalId = 0;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[1].s32RefPocL0 = 0;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[1].s32RefPocL1 = 0;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[2].u32PictureType = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[2].s32PocOffset = 3;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[2].u32PictureQp = qp;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[2].s32NumRefPictureL0 = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[2].u32TemporalId = 2;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[2].s32RefPocL0 = 2;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[2].s32RefPocL1 = 0;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[3].u32PictureType = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[3].s32PocOffset = 4;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[3].u32PictureQp = qp;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[3].s32NumRefPictureL0 = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[3].u32TemporalId = 0;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[3].s32RefPocL0 = 0;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[3].s32RefPocL1 = 0;
        pstGopAttr->u32GopPresetIdx = 0;
    } else if (presetidx == 11) {
        // I-P3-P2-P3-P1-P3-P2-P3-P0 -- P3-P2-P3-P1-P3-P2-P3-P0
        pstGopAttr->stCustomGopParam.u32CustomGopSize = 8;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[0].u32PictureType = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[0].s32PocOffset = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[0].u32PictureQp = 7;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[0].s32NumRefPictureL0 = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[0].u32TemporalId = 3;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[0].s32RefPocL0 = 0;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[0].s32RefPocL1 = 0;

        pstGopAttr->stCustomGopParam.stCustomGopPicture[1].u32PictureType = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[1].s32PocOffset = 2;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[1].u32PictureQp = 5;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[1].s32NumRefPictureL0 = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[1].u32TemporalId = 2;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[1].s32RefPocL0 = 0;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[1].s32RefPocL1 = 0;

        pstGopAttr->stCustomGopParam.stCustomGopPicture[2].u32PictureType = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[2].s32PocOffset = 3;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[2].u32PictureQp = 7;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[2].s32NumRefPictureL0 = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[2].u32TemporalId = 3;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[2].s32RefPocL0 = 2;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[2].s32RefPocL1 = 0;

        pstGopAttr->stCustomGopParam.stCustomGopPicture[3].u32PictureType = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[3].s32PocOffset = 4;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[3].u32PictureQp = 3;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[3].s32NumRefPictureL0 = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[3].u32TemporalId = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[3].s32RefPocL0 = 0;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[3].s32RefPocL1 = 0;

        pstGopAttr->stCustomGopParam.stCustomGopPicture[4].u32PictureType = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[4].s32PocOffset = 5;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[4].u32PictureQp = 7;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[4].s32NumRefPictureL0 = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[4].u32TemporalId = 3;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[4].s32RefPocL0 = 4;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[4].s32RefPocL1 = 0;

        pstGopAttr->stCustomGopParam.stCustomGopPicture[5].u32PictureType = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[5].s32PocOffset = 6;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[5].u32PictureQp = 5;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[5].s32NumRefPictureL0 = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[5].u32TemporalId = 2;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[5].s32RefPocL0 = 4;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[5].s32RefPocL1 = 0;

        pstGopAttr->stCustomGopParam.stCustomGopPicture[6].u32PictureType = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[6].s32PocOffset = 7;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[6].u32PictureQp = 7;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[6].s32NumRefPictureL0 = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[6].u32TemporalId = 3;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[6].s32RefPocL0 = 6;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[6].s32RefPocL1 = 0;

        pstGopAttr->stCustomGopParam.stCustomGopPicture[7].u32PictureType = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[7].s32PocOffset = 8;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[7].u32PictureQp = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[7].s32NumRefPictureL0 = 1;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[7].u32TemporalId = 0;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[7].s32RefPocL0 = 0;
        pstGopAttr->stCustomGopParam.stCustomGopPicture[7].s32RefPocL1 = 0;
        pstGopAttr->u32GopPresetIdx = 0;
    }

    return 0;
}

