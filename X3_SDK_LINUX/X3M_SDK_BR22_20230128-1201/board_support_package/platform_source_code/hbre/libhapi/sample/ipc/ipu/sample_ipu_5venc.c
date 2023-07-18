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

#include "hb_vin_api.h"
#include "hb_mipi_api.h"
#include "hb_vio_interface.h"

#include "hb_comm_venc.h"
#include "hb_venc.h"
#include "hb_vdec.h"
#include "hb_vp_api.h"
#include "hb_rgn.h"
// #include "hb_vot.h"
#include "hb_vps_api.h"
// #include "hb_utils.h"
#include "sample.h"
#include <arm_neon.h>

static pthread_t vens_thid[6];
static pthread_t rgn_thid[6];
static pthread_t vps_thid[6]={0};
    // pthread_t vdec_thid;
static vencParam vencparam[6]={0};
static rgnParam  rgnparam[6]={0};
static int rgnenable = 0;
static vencParam vpsparam[6]={0};

extern int ion_alloc_phy(int size, int *fd, char **vaddr, unsigned long *paddr);

int sample_reconfig_profilelevel()
{
    int ret;
    static int profile_idx = 0;

    printf("=================sample_vps_unbind_vin\n");
    if (g_bindflag == 1)
        ret = sample_vps_unbind_venc(vencparam[2].vpsGrp,
            vencparam[2].vpsChn, vencparam[2].veChn);
    vencparam[2].quit = 1;
    // usleep(100000);
    if (vencparam[2].type > 0) ret = pthread_join(vens_thid[2], NULL);
    profile_idx++;
    if (profile_idx > 3) profile_idx = 0;
    if (profile_idx == 0)
        ret = sample_venc_seth264_profile(vencparam[2].veChn, vencparam[2].type,
            vencparam[2].width, vencparam[2].height, vencparam[2].bitrate,
            HB_H264_PROFILE_BP, HB_H264_LEVEL5);
    else if (profile_idx == 1)
        ret = sample_venc_seth264_profile(vencparam[2].veChn, vencparam[2].type,
            vencparam[2].width, vencparam[2].height, vencparam[2].bitrate,
            HB_H264_PROFILE_MP, HB_H264_LEVEL5_1);
    else
        ret = sample_venc_seth264_profile(vencparam[2].veChn, vencparam[2].type,
            vencparam[2].width, vencparam[2].height, vencparam[2].bitrate,
            HB_H264_PROFILE_HP, HB_H264_LEVEL5_2);
    if (ret < 0) {
        printf("sample_venc_setroi error!\n");
        return ret;
    }

    vencparam[2].quit = 0;
    if (vencparam[2].type > 0) {
        if (g_bindflag == 1) {
            ret = sample_vps_bind_venc(vencparam[2].vpsGrp,
                            vencparam[2].vpsChn, vencparam[2].veChn);
            printf("vps_bind_venc ret %d\n", ret);
        }
        ret = pthread_create(&vens_thid[2], NULL,
                            (void *)venc_thread, &(vencparam[2]));
    }

    return 0;
}

int sample_reconfig_rcmode(int type)
{
    int ret;
    static int rc_idx = 0;

    printf("=================sample_reconfig_rcmode\n");
    if (g_bindflag == 1)
        ret = sample_vps_unbind_venc(vencparam[2].vpsGrp,
            vencparam[2].vpsChn, vencparam[2].veChn);
    vencparam[2].quit = 1;
    // usleep(100000);
    if (vencparam[2].type > 0) ret = pthread_join(vens_thid[2], NULL);
    if (type == 0) {
        if (rc_idx++ > 4) rc_idx = 0;
    } else {
        rc_idx++;
        if (rc_idx < 5) rc_idx = 5;
        else if (rc_idx > 9) rc_idx = 5;
    }
    printf("sample_reconfig_rcmode: %d\n", rc_idx);
    ret = sample_venc_set_rcmode(vencparam[2].veChn, vencparam[2].type,
        vencparam[2].width, vencparam[2].height, vencparam[2].bitrate,
        (VENC_RC_MODE_E)rc_idx);
    if (ret < 0) {
        printf("sample_venc_setroi error!\n");
        return ret;
    }

    vencparam[2].quit = 0;
    if (vencparam[2].type > 0) {
        if (g_bindflag == 1) {
            ret = sample_vps_bind_venc(vencparam[2].vpsGrp,
                            vencparam[2].vpsChn, vencparam[2].veChn);
            printf("vps_bind_venc ret %d\n", ret);
        }
        ret = pthread_create(&vens_thid[2], NULL,
                            (void *)venc_thread, &(vencparam[2]));
    }
    printf("=================sample_reconfig_rcmode end.\n");
    return 0;
}

int sample_reconfig_roi()
{
    int ret;

    printf("=================sample_reconfig_roi\n");
    if (g_bindflag == 1)
        ret = sample_vps_unbind_venc(vencparam[0].vpsGrp,
            vencparam[0].vpsChn, vencparam[0].veChn);
    vencparam[0].quit = 1;
    // usleep(100000);
    if (vencparam[0].type > 0) ret = pthread_join(vens_thid[0], NULL);

    ret = sample_venc_setroi(vencparam[0].veChn, vencparam[0].type,
        vencparam[0].width, vencparam[0].height, vencparam[0].bitrate);
    if (ret < 0) {
        printf("sample_venc_setroi error!\n");
        return ret;
    }

    vencparam[0].quit = 0;
    if (vencparam[0].type > 0) {
        if (g_bindflag == 1) {
            ret = sample_vps_bind_venc(vencparam[0].vpsGrp,
                            vencparam[0].vpsChn, vencparam[0].veChn);
            printf("vps_bind_venc ret %d\n", ret);
        }
        ret = pthread_create(&vens_thid[0], NULL,
                            (void *)venc_thread, &(vencparam[0]));
    }

    sample_venc_setrcattr(0);
    printf("=================sample_reconfig_roi end\n");
    return 0;
}

int sample_ipu_reconfig()
{
    int ret;
    static int idx = 0;

    printf("=================sample_vin_unbind_vps\n");
    if (g_bindflag == 1)
        ret = sample_vps_unbind_venc(vencparam[2].vpsGrp,
            vencparam[2].vpsChn, vencparam[2].veChn);
    vencparam[2].quit = 1;
    // usleep(100000);
    if (vencparam[2].type > 0) ret = pthread_join(vens_thid[2], NULL);

    if ((idx%3) == 0) {
        vencparam[2].width = 1280;
        vencparam[2].height = 720;
    } else if ((idx%3) == 1) {
        vencparam[2].width = 704;
        vencparam[2].height = 576;
    } else {
        vencparam[2].width = 1920;
        vencparam[2].height = 1080;
    }
    idx++;

    //
    ret = sample_vps_chn_reconfig(vencparam[2].vpsGrp,
            vencparam[2].vpsChn, vencparam[2].width, vencparam[2].height);
    if (ret) {
        printf("sample_vps_chn_reconfig failed: %x\n", ret);
        return ret;
    }

    ret = sample_venc_reinit(vencparam[2].veChn, vencparam[2].type,
        vencparam[2].width, vencparam[2].height, vencparam[2].bitrate);
    if (ret < 0) {
        printf("sample_venc_reinit error!\n");
        return ret;
    }

    vencparam[2].quit = 0;
    if (vencparam[2].type > 0) {
        if (g_bindflag == 1) {
            ret = sample_vps_bind_venc(vencparam[2].vpsGrp,
                            vencparam[2].vpsChn, vencparam[2].veChn);
            printf("vps_bind_venc ret %d\n", ret);
        }
        ret = pthread_create(&vens_thid[2], NULL,
                            (void *)venc_thread, &(vencparam[2]));
    }

    // printf("=================hb_vps_start done\n");
    // for (i=0; i<5; i++) {
    //     printf("vencparam[%d].type: %x\n", i, vencparam[i].type);
    //     if (vencparam[i].type == 0) continue;
    //     ret = sample_venc_reinit(vencparam[i].veChn, vencparam[i].type,
    //         vencparam[i].width, vencparam[i].height, vencparam[i].bitrate);
    //     if (ret < 0) {
    //         printf("sample_venc_init error!\n");
    //         return ret;
    //     }
    // }

    // for (i=0; i<5; i++) {
    //     vencparam[i].quit = 0;
    //     if (vencparam[i].type > 0) {
    //         if (g_bindflag == 1)
    //             ret = sample_vps_bind_venc(vencparam[i].vpsGrp,
    //                             vencparam[i].vpsChn, vencparam[i].veChn);
    //         ret = pthread_create(&vens_thid[i], NULL,
    //                             (void *)venc_thread, &(vencparam[i]));
    //     }
    // }

    return 0;
}

int grp_process_data(int grp_pipeid, hb_vio_buffer_t *in_buf,
		hb_vio_buffer_t *out_buf)
{
	int ret = -1;
	ret = HB_VPS_SendFrame(grp_pipeid, in_buf, 1000);
	if (ret) {
			printf("HB_VPS_SendFrame error grp = %d!!!\n", grp_pipeid);
			return ret;
	} else {
			// printf("HB_VPS_SendFrame ok\n");
	}
	ret = HB_VPS_GetChnFrame(grp_pipeid, 0, out_buf, 2000);

	return ret;
}

void pym_to_gdc_thread(void *param)
{
    int ret;
    pym_buffer_t out_pym_buf;
    struct timeval ts0;
    int dumpnum = 10;
    hb_vio_buffer_t out_buf, in_buf;
    time_t start, last = time(NULL);
    int idx = 0;

    printf("pym_get_thread in===========\n");

    while (g_exit == 0) {
        ret = HB_VPS_GetChnFrame(0, 6, &out_pym_buf, 2000);
        if (ret != 0) {
            printf("HB_VPS_GetChnFrame error!!!\n");
            continue;
        }
        start = time(NULL);
        idx++;
        if (start > last) {
            gettimeofday(&ts0, NULL);
            printf("[%ld.%06ld]pym fps %d\n", ts0.tv_sec, ts0.tv_usec, idx);
            last = start;
            idx = 0;
        }

        gettimeofday(&ts0, NULL);

        if (access("./pymdump", F_OK) == 0) {
            system("rm ./pymdump");
            dumpnum = 0;
        }

        if (dumpnum < 3) {
            char fname[128];
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

        in_buf.img_addr.paddr[0] = out_pym_buf.pym[0].paddr[0];
		in_buf.img_addr.paddr[1] = out_pym_buf.pym[0].paddr[1];
		in_buf.img_addr.addr[0] = out_pym_buf.pym[0].addr[0];
		in_buf.img_addr.addr[1] = out_pym_buf.pym[0].addr[1];
		in_buf.img_addr.height = out_pym_buf.pym[0].height;
		in_buf.img_addr.width = out_pym_buf.pym[0].width;
		in_buf.img_addr.stride_size = out_pym_buf.pym[0].stride_size;
		in_buf.img_info.planeCount = 2;
        ret = grp_process_data(1, &in_buf, &out_buf);
        if (ret == 0) {
            // printf("gdc: w x h = %d x %d\n", out_buf.img_addr.width, out_buf.img_addr.height);
            HB_VPS_ReleaseChnFrame(1, 0, &out_buf);
        }

        ret = HB_VPS_ReleaseChnFrame(0, 6, &out_pym_buf);
        if (ret) {
            printf("HB_VPS_ReleaseChnFrame error!!!\n");
            break;
        }
    }

    // src_y_data = out_pym_buf.pym_roi[0][0].addr[0];
    // src_uv_data = out_pym_buf.pym_roi[0][0].addr[1];
    // src_h = 704;//out_pym_buf.pym_roi[0][0].height;
    // src_w = out_pym_buf.pym_roi[0][0].width;
    printf("pym_get_thread out===========\n");
    g_exit = 1;

    return;
}

void test_getvpschan_thread(void *vencpram)
{
	int ret;
    int vpsGrp = 0;
    int vpsChn = 0;
    hb_vio_buffer_t out_buf;
    vencParam *vparam = (vencParam*)vencpram;
    int idx = 0;

    vpsGrp = 0;  // vparam->vpsGrp;
    vpsChn = 2;  // vparam->vpsChn;

	printf("test_getvpschan_thread in, %d,%d\n", vpsGrp, vpsChn);

	while (g_exit == 0) {
        if (vparam->quit == 1) {
            break;
        }
        idx++;
        // if (idx%4 == 0)
        //     HB_VPS_SetChnRotate(0, 2, ROTATION_0);
        // else if (idx%4 == 1)
        //     HB_VPS_SetChnRotate(0, 2, ROTATION_90);
        // else if (idx%4 == 2)
        //     HB_VPS_SetChnRotate(0, 2, ROTATION_180);
        // else
        //     HB_VPS_SetChnRotate(0, 2, ROTATION_270);
        usleep(1000000);
	/* coverity[overrun-buffer-val] */
        ret = HB_VPS_GetChnFrame(vpsGrp, vpsChn, &out_buf, 2000);
        if (ret != 0) {
            printf("HB_VPS_GetChnFrame error %d!!!\n", ret);
            continue;
        } else {
            // printf("HB_VPS_GetChnFrame done\n");
            
            char fname[32];
            int width, height;
            width = out_buf.img_addr.width;
            height = out_buf.img_addr.height;
            sprintf(fname, "vps_%d_%d_%dx%d_%d.yuv", vpsGrp, vpsChn,
                width, height, idx);
            printf("widthxheight = %d x %d\n", width, height);
            // printf("start dump %s\n", fname);
            // FILE *fw = fopen(fname, "wb");
            // if (fw != NULL) {
            //     fwrite(out_buf.img_addr.addr[0], width * height, 1, fw);
            //     fwrite(out_buf.img_addr.addr[1], width * height / 2, 1, fw);
            //     fclose(fw);
            // }
        }
        // ret = HB_VPS_GetChnFrame(vpsGrp, vpsChn, &out_buf1, 2000);
        // if (ret != 0) {
        //     printf("HB_VPS_GetChnFrame1 error %d!!!\n", ret);
        //     ret = HB_VPS_ReleaseChnFrame(vpsGrp, vpsChn, &out_buf);
        //     continue;
        // } else {
        //     printf("HB_VPS_GetChnFrame1 done\n");
        // }

        // ret = HB_VPS_GetChnFrame(vpsGrp, vpsChn, &out_buf2, 2000);
        // if (ret != 0) {
        //     printf("HB_VPS_GetChnFrame2 error %d!!!\n", ret);
        //     ret = HB_VPS_ReleaseChnFrame(vpsGrp, vpsChn, &out_buf);
        //     ret = HB_VPS_ReleaseChnFrame(vpsGrp, vpsChn, &out_buf1);
        //     continue;
        // } else {
        //     printf("HB_VPS_GetChnFrame2 done\n");
        // }

        // ret = HB_VPS_GetChnFrame(vpsGrp, vpsChn, &out_buf3, 2000);
        // if (ret != 0) {
        //     printf("HB_VPS_GetChnFrame3 error %d!!!\n", ret);
        //     ret = HB_VPS_ReleaseChnFrame(vpsGrp, vpsChn, &out_buf);
        //     ret = HB_VPS_ReleaseChnFrame(vpsGrp, vpsChn, &out_buf1);
        //     ret = HB_VPS_ReleaseChnFrame(vpsGrp, vpsChn, &out_buf2);
        //     continue;
        // } else {
        //     printf("HB_VPS_GetChnFrame3 done\n");
        // }

        ret = HB_VPS_ReleaseChnFrame(vpsGrp, vpsChn, &out_buf);
        // ret = HB_VPS_ReleaseChnFrame(vpsGrp, vpsChn, &out_buf1);
        // ret = HB_VPS_ReleaseChnFrame(vpsGrp, vpsChn, &out_buf2);
        // ret = HB_VPS_ReleaseChnFrame(vpsGrp, vpsChn, &out_buf3);
	}

	printf("test_getvpschan_thread exit\n");
}

void test_getvinchan_thread(void *vencpram)
{
	int ret;
    hb_vio_buffer_t out_buf;
    int vpsGrp = 0;
    int vpsChn = 1;

    vencParam *vparam = (vencParam*)vencpram;

	printf("test_getvinchan_thread in, %d,%d\n", vpsGrp, vpsChn);

	while (g_exit == 0) {
        if (vparam->quit == 1) {
            break;
        }
        usleep(100000);
        ret = HB_VIN_GetChnFrame(vpsGrp, vpsChn, &out_buf, 2000);
        if (ret != 0) {
            printf("HB_VIN_GetChnFrame error %d!!!\n", ret);
            continue;
        } else {
            printf("HB_VIN_GetChnFrame done\n");
        }

        ret = HB_VIN_ReleaseChnFrame(vpsGrp, vpsChn, &out_buf);
	}

	printf("test_getvinchan_thread exit\n");
}

void pym_get_thread(void *param)
{
    int ret;
    pym_buffer_t out_pym_buf;
    // int mmz_size;
    struct timeval ts0, ts1, ts2, ts3, ts4, ts5;
    int pts = 0;
    int dumpnum = 10;

    VIDEO_FRAME_S pstFrame;
    // memset(&pstFrame, 0, sizeof(VIDEO_FRAME_S));

    printf("pym_get_thread in===========\n");
    // veChn = vparam->veChn;
    // width = vparam->width;
    // height = vparam->height;

    // mmz_size = width * height * 3 / 2;
    
					
	// gettimeofday(&ts, NULL);			
	//snprintf(str, sizeof(str), "%ld.%06ld", ts.tv_sec, ts.tv_usec);			
    time_t start, last = time(NULL);
    int idx = 0;
    while (g_exit == 0) {
        ret = HB_VPS_GetChnFrame(0, 6, &out_pym_buf, 2000);
        if (ret != 0) {
            printf("HB_VPS_GetChnFrame error!!!\n");
            continue;
        }
        start = time(NULL);
        idx++;
        if (start > last) {
            gettimeofday(&ts0, NULL);
            printf("[%ld.%06ld]pym fps %d\n", ts0.tv_sec, ts0.tv_usec, idx);
            last = start;
            idx = 0;
        }

        gettimeofday(&ts0, NULL);
        //printf("\n[%d.%5d]pym data %d %d rdy\n", ts0.tv_sec, ts0.tv_usec, pts, pts*3000);

        // printf("pym layer0: wxh = %dx%d\n", out_pym_buf.pym[0].width, out_pym_buf.pym[0].height);
        // printf("pym layer1: wxh = %dx%d\n", out_pym_buf.pym_roi[0][0].width, out_pym_buf.pym_roi[0][0].height);
        // printf("pym layer4: wxh = %dx%d\n", out_pym_buf.pym[1].width, out_pym_buf.pym[1].height);
        // printf("pym layer5: wxh = %dx%d\n", out_pym_buf.pym_roi[1][0].width, out_pym_buf.pym_roi[1][0].height);
        // printf("pym layer6: wxh = %dx%d\n", out_pym_buf.pym_roi[1][1].width, out_pym_buf.pym_roi[1][1].height);
        // printf("pym layer7: wxh = %dx%d\n", out_pym_buf.pym_roi[1][2].width, out_pym_buf.pym_roi[1][2].height);

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

        // 3840x2160
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
        // if (g_osd_flag > 0) {
        //     gettimeofday(&ts0, NULL);
        //     osd_process_t1* osd_info;
        //     osd_info = (osd_process_t1*)malloc(sizeof(osd_process_t1));
        //     osd_info->width = 3840/4;
        //     osd_info->height = 2160/4;
        //     osd_info->start_x = 0;
        //     osd_info->start_y = 0;
        //     osd_info->image_width = 3840;
        //     osd_info->src_addr = out_pym_buf.pym[2].addr[0];
        //     osd_info->tar_y_addr = pstFrame.stVFrame.vir_ptr[0];
        //     osd_info->tar_uv_addr = pstFrame.stVFrame.vir_ptr[1];
        //     osd_info->color_map = color_map;
        //     osd_info->overlay_mode = 0;
        //     ipu_osd_process_workfun1((void *)osd_info);
        //     gettimeofday(&ts1, NULL);
        // }
        ret = HB_VENC_SendFrame(0, &pstFrame, 3000);
        if (ret < 0) {
            printf("HB_VENC_SendFrame 0 error!!!\n");
            // break;
        }
        gettimeofday(&ts1, NULL);

        // 1080P
        memset(&pstFrame, 0, sizeof(VIDEO_FRAME_S));
        pstFrame.stVFrame.pix_format = HB_PIXEL_FORMAT_NV12;
        
        pstFrame.stVFrame.width = out_pym_buf.pym[1].width;
        pstFrame.stVFrame.height = out_pym_buf.pym[1].height;
        pstFrame.stVFrame.size = out_pym_buf.pym[1].width *
                out_pym_buf.pym[1].height * 3 / 2;

        pstFrame.stVFrame.phy_ptr[0] = out_pym_buf.pym[1].paddr[0];
        pstFrame.stVFrame.phy_ptr[1] = out_pym_buf.pym[1].paddr[1];

        pstFrame.stVFrame.vir_ptr[0] = out_pym_buf.pym[1].addr[0];
        pstFrame.stVFrame.vir_ptr[1] = out_pym_buf.pym[1].addr[1];
        pstFrame.stVFrame.pts = pts;
        // if (g_osd_flag > 0) {
        //     gettimeofday(&ts0, NULL);
        //     osd_process_t1* osd_info;
        //     osd_info = (osd_process_t1*)malloc(sizeof(osd_process_t1));
        //     osd_info->width = 1920/4;
        //     osd_info->height = 1080/4;
        //     osd_info->start_x = 0;
        //     osd_info->start_y = 0;
        //     osd_info->image_width = 1920;
        //     osd_info->src_addr = out_pym_buf.pym[3].addr[0];
        //     osd_info->tar_y_addr = pstFrame.stVFrame.vir_ptr[0];
        //     osd_info->tar_uv_addr = pstFrame.stVFrame.vir_ptr[1];
        //     osd_info->color_map = color_map;
        //     osd_info->overlay_mode = 0;
        //     ipu_osd_process_workfun1((void *)osd_info);
        //     gettimeofday(&ts1, NULL);
        //     // printf("memcpy noncached use time: %d us\n", 
        //     //     (ts1.tv_sec - ts0.tv_sec)*1000000 + ts1.tv_usec -ts0.tv_usec);
        // }
        ret = HB_VENC_SendFrame(1, &pstFrame, 3000);
        if (ret < 0) {
            printf("HB_VENC_SendFrame 1 error!!!\n");
            // break;
        }
        gettimeofday(&ts2, NULL);

        // 1080P
        memset(&pstFrame, 0, sizeof(VIDEO_FRAME_S));
        pstFrame.stVFrame.pix_format = HB_PIXEL_FORMAT_NV12;
        // pstFrame.stVFrame.width = out_pym_buf.pym_roi[1][0].width;
        // pstFrame.stVFrame.height = out_pym_buf.pym_roi[1][0].height;
        // pstFrame.stVFrame.size = out_pym_buf.pym_roi[1][0].width *
        //                         out_pym_buf.pym_roi[1][0].height * 3 / 2;

        // pstFrame.stVFrame.phy_ptr[0] = out_pym_buf.pym_roi[1][0].paddr[0];
        // pstFrame.stVFrame.phy_ptr[1] = out_pym_buf.pym_roi[1][0].paddr[1];

        // pstFrame.stVFrame.vir_ptr[0] = out_pym_buf.pym_roi[1][0].addr[0];
        // pstFrame.stVFrame.vir_ptr[1] = out_pym_buf.pym_roi[1][0].addr[1];
        pstFrame.stVFrame.width = out_pym_buf.pym[1].width;
        pstFrame.stVFrame.height = out_pym_buf.pym[1].height;
        pstFrame.stVFrame.size = out_pym_buf.pym[1].width *
                out_pym_buf.pym[1].height * 3 / 2;

        pstFrame.stVFrame.phy_ptr[0] = out_pym_buf.pym[1].paddr[0];
        pstFrame.stVFrame.phy_ptr[1] = out_pym_buf.pym[1].paddr[1];

        pstFrame.stVFrame.vir_ptr[0] = out_pym_buf.pym[1].addr[0];
        pstFrame.stVFrame.vir_ptr[1] = out_pym_buf.pym[1].addr[1];
        pstFrame.stVFrame.pts = pts;
        // if (g_osd_flag > 0) {
        //     gettimeofday(&ts0, NULL);
        //     osd_process_t1* osd_info;
        //     osd_info = (osd_process_t1*)malloc(sizeof(osd_process_t1));
        //     osd_info->width = 1920/4;
        //     osd_info->height = 1080/4;
        //     osd_info->start_x = 0;
        //     osd_info->start_y = 0;
        //     osd_info->image_width = 1920;
        //     osd_info->src_addr = out_pym_buf.pym[3].addr[0];
        //     osd_info->tar_y_addr = pstFrame.stVFrame.vir_ptr[0];
        //     osd_info->tar_uv_addr = pstFrame.stVFrame.vir_ptr[1];
        //     osd_info->color_map = color_map;
        //     osd_info->overlay_mode = 0;
        //     ipu_osd_process_workfun1((void *)osd_info);
        //     gettimeofday(&ts1, NULL);
        //     // printf("memcpy noncached use time: %d us\n", 
        //     //     (ts1.tv_sec - ts0.tv_sec)*1000000 + ts1.tv_usec -ts0.tv_usec);
        // }
        ret = HB_VENC_SendFrame(2, &pstFrame, 3000);
        if (ret < 0) {
            printf("HB_VENC_SendFrame 2 error!!!\n");
            // break;
        }
        gettimeofday(&ts3, NULL);

        // 704x576
        memset(&pstFrame, 0, sizeof(VIDEO_FRAME_S));
        pstFrame.stVFrame.pix_format = HB_PIXEL_FORMAT_NV12;
        pstFrame.stVFrame.width = out_pym_buf.pym_roi[1][1].width;
        pstFrame.stVFrame.height = out_pym_buf.pym_roi[1][1].height;
        pstFrame.stVFrame.size = out_pym_buf.pym_roi[1][1].width *
                                out_pym_buf.pym_roi[1][1].height * 3 / 2;

        pstFrame.stVFrame.phy_ptr[0] = out_pym_buf.pym_roi[1][1].paddr[0];
        pstFrame.stVFrame.phy_ptr[1] = out_pym_buf.pym_roi[1][1].paddr[1];

        pstFrame.stVFrame.vir_ptr[0] = out_pym_buf.pym_roi[1][1].addr[0];
        pstFrame.stVFrame.vir_ptr[1] = out_pym_buf.pym_roi[1][1].addr[1];
        pstFrame.stVFrame.pts = pts;
        // if (g_osd_flag > 0) {
        //     gettimeofday(&ts0, NULL);
        //     osd_process_t1* osd_info;
        //     osd_info = (osd_process_t1*)malloc(sizeof(osd_process_t1));
        //     osd_info->width = 704/4;
        //     osd_info->height = 576/4;
        //     osd_info->start_x = 0;
        //     osd_info->start_y = 0;
        //     osd_info->image_width = 704;
        //     osd_info->src_addr = out_pym_buf.pym[2].addr[0];
        //     osd_info->tar_y_addr = pstFrame.stVFrame.vir_ptr[0];
        //     osd_info->tar_uv_addr = pstFrame.stVFrame.vir_ptr[1];
        //     osd_info->color_map = color_map;
        //     osd_info->overlay_mode = 0;
        //     ipu_osd_process_workfun1((void *)osd_info);
        //     gettimeofday(&ts1, NULL);
        // }
        ret = HB_VENC_SendFrame(3, &pstFrame, 3000);
        if (ret < 0) {
            printf("HB_VENC_SendFrame 3 error!!!\n");
            // break;
        }
        gettimeofday(&ts4, NULL);

        // 704x576
        memset(&pstFrame, 0, sizeof(VIDEO_FRAME_S));
        pstFrame.stVFrame.pix_format = HB_PIXEL_FORMAT_NV12;
        pstFrame.stVFrame.width = out_pym_buf.pym_roi[1][2].width;
        pstFrame.stVFrame.height = out_pym_buf.pym_roi[1][2].height;
        pstFrame.stVFrame.size = out_pym_buf.pym_roi[1][2].width *
                                out_pym_buf.pym_roi[1][2].height * 3 / 2;
        pstFrame.stVFrame.phy_ptr[0] = out_pym_buf.pym_roi[1][2].paddr[0];
        pstFrame.stVFrame.phy_ptr[1] = out_pym_buf.pym_roi[1][2].paddr[1];
        pstFrame.stVFrame.vir_ptr[0] = out_pym_buf.pym_roi[1][2].addr[0];
        pstFrame.stVFrame.vir_ptr[1] = out_pym_buf.pym_roi[1][2].addr[1];
        pstFrame.stVFrame.pts = pts;
        // if (g_osd_flag > 0) {
        //     gettimeofday(&ts0, NULL);
        //     osd_process_t1* osd_info;
        //     osd_info = (osd_process_t1*)malloc(sizeof(osd_process_t1));
        //     osd_info->width = 704/4;
        //     osd_info->height = 576/4;
        //     osd_info->start_x = 0;
        //     osd_info->start_y = 0;
        //     osd_info->image_width = 704;
        //     osd_info->src_addr = out_pym_buf.pym[2].addr[0];
        //     osd_info->tar_y_addr = pstFrame.stVFrame.vir_ptr[0];
        //     osd_info->tar_uv_addr = pstFrame.stVFrame.vir_ptr[1];
        //     osd_info->color_map = color_map;
        //     osd_info->overlay_mode = 0;
        //     ipu_osd_process_workfun1((void *)osd_info);
        //     gettimeofday(&ts1, NULL);
        // }
        ret = HB_VENC_SendFrame(4, &pstFrame, 3000);
        if (ret < 0) {
            printf("HB_VENC_SendFrame 4 error!!!\n");
            // break;
        }
        gettimeofday(&ts5, NULL);
        // printf("\n[%d.%06d]pym data %d %d rdy, [%d.%06d],[%d.%06d],[%d.%06d],[%d.%06d],[%d.%06d]\n", 
        //     ts0.tv_sec, ts0.tv_usec, pts, pts*3000, ts1.tv_sec, ts1.tv_usec, ts2.tv_sec, ts2.tv_usec,
        //     ts3.tv_sec, ts3.tv_usec, ts4.tv_sec, ts4.tv_usec, ts5.tv_sec, ts5.tv_usec);
        pts++;
        ret = HB_VPS_ReleaseChnFrame(0, 6, &out_pym_buf);
        if (ret) {
            printf("HB_VPS_ReleaseChnFrame error!!!\n");
            break;
        }
    }

    // src_y_data = out_pym_buf.pym_roi[0][0].addr[0];
    // src_uv_data = out_pym_buf.pym_roi[0][0].addr[1];
    // src_h = 704;//out_pym_buf.pym_roi[0][0].height;
    // src_w = out_pym_buf.pym_roi[0][0].width;
    printf("pym_get_thread out===========\n");
    g_exit = 1;

    return;
}

void venc_from_pym_thread(void *vencpram)
{
	int ret;
    int width = 1280, height = 720;
    int mmz_size = width * height * 3 / 2;
    int veChn = 0;
    int vpsGrp = 0;
    int vpsChn = 0;
    vencParam *vparam = (vencParam*)vencpram;
    FILE *outFile;
    char fname[32];

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
            ret = HB_VENC_GetStream(veChn, &pstStream, 300);
            if (ret < 0) {
                // printf("HB_VENC_GetStream error!!!\n");
                //break;
            } else {
                struct timeval ts0;
                gettimeofday(&ts0, NULL);
                // printf("[%d.%5d]venc%d %d\n", ts0.tv_sec, ts0.tv_usec,
                //      veChn, pstStream.pstPack.pts);
                if (g_save_flag == 1) {
                    fwrite(pstStream.pstPack.vir_ptr,
                        pstStream.pstPack.size, 1, outFile);
                }
                
                HB_VENC_ReleaseStream(veChn, &pstStream);
            }
        // }
	}

    if (g_save_flag == 1) {
        fclose(outFile);
    }

    // ret = HB_VENC_CloseFd(veChn, pollFd);

	printf("venc_from_pym_thread exit\n");
    return;
}

int hb_vps_rotate_init(int pipeId, ROTATION_E rotate)
{
	int ret = 0;

	VPS_GRP_ATTR_S grp_attr;
	VPS_CHN_ATTR_S chn_attr;

	memset(&grp_attr, 0, sizeof(VPS_GRP_ATTR_S));
	grp_attr.maxW = 3840;
	grp_attr.maxH = 2160;
    grp_attr.frameDepth = 1;
	ret = HB_VPS_CreateGrp(pipeId, &grp_attr);
	if (ret) {
		printf("HB_VPS_CreateGrp error!!!\n");
	} else {
		printf("created a group ok:GrpId = %d\n", pipeId);
	}

	ret = HB_VPS_SetGrpRotate(pipeId, rotate);
	if (ret) {
		 printf("HB_VPS_SetGrpRotate %d error!!!\n", pipeId);
		 return ret;
	} else {
		 printf("HB_VPS_SetGrpRotate ok: grp_id = %d\n", pipeId);
	}

    memset(&chn_attr, 0, sizeof(VPS_CHN_ATTR_S));
    chn_attr.enScale = 1;
    chn_attr.frameDepth = 2;
    if (rotate == ROTATION_90 || rotate == ROTATION_270) {
        chn_attr.width = 2160;
        chn_attr.height = 3840;
    } else {
        chn_attr.width = 3840;
        chn_attr.height = 2160;
    }

    ret = HB_VPS_SetChnAttr(pipeId, 2, &chn_attr);
    if (ret) {
        printf("HB_VPS_SetChnAttr error!!!\n");
    } else {
        printf("set chn Attr ok: GrpId = %d, chn_id = %d\n",
                                pipeId, 2);
    }
    HB_VPS_EnableChn(pipeId, 2);

    if (rotate == ROTATION_90 || rotate == ROTATION_270) {
        chn_attr.width = 1080;
        chn_attr.height = 1920;
    } else {
        chn_attr.width = 1920;
        chn_attr.height = 1080;
    }
    ret = HB_VPS_SetChnAttr(pipeId, 1, &chn_attr);
    if (ret) {
        printf("HB_VPS_SetChnAttr error!!!\n");
    } else {
        printf("set chn Attr ok: GrpId = %d, chn_id = %d\n",
                                pipeId, 1);
    }
    HB_VPS_EnableChn(pipeId, 1);

    ret = HB_VPS_StartGrp(pipeId);
	if (ret) {
			printf("HB_VPS_StartGrp %d error!!!\n", pipeId);
			return ret;
	} else {
			printf("start grp ok: grp_id = %d\n", pipeId);
	}

	return ret;
}

int sample_reconfig_rotate(int pipeId)
{
    int ret;
    static int rotate = 0;
    printf("vin unbind vps\n");
    sample_vin_unbind_vps(0, vin_vps_mode);
    printf("wait vps get frame thread quit\n");
    vpsparam[0].quit = 1;
    if (vps_thid[0] > 0)
        pthread_join(vps_thid[0], NULL);
    printf("vps stop/destory\n");
    HB_VPS_StopGrp(pipeId);
    HB_VPS_DestroyGrp(pipeId);
    printf("vps reinit\n");
    hb_vps_rotate_init(pipeId, (ROTATION_E)(rotate%4));
    printf("vin bind vps\n");
    sample_vin_bind_vps(0, vin_vps_mode);
    rotate++;
    printf("create vps get frame thread\n");
    vpsparam[0].quit = 0;
    ret = pthread_create(&vps_thid[0], NULL,
        (void *)test_getvpschan_thread, &(vpsparam[0]));

    return ret;
}

int hb_vps1_init(int pipeId)
{
	int ret = 0;

	VPS_GRP_ATTR_S grp_attr;
	VPS_CHN_ATTR_S chn_attr;

	memset(&grp_attr, 0, sizeof(VPS_GRP_ATTR_S));
	grp_attr.maxW = 1920;
	grp_attr.maxH = 1080;
    grp_attr.frameDepth = 1;
	ret = HB_VPS_CreateGrp(pipeId, &grp_attr);
	if (ret) {
		printf("HB_VPS_CreateGrp error!!!\n");
	} else {
		printf("created a group ok:GrpId = %d\n", pipeId);
	}
    ret = HB_SYS_SetVINVPSMode(pipeId, vin_vps_mode);

	// ret = HB_VPS_SetGrpRotate(pipeId, ROTATION_180);
	// if (ret) {
	// 	 printf("HB_VPS_SetGrpRotate %d error!!!\n", pipeId);
	// 	 return ret;
	// } else {
	// 	 printf("HB_VPS_SetGrpRotate ok: grp_id = %d\n", pipeId);
	// }
    memset(&chn_attr, 0, sizeof(VPS_CHN_ATTR_S));
    chn_attr.enScale = 1;
    chn_attr.frameDepth = 1;
    chn_attr.width = 1920;
    chn_attr.height = 1080;

    // ret = HB_VPS_SetChnAttr(pipeId, 1, &chn_attr);
    // if (ret) {
    //     printf("HB_VPS_SetChnAttr error!!!\n");
    // } else {
    //     printf("set chn Attr ok: GrpId = %d, chn_id = %d\n",
    //                             pipeId, 1);
    // }
    // HB_VPS_EnableChn(pipeId, 1);

    // chn_attr.width = 1920;
    // chn_attr.height = 1080;

    // ret = HB_VPS_SetChnAttr(pipeId, 2, &chn_attr);
    // if (ret) {
    //     printf("HB_VPS_SetChnAttr error!!!\n");
    // } else {
    //     printf("set chn Attr ok: GrpId = %d, chn_id = %d\n",
    //                             pipeId, 2);
    // }
    // HB_VPS_EnableChn(pipeId, 2);

    // chn_attr.width = 704;
    // chn_attr.height = 576;

    // ret = HB_VPS_SetChnAttr(pipeId, 3, &chn_attr);
    // if (ret) {
    //     printf("HB_VPS_SetChnAttr error!!!\n");
    // } else {
    //     printf("set chn Attr ok: GrpId = %d, chn_id = %d\n",
    //                             pipeId, 3);
    // }
    // HB_VPS_EnableChn(pipeId, 3);

    // ret = HB_VPS_SetChnAttr(pipeId, 6, &chn_attr);
    // if (ret) {
    //     printf("HB_VPS_SetChnAttr error!!!\n");
    // } else {
    //     printf("set chn Attr ok: GrpId = %d, chn_id = %d\n",
    //                             pipeId, 6);
    // }

    // memset(&pym_chn_attr, 0, sizeof(VPS_PYM_CHN_ATTR_S));
    // pym_chn_attr.timeout = 2000;
    // pym_chn_attr.ds_layer_en = 4;
    // pym_chn_attr.us_layer_en = 0;
    // pym_chn_attr.frame_id = 0;
    // pym_chn_attr.frameDepth = 1;
    
    // // pym_chn_attr.ds_info[1].factor = 32,
    // // pym_chn_attr.ds_info[1].roi_x = 0,
    // // pym_chn_attr.ds_info[1].roi_y = 0,
    // // pym_chn_attr.ds_info[1].roi_width = 1920,
    // // pym_chn_attr.ds_info[1].roi_height = 1080,

    // ret = HB_VPS_SetPymChnAttr(pipeId, 6, &pym_chn_attr);
    // if (ret) {
    //     printf("HB_VPS_SetPymChnAttr error!!!\n");
    // } else {
    //     printf("HB_VPS_SetPymChnAttr ok: grp_id = %d g_pym_chn = %d\n",
    //                             pipeId, 6);
    // }
    // HB_VPS_EnableChn(pipeId, 6);

    ret = HB_VPS_StartGrp(pipeId);
	if (ret) {
			printf("HB_VPS_StartGrp %d error!!!\n", pipeId);
			return ret;
	} else {
			printf("start grp ok: grp_id = %d\n", pipeId);
	}

	return ret;
}

int sample_singlepipe_5venc()
{
    int ret, i, venctype;
    // int rgnenable = 0;
    // pthread_t vens_thid[6];
    // pthread_t rgn_thid[6];
    // // pthread_t vdec_thid;
    // vencParam vencparam[6]={0};
    // rgnParam  rgnparam[6]={0};

    memset(vencparam, 0, sizeof(vencparam));

    start_time = time(NULL);
	end_time = start_time + run_time;

    venctype = g_venc_flag;
    if (venctype > 4) venctype = 1;

    if (g_venc_flag > 100000)
    {
        venctype = g_venc_flag/10000%10;
        if (venctype > 4) venctype = 1;
    }

    vencparam[0].veChn = 4;
    vencparam[0].type = venctype;
    vencparam[0].width = 704;
    vencparam[0].height = 576;
    vencparam[0].vpsGrp = 0;
    vencparam[0].vpsChn = 0;
    vencparam[0].bitrate = 768;
    vencparam[0].quit = 0;

    if (g_venc_flag > 100000) {
        venctype = g_venc_flag%10;
        if (venctype > 4) venctype = 1;
    }
    vencparam[1].veChn = 0;
    vencparam[1].type = venctype;
    vencparam[1].width = 2688;
    vencparam[1].height = 1520;
    vencparam[1].vpsGrp = 0;
    vencparam[1].vpsChn = 5;
    vencparam[1].bitrate = 8000;
    vencparam[1].quit = 0;

    if (g_venc_flag > 100000) {
        venctype = g_venc_flag/100%10;
        if (venctype > 4) venctype = 1;
    }
    vencparam[2].veChn = 2;
    vencparam[2].type = venctype;
    vencparam[2].width = 1920;
    vencparam[2].height = 1080;
    vencparam[2].vpsGrp = 0;
    vencparam[2].vpsChn = 1;
    vencparam[2].bitrate = 4000;
    vencparam[2].quit = 0;

    if (g_venc_flag > 100000) {
        venctype = g_venc_flag/10%10;
        if (venctype > 4) venctype = 1;
    }
    vencparam[3].veChn = 1;
    vencparam[3].type = venctype;
    vencparam[3].width = 1920;
    vencparam[3].height = 1080;
    vencparam[3].vpsGrp = 0;
    vencparam[3].vpsChn = 3;
    vencparam[3].bitrate = 4000;
    vencparam[3].quit = 0;

    if (g_venc_flag > 100000) {
        venctype = g_venc_flag/1000%10;
        if (venctype > 4) venctype = 1;
    }
    vencparam[4].veChn = 3;
    vencparam[4].type = venctype;
    vencparam[4].width = 704;
    vencparam[4].height = 576;
    vencparam[4].vpsGrp = 0;
    vencparam[4].vpsChn = 4;
    vencparam[4].bitrate = 768;
    vencparam[4].quit = 0;

    if ((sensorId[0] == OS8A10_25FPS_3840P_RAW10_LINEAR) ||
        (sensorId[0] == OS8A10_25FPS_3840P_RAW10_DOL2) ||
        (sensorId[0] == SIF_TEST_PATTERN0_2160P)) {
        vencparam[1].width = 3840;
        vencparam[1].height = 2160;
    } else if ((sensorId[0] == SIF_TEST_PATTERN_12M_RAW12) ||
        (sensorId[0] == S5KGM1SP_30FPS_4000x3000_RAW10)) {
        vencparam[1].width = 4000;
        vencparam[1].height = 3000;
    }

    // vencparam[5].veChn = 5;
    // vencparam[5].type = 0;
    // vencparam[5].width = 720;
    // vencparam[5].height = 1280;
    // vencparam[5].vpsGrp = 0;
    // vencparam[5].vpsChn = 0;
    // vencparam[5].bitrate = 2000;
    if (g_venc_flag > 0) {
        ret = sample_venc_common_init();
        if (ret < 0) {
            printf("sample_venc_common_init error!\n");
            return ret;
        }

        for (i=0; i < 5; i++) {
            printf("vencparam[%d].type: %x\n", i, vencparam[i].type);
            if (vencparam[i].type == 0) continue;
            ret = sample_venc_init(vencparam[i].veChn, vencparam[i].type,
                vencparam[i].width, vencparam[i].height, vencparam[i].bitrate);
            if (ret < 0) {
                printf("sample_venc_init error!\n");
                goto err;
            }

            ret = sample_venc_start(vencparam[i].veChn);
            if (ret < 0) {
                printf("sample_start error!\n");
                goto err;
            }
        }
    }
    // sample_set_vencfps(0, 30, 12);
    printf("=============================================\n");
    printf("sensorId[0]: %d, bus[0]: %d, port[0]: %d, mipiIdx[0]: %d,"
            "serdes_index[0]: %d, serdes_port[0]: %d\n", sensorId[0], bus[0],
            port[0], mipiIdx[0], serdes_index[0], serdes_port[0]);
    ret = hb_vin_vps_init(0, sensorId[0], mipiIdx[0],  serdes_port[0], vencparam);
	if (ret < 0) {
        hb_vin_vps_deinit(0, sensorId[0]);
		printf("hb_vin_vps_init error!\n");
		return ret;
	}

    if ((sensorId[0] < SIF_TEST_PATTERN0_1080P)
            || (sensorId[0] > SIF_TEST_PATTERN0_2160P)) {
        if ((sensorId[0] == OS8A10_25FPS_3840P_RAW10_LINEAR) ||
            (sensorId[0] == OS8A10_25FPS_3840P_RAW10_DOL2)) {
            if (g_use_x3clock == 1) {
                printf("========HB_MIPI_EnableSensorClock==============\n");
                HB_MIPI_EnableSensorClock(mipiIdx[0]);
            }
		}
        ret = hb_sensor_init(0, sensorId[0], bus[0], port[0], mipiIdx[0],
                serdes_index[0], serdes_port[0]);
        if (ret < 0) {
            printf("hb_sensor_init error! do vio deinit\n");
            hb_vin_vps_deinit(0, sensorId[0]);
            return ret;
        }

        ret = hb_mipi_init(sensorId[0], mipiIdx[0]);
        if (ret < 0) {
            printf("hb_mipi_init error! do vio deinit\n");
            hb_vin_vps_deinit(0, sensorId[0]);
            return ret;
        }
    } else {  // testpattern
        printf("=================================\n");
        ret = hb_sensor_init(0, sensorId[0], bus[0], port[0], mipiIdx[0],
                serdes_index[0], serdes_port[0]);
        if (ret < 0) {
            printf("hb_sensor_init error! do vio deinit\n");
            hb_vin_vps_deinit(0, sensorId[0]);
            return ret;
        }
        ret = HB_MIPI_SetSensorFps(0, g_testpattern_fps);
        if (ret < 0) {
            printf("HB_MIPI_SetSensorFps error! do vio deinit\n");
            return ret;
        }
	}

	ret = hb_vin_vps_start(0);
	if (ret < 0) {
		printf("hb_vin_vps_start error! do cam && vio deinit\n");
		hb_sensor_deinit(sensorId[0]);
        hb_mipi_deinit(mipiIdx[0]);
        hb_vin_vps_stop(0);
		hb_vin_vps_deinit(0, sensorId[0]);
		return ret;
	}
    printf("hb_vin_vps_start ok\n");
    if ((sensorId[0] < SIF_TEST_PATTERN0_1080P)
            || (sensorId[0] > SIF_TEST_PATTERN0_2160P)) {
        ret = hb_sensor_start(0);
        if (ret < 0) {
            printf("hb_mipi_start error! do cam && vio deinit\n");
            hb_sensor_stop(0);
            hb_mipi_stop(mipiIdx[0]);
            hb_sensor_deinit(sensorId[0]);
            hb_mipi_deinit(mipiIdx[0]);
            hb_vin_vps_stop(0);
            hb_vin_vps_deinit(0, sensorId[0]);
            return ret;
        }

        ret = hb_mipi_start(mipiIdx[0]);
        if (ret < 0) {
            printf("hb_mipi_start error! do cam && vio deinit\n");
            hb_mipi_stop(mipiIdx[0]);
            hb_mipi_deinit(mipiIdx[0]);
            hb_vin_vps_deinit(0, sensorId[0]);
            return ret;
        }
    }
    printf("hb_mipi_start ok\n");

    if (need_chn_rotate) {
        ret = hb_vps1_init(1);
        if (ret < 0) {
            printf("hb_vps1_init failed\n");
        } else {
            printf("hb_vps1_init ok\n");
        }
        ret = sample_vps_bind_vps(0, 2, 1);
        if (ret < 0) {
            printf("sample_vps_bind_vps failed\n");
        } else {
            printf("sample_vps_bind_vps ok\n");
        }
    }

    if (g_use_ipu == 1) {
        if (vencparam[0].type > 0) {
            if (g_bindflag == 1) {
                ret = sample_vps_bind_venc(vencparam[0].vpsGrp,
                        vencparam[0].vpsChn, vencparam[0].veChn);
                printf("sample_vps %d bind_venc %d\n",
                    vencparam[0].vpsChn, vencparam[0].veChn);
            }
        }
        if (vencparam[1].type > 0) {
            if (g_bindflag == 1) {
                ret = sample_vps_bind_venc(vencparam[1].vpsGrp,
                        vencparam[1].vpsChn, vencparam[1].veChn);
                printf("sample_vps %d bind venc %d\n",
                    vencparam[1].vpsChn, vencparam[1].veChn);
            }
        }
        if (vencparam[2].type > 0) {
            // vencparam[2].vpsGrp = 1;
            if (g_bindflag == 1) {
                ret = sample_vps_bind_venc(vencparam[2].vpsGrp,
                        vencparam[2].vpsChn, vencparam[2].veChn);
                printf("sample_vps %d bind venc %d\n",
                    vencparam[2].vpsChn, vencparam[2].veChn);
            }
        }
        if (vencparam[3].type > 0) {
            if (g_bindflag == 1) {
                ret = sample_vps_bind_venc(vencparam[3].vpsGrp,
                        vencparam[3].vpsChn, vencparam[3].veChn);
                printf("sample_vps %d bind venc %d\n",
                    vencparam[3].vpsChn, vencparam[3].veChn);
            }
        }
        if (vencparam[4].type > 0) {
            if (g_bindflag == 1) {
                ret = sample_vps_bind_venc(vencparam[4].vpsGrp,
                        vencparam[4].vpsChn, vencparam[4].veChn);
                printf("sample_vps %d bind venc %d\n",
                    vencparam[4].vpsChn, vencparam[4].veChn);
            }
        }
    }

    if (g_iar_enable) {
        ret = sample_vot_init();
        if (ret) {
            printf("sample_vot_init failed\n");
            return ret;
        }
        ret = sample_vps_bind_vot(0, 2, 0);
        printf("sample_vps_bind_vot: %d\n", ret);
    }

    for (int i = 0; i < 6; i++) {
        rgnparam[i].rgnChn = i;
        rgnparam[i].x = 50;
        rgnparam[i].y = 50;
        rgnparam[i].w = 640;
        rgnparam[i].h = 100;
    }

    // do_sync_decoding(NULL);

    rgnenable = g_osd_flag;
    if (g_use_ipu == 0) rgnenable = 0;
    if (rgnenable & 0x1) {
        ret = pthread_create(&rgn_thid[0], NULL,
            (void *)sample_rgn_thread, &(rgnparam[0]));
    }
    if (rgnenable & 0x2) {
        ret = pthread_create(&rgn_thid[1], NULL,
		    (void *)sample_rgn_thread, &(rgnparam[1]));
    }
    if (rgnenable & 0x4) {
        ret = pthread_create(&rgn_thid[2], NULL,
		    (void *)sample_rgn_thread, &(rgnparam[2]));
    }
    if (rgnenable & 0x8) {
        ret = pthread_create(&rgn_thid[3], NULL,
		    (void *)sample_rgn_thread, &(rgnparam[3]));
    }
    if (rgnenable & 0x10) {
        ret = pthread_create(&rgn_thid[4], NULL,
		    (void *)sample_rgn_thread, &(rgnparam[4]));
    }
    if (rgnenable & 0x20) {
        ret = pthread_create(&rgn_thid[5], NULL,
		    (void *)sample_rgn_thread, &(rgnparam[5]));
    }

    if (g_use_ipu == 1) {
        if (vencparam[0].type > 0) {
            ret = pthread_create(&vens_thid[0], NULL,
                (void *)venc_thread, &(vencparam[0]));
        }
        if (vencparam[1].type > 0) {
            ret = pthread_create(&vens_thid[1], NULL,
                (void *)venc_thread, &(vencparam[1]));
        }
        if (vencparam[2].type > 0) {
            ret = pthread_create(&vens_thid[2], NULL,
                (void *)venc_thread, &(vencparam[2]));
        }
        if (vencparam[3].type > 0) {
            ret = pthread_create(&vens_thid[3], NULL,
                (void *)venc_thread, &(vencparam[3]));
        }
        if (vencparam[4].type > 0) {
            ret = pthread_create(&vens_thid[4], NULL,
                (void *)venc_thread, &(vencparam[4]));
        }
    } else {
        ret = pthread_create(&vens_thid[5], NULL, (void *)pym_get_thread, NULL);
        if (vencparam[0].type > 0) {
            ret = pthread_create(&vens_thid[0], NULL,
                (void *)venc_from_pym_thread, &(vencparam[0]));
        }
        if (vencparam[1].type > 0) {
            ret = pthread_create(&vens_thid[1], NULL,
                (void *)venc_from_pym_thread, &(vencparam[1]));
        }
        if (vencparam[2].type > 0) {
            ret = pthread_create(&vens_thid[2], NULL,
                (void *)venc_from_pym_thread, &(vencparam[2]));
        }
        if (vencparam[3].type > 0) {
            ret = pthread_create(&vens_thid[3], NULL,
                (void *)venc_from_pym_thread, &(vencparam[3]));
        }
        if (vencparam[4].type > 0) {
            ret = pthread_create(&vens_thid[4], NULL,
                (void *)venc_from_pym_thread, &(vencparam[4]));
        }
    }

    // if (g_vps_frame_depth > 0) {
    //     ret = pthread_create(&vens_thid[5], NULL,
    //         (void *)test_getvpschan_thread, &(vpsparam[2]));
    // //     // ret = pthread_create(&vens_thid[5], NULL,
    // //     //     (void *)test_getvinchan_thread, &(vpsparam[2]));
    // }

    return 0;

err:
    // ret = sample_vps_deinit(0);
    // sample_vot_wb_deinit();
    // sample_vdec_deinit(vdecchn);
    if (vencparam[0].type > 0) {
         sample_venc_stop(vencparam[0].veChn);
        sample_venc_deinit(vencparam[0].veChn);
    }
    if (vencparam[1].type > 0) {
        sample_venc_stop(vencparam[1].veChn);
        sample_venc_deinit(vencparam[1].veChn);
    }
    if (vencparam[2].type > 0) {
        sample_venc_stop(vencparam[2].veChn);
        sample_venc_deinit(vencparam[2].veChn);
    }
    if (vencparam[3].type > 0) {
        sample_venc_stop(vencparam[3].veChn);
        sample_venc_deinit(vencparam[3].veChn);
    }
    if (vencparam[4].type > 0) {
        sample_venc_stop(vencparam[4].veChn);
        sample_venc_deinit(vencparam[4].veChn);
    }
    // if (vencparam[5].type > 0) {
    //     sample_venc_stop(5);
    //     sample_venc_deinit(5);
    // }

    if (g_venc_flag > 0)
        sample_venc_common_deinit();

    if (g_iar_enable)
        sample_vot_deinit();

    // hb_vps_stop(1);
    // hb_vps_deinit(1);
    if ((sensorId[0] < SIF_TEST_PATTERN0_1080P)
            || (sensorId[0] > SIF_TEST_PATTERN0_2160P)) {
        hb_sensor_stop(0);
        hb_mipi_stop(mipiIdx[0]);
    }
	hb_vin_vps_stop(0);
	// hb_vin_sif_isp_stop(groupId);
    if ((sensorId[0] < SIF_TEST_PATTERN0_1080P)
            || (sensorId[0] > SIF_TEST_PATTERN0_2160P)) {
        hb_mipi_deinit(mipiIdx[0]);
        hb_sensor_deinit(0);
        if ((sensorId[0] == OS8A10_25FPS_3840P_RAW10_LINEAR) ||
            (sensorId[0] == OS8A10_25FPS_3840P_RAW10_DOL2)) {
            if (g_use_x3clock == 1)
			    HB_MIPI_DisableSensorClock(mipiIdx[0]);
		}
    }
	hb_vin_vps_deinit(0, sensorId[0]);

    return -1;
}

int sample_switch_dol2_linear()
{
    int ret;
    sample_vin_unbind_vps(0, vin_vps_mode);
    if ((sensorId[0] < SIF_TEST_PATTERN0_1080P)
            || (sensorId[0] > SIF_TEST_PATTERN0_2160P)) {
        hb_sensor_stop(0);
        hb_mipi_stop(mipiIdx[0]);
        hb_mipi_deinit(mipiIdx[0]);
        hb_sensor_deinit(0);
    }
    hb_mode_stop(0);
    // sample_vin_unbind_vps(0, vin_vps_mode);
    hb_mode_destroy(0);
    
    if (sensorId[0] == OS8A10_25FPS_3840P_RAW10_DOL2) {
        sensorId[0] = OS8A10_25FPS_3840P_RAW10_LINEAR;
        printf("==========switch from OS8A10_25FPS_3840P_RAW10_DOL2 to OS8A10_25FPS_3840P_RAW10_LINEAR\n");
    } else if (sensorId[0] == OS8A10_25FPS_3840P_RAW10_LINEAR) {
        sensorId[0] = OS8A10_25FPS_3840P_RAW10_DOL2;
        printf("==========switch from  OS8A10_25FPS_3840P_RAW10_LINEAR to OS8A10_25FPS_3840P_RAW10_DOL2\n");
    }
    hb_mode_init(0, sensorId[0], mipiIdx[0],  serdes_port[0], vin_vps_mode);
    if ((sensorId[0] < SIF_TEST_PATTERN0_1080P)
            || (sensorId[0] > SIF_TEST_PATTERN0_2160P)) {
        if ((sensorId[0] == OS8A10_25FPS_3840P_RAW10_LINEAR) ||
            (sensorId[0] == OS8A10_25FPS_3840P_RAW10_DOL2)) {
            if (g_use_x3clock == 1) {
                printf("========HB_MIPI_EnableSensorClock==============\n");
                HB_MIPI_EnableSensorClock(mipiIdx[0]);
            }
		}
        ret = hb_sensor_init(0, sensorId[0], bus[0], port[0], mipiIdx[0],
                serdes_index[0], serdes_port[0]);
        if (ret < 0) {
            printf("hb_sensor_init error! do vio deinit\n");
            // hb_vin_vps_deinit(0, sensorId[0]);
            return ret;
        }

        ret = hb_mipi_init(sensorId[0], mipiIdx[0]);
        if (ret < 0) {
            printf("hb_mipi_init error! do vio deinit\n");
            // hb_vin_vps_deinit(0, sensorId[0]);
            return ret;
        }
    }

    hb_mode_start(0);
    hb_sensor_start(0);
    hb_mipi_start(mipiIdx[0]);

    return 0;
}

int sample_singlepipe_5venc_deinit()
{
    int ret = 0;

    if (g_use_ipu) {
        if (vencparam[0].type > 0) {
            if (g_bindflag == 1)
                ret = sample_vps_unbind_venc(vencparam[0].vpsGrp,
                        vencparam[0].vpsChn, vencparam[0].veChn);
        }
        if (vencparam[1].type > 0) {
            if (g_bindflag == 1)
                ret = sample_vps_unbind_venc(vencparam[1].vpsGrp,
                        vencparam[1].vpsChn, vencparam[1].veChn);
        }
        if (vencparam[2].type > 0) {
            if (g_bindflag == 1)
                ret = sample_vps_unbind_venc(vencparam[2].vpsGrp,
                        vencparam[2].vpsChn, vencparam[2].veChn);
        }
        if (vencparam[3].type > 0) {
            if (g_bindflag == 1)
                ret = sample_vps_unbind_venc(vencparam[3].vpsGrp,
                        vencparam[3].vpsChn, vencparam[3].veChn);
        }
        if (vencparam[4].type > 0) {
            if (g_bindflag == 1)
                ret = sample_vps_unbind_venc(vencparam[4].vpsGrp,
                        vencparam[4].vpsChn, vencparam[4].veChn);
        }
    }

    ret = sample_vps_unbind_vot(0, 2, 0);
    // ret = sample_vps_unbind_vot(0, 6, 1);
    // if (g_vps_frame_depth > 0) ret = pthread_join(vens_thid[5], NULL);
    if (vencparam[0].type > 0) ret = pthread_join(vens_thid[0], NULL);
    if (vencparam[1].type > 0) ret = pthread_join(vens_thid[1], NULL);
    if (vencparam[2].type > 0) ret = pthread_join(vens_thid[2], NULL);
    if (vencparam[3].type > 0) ret = pthread_join(vens_thid[3], NULL);
    if (vencparam[4].type > 0) ret = pthread_join(vens_thid[4], NULL);

    // if (vencparam[5].type > 0) ret = pthread_join(vens_thid[5], NULL);
    if (rgnenable & 0x1) ret = pthread_join(rgn_thid[0], NULL);
    if (rgnenable & 0x2) ret = pthread_join(rgn_thid[1], NULL);
    if (rgnenable & 0x4) ret = pthread_join(rgn_thid[2], NULL);
    if (rgnenable & 0x8) ret = pthread_join(rgn_thid[3], NULL);
    if (rgnenable & 0x10) ret = pthread_join(rgn_thid[4], NULL);
    if (rgnenable & 0x20) ret = pthread_join(rgn_thid[5], NULL);

    // ret = pthread_create(&vdec_thid, NULL, (void *)do_sync_decoding, NULL);
    // FILE *outFile;
    // VIDEO_STREAM_S pstStream;
    // memset(&pstStream, 0, sizeof(VIDEO_STREAM_S));

    // outFile = fopen("./vencxx_stream.h264", "wb");
    // while (g_exit == 0) {
    //     // sleep(1);
    //     ret = HB_VENC_GetStream(0, &pstStream, 3000);
    //     if (ret < 0) {
	// 		printf("HB_VENC_GetStream error!!!\n");
    //         //break;
	// 	} else {
    //         fwrite(pstStream.pstPack.vir_ptr,
    //             pstStream.pstPack.size, 1, outFile);
    //      // printf("pstStream.pstPack.size = %d\n", pstStream.pstPack.size);
    //         HB_VENC_ReleaseStream(0, &pstStream);
    //     }
    // }
    // fclose(outFile);

    // ret = sample_vps_deinit(0);
    // sample_vot_wb_deinit();
    // sample_vdec_deinit(vdecchn);
    if (vencparam[0].type > 0) {
         sample_venc_stop(vencparam[0].veChn);
        sample_venc_deinit(vencparam[0].veChn);
    }
    if (vencparam[1].type > 0) {
        sample_venc_stop(vencparam[1].veChn);
        sample_venc_deinit(vencparam[1].veChn);
    }
    if (vencparam[2].type > 0) {
        sample_venc_stop(vencparam[2].veChn);
        sample_venc_deinit(vencparam[2].veChn);
    }
    if (vencparam[3].type > 0) {
        sample_venc_stop(vencparam[3].veChn);
        sample_venc_deinit(vencparam[3].veChn);
    }
    if (vencparam[4].type > 0) {
        sample_venc_stop(vencparam[4].veChn);
        sample_venc_deinit(vencparam[4].veChn);
    }

    if (g_venc_flag > 0)
        sample_venc_common_deinit();

    if (g_iar_enable)
        sample_vot_deinit();

    // hb_vps_stop(1);
    // hb_vps_deinit(1);
    if ((sensorId[0] < SIF_TEST_PATTERN0_1080P)
            || (sensorId[0] > SIF_TEST_PATTERN0_2160P)) {
        hb_sensor_stop(0);
        hb_mipi_stop(mipiIdx[0]);
    }
	hb_vin_vps_stop(0);
	// hb_vin_sif_isp_stop(groupId);
    if ((sensorId[0] < SIF_TEST_PATTERN0_1080P)
            || (sensorId[0] > SIF_TEST_PATTERN0_2160P)) {
        hb_mipi_deinit(mipiIdx[0]);
        hb_sensor_deinit(0);
        if ((sensorId[0] == OS8A10_25FPS_3840P_RAW10_LINEAR) ||
            (sensorId[0] == OS8A10_25FPS_3840P_RAW10_DOL2)) {
            if (g_use_x3clock == 1)
			    HB_MIPI_DisableSensorClock(mipiIdx[0]);
		}
    }
	hb_vin_vps_deinit(0, sensorId[0]);

    // HB_VDEC_Module_Uninit();

    return ret;
}

int prepare_user_buf(void *buf, uint32_t size_y, uint32_t size_uv)
{
	int ret;
	hb_vio_buffer_t *buffer = (hb_vio_buffer_t *)buf;

	if (buffer == NULL)
		return -1;

	buffer->img_info.fd[0] = ion_open();
	buffer->img_info.fd[1] = ion_open();
    printf("buffer->img_info.fd[0]: %d, buffer->img_info.fd[1]: %d\n",
        buffer->img_info.fd[0], buffer->img_info.fd[1]);
	ret  = ion_alloc_phy(size_y, &buffer->img_info.fd[0],
						&buffer->img_addr.addr[0], &buffer->img_addr.paddr[0]);
	if (ret) {
		printf("ion_alloc_phy %d error\n", size_y);
		return ret;
	}
	ret = ion_alloc_phy(size_uv, &buffer->img_info.fd[1],
						&buffer->img_addr.addr[1], &buffer->img_addr.paddr[1]);
	if (ret) {
		printf("ion_alloc_phy %d error\n", size_uv);
		return ret;
	}

	printf("buf:y: vaddr = 0x%p paddr = 0x%lx; uv: vaddr = 0x%p, paddr = 0x%lx\n",
				buffer->img_addr.addr[0], buffer->img_addr.paddr[0],
				buffer->img_addr.addr[1], buffer->img_addr.paddr[1]);
	return 0;
}

int sample_ipu_feedback(char *yuvname)
{
    int ret;
	int img_in_fd;
    pym_buffer_t out_pym_buf;
    hb_vio_buffer_t out_buf;
    hb_vio_buffer_t feedback_buf;
    int size_y, size_uv;
    int w = 3840;
    int h = 2160;

    ret = hb_vps_init(0);
    if (ret != 0) {
        printf("hb_vps_init failed\n");
        return -1;
    }

    ret = hb_vps_start(0);
    if (ret != 0) {
        printf("hb_vps_start failed\n");
        return -1;
    }

    memset(&feedback_buf, 0, sizeof(hb_vio_buffer_t));
	size_y = w * h;
	size_uv = size_y / 2;
	prepare_user_buf(&feedback_buf, size_y, size_uv);
	feedback_buf.img_info.planeCount = 2;
	feedback_buf.img_info.img_format = 8;
	feedback_buf.img_addr.width = w;
	feedback_buf.img_addr.height = h;
	feedback_buf.img_addr.stride_size = w;

	img_in_fd = open(yuvname, O_RDWR | O_CREAT, 0644);
	if (img_in_fd < 0) {
		printf("open file %s failed !\n", yuvname);
		return 0;
	}

	read(img_in_fd, feedback_buf.img_addr.addr[0], size_y);
	usleep(10 * 1000);
	read(img_in_fd, feedback_buf.img_addr.addr[1], size_uv);
	usleep(10 * 1000);
	close(img_in_fd);
	/*dumpToFile2plane("in_image.yuv", feedback_buf.img_addr.addr[0],
						feedback_buf.img_addr.addr[1],
						size_y, size_uv);*/

	while(g_exit == 0) {
		ret = HB_VPS_SendFrame(0, &feedback_buf, 1000);
        if (ret != 0) {
            printf("HB_VPS_SendFrame error!!!\n");
            break;
        }

	/* coverity[overrun-buffer-val] */
	ret = HB_VPS_GetChnFrame(0, 0, &out_buf, 2000);
        if (ret != 0) {
            printf("HB_VPS_GetChnFrame error!!!\n");
            break;
        }

        if (1) {
            char fname[32];
            sprintf(fname, "vps_%d_%d_%d.yuv", 0, 0, 0);
            printf("start dump %s\n", fname);
            FILE *fw = fopen(fname, "wb");
            if (fw != NULL) {
                fwrite(out_buf.img_addr.addr[0], out_buf.img_addr.width * out_buf.img_addr.height, 1, fw);
                fwrite(out_buf.img_addr.addr[1], out_buf.img_addr.width * out_buf.img_addr.height / 2, 1, fw);
                fclose(fw);
            }
        }

        ret = HB_VPS_ReleaseChnFrame(0, 0, &out_buf);
        if (ret) {
            printf("HB_VPS_ReleaseChnFrame error!!!\n");
        }

        ret = HB_VPS_GetChnFrame(0, 6, &out_pym_buf, 2000);
        if (ret != 0) {
            printf("HB_VPS_GetChnFrame error!!!\n");
            break;
        }

        if (1) {
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
                    sprintf(fname, "pym_%d_%d.yuv", layer/4, 0);
                } else {
                    sprintf(fname, "pymroi_%d_%d_%d.yuv",
                            layer/4, roi-1, 0);
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
        }
        ret = HB_VPS_ReleaseChnFrame(0, 6, &out_pym_buf);
        if (ret) {
            printf("HB_VPS_ReleaseChnFrame error!!!\n");
            break;
        }
        sleep(1);
        break;
    }

    hb_vps_stop(0);
    hb_vps_deinit(0);

    return 0;
}
