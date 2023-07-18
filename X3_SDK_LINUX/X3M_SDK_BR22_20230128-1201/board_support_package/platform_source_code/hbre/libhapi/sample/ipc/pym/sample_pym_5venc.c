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
    // pthread_t vdec_thid;
static vencParam vencparam[6]={0};
static rgnParam  rgnparam[6]={0};
static int rgnenable = 0;
// int sif_raw_dump_func()
// {
// 	static int loop_count = 0;
// 	struct timeval time_now = { 0 };
// 	struct timeval time_next = { 0 };
// 	int size = -1;
// 	char file_name[100] = { 0 };
// 	int ret = 0;
// 	hb_vio_buffer_t *sif_raw = NULL;
// 	int pipeId = 0;

// 	sif_raw = (hb_vio_buffer_t *) malloc(sizeof(hb_vio_buffer_t));
// 	memset(sif_raw, 0, sizeof(hb_vio_buffer_t));

// 	printf("sif_raw_dump_func begin===\n");
// 	ret = HB_VIN_DumpDevRaw(pipeId, sif_raw, -1);
//     printf("sif_raw_dump_func end %d===\n", ret);
// 	if (ret < 0) {
// 		printf("HB_VIN_DumpDevRaw error!!!\n");
// 	} else {
// 		if(1) {
// 			// normal_buf_info_print(sif_raw);
// 			size = sif_raw->img_addr.stride_size * sif_raw->img_addr.height;
// 			printf("raw stride_size(%u) w x h%u x %u  size %d\n",
// 				sif_raw->img_addr.stride_size,
// 				sif_raw->img_addr.width, sif_raw->img_addr.height, size);
// 			snprintf(file_name, sizeof(file_name),
// 					"sif_raw%d.raw", loop_count);

// 		   gettimeofday(&time_now, NULL);
// 		   dumpToFile(file_name, sif_raw->img_addr.addr[0], size);
// 		   gettimeofday(&time_next, NULL);
// 		   int time_cost = time_cost_ms(&time_now, &time_next);
// 		   printf("dumpToFile raw cost time %d ms", time_cost);
// 		   loop_count++;
// 		}
// 	}
// 	free(sif_raw);
// 	sif_raw = NULL;
// }

// int sample_singlepipe_reconfig()
// {
//     int i = 0;
//     int ret;

//     for (i=0; i<5; i++) {
//         if (g_venc_flag == 0 ) break;
//         if (g_bindflag == 1)
//             ret = sample_vps_unbind_venc(vencparam[i].vpsGrp,
//                     vencparam[i].vpsChn, vencparam[i].veChn);
//         vencparam[i].quit = 1;
//     }

//     for (i=0; i<5; i++) {
//         if (vencparam[i].type > 0) ret = pthread_join(vens_thid[i], NULL);
//     }
//     printf("=================sample_vin_unbind_vps\n");
//     ret = sample_vin_unbind_vps(0, vin_vps_mode);
//     if (ret != 0) {
//         printf("sample_vin_unbind_vps failed\n");
//         return ret;
//     }

//     if ((sensorId[0] < SIF_TEST_PATTERN0_1080P)
//             || (sensorId[0] > SIF_TEST_PATTERN0_2160P)) {
//         hb_sensor_stop(0);
//         hb_mipi_stop(mipiIdx[0]);
//     }
// 	hb_vin_vps_stop(0);
// 	// hb_vin_sif_isp_stop(groupId);
//     if ((sensorId[0] < SIF_TEST_PATTERN0_1080P)
//             || (sensorId[0] > SIF_TEST_PATTERN0_2160P)) {
//         hb_mipi_deinit(mipiIdx[0]);
//         hb_sensor_deinit(0);
//         if ((sensorId[0] == OS8A10_25FPS_3840P_RAW10_LINEAR) ||
//             (sensorId[0] == OS8A10_25FPS_3840P_RAW10_DOL2)) {
// 			HB_MIPI_DisableSensorClock(mipiIdx[0]);
// 		}
//     }
// 	hb_vin_vps_deinit(0, sensorId[0]);

//     vencparam[2].width = 1280;
//     vencparam[2].height = 720;

//     ret = hb_vin_vps_init(0, sensorId[0], mipiIdx[0],  serdes_port[0], vencparam);
    
// 	if (ret < 0) {
// 		printf("hb_vin_vps_init error!\n");
// 		return ret;
// 	}

//     if ((sensorId[0] < SIF_TEST_PATTERN0_1080P)
//             || (sensorId[0] > SIF_TEST_PATTERN0_2160P)) {
//         if ((sensorId[0] == OS8A10_25FPS_3840P_RAW10_LINEAR) ||
//             (sensorId[0] == OS8A10_25FPS_3840P_RAW10_DOL2)) {
// 			HB_MIPI_EnableSensorClock(mipiIdx[0]);
// 		}
//         ret = hb_sensor_init(0, sensorId[0], bus[0], port[0], mipiIdx[0],
//                 serdes_index[0], serdes_port[0]);
//         if (ret < 0) {
//             printf("hb_sensor_init error! do vio deinit\n");
//             hb_vin_vps_deinit(0, sensorId[0]);
//             return ret;
//         }

//         ret = hb_mipi_init(sensorId[0], mipiIdx[0]);
//         if (ret < 0) {
//             printf("hb_mipi_init error! do vio deinit\n");
//             hb_vin_vps_deinit(0, sensorId[0]);
//             return ret;
//         }
//     }

// 	ret = hb_vin_vps_start(0);
// 	if (ret < 0) {
// 		printf("hb_vin_vps_start error! do cam && vio deinit\n");
// 		hb_sensor_deinit(sensorId[0]);
//         hb_mipi_deinit(mipiIdx[0]);
//         hb_vin_vps_stop(0);
// 		hb_vin_vps_deinit(0, sensorId[0]);
// 		return ret;
// 	}

//     if ((sensorId[0] < SIF_TEST_PATTERN0_1080P)
//             || (sensorId[0] > SIF_TEST_PATTERN0_2160P)) {
//         ret = hb_sensor_start(0);
//         if (ret < 0) {
//             printf("hb_mipi_start error! do cam && vio deinit\n");
//             hb_sensor_stop(0);
//             hb_mipi_stop(mipiIdx[0]);
//             hb_sensor_deinit(sensorId[0]);
//             hb_mipi_deinit(mipiIdx[0]);
//             hb_vin_vps_stop(0);
//             hb_vin_vps_deinit(0, sensorId[0]);
//             return ret;
//         }

//         ret = hb_mipi_start(mipiIdx[0]);
//         if (ret < 0) {
//             printf("hb_mipi_start error! do cam && vio deinit\n");
//             hb_mipi_stop(mipiIdx[0]);
//             hb_mipi_deinit(mipiIdx[0]);
//             hb_vin_vps_deinit(0, sensorId[0]);
//             return ret;
//         }
//     }

//     // printf("=================hb_vps_stop start\n");
//     // hb_vps_stop(0);
//     // printf("=================hb_vps_deinit start\n");
//     // hb_vps_deinit(0);
//     // printf("=================hb_vps_deinit done\n");
//     // //
//     // // vencparam[2].veChn = 2;
//     // // vencparam[2].type = 1;
//     // vencparam[2].width = 1280;
//     // vencparam[2].height = 720;
//     // // vencparam[2].vpsGrp = 0;
//     // // vencparam[2].vpsChn = 1;
//     // // vencparam[2].bitrate = 2000;
//     // // vencparam[2].quit = 0;

//     // //
//     // printf("=================hb_vps_init start\n");
//     // ret = hb_vps_init_param(0, 3840, 2160, vencparam);
//     // if (ret != 0) {
//     //     printf("hb_vps_init_param failed\n");
//     //     return ret;
//     // }
//     // printf("=================hb_vps_start start\n");
//     // ret = hb_vps_start(0);
//     // if (ret != 0) {
//     //     printf("hb_vps_start failed\n");
//     //     return ret;
//     // }
//     // printf("=================hb_vps_start done\n");
//     // for (i=0; i<5; i++) {
//     //     printf("vencparam[%d].type: %x\n", i, vencparam[i].type);
//     //     if (vencparam[i].type == 0) continue;
//     //     ret = sample_venc_reinit(vencparam[i].veChn, vencparam[i].type,
//     //         vencparam[i].width, vencparam[i].height, vencparam[i].bitrate);
//     //     if (ret < 0) {
//     //         printf("sample_venc_init error!\n");
//     //         return ret;
//     //     }
//     // }

//     // for (i=0; i<5; i++) {
//     //     vencparam[i].quit = 0;
//     //     if (vencparam[i].type > 0) {
//     //         if (g_bindflag == 1)
//     //             ret = sample_vps_bind_venc(vencparam[i].vpsGrp,
//     //                             vencparam[i].vpsChn, vencparam[i].veChn);
//     //         ret = pthread_create(&vens_thid[i], NULL,
//     //                             (void *)venc_thread, &(vencparam[i]));
//     //     }
//     // }

//     return 0;   
// }
// void *ipu_osd_process_workfun(void *osd_info);
typedef struct osd_process_s1
{
	uint32_t width;
	uint32_t height;
	uint32_t start_x;
	uint32_t start_y;
	uint32_t image_width;
	char *src_addr;
	char *tar_y_addr;
	char *tar_uv_addr;
	uint32_t *color_map;
	bool overlay_mode;
}osd_process_t1;
void ipu_osd_process_workfun1(void *osd_info)
{
	osd_process_t1 *osd_pro = (osd_process_t1 *)osd_info;
	char color_index;
	char high_color, low_color;
	int tar_x, tar_y;
	char *addr_y;
	char *addr_uv;
	uint16_t color;
	uint32_t w_count;
	char *src_y_addr, *src_y_addr_or;
	char *src_uv_addr, *src_uv_addr_or;
	uint32_t invert_en;

	w_count = osd_pro->width / 16;
	src_y_addr = osd_pro->src_addr + osd_pro->height * osd_pro->width / 2;
	src_uv_addr = src_y_addr + osd_pro->height * osd_pro->width;
	src_y_addr_or = src_uv_addr + osd_pro->height * osd_pro->width / 2;
	src_uv_addr_or = src_y_addr_or + osd_pro->height * osd_pro->width;
	invert_en = osd_pro->overlay_mode;

	uint8x16_t invert_tmp;
	if (invert_en == 1) {
		invert_tmp = vdupq_n_u8(0xff);
	}

	for (int h = 0; h < osd_pro->height; h++) {
		for (int w = 0; w < w_count; w++) {
			tar_x = osd_pro->start_x + w * 16;
			tar_y = osd_pro->start_y + h;

			uint8x16_t y_tmp = vld1q_u8(src_y_addr + (h * w_count + w) * 16);

			uint8x16_t y_invert_tmp, y_result_tmp;
			if (invert_en == 1) {
				y_invert_tmp = vsubq_u8(invert_tmp, y_tmp);
			}

			uint8x16_t y_or_tmp = vld1q_u8(src_y_addr_or + (h * w_count + w) * 16);
			uint8x16_t y_img_tmp = vld1q_u8(
					osd_pro->tar_y_addr + tar_y * osd_pro->image_width + tar_x);
			uint8x16_t y_or_img_tmp = vandq_u8(y_or_tmp, y_img_tmp);

			if (invert_en == 1) {
				y_result_tmp = vaddq_u8(y_or_img_tmp, y_invert_tmp);
			} else {
				y_result_tmp = vaddq_u8(y_or_img_tmp, y_tmp);
			}

			vst1q_u8(osd_pro->tar_y_addr + tar_y * osd_pro->image_width + tar_x,
					y_result_tmp);

			if ((h % 2) == 0) {
				uint8x16_t uv_tmp = vld1q_u8(src_uv_addr + (h / 2 * w_count + w) * 16);

				uint8x16_t uv_invert_tmp, uv_result_tmp;
				if (invert_en == 1) {
					uv_invert_tmp = vsubq_u8(invert_tmp, uv_tmp);
				}

				uint8x16_t uv_or_tmp = vld1q_u8(src_uv_addr_or +
								(h / 2 * w_count + w) * 16);
				uint8x16_t uv_img_tmp = vld1q_u8(osd_pro->tar_uv_addr +
								(tar_y / 2 * osd_pro->image_width + tar_x));
				uint8x16_t uv_or_img_tmp = vandq_u8(uv_or_tmp, uv_img_tmp);

				if (invert_en == 1) {
					uv_result_tmp = vaddq_u8(uv_or_img_tmp, uv_invert_tmp);
				} else {
					uv_result_tmp = vaddq_u8(uv_or_img_tmp, uv_tmp);
				}

				vst1q_u8(osd_pro->tar_uv_addr +
						(tar_y / 2 * osd_pro->image_width + tar_x),
						uv_result_tmp);
			}
		}
	}
	free(osd_info);
}

void pym_get_thread(void *param)
{
    int ret;
    pym_buffer_t out_pym_buf;
    int width, height;
    // int mmz_size;
    struct timeval ts0, ts1, ts2, ts3, ts4, ts5;
    int pts = 0;
    int dumpnum = 10;
    uint32_t color_map[15] = {0xFFFFFF, 0x000000, 0x808000, 0x00BFFF, 0x00FF00,
							0xFFFF00, 0x8B4513, 0xFF8C00, 0x800080, 0xFFC0CB,
							0xFF0000, 0x98F898, 0x00008B, 0x006400, 0x8B0000};

    VIDEO_FRAME_S pstFrame;
    // memset(&pstFrame, 0, sizeof(VIDEO_FRAME_S));

    printf("pym_get_thread in===========\n");
    // veChn = vparam->veChn;
    // width = vparam->width;
    // height = vparam->height;

    // mmz_size = width * height * 3 / 2;
    
					
	// gettimeofday(&ts, NULL);			
	//snprintf(str, sizeof(str), "%ld.%06ld", ts.tv_sec, ts.tv_usec);			
    time_t start, last=-1;
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
            printf("pym fps %d\n", idx);
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
        if (g_osd_flag > 0) {
            gettimeofday(&ts0, NULL);
            osd_process_t1* osd_info;
            osd_info = (osd_process_t1*)malloc(sizeof(osd_process_t1));
            osd_info->width = 3840/4;
            osd_info->height = 2160/4;
            osd_info->start_x = 0;
            osd_info->start_y = 0;
            osd_info->image_width = 3840;
            osd_info->src_addr = out_pym_buf.pym[2].addr[0];
            osd_info->tar_y_addr = pstFrame.stVFrame.vir_ptr[0];
            osd_info->tar_uv_addr = pstFrame.stVFrame.vir_ptr[1];
            osd_info->color_map = color_map;
            osd_info->overlay_mode = 0;
            ipu_osd_process_workfun1((void *)osd_info);
            gettimeofday(&ts1, NULL);
        }
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
        if (g_osd_flag > 0) {
            gettimeofday(&ts0, NULL);
            osd_process_t1* osd_info;
            osd_info = (osd_process_t1*)malloc(sizeof(osd_process_t1));
            osd_info->width = 1920/4;
            osd_info->height = 1080/4;
            osd_info->start_x = 0;
            osd_info->start_y = 0;
            osd_info->image_width = 1920;
            osd_info->src_addr = out_pym_buf.pym[3].addr[0];
            osd_info->tar_y_addr = pstFrame.stVFrame.vir_ptr[0];
            osd_info->tar_uv_addr = pstFrame.stVFrame.vir_ptr[1];
            osd_info->color_map = color_map;
            osd_info->overlay_mode = 0;
            ipu_osd_process_workfun1((void *)osd_info);
            gettimeofday(&ts1, NULL);
            // printf("memcpy noncached use time: %d us\n", 
            //     (ts1.tv_sec - ts0.tv_sec)*1000000 + ts1.tv_usec -ts0.tv_usec);
        }
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
        if (g_osd_flag > 0) {
            gettimeofday(&ts0, NULL);
            osd_process_t1* osd_info;
            osd_info = (osd_process_t1*)malloc(sizeof(osd_process_t1));
            osd_info->width = 1920/4;
            osd_info->height = 1080/4;
            osd_info->start_x = 0;
            osd_info->start_y = 0;
            osd_info->image_width = 1920;
            osd_info->src_addr = out_pym_buf.pym[3].addr[0];
            osd_info->tar_y_addr = pstFrame.stVFrame.vir_ptr[0];
            osd_info->tar_uv_addr = pstFrame.stVFrame.vir_ptr[1];
            osd_info->color_map = color_map;
            osd_info->overlay_mode = 0;
            ipu_osd_process_workfun1((void *)osd_info);
            gettimeofday(&ts1, NULL);
            // printf("memcpy noncached use time: %d us\n", 
            //     (ts1.tv_sec - ts0.tv_sec)*1000000 + ts1.tv_usec -ts0.tv_usec);
        }
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
        if (g_osd_flag > 0) {
            
        }
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
        if (g_osd_flag > 0) {
            
        }
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
    int idx = 0;
    int groupId = 0;
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

int sample_singlepipe_5venc()
{
    int ret, i, venctype;
    // int rgnenable = 0;
    // pthread_t vens_thid[6];
    // pthread_t rgn_thid[6];
    // // pthread_t vdec_thid;
    // vencParam vencparam[6]={0};
    // rgnParam  rgnparam[6]={0};
    int vdecchn = 0;
    int vdectype = 96;

    start_time = time(NULL);
	end_time = start_time + run_time;

    if ((sensorId[0] == OS8A10_25FPS_3840P_RAW10_LINEAR) ||
        (sensorId[0] == OS8A10_25FPS_3840P_RAW10_DOL2) ||
        (sensorId[0] == SIF_TEST_PATTERN0_2160P)) {
        g_4kmode = 5;
    }

    venctype = g_venc_flag;
    if (venctype > 2) venctype = 2;

    vencparam[0].veChn = 4;
    vencparam[0].type = venctype;
    vencparam[0].width = 704;
    vencparam[0].height = 576;
    vencparam[0].vpsGrp = 0;
    vencparam[0].vpsChn = 0;
    vencparam[0].bitrate = 2000;
    vencparam[0].quit = 0;

    vencparam[1].veChn = 0;
    vencparam[1].type = venctype;
    vencparam[1].width = 2688;
    vencparam[1].height = 1520;
    vencparam[1].vpsGrp = 0;
    vencparam[1].vpsChn = 5;
    vencparam[1].bitrate = 8000;
    vencparam[1].quit = 0;

    vencparam[2].veChn = 2;
    vencparam[2].type = venctype;
    vencparam[2].width = 1920;
    vencparam[2].height = 1080;
    vencparam[2].vpsGrp = 0;
    vencparam[2].vpsChn = 1;
    vencparam[2].bitrate = 4000;
    vencparam[2].quit = 0;

    vencparam[3].veChn = 1;
    vencparam[3].type = venctype;
    vencparam[3].width = 1920;
    vencparam[3].height = 1080;
    vencparam[3].vpsGrp = 0;
    vencparam[3].vpsChn = 3;
    vencparam[3].bitrate = 4000;
    vencparam[3].quit = 0;

    vencparam[4].veChn = 3;
    vencparam[4].type = venctype;
    vencparam[4].width = 704;
    vencparam[4].height = 576;
    vencparam[4].vpsGrp = 0;
    vencparam[4].vpsChn = 4;
    vencparam[4].bitrate = 768;
    vencparam[4].quit = 0;

    if (g_4kmode == 5) {
        vencparam[1].width = 3840;
        vencparam[1].height = 2160;
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
                return ret;
            }

            ret = sample_venc_start(vencparam[i].veChn);
            if (ret < 0) {
                printf("sample_start error!\n");
                sample_venc_common_deinit();
                return ret;
            }
        }
    }
    printf("=============================================\n");
    printf("sensorId[0]: %d, bus[0]: %d, port[0]: %d, mipiIdx[0]: %d,"
            "serdes_index[0]: %d, serdes_port[0]: %d\n", sensorId[0], bus[0],
            port[0], mipiIdx[0], serdes_index[0], serdes_port[0]);
    ret = hb_vin_vps_init(0, sensorId[0], mipiIdx[0],  serdes_port[0], vencparam);
    
	if (ret < 0) {
		printf("hb_vin_vps_init error!\n");
		return ret;
	}

    if ((sensorId[0] < SIF_TEST_PATTERN0_1080P)
            || (sensorId[0] > SIF_TEST_PATTERN0_2160P)) {
        if ((sensorId[0] == OS8A10_25FPS_3840P_RAW10_LINEAR) ||
            (sensorId[0] == OS8A10_25FPS_3840P_RAW10_DOL2)) {
			HB_MIPI_EnableSensorClock(mipiIdx[0]);
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
    // ret = hb_vps_init(1);
    // if (ret) {
    //     printf("hb_vps_init 1 failed\n");
    //     goto err;
    // }
    // ret = hb_vps_start(1);
    // if (ret) {
    //     printf("hb_vps_start 1 failed\n");
    //     goto err;
    // }

    if (g_iar_enable) {
        ret = sample_vot_init();
        if (ret) {
            printf("sample_vot_init failed\n");
            return ret;
        }
    }

    // HB_VDEC_Module_Init();
    // ret = sample_vdec_init(vdectype, vdecchn);
    // if (ret) {
    //     printf("sample_vdec_init failed\n");
    //     goto err;
    // }
    // sample_vdec_bind_vot(0, 1);
    // sample_vdec_bind_vps(0, 1, 0);
    // sample_vps_bind_vot(1,0,0);

    // ret = sample_vot_wb_init(1, 4);
    // if (ret != 0) {
    //     printf("sample_vot_wb_init error!!!\n");
    //     return -1;
    // }

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
            if (g_bindflag == 1)
                ret = sample_vps_bind_venc(vencparam[0].vpsGrp,
                        vencparam[0].vpsChn, vencparam[0].veChn);
            ret = pthread_create(&vens_thid[0], NULL,
                (void *)venc_thread, &(vencparam[0]));
        }
        if (vencparam[1].type > 0) {
            if (g_bindflag == 1)
                ret = sample_vps_bind_venc(vencparam[1].vpsGrp,
                        vencparam[1].vpsChn, vencparam[1].veChn);
            ret = pthread_create(&vens_thid[1], NULL,
                (void *)venc_thread, &(vencparam[1]));
        }
        if (vencparam[2].type > 0) {
            if (g_bindflag == 1)
                ret = sample_vps_bind_venc(vencparam[2].vpsGrp,
                        vencparam[2].vpsChn, vencparam[2].veChn);
            ret = pthread_create(&vens_thid[2], NULL,
                (void *)venc_thread, &(vencparam[2]));
        }
        if (vencparam[3].type > 0) {
            if (g_bindflag == 1)
                ret = sample_vps_bind_venc(vencparam[3].vpsGrp,
                        vencparam[3].vpsChn, vencparam[3].veChn);
            ret = pthread_create(&vens_thid[3], NULL,
                (void *)venc_thread, &(vencparam[3]));
        }
        if (vencparam[4].type > 0) {
            if (g_bindflag == 1)
                ret = sample_vps_bind_venc(vencparam[4].vpsGrp,
                        vencparam[4].vpsChn, vencparam[4].veChn);
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

    // ret = sample_vps_bind_vot(0, 1, 0);
    // ret = sample_vps_bind_vot(0, 6, 1);

    return 0;
}

int sample_singlepipe_5venc_deinit()
{
    int ret;

    if (g_use_ipu == 0) {
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

    // ret = sample_vps_unbind_vot(0, 1, 0);
    // ret = sample_vps_unbind_vot(0, 6, 1);
    if (g_use_ipu == 0) ret = pthread_join(vens_thid[5], NULL);
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

err:
    // ret = sample_vps_deinit(0);
    // sample_vot_wb_deinit();
    // sample_vdec_deinit(vdecchn);
    if (vencparam[0].type > 0) {
         sample_venc_stop(0);
        sample_venc_deinit(0);
    }
    if (vencparam[1].type > 0) {
        sample_venc_stop(1);
        sample_venc_deinit(1);
    }
    if (vencparam[2].type > 0) {
        sample_venc_stop(2);
        sample_venc_deinit(2);
    }
    if (vencparam[3].type > 0) {
        sample_venc_stop(3);
        sample_venc_deinit(3);
    }
    if (vencparam[4].type > 0) {
        sample_venc_stop(4);
        sample_venc_deinit(4);
    }
    if (vencparam[5].type > 0) {
        sample_venc_stop(5);
        sample_venc_deinit(5);
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
			HB_MIPI_DisableSensorClock(mipiIdx[0]);
		}
    }
	hb_vin_vps_deinit(0, sensorId[0]);

    // HB_VDEC_Module_Uninit();

    return 0;
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

	printf("buf:y: vaddr = 0x%x paddr = 0x%x; uv: vaddr = 0x%x, paddr = 0x%x\n",
				buffer->img_addr.addr[0], buffer->img_addr.paddr[0],
				buffer->img_addr.addr[1], buffer->img_addr.paddr[1]);
	return 0;
}

int sample_ipu_feedback(char *yuvname)
{
    int ret;
	int img_in_fd, img_out_fd;
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

	img_in_fd = open(yuvname, O_RDWR | O_CREAT);
	if (img_in_fd < 0) {
		printf("open file %d failed !\n", yuvname);
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
