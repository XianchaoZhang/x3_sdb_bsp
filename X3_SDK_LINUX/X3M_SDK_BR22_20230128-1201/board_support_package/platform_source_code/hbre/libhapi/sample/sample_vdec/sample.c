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
#include "hb_vps_api.h"
#include "sample.h"
#include "hb_vot.h"

time_t start_time, now;
time_t run_time = 10000000;
time_t end_time = 0;
int g_vin_fd[MAX_SENSOR_NUM];
int sensorId[MAX_SENSOR_NUM];
int mipiIdx[MAX_MIPIID_NUM];
int bus[MAX_SENSOR_NUM];
int port[MAX_SENSOR_NUM];
int serdes_index[MAX_SENSOR_NUM];
int serdes_port[MAX_SENSOR_NUM];
int groupMask;
int need_clk;
int raw_dump;
int yuv_dump;
int need_cam;
int sample_mode = 0;
int data_type;
int vin_vps_mode = VIN_ONLINE_VPS_ONLINE;
int need_grp_rotate;
int need_chn_rotate;
int need_ipu;
int need_pym;
int vps_dump;
int need_md;

char *g_vdecfilename[8] = {NULL};

int g_bindflag = 1;
int g_save_flag = 1;
int g_exit = 0;
int g_venc_flag = 1;
int g_osd_flag = 0;
int g_4kmode = 0;
int g_width = 3840;
int g_height = 2160;

void venc_thread(void *vencpram)
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

	printf("venc_thread in, %d,%d,%d,%d,%d\n",
        veChn, vpsGrp, vpsChn, width, height);

	while (g_exit == 0) {
        if (g_bindflag == 0) {
            ret = HB_VPS_GetChnFrame(vpsGrp, vpsChn, &out_buf, 2000);
            if (ret != 0) {
                printf("HB_VPS_GetChnFrame error!!!\n");
                continue;
            }

            // normal_buf_info_print(&out_buf);

            pstFrame.stVFrame.phy_ptr[0] = out_buf.img_addr.paddr[0];
            pstFrame.stVFrame.phy_ptr[1] = out_buf.img_addr.paddr[1];

            pstFrame.stVFrame.vir_ptr[0] = out_buf.img_addr.addr[0];
            pstFrame.stVFrame.vir_ptr[1] = out_buf.img_addr.addr[1];

            ret = HB_VENC_SendFrame(veChn, &pstFrame, 3000);
            if (ret < 0) {
                printf("HB_VENC_SendFrame error!!!\n");
                break;
            }
        }

        ret = HB_VENC_GetStream(veChn, &pstStream, 3000);
        if (ret < 0) {
			printf("HB_VENC_GetStream error!!!\n");
            //break;
		} else {
            HB_VENC_ReleaseStream(veChn, &pstStream);
            if (g_save_flag == 1) {
                fwrite(pstStream.pstPack.vir_ptr,
                    pstStream.pstPack.size, 1, outFile);
            }
        }

        if (g_bindflag == 0) {
            ret = HB_VPS_ReleaseChnFrame(vpsGrp, vpsChn, &out_buf);
            if (ret) {
                printf("HB_VPS_ReleaseChnFrame error!!!\n");
            }
        }
	}

    if (g_save_flag == 1) {
        fclose(outFile);
    }
	printf("vi_to_vps_thread exit\n");
}

int sample_vdec_vot_test()
{
    int ret;
    pthread_t vdec_thid;
	int idx = 0;
    ret = sample_vot_init();
    if (ret) {
        printf("sample_vot_init failed\n");
        return ret;
    }

    HB_VDEC_Module_Init();
    ret = sample_vdec_init(96, 0);
    if (ret) {
        printf("sample_vdec_init failed\n");
        return ret;
    }

    ret = sample_vdec_bind_vot(0, 0);
    if (ret) {
        printf("sample_vdec_bind_vot failed\n");
        return ret;
    }

    ret = pthread_create(&vdec_thid, NULL, (void *)do_sync_decoding, NULL);
	// FILE *fw = fopen("xxxx.yuv", "wb");
	// while(g_exit == 0) {
	// 	VIDEO_FRAME_S stFrameInfo;
	// 	ret = HB_VDEC_GetFrame(0, &stFrameInfo, 1000);
	// 	if (ret == 0) {
	// 		if (idx++<1000) {
	// 			// FILE *fw = fopen("xxxx.yuv", "ab");
	// 			fwrite(stFrameInfo.stVFrame.vir_ptr[0], stFrameInfo.stVFrame.height * stFrameInfo.stVFrame.width, 1 ,fw);
	// 			fwrite(stFrameInfo.stVFrame.vir_ptr[1], stFrameInfo.stVFrame.height * stFrameInfo.stVFrame.width/2, 1 ,fw);
	// 			// fclose(fw);
	// 		}
	// 		HB_VDEC_ReleaseFrame(0, &stFrameInfo);
	// 	}
	// }
	// fclose(fw);

	// printf("get out\n");
    ret = pthread_join(vdec_thid, NULL);

	sample_vdec_stop(0);

	ret = sample_vdec_unbind_vot(0, 0);
    if (ret) {
        printf("sample_vdec_bind_venc failed\n");
        return ret;
    }

	// sample_vdec_deinit(0);
    HB_VDEC_Module_Uninit();

	sample_vot_deinit();

    return ret;
}

int memcpy_4in1(unsigned char *py, unsigned char *puv, int width, int height,
			hb_vio_buffer_t *stFrameInfo, int idx)
{
	int w, h, i;
	unsigned char *pdsty, *pdstuv;

	w = stFrameInfo->img_addr.width;
	h = stFrameInfo->img_addr.height;

	if (idx == 0) {
		pdsty = py;
		pdstuv = puv;
	} else if (idx == 1) {
		pdsty = py + width/2;
		pdstuv = puv + width/2;
	} else if (idx == 2) {
		pdsty = py + width*height/2;
		pdstuv = puv + width*height/4;
	} else {
		pdsty = py + width*height/2 + width/2;
		pdstuv = puv + width*height/4 + width/2;
	}
	
	for (i=0; i<h; i++) {
		memcpy(pdsty, stFrameInfo->img_addr.addr[0]+i*stFrameInfo->img_addr.stride_size, w);
		pdsty += width;
	}

	for (i=0; i<h/2; i++) {
		memcpy(pdstuv, stFrameInfo->img_addr.addr[1]+i*stFrameInfo->img_addr.stride_size, w);
		pdstuv += width;
	}

	return 0;
}

int memcpy_9in1(unsigned char *py, unsigned char *puv, int width, int height,
			hb_vio_buffer_t *stFrameInfo, int idx)
{
	int w, h, i;
	unsigned char *pdsty, *pdstuv;

	w = stFrameInfo->img_addr.width;
	h = stFrameInfo->img_addr.height;

	if (idx == 0) {
		pdsty = py;
		pdstuv = puv;
	} else if (idx == 1) {
		pdsty = py + width/3;
		pdstuv = puv + width/3;
	} else if (idx == 2) {
		pdsty = py + width*2/3;
		pdstuv = puv + width*2/3;
	} else if (idx == 3) {
		pdsty = py + width*height/3;
		pdstuv = puv + width*height/6;
	} else if (idx == 4) {
		pdsty = py + width*height/3 + width/3;
		pdstuv = puv + width*height/6 + width/3;
	} else if (idx == 5) {
		pdsty = py + width*height/3 + width*2/3;
		pdstuv = puv + width*height/6 + width*2/3;
	} else if (idx == 6) {
		pdsty = py + width*height*2/3;
		pdstuv = puv + width*height*2/6;
	} else if (idx == 7) {
		pdsty = py + width*height*2/3 + width/3;
		pdstuv = puv + width*height*2/6 + width/3;
	} else if (idx == 8) {
		pdsty = py + width*height*2/3 + width*2/3;
		pdstuv = puv + width*height*2/6 + width*2/3;
	}
	
	for (i=0; i<h; i++) {
		memcpy(pdsty, stFrameInfo->img_addr.addr[0]+i*stFrameInfo->img_addr.stride_size, w);
		pdsty += width;
	}

	for (i=0; i<h/2; i++) {
		memcpy(pdstuv, stFrameInfo->img_addr.addr[1]+i*stFrameInfo->img_addr.stride_size, w);
		pdstuv += width;
	}

	return 0;
}

int sample_multiply_vdec_vot_test(int mode)
{
    int ret;
    pthread_t vdec_thid[9];
	int idx = 0, i;
	int chnnum = 4;
	vdecParam vdecparam[9];
	VIDEO_FRAME_S stFrameInfo;
	hb_vio_buffer_t out_buf;
	unsigned char *pnv12;
	VOT_FRAME_INFO_S stFrame = {};

	if (mode == 1) chnnum = 4;
	else if (mode == 2) chnnum = 8;
    ret = sample_vot_init();
    if (ret) {
        printf("sample_vot_init failed\n");
        return ret;
    }

    HB_VDEC_Module_Init();
	for (i=0; i<chnnum; i++) {
		ret = sample_vdec_init(96, i);
		if (ret) {
			printf("sample_vdec_init failed\n");
			return ret;
		}
		hb_vps_init(i, mode);
		sample_vdec_bind_vps(i, i, 0);
	}
    // ret = sample_vdec_bind_vot(0, 0);
    // if (ret) {
    //     printf("sample_vdec_bind_vot failed\n");
    //     return ret;
    // }
	for (i=0; i<chnnum; i++) {
		vdecparam[i].vdecchn = i;
		vdecparam[i].loop = 1;
		memcpy(vdecparam[i].fname, g_vdecfilename[i], strlen(g_vdecfilename[i])+1);
    	ret = pthread_create(&vdec_thid[i], NULL, (void *)do_sync_decoding, &vdecparam[i]);
	}

	pnv12 = (unsigned char *)malloc(1920 * 1080 * 3/2);
	
	// FILE *fw = fopen("xxxx.yuv", "wb");
	while(g_exit == 0) {
		if (mode == 1) {
			ret = HB_VPS_GetChnFrame(0, 1, &out_buf, 100);
			if (ret == 0) {
				memcpy_4in1(pnv12, pnv12+1920*1080, 1920, 1080, &out_buf, 0);
				ret = HB_VPS_ReleaseChnFrame(0, 1, &out_buf);
			}
			ret = HB_VPS_GetChnFrame(1, 1, &out_buf, 100);
			if (ret == 0) {
				memcpy_4in1(pnv12, pnv12+1920*1080, 1920, 1080, &out_buf, 1);
				ret = HB_VPS_ReleaseChnFrame(1, 1, &out_buf);
			}
			ret = HB_VPS_GetChnFrame(2, 1, &out_buf, 100);
			if (ret == 0) {
				memcpy_4in1(pnv12, pnv12+1920*1080, 1920, 1080, &out_buf, 2);
				ret = HB_VPS_ReleaseChnFrame(2, 1, &out_buf);
			}
			ret = HB_VPS_GetChnFrame(3, 1, &out_buf, 100);
			if (ret == 0) {
				memcpy_4in1(pnv12, pnv12+1920*1080, 1920, 1080, &out_buf, 3);
				ret = HB_VPS_ReleaseChnFrame(3, 1, &out_buf);
			}
		} else if (mode == 2){
			for (i=0; i<chnnum; i++) {
				ret = HB_VPS_GetChnFrame(i, 1, &out_buf, 100);
				if (ret == 0) {
					memcpy_9in1(pnv12, pnv12+1920*1080, 1920, 1080, &out_buf, i);
					ret = HB_VPS_ReleaseChnFrame(i, 1, &out_buf);
				}
			}
		}
		// ret = HB_VDEC_GetFrame(0, &stFrameInfo, 100);
		// if (ret == 0) {
		// 	memcpy(pnv12, stFrameInfo.stVFrame.vir_ptr[0], stFrameInfo.stVFrame.height * stFrameInfo.stVFrame.width);
		// 	memcpy(pnv12+1920*1080, stFrameInfo.stVFrame.vir_ptr[1], stFrameInfo.stVFrame.height * stFrameInfo.stVFrame.width/2);
			stFrame.addr = pnv12;
			stFrame.size = 1920*1080*3/2;
			ret = HB_VOT_SendFrame(0, 0, &stFrame, -1);
		// }
		// 	if (ret == 0) {
		// for (i=0; i<chnnum; i++) {
		// 	ret = HB_VDEC_GetFrame(i, &stFrameInfo, 30);
		// 	if (ret == 0) {
		// 		// if (idx++<1000) {
		// 		// 	// FILE *fw = fopen("xxxx.yuv", "ab");
		// 		// 	fwrite(stFrameInfo.stVFrame.vir_ptr[0], stFrameInfo.stVFrame.height * stFrameInfo.stVFrame.width, 1 ,fw);
		// 		// 	fwrite(stFrameInfo.stVFrame.vir_ptr[1], stFrameInfo.stVFrame.height * stFrameInfo.stVFrame.width/2, 1 ,fw);
		// 		// 	// fclose(fw);
		// 		// }
		// 		printf("vdec %d, %dx%d\n", i, stFrameInfo.stVFrame.width, stFrameInfo.stVFrame.height);
		// 		HB_VDEC_ReleaseFrame(i, &stFrameInfo);
		// 	}
		// }
	}
	// fclose(fw);

	// printf("get out\n");
	for (i=0; i<chnnum; i++) {
    	ret = pthread_join(vdec_thid[i], NULL);
	}

	for (i=0; i<chnnum; i++) {
		sample_vdec_unbind_vps(i, i, 0);
	}
	for (i=0; i<chnnum; i++) {
		sample_vdec_stop(i);
	}

	// ret = sample_vdec_unbind_vot(0, 0);
    // if (ret) {
    //     printf("sample_vdec_bind_venc failed\n");
    //     return ret;
    // }

	// sample_vdec_deinit(0);
    HB_VDEC_Module_Uninit();

	sample_vot_deinit();

    return ret;
}

int sample_vdec_test()
{
    int ret;
    pthread_t vdec_thid;

    ret = sample_venc_common_init();
    if (ret < 0) {
        printf("sample_venc_common_init error!\n");
        return ret;
    }

    ret = sample_venc_init(0, 1, g_width, g_height, 8000);
    if (ret < 0) {
        printf("sample_venc_init error!\n");
        return ret;
    }

    ret = sample_venc_start(0);
    if (ret < 0) {
        printf("sample_start error!\n");
        sample_venc_deinit(0);
        sample_venc_common_deinit();
        return ret;
    }

    // ret = sample_vot_init();
    // if (ret) {
    //     printf("sample_vot_init failed\n");
    //     ret ret;
    // }

    HB_VDEC_Module_Init();
    ret = sample_vdec_init(96, 0);
    if (ret) {
        printf("sample_vdec_init failed\n");
        return ret;
    }

    ret = sample_vdec_bind_venc(0, 0);
    if (ret) {
        printf("sample_vdec_bind_venc failed\n");
        return ret;
    }

    ret = pthread_create(&vdec_thid, NULL, (void *)do_sync_decoding, NULL);
    // ret = pthread_join(vdec_thid, NULL);
    FILE *outFile;
    VIDEO_STREAM_S pstStream;
    memset(&pstStream, 0, sizeof(VIDEO_STREAM_S));

    outFile = fopen("./vencxx_stream.h264", "wb");
    while (g_exit == 0) {
        // sleep(1);
        ret = HB_VENC_GetStream(0, &pstStream, 3000);
        if (ret < 0) {
			printf("HB_VENC_GetStream error!!!\n");
            // break;
		} else {
            fwrite(pstStream.pstPack.vir_ptr,
                pstStream.pstPack.size, 1, outFile);
            // printf("pstStream.pstPack.size = %d\n", pstStream.pstPack.size);
            HB_VENC_ReleaseStream(0, &pstStream);
        }
    }
    fclose(outFile);

	ret = sample_vdec_unbind_venc(0, 0);
    if (ret) {
        printf("sample_vdec_bind_venc failed\n");
        return ret;
    }

    sample_venc_stop(0);
    sample_venc_deinit(0);
    sample_venc_common_deinit();

	sample_vdec_deinit(0);
    // sample_vot_deinit();
    HB_VDEC_Module_Uninit();

    return ret;
}

int sample_jpeg_decoding_test()
{
	int ret;

	HB_VDEC_Module_Init();
    ret = sample_vdec_init(26, 0);
    if (ret) {
        printf("sample_vdec_init failed\n");
        return ret;
    }

	ret = sample_jpeg_decoding();
	if (ret) {
        printf("sample_vdec_init failed\n");
        return ret;
    }
	sleep(10);
printf("-------1\n");
	ret = sample_vdec_stop(0);
	if (ret) {
        printf("sample_vdec_stop failed\n");
        return ret;
    }
	printf("-------2\n");
	ret = sample_vdec_deinit(0);
	if (ret) {
        printf("sample_vdec_deinit failed\n");
        return ret;
    }
printf("-------3\n");
    HB_VDEC_Module_Uninit();

	return 0;
}

void parse_sensorid_func(char *optarg)
{
	char *p = optarg, *d;
	int i = 0;
	printf("optarg %s\n", optarg);
	while (p && *p && i < MAX_SENSOR_NUM) {
		d = strchr(p, ',');
		if (d)
			*d = '\0';
		sensorId[i] = atoi(p);
		printf("i %d sensorId[i] %d===========\n", i, sensorId[i]);
		i++;
		p = (d) ? (d + 1) : NULL;
	}
	return;
}

void parse_mipiIdx_func(char *optarg)
{
	char *p = optarg, *d;
	int i = 0;
	printf("optarg %s\n", optarg);
	while (p && *p && i < MAX_MIPIID_NUM) {
		d = strchr(p, ',');
		if (d)
			*d = '\0';
		mipiIdx[i] = atoi(p);
		printf("i %d mipiIdx[i] %d===========\n", i, mipiIdx[i]);
		i++;
		p = (d) ? (d + 1) : NULL;
	}
	return;
}
void parse_bus_func(char *optarg)
{
	char *p = optarg, *d;
	int i = 0;
	printf("optarg %s\n", optarg);
	while (p && *p && i < MAX_MIPIID_NUM) {
		d = strchr(p, ',');
		if (d)
			*d = '\0';
		bus[i] = atoi(p);
		printf("i %d bus[i] %d===========\n", i, bus[i]);
		i++;
		p = (d) ? (d + 1) : NULL;
	}
	return;
}
void parse_port_func(char *optarg)
{
	char *p = optarg, *d;
	int i = 0;
	printf("optarg %s\n", optarg);
	while (p && *p && i < MAX_MIPIID_NUM) {
		d = strchr(p, ',');
		if (d)
			*d = '\0';
		port[i] = atoi(p);
		printf("i %d port[i] %d===========\n", i, port[i]);
		i++;
		p = (d) ? (d + 1) : NULL;
	}
	return;
}
void parse_serdes_index_func(char *optarg)
{
	char *p = optarg, *d;
	int i = 0;
	printf("optarg %s\n", optarg);
	while (p && *p && i < MAX_MIPIID_NUM) {
		d = strchr(p, ',');
		if (d)
			*d = '\0';
		serdes_index[i] = atoi(p);
		printf("i %d serdes_index[i] %d===========\n", i, serdes_index[i]);
		i++;
		p = (d) ? (d + 1) : NULL;
	}
	return;
}
void parse_serdes_port_func(char *optarg)
{
	char *p = optarg, *d;
	int i = 0;
	printf("optarg %s\n", optarg);
	while (p && *p && i < MAX_MIPIID_NUM) {
		d = strchr(p, ',');
		if (d)
			*d = '\0';
		serdes_port[i] = atoi(p);
		printf("i %d serdes_index[i] %d===========\n", i, serdes_port[i]);
		i++;
		p = (d) ? (d + 1) : NULL;
	}
	return;
}

void print_usage(const char *prog)
{
	printf("Usage: %s \n", prog);
	puts("  -s --sensor_id       sensor_id\n"
		 "  -i --mipi_idx        mipi_idx\n"
	     "  -p --group_id        group_id\n"
	     "  -c --real_camera     real camera enable\n"
	     "  -r --run_time         time measured in seconds the program runs\n"
		 "  -d --dump_raw		  dump raw img\n"
		 "  -y --dump_yuv	      dump yuvimg\n"
	     "  -t --data_type        data type\n"
	     "  -m --multi_thread	 multi thread get\n"
	     "  -k --need_clk        need_clk from som\n"
	     "  -b --bus             bus number\n"
   	     "  -o --port            port\n"
   	     "  -S --serdes_index    serdes_index\n"
   	     "  -O --sedres_port     serdes_port\n"
		"-M --vin_vps_mode\n"
		"-X --need_grp_rotate\n"
		"-Y --need_chn_rotate\n"
		"-I --need_ipu\n"
		"-P --need_pym\n"
		"-D --vps_dump\n"
		"-N --need_md\n"
		"-R --need_osd\n");
	exit(1);
}

void parse_opts(int argc, char *argv[])
{
	while (1) {
		static const char short_options[] =
		    "s:w:h:e:i:p:c:r:d:f:y:t:m:k:b:o:W:S:O:M:X:Y:I:P:D:N:R:";
		static const struct option long_options[] = {
			{"sensorId", 1, 0, 's'},
			{"mipiIdx",  1, 0, 'i'},
			{"groupMask", 1, 0, 'p'},
			{"real_camera", 1, 0, 'c'},
			{"run_time", 1, 0, 'r'},
			{"raw_dump", 1, 0, 'd'},
			{"yuv_dump", 1, 0, 'y'},
			{"data_type", 1, 0, 't'},
			{"multi_thread", 1, 0, 'm'},
			{"need_clk", 1, 0, 'k'},
			{"bus", 1, 0, 'b'},
			{"port", 1, 0, 'o'},
			{"serdes_index", 1, 0, 'S'},
			{"serdes_port", 1, 0, 'O'},
			{"vin_vps_mode", 1, 0, 'M'},
			{"need_grp_rotate", 1, 0, 'X'},
			{"need_chn_rotate", 1, 0, 'Y'},
			{"need_ipu", 1, 0, 'I'},
			{"need_pym", 1, 0, 'P'},
			{"vps_dump", 1, 0, 'D'},
			{"need_md", 1, 0, 'N'},
			{"enctype", 1, 0, 'e'},
			{"savefile", 1, 0, 'W'},
			{"need_osd", 1, 0, 'R'},
			{"width", 1, 0, 'w'},
			{"height", 1, 0, 'h'},
			{"decfile", 1, 0, 'f'},
			{NULL, 0, 0, 0},
		};

		int cmd_ret;

		cmd_ret =
		    getopt_long(argc, argv, short_options, long_options, NULL);

		if (cmd_ret == -1)
			break;
		printf("cmd_ret %d optarg %s\n", cmd_ret, optarg);
	    switch (cmd_ret) {
		case 's':
			parse_sensorid_func(optarg);
			break;
		case 'i':
			parse_mipiIdx_func(optarg);
			break;
		case 'p':
			groupMask = atoi(optarg);;
			printf("groupMask = %d\n", groupMask);
			break;
		case 'c':
			need_cam = atoi(optarg);
			printf("need_cam = %d\n", need_cam);
			break;
		case 'r':
			run_time = atoi(optarg);
			printf("run_time = %ld\n", run_time);
			break;
		case 'm':
			sample_mode = atoi(optarg);
			printf("sample_mode = %d\n", sample_mode);
			break;
		case 'e':
			g_venc_flag = atoi(optarg);
			printf("g_venc_flag = %d\n", g_venc_flag);
			break;
		case 'W':
			g_save_flag = atoi(optarg);
			printf("g_save_flag = %d\n", g_save_flag);
			break;
		case 'w':
			g_width = atoi(optarg);
			printf("g_width = %d\n", g_width);
			break;
		case 'h':
			g_height = atoi(optarg);
			printf("g_height = %d\n", g_height);
			break;
		case 'f':
			g_vdecfilename[0] = optarg;
			printf("g_vdecfilename = %s\n", g_vdecfilename[0]);
			break;
		
		// case 't':
		// 	data_type = atoi(optarg);
		// 	printf("data_type = %d\n", data_type);
		// 	break;
		// case 'd':
		// 	raw_dump = atoi(optarg);
		// 	printf("raw_dump = %d\n", raw_dump);
		// 	break;
		// case 'y':
		// 	yuv_dump = atoi(optarg);
		// 	printf("yuv_dump = %d\n", yuv_dump);
		// 	break;
		// case 'k':
		// 	need_clk = atoi(optarg);
		// 	printf("need_clk = %d\n", need_clk);
		// 	break;
		// case 'b':
		// 	parse_bus_func(optarg);
		// 	break;
		// case 'o':
		// 	parse_port_func(optarg);
		// 	break;
		// case 'S':
		// 	parse_serdes_index_func(optarg);
		// 	break;
		// case 'O':
		// 	parse_serdes_port_func(optarg);
		// 	break;
		case 'M':
			vin_vps_mode = atoi(optarg);
			printf("vin_vps_mode = %d\n", vin_vps_mode);
			break;
		// case 'X':
		// 	need_grp_rotate = atoi(optarg);
		// 	printf("need_grp_rotate = %d\n", need_grp_rotate);
		// 	break;
		// case 'Y':
		// 	need_chn_rotate = atoi(optarg);
		// 	printf("need_chn_rotate = %d\n", need_chn_rotate);
		// 	break;
		// case 'I':
		// 	need_ipu = atoi(optarg);
		// 	printf("need_ipu = %d\n", need_ipu);
		// 	break;
		// case 'P':
		// 	need_pym = atoi(optarg);
		// 	printf("need_pym = %d\n", need_pym);
		// 	break;
		// case 'D':
		// 	vps_dump = atoi(optarg);
		// 	printf("vps_dump = %d\n", vps_dump);
		// 	break;
		// case 'N':
		// 	need_md = atoi(optarg);
		// 	printf("need_md = %d\n", need_md);
		// 	break;
		case 'R':
			g_osd_flag = atoi(optarg);
			printf("g_osd_flag = %d\n", g_osd_flag);
			break;

		default:
			print_usage(argv[0]);
			break;
		}
	}
}

int hb_vps_init(int pipeId, int mode)
{
	int ret = 0;

	VPS_GRP_ATTR_S grp_attr;
	VPS_CHN_ATTR_S chn_attr;
    VPS_PYM_CHN_ATTR_S pym_chn_attr;

	memset(&grp_attr, 0, sizeof(VPS_GRP_ATTR_S));
	if (mode == 2) {
		grp_attr.maxW = 1280;
		grp_attr.maxH = 720;	
	} else {
		grp_attr.maxW = 1920;
		grp_attr.maxH = 1080;
	}
    grp_attr.frameDepth = 1;
	ret = HB_VPS_CreateGrp(pipeId, &grp_attr);
	if (ret) {
		printf("HB_VPS_CreateGrp error!!!\n");
	} else {
		printf("created a group ok:GrpId = %d\n", pipeId);
	}
    // ret = HB_SYS_SetVINVPSMode(pipeId, 1);

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
	if (mode == 2) {
    	chn_attr.width = 1920/3;
    	chn_attr.height = 1080/3;
	} else if (mode == 1){
		chn_attr.width = 1920/2;
    	chn_attr.height = 1080/2;
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

int sample_vps_bind_vot(int vpsGrp, int vpsChn, int votChn)
{
    int s32Ret = 0;
    struct HB_SYS_MOD_S src_mod, dst_mod;

	src_mod.enModId = HB_ID_VPS;
	src_mod.s32DevId = vpsGrp;
	src_mod.s32ChnId = vpsChn;
	dst_mod.enModId = HB_ID_VOT;
	dst_mod.s32DevId = 0;
	dst_mod.s32ChnId = votChn;
	s32Ret = HB_SYS_Bind(&src_mod, &dst_mod);
	if (s32Ret != 0)
		printf("HB_SYS_Bind failed\n");

    return s32Ret;
}

int sample_vps_unbind_vot(int vpsGrp, int vpsChn, int votChn)
{
    int s32Ret = 0;
    struct HB_SYS_MOD_S src_mod, dst_mod;

	src_mod.enModId = HB_ID_VPS;
	src_mod.s32DevId = vpsGrp;
	src_mod.s32ChnId = vpsChn;
	dst_mod.enModId = HB_ID_VOT;
	dst_mod.s32DevId = 0;
	dst_mod.s32ChnId = votChn;
	s32Ret = HB_SYS_UnBind(&src_mod, &dst_mod);
	if (s32Ret != 0)
		printf("HB_SYS_Bind failed\n");

    return s32Ret;
}

int sample_vdec_vps_venc_vot()
{
    int ret;
    pthread_t vdec_thid;
	int vdecchn = 0;
	int vencchn = 0;
	int vpsgrp = 1, vpschn0 = 0, vpschn1 = 1, vpschn2 = 2;

    // ret = sample_venc_common_init();
    // if (ret < 0) {
    //     printf("sample_venc_common_init error!\n");
    //     return ret;
    // }

    // ret = sample_venc_init(vencchn, 1, 704, 576, 1000);
    // if (ret < 0) {
    //     printf("sample_venc_init error!\n");
    //     return ret;
    // }

    // ret = sample_venc_start(vencchn);
    // if (ret < 0) {
    //     printf("sample_start error!\n");
    //     sample_venc_deinit(vencchn);
    //     sample_venc_common_deinit();
    //     return ret;
    // }

	ret = hb_vps_init(vpsgrp, 0);
    if (ret) {
        printf("hb_vps_init 0 failed\n");
		// sample_venc_deinit(vencchn);
        // sample_venc_common_deinit();
		return ret;
    }
    // ret = hb_vps_start(vpsgrp);
    // if (ret) {
    //     printf("hb_vps_start 0 failed\n");
	// 	// hb_vps_deinit(vpsgrp);
	// 	// sample_venc_deinit(vencchn);
    //     // sample_venc_common_deinit();
	// 	return ret;
    // }

    ret = sample_vot_init();
    if (ret) {
        printf("sample_vot_init failed\n");
		// hb_vps_stop(vpsgrp);
		// hb_vps_deinit(vpsgrp);
		// sample_venc_deinit(vencchn);
        // sample_venc_common_deinit();
        return ret;
    }

    HB_VDEC_Module_Init();
    ret = sample_vdec_init(96, vdecchn);
    if (ret) {
        printf("sample_vdec_init failed\n");
		// hb_vps_stop(vpsgrp);
		// hb_vps_deinit(vpsgrp);
		// // sample_venc_deinit(vencchn);
        // // sample_venc_common_deinit();
		sample_vdec_deinit(vdecchn);
        return ret;
    }

    // ret = sample_vdec_bind_venc(0, 0);
    // if (ret) {
    //     printf("sample_vdec_bind_venc failed\n");
    //     return ret;
    // }
	ret = sample_vdec_bind_vps(vdecchn, vpsgrp, 0);
	if (ret) {
        printf("sample_vdec_bind_vps failed\n");
        goto err;
    }
    printf("sample_vdec_bind_vps ok\n");

	// ret = sample_vps_bind_venc(vpsgrp, 0, vencchn);
	// if (ret) {
    //     printf("sample_vps_bind_venc failed\n");
    //     goto err;
	// }
    // printf("sample_vps_bind_venc ok\n");
	ret = sample_vps_bind_vot(vpsgrp, 1, 0);
	if (ret) {
        printf("sample_vps_bind_vot 0 failed\n");
        goto err;
	}
    printf("sample_vps_bind_vot 0 ok\n");
	// ret = sample_vps_bind_vot(vpsgrp, 2, 1);
	// if (ret) {
    //     printf("sample_vps_bind_vot 1 failed\n");
    //     goto err;
	// }
    // printf("sample_vps_bind_vot 1 ok\n");
    ret = pthread_create(&vdec_thid, NULL, (void *)do_sync_decoding, NULL);
    ret = pthread_join(vdec_thid, NULL);
    // FILE *outFile;
    // VIDEO_STREAM_S pstStream;
    // memset(&pstStream, 0, sizeof(VIDEO_STREAM_S));
    // if (g_save_flag == 1) {
    //     outFile = fopen("./vencxx_stream.h264", "wb");
    // }
    // while (g_exit == 0) {
    //     // sleep(1);
    //     ret = HB_VENC_GetStream(vencchn, &pstStream, 3000);
    //     if (ret < 0) {
	// 		printf("HB_VENC_GetStream error!!!\n");
    //         g_exit = 1;
    //         break;
	// 	} else {
    //         if (g_save_flag == 1) {
    //             fwrite(pstStream.pstPack.vir_ptr,
    //                 pstStream.pstPack.size, 1, outFile);
    //         }
    //         // printf("pstStream.pstPack.size = %d\n", pstStream.pstPack.size);
    //         HB_VENC_ReleaseStream(vencchn, &pstStream);
    //     }
    // }

    // if (g_save_flag == 1) {
    //     fclose(outFile);
    // }

	ret = sample_vdec_unbind_vps(vdecchn, vpsgrp, 0);
    if (ret) {
        printf("sample_vdec_bind_venc failed\n");
        goto err;
    }
	// ret = sample_vps_unbind_venc(vpsgrp, 0, vencchn);
    // if (ret) {
    //     printf("sample_vps_unbind_vot failed\n");
    //     goto err;
    // }
	ret = sample_vps_unbind_vot(vpsgrp, 1, 0);
    if (ret) {
        printf("sample_vps_unbind_vot failed\n");
        goto err;
    }
	// ret = sample_vps_unbind_vot(vpsgrp, 2, 1);
    // if (ret) {
    //     printf("sample_vps_unbind_vot failed\n");
    // }

err:
	sample_vot_deinit();

    // sample_venc_stop(0);
    // sample_venc_deinit(0);
    // sample_venc_common_deinit();

	sample_vdec_deinit(0);
    // sample_vot_deinit();
    HB_VDEC_Module_Uninit();

    return ret;
}

int check_end(void)
{
	time_t now = time(NULL);
	// printf("Time info :: now(%ld), end_time(%ld) run time(%ld)!\n",
	// now, end_time, run_time);
	return !(now > end_time && run_time > 0);
}

void intHandler(int dummy)
{
	g_exit = 1;
	printf("rcv int signal\n");
}

int main(int argc, char *argv[])
{
	if (argc < 2) {
		print_usage(argv[0]);
		printf("leave, World! \n");
		exit(1);
	}
    parse_opts(argc, argv);

    signal(SIGINT, intHandler);

    if (sample_mode == 0) {
        // sample_singlepipe_5venc();
    } else if (sample_mode == 1) {
        // sample_multpipe_multvenc();
    } else if (sample_mode == 2) {
		sample_vdec_vot_test();
	} else if (sample_mode == 3) {
		// sample_vdec_vps_venc_vot();
	} else if (sample_mode == 4) {
		sample_jpeg_decoding_test();
	} else if (sample_mode == 5) {
		sample_vdec_test();
	} else if (sample_mode == 6) {
		sample_vdec_vps_venc_vot();
	} else if (sample_mode == 7) {
		g_vdecfilename[0] = "t1.h264";
		g_vdecfilename[1] = "t1.h264";
		g_vdecfilename[2] = "t1.h264";
		g_vdecfilename[3] = "t1.h264";
		sample_multiply_vdec_vot_test(1);
	} else if (sample_mode == 8) {
		g_vdecfilename[0] = "720p.h264";
		g_vdecfilename[1] = "720p_1.h264";
		g_vdecfilename[2] = "720p_2.h264";
		g_vdecfilename[3] = "720p.h264";
		g_vdecfilename[4] = "720p_1.h264";
		g_vdecfilename[5] = "720p_2.h264";
		g_vdecfilename[6] = "720p_3.h264";
		g_vdecfilename[7] = "720p_3.h264";
		sample_multiply_vdec_vot_test(2);
	}

    return 0;
}
