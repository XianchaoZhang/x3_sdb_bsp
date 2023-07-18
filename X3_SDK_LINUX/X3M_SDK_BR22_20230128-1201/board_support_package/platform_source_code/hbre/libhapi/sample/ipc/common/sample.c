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

time_t start_time, now;
time_t run_time = 10000000;
time_t end_time = 0;
int g_vin_fd[MAX_SENSOR_NUM];
int sensorId[MAX_SENSOR_NUM]={5,};
int mipiIdx[MAX_MIPIID_NUM]={1,};
int bus[MAX_SENSOR_NUM]={5,};
int port[MAX_SENSOR_NUM]={0,};
int serdes_index[MAX_SENSOR_NUM]={0,};
int serdes_port[MAX_SENSOR_NUM]={0,};
int groupMask = 1;
int need_clk;
int raw_dump;
int yuv_dump;
int need_cam;
int sample_mode = 0;
int data_type;
int vin_vps_mode = VIN_ONLINE_VPS_ONLINE;
int need_grp_rotate = 0;
int need_chn_rotate = 0;
int need_ipu;
int need_pym;
int vps_dump;
int need_md;
int vc_num = 1;
int need_dol2 = 0;
int g_taskbufsize = 8192;
int g_bindflag = 1;
int g_save_flag = 0;
int g_exit = 0;
int g_venc_flag = 1;
int g_osd_flag = 63;  // 0x3f;
int g_4kmode = 0;
int g_iar_enable = 1;
int g_bpu_usesample = 0;
int g_use_ldc = 0;
int g_set_qos = 0;
int g_use_ipu = 0;
int g_testpattern_fps = 30;
int g_chanage_res = 100000000;
int g_use_x3clock = 1;
int g_calib = 1;
int g_temperMode = 2;
int g_frame_rate = 30;
int g_vps_frame_depth = 2;
char* g_ipu_feedback = NULL;
int g_use_input = 0;
int g_intra_qp = 30;
int g_inter_qp = 32;
// int sample_vdec_vot_test()
// {
//     int ret;
//     pthread_t vdec_thid;

//     ret = sample_vot_init();
//     if (ret) {
//         printf("sample_vot_init failed\n");
//         return ret;
//     }

//     HB_VDEC_Module_Init();
//     ret = sample_vdec_init(96, 0);
//     if (ret) {
//         printf("sample_vdec_init failed\n");
//         return ret;
//     }

//     ret = sample_vdec_bind_vot(0, 0);
//     if (ret) {
//         printf("sample_vdec_bind_vot failed\n");
//         return ret;
//     }

//     ret = pthread_create(&vdec_thid, NULL, (void *)do_sync_decoding, NULL);
//     ret = pthread_join(vdec_thid, NULL);

// 	sample_vdec_stop(0);

// 	ret = sample_vdec_unbind_vot(0, 0);
//     if (ret) {
//         printf("sample_vdec_bind_venc failed\n");
//         return ret;
//     }

// 	sample_vdec_deinit(0);
//     HB_VDEC_Module_Uninit();

// 	sample_vot_deinit();

//     return ret;
// }

// int sample_vdec_test()
// {
//     int ret;
//     pthread_t vdec_thid;

//     ret = sample_venc_common_init();
//     if (ret < 0) {
//         printf("sample_venc_common_init error!\n");
//         return ret;
//     }

//     ret = sample_venc_init(0, 1, 704, 576, 1000);
//     if (ret < 0) {
//         printf("sample_venc_init error!\n");
//         return ret;
//     }

//     ret = sample_venc_start(0);
//     if (ret < 0) {
//         printf("sample_start error!\n");
//         sample_venc_deinit(0);
//         sample_venc_common_deinit();
//         return ret;
//     }

//     // ret = sample_vot_init();
//     // if (ret) {
//     //     printf("sample_vot_init failed\n");
//     //     ret ret;
//     // }

//     HB_VDEC_Module_Init();
//     ret = sample_vdec_init(96, 0);
//     if (ret) {
//         printf("sample_vdec_init failed\n");
//         return ret;
//     }

//     ret = sample_vdec_bind_venc(0, 0);
//     if (ret) {
//         printf("sample_vdec_bind_venc failed\n");
//         return ret;
//     }

//     ret = pthread_create(&vdec_thid, NULL, (void *)do_sync_decoding, NULL);
//     // ret = pthread_join(vdec_thid, NULL);
//     FILE *outFile;
//     VIDEO_STREAM_S pstStream;
//     memset(&pstStream, 0, sizeof(VIDEO_STREAM_S));

//     outFile = fopen("./vencxx_stream.h264", "wb");
//     while (g_exit == 0) {
//         // sleep(1);
//         ret = HB_VENC_GetStream(0, &pstStream, 3000);
//         if (ret < 0) {
// 			printf("HB_VENC_GetStream error!!!\n");
//             // break;
// 		} else {
//             fwrite(pstStream.pstPack.vir_ptr,
//                 pstStream.pstPack.size, 1, outFile);
//             // printf("pstStream.pstPack.size = %d\n", pstStream.pstPack.size);
//             HB_VENC_ReleaseStream(0, &pstStream);
//         }
//     }
//     fclose(outFile);

// 	ret = sample_vdec_unbind_venc(0, 0);
//     if (ret) {
//         printf("sample_vdec_bind_venc failed\n");
//         return ret;
//     }

//     sample_venc_stop(0);
//     sample_venc_deinit(0);
//     sample_venc_common_deinit();

// 	sample_vdec_deinit(0);
//     // sample_vot_deinit();
//     HB_VDEC_Module_Uninit();

//     return ret;
// }

// void parse_sensorid_func(char *optarg)
// {
// 	char *p = optarg, *d;
// 	int i = 0;
// 	printf("optarg %s\n", optarg);
// 	while (p && *p && i < MAX_SENSOR_NUM) {
// 		d = strchr(p, ',');
// 		if (d)
// 			*d = '\0';
// 		sensorId[i] = atoi(p);
// 		printf("i %d sensorId[i] %d===========\n", i, sensorId[i]);
// 		i++;
// 		p = (d) ? (d + 1) : NULL;
// 	}
// 	return;
// }

// void parse_mipiIdx_func(char *optarg)
// {
// 	char *p = optarg, *d;
// 	int i = 0;
// 	printf("optarg %s\n", optarg);
// 	while (p && *p && i < MAX_MIPIID_NUM) {
// 		d = strchr(p, ',');
// 		if (d)
// 			*d = '\0';
// 		mipiIdx[i] = atoi(p);
// 		printf("i %d mipiIdx[i] %d===========\n", i, mipiIdx[i]);
// 		i++;
// 		p = (d) ? (d + 1) : NULL;
// 	}
// 	return;
// }
// void parse_bus_func(char *optarg)
// {
// 	char *p = optarg, *d;
// 	int i = 0;
// 	printf("optarg %s\n", optarg);
// 	while (p && *p && i < MAX_MIPIID_NUM) {
// 		d = strchr(p, ',');
// 		if (d)
// 			*d = '\0';
// 		bus[i] = atoi(p);
// 		printf("i %d bus[i] %d===========\n", i, bus[i]);
// 		i++;
// 		p = (d) ? (d + 1) : NULL;
// 	}
// 	return;
// }
// void parse_port_func(char *optarg)
// {
// 	char *p = optarg, *d;
// 	int i = 0;
// 	printf("optarg %s\n", optarg);
// 	while (p && *p && i < MAX_MIPIID_NUM) {
// 		d = strchr(p, ',');
// 		if (d)
// 			*d = '\0';
// 		port[i] = atoi(p);
// 		printf("i %d port[i] %d===========\n", i, port[i]);
// 		i++;
// 		p = (d) ? (d + 1) : NULL;
// 	}
// 	return;
// }
// void parse_serdes_index_func(char *optarg)
// {
// 	char *p = optarg, *d;
// 	int i = 0;
// 	printf("optarg %s\n", optarg);
// 	while (p && *p && i < MAX_MIPIID_NUM) {
// 		d = strchr(p, ',');
// 		if (d)
// 			*d = '\0';
// 		serdes_index[i] = atoi(p);
// 		printf("i %d serdes_index[i] %d===========\n", i, serdes_index[i]);
// 		i++;
// 		p = (d) ? (d + 1) : NULL;
// 	}
// 	return;
// }
// void parse_serdes_port_func(char *optarg)
// {
// 	char *p = optarg, *d;
// 	int i = 0;
// 	printf("optarg %s\n", optarg);
// 	while (p && *p && i < MAX_MIPIID_NUM) {
// 		d = strchr(p, ',');
// 		if (d)
// 			*d = '\0';
// 		serdes_port[i] = atoi(p);
// 		printf("i %d serdes_index[i] %d===========\n", i, serdes_port[i]);
// 		i++;
// 		p = (d) ? (d + 1) : NULL;
// 	}
// 	return;
// }

// void print_usage(const char *prog)
// {
// 	printf("Usage: %s \n", prog);
// 	puts("  -s --sensor_id       sensor_id\n"
// 		 "  -i --mipi_idx        mipi_idx\n"
// 	     "  -p --group_id        group_id\n"
// 	     "  -c --real_camera     real camera enable\n"
// 	     "  -r --run_time         time measured in seconds the program runs\n"
// 		 "  -d --dump_raw		  dump raw img\n"
// 		 "  -y --dump_yuv	      dump yuvimg\n"
// 	     "  -t --data_type        data type\n"
// 	     "  -m --multi_thread	 multi thread get\n"
// 	     "  -k --need_clk        need_clk from som\n"
// 	     "  -b --bus             bus number\n"
//    	     "  -o --port            port\n"
//    	     "  -S --serdes_index    serdes_index\n"
//    	     "  -O --sedres_port     serdes_port\n"
// 		"-M --vin_vps_mode\n"
// 		"-X --need_grp_rotate\n"
// 		"-Y --need_chn_rotate\n"
// 		"-I --need_ipu\n"
// 		"-P --need_pym\n"
// 		"-D --vps_dump\n"
// 		"-N --need_md\n"
// 		"-R --need_osd\n");
// 	exit(1);
// }

// void parse_opts(int argc, char *argv[])
// {
// 	while (1) {
// 		static const char short_options[] =
// 		    "s:w:e:i:p:c:r:d:y:t:m:k:b:o:S:O:M:X:Y:I:P:D:N:R:";
// 		static const struct option long_options[] = {
// 			{"sensorId", 1, 0, 's'},
// 			{"mipiIdx",  1, 0, 'i'},
// 			{"groupMask", 1, 0, 'p'},
// 			{"real_camera", 1, 0, 'c'},
// 			{"run_time", 1, 0, 'r'},
// 			{"raw_dump", 1, 0, 'd'},
// 			{"yuv_dump", 1, 0, 'y'},
// 			{"data_type", 1, 0, 't'},
// 			{"multi_thread", 1, 0, 'm'},
// 			{"need_clk", 1, 0, 'k'},
// 			{"bus", 1, 0, 'b'},
// 			{"port", 1, 0, 'o'},
// 			{"serdes_index", 1, 0, 'S'},
// 			{"serdes_port", 1, 0, 'O'},
// 			{"vin_vps_mode", 1, 0, 'M'},
// 			{"need_grp_rotate", 1, 0, 'X'},
// 			{"need_chn_rotate", 1, 0, 'Y'},
// 			{"need_ipu", 1, 0, 'I'},
// 			{"need_pym", 1, 0, 'P'},
// 			{"vps_dump", 1, 0, 'D'},
// 			{"need_md", 1, 0, 'N'},
// 			{"enctype", 1, 0, 'e'},
// 			{"savefile", 1, 0, 'w'},
// 			{"need_osd", 1, 0, 'R'},
// 			{NULL, 0, 0, 0},
// 		};

// 		int cmd_ret;

// 		cmd_ret =
// 		    getopt_long(argc, argv, short_options, long_options, NULL);

// 		if (cmd_ret == -1)
// 			break;
// 		printf("cmd_ret %d optarg %s\n", cmd_ret, optarg);
// 	    switch (cmd_ret) {
// 		case 's':
// 			parse_sensorid_func(optarg);
// 			break;
// 		case 'i':
// 			parse_mipiIdx_func(optarg);
// 			break;
// 		case 'p':
// 			groupMask = atoi(optarg);;
// 			printf("groupMask = %d\n", groupMask);
// 			break;
// 		case 'c':
// 			need_cam = atoi(optarg);
// 			printf("need_cam = %d\n", need_cam);
// 			break;
// 		case 'r':
// 			run_time = atoi(optarg);
// 			printf("run_time = %ld\n", run_time);
// 			break;
// 		case 'm':
// 			sample_mode = atoi(optarg);
// 			printf("sample_mode = %d\n", sample_mode);
// 			break;
// 		case 'e':
// 			g_venc_flag = atoi(optarg);
// 			printf("g_venc_flag = %d\n", g_venc_flag);
// 			break;
// 		case 'w':
// 			g_save_flag = atoi(optarg);
// 			printf("g_save_flag = %d\n", g_save_flag);
// 			break;
// 		// case 't':
// 		// 	data_type = atoi(optarg);
// 		// 	printf("data_type = %d\n", data_type);
// 		// 	break;
// 		// case 'd':
// 		// 	raw_dump = atoi(optarg);
// 		// 	printf("raw_dump = %d\n", raw_dump);
// 		// 	break;
// 		// case 'y':
// 		// 	yuv_dump = atoi(optarg);
// 		// 	printf("yuv_dump = %d\n", yuv_dump);
// 		// 	break;
// 		// case 'k':
// 		// 	need_clk = atoi(optarg);
// 		// 	printf("need_clk = %d\n", need_clk);
// 		// 	break;
// 		case 'b':
// 			parse_bus_func(optarg);
// 			break;
// 		case 'o':
// 			parse_port_func(optarg);
// 			break;
// 		case 'S':
// 			parse_serdes_index_func(optarg);
// 			break;
// 		case 'O':
// 			parse_serdes_port_func(optarg);
// 			break;
// 		case 'M':
// 			vin_vps_mode = atoi(optarg);
// 			printf("vin_vps_mode = %d\n", vin_vps_mode);
// 			break;
// 		// case 'X':
// 		// 	need_grp_rotate = atoi(optarg);
// 		// 	printf("need_grp_rotate = %d\n", need_grp_rotate);
// 		// 	break;
// 		// case 'Y':
// 		// 	need_chn_rotate = atoi(optarg);
// 		// 	printf("need_chn_rotate = %d\n", need_chn_rotate);
// 		// 	break;
// 		// case 'I':
// 		// 	need_ipu = atoi(optarg);
// 		// 	printf("need_ipu = %d\n", need_ipu);
// 		// 	break;
// 		// case 'P':
// 		// 	need_pym = atoi(optarg);
// 		// 	printf("need_pym = %d\n", need_pym);
// 		// 	break;
// 		// case 'D':
// 		// 	vps_dump = atoi(optarg);
// 		// 	printf("vps_dump = %d\n", vps_dump);
// 		// 	break;
// 		// case 'N':
// 		// 	need_md = atoi(optarg);
// 		// 	printf("need_md = %d\n", need_md);
// 		// 	break;
// 		case 'R':
// 			g_osd_flag = atoi(optarg);
// 			printf("g_osd_flag = %d\n", g_osd_flag);
// 			break;

// 		default:
// 			print_usage(argv[0]);
// 			break;
// 		}
// 	}
// }

int check_end(void)
{
	time_t now = time(NULL);
	// printf("Time info :: now(%ld), end_time(%ld) run time(%ld)!\n",
	// now, end_time, run_time);
	return !(now > end_time && run_time > 0);
}

// void intHandler(int dummy)
// {
// 	g_exit = 1;
// 	printf("rcv int signal\n");
// }

// int main(int argc, char *argv[])
// {

// 	if (argc < 2) {
// 		print_usage(argv[0]);
// 		printf("leave, World! \n");
// 		exit(1);
// 	}
//     parse_opts(argc, argv);

//     signal(SIGINT, intHandler);

//     if (sample_mode == 0) {
//         sample_singlepipe_5venc();
//     } else if (sample_mode == 1) {
//         sample_multpipe_multvenc();
//     } else if (sample_mode == 2) {
// 		sample_vdec_vot_test();
// 	} else if (sample_mode == 3) {
// 		sample_vdec_vps_venc_vot();
// 	}

//     return 0;
// }
