/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <signal.h>
#include <getopt.h>
#include <time.h>
#include <pthread.h>
#include <string.h>

#include "hb_vps_api.h"
/*#include "sif.h"*/
#include "hb_sys.h"
#include "vps_common.h"

#define BIT(n)  (1UL << (n))

int g_time_duration = 5;
int g_group = 1;
int g_exit = 0;
int g_venc_flag = 1;
int loop;
int g_save_flag = 0;

void print_usage(const char *prog)
{
	printf("Usage: %s \n", prog);
	puts("  -t --time\n"
		"  -G --group num 1:group0 3:group0 group1 7: group0~1"
		"  -l --loop"
		"  -\n");
	exit(1);
}

void parse_opts(int argc, char *argv[])
{
	while (1) {
		static const char short_options[] =
			"t:G:l:";
		static const struct option long_options[] = {
			{"time_duration", 1, 0, 't'},
			{"g_group", 1, 0, 'G'},
			{"loop", 1, 0, 'l'},
			{NULL, 0, 0, 0},
		};

		int cmd_ret;

		cmd_ret =
		    getopt_long(argc, argv, short_options, long_options, NULL);

		if (cmd_ret == -1)
			break;

		switch (cmd_ret) {			
		case 'l':
			loop = atoi(optarg);
			printf("loop = %d\n", loop);
			break;				
		case 't':
			g_time_duration = atoi(optarg);
			printf("g_time_duration = %d\n", g_time_duration);
			break;	
		case 'G':
			g_group = atoi(optarg);
			printf("g_group = %d\n", g_group);
			break;		
		default:
			print_usage(argv[0]);
			break;
		}
	}
}
void intHandler(int dummy)
{
	g_exit = 1;
	printf("intHandler signal\n");
}
void vps_init(int grp_id)
{
	int ret;
	VPS_GRP_ATTR_S grp_attr;
	VPS_CHN_ATTR_S chn_attr;
	VPS_PYM_CHN_ATTR_S pym_chn_attr;
	
	ret = HB_SYS_SetVINVPSMode(grp_id, VIN_ONLINE_VPS_ONLINE);
	if (ret)
		printf("HB_SYS_SetVINVPSMode error\n");

	memset(&grp_attr, 0, sizeof(VPS_GRP_ATTR_S));
	grp_attr.maxW = 1920;
	grp_attr.maxH = 1080;
	ret = HB_VPS_CreateGrp(grp_id, &grp_attr);
	if (ret) {
		printf("HB_VPS_CreateGrp error!!!\n");
	} else {
		printf("created a group ok:grp_id = %d\n", grp_id);
	}	
	
	memset(&chn_attr, 0, sizeof(VPS_CHN_ATTR_S));
	chn_attr.enScale = 1;
	chn_attr.width = 1920;
	chn_attr.height = 1080;
	chn_attr.frameDepth = 6;	
	ret = HB_VPS_SetChnAttr(grp_id, 6, &chn_attr);
	if (ret) {
		printf("HB_VPS_SetChnAttr error!!!\n");
	} else {
		printf("set chn Attr ok: GrpId = %d, chn_id = %d\n",
								grp_id, 6);
	}
	HB_VPS_EnableChn(grp_id, 6);
	
	memset(&pym_chn_attr, 0, sizeof(VPS_PYM_CHN_ATTR_S));
	pym_chn_attr.timeout = 2000;
	pym_chn_attr.ds_layer_en = 23;
	pym_chn_attr.us_layer_en = 0;
	pym_chn_attr.frame_id = 0;
	pym_chn_attr.frameDepth = 6;
	ret = HB_VPS_SetPymChnAttr(grp_id, 6, &pym_chn_attr);
	if (ret) {
		printf("HB_VPS_SetPymChnAttr error!!!\n");
	} else {
		printf("HB_VPS_SetPymChnAttr ok: grp_id = %d g_pym_chn = %d\n",
								grp_id, 6);
	}
	HB_VPS_EnableChn(grp_id, 6);
}
	
int main(int argc, char *argv[])
{
	int grp_id = 0;
	int ret;
	int i;

	parse_opts(argc, argv);

	if (argc < 2) {
		print_usage(argv[0]);
		printf("exit \n");
		exit(1);
	}	
	
	if (g_venc_flag > 0) {
		ret = dual_singlepipe_venc_init();
		if (ret < 0) {
			printf("dual_singlepipe_venc_init error! do venc_deinit \n");
			dual_singlepipe_venc_deinit();
			return ret;
		}
	}
		
	for (grp_id = 0; grp_id < 6; grp_id ++) {
		if (BIT(grp_id) & g_group) {
			vps_init(grp_id);			
			ret = HB_VPS_StartGrp(grp_id);
			if (ret) {
				printf("HB_VPS_StartGrp error!!!\n");
			} else {
				printf("start grp ok: grp_id = %d\n", grp_id);
			}
		}
	}
	if (g_venc_flag > 0) {
		 dual_venc_pthread_start();
	}
	signal(SIGINT, intHandler);
	
	while(loop) {
		loop--;
		if (loop%50==0)
			printf("============1======== loop: %d\n", loop);
		if (g_exit == 1) break;
		usleep(100000);
	}	
	
	if (g_venc_flag > 0) {
		dual_singlepipe_venc_deinit();
	}
	for (grp_id = 0; grp_id < 6; grp_id++) {
		if (BIT(grp_id) & g_group) {
			ret = HB_VPS_StopGrp(grp_id);
			if (ret) {
				printf("HB_VPS_StopGrp error!!!\n");
			} else {
				printf("HB_VPS_StopGrp ok: grp_id = %d\n", grp_id);
			}
			ret = HB_VPS_DestroyGrp(grp_id);
			if (ret) {
				printf("HB_VPS_DestroyGrp error!!!\n");
			} else {
				printf("HB_VPS_DestroyGrp ok: grp_id = %d\n", grp_id);
			}
		}
	}

	printf("=============vps test success===========\n");

	return 0;
}
