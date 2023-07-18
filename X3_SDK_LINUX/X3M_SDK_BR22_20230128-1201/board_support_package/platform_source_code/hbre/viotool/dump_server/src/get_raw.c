#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>
#include <sys/types.h>

#include "server_cmd.h"
#include "server_core.h"
#include "common.h"
#include "com_api.h"
#include "vio_func.h"
#include "list_func.h"


pthread_t pid_raw;
static uint32_t run_raw = 0;

static void send_rawdata(struct cmd_header *info, void *ptr)
{
	char *out = NULL;
	int ret = 0;
	int len = 0;

	if ((info == NULL) || (ptr == NULL)) {
		vmon_dbg("param is error.\n");
		return;
	}

	vmon_dbg("length = %d, type %d, format %d, weight %d, height %d, stride %d, code_type %d \n", info->length, info->type, info->format, info->width, info->height, info->stride, info->code_type);

	acquire_mutex();
	ret = send_data(info, sizeof(struct cmd_header));
	if (ret < 0) {
		goto err;
	}

	//remove frame_id
	out = ptr;
	*out = 0;
	*(out + 1) = 0;
	ret = send_data(ptr, info->length);
	if (ret < 0) {
		goto err;
	}

	release_mutex();
	return;
err:
	vmon_err("close socket fd.\n");
	err_handler(0);
	release_mutex();
}

static void *send_raw_data(void *param)
{
	int ret = 0;
	struct buf_info info;
	uint32_t id = 0;
	uint32_t temp = 0;

	while(run_raw) {
		memset(&info, 0, sizeof(struct buf_info));

		if (send_raw_enable()) {
			//printf("[%s--%d]-------------------------\n", __func__, __LINE__);
			ret = get_raw_buf(&info);
		} else {
			ret = -1;
		}

		if (ret < 0) {
			//printf("[%s--%d]-----get raw failed------\n", __func__, __LINE__);
			usleep(30*1000);
		} else {
			id++;
			vmon_dbg("frame id is %d! \n", id);
//send data
			info.header_info.frame_plane = 0;
			send_rawdata(&info.header_info, info.ptr[0]);
			info.header_info.frame_plane = 1;
			send_rawdata(&info.header_info, info.ptr[1]);
			info.header_info.frame_plane = 2;
			send_rawdata(&info.header_info, info.ptr[2]);
			info.header_info.frame_plane = 3;
			send_rawdata(&info.header_info, info.ptr[3]);
			print_timestamp();//time
			for (temp = 0; temp < 4; temp++) {
				if (info.ptr[temp]) {
					free(info.ptr[temp]);
					info.ptr[temp] = NULL;
				}
			}
		}
	}
	return NULL;
}

int start_send_raw_pic(void)
{
	run_raw = 1;
	if (pthread_create(&pid_raw, NULL, send_raw_data, NULL) != 0) {
		vmon_info("start send raw data!\n");
		return -1;
	}

	return 0;
}

void stop_send_raw_pic(void)
{
	run_raw = 0;
	if (pid_raw) {
		pthread_join(pid_raw, NULL);
		pid_raw = 0;
	}
	vmon_info("stop send raw data!\n");
}
