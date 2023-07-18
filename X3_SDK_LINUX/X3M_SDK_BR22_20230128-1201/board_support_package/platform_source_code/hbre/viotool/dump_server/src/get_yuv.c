#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <fcntl.h>
#include <signal.h>
#include <pthread.h>

#define __USE_GNU
#include<sched.h>
#include<ctype.h>

#include "server_cmd.h"
#include "server_core.h"
#include "common.h"
#include "com_api.h"
#include "vio_func.h"
#include "code_func.h"
#include "list_func.h"

pthread_t pid_yuv;
static uint32_t run_yuv = 0;

/**
 * @brief send yuv data to pc
 *
 * @param info
 * @param ptr y channel data info
 * @param size y channel size
 * @param ptr1 uv channel data info
 * @param size1 uv channel size
 * @return
 *   @retval
 */
static void send_yuvdata(struct cmd_header *info, void *ptr,
	uint32_t size, void *ptr1, uint32_t size1)
{
	char *out = NULL;
	uint32_t count = 0;
	int ret = 0;

	vmon_dbg("length = %d, type %d, format %d, weight %d, height %d, stride %d, code_type %d \n", info->length, info->type, info->format, info->width, info->height, info->stride, info->code_type);

	acquire_mutex();
	ret = send_data(info, sizeof(struct cmd_header));
	if (ret < 0) {
		goto err;
	}

	if ((ptr != NULL) && (size != 0)) {
		ret = send_data(ptr, size);
		if (ret < 0) {
			goto err;
		}
	}

	if ((ptr1 != NULL) && (size1 != 0)) {
		ret = send_data(ptr1, size1);
		if (ret < 0) {
			goto err;
		}
	}

	release_mutex();
	return;
err:
	vmon_dbg("send_data error. \n");
	err_handler(0);
	release_mutex();
}

static int set_policy_max(void)
{
	int ret = 0;
	pthread_attr_t attr;
	struct sched_param sched;
	int policy = 0;

	// get pthread info
	ret = pthread_attr_init(&attr);
	if (!ret) {
		vmon_err("get pthread info failed !\n");
		goto set_err;
	}

	// get pthread policy
	ret = pthread_attr_getschedpolicy(&attr, &policy);
	if ((policy == SCHED_FIFO) || (policy == SCHED_RR)) {
		//ret = pthread_attr_setschedpolicy(attr, policy);
	}

	ret = pthread_attr_getschedparam(&attr, &sched);
	if (!ret) {
		sched.sched_priority = 0;
		ret = pthread_attr_setschedparam(&attr, &sched);
	}
set_err:
	return ret;
}

static void *send_yuv_data(void *param)
{
	int ret = 0;
	struct buf_info info;
	hb_vio_buffer_t buf;
	uint32_t id = 0;
	uint32_t size = 0;

	sleep(0.1);
	memset(&info, 0, sizeof(struct buf_info));
	while(run_yuv) {
		memset(&info, 0, sizeof(struct buf_info));
		ret = -1;

		if (send_yuv_enable()) {
			ret = get_yuv_buf(&info);
		} else if (send_jepg_enable()) {
			ret = get_jepg_buf(&info);
		} else if (send_video_enable()) {
			ret = get_video_buf(&info);
		}

		if (ret < 0) {
			usleep(3*1000);
		} else {
			id++;
			vmon_dbg("yuv send count id is %d!\n", id);
			print_timestamp();//time

			if (send_yuv_enable() == 2) { //enable gdc func
				ret = get_gdc_data(&info, &buf);
				if (ret >= 0) {
					size = buf.img_addr.stride_size * buf.img_addr.height;
					info.header_info.width = buf.img_addr.width;
					info.header_info.height = buf.img_addr.height;
					info.header_info.stride = buf.img_addr.stride_size;
					info.header_info.length = size + size/2;
					send_yuvdata(&info.header_info, buf.img_addr.addr[0], size,
						buf.img_addr.addr[1], size/2);
					//free gdc buf
					free_gdc_data(info.pipe_line, &buf);
				}
			} else {
				if (info.buf_valib == 1) {
					size = info.buf.img_addr.stride_size * info.buf.img_addr.height;
					send_yuvdata(&info.header_info, info.buf.img_addr.addr[0], size,
						info.buf.img_addr.addr[1], size/2);
				} else {
					send_yuvdata(&info.header_info, info.ptr[0], info.header_info.length, info.ptr[1], 0);
				}
			}

			print_timestamp();//time
			free_yuv_buf(&info);
		}
	}
	pthread_exit(NULL);
}

int start_send_yuv_pic(void)
{
	run_yuv = 1;
	if (pthread_create(&pid_yuv, NULL, send_yuv_data, NULL) != 0) {
		vmon_dbg("start send data!\n");
		run_yuv = 0;
		return -1;
	}

	return 0;
}

void stop_send_yuv_pic(void)
{
	run_yuv = 0;
	if (pid_yuv) {
		pthread_join(pid_yuv, NULL);
		pid_yuv = 0;
	}
	vmon_dbg("stop send data!\n");
}
