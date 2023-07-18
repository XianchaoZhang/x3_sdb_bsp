#define _GNU_SOURCE
#include <sched.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>
#include <fcntl.h>
#include <signal.h>

#include "com_api.h"
#include "server_config.h"
#include "server_core.h"
#include "hb_vio_interface.h"
#include "vio_func.h"

typedef struct vio_ctx_s {
	struct buf_list rawdata_buf;
	struct buf_list yuvdata_buf;
	pthread_t pid_yuv_process;
	pthread_t pid_raw_process;
	uint32_t run_yuv_process;
	uint32_t run_raw_process;
	uint32_t pipe_line;
	uint32_t raw_enable;
	uint32_t raw_buf_num;
	uint32_t yuv_enable;
	uint32_t yuv_channel;
	uint32_t yuv_buf_num;
	uint32_t gdc_rotation;
} vio_ctx_t;

vio_ctx_t vio_ctx;

static int get_data_func(uint32_t pipe_id, uint32_t data_type, hb_vio_buffer_t *buf)
{
	int ret = 0;
	int size = -1;

	ret = hb_vio_get_data(pipe_id, data_type, buf);

	if(ret < 0) {
		//printf("yuv dump failed.\n");
	} else {
		size = buf->img_addr.stride_size * buf->img_addr.height;
		vmon_dbg("yuv stride_size(%u) w x h (%u x %u)  size %d\n", buf->img_addr.stride_size, buf->img_addr.width, buf->img_addr.height, size);
	}

	return ret;
}

static void free_data_func(uint32_t pipe_id, hb_vio_buffer_t *buf)
{
	int ret = 0;
	ret = hb_vio_free_ipubuf(pipe_id, buf);
	if (ret <0) {
		vmon_err("vio buf free failed.\n");
	}
}

static int raw_yuv_dump(uint32_t pipe_id, hb_vio_buffer_t *sif_raw, hb_vio_buffer_t *isp_yuv)
{
	int ret = -1;
	uint32_t size = 0;

	ret = hb_vio_get_data(pipe_id, HB_VIO_SIF_RAW_DATA, sif_raw);
	/** raw */
	if (ret < 0) {
	// printf("raw dump failed.\n");
	} else {
		size = sif_raw->img_addr.stride_size * sif_raw->img_addr.height;
		vmon_dbg("raw stride_size(%u) w x h%u x %u  size %d\n", sif_raw->img_addr.stride_size, sif_raw->img_addr.width, sif_raw->img_addr.height, size);
	}

	return ret;
}

int get_yuv_pipe_info(uint32_t pipe_line, uint32_t channel,
	uint32_t *width, uint32_t *height)
{
	chn_img_info_t info;

	// get vio info
	memset(&info, 0x00, sizeof(chn_img_info_t));
	VIO_INFO_TYPE_E info_type = HB_VIO_INFO_MAX;
	switch (channel) {
	case HB_VIO_IPU_DS0_DATA:
		info_type = HB_VIO_IPU_DS0_IMG_INFO;
		break;
	case HB_VIO_IPU_DS1_DATA:
		info_type = HB_VIO_IPU_DS1_IMG_INFO;
		break;
	case HB_VIO_IPU_DS2_DATA:
		info_type = HB_VIO_IPU_DS2_IMG_INFO;
		break;
	case HB_VIO_IPU_DS3_DATA:
		info_type = HB_VIO_IPU_DS3_IMG_INFO;
		break;
	case HB_VIO_IPU_DS4_DATA:
		info_type = HB_VIO_IPU_DS4_IMG_INFO;
		break;
	case HB_VIO_IPU_US_DATA:
		info_type = HB_VIO_IPU_US_IMG_INFO;
		break;
	default:
		break;
	}
	if (info_type == HB_VIO_INFO_MAX) {
		printf("%s Invalid camera information.\n", __func__);
		return -1;
	}
	if (hb_vio_get_param(pipe_line, info_type, &info)) {
		printf("%s Failed to get camera information.\n", __func__);
		return -1;
	}
	if (info.buf_count <= 0 || info.format != HB_YUV420SP) {
		printf("%s Invalid IPU buffer count or format.\n", __func__);
		return -1;
	}

	*width = info.width;
	*height = info.height;

	return 0;
}

int get_raw_buf(struct buf_info *data)
{
	int ret = 0;
	struct buf_list_node *node = NULL;

	if (buf_list_empty(&vio_ctx.rawdata_buf)) {
		//vmon_err("-------buf is empty---------\n");
		return -1;
	} else {
		ret = buf_list_remove_node(&vio_ctx.rawdata_buf, &node);
		if (ret < 0) {
			//vmon_err("-------get buff fialed---------\n");
		} else {
			memcpy(data, &node->buf, sizeof(struct buf_info));
			free(node);
			node = NULL;
		}
	}

	return ret;
}

void free_raw_buf(struct buf_info *data)
{
#if ENABELCPY

#else
	//free_data_func(pipe_line, &data->buf);
#endif
}

int get_yuv_buf(struct buf_info *data)
{
	int ret = 0;
	struct buf_list_node *node = NULL;

	if (buf_list_empty(&vio_ctx.yuvdata_buf)) {
		//vmon_err("-------buf is empty---------\n");
		return -1;
	} else {
		ret = buf_list_remove_node(&vio_ctx.yuvdata_buf, &node);
		if (ret < 0) {
			//vmon_err("-------get buff fialed---------\n");
		} else {
			memcpy(data, &node->buf, sizeof(struct buf_info));
			free(node);
			node = NULL;
		}
	}

	return ret;
}

void free_yuv_buf(struct buf_info *data)
{
	if (data->buf_valib == 1) {
		free_data_func(data->pipe_line, &data->buf);
	} else {
		buf_free(data);
	}
}



static int get_raw_data(struct buf_info *data, uint32_t skip)
{
	int ret = -1;
	uint32_t temp = 0;
	uint32_t size = 0;
	uint32_t pipe_line = 0;

	hb_vio_buffer_t buf;
	hb_vio_buffer_t raw;

	get_raw_pipeline_info(&pipe_line);
	ret = raw_yuv_dump(pipe_line, &raw, &buf);
	if (ret < 0) {
		vmon_err("-----get raw failed------\n");
	} else {
		if (!skip) {
			size = raw.img_addr.stride_size * raw.img_addr.height;
			if (raw.img_addr.stride_size == raw.img_addr.width) {
				data->header_info.format = RAW_8;
			} else if (raw.img_addr.stride_size <= (raw.img_addr.width + raw.img_addr.width*3/10)) {
				data->header_info.format = RAW_10;
			} else if (raw.img_addr.stride_size <= (raw.img_addr.width + raw.img_addr.width*3/5)) {
				data->header_info.format = RAW_12;
			} else if (raw.img_addr.stride_size <= raw.img_addr.width + raw.img_addr.width) {
				data->header_info.format = RAW_14;
			} else if (raw.img_addr.stride_size <= (raw.img_addr.width + raw.img_addr.width*11/10)) {
				data->header_info.format = RAW_16;
			} else {
				ret = -1;
				goto err;
			}

			if (raw.img_addr.width % 16 != 0) {
				data->header_info.width = 16 * ((raw.img_addr.width / 16)  + 1);
			} else {
				data->header_info.width = raw.img_addr.width;
			}

			data->header_info.type = RAW_DATA;
			data->header_info.height = raw.img_addr.height;
			data->header_info.stride = raw.img_addr.stride_size;
			data->header_info.length = size;
			data->header_info.frame_id = raw.img_info.frame_id;
			data->header_info.frame_plane = 0;
			data->header_info.code_type = 1;

			for (temp = 0; temp < raw.img_info.planeCount; temp++) {
				data->ptr[temp] = malloc(data->header_info.length);
				if(data->ptr[temp] == NULL) {
					vmon_err("malloc size file!\n");
					ret = -1;
				} else {
					memcpy(data->ptr[temp], raw.img_addr.addr[temp], size);
				}
			}

		} else {
			ret = -1;
		}
		hb_vio_free_sifbuf(pipe_line, &raw);
	}
err:
	return ret;
}

int get_gdc_data(struct buf_info *data, hb_vio_buffer_t *dst_buf)
{
	int ret = 0;
	uint32_t need_gdc_fb = vio_ctx.gdc_rotation;

	ret = hb_vio_run_gdc(data->pipe_line, &data->buf, dst_buf, need_gdc_fb);
	if (ret < 0) {
		ret = -1;
	}
	return ret;
}

void free_gdc_data(uint32_t pipe_line, hb_vio_buffer_t *dst_buf)
{
	hb_vio_free_gdcbuf(pipe_line, dst_buf);
}


static int get_yuv_data(struct buf_info *data, uint32_t skip)
{
	uint32_t size = 0;
	int ret = -1;
	hb_vio_buffer_t buf;
	uint32_t pipe_line;
	uint32_t yuv_channel;

	get_yuv_pipeline_info(&pipe_line, &yuv_channel);
	ret = get_data_func(pipe_line, yuv_channel, &buf);
	if (ret < 0) {
		vmon_err("-------get yuv failed---------\n");
	} else {
		if (!skip) {
			size = buf.img_addr.stride_size * buf.img_addr.height;

			data->header_info.type = YUV_DATA;//yuv
			data->header_info.format = YUVNV12;//
			data->header_info.width = buf.img_addr.width;
			data->header_info.height = buf.img_addr.height;
			data->header_info.stride = buf.img_addr.stride_size;
			data->header_info.length = size + size/2;
			data->header_info.frame_id = buf.img_info.frame_id;
			data->header_info.frame_plane = 0;
			data->header_info.code_type = 1;
#if ENABELCPY
			data->ptr[0] = malloc(data->header_info.length);
			if(data->ptr[0] == NULL) {
				vmon_err("malloc size failed!\n");
				ret = -1;
			} else {
				memcpy(data->ptr[0], buf.img_addr.addr[0], size);
				memcpy(data->ptr[0] + size, buf.img_addr.addr[1], size/2);
			}
			data->buf_valib = 0;
#else
			memcpy(&data->buf, &buf, sizeof(hb_vio_buffer_t));
			data->buf_valib = 1;
			data->pipe_line = pipe_line;
#endif
		} else {
			free_data_func(pipe_line, &buf);
			return -1;
		}
#if ENABELCPY
		free_data_func(pipe_line, &buf);
#endif
	}
	return ret;
}

static uint32_t yuvbuf_save = 0;
static uint32_t check_state_buf(struct buf_list *data_buf)
{
	uint32_t buf_len = 0;
	uint32_t skip = 0;
	int ret = 0;
	struct buf_info data;

	buf_len = get_list_length(data_buf);
	vio_ctx.yuv_buf_num = get_yuv_serial_num();

	if ( vio_ctx.yuv_buf_num > 2) {
		if (yuvbuf_save == 0) {
			if (buf_len > vio_ctx.yuv_buf_num) {
				skip = 1;
				yuvbuf_save = 1;
			}
		} else {
			if (buf_len < 3) {
				yuvbuf_save = 0;
			} else {
				skip = 1;
			}
		}
	} else {
		if (buf_len > vio_ctx.yuv_buf_num) {
			ret = get_yuv_buf(&data);
			if (ret >= 0) {
				free_yuv_buf(&data);
			}
		}
	}

	return skip;
}

static void *get_yuv_process(void *param)
{
	int ret = 0;
	struct buf_info info;
	struct buf_list_node *node = NULL;
	uint32_t skip = 0;

// pthread set to cpu
	cpu_set_t mask;
	CPU_ZERO(&mask); // init
        CPU_SET(0, &mask); // set

	if (pthread_setaffinity_np(pthread_self(), sizeof(mask), &mask) < 0) {
		vmon_err("sched_setaffinity");
	}

	memset(&info, 0, sizeof(struct buf_info));
	while(vio_ctx.run_yuv_process) {
		skip = check_state_buf(&vio_ctx.yuvdata_buf);
		ret = get_yuv_data(&info, skip);
		if (ret < 0) {
			//vmon_err("[%s--%d]-------get yuv failed---------\n");
		} else {
			if (!skip) {
				node = malloc(sizeof(struct buf_list_node));
				if (node == NULL) {
					vmon_err("malloc size failed!\n");
					#if ENABELCPY
						buf_free(&info);
					#else
						free_data_func(info.pipe_line, &info.buf);
					#endif
				} else {
					memcpy(&node->buf, &info, sizeof(struct buf_info));
					ret = buf_list_add_node(&vio_ctx.yuvdata_buf, &node);
					if (ret < 0) {
						#if ENABELCPY
							buf_free(&node->buf);
						#else
							free_data_func(info.pipe_line, &node->buf.buf);
						#endif
						free(node);
					}
					node = NULL;
				}
			}
		}
	}
	pthread_exit(NULL);
}

static int start_get_yuv_process(void)
{
	int ret = 0;

	ret = buf_list_init(&vio_ctx.yuvdata_buf);
	vio_ctx.run_yuv_process = 1;
	if (pthread_create(&vio_ctx.pid_yuv_process, NULL, get_yuv_process, NULL) != 0) {
		vmon_info("start send data!\n");
		vio_ctx.run_yuv_process = 0;
		buf_list_deinit(&vio_ctx.yuvdata_buf);
		return -1;
	}

	return 0;
}

static void stop_get_yuv_process(void)
{
	vio_ctx.run_yuv_process = 0;
	if (vio_ctx.pid_yuv_process) {
		pthread_join(vio_ctx.pid_yuv_process, NULL);
		vio_ctx.pid_yuv_process = 0;
		vmon_info("stop_get_yuv_process\n");
	}
	buf_list_deinit(&vio_ctx.yuvdata_buf);
}

static uint32_t rawbuf_save = 0;
static uint32_t check_state_rawbuf(struct buf_list *data_buf)
{
	uint32_t buf_len = 0;
	uint32_t skip = 0;
	int ret = 0;
	struct buf_info data;

	buf_len = get_list_length(data_buf);
	vio_ctx.raw_buf_num = get_raw_serial_num();

	if ( vio_ctx.raw_buf_num > 2) {
		if (rawbuf_save == 0) {
			if (buf_len > vio_ctx.raw_buf_num) {
				skip = 1;
				rawbuf_save = 1;
			}
		} else {
			if (buf_len < 3) {
				rawbuf_save = 0;
			} else {
				skip = 1;
			}
		}
	} else {
		if (buf_len > vio_ctx.raw_buf_num) {
			skip = 1;
		}
#if 0
		if (buf_len > vio_ctx.yuv_buf_num) {
			ret = get_raw_buf(&data);
			if (ret >= 0) {
				free_raw_buf(&data);
			}
		}
#endif
	}

	return skip;
}

static void *get_raw_process(void *param)
{
	int ret = 0;
	uint32_t count = 0;
	uint32_t skip = 0;
	struct buf_info info;
	struct buf_list_node *node = NULL;
	uint32_t buf_len = 0;

	while(vio_ctx.run_raw_process) {
		memset(&info, 0, sizeof(struct buf_info));

		skip = check_state_rawbuf(&vio_ctx.rawdata_buf);
		ret = get_raw_data(&info, skip);
		if (ret < 0) {
			buf_free(&info);
			//vmon_info("[%s--%d]--get raw failed--\n", __func__, __LINE__);
		} else {
			if (!skip) {
				node = malloc(sizeof(struct buf_list_node));
				if (node == NULL) {
					vmon_err("[%s--%d]--malloc failed--\n", __func__, __LINE__);
					buf_free(&info);
				} else {
					memcpy(&node->buf, &info, sizeof(struct buf_info));
					ret = buf_list_add_node(&vio_ctx.rawdata_buf, &node);
					if (ret < 0) {
						buf_free(&node->buf);
						free(node);
					}
					node = NULL;
				}
			}
		}
	}
	pthread_exit(NULL);
}

static int start_get_raw_process(void)
{
	int ret = 0;

	ret = buf_list_init(&vio_ctx.rawdata_buf);
	vio_ctx.run_raw_process = 1;
	if (pthread_create(&vio_ctx.pid_raw_process, NULL, get_raw_process, NULL) != 0) {
		vmon_dbg("<%s: %d>start send data!\n", __func__, __LINE__);
		vio_ctx.run_raw_process = 0;
		buf_list_deinit(&vio_ctx.rawdata_buf);
		return -1;
	}

        return 0;
}

static void stop_get_raw_process(void)
{
	vio_ctx.run_raw_process = 0;
	if (vio_ctx.pid_raw_process) {
		pthread_join(vio_ctx.pid_raw_process, NULL);
		vio_ctx.pid_raw_process = 0;
	}
	buf_list_deinit(&vio_ctx.rawdata_buf);
}

int init_process(uint32_t gdc_rotation)
{
	int ret = 0;

	memset(&vio_ctx, 0, sizeof(vio_ctx_t));
	vio_ctx.gdc_rotation = gdc_rotation;
	raw_config_info(&vio_ctx.raw_enable, &vio_ctx.pipe_line);
	yuv_config_info(&vio_ctx.yuv_enable, &vio_ctx.pipe_line, &vio_ctx.yuv_channel);
	if (vio_ctx.raw_enable == 1) {
		vmon_dbg("start raw process!\n");
		ret = start_get_raw_process();
		if (ret < 0)
			goto start_raw_err;
	}
	if (vio_ctx.yuv_enable == 1) {
		vmon_dbg("start yuv process!\n");
		ret = start_get_yuv_process();
		if (ret < 0)
			goto start_yuv_err;
	}

	return ret;

start_yuv_err:
	stop_get_raw_process();
start_raw_err:
	return ret;

}

void deinit_process(void)
{
	stop_get_raw_process();
	stop_get_yuv_process();
}
