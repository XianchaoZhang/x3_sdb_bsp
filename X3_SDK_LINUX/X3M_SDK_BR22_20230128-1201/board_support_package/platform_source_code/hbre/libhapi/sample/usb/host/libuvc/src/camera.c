/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * camera.c
 *	camera function
 *
 * Copyright (C) 2019 Horizon Robotics, Inc.
 *
 * Contact: jianghe xu<jianghe.xu@horizon.ai>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <assert.h>

#include <linux/usb/ch9.h>
#include <linux/usb/video.h>
#include <linux/videodev2.h>

#include "camera.h"
#include "v4l2.h"
#include "utils.h"

/******************************************
 *      Function Declaration              *
 *****************************************/
#define CLEAR(x) memset(&(x), 0, sizeof(x))

static void *streaming_loop(void *arg);

/******************************************
 *      Function Implementation           *
 *****************************************/
camera_t *camera_open(const char *devname)
{
	camera_t *cam;
	struct v4l2_device *dev;

	trace_in();

	cam = calloc(1, sizeof(camera_t));
	if (!cam)
		return NULL;

	dev = v4l2_open(devname);

	if (!dev)
		goto fail;

	memset(&cam->params, 0, sizeof(camera_param_t));
	cam->name = devname;
	cam->v4l2_dev = dev;
	cam->nbufs = 4;			/* 4 buffers in default */
	cam->io_method = IO_MMAP;	/* io mmap as default */
	cam->quit = 0;
	cam->cb = NULL;
	cam->user_args = NULL;
	cam->fmt_enum = NULL;

	trace_out();

	return cam;

fail:
	free(cam);

	return NULL;
}

int camera_enum_format(camera_t *cam, format_enums *fmt_enum, int show_info)
{
	struct v4l2_format_desc *format;
	struct v4l2_frame_desc *frame;
	struct v4l2_ival_desc *ival;

	int ret, cnt;
	int i = 0, j = 0, k = 0;

	trace_in();

	if (!cam || !cam->v4l2_dev)
		return -1;

	if (!cam->fmt_enum) {
		cam->fmt_enum = calloc(1, sizeof(format_enums));
		if (!cam->fmt_enum)
			return -ENOMEM;

		cnt = cam->v4l2_dev->fmt_cnt;
		cam->fmt_enum->fmt_desc = calloc(cnt, sizeof(format_desc));
		cam->fmt_enum->format_count = cnt;
		assert(cam->fmt_enum->fmt_desc != NULL);

		if (list_empty(&cam->v4l2_dev->formats)) {
			printf("some error happen. formats enums is empty.");
			return -1;
		}

		if (show_info)
			printf("enum format:\n");

		list_for_each_entry(format, &cam->v4l2_dev->formats, list) {
			cam->fmt_enum->fmt_desc[i].pixelformat =
				format->pixelformat;
			strncpy(cam->fmt_enum->fmt_desc[i].description,
					format->description, 32);
			if (show_info)
				printf("\t%d.%s\n", i, format->description);

			cam->fmt_enum->fmt_desc[i].frm_desc =
				calloc(format->frm_cnt, sizeof(frame_desc));
			cam->fmt_enum->fmt_desc[i].frame_count = format->frm_cnt;
			assert(cam->fmt_enum->fmt_desc[i].frm_desc != NULL);

			j = 0;
			list_for_each_entry(frame, &format->frames, list) {
				cam->fmt_enum->fmt_desc[i].frm_desc[j].width =
					frame->max_width;
				cam->fmt_enum->fmt_desc[i].frm_desc[j].height =
					frame->max_height;
				if (show_info)
					printf("\t\t%d %dx%d\n", j,
							frame->max_width,
							frame->max_height);

				cam->fmt_enum->fmt_desc[i].frm_desc[j].ival =
					calloc(frame->ivals_cnt, sizeof(ival_desc));
				assert(cam->fmt_enum->fmt_desc[i].frm_desc[j].ival != NULL);
				cam->fmt_enum->fmt_desc[i].frm_desc[j].ival_cnt = frame->ivals_cnt;

				k = 0;
				list_for_each_entry(ival, &frame->ivals, list) {
					cam->fmt_enum->fmt_desc[i].frm_desc[j].ival[k].min = ival->min;
					cam->fmt_enum->fmt_desc[i].frm_desc[j].ival[k].max = ival->max;
					cam->fmt_enum->fmt_desc[i].frm_desc[j].ival[k].step = ival->step;

					if (show_info) {
						printf("\t\t\t%d min-%u/%u max-%u/%u step-%u/%u\n", k,
							cam->fmt_enum->fmt_desc[i].frm_desc[j].\
							ival[k].min.numerator,
							cam->fmt_enum->fmt_desc[i].frm_desc[j].\
							ival[k].min.denominator,
							cam->fmt_enum->fmt_desc[i].frm_desc[j].\
							ival[k].max.numerator,
							cam->fmt_enum->fmt_desc[i].frm_desc[j].\
							ival[k].max.denominator,
							cam->fmt_enum->fmt_desc[i].frm_desc[j].\
							ival[k].step.numerator,
							cam->fmt_enum->fmt_desc[i].frm_desc[j].\
							ival[k].step.denominator);
					}
					k++;
				}
				j++;
			}
			i++;
		}
	}

	*fmt_enum = *cam->fmt_enum;

	trace_out();

	return 0;
}

int camera_show_format(camera_t *cam)
{
	int i, j, k;
	ival_desc* ival = NULL;
	trace_in();

	if (!cam || !cam->v4l2_dev)
		return -1;

	if (!cam->fmt_enum)
		return -1;

	printf("enum formats:\n");
	for (i = 0; i < cam->fmt_enum->format_count; i++) {
		printf("\t%d.%s\n", i, cam->fmt_enum->fmt_desc[i].description);
		for (j = 0; j < cam->fmt_enum->fmt_desc[i].frame_count; j++) {
			printf("\t\t%d %dx%d\n", j,
				cam->fmt_enum->fmt_desc[i].frm_desc[j].width,
				cam->fmt_enum->fmt_desc[i].frm_desc[j].height);

			ival = cam->fmt_enum->fmt_desc[i].frm_desc[j].ival;
			for (k = 0; k < cam->fmt_enum->fmt_desc[i].frm_desc[j].ival_cnt; k++) {
				printf("\t\t\tmin-%.2f(FPS), max-%.2f(FPS), step-%.2f(FPS)\n", k,
					((float)ival[k].max.denominator)/((float)ival[k].max.numerator),
					((float)ival[k].min.denominator)/((float)ival[k].min.numerator),
					((float)ival[k].step.denominator)/((float)ival[k].step.numerator));
			}
		}
	}

	trace_out();

	return 0;
}

int camera_set_params(camera_t *cam, camera_param_t *params)
{
	struct v4l2_format fmt;
	int r;

	trace_in();

	if (!cam || !cam->v4l2_dev || !params)
		return -1;

	/*
	 * Try to set the default format at the V4L2 video capture
	 * device as requested by the user.
	 */
	CLEAR(fmt);
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width = params->width;
	fmt.fmt.pix.height = params->height;
	fmt.fmt.pix.field = V4L2_FIELD_ANY;
	switch (params->fcc) {
	case FCC_YUY2:
		fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
		fmt.fmt.pix.sizeimage = params->width * params->height * 2;
		break;
	case FCC_NV12:
		fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;
		fmt.fmt.pix.sizeimage = params->width * params->height * 1.5;
		break;
	case FCC_MJPEG:
		fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
		fmt.fmt.pix.sizeimage = params->width * params->height * 1.5 / 2.0;
		break;
	case FCC_H264:
	case FCC_H265:
		fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_H264;
		fmt.fmt.pix.sizeimage = params->width * params->height * 1.5 / 2.0;
		break;
	default:
		printf("%s: Not support format (%d)\n", __func__, params->fcc);
		return -1;
	}

	r = v4l2_set_format(cam->v4l2_dev, &fmt);

	/* set frame rate */
	if (!r && params->fps)
		r = v4l2_set_frame_rate(cam->v4l2_dev, params->fps);

	cam->params = *params;

	if (r)
		printf(">>>set video format: %s@%dx%d(%dfps)\n",
				fcc_format_to_string(params->fcc),
				params->width, params->height,
				params->fps);

	trace_out();

	return r;
}

int camera_set_format(camera_t *cam, int format_index, int frame_index, int fps)
{
	camera_param_t params;
	int r;

	trace_in();

	if (!cam || !cam->v4l2_dev || !cam->fmt_enum
			|| !cam->fmt_enum->fmt_desc || !cam->fmt_enum->fmt_desc->frm_desc) {
		printf("error in %s(%d)\n", __func__, __LINE__);
		return -1;
	}

	if (format_index > cam->fmt_enum->format_count) {
		printf("error in %s(%d)\n", __func__, __LINE__);
		return -1;
	}

	if (frame_index > cam->fmt_enum->fmt_desc[format_index].frame_count) {
		printf("error in %s(%d)\n", __func__, __LINE__);
		return -1;
	}

	params.fcc = pixelformat_to_fcc(cam->fmt_enum->
			fmt_desc[format_index].pixelformat);
	params.width = cam->fmt_enum->
		fmt_desc[format_index].frm_desc[frame_index].width;
	params.height = cam->fmt_enum->
		fmt_desc[format_index].frm_desc[frame_index].height;
	/* 0 means using default fps */
	params.fps = fps;
	r = camera_set_params(cam, &params);
	if (r < 0) {
		printf("error in %s(%d)\n", __func__, __LINE__);
		return -1;
	}

	trace_out();

	return r;
}

int camera_set_framerate(camera_t *cam, int fps)
{
	int r;

	trace_in();

	if (!cam || !cam->v4l2_dev)
		return -1;

	r = v4l2_set_frame_rate(cam->v4l2_dev, fps);

	trace_out();

	return r;
}

int camera_start_streaming(camera_t *cam,
		camera_frame_callback_t *cb,
		void *user_args)
{
	enum v4l2_memory io_mem;
	int i, r;

	trace_in();

	if (!cam || !cam->v4l2_dev)
		return -1;

	/* request buffer */
	switch (cam->io_method) {
	case IO_MMAP:
		io_mem = V4L2_MEMORY_MMAP;
		break;
	case IO_USERPTR:
		io_mem = V4L2_MEMORY_USERPTR;
		break;
	case IO_OVERLAY:
		io_mem = V4L2_MEMORY_OVERLAY;
		break;
	case IO_DMABUF:
		io_mem = V4L2_MEMORY_DMABUF;
		break;
	default:
		printf("io_method(%d), only support mmap and user ptr now\n",
				cam->io_method);
		return -1;
	}

	if (io_mem != V4L2_MEMORY_MMAP) {
		printf("only support mmap io method currently!\n");
		return -1;
	}

	r = v4l2_alloc_buffers(cam->v4l2_dev, io_mem, cam->nbufs);
	if (r < 0)
		return -1;

	/* if io_mmap, needs to mmap v4l2 buffer to user space */
	if (io_mem == V4L2_MEMORY_MMAP) {
		r = v4l2_mmap_buffers(cam->v4l2_dev);
		if (r < 0)
			goto fail1;
	}

	/* queue all allocated buffers */
	for (i = 0; i < cam->v4l2_dev->buffers.nbufs; ++i) {
		struct video_buffer buf = {
			.index = i,
			.size = cam->v4l2_dev->buffers.buffers[i].size,
		};

		r = v4l2_queue_buffer(cam->v4l2_dev, &buf);
		if (r < 0) {
			printf("v4l2_queue_buffer failed\n");
			goto fail1;
		}
	}

	/** if user set callback function, launch a thread to grab frames.
	 * And callback function to notify frames to caller.
	 */
	if (cb) {
		cam->quit = 0;
		r = pthread_create(&cam->cam_pid, NULL,
				streaming_loop, cam);
		if (r < 0) {
			printf("%s pthread create failed\n", __func__);
			goto fail1;
		}

		cam->cb = cb;
		cam->user_args = user_args;
	}

	/* stream on */
	r = v4l2_stream_on(cam->v4l2_dev);
	if (r < 0)
		goto fail1;

	trace_out();

	return 0;

fail1:
	v4l2_free_buffers(cam->v4l2_dev);

	return r;
}

int camera_grab_stream(camera_t *cam)
{
	trace_in();

	// TODO: current just support callback method,
	// grab method will be added later

	trace_out();

	return 0;
}

int camera_stop_streaming(camera_t *cam)
{
	int r;

	trace_in();

	if (!cam || !cam->v4l2_dev)
		return -1;

	/* if has user callback, needs to join the streaming_loop thread */
	if (cam->cb && cam->cam_pid) {
		cam->quit = 1;
		r = pthread_join(cam->cam_pid, NULL);
		if (r < 0) {
			printf("streaming_loop pthread join failed\n");
			return -1;
		}
	}

	/* stop streaming */
	r = v4l2_stream_off(cam->v4l2_dev);
	if (r < 0)
		return -1;

	/* release v4l2 buffers */
	r = v4l2_free_buffers(cam->v4l2_dev);
	if (r < 0)
		return -1;

	trace_out();

	return 0;
}

void camera_close(camera_t *cam)
{
	int i, j;

	if (!cam || !cam->v4l2_dev)
		return;

	if (cam->fmt_enum) {
		if (cam->fmt_enum->fmt_desc) {
			for (i = 0; i < cam->fmt_enum->format_count; i++) {
				if (cam->fmt_enum->fmt_desc[i].frm_desc) {
					for (j = 0; j < cam->fmt_enum->fmt_desc[i].frame_count; j++) {
						if (cam->fmt_enum->fmt_desc[i].frm_desc[j].ival)
							free(cam->fmt_enum->fmt_desc[i].frm_desc[j].ival);
					}
					free(cam->fmt_enum->fmt_desc[i].frm_desc);
				}
			}
			free(cam->fmt_enum->fmt_desc);
		}

		free(cam->fmt_enum);
		cam->fmt_enum = NULL;
	}

	v4l2_close(cam->v4l2_dev);
	free(cam);
}

static void *streaming_loop(void *arg)
{
	camera_t *cam = (camera_t *)arg;
	struct v4l2_device *v4l2_dev;
	struct video_buffer buf = {0, };
	struct video_frame frame = {0, };
	struct timeval tv;
	fd_set fds;
	int r;

	trace_in();

	if (!cam || !cam->v4l2_dev)
		return (void *)0;

	v4l2_dev = cam->v4l2_dev;
	while (!cam->quit)
	{
		FD_ZERO(&fds);
		FD_SET(v4l2_dev->fd, &fds);

		/* timeout */
		tv.tv_sec = 5;
		tv.tv_usec = 0;

		r = select(v4l2_dev->fd + 1, &fds, NULL, NULL, &tv);
		if (-1 == r) {
			printf("select error %d, %s\n", errno, strerror(errno));
			if (EINTR == errno)
				continue;
			break;
		}

		if (0 == r) {
			printf("select timeout\n");
			/* don't break util cam->quit */
			continue;
		}

		/* dequeue a (filled) buffer from the video device */
		r = v4l2_dequeue_buffer(v4l2_dev, &buf);
		if (r < 0) {
			printf("dequeue buffer from video device failed\n");
			break;
		}

		frame.fcc = cam->params.fcc;
		frame.width = cam->params.width;
		frame.height = cam->params.height;
		frame.length = buf.bytesused;
		frame.mem = v4l2_dev->buffers.buffers[buf.index].mem;

		/** callback function to process video frames.
		 * Attention: this buffer needs to be handled with memcpy
		 * (to target queue/display etc...), as after callback,
		 * video frame will be returned to v4l2 module.
		 */
		if (cam->cb)
			cam->cb(&frame, cam->user_args);

		/* queue buffer back into the video device */
		r = v4l2_queue_buffer(v4l2_dev, &buf);
		if (r < 0) {
			printf("queue buffer back into video device failed\n");
			break;
		}
	}

	trace_out();

	return (void *)0;
}

fcc_format pixelformat_to_fcc(unsigned int pixelformat)
{
	fcc_format fcc;

	switch (pixelformat) {
	case V4L2_PIX_FMT_YUYV:
		fcc = FCC_YUY2;
		break;
	case V4L2_PIX_FMT_NV12:
		fcc = FCC_NV12;
		break;
	case V4L2_PIX_FMT_MJPEG:
		fcc = FCC_MJPEG;
		break;
	case V4L2_PIX_FMT_H264:
		fcc = FCC_H264;
		break;
	default:
		fcc = FCC_INVALID;
		break;
	}

	return fcc;
}

char *fcc_format_to_string(fcc_format fcc)
{
	switch (fcc) {
	case FCC_YUY2:
		return "yuyv";
	case FCC_NV12:
		return "nv12";
	case FCC_MJPEG:
		return "mjpeg";
	case FCC_H264:
		return "h264";
	case FCC_H265:
		return "h265";
	default:
		return "unknown";
	}
}
