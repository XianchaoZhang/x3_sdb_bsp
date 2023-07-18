/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * uvc_demo.c
 *	uvc demo application that feed data from uvc
 *
 * Copyright (C) 2019 Horizon Robotics, Inc.
 *
 * Contact: jianghe xu<jianghe.xu@horizon.ai>
 */

#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "camera.h"

/******************************************
 *          Definition                    *
 *****************************************/
#define DUMP_ENABLE 1

/******************************************
 *          Function Declaration          *
 *****************************************/
static void got_frame_handler(struct video_frame *frame, void *user_args);

/******************************************
 *          Function Implementation       *
 *****************************************/
static void dump_prepare(int *dump_fd)
{
	const char *dump_file = "usb_webcam.dump";
	int fd;

	fd = open(dump_file, O_CREAT | O_RDWR, 0664);
	if (fd < 0) {
		printf("open %s failed\n", dump_file);
	}

	*dump_fd = fd;

	return;
}

static void dump_finish(int fd)
{
	close(fd);
}

static void wait_user_choose_format(int *format_index, int *frame_index)
{
	printf("choose format index:\n");
	scanf("%d", format_index);

	printf("choose frame index:\n");
	scanf("%d", frame_index);
}

static void usage(const char *argv0)
{
	fprintf(stderr, "Usage: %s [options]\n", argv0);
	fprintf(stderr, "Available options are\n");
	fprintf(stderr,
		" -d		v4l2 device name\n");
}

int main(int argc, char *argv[])
{
	camera_t *cam = NULL;
	format_enums fmt_enums;
	char *v4l2_devname = "/dev/video8";
	int format_index, frame_index;
	int dump_fd = 0;
	int r, opt;

	while ((opt = getopt(argc, argv, "hs:n:m:")) != -1) {
		switch (opt) {
		case 'd':
			v4l2_devname = optarg;
			break;

		default:
			printf("Invalid option '-%c'\n", opt);
			usage(argv[0]);
			return 1;
		}
	}

	cam = camera_open(v4l2_devname);
	if (!cam) {
		printf("camera_open failed\n");
		return -1;
	}

	r = camera_enum_format(cam, &fmt_enums, 0);
	if (r < 0) {
		printf("camera enum format failed\n");
		goto fail1;
	}

	camera_show_format(cam);

	wait_user_choose_format(&format_index, &frame_index);
	printf("user input format_index(%d), frame_index(%d)\n",
			format_index, frame_index);

	r = camera_set_format(cam, format_index, frame_index, 0);
	if (r < 0) {
		printf("camera set format failed\n");
		goto fail1;
	}

	r = camera_set_framerate(cam, 30);
	if (r < 0) {
		printf("camera set frame rate failed\n");
		goto fail1;
	}

	if (DUMP_ENABLE)
		dump_prepare(&dump_fd);

	r = camera_start_streaming(cam, got_frame_handler, &dump_fd);
	if (r < 0) {
		printf("camera start streaming failed\n");
		goto fail1;
	}

	/* main loop */
	printf("'q' for exit\n");
	while (getchar() != 'q') {
		sleep(1);
	}

	r = camera_stop_streaming(cam);
	if (r < 0) {
		printf("camera stop streaming failed\n");
		goto fail1;
	}


	if (DUMP_ENABLE)
		dump_finish(dump_fd);

	camera_close(cam);

	return 0;

fail1:
	camera_close(cam);

	return r;
}

static void got_frame_handler(struct video_frame *frame, void *user_args)
{
	int actual = 0, dump_fd;

	if (!frame || !frame->mem || !user_args || frame->length < 0)
		return;

	printf("got frame(%s: %uX%u)!! mem(%p), length(%u)\n",
			fcc_format_to_string(frame->fcc), frame->width,
			frame->height, frame->mem, frame->length);

	dump_fd = *(int *)user_args;

	if (dump_fd <= 0) {
		printf("invalid dump_fd, can't dump file...\n");
		return;
	}

	/* dump video frame to file */
	actual = write(dump_fd, frame->mem, frame->length);
	if (actual != frame->length)
		printf("write expect %d bytes, actual written %d bytes\n",
				frame->length, actual);

	return;
}
