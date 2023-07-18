/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * usb_webcam.c
 *	usb webcam demo app's main entry
 *
 * Copyright (C) 2019 Horizon Robotics, Inc.
 *
 * Contact: jianghe xu<jianghe.xu@horizon.ai>
 */

#include <unistd.h>
#include <errno.h>
#include <poll.h>
#include <signal.h>
#include <sys/eventfd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include "hb_video.h"
#include "uvc_gadget_api.h"
#include "uvc_event_handler.h"
#include "camera_lib.h"
#include "uevent_helper.h"
#include "utils.h"
#include "yuv2yuv.h"

#define UEVENT_MSG_LEN 4096
#define EVENT_KEYBOARD_QUIT 1
#define EVENT_KOBJECT_UVC_ADD 2
#define EVENT_KOBJECT_UVC_REMOVE 3

#define GUVC_RESTART "/tmp/guvc_restart"

static camera_comp_t *cam;
static int event_fd;

static int uvc_get_frame(struct uvc_context *uvc_ctx,
				void **buf_to, int *buf_len, void **entity,
				void *userdata)
{
	video_context *video_ctx = (video_context *)userdata;

	if (!uvc_ctx || !uvc_ctx->udev || !video_ctx)
		return -EINVAL;

#ifdef UVC_DEBUG
	interval_cost_trace_in();
#endif

	struct uvc_device *dev = uvc_ctx->udev;

	/* raw video>> nv12 & yuy2 */
	if (!video_ctx->has_venc) {
		/* just support nv12 & yuy2 now! */
		if (dev->fcc != V4L2_PIX_FMT_YUYV &&
				dev->fcc != V4L2_PIX_FMT_NV12)
			goto error;

		/* check mono_raw_q */
		if (!video_ctx->mono_raw_q)
			goto error;

		raw_single_buffer *mono_raw_q = video_ctx->mono_raw_q;

		pthread_mutex_lock(&mono_raw_q->mutex);
		if (!mono_raw_q->used) {
			pthread_mutex_unlock(&mono_raw_q->mutex);
			goto error;
		}

		/* check if resolution matching */
		if (video_ctx->req_width != mono_raw_q->pym_buffer.pym[0].width
				|| video_ctx->req_height != mono_raw_q->pym_buffer.pym[0].height) {
			fprintf(stderr, "resolution mismatched!! req_width(%u), "
					"req_height(%u), actual_width(%u), "
					"actual_height(%u)\n",
					video_ctx->width, video_ctx->height,
					mono_raw_q->pym_buffer.pym[0].width,
					mono_raw_q->pym_buffer.pym[0].height);
			pthread_mutex_unlock(&mono_raw_q->mutex);
			goto error;
		}

		if (!video_ctx->tmp_buffer) {
			fprintf(stderr, "calloc for tmp_buffer failed!! Out Of Memory??\n");
			pthread_mutex_unlock(&mono_raw_q->mutex);
			goto error;
		}

		if (video_ctx->format == VIDEO_FMT_NV12) {
			/*
			 * prepare continues nv12(2 plane, y +  interleaved cbcr) data
			 * as pyramid & ipu's y data & cbcr data is not continues in x3 platform.
			 * */
			unsigned int plane_size = video_ctx->width * video_ctx->height;
			memcpy(video_ctx->tmp_buffer,
					mono_raw_q->pym_buffer.pym[0].addr[0],
					plane_size);

			memcpy(video_ctx->tmp_buffer + plane_size,
					mono_raw_q->pym_buffer.pym[0].addr[1],
					plane_size / 2);
		} else if (video_ctx->format == VIDEO_FMT_YUY2) {
			hb_yuv2yuv_nv12toyuy2_neon((uint8_t *)mono_raw_q->pym_buffer.pym[0].addr[0],
					(uint8_t *)mono_raw_q->pym_buffer.pym[0].addr[1],
					NULL,
					video_ctx->tmp_buffer,
					video_ctx->width,
					video_ctx->height,
					1);
		}

		/* apply frame to *buf_to */
		*buf_to = video_ctx->tmp_buffer;
		*buf_len = video_ctx->buffer_size;
		*entity = mono_raw_q;
#if 0
		/* apply frame to *buf_to */
		*buf_to = mono_raw_q->pym_buffer.pym[0].addr[0];
		*buf_len = mono_raw_q->pym_buffer.pym[0].width *
				mono_raw_q->pym_buffer.pym[0].height * 3 / 2;
		*entity = mono_raw_q;
#endif

		pthread_mutex_unlock(&mono_raw_q->mutex);
	} else { /* compressed video>> mjpeg, h264 & h265 */
		venc_context *venc_ctx = video_ctx->venc_ctx;

		/* check venc_context */
		if (!venc_ctx)
			goto error;

		/* check mono_venc_q */
		if (!venc_ctx->mono_venc_q)
			goto error;

		venc_single_buffer *mono_venc_q = venc_ctx->mono_venc_q;

		pthread_mutex_lock(&mono_venc_q->mutex);
		if (!mono_venc_q->used) {
			pthread_mutex_unlock(&mono_venc_q->mutex);
			goto error;
		}

		/* get mono_venc_q vstream and apply frame to *buf_to */
		*buf_to = mono_venc_q->vstream.pstPack.vir_ptr;
		*buf_len = mono_venc_q->vstream.pstPack.size;
		*entity = mono_venc_q;

		pthread_mutex_unlock(&mono_venc_q->mutex);
	}

#ifdef UVC_DEBUG
	interval_cost_trace_out();
#endif

#ifdef UVC_DUMP
	write_dump_file(video_ctx->dump_fd, *buf_to, *buf_len);
#endif

	return 0;

error:
	return -EFAULT;
}

static void uvc_release_frame(struct uvc_context *uvc_ctx,
				     void **entity, void *userdata)
{
	video_context *video_ctx = (video_context *)userdata;
	if (!uvc_ctx || !entity || !(*entity) || !video_ctx)
		return;

	/* raw video>> nv12 & yuy2 */
	if (!video_ctx->has_venc) {
		raw_single_buffer *mono_raw_q = (raw_single_buffer *)*entity;

		pthread_mutex_lock(&mono_raw_q->mutex);

		HB_VPS_ReleaseChnFrame(0, 6, &mono_raw_q->pym_buffer);
		memset(&mono_raw_q->pym_buffer, 0, sizeof(pym_buffer_t));
		mono_raw_q->used = 0;
		*entity = 0;

		pthread_mutex_unlock(&mono_raw_q->mutex);
	} else { /* compressed video>> mjpeg, h264 & h265 */
		venc_context *venc_ctx = video_ctx->venc_ctx;
		if (!venc_ctx)
			return;

		venc_single_buffer *mono_venc_q = (venc_single_buffer *)*entity;

		pthread_mutex_lock(&mono_venc_q->mutex);

		HB_VENC_ReleaseStream(venc_ctx->venc_chn, &mono_venc_q->vstream);
		memset(&mono_venc_q->vstream, 0, sizeof(VIDEO_STREAM_S));
		mono_venc_q->used = 0;
		*entity = 0;

		pthread_mutex_unlock(&mono_venc_q->mutex);
	}

	return;
}

static void uvc_streamon_off(struct uvc_context *uvc_ctx, int is_on, void *userdata)
{
	struct uvc_device *dev;
	video_context *video_ctx = (video_context *)userdata;
	unsigned int width, height, fcc;

	if (!uvc_ctx || !uvc_ctx->udev)
		return;

	dev = uvc_ctx->udev;
	fcc = dev->fcc;
	width = dev->width;
	height = dev->height;

	if (is_on) {
		video_ctx->format = fcc_to_video_format(fcc);
		video_ctx->width = width;
		video_ctx->height = height;
		video_ctx->req_width = width;
		video_ctx->req_height = height;

		printf("##STREAMON is_on(%d)## %s(%ux%u)\n", is_on,
		       fcc_to_string(fcc), width, height);

		if (hb_video_init(video_ctx)) {
			fprintf(stderr, "hb_vedio_init failed\n");
			free(video_ctx);
			goto error0;
		}

		if (hb_video_prepare(video_ctx)) {
			fprintf(stderr, "hb_video_prepare failed\n");
			goto error1;
		}

		if (hb_video_start(video_ctx)) {
			fprintf(stderr, "hb_video_start failed\n");
			goto error2;
		}

		if (video_ctx->tmp_buffer) {
			fprintf(stderr, "some error happen?? tmp_buffer(%p) not freed\n",
					video_ctx->tmp_buffer);
			free(video_ctx->tmp_buffer);
			video_ctx->tmp_buffer = NULL;
		}

		/* prepare tmp buffer for video convert (eg. nv12 to yuy2) */
		if (video_ctx->format == VIDEO_FMT_YUY2) {
			video_ctx->buffer_size = width * height * 2;
			video_ctx->tmp_buffer = calloc(1, video_ctx->buffer_size);

			printf("%s function. calloc tmp buffer for yuy2\n", __func__);

			if (!video_ctx->tmp_buffer) {
				fprintf(stderr, "calloc for tmp_buffer failed\n");
				goto error2;
			}
		} else if (video_ctx->format == VIDEO_FMT_NV12) {
			video_ctx->buffer_size = width * height * 3 / 2;
			video_ctx->tmp_buffer = calloc(1, video_ctx->buffer_size);

			printf("%s function. calloc tmp buffer for nv12\n", __func__);

			if (!video_ctx->tmp_buffer) {
				fprintf(stderr, "calloc for tmp_buffer failed\n");
				goto error2;
			}
		}

#ifdef UVC_DUMP
		video_ctx->dump_fd = open_dump_file(fcc_to_string(fcc), width, height);
#endif
	} else {
		printf("##STREAMOFF is_on(%d)## %s(%ux%u)\n", is_on,
		       fcc_to_string(fcc), width, height);

		if (hb_video_stop(video_ctx)) {
			fprintf(stderr, "hb_video_stop failed\n");
			goto error2;
		}

		hb_video_finalize(video_ctx);

		hb_video_deinit(video_ctx);

		/* release tmp buffer not in use */
		if (video_ctx->tmp_buffer) {
			free(video_ctx->tmp_buffer);
			video_ctx->tmp_buffer = NULL;
		}
#ifdef UVC_DUMP
		close_dump_file(&video_ctx->dump_fd);
#endif
	}

	return;

error2:
	hb_video_finalize(video_ctx);
error1:
	hb_video_deinit(video_ctx);
error0:
	return;
}

static int uvc_event_setup_handle(struct uvc_context *ctx,
				uint8_t req, uint8_t cs, uint8_t entity_id,
				struct uvc_request_data *resp,
				void *userdata)
{
	int ret = -1;

	switch (entity_id) {
		case UVC_CTRL_CAMERA_TERMINAL_ID:
			ret = uvc_camera_terminal_setup_event(ctx, req, cs, resp, userdata);
			break;

		case UVC_CTRL_PROCESSING_UNIT_ID:
			ret = uvc_process_unit_setup_event(ctx, req, cs, resp, userdata);
			break;

		case UVC_CTRL_EXTENSION_UNIT_ID:
			ret = uvc_extension_unit_setup_event(ctx, req, cs, resp, userdata);
			break;

		default:
			printf("EVENT NOT CARE, entity_id(0x%x), req(0x%x), cs(0x%x)\n",
					entity_id, req, cs);
			break;
	}

	return ret;
}

static int uvc_event_data_handle(struct uvc_context *ctx,
				uint8_t req, uint8_t cs, uint8_t entity_id,
				struct uvc_request_data *data,
				void *userdata)
{
	int ret = -1;

	switch (entity_id) {
		case UVC_CTRL_CAMERA_TERMINAL_ID:
			ret = uvc_camera_terminal_data_event(ctx, req, cs, data, userdata);
			break;

		case UVC_CTRL_PROCESSING_UNIT_ID:
			ret = uvc_process_unit_data_event(ctx, req, cs, data, userdata);
			break;

		case UVC_CTRL_EXTENSION_UNIT_ID:
			ret = uvc_extension_unit_data_event(ctx, req, cs, data, userdata);
			break;

		default:
			printf("EVENT NOT CARE, entity_id(0x%x), req(0x%x), cs(0x%x)\n",
					entity_id, req, cs);
			break;
	}

	return ret;
}

static int uvc_start(struct uvc_context **ctx, mipi_sensor_type sensor_type,
		struct uvc_params *params)
{
	video_context *video_ctx = NULL;
	struct uvc_context *uvc_ctx = NULL;

	video_ctx = hb_video_alloc_context(sensor_type);
	if (!video_ctx) {
		fprintf(stderr, "hb_video_alloc_context failed\n");
		return -1;
	}

	/* init uvc device libguvc find a uvc device, no front-end v4l2 dev */
	if (uvc_gadget_init(&uvc_ctx, NULL, NULL, params)) {
		fprintf(stderr, "uvc_gadget_init failed\n");

		goto error1;
	}

	/* open front-end camera device for isp, ipu, venc setting... */
	if (camera_open(&cam) < 0) {
		fprintf(stderr, "camera_open failed\n");

		goto error2;
	}

	/* streamon_off/prepare/release buffer with video queue */
	uvc_set_streamon_handler(uvc_ctx, uvc_streamon_off, video_ctx);
	uvc_set_prepare_data_handler(uvc_ctx, uvc_get_frame, video_ctx);
	uvc_set_release_data_handler(uvc_ctx, uvc_release_frame, video_ctx);

	/* init detail control request function list... */
	uvc_control_request_param_init();
	uvc_control_request_callback_init();
	uvc_set_event_handler(uvc_ctx, uvc_event_setup_handle, uvc_event_data_handle,
			UVC_CTRL_CAMERA_TERMINAL_EVENT |
			UVC_CTRL_PROCESSING_UNIT_EVENT |
			UVC_CTRL_EXTENSION_UNIT_EVENT,
			cam);

	if (uvc_gadget_start(uvc_ctx)) {
		fprintf(stderr, "uvc_gadget_start failed\n");
		goto error3;
	}

	*ctx = uvc_ctx;

	return 0;

error3:
	camera_close(cam);
error2:
	uvc_gadget_deinit(uvc_ctx);
error1:
	hb_video_free_context(video_ctx);

	return -1;
}

static void uvc_stop(struct uvc_context **ctx)
{
	if (!ctx || !*ctx)
		return;

	struct uvc_context *uvc_ctx = *ctx;

	if (uvc_gadget_stop(uvc_ctx))
		fprintf(stderr, "uvc_gdaget_stop failed\n");

	if (cam) {
		camera_close(cam);
		cam = NULL;
	}

	uvc_gadget_deinit(uvc_ctx);

	*ctx = NULL;

	return;
}

#if 0
static void *kbhit_monitor(void *arg)
{
	uint64_t event;

	if (!arg)
		return (void *)0;

	int event_fd = *(int *)arg;

	printf("'q' for exit\n");
	while (getchar() != 'q');

	event = EVENT_KEYBOARD_QUIT;
	write(event_fd, &event, sizeof(uint64_t));

	return (void *)0;
}
#endif

static void *dwc3_uevent_monitor(void *arg)
{
	struct uevent uevent;
	struct timeval timeout = {1, 0};  // 1s timeout
	char msg[UEVENT_MSG_LEN+2];
	int *event_fd = (int *)arg;
	uint64_t event;
	ssize_t n;
	int sock;

	if (!event_fd)
		return (void *)0;

	sock = open_uevent_socket();

	if (sock < 0)
		return (void *)0;

	if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO,
			(const char *)&timeout, sizeof(timeout)) < 0) {
		fprintf(stderr, "setsockopt for socket receive timeout failed\n");

		close_uevent_socket(sock);
		return (void *)0;
	}

	do {
		n = recv(sock, msg, UEVENT_MSG_LEN, 0);

		if (n < 0 || n > UEVENT_MSG_LEN)
			continue;

		msg[n] = '\0';
		msg[n+1] = '\0';

		parse_uevent(msg, &uevent);

		if (strncmp(uevent.subsystem, "video4linux", 11) == 0 &&
			strncmp(uevent.devname, "video", 5) == 0 &&
			strstr(uevent.path, "gadget")) {
			if (strncmp(uevent.action, "add", 3) == 0) {
				event = EVENT_KOBJECT_UVC_ADD;
				if (*event_fd)
					write(*event_fd, &event, sizeof(uint64_t));
			} else if (strncmp(uevent.action,
						"remove", 5) == 0) {
				event = EVENT_KOBJECT_UVC_REMOVE;
				if (*event_fd)
					write(*event_fd, &event, sizeof(uint64_t));
			}
		}
	} while (*event_fd);	/* re-use event_fd as thread quit flag */

	close_uevent_socket(sock);

	return (void *)0;
}

static void signal_handler(int signal)
{
	uint64_t event;

	if (signal == SIGINT || signal == SIGTERM
			|| signal == SIGABRT || signal == SIGKILL ) {
		event = EVENT_KEYBOARD_QUIT;

		printf("%s receive signal(%d)\n", __func__, signal);

		if (event_fd) {
			write(event_fd, &event, sizeof(uint64_t));
			printf("write singal out event to event fd\n");
		}
	}
}

static void main_loop(struct uvc_context **uvc_ctx,
		mipi_sensor_type sensor_type,
		struct uvc_params *params)
{
	struct pollfd pfd;
	// pthread_t kb_tid;
	pthread_t uevent_tid;
	uint64_t event;
	ssize_t sz;
	int uvc_quit = 0;

	if (!uvc_ctx || !params)
		return;

	event_fd = eventfd(0, 0);

#if 0
	if (pthread_create(&kb_tid, NULL, kbhit_monitor, &event_fd) < 0) {
		fprintf(stderr, "kbhit_monitor thread create failed\n");
		goto error1;
	}
#endif

	signal(SIGKILL, signal_handler);
	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);
	signal(SIGABRT, signal_handler);

	if (pthread_create(&uevent_tid, NULL, dwc3_uevent_monitor, &event_fd) < 0) {
		fprintf(stderr, "uevent_monitor thread create failed\n");
		goto error2;
	}

	pfd.fd = event_fd;
	pfd.events = POLLIN;
	while (!uvc_quit) {
		if (poll(&pfd, 1, 0) == 1) {
			sz = read(event_fd, &event, sizeof(uint64_t));
			if (sz != sizeof(uint64_t)) {
				fprintf(stderr, "eventfd read abnormal. sz(%lu)\n",
						sz);
				continue;
			}

			printf("receive event %lu\n", event);

			switch (event) {
			case EVENT_KEYBOARD_QUIT:
				printf("recv keyboard quit event\n");
				uvc_quit = 1;
				break;
			case EVENT_KOBJECT_UVC_ADD:
				printf("recv uvc kobject add event\n");
				if (uvc_start(uvc_ctx, sensor_type, params) < 0)
					printf("%s uvc_start failed\n", __func__);
				break;
			case EVENT_KOBJECT_UVC_REMOVE:
				printf("recv uvc kobject remove event\n");
				uvc_stop(uvc_ctx);
				break;
			default:
				break;
			}
		}

		if (uvc_quit)
			break;

		usleep(300 * 1000);
	}

	/* dwc3_uevent_monitor thread re-use event_fd as thread quit flag */
	event_fd = 0;
	pthread_join(uevent_tid, NULL);

error2:
// error1:
	return;
}

static void usage(const char *argv0)
{
	fprintf(stderr, "Usage: %s [options]\n", argv0);
	fprintf(stderr, "Available options are\n");
	fprintf(stderr, " -b		Use bulk mode\n");
	fprintf(stderr, " -a		use streaming multi alternate setting\n");
	fprintf(stderr,
		" -e <mipi_sensor_type> Just refer to mipi_sensor_type enum\n\t"
		"0 = 4M test pattern\n\t"
		"2 = 12M test pattern\n\t"
		"3 = 8M(4k) test pattern\n\t"
		"5 = os8a10\n\t"
		"11 = f37\n\t"
		"12 = imx307\n\t"
		"13 = ar0233\n");
	fprintf(stderr,
		" -m		Streaming mult for ISOC (b/w 0 and 2)\n");
	fprintf(stderr, " -t		Streaming burst (b/w 0 and 15)\n");
	fprintf(stderr, " -p		Max packet size bytes for ISOC (0-3072)\n");
	fprintf(stderr,
		" -s <speed>	Select USB bus speed (b/w 0 and 2)\n\t"
		"2 = Full Speed (FS)\n\t"
		"3 = High Speed (HS)\n\t"
		"5 = Super Speed (SS)\n\t"
		"? = Super Speed (SS)\n");
	fprintf(stderr,
		" -n		Number of Video buffers (b/w 2 and 32)\n");
}

static int uvc_init_params_with_argv(int argc, char *argv[],
		struct uvc_params *params, mipi_sensor_type *sensor_type)
{
	int opt;

	/* init default user params */
	uvc_gadget_user_params_init(params);
	params->bulk_mode = 0;  /* use isoc mode */
	params->h264_quirk = 0;  /* xj3 need to combine sps, pps, IDR together */
	params->mult_alts = 0;  /* no streaming multi alternate setting, only alt0 and alt1 */

	while ((opt = getopt(argc, argv, "hbas:n:e:m:t:p:")) != -1) {
		switch (opt) {
		case 'b':
			params->bulk_mode = 1;
			break;

		case 'a':
			params->mult_alts = 1;  /* uvc use streaming multi alternate settings */
			break;

		case 'e':
			if (atoi(optarg) < 0 || atoi(optarg) >= SENOSR_MAX) {
				usage(argv[0]);
				return -1;
			}

			*sensor_type = atoi(optarg);
			printf("Use sensor type %d\n", *sensor_type);
			break;
		case 'n':
			if (atoi(optarg) < 2 || atoi(optarg) > 32) {
				usage(argv[0]);
				return -1;
			}

			params->nbufs = atoi(optarg);
			printf("Number of buffers requested = %d\n",
			       params->nbufs);
			break;

		case 's':
			if (atoi(optarg) < 0 || atoi(optarg) > 5) {
				usage(argv[0]);
				return -1;
			}

			params->speed = atoi(optarg);
			break;

		case 'm':
			if (atoi(optarg) < 0 || atoi(optarg) > 2) {
				usage(argv[0]);
				return -1;
			}

			params->mult = atoi(optarg);
			printf("Requested Mult value = %d\n", params->mult);
			break;
		case 'p':
			if (atoi(optarg) < 0 || atoi(optarg) > 3072) {
				usage(argv[0]);
				return -1;
			}

			params->maxpkt_quirk = atoi(optarg);
			printf("Max Packet Size value = %d\n", params->maxpkt_quirk);
			break;
		case 't':
			if (atoi(optarg) < 0 || atoi(optarg) > 15) {
				usage(argv[0]);
				return -1;
			}

			params->burst = atoi(optarg);
			printf("Requested Burst value = %d\n", params->burst);
			break;

		case 'h':
			usage(argv[0]);
			return -1;

		default:
			printf("Invalid option '-%c'\n", opt);
			usage(argv[0]);
			return -1;
		}
	}

	return 0;
}

int main(int argc, char *argv[])
{
	struct uvc_context *uvc_ctx = NULL;
	struct uvc_params params;
	mipi_sensor_type sensor_type = SENSOR_INVALID;

	if (uvc_init_params_with_argv(argc, argv, &params, &sensor_type))
		goto error;

	if (uvc_start(&uvc_ctx, sensor_type, &params))
		goto error;

	main_loop(&uvc_ctx, sensor_type, &params);

	uvc_stop(&uvc_ctx);

	return 0;

error:
	return 1;
}
