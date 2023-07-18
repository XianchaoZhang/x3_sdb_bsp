/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * uvc_vps.c
 *	uvc host demo application that feed data from uvc -> vps -> bpu
 *
 * Copyright (C) 2019 Horizon Robotics, Inc.
 *
 * Contact: jianghe xu<jianghe.xu@horizon.ai>
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <assert.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>

#include "camera.h"
#include "uvc_helper.h"

/******************************************
 *          Definition                    *
 *****************************************/
#define DUMP_ENABLE 1
// #define DURATION_TIME_DEBUG

/******************************************
 *          Function Declaration          *
 *****************************************/
static void got_frame_handler(struct video_frame *frame, void *user_args);

/******************************************
 *          Function Implementation       *
 *****************************************/
static void dump_prepare(int *dump_fd, const char* dump_name)
{
    int fd;

    fd = open(dump_name, O_CREAT | O_RDWR, 0664);
    if (fd < 0) {
        printf("open %s failed\n", dump_name);
    }

    *dump_fd = fd;

    return;
}

static void dump_finish(int fd)
{
    close(fd);
}

/******************************************
 *        Time of Duration Function       *
 *****************************************/
struct signal_handle_arg
{
    timer_t* timerid;
    camera_t* cam;
    int dump_fd;
    struct timeval* tv1;
};

#ifdef DURATION_TIME_DEBUG
static inline void signal_handle_arg_init(
    struct signal_handle_arg* arg, timer_t* t, camera_t* cam,
    int dump_fd, struct timeval* tv1)
#else
static inline void signal_handle_arg_init(
    struct signal_handle_arg* arg, timer_t* t, camera_t* cam,
    int dump_fd)
#endif
{
    arg->timerid = t;
    arg->cam = cam;
    arg->dump_fd = dump_fd;
#ifdef DURATION_TIME_DEBUG
    arg->tv1 = tv1;
#endif
}
#ifdef DURATION_TIME_DEBUG
static void time_diff(const struct timeval* tv1,
        const struct timeval* tv2, struct timeval* diff)
{
    diff->tv_sec = diff->tv_usec = 0;
    if ((tv1->tv_sec == tv2->tv_sec) && (tv1->tv_usec <= tv2->tv_usec)) {
        diff->tv_usec = tv2->tv_usec - tv1->tv_usec;
    } else if (tv1->tv_sec == (tv2->tv_sec - 1)) {
        diff->tv_usec = 1000000 - tv1->tv_usec + tv2->tv_usec;
        if (diff->tv_usec >= 1000000) {
            diff->tv_sec = 1;
            diff->tv_usec -= 1000000;
        }
    } else if (tv1->tv_sec < (tv2->tv_sec - 1)) {
        diff->tv_sec = tv2->tv_sec - tv1->tv_sec - 1;
        diff->tv_usec = 1000000 - tv1->tv_usec + tv2->tv_usec;
        if (diff->tv_usec >= 1000000) {
            diff->tv_sec += 1;
            diff->tv_usec -= 1000000;
        }
    }
}
#endif

static void signal_handler(int sig, siginfo_t* si, void* uc)
{
    int r = 0;
    struct signal_handle_arg* sig_arg =
            (struct signal_handle_arg*)si->si_value.sival_ptr;
    if (timer_delete(*sig_arg->timerid) < 0)
        perror("timer_delete failed");

#ifdef DURATION_TIME_DEBUG
    struct timeval tv2 = {0}, diff = {0};
    r = gettimeofday(&tv2, NULL);
    if (r < 0)
        perror("gettimeofday tv2 failed");

    time_diff(sig_arg->tv1, &tv2, &diff);
    printf("time diff: sec %ld, usec %ld\n", diff.tv_sec, diff.tv_usec);
#endif

    r = camera_stop_streaming(sig_arg->cam);
    if (r < 0) {
        printf("camera stop streaming failed\n");
        camera_close(sig_arg->cam);
        exit(1);
    }

    if (DUMP_ENABLE)
        dump_finish(sig_arg->dump_fd);

    camera_close(sig_arg->cam);
    exit(0);
}

static int timer_init(clockid_t clockid,
        struct signal_handle_arg* arg, time_t value_sec)
{
    int ret = 0;
    struct sigevent evp = {0};
    struct itimerspec ts = {0};
    struct sigaction sa = {0};

    sa.sa_flags = SA_SIGINFO;
    sa.sa_sigaction = signal_handler;
    sigemptyset(&sa.sa_mask);
    ret = sigaction(SIGRTMAX, &sa, NULL);
    if (ret < 0) {
        perror("sigaction failed");
        return ret;
    }

    evp.sigev_value.sival_ptr = arg;
    evp.sigev_notify = SIGEV_SIGNAL;
    evp.sigev_signo = SIGRTMAX;
    ret = timer_create(clockid, &evp, arg->timerid);
    if (ret < 0) {
        perror("timer_create failed");
        return ret;
    }

    ts.it_interval.tv_sec = 0;
    ts.it_interval.tv_nsec = 0;
    ts.it_value.tv_sec = value_sec;
    ts.it_value.tv_nsec = 0;

    ret = timer_settime(*arg->timerid, 0, &ts, NULL);
    if (ret < 0) {
        perror("timer_settime failed");
        goto out;
    }
    return ret;
out:
    ret = timer_delete(*arg->timerid);
    if (ret < 0)
        perror("timer_delete failed");
    return ret;
}


static void wait_user_choose_format(int *format_index,
        int *frame_index, int *fps)
{
    printf("choose format index:\n");
    scanf("%d", format_index);

    printf("choose frame index:\n");
    scanf("%d", frame_index);

    printf("choose fps:\n");
    scanf("%d", fps);
}

static void usage(const char *argv0)
{
    fprintf(stderr, "Usage: %s [options]\n", argv0);
    fprintf(stderr, "Available options are\n");
    fprintf(stderr,
        " -d		v4l2 device name(default /dev/video8)\n");
    fprintf(stderr,
        " -t		time(seconds) of duration(default enter q to exit)\n");
    fprintf(stderr,
        " -o		dump file name(default usb_webcam.dump)\n");
}

int main(int argc, char *argv[])
{
    camera_t *cam = NULL;
    format_enums fmt_enums;
    char *v4l2_devname = "/dev/video8";
    char *dump_name = "usb_webcam.dump";
    int format_index, frame_index, fps = 0, set_fps;
    int dump_fd = 0;
    int r, opt;

    timer_t timerid;
    time_t time_out = 0;
    struct signal_handle_arg arg;

#ifdef DURATION_TIME_DEBUG
    struct timeval tv1 = {0};
#endif

    while ((opt = getopt(argc, argv, "hd:t:o:")) != -1) {
        switch (opt) {
        case 'd':
            v4l2_devname = optarg;
            break;

        case 't':
            time_out = strtol(optarg, NULL, 10);
            if (time_out <= 0) {
                printf("Timeout error, timeout has to be greater than 0\n");
                return 1;
            }
            break;
        case 'o':
            dump_name = optarg;
            break;

        case 'h':
            usage(argv[0]);
            return 1;

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

    wait_user_choose_format(&format_index, &frame_index, &fps);
    printf("user input format_index(%d), frame_index(%d), fps(%d)\n",
            format_index, frame_index, fps);

    r = camera_set_format(cam, format_index, frame_index, fps);
    if (r < 0) {
        printf("camera set format failed\n");
        goto fail1;
    }

    if (fps)
        set_fps = fps;
    r = camera_set_framerate(cam, fps);
    if (r < 0) {
        printf("camera set frame rate failed\n");
        goto fail1;
    }

    if (DUMP_ENABLE)
        dump_prepare(&dump_fd, dump_name);

    r = camera_start_streaming(cam, got_frame_handler, &dump_fd);
    if (r < 0) {
        printf("camera start streaming failed. r(%d)\n", r);
        goto fail1;
    }

    /* main loop */
    printf("'q' for exit\n");

    if (time_out > 0) {
#ifdef DURATION_TIME_DEBUG
        signal_handle_arg_init(&arg, &timerid, cam, dump_fd, &tv1);
        r = gettimeofday(&tv1, NULL);
        if (r < 0)
            perror("gettimeofday tv1 failed");
#else
        signal_handle_arg_init(&arg, &timerid, cam, dump_fd);
#endif

        timer_init(CLOCK_MONOTONIC, &arg, time_out);
    }

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
    int r;
    struct timeval tval = {0};
    struct tm tm = {0};
    static unsigned int frame_cnt = 0;

    if (!frame || !frame->mem || !user_args || frame->length < 0)
        return;
    ++frame_cnt;

    r = gettimeofday(&tval, NULL);
    if (0 == r) {
        localtime_r(&tval.tv_sec, &tm);
        printf("[%d/%02d/%02d %02d:%02d:%02d.%06ld] got %010u "
                "frame(%s:%uX%u), length(%u)\n",
            tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour,
            tm.tm_min, tm.tm_sec, tval.tv_usec, frame_cnt,
            fcc_format_to_string(frame->fcc), frame->width,
            frame->height, frame->length);
    } else {
        printf("got frame(%s:%uX%u)!! mem(%p), length(%u)\n",
            fcc_format_to_string(frame->fcc), frame->width,
            frame->height, frame->mem, frame->length);
    }

    dump_fd = *(int *)user_args;

    if (dump_fd <= 0) {
        printf("invalid dump_fd, can't dump file...\n");
        return;
    }

    if (frame->fcc == FCC_YUY2) {
        int dest_size = frame->width * frame->height * 3 / 2;
        unsigned char *dest = calloc(1, dest_size);
        assert(dest != NULL);
        r = convert_yuy2_to_nv12(frame->mem, dest, frame->width,
                frame->height);
        if (r) {
            printf("convert data from yuy2 to nv12 failed...\n");
	    free(dest);
            return;
        }

        /* dump video frame to file */
        actual = write(dump_fd, dest, dest_size);
        if (actual != frame->length)
            printf("[nv12]write expect %d bytes, actual written %d bytes\n",
                    dest_size, actual);
        free(dest);
    } else if (frame->fcc == FCC_MJPEG) {
        /* dump video frame to file */
        actual = write(dump_fd, frame->mem, frame->length);
        if (actual != frame->length)
            printf("[mjpeg]write expect %d bytes, actual written %d bytes\n",
                    frame->length, actual);
    }

    return;
}
