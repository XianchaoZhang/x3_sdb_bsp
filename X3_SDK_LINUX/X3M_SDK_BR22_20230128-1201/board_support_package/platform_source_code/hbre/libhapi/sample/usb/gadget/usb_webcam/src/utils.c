/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * utils.c
 *	utils for debug & trace
 *
 * Copyright (C) 2019 Horizon Robotics, Inc.
 *
 * Contact: jianghe xu<jianghe.xu@horizon.ai>
 */

#include "utils.h"
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <time.h>

int g_log_level = UVC_LOG_TRACE;
static char dump_url[256] = {0, };

int open_dump_file(const char *fmt, unsigned int width, unsigned int height)
{
	int fd = -1;

	if (!fmt)
		return -1;

	time_t rawtime = time(NULL);

	struct tm *ptm = localtime(&rawtime);

	snprintf(dump_url, sizeof(dump_url),
			"/userdata/uvc-dump-%s-%ux%u-[%04d-%02d-%02d %02d:%02d:%02d].data",
			fmt, width, height,
			ptm->tm_year + 1970, ptm->tm_mon, ptm->tm_mday,
			ptm->tm_hour, ptm->tm_min, ptm->tm_sec);
	dump_url[255] = '\0';

	fd = open(dump_url, O_CREAT|O_RDWR, 0666);
	if (fd < 0)
		fprintf(stderr, "open %s failed, errno(%d - %m)\n",
				dump_url, errno);

	printf("open %s for uvc data dump\n", dump_url);

	return fd;
}

void write_dump_file(int fd, const void *buf, size_t count)
{
	if (fd > 0 && buf && count &&
			access("/tmp/uvc-dump-enable", F_OK) == 0) {
		if (write(fd, buf, count) < 0)
			fprintf(stderr, "uvc dump data failed. "
					"buf(%p), count(%ld)\n",
					buf, count);
	}
}

void close_dump_file(int *fd)
{
	if (*fd > 0) {
		printf("close %s\n", dump_url);

		/* if uvc_dump not enabled, just delete the file */
		if (access("/tmp/uvc-dump-enable", F_OK) < 0) {
			printf("delete file %s\n", dump_url);

			unlink(dump_url);
			memset(dump_url, 0, 256);
		}

		close(*fd);
		*fd = -1;
	}
}
