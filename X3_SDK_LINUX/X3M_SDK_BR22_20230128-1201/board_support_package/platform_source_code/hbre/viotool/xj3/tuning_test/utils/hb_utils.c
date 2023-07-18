/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#include <string.h>
#include <stdlib.h>
#include "hb_utils.h"

int dumpToFile(char *filename, char *srcBuf, unsigned int size)
{
	FILE *yuvFd = NULL;
	char *buffer = NULL;

	yuvFd = fopen(filename, "w+");

	if (yuvFd == NULL) {
		vio_err("ERRopen(%s) fail", filename);
		return -1;
	}

	buffer = (char *)malloc(size);

	if (buffer == NULL) {
		vio_err(":malloc file");
		fclose(yuvFd);
		return -1;
	}

	memcpy(buffer, srcBuf, size);

	fflush(stdout);

	fwrite(buffer, 1, size, yuvFd);

	fflush(yuvFd);

	if (yuvFd)
		fclose(yuvFd);
	if (buffer)
		free(buffer);

	vio_err("filedump(%s, size(%d) is successed!!", filename, size);

	return 0;
}

int dumpToFile2plane(char *filename, char *srcBuf, char *srcBuf1,
		     unsigned int size, unsigned int size1)
{

	FILE *yuvFd = NULL;
	char *buffer = NULL;

	yuvFd = fopen(filename, "w+");

	if (yuvFd == NULL) {
		vio_err("open(%s) fail", filename);
		return -1;
	}

	buffer = (char *)malloc(size + size1);

	if (buffer == NULL) {
		vio_err("ERR:malloc file");
		fclose(yuvFd);
		return -1;
	}

	memcpy(buffer, srcBuf, size);
	memcpy(buffer + size, srcBuf1, size1);

	fflush(stdout);

	fwrite(buffer, 1, size + size1, yuvFd);

	fflush(yuvFd);

	if (yuvFd)
		fclose(yuvFd);
	if (buffer)
		free(buffer);

	vio_err("DEBUG:filedump(%s, size(%d) is successed!!", filename, size);

	return 0;
}

void time_add_ms(struct timeval *time, uint16_t ms)
{
	time->tv_usec += ms * 1000;
	if (time->tv_usec >= 1000000) {
		time->tv_sec += time->tv_usec / 1000000;
		time->tv_usec %= 1000000;
	}
}

int get_cost_time_ms(struct timeval *start, struct timeval *end)
{
	int time_ms = -1;
	time_ms = ((end->tv_sec * 1000 + end->tv_usec /1000) -
		(start->tv_sec * 1000 + start->tv_usec /1000));
	return time_ms;
}
