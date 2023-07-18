/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#include <string.h>
#include <ctype.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define REPORT_LEN 64

int main()
{
	int i, r;
	char hid_data[REPORT_LEN];
	char recv_data[REPORT_LEN];
	int fd = -1;

	fd = open("/dev/hidg0", O_RDWR, 0666);
	if (fd < 0) {
		printf("open /dev/hidg0 fail\n");
		return -1;
	}

	for (i = 0; i < REPORT_LEN; i++) {
		hid_data[i] = i;
	}

	while (1) {
		r = read(fd, recv_data, REPORT_LEN);
		if (r <= 0) {
			printf("%s %d read hid fail\n", __func__, __LINE__);
		} else {
			printf("read date");
			for (i = 0; i < r; i++) {
				printf("%x ", recv_data[i]);
			}
			printf("\n");
		}
		if (write(fd, hid_data, REPORT_LEN) != REPORT_LEN) {
			printf("%s %d write fail\n", __func__, __LINE__);
			return -1;
		}
	}
	close(fd);
	return 0;
}
