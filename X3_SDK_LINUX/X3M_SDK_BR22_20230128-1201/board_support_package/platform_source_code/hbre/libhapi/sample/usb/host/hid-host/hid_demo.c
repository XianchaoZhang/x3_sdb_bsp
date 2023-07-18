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
    int i;
    char hid_data[REPORT_LEN];
    char recv_data[REPORT_LEN];
    int fd = -1;
    int len;

    fd = open("/dev/hidraw1", O_RDWR, 0666);
    if (fd < 0) {
        printf("open dev fail\n");
        return -1;
    }

    for (i = 0; i < REPORT_LEN; i++) {
        hid_data[i] = i;
    }

    while (1) {
        printf("---loop---\n");
        /*write*/
        len = write(fd, hid_data, REPORT_LEN);
        printf("write len %d\n", len);
        /*read*/
        if (read(fd, recv_data, REPORT_LEN) != REPORT_LEN) {
            printf("%s %d read fail\n", __func__, __LINE__);
        } else {
            printf("read date ");
            for (i = 0; i < REPORT_LEN; i++) {
                printf("%x ", recv_data[i]);
            }
            printf("\n");
        }
        sleep(2);
    }
    close(fd);
    return 0;
}
