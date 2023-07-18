/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2016 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#include <logging.h>
#include "hb_spi.h"
#include "inc/cam_common.h"

int hb_spi_read_block(int fd, char *buf, int count)
{
    int ret = RET_OK;

	struct spi_ioc_transfer rx = {
			.rx_buf = (unsigned long)buf,
			.tx_buf = 0,
			.len = count,
	};

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &rx);
	if (ret < 1) {
		pr_err("can't read spi message\n");
		return -RET_ERROR;
	}
	return ret;
}

int hb_spi_write_block(int fd, char *buf, int count)
{
	int ret = RET_OK;

	struct spi_ioc_transfer tx = {
			.tx_buf = (unsigned long)buf,
			.rx_buf = 0,
			.len = count,
	};
	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tx);
	if (ret < 1) {
		pr_err("can't write spi message\n");
		return -RET_ERROR;
	}

	return ret;
}
