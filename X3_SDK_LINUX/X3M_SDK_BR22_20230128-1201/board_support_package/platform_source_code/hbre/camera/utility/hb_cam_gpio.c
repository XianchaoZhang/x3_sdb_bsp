/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2016 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>

#include "hb_cam_gpio.h"
#include "inc/cam_common.h"
#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define MAX_BUF	64

/****************************************************************
 * gpio_export
 ****************************************************************/
int gpio_export(unsigned int gpio)
{
	int fd;
	int len;
	char buf[MAX_BUF];

	fd = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
	if (fd < 0) {
		perror("gpio/export");
		return fd;
	}

	len = snprintf(buf, sizeof(buf), "%d", gpio);
	if (len < 0) {
		perror("snprintf");
		goto err;
	}
	if (write(fd, buf, (size_t)len) < 0) {
		perror("write");
		goto err;
	}

	close(fd);
	return RET_OK;

err:
	close(fd);
	return -1;
}

/****************************************************************
 * gpio_unexport
 ****************************************************************/
int gpio_unexport(unsigned int gpio)
{
	int fd, len;
	char buf[MAX_BUF];

	fd = open(SYSFS_GPIO_DIR "/unexport", O_WRONLY | O_CLOEXEC);
	if (fd < 0) {
		perror("gpio/export");
		return fd;
	}

	len = snprintf(buf, sizeof(buf), "%d", gpio);
	if (len < 0) {
		perror("snprintf");
		goto err;
	}
	if (write(fd, buf, (size_t)len) < 0) {
		perror("write");
		goto err;
	}

	close(fd);
	return RET_OK;

err:
	close(fd);
	return -1;
}

/****************************************************************
 * gpio_set_dir
 ****************************************************************/
int gpio_set_dir(unsigned int gpio, unsigned int out_flag)
{
	int fd;
	char buf[MAX_BUF];

	snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR  "/gpio%d/direction", gpio);

	fd = open(buf, O_WRONLY | O_CLOEXEC);
	if (fd < 0) {
		perror("gpio/direction");
		return fd;
	}

	if (out_flag) {
		if (write(fd, "out", 4) < 0)
			goto err;
	} else {
		if (write(fd, "in", 3) < 0)
			goto err;
	}

	close(fd);
	return RET_OK;

err:
	perror("write");
	close(fd);
	return -1;
}

/****************************************************************
 * gpio_set_value
 ****************************************************************/
int gpio_set_value(unsigned int gpio, unsigned int value)
{
	int fd;
	char buf[MAX_BUF];

	snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);

	fd = open(buf, O_WRONLY | O_CLOEXEC);
	if (fd < 0) {
		perror("gpio/set-value");
		return fd;
	}

	if (value) {
		if (write(fd, "1", 2) < 0)
			goto err;
	} else {
		if (write(fd, "0", 2) < 0)
			goto err;
	}

	close(fd);
	return RET_OK;

err:
	perror("write");
	close(fd);
	return -1;
}

/****************************************************************
 * gpio_get_value
 ****************************************************************/
int gpio_get_value(unsigned int gpio, unsigned int *value)
{
	int fd;
	char buf[MAX_BUF];
	char ch;

	snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);

	fd = open(buf, O_RDONLY | O_CLOEXEC);
	if (fd < 0) {
		perror("gpio/get-value");
		return fd;
	}

	if (read(fd, &ch, 1) < 0) {
		perror("read");
		close(fd);
		return -1;
	}

	if (ch != '0') {
		*value = 1;
	} else {
		*value = 0;
	}

	close(fd);
	return RET_OK;
}

/****************************************************************
 * gpio_set_edge
 ****************************************************************/
int gpio_set_edge(unsigned int gpio, char *edge)
{
	int fd;
	char buf[MAX_BUF];

	snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/edge", gpio);

	fd = open(buf, O_WRONLY | O_CLOEXEC);
	if (fd < 0) {
		perror("gpio/set-edge");
		return fd;
	}

	if (write(fd, edge, strlen(edge) + 1) < 0) {
		perror("write");
		close(fd);
		return -1;
	}
	close(fd);
	return RET_OK;
}
