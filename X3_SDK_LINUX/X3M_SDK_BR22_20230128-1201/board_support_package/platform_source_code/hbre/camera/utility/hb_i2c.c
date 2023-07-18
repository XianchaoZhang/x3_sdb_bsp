/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2016 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>

#include <logging.h>
#include "hb_i2c.h"
#include "inc/cam_common.h"

// PRQA S ALL ++

static int cam_fd[CAM_FD_NUMBER];
static int cam_bus_cnt[CAM_FD_NUMBER];
static pthread_mutex_t mutex[CAM_FD_NUMBER];

int hb_i2c_init(int bus)
{
	int ret = RET_OK;
	char str[12] = {0};

	if(bus < 0 || bus >= CAM_FD_NUMBER) {
		pr_err("Invalid bus num %d", bus);
		return -RET_ERROR;
	}

	if(cam_fd[bus] <= 0) {
		snprintf(str, sizeof(str), "/dev/i2c-%d", bus);
		cam_fd[bus]= open(str, O_RDWR | O_CLOEXEC);
		if(cam_fd[bus] < 0) {
			pr_err("open i2c-%d fail\n", bus);
			return -RET_ERROR;
		}
		if (pthread_mutex_init(&mutex[bus],  NULL) != 0 ) {
	        pr_err("Init metux error.");
			close(cam_fd[bus]);
			cam_fd[bus] = -1;
			return -RET_ERROR;
	    }
	}
	cam_bus_cnt[bus]++;
	pr_info("bus %d cam_bus_cnt[bus] %d cam_fd[bus] %d\n",
		bus, cam_bus_cnt[bus], cam_fd[bus]);
	return ret;
}

int hb_i2c_deinit(int bus)
{
	int ret = RET_OK;

	if(bus < 0 || bus >= CAM_FD_NUMBER) {
		pr_err("Invalid bus num %d", bus);
		return -RET_ERROR;
	}
	if(cam_bus_cnt[bus] > 0)
		cam_bus_cnt[bus]--;
	if(cam_bus_cnt[bus] == 0 && cam_fd[bus] > 0) {
		close(cam_fd[bus]);
		cam_fd[bus] = -1;
		pthread_mutex_destroy(&mutex[bus]);
	}

	pr_info("bus %d cam_bus_cnt[bus] %d cam_fd[bus] %d\n",
		bus, cam_bus_cnt[bus], cam_fd[bus]);
	return ret;
}

int hb_i2c_read_reg16_data16(int bus, uint8_t i2c_addr, uint16_t reg_addr)
{
	int ret = RET_OK, value = 0;
	unsigned char wr_data[2];
	unsigned char rdata[2] = {0};
	int val = 0;
	ssize_t sys_ret;

	wr_data[0] = (uint8_t)((reg_addr >> 8) & 0x00ff);
	wr_data[1] = (uint8_t)(reg_addr & 0x00ff);

	val = pthread_mutex_lock(&mutex[bus]);
    	if (val != 0) {
	   pr_err("mutex lock error val %d \n", val);
	   return -RET_ERROR;
  	}
	ret = ioctl(cam_fd[bus], I2C_SLAVE_FORCE, i2c_addr);
	if (ret < 0) {
		pr_err("unable open camera with addr 0x%02x ioctl I2C_SLAVE_FORCE error\n", i2c_addr);
		pthread_mutex_unlock(&mutex[bus]);
		return -RET_ERROR;
	}
	sys_ret = write(cam_fd[bus], wr_data, 2);
	if(sys_ret != 2) {
		pr_err("write reg16_data16 fail\n");
	    pthread_mutex_unlock(&mutex[bus]);
		return -RET_ERROR;
	}
	sys_ret = read(cam_fd[bus], &rdata, 2);
	if(sys_ret != 2) {
		pr_err("read reg16_data16 fail\n");
		pthread_mutex_unlock(&mutex[bus]);
		return -RET_ERROR;
	}
	value = (rdata[0] << 8) | rdata[1];
	pr_info("success with value 0x%04x \n", value);
	pthread_mutex_unlock(&mutex[bus]);
	return value;
}

int hb_i2c_read_reg16_data8(int bus, uint8_t i2c_addr, uint16_t reg_addr)
{
	int ret = RET_OK, value = 0;
	unsigned char wr_data[2];
	unsigned char rdata = 0;
	int val = 0;
	ssize_t sys_ret;

	wr_data[0] = (uint8_t)((reg_addr >> 8) & 0x00ff);
	wr_data[1] = (uint8_t)(reg_addr & 0x00ff);

	val = pthread_mutex_lock(&mutex[bus]);
	if(val != 0) {
		pr_err("mutex lock error val %d \n", val);
	   	return -RET_ERROR;
	}
	ret = ioctl(cam_fd[bus], I2C_SLAVE_FORCE, i2c_addr);
	if (ret < 0) {
		pr_err("unable open camera with addr 0x%02x ioctl I2C_SLAVE_FORCE error\n", i2c_addr);
		pthread_mutex_unlock(&mutex[bus]);
		return -RET_ERROR;
	}
	sys_ret = write(cam_fd[bus], wr_data, 2);
	if(sys_ret != 2) {
		pr_err("write reg16_data8 fail\n");
		pthread_mutex_unlock(&mutex[bus]);
		return -RET_ERROR;
	}
	sys_ret = read(cam_fd[bus], &rdata, 1);
	if(sys_ret != 1) {
		pr_err("read reg16_data8 fail\n");
		pthread_mutex_unlock(&mutex[bus]);
		return -RET_ERROR;
	}
	value = rdata;
	pthread_mutex_unlock(&mutex[bus]);
	return value;
}

int hb_i2c_read_reg8_data8(int bus, uint8_t i2c_addr, uint8_t reg_addr)
{
	int ret = RET_OK, value = 0;
	unsigned char wr_data;
	unsigned char rdata = 0;
	int val = 0;
	ssize_t sys_ret;

	wr_data = reg_addr;
	val = pthread_mutex_lock(&mutex[bus]);
	if(val != 0) {
		pr_err("mutex lock error val %d \n", val);
	    return -RET_ERROR;
	}
	ret = ioctl(cam_fd[bus], I2C_SLAVE_FORCE, i2c_addr);
	if (ret < 0) {
		pr_err("unable open camera with addr 0x%02x ioctl I2C_SLAVE_FORCE error\n", i2c_addr);
		pthread_mutex_unlock(&mutex[bus]);
		return -RET_ERROR;
	}
	sys_ret = write(cam_fd[bus], &wr_data, 1);
	if(sys_ret != 1) {
		pr_err("write reg8_data8 fail\n");
		pthread_mutex_unlock(&mutex[bus]);
		return -RET_ERROR;
	}
	sys_ret = read(cam_fd[bus], &rdata, 1);
	if(sys_ret != 1) {
		pr_err("read reg8_data8 fail\n");
		pthread_mutex_unlock(&mutex[bus]);
		return -RET_ERROR;
	}
	value = rdata;
	pthread_mutex_unlock(&mutex[bus]);
	return value;
}

int hb_i2c_read_reg8_data16(int bus, uint8_t i2c_addr, uint8_t reg_addr)
{
	int ret = RET_OK, value = 0;
	unsigned char wr_data;
	unsigned char rdata[2] = {0};
	int val = 0;
	ssize_t sys_ret;

	wr_data = reg_addr;
	val = pthread_mutex_lock(&mutex[bus]);
	if(val != 0) {
		pr_err("mutex lock error val %d \n", val);
	    return -RET_ERROR;
	}
	ret = ioctl(cam_fd[bus], I2C_SLAVE_FORCE, i2c_addr);
	if (ret < 0) {
		pr_err("unable open camera with addr 0x%02x ioctl I2C_SLAVE_FORCE error\n", i2c_addr);
		pthread_mutex_unlock(&mutex[bus]);
		return -RET_ERROR;
	}
	sys_ret = write(cam_fd[bus], &wr_data, 1);
	if(sys_ret != 1) {
		pr_err("write reg8_data16 fail\n");
		pthread_mutex_unlock(&mutex[bus]);
		return -RET_ERROR;
	}
	sys_ret = read(cam_fd[bus], &rdata, 2);
	if(sys_ret != 2) {
		pr_err("read reg8_data16 fail\n");
		pthread_mutex_unlock(&mutex[bus]);
		return -RET_ERROR;
	}
	value = (rdata[0] << 8) | rdata[1];
	pthread_mutex_unlock(&mutex[bus]);
	return value;
}

int hb_i2c_write_reg16_data16(int bus, uint8_t i2c_addr,
		uint16_t reg_addr, uint16_t value)
{
	int ret = RET_OK;
	unsigned char wr_data[4];
	int val = 0;
	ssize_t sys_ret;

	val = pthread_mutex_lock(&mutex[bus]);
	if(val != 0) {
		pr_err("mutex lock error val %d \n", val);
	    return -RET_ERROR;
	}
	ret = ioctl(cam_fd[bus], I2C_SLAVE_FORCE, i2c_addr);
	if (ret < 0) {
		pr_err("unable open camera with addr 0x%02x ioctl I2C_SLAVE_FORCE error\n", i2c_addr);
		pthread_mutex_unlock(&mutex[bus]);
		return -RET_ERROR;
	}
	wr_data[0] = (uint8_t)(reg_addr >> 8) & 0x00ff;
	wr_data[1] = (uint8_t)reg_addr & 0x00ff;
	wr_data[2] = (uint8_t)(value>>8) & 0x00ff;
	wr_data[3] = (uint8_t)(value) & 0x00ff;

	sys_ret = write(cam_fd[bus], wr_data, 4);
	if(sys_ret != 4) {
		 pthread_mutex_unlock(&mutex[bus]);
		 return -RET_ERROR;
	}
	pthread_mutex_unlock(&mutex[bus]);
	return RET_OK;
}

int hb_i2c_write_reg16_data8(int bus, uint8_t i2c_addr,
		uint16_t reg_addr, uint8_t value)
{
	int ret = RET_OK;
	unsigned char wr_data[3];
	int val = 0;
	ssize_t sys_ret;

	val = pthread_mutex_lock(&mutex[bus]);
	if(val != 0) {
		pr_err("mutex lock error val %d \n", val);
	    return -RET_ERROR;
	}
	ret = ioctl(cam_fd[bus], I2C_SLAVE_FORCE, i2c_addr);
	if (ret < 0) {
		pr_err("unable open camera with addr 0x%02x ioctl I2C_SLAVE_FORCE error\n",
				i2c_addr);
		pthread_mutex_unlock(&mutex[bus]);
		return -RET_ERROR;
	}
	wr_data[0] = (uint8_t)(reg_addr >> 8) & 0x00ff;
	wr_data[1] = (uint8_t)reg_addr & 0x00ff;
	wr_data[2] = value;

	sys_ret = write(cam_fd[bus], wr_data, 3);
	if(sys_ret != 3) {
		 pthread_mutex_unlock(&mutex[bus]);
		 return -RET_ERROR;
	}
	pthread_mutex_unlock(&mutex[bus]);
	return RET_OK;
}

int hb_i2c_write_reg8_data16(int bus, uint8_t i2c_addr,
		uint8_t reg_addr, uint16_t value)
{
	int ret = RET_OK;
	unsigned char wr_data[3];
	int val = 0;
	ssize_t sys_ret;

	val = pthread_mutex_lock(&mutex[bus]);
	if(val != 0) {
		pr_err("mutex lock error val %d \n", val);
	    return -RET_ERROR;
	}
	ret = ioctl(cam_fd[bus], I2C_SLAVE_FORCE, i2c_addr);
	if (ret < 0) {
		pr_err("unable open camera with addr 0x%02x ioctl I2C_SLAVE_FORCE error\n", i2c_addr);
		pthread_mutex_unlock(&mutex[bus]);
		return -RET_ERROR;
	}
	wr_data[0] = reg_addr;
	wr_data[1]=(uint8_t)(value>>8) & 0x00ff;
	wr_data[2]=(uint8_t)(value) & 0x00ff;

	sys_ret = write(cam_fd[bus], wr_data, 3);
	if(sys_ret != 3) {
		 pthread_mutex_unlock(&mutex[bus]);
		 return -RET_ERROR;
	}
	pthread_mutex_unlock(&mutex[bus]);
	return RET_OK;
}

int hb_i2c_write_reg8_data8(int bus, uint8_t i2c_addr,
		uint8_t reg_addr, uint8_t value)
{
	int ret = RET_OK;
	unsigned char wr_data[2];
	int val = 0;
	ssize_t sys_ret;

	val = pthread_mutex_lock(&mutex[bus]);
	if(val != 0) {
		pr_err("mutex lock error val %d \n", val);
	    return -RET_ERROR;
	}

	ret = ioctl(cam_fd[bus], I2C_SLAVE_FORCE, i2c_addr);
	if (ret < 0) {
		pr_err("unable open camera with addr 0x%02x ioctl I2C_SLAVE_FORCE error\n", i2c_addr);
		pthread_mutex_unlock(&mutex[bus]);
		return -RET_ERROR;
	}
	wr_data[0] = reg_addr;
	wr_data[1] = value;

	sys_ret = write(cam_fd[bus], wr_data, 2);
	if(sys_ret != 2) {
		 pthread_mutex_unlock(&mutex[bus]);
		 return -RET_ERROR;
	}
	pthread_mutex_unlock(&mutex[bus]);
	return RET_OK;
}

int hb_i2c_write_block(int bus, uint8_t i2c_addr,
	uint16_t reg_addr, uint32_t value, uint8_t cnt)
{
	int ret = RET_OK;
	struct i2c_rdwr_ioctl_data data;
	unsigned char sendbuf[12] = {0};
	unsigned char *ptr;
	int val = 0;

	val = pthread_mutex_lock(&mutex[bus]);
	if(val != 0) {
		pr_err("mutex lock error val %d \n", val);
	    return -RET_ERROR;
	}
	if (cnt > 4)
		cnt = 4;

	data.msgs = (struct i2c_msg *)calloc(4, sizeof(struct i2c_msg));

	data.nmsgs = cnt;

	ptr = sendbuf;
	data.msgs[0].len = 3;
	data.msgs[0].addr = i2c_addr;
	data.msgs[0].flags = 0;
	data.msgs[0].buf = ptr;
	data.msgs[0].buf[0] = (reg_addr >> 8u) & 0xffu;
	data.msgs[0].buf[1] = reg_addr & 0xffu;
	data.msgs[0].buf[2] = value & 0xff;

	ptr = sendbuf + 3;
	data.msgs[1].len = 3;
	data.msgs[1].addr = i2c_addr;
	data.msgs[1].flags = 0;
	data.msgs[1].buf = ptr;
	data.msgs[1].buf[0] = ((reg_addr+1) >> 8u) & 0xffu;
	data.msgs[1].buf[1] = (reg_addr+1) & 0xffu;
	data.msgs[1].buf[2] = (value >> 8) & 0xff;

	ptr = sendbuf + 6;
	data.msgs[2].len = 3;
	data.msgs[2].addr = i2c_addr;
	data.msgs[2].flags = 0;
	data.msgs[2].buf = ptr;
	data.msgs[2].buf[0] = ((reg_addr+2) >> 8u) & 0xffu;
	data.msgs[2].buf[1] = (reg_addr+2) & 0xffu;
	data.msgs[2].buf[2] = (value >> 16) & 0xff;

	ptr = sendbuf + 9;
	data.msgs[3].len = 3;
	data.msgs[3].addr = i2c_addr;
	data.msgs[3].flags = 0;
	data.msgs[3].buf = ptr;
	data.msgs[3].buf[0] = ((reg_addr+3) >> 8u) & 0xffu;
	data.msgs[3].buf[1] = (reg_addr+3) & 0xffu;
	data.msgs[3].buf[2] = (uint8_t)(value >> 24) & 0xffu;

	ret = ioctl(cam_fd[bus], I2C_RDWR, (unsigned long)&data);
	if (ret < 0) {
		pr_err("%s failed\n", __func__);
	} else {
		ret = RET_OK;
	}
	free(data.msgs);
	pthread_mutex_unlock(&mutex[bus]);
	return ret;
}

int hb_i2c_read_block_reg16(int bus, uint8_t i2c_addr,
		uint16_t reg_addr, char *buf, uint32_t count)
{
    int ret = RET_OK;
	struct i2c_rdwr_ioctl_data data;
	unsigned char sendbuf[12] = {0};
	int val = 0;

	val = pthread_mutex_lock(&mutex[bus]);
	if(val != 0) {
		pr_err("mutex lock error val %d \n", val);
	    return -RET_ERROR;
	}
	data.msgs = (struct i2c_msg *)calloc(2, sizeof(struct i2c_msg));
	data.nmsgs = 2;

	data.msgs[0].len = 2;
	data.msgs[0].addr = i2c_addr;
	data.msgs[0].flags = 0;
	data.msgs[0].buf = sendbuf;
	data.msgs[0].buf[0] = (reg_addr >> 8u) & 0xffu;
	data.msgs[0].buf[1] = reg_addr & 0xffu;

	data.msgs[1].len = (uint8_t)count;
	data.msgs[1].addr = i2c_addr;
	data.msgs[1].flags = 1;
	data.msgs[1].buf = buf;

	ret = ioctl(cam_fd[bus], I2C_RDWR, (unsigned long)&data);
	if (ret < 0) {
		pr_err("%s failed\n", __func__);
	} else {
		ret = RET_OK;
	}
	free(data.msgs);
	pthread_mutex_unlock(&mutex[bus]);
	return ret;
}

int hb_i2c_read_block_reg8(int bus, uint8_t i2c_addr,
		uint16_t reg_addr, char *buf, uint32_t count)
{
    int ret = RET_OK;
	struct i2c_rdwr_ioctl_data data;
	unsigned char sendbuf[12] = {0};
	int val = 0;

	val = pthread_mutex_lock(&mutex[bus]);
	if(val != 0) {
		pr_err("mutex lock error val %d \n", val);
	    return -RET_ERROR;
	}
	data.msgs = (struct i2c_msg *)calloc(2, sizeof(struct i2c_msg));
	data.nmsgs = 2;

	data.msgs[0].len = 2;
	data.msgs[0].addr = i2c_addr;
	data.msgs[0].flags = 0;
	data.msgs[0].buf = sendbuf;
	data.msgs[0].buf[0] = reg_addr & 0xffu;

	data.msgs[1].len = (uint8_t)count;
	data.msgs[1].addr = i2c_addr;
	data.msgs[1].flags = 1;
	data.msgs[1].buf = buf;

	ret = ioctl(cam_fd[bus], I2C_RDWR, (unsigned long)&data);
	if (ret < 0) {
		pr_err("%s failed\n", __func__);
	} else {
		ret = RET_OK;
	}
	free(data.msgs);
	pthread_mutex_unlock(&mutex[bus]);
	return ret;
}
int hb_i2c_write_block_reg16(int bus, uint8_t i2c_addr,
	uint16_t reg_addr, char *buf, uint32_t count)
{
	int ret = RET_OK;
	struct i2c_rdwr_ioctl_data data;
	char *sendbuf = NULL;
	int val = 0;

	val = pthread_mutex_lock(&mutex[bus]);
	if(val != 0) {
		pr_err("mutex lock error val %d \n", val);
	    return -RET_ERROR;
	}
	sendbuf = malloc(count + 2);
	if(sendbuf == NULL) {
		pr_err("malloc error\n");
	    return -RET_ERROR;
	}
	data.msgs = (struct i2c_msg *)calloc(1, sizeof(struct i2c_msg));
	data.nmsgs = 1;

	data.msgs[0].len = (uint8_t)(2 + count);
	data.msgs[0].addr = i2c_addr;
	data.msgs[0].flags = 0;
	data.msgs[0].buf = sendbuf;
	sendbuf[0] = (reg_addr >> 8u) & 0xffu;
	sendbuf[1] = reg_addr & 0xffu;
	memcpy(sendbuf+2, buf, count);

	ret = ioctl(cam_fd[bus], I2C_RDWR, (unsigned long)&data);
	if (ret < 0) {
		pr_err("%s failed\n", __func__);
	} else {
		ret = RET_OK;
	}
	free(data.msgs);
	free(sendbuf);
	pthread_mutex_unlock(&mutex[bus]);
	return ret;
}

int hb_i2c_write_block_reg8(int bus, uint8_t i2c_addr,
		uint16_t reg_addr, char *buf, uint32_t count)
{
	int ret = RET_OK;
	struct i2c_rdwr_ioctl_data data;
	unsigned char sendbuf[12] = {0};
	int val = 0;

	val = pthread_mutex_lock(&mutex[bus]);
	if(val != 0) {
		pr_err("mutex lock error val %d \n", val);
	    return -RET_ERROR;
	}
	data.msgs = (struct i2c_msg *)calloc(2, sizeof(struct i2c_msg));
	data.nmsgs = 2;

	data.msgs[0].len = 2;
	data.msgs[0].addr = i2c_addr;
	data.msgs[0].flags = 0;
	data.msgs[0].buf = sendbuf;
	data.msgs[0].buf[0] = reg_addr & 0xffu;

	data.msgs[1].len = (uint8_t)count;
	data.msgs[1].addr = i2c_addr;
	data.msgs[1].flags = 0;
	data.msgs[1].buf = buf;

	ret = ioctl(cam_fd[bus], I2C_RDWR, (unsigned long)&data);
	if (ret < 0) {
		pr_err("%s failed\n", __func__);
	} else {
		ret = RET_OK;
	}
	free(data.msgs);
	pthread_mutex_unlock(&mutex[bus]);
	return ret;
}

// PRQA S ALL --
