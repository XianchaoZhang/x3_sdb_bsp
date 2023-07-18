/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2016 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/file.h>
#include <linux/i2c-dev.h>
#include <dlfcn.h>
#include <linux/videodev2.h>
#include <logging.h>
#include <linux/spi/spidev.h>
#include <pthread.h>
#include <time.h>

#include "cJSON.h"
#include "hb_cam_utility.h"
#include "hb_cam_gpio.h"
#include "hb_i2c.h"
#include "hb_spi.h"
#include "hb_cam_interface.h"
#include "cam_common.h"
#include "hb_vin.h"

static int cam_spi_fd[CAM_FD_NUMBER];
static int bus_num = 0;
static uint8_t maddr;
static int gpio_fd[MAX_GPIO_NUM] = {-1};
static uint32_t ae_share_flag;	//src port:0xA0 + bit[16~31], dst port:bit[0~15]
static int snsfds[CAM_MAX_NUM] = {-1};
#define CAM_I2C_RETRY_MAX	10

static int sensor_ctrl_start_i2cbus(uint32_t port,
			uint32_t ops_flag, void *ops);
static void sensor_ctrl_stop_i2cbus(uint32_t port);

void DOFFSET(uint32_t *x, uint32_t n)
{
	switch (n) {
	case 2:
	*x = ((((*x) & 0xff00) >> 8) | (((*x) & 0xff) << 8));
	break;
	case 3:
	*x = (((*x) & 0x0000ff00) + (((*x) & 0xff0000) >> 16) + (((*x) & 0xff) << 16));
	break;
	case 4:
	*x = ((((((*x) & 0xff000000) >> 8) + (((*x) & 0xff0000) << 8)) >> 16) |
		(((((*x) & 0xff00) >> 8) + (((*x) & 0xff) << 8)) << 16));
	break;
	}
}

int camera_power_ctrl(uint32_t gpio, uint32_t on_off)
{
	int ret = RET_OK;
	gpio_info_t gpio_info;
	memset(&gpio_info, 0, sizeof(gpio_info_t));
	gpio_info.gpio = gpio;
	gpio_info.gpio_level = on_off;

	if(gpio >= MAX_GPIO_NUM) {
		pr_err("gpio %d out of range %d\n", gpio);
		return -1;
	}
	if (gpio_fd[gpio] >= 0) {
		ret = ioctl(gpio_fd[gpio], SENSOR_GPIO_CONTROL, &gpio_info);
	} else {
		ret = -1;
	}
	if (ret < 0) {
		pr_err("!!! power_ctrl error ret = %d\n", ret);
	}
	return ret;
}

int serdes_power_ctrl(int32_t serdes_fd, uint32_t gpio, uint32_t on_off)
{
	int ret = RET_OK;
	gpio_info_t gpio_info;
	memset(&gpio_info, 0, sizeof(gpio_info_t));
	gpio_info.gpio = gpio;
	gpio_info.gpio_level = on_off;

	if (serdes_fd >= 0) {
		ret = ioctl(serdes_fd, SERDES_GPIO_CONTROL, &gpio_info);
	} else {
		ret = -1;
	}
	if (ret < 0) {
		pr_err("!!! power_ctrl error ret = %d\n", ret);
	}
	return ret;
}

int camera_write_block(uint16_t reg, uint32_t val, int conti_cnt)
{
	int ret = RET_OK;

	if (!maddr) {
		pr_err("i2c addr not set\n");
		return -RET_ERROR;
	}
	ret = hb_i2c_write_block(bus_num, maddr, reg, val, (uint8_t)conti_cnt);
	if(ret < 0) {
		pr_err("camera write reg %x, val %x failed\n", reg, val);
		return ret;
	}
	return RET_OK;
}

int camera_write_reg(uint16_t reg, uint8_t val)
{
	int ret = RET_OK;

	if (!maddr) {
		pr_err("i2c addr not set\n");
		return -1;
	}
	ret = hb_i2c_write_reg16_data8(bus_num, maddr, reg, val);
	if(ret < 0) {
		pr_err("camera: write %x = %x\n", reg, val);
		return -RET_ERROR;
	}

	return RET_OK;
}

int camera_read_reg(uint16_t reg, uint8_t *val)
{
	int ret = RET_OK;

	if (!maddr) {
		pr_err("i2c addr not set\n");
		return -RET_ERROR;
	}
	ret = hb_i2c_read_reg16_data8(bus_num, maddr, reg);
	if (ret < 0) {
		pr_err("i2c read reg16 data8 fail!!! %d\n", ret);
		return ret;
	}
	*val = (uint8_t)ret;

	return 0;
}

int camera_read_reg8(uint8_t reg, uint8_t *val)
{
	int ret = RET_OK;

	if (!maddr) {
		pr_err("i2c addr not set\n");
		return -RET_ERROR;
	}

	ret = hb_i2c_read_reg8_data8(bus_num, maddr, reg);
	if (ret < 0) {
		pr_err("i2c read reg8 data8 fail!!! %d\n", ret);
		return ret;
	}
	*val = (uint8_t)ret;

	return ret;
}
int camera_write_addr16_reg16(uint16_t addr, uint16_t value)
{
	int ret = RET_OK;

	ret = hb_i2c_write_reg16_data16(bus_num, maddr, addr, value);
	if(ret < 0) {
		ret = -RET_ERROR;
		pr_err("camera_write_addr16_reg16 failed!\n");
	}
	return ret;
}

int camera_write_array(int bus, uint32_t i2c_addr, int reg_width,
			int setting_size, uint32_t *cam_setting)
{
	x2_camera_i2c_t i2c_cfg;
	int ret = RET_OK, i, k;

	i2c_cfg.i2c_addr = i2c_addr;
	i2c_cfg.reg_size = reg_width;

	for(i = 0; i < setting_size; i++) {
		i2c_cfg.reg = cam_setting[2 * i];
		i2c_cfg.data = cam_setting[2 * i + 1];
		if (i2c_cfg.reg_size == 2) {
			if (i2c_cfg.reg > 0xffff)
				return -RET_ERROR;
			ret = hb_i2c_write_reg16_data8(bus, (uint8_t)i2c_cfg.i2c_addr,
						(uint16_t)i2c_cfg.reg, (uint8_t)i2c_cfg.data);
		} else {
			if (i2c_cfg.reg > 0xff)
				return -RET_ERROR;
			ret = hb_i2c_write_reg8_data8(bus, (uint8_t)i2c_cfg.i2c_addr,
					(uint8_t)i2c_cfg.reg, (uint8_t)i2c_cfg.data);
		}
		k = CAM_I2C_RETRY_MAX;
		while (ret < 0 && k--) {
			if (k % 10 == 9)
				usleep(200 * 1000);
			if (i2c_cfg.reg_size == 2)
				ret = hb_i2c_write_reg16_data8(bus, (uint8_t)i2c_cfg.i2c_addr,
						(uint16_t)i2c_cfg.reg, (uint8_t)i2c_cfg.data);
			else
				ret = hb_i2c_write_reg8_data8(bus, (uint8_t)i2c_cfg.i2c_addr,
						(uint8_t)i2c_cfg.reg, (uint8_t)i2c_cfg.data);
		}
		if (ret < 0) {
			pr_err("camera write 0x%2x fail \n", i2c_cfg.reg);
			break;
		}
	}
	return ret;
}

int camera_i2c_write_block(int bus, int reg_width, int device_addr,  uint32_t reg_addr, char *buffer, uint32_t size)
{
	int ret = RET_OK;

	if(reg_width == REG_WIDTH_16bit) {
		if(reg_addr > 0xffff)
		   return -RET_ERROR;
		ret = hb_i2c_write_block_reg16(bus, (uint8_t)device_addr,
					(uint16_t)reg_addr, buffer, size);
	} else {
		if(reg_addr > 0xff)
		  return -RET_ERROR;
		ret = hb_i2c_write_block_reg8(bus, (uint8_t)device_addr,
					(uint8_t)reg_addr, buffer, size);
	}
	return ret;
}

int camera_i2c_read_block(int bus, int reg_width, int device_addr,  uint32_t reg_addr, char *buffer, uint32_t size)
{
	int ret = RET_OK;

	if(reg_width == REG_WIDTH_16bit) {
		if(reg_addr > 0xffff)
		   return -RET_ERROR;
		ret = hb_i2c_read_block_reg16(bus, (uint8_t)device_addr,
					(uint16_t)reg_addr, buffer, size);
	} else {
		if(reg_addr > 0xff)
		  return -RET_ERROR;
		ret = hb_i2c_read_block_reg8(bus, (uint8_t)device_addr,
					(uint8_t)reg_addr, buffer, size);
	}
	return ret;
}

int camera_i2c_write16(int bus, int reg_width, uint8_t sensor_addr,
		uint32_t reg_addr, uint16_t value)
{
	int ret = RET_OK;

	if(reg_width == REG_WIDTH_16bit) {
		if(reg_addr > 0xffff)
		   return -RET_ERROR;
		ret = hb_i2c_write_reg16_data16(bus, sensor_addr, (uint16_t)reg_addr, value);
	} else {
		if(reg_addr > 0xff)
		  return -RET_ERROR;
		ret = hb_i2c_write_reg8_data16(bus, sensor_addr, (uint8_t)reg_addr, value);
	}
	return ret;
}

int camera_i2c_write8(int bus, int reg_width, uint8_t sensor_addr,
		uint32_t reg_addr, uint8_t value)
{
	int ret = RET_OK;

	if(reg_width == REG_WIDTH_16bit) {
		if(reg_addr > 0xffff)
		   return -RET_ERROR;
		ret = hb_i2c_write_reg16_data8(bus, sensor_addr, (uint16_t)reg_addr, value);
	} else {
		if(reg_addr > 0xff)
		  return -RET_ERROR;
		ret = hb_i2c_write_reg8_data8(bus, sensor_addr, (uint8_t)reg_addr, value);
	}
	return ret;
}

int camera_i2c_read16(int bus, int reg_width, uint8_t sensor_addr,
		uint32_t reg_addr)
{
	int val = 0;

	if(reg_width == REG_WIDTH_16bit) {
		if(reg_addr > 0xffff)
		   return -RET_ERROR;
		val = hb_i2c_read_reg16_data16(bus, sensor_addr, (uint16_t)reg_addr);
	} else {
		if(reg_addr > 0xff)
		  return -RET_ERROR;
		val = hb_i2c_read_reg8_data16(bus, sensor_addr, (uint8_t)reg_addr);
	}
	pr_info(" reg_addr 0x%x val 0x%x\n", reg_addr, val);
	return val;
}

/*16位或8位地址 读出来得数据8位*/
int camera_i2c_read8(int bus, int reg_width, uint8_t sensor_addr,
		uint32_t reg_addr)
{
	int val = 0;

	if(reg_width == REG_WIDTH_16bit) {
		if(reg_addr > 0xffff)
		   return -RET_ERROR;
		val = hb_i2c_read_reg16_data8(bus, sensor_addr, (uint16_t)reg_addr);
	} else {
		if(reg_addr > 0xff)
		  return -RET_ERROR;
		val = hb_i2c_read_reg8_data8(bus, sensor_addr, (uint8_t)reg_addr);
	}
	return val;
}

int camera_spi_write_block(int bus, char *buffer, int size)
{
	int ret = RET_OK;

	ret = hb_spi_write_block(cam_spi_fd[bus], buffer, size);
	if(ret < 0) {
		pr_err("camera: spi write 0x%x block fail\n", buffer[0]);
		return -HB_CAM_SPI_WRITE_BLOCK_FAIL;
	}
	return ret;
}

int camera_spi_read_block(int bus, char *buffer, int size)
{
	int ret = RET_OK;

	ret = hb_spi_read_block(cam_spi_fd[bus], buffer, size);
	if(ret < 0) {
		pr_err("camera: spi read block fail\n");
		return -HB_CAM_SPI_READ_BLOCK_FAIL;
	}
	return ret;
}

static int cam_spi_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	char str[12] = {0};
	uint8_t mode, bits;
	uint32_t speed;

	if(cam_spi_fd[sensor_info->bus_num] <= 0) {
		snprintf(str, sizeof(str), "/dev/spidev%d.%d", sensor_info->bus_num, sensor_info->spi_info.spi_cs);
		cam_spi_fd[sensor_info->bus_num]= open(str, O_RDWR | O_CLOEXEC);
		if(cam_spi_fd[sensor_info->bus_num] < 0) {
			pr_err("open spidev%d.%d fail\n", sensor_info->bus_num, sensor_info->spi_info.spi_cs);
			return -RET_ERROR;
		}
	}
	ret = ioctl(cam_spi_fd[sensor_info->bus_num], SPI_IOC_RD_MODE, &mode);
    if (ret < 0) {
        pr_err("can't set spi mode");
		return -RET_ERROR;
    }
	ret = ioctl(cam_spi_fd[sensor_info->bus_num], SPI_IOC_WR_MODE, &sensor_info->spi_info.spi_mode);
    if (ret < 0) {
        pr_err("can't set spi mode");
		return -RET_ERROR;
    }
    ret = ioctl(cam_spi_fd[sensor_info->bus_num], SPI_IOC_RD_BITS_PER_WORD, &bits);
    if (ret < 0) {
        pr_err("can't set spi mode");
		return -RET_ERROR;
    }
	ret = ioctl(cam_spi_fd[sensor_info->bus_num], SPI_IOC_WR_MAX_SPEED_HZ, &sensor_info->spi_info.spi_speed);
    if (ret < 0) {
        pr_err("can't set spi mode");
		return -RET_ERROR;
    }
    ret = ioctl(cam_spi_fd[sensor_info->bus_num], SPI_IOC_RD_MAX_SPEED_HZ, &speed);
    if (ret < 0) {
        pr_err("can't set spi mode");
		return -RET_ERROR;
    }
	return ret;
}

static int camera_open_sensorfd(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	char str[12] = {0};

	ret = hb_vin_open(sensor_info->entry_num);
	if (ret < 0) {
		pr_err("port %d vin host %d open fail\n",
			sensor_info->dev_port, sensor_info->entry_num);
		return -RET_ERROR;
	}
	snprintf(str, sizeof(str), "/dev/port_%d", sensor_info->dev_port);
	if(sensor_info->sen_devfd <= 0) {
		if ((sensor_info->sen_devfd = open(str, O_RDWR | O_CLOEXEC)) < 0) {
			pr_err("sensor_%d open fail\n", sensor_info->dev_port);
			hb_vin_close(sensor_info->entry_num);
			return -RET_ERROR;
		}
		snsfds[sensor_info->port] = sensor_info->sen_devfd;
		pr_info("/dev/port_%d success sensor_info->sen_devfd %d\n",
		sensor_info->dev_port, sensor_info->sen_devfd);
	}

	return ret;
}
#define STRING_LENGTH  24
static int serdes_open_serdesfd(deserial_info_t *serdes_info)
{
	int ret = RET_OK;
	char str[STRING_LENGTH] = {0};

	snprintf(str, sizeof(str), "/dev/serdes_%d", serdes_info->serdes_index);
	if(serdes_info->serdes_fd <= 0) {
		if ((serdes_info->serdes_fd = open(str, O_RDWR | O_CLOEXEC)) < 0) {
			pr_err("serdes_%d open  errno %d fail %s\n",
				serdes_info->serdes_index, errno, strerror(errno));
			return -RET_ERROR;
		}
		pr_info("/dev/serdes_%d success serdes_info->serdes_fd %d\n",
			serdes_info->serdes_index, serdes_info->serdes_fd);
	}

	return ret;
}

static int camera_create_share_mutex(sensor_info_t *sensor_info)
{
	int ret = RET_OK;

	/*Initializing process mutexes and shared memory*/
	if (sensor_info->entry_num == 0 && sensor_info->mp == NULL) {
		sensor_info->mp = camera_create_mutex_package(ENTRY0_KEY);
	} else if (sensor_info->entry_num == 1 && sensor_info->mp == NULL) {
		sensor_info->mp = camera_create_mutex_package(ENTRY1_KEY);
	} else if (sensor_info->entry_num == 2 && sensor_info->mp == NULL) {
		sensor_info->mp = camera_create_mutex_package(ENTRY2_KEY);
	} else if (sensor_info->entry_num == 3 && sensor_info->mp == NULL) {
		sensor_info->mp = camera_create_mutex_package(ENTRY3_KEY);
	}
	if(sensor_info->mp != NULL) {
		pr_info("mp %p shmid %d \n", sensor_info->mp,
			sensor_info->mp->shmid); /* PRQA S ALL */
	} else {
		pr_err("create_mutex_package fail, mp is null\n");
		ret = -RET_ERROR;
	}
	pr_info("mp %p shmid %d  entry_num %d\n", sensor_info->mp,
		sensor_info->mp->shmid, sensor_info->entry_num); /* PRQA S ALL */
	return ret;
}

static int camera_init_deinit_lock(sensor_info_t *sensor_info, int *init_cnt)
{
	int ret = RET_OK;

	if (sensor_info->dev_port < 0) {
		*init_cnt = 0;
		pr_info("%s ignore dev_port,return ok\n", __func__);
		return RET_OK;
	}
	ret = ioctl(sensor_info->sen_devfd, SENSOR_USER_LOCK, 0);
	if (ret < 0) {
		pr_err("sen_devfd %d ioctl SENSOR_USER_LOCK fail(%s)\n",
			sensor_info->sen_devfd, strerror(errno));
		return -RET_ERROR;
	}
	ret = ioctl(sensor_info->sen_devfd, SENSOR_GET_INIT_CNT, init_cnt);
	if (ret < 0) {
		ret = ioctl(sensor_info->sen_devfd, SENSOR_USER_UNLOCK, 0);
		if (ret < 0) {
			pr_err("sen_devfd %d ioctl SENSOR_USER_UNLOCK fail(%s)",
				sensor_info->sen_devfd, strerror(errno));
		}
		pr_err("sen_devfd %d ioctl SENSOR_GET_INIT_CNT fail(%s)\n",
				sensor_info->sen_devfd, strerror(errno));
		return -RET_ERROR;
	}
	return ret;
}

static int camera_init_deinit_get_unlock(sensor_info_t *sensor_info)
{
	int init_cnt;
	int ret = RET_OK;

	if (sensor_info->dev_port < 0) {
		pr_info("%s ignore dev_port,return ok\n", __func__);
		return RET_OK;
	}
	ret = ioctl(sensor_info->sen_devfd, SENSOR_GET_INIT_CNT, &init_cnt);
	if (ret < 0) {
		ret = ioctl(sensor_info->sen_devfd, SENSOR_USER_UNLOCK, 0);
		if (ret < 0) {
			pr_err("sen_devfd %d ioctl SENSOR_USER_UNLOCK fail(%s)",
				sensor_info->sen_devfd, strerror(errno));
		}
		pr_err("sen_devfd %d ioctl SENSOR_GET_INIT_CNT fail(%s)\n",
			sensor_info->sen_devfd, strerror(errno));
		return -RET_ERROR;
	}
	init_cnt += 1;
	ret = ioctl(sensor_info->sen_devfd, SENSOR_SET_INIT_CNT, &init_cnt);
	if (ret < 0) {
		ret = ioctl(sensor_info->sen_devfd, SENSOR_USER_UNLOCK, 0);
		if (ret < 0) {
			pr_err("sen_devfd %d ioctl SENSOR_USER_UNLOCK fail(%s)",
				sensor_info->sen_devfd, strerror(errno));
		}
		pr_err("sen_devfd %d ioctl SENSOR_SET_INIT_CNT fail(%s)\n",
			sensor_info->sen_devfd, strerror(errno));
		return -RET_ERROR;
	}
	ret = ioctl(sensor_info->sen_devfd, SENSOR_USER_UNLOCK, 0);
	if (ret < 0) {
		pr_err("sen_devfd %d ioctl SENSOR_USER_UNLOCK fail(%s)\n",
			sensor_info->sen_devfd, strerror(errno));
		return -RET_ERROR;
	}
	return ret;
}

static int camera_init_deinit_put_unlock(sensor_info_t *sensor_info)
{
	int init_cnt;
	int ret = RET_OK;

	if (sensor_info->dev_port < 0) {
		pr_info("%s ignore dev_port,return ok\n", __func__);
		return RET_OK;
	}
	ret = ioctl(sensor_info->sen_devfd, SENSOR_GET_INIT_CNT, &init_cnt);
	if (ret < 0) {
		ret = ioctl(sensor_info->sen_devfd, SENSOR_USER_UNLOCK, 0);
		if (ret < 0) {
			pr_err("sen_devfd %d ioctl SENSOR_USER_UNLOCK fail(%s)\n",
				sensor_info->sen_devfd, strerror(errno));
		}
		pr_err("sen_devfd %d ioctl SENSOR_GET_INIT_CNT fail(%s)\n",
			sensor_info->sen_devfd, strerror(errno));
		return -RET_ERROR;
	}
	init_cnt -= 1;
	ret = ioctl(sensor_info->sen_devfd, SENSOR_SET_INIT_CNT, &init_cnt);
	if (ret < 0) {
		ret = ioctl(sensor_info->sen_devfd, SENSOR_USER_UNLOCK, 0);
		if (ret < 0) {
			pr_err("sen_devfd %d ioctl SENSOR_USER_UNLOCK fail(%s)\n",
					sensor_info->sen_devfd, strerror(errno));
		}
		pr_err("sen_devfd %d ioctl SENSOR_SET_INIT_CNT fail(%s)\n",
			sensor_info->sen_devfd, strerror(errno));
		return -RET_ERROR;
	}
	ret = ioctl(sensor_info->sen_devfd, SENSOR_USER_UNLOCK, 0);
	if (ret < 0) {
		pr_err("sen_devfd %d ioctl SENSOR_USER_UNLOCK fail(%s)\n",
			sensor_info->sen_devfd, strerror(errno));
		return -RET_ERROR;
	}
	return ret;
}

static int camera_start_stop_lock(sensor_info_t *sensor_info, int *start_cnt)
{
	int ret = RET_OK;

	if (sensor_info->dev_port < 0) {
		*start_cnt = 0;
		pr_info("%s ignore dev_port,return ok\n", __func__);
		return RET_OK;
	}
	ret = ioctl(sensor_info->sen_devfd, SENSOR_USER_LOCK, 0);
	if (ret < 0) {
		pr_err("sen_devfd %d ioctl SENSOR_USER_LOCK fail(%s)\n",
			sensor_info->sen_devfd, strerror(errno));
		return -RET_ERROR;
	}
	ret = ioctl(sensor_info->sen_devfd, SENSOR_GET_START_CNT, start_cnt);
	if (ret < 0) {
		ret = ioctl(sensor_info->sen_devfd, SENSOR_USER_UNLOCK, 0);
		if (ret < 0) {
			pr_err("sen_devfd %d ioctl SENSOR_USER_UNLOCK fail(%s)\n",
					sensor_info->sen_devfd, strerror(errno));
		}
		pr_err("sen_devfd %d ioctl SENSOR_GET_START_CNT fail(%s)\n",
			sensor_info->sen_devfd, strerror(errno));
		return -RET_ERROR;
	}
	return ret;
}

int camera_start_stop_get_unlock(sensor_info_t *sensor_info)
{
	int start_cnt;
	int ret = RET_OK;

	if (sensor_info->dev_port < 0) {
		pr_info("%s ignore dev_port,return ok\n", __func__);
		return RET_OK;
	}
	ret = ioctl(sensor_info->sen_devfd, SENSOR_GET_START_CNT, &start_cnt);
	if (ret < 0) {
		ret = ioctl(sensor_info->sen_devfd, SENSOR_USER_UNLOCK, 0);
		if (ret < 0) {
			pr_err("sen_devfd %d ioctl SENSOR_USER_UNLOCK fail(%s)\n",
					sensor_info->sen_devfd, strerror(errno));
		}
		pr_err("sen_devfd %d ioctl SENSOR_GET_START_CNT fail(%s)\n",
			sensor_info->sen_devfd, strerror(errno));
		return -RET_ERROR;
	}
	start_cnt += 1;
	ret = ioctl(sensor_info->sen_devfd, SENSOR_SET_START_CNT, &start_cnt);
	if (ret < 0) {
		ret = ioctl(sensor_info->sen_devfd, SENSOR_USER_UNLOCK, 0);
		if (ret < 0) {
			pr_err("sen_devfd %d ioctl SENSOR_USER_UNLOCK fail(%s)\n",
					sensor_info->sen_devfd, strerror(errno));
		}
		pr_err("sen_devfd %d ioctl SENSOR_SET_START_CNT fail(%s)\n",
			sensor_info->sen_devfd, strerror(errno));
		return -RET_ERROR;
	}
	ret = ioctl(sensor_info->sen_devfd, SENSOR_USER_UNLOCK, 0);
	if (ret < 0) {
		pr_err("sen_devfd %d ioctl SENSOR_USER_UNLOCK fail(%s)\n",
			sensor_info->sen_devfd, strerror(errno));
		return -RET_ERROR;
	}
	return ret;
}

static int camera_start_stop_put_unlock(sensor_info_t *sensor_info)
{
	int start_cnt;
	int ret = RET_OK;

	if (sensor_info->dev_port < 0) {
		pr_info("%s ignore dev_port,return ok\n", __func__);
		return RET_OK;
	}
	ret = ioctl(sensor_info->sen_devfd, SENSOR_GET_START_CNT, &start_cnt);
	if (ret < 0) {
		ret = ioctl(sensor_info->sen_devfd, SENSOR_USER_UNLOCK, 0);
		if (ret < 0) {
			pr_err("sen_devfd %d ioctl SENSOR_USER_UNLOCK fail(%s)\n",
					sensor_info->sen_devfd, strerror(errno));
		}
		pr_err("sen_devfd %d ioctl SENSOR_GET_START_CNT fail(%s)\n",
			sensor_info->sen_devfd, strerror(errno));
		return -RET_ERROR;
	}
	start_cnt -= 1;
	ret = ioctl(sensor_info->sen_devfd, SENSOR_SET_START_CNT, &start_cnt);
	if (ret < 0) {
		ret = ioctl(sensor_info->sen_devfd, SENSOR_USER_UNLOCK, 0);
		if (ret < 0) {
			pr_err("sen_devfd %d ioctl SENSOR_USER_UNLOCK fail(%s)\n",
					sensor_info->sen_devfd, strerror(errno));
		}
		pr_err("sen_devfd %d ioctl SENSOR_SET_START_CNT fail(%s)\n",
			sensor_info->sen_devfd, strerror(errno));
		return -RET_ERROR;
	}
	ret = ioctl(sensor_info->sen_devfd, SENSOR_USER_UNLOCK, 0);
	if (ret < 0) {
		pr_err("sen_devfd %d ioctl SENSOR_USER_UNLOCK fail(%s)\n",
			sensor_info->sen_devfd, strerror(errno));
		return -RET_ERROR;
	}
	pr_info("camera_start_stop_put_unlock start_cnt %d dev_port %d\n",
		start_cnt, sensor_info->dev_port);
	return ret;
}

static int serdes_init_deinit_lock(deserial_info_t *deserial_info,
		int *init_cnt)
{
	int ret = RET_OK;

	ret = ioctl(deserial_info->serdes_fd, SERDES_USER_LOCK, 0);
	if (ret < 0) {
		pr_err("serdes_fd %d ioctl SENSOR_USER_LOCK fail(%s)\n",
			deserial_info->serdes_fd, strerror(errno));
		return -RET_ERROR;
	}
	ret = ioctl(deserial_info->serdes_fd, SERDES_GET_INIT_CNT, init_cnt);
	if (ret < 0) {
		ret = ioctl(deserial_info->serdes_fd, SERDES_USER_UNLOCK, 0);
		if (ret < 0) {
			pr_err("serdes_fd %d ioctl SERDES_USER_UNLOCK fail(%s)",
				deserial_info->serdes_fd, strerror(errno));
		}
		pr_err("sen_devfd %d ioctl SERDES_GET_INIT_CNT fail(%s)\n",
				deserial_info->serdes_fd, strerror(errno));
		return -RET_ERROR;
	}
	return ret;
}

static int serdes_init_deinit_get_unlock(deserial_info_t *deserial_info)
{
	int init_cnt;
	int ret = RET_OK;

	ret = ioctl(deserial_info->serdes_fd, SERDES_GET_INIT_CNT, &init_cnt);
	if (ret < 0) {
		ret = ioctl(deserial_info->serdes_fd, SERDES_USER_UNLOCK, 0);
		if (ret < 0) {
			pr_err("serdes_fd %d ioctl SERDES_USER_UNLOCK fail(%s)",
				deserial_info->serdes_fd, strerror(errno));
		}
		pr_err("serdes_fd %d ioctl SERDES_GET_INIT_CNT fail(%s)\n",
			deserial_info->serdes_fd, strerror(errno));
		return -RET_ERROR;
	}
	init_cnt += 1;
	ret = ioctl(deserial_info->serdes_fd, SERDES_SET_INIT_CNT, &init_cnt);
	if (ret < 0) {
		ret = ioctl(deserial_info->serdes_fd, SERDES_USER_UNLOCK, 0);
		if (ret < 0) {
			pr_err("serdes_fd %d ioctl SERDES_USER_UNLOCK fail(%s)",
				deserial_info->serdes_fd, strerror(errno));
		}
		pr_err("serdes_fd %d ioctl SERDES_SET_INIT_CNT fail(%s)\n",
			deserial_info->serdes_fd, strerror(errno));
		return -RET_ERROR;
	}
	ret = ioctl(deserial_info->serdes_fd, SERDES_USER_UNLOCK, 0);
	if (ret < 0) {
		pr_err("serdes_fd %d ioctl SERDES_USER_UNLOCK fail(%s)\n",
			deserial_info->serdes_fd, strerror(errno));
	}
	return ret;
}

int hb_deserial_power_ctrl(deserial_info_t *deserial_info)
{
	int32_t ret = 0;
	int32_t init_cnt, gpio;

	if(deserial_info->power_mode == 0)
		return ret;
	ret = serdes_open_serdesfd(deserial_info);
	if (ret < 0) {
		return ret;
	}
	/*lock */
	ret = serdes_init_deinit_lock(deserial_info, &init_cnt);
	if (ret < 0) {
		pr_err("serdes_init_deinit_lock fail init_cnt %d serdes_index %d\n",
			init_cnt, deserial_info->serdes_index);
		goto close_serdesfd;
	}
	pr_info("deserial_info serdes_index %d init_cnt %d\n",
			deserial_info->serdes_index, init_cnt);
	if(init_cnt == 0) {
		if(deserial_info->power_mode) {
			for(gpio = 0; gpio < deserial_info->gpio_num; gpio++) {
				if(deserial_info->gpio_pin[gpio] >= 0) {
					ret = serdes_power_ctrl(deserial_info->serdes_fd,
						deserial_info->gpio_pin[gpio],
						deserial_info->gpio_level[gpio]);
					if(ret < 0) {
						pr_err("deserial_power_ctrl fail\n");
					    goto unlock;
					}
				}
				usleep(deserial_info->power_delay*1000);
				ret = serdes_power_ctrl(deserial_info->serdes_fd,
						deserial_info->gpio_pin[gpio],
						1-deserial_info->gpio_level[gpio]);
				if(ret < 0) {
					pr_err("deserial_power_ctrl fail\n");
				    goto unlock;
				}
			}
		}
	}
	/*unlock*/
	ret = serdes_init_deinit_get_unlock(deserial_info);
	if (ret < 0) {
		pr_err("serdes_init_deinit_get_unlock fail\n");
	}
	return ret;
unlock:
	ret = ioctl(deserial_info->serdes_fd, SERDES_USER_UNLOCK, 0);
	if (ret < 0) {
		pr_err("serdes_fd %d ioctl SERDES_USER_UNLOCK fail(%s)",
			deserial_info->serdes_fd, strerror(errno));
	}
close_serdesfd:
	if(deserial_info->serdes_fd > 0) {
		close(deserial_info->serdes_fd);
		deserial_info->serdes_fd = -1;
	}
	return ret;
}

int hb_deserial_init(deserial_info_t *deserial_info)
{
	int ret = RET_OK;
	char deserial_buff[128] = {0};

	hb_i2c_init(deserial_info->bus_num);
	ret = hb_deserial_power_ctrl(deserial_info);
	if(ret < 0) {
		pr_err("hb_deserial_power_ctrl error %d index %d\n",
			ret, deserial_info->serdes_index);
		goto free_i2cinit;
	}
	if(deserial_info->deserial_name) {
		snprintf(deserial_buff, sizeof(deserial_buff), "lib%s.so",
				deserial_info->deserial_name);
		if(NULL == deserial_info->deserial_fd) {
			deserial_info->deserial_fd =  dlopen(deserial_buff, RTLD_LAZY);
			if (deserial_info->deserial_fd == NULL) {
					pr_err("dlopen get error %s\n", dlerror());
					goto free_i2cinit;
			}
		}
		dlerror();
		deserial_info->deserial_ops = dlsym(deserial_info->deserial_fd,
				deserial_info->deserial_name);
		if (deserial_info->deserial_ops == NULL) {
			pr_err("dlsym get error %s\n", dlerror());
			goto free_deserial;
		}
	} else {
		pr_err("there's no deserial\n");
		goto free_i2cinit;
	}
	return ret;

free_deserial:
	if(deserial_info->deserial_fd) {
	    dlclose(deserial_info->deserial_fd);
	    deserial_info->deserial_fd = NULL;
	}
free_i2cinit:
	hb_i2c_deinit(deserial_info->bus_num);
	return -RET_ERROR;
}

int hb_camera_sensorlib_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	char name_buff[128] = {0};

	snprintf(name_buff, sizeof(name_buff),  "lib%s.so", sensor_info->sensor_name);
	if(NULL == sensor_info->sensor_fd) {
		sensor_info->sensor_fd =  dlopen(name_buff, RTLD_LAZY);
			if (sensor_info->sensor_fd == NULL) {
				pr_err("dlopen %s  error %s\n", name_buff, dlerror());
				return -1;
			}
	}
	dlerror();
	sensor_info->sensor_ops = dlsym(sensor_info->sensor_fd, sensor_info->sensor_name);
	if (sensor_info->sensor_ops == NULL) {
		pr_err("dlsym get error %s\n", dlerror());
		goto err;
	}
	return ret;
err:
	if(sensor_info->sensor_fd) {
		dlclose(sensor_info->sensor_fd);
		sensor_info->sensor_fd = NULL;
	}
	return -RET_ERROR;
}
static int32_t hb_sensor_init_process(sensor_info_t *sensor_info)
{
	int32_t ret = 0;
	uint32_t src_port = 0;
	uint32_t userspace_enable = 0;
	uint32_t port = sensor_info->port;

	src_port = (ae_share_flag >> 16) & 0xff;
	if (port < CAM_MAX_NUM && snsfds[port] > 0 && src_port - 0xA0 == port) {
		ret = ioctl(snsfds[port], SENSOR_AE_SHARE, &ae_share_flag);
		if (ret < 0) {
			pr_err("%s port_%d ioctl fail %d\n", __func__, port, ret);
			return ret;
		}
	}
	if (NULL != ((sensor_module_t *)(sensor_info->sensor_ops))->ae_share_init) {
		ret = ((sensor_module_t *)(sensor_info->sensor_ops))->ae_share_init(ae_share_flag);
		if (ret < 0) {
			pr_err("ae_share_init fail\n");
			return ret;
		}
		ae_share_flag = 0;
	}
	ret = ((sensor_module_t *)(sensor_info->sensor_ops))->init(sensor_info);
	if (ret < 0) {
		pr_err("sensor_init fail\n");
		return ret;
	}

	if (NULL != ((sensor_module_t *)(sensor_info->sensor_ops))->userspace_control) {
		ret = ((sensor_module_t *)(sensor_info->sensor_ops))->userspace_control(port, &userspace_enable);
		if ((ret == 0) && (userspace_enable)) {
			ret = sensor_ctrl_start_i2cbus(sensor_info->dev_port, userspace_enable, sensor_info);
			if (ret < 0) {
				pr_err("sensor_statr pthread fail\n");
			}
		}
	}
	return ret;
}

void camera_res_release(sensor_info_t *sensor_info)
{
	int i;

	hb_i2c_deinit(sensor_info->bus_num);
	if (cam_spi_fd[sensor_info->bus_num] > 0) {
		close(cam_spi_fd[sensor_info->bus_num]);
		cam_spi_fd[sensor_info->bus_num] = -1;
	}
	if (sensor_info->power_mode) {
		for(i = 0; i < sensor_info->gpio_num; i++) {
			if (gpio_fd[sensor_info->gpio_pin[i]] >= 0) {
				close(gpio_fd[sensor_info->gpio_pin[i]]);
				gpio_fd[sensor_info->gpio_pin[i]] = -1;
			}
		}
	}
	if (sensor_info->sensor_fd) {
		dlclose(sensor_info->sensor_fd);
		sensor_info->sensor_fd = NULL;
		sensor_info->sensor_ops = NULL;
	}
	if (sensor_info->sen_devfd >= 0) {
		close(sensor_info->sen_devfd);
		sensor_info->sen_devfd = -1;
	}
	/*Use its flag variable to index the shared memory it occupies and destroy it*/
	camera_destory_mutex_package(sensor_info->mp);
	sensor_info->mp = NULL;
}

/*for hapi*/
int32_t hb_cam_utility_ex(int32_t cam_ctl, sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK, i;
	int32_t init_cnt = 0;
	int32_t start_cnt = 0;
	uint32_t port = sensor_info->port;
	uint32_t userspace_enable = 0;
	char str[12] = {0};

	if(cam_ctl == CAM_INIT) {
		if(sensor_info->bus_type == I2C_BUS) {
			hb_i2c_init(sensor_info->bus_num);
		} else if (sensor_info->bus_type == SPI_BUS) {
			cam_spi_init(sensor_info);
		}
		maddr = (uint8_t)sensor_info->sensor_addr;
		snprintf(str, sizeof(str), "/dev/port_%d", 0);
		if(sensor_info->power_mode) {
			for(i = 0; i < sensor_info->gpio_num; i++) {
				gpio_fd[sensor_info->gpio_pin[i]] = open(str, O_RDWR | O_CLOEXEC);
				if(gpio_fd[sensor_info->gpio_pin[i]] < 0)
					goto close_i2cfd;
			}
		}
		ret = camera_open_sensorfd(sensor_info);
		if (ret < 0) {
			pr_err("camera_open_sensorfd fail, port %d\n",
				sensor_info->port);
			goto close_gpiofd;
		}
		ret = camera_create_share_mutex(sensor_info);
		if (ret < 0) {
			pr_err("camera_create_share_mutex fail\n");
			goto close_senfd;
		}
		/*lock */
		ret = camera_init_deinit_lock(sensor_info, &init_cnt);
		if (ret < 0) {
			pr_err("camera_init_deinit_lock fail init_cnt %d port %d\n",
				init_cnt, sensor_info->port);
			goto err;
		}
		pr_info("sensor_info dev_port %d init_cnt %d\n",
				sensor_info->dev_port, init_cnt);
		if(init_cnt == 0) {
			ret = hb_sensor_init_process(sensor_info);
			if (ret < 0) {
				pr_err("hb_sensor_init_process fail\n");
				ioctl(sensor_info->sen_devfd, SENSOR_USER_UNLOCK, 0);
				goto err;
			}
		}
		ret = camera_init_deinit_get_unlock(sensor_info);
		if (ret < 0) {
			pr_err("camera_init_deinit_get_unlock fail\n");
			goto err;
		}
	}
	if(cam_ctl == CAM_START) {
		if(NULL == sensor_info->sensor_ops || NULL == ((sensor_module_t *)(sensor_info->sensor_ops))->start) {
			return -RET_ERROR;
		}
		ret = camera_start_stop_lock(sensor_info, &start_cnt);
		if (ret < 0) {
			pr_err("CAM_START camera_start_stop_lock fail\n");
			return -RET_ERROR;
		}
		pr_info("CAM_START dev_port %d start_cnt %d \n",
			sensor_info->dev_port, start_cnt);
		if (start_cnt == 0) {
			ret = ((sensor_module_t *)(sensor_info->sensor_ops))->start(sensor_info);
			if (ret < 0) {
				ioctl(sensor_info->sen_devfd, SENSOR_USER_UNLOCK, 0);
				return -RET_ERROR;
			}
		}
		ret = camera_start_stop_get_unlock(sensor_info);
		if (ret < 0) {
			pr_err("CAM_START camera_start_stop_get_unlock fail\n");
			return -RET_ERROR;
		}
	}
	if(cam_ctl == CAM_STOP) {
		if(NULL == sensor_info->sensor_ops || NULL == ((sensor_module_t *)(sensor_info->sensor_ops))->stop) {
			return -RET_ERROR;
		}
		ret = camera_start_stop_lock(sensor_info, &start_cnt);
		if (ret < 0) {
			pr_err("CAM_STOP camera_start_stop_lock fail\n");
			return -RET_ERROR;
		}
		pr_info("camera_start_stop_lock start_cnt %d dev_port %d\n",
			start_cnt, sensor_info->dev_port);
		ae_share_flag = 0;
		if(start_cnt == 1) {
			ret = ((sensor_module_t *)(sensor_info->sensor_ops))->stop(sensor_info);
			if (ret < 0) {
				ioctl(sensor_info->sen_devfd, SENSOR_USER_UNLOCK, 0);
				return -RET_ERROR;
			}
		}
		ret = camera_start_stop_put_unlock(sensor_info);
		if (ret < 0) {
			pr_err("CAM_STOP camera_start_stop_put_unlock fail\n");
			return -RET_ERROR;
		}
	}
	if(cam_ctl == CAM_DEINIT) {
		if(NULL == sensor_info->sensor_ops ||
			NULL == ((sensor_module_t *)(sensor_info->sensor_ops))->deinit) {
			pr_err("deinit ops is null port %d\n", sensor_info->dev_port);
		}
		camera_init_deinit_lock(sensor_info, &init_cnt);
		if(init_cnt == 1) {
			ret = ((sensor_module_t *)(sensor_info->sensor_ops))->deinit(sensor_info);
			// stop pthred
			if (NULL != ((sensor_module_t *)(sensor_info->sensor_ops))->userspace_control) {
				ret = ((sensor_module_t *)(sensor_info->sensor_ops))->userspace_control(port, &userspace_enable);
				if ((ret == 0) && (userspace_enable)) {
					sensor_ctrl_stop_i2cbus(sensor_info->dev_port);
				}
			}
		}
		camera_init_deinit_put_unlock(sensor_info);
		camera_res_release(sensor_info);
	}
	return ret;
err:
	camera_destory_mutex_package(sensor_info->mp);
	sensor_info->mp = NULL;
close_senfd:
	if (sensor_info->sen_devfd >= 0) {
		close(sensor_info->sen_devfd);
		sensor_info->sen_devfd = -1;
	}
	hb_vin_close(sensor_info->entry_num);
close_gpiofd:
	if(sensor_info->power_mode) {
		for(i = 0; i < sensor_info->gpio_num; i++) {
			if(gpio_fd[sensor_info->gpio_pin[i]] >= 0) {
				close(gpio_fd[sensor_info->gpio_pin[i]]);
				gpio_fd[sensor_info->gpio_pin[i]] = -1;
			}
		}
	}
close_i2cfd:
	hb_i2c_deinit(sensor_info->bus_num);
	if(cam_spi_fd[sensor_info->bus_num] > 0) {
		close(cam_spi_fd[sensor_info->bus_num]);
		cam_spi_fd[sensor_info->bus_num] = -1;
	}
	return -RET_ERROR;
}

int32_t hb_cam_utility(int32_t cam_ctl, sensor_info_t *sensor_info)
{
	int ret = RET_OK, i;
	uint32_t userspace_enable = 0;
	uint32_t port = sensor_info->port;
	char str[12] = {0};

	if(cam_ctl == CAM_INIT) {
		if(sensor_info->bus_type == I2C_BUS) {
			hb_i2c_init(sensor_info->bus_num);
		} else if (sensor_info->bus_type == SPI_BUS) {
			cam_spi_init(sensor_info);
		}
		maddr = (uint8_t)sensor_info->sensor_addr;
		snprintf(str, sizeof(str), "/dev/port_%d", 0);
		if(sensor_info->power_mode) {
			for(i = 0; i < sensor_info->gpio_num; i++) {
				gpio_fd[sensor_info->gpio_pin[i]] = open(str, O_RDWR | O_CLOEXEC);
				if(gpio_fd[sensor_info->gpio_pin[i]] < 0)
					goto close_i2cfd;
			}
		}
		ret = hb_sensor_init_process(sensor_info);
		if (ret < 0) {
			pr_err("hb_sensor_init_process fail\n");
			goto close_gpiofd;
		}
	}
	if(cam_ctl == CAM_START) {
		if(NULL == sensor_info->sensor_ops || NULL == ((sensor_module_t *)(sensor_info->sensor_ops))->start) {
			return -RET_ERROR;
		}
		ret = ((sensor_module_t *)(sensor_info->sensor_ops))->start(sensor_info);
		if (ret < 0) {
			pr_err("sensor_start fail port %d\n", sensor_info->port);
			return -RET_ERROR;
		}
	}
	if(cam_ctl == CAM_STOP) {
		if(NULL == sensor_info->sensor_ops || NULL == ((sensor_module_t *)(sensor_info->sensor_ops))->stop) {
			return -RET_ERROR;
		}
		ae_share_flag = 0;
		ret = ((sensor_module_t *)(sensor_info->sensor_ops))->stop(sensor_info);
		if (ret < 0) {
			pr_err("sensor_stop fail port %d\n", sensor_info->port);
			return -RET_ERROR;
		}
	}
	if(cam_ctl == CAM_DEINIT) {
		if(NULL == sensor_info->sensor_ops || NULL == ((sensor_module_t *)(sensor_info->sensor_ops))->deinit) {
			pr_err("deinit ops is null port %d\n", sensor_info->dev_port);
		}
		ret = ((sensor_module_t *)(sensor_info->sensor_ops))->deinit(sensor_info);
		// stop pthred
		if (NULL != ((sensor_module_t *)(sensor_info->sensor_ops))->userspace_control) {
			ret = ((sensor_module_t *)(sensor_info->sensor_ops))->userspace_control(port, &userspace_enable);
			if ((ret == 0) && (userspace_enable)) {
				sensor_ctrl_stop_i2cbus(sensor_info->dev_port);
			}
		}
	}
	return ret;
close_gpiofd:
	if(sensor_info->power_mode) {
		for(i = 0; i < sensor_info->gpio_num; i++) {
			if(gpio_fd[sensor_info->gpio_pin[i]] >= 0) {
				close(gpio_fd[sensor_info->gpio_pin[i]]);
				gpio_fd[sensor_info->gpio_pin[i]] = -1;
			}
		}
	}
close_i2cfd:
	hb_i2c_deinit(sensor_info->bus_num);
	if(cam_spi_fd[sensor_info->bus_num] > 0) {
		close(cam_spi_fd[sensor_info->bus_num]);
		cam_spi_fd[sensor_info->bus_num] = -1;
	}
	return -RET_ERROR;
}

int hb_cam_ae_share_init(uint32_t port, uint32_t flag)
{
	int ret = 0;
	uint32_t src_port = 0;

	ae_share_flag = flag;
	src_port = (ae_share_flag >> 16) & 0xff;

	if (port < CAM_MAX_NUM && snsfds[port] > 0 && src_port - 0xA0 == port) {
		ret = ioctl(snsfds[port], SENSOR_AE_SHARE, &ae_share_flag);
		if (ret < 0) {
			pr_err("%s port_%d ioctl fail %d\n", __func__, port, ret);
			return ret;
		}
	}
	return ret;
}

int hb_deserial_deinit(deserial_info_t *deserial_info)
{
	if(deserial_info->deserial_fd) {
	   dlclose(deserial_info->deserial_fd);
	   deserial_info->deserial_fd = NULL;
	   deserial_info->deserial_ops = NULL;
	}
	if(deserial_info->serdes_fd > 0) {
		close(deserial_info->serdes_fd);
		deserial_info->serdes_fd = -1;
	}
	hb_i2c_deinit(deserial_info->bus_num);
	return 0;
}

int hb_deserial_start_physical(deserial_info_t *deserial_info)
{
	int ret = RET_OK;

	ret = ((deserial_module_t *)(deserial_info->deserial_ops))->start_physical(deserial_info);
	if (ret < 0) {
		pr_err("hb_deserial_start_physical error\n");
		return -HB_CAM_START_PHYSICAL_FAIL;
	}

	return ret;
}
int camera_init_utility(sensor_info_t *sensor_info)
{
	int ret = 0;
	int init_cnt = 0;

	ret = camera_open_sensorfd(sensor_info);
	if (ret < 0) {
		return ret;
	}
	ret = hb_camera_sensorlib_init(sensor_info);
	if(ret < 0) {
		pr_err("hb_camera_sensorlib_init error! CAM_INIT\n");
		ret = ioctl(sensor_info->sen_devfd, SENSOR_USER_UNLOCK, 0);
		if (ret < 0) {
			pr_err("sen_devfd %d ioctl SENSOR_USER_UNLOCK fail(%s)\n",
					sensor_info->sen_devfd, strerror(errno));
		}
		goto err;
	}
	ret = camera_create_share_mutex(sensor_info);
	if (ret < 0) {
		pr_err("camera_create_share_mutex fail\n");
		goto err1;
	}
	/*lock */
	ret = camera_init_deinit_lock(sensor_info, &init_cnt);
	if (ret < 0) {
		pr_err("camera_init_deinit_lock fail init_cnt %d port %d\n",
			init_cnt, sensor_info->port);
		goto err2;
	}
	pr_info("sensor_info dev_port %d init_cnt %d\n",
			sensor_info->dev_port, init_cnt);
	if(init_cnt == 0) {
		ret = hb_cam_utility(CAM_INIT, sensor_info);
		if(ret < 0) {
			pr_err("hb_cam_utility error! CAM_INIT\n");
			ret = ioctl(sensor_info->sen_devfd, SENSOR_USER_UNLOCK, 0);
			if (ret < 0) {
				pr_err("sen_devfd %d ioctl SENSOR_USER_UNLOCK fail(%s)",
					sensor_info->sen_devfd, strerror(errno));
			}
			goto err1;
		}
	}
	ret = hb_vin_init(sensor_info->entry_num);
	if(ret < 0) {
		pr_err("hb_vin_init error! ret %d entry_num %d\n",
			ret, sensor_info->entry_num);
		ret = ioctl(sensor_info->sen_devfd, SENSOR_USER_UNLOCK, 0);
		if (ret < 0) {
			pr_err("sen_devfd %d ioctl SENSOR_USER_UNLOCK fail(%s)",
				sensor_info->sen_devfd, strerror(errno));
		}
		goto err1;
	}
	/*unlock*/
	ret = camera_init_deinit_get_unlock(sensor_info);
	if (ret < 0) {
		pr_err("camera_init_deinit_get_unlock fail\n");
		goto err2;
	}
	return ret;

err2:
	camera_destory_mutex_package(sensor_info->mp);
	sensor_info->mp = NULL;
err1:
	if(sensor_info->sensor_fd) {
		dlclose(sensor_info->sensor_fd);
		sensor_info->sensor_fd = NULL;
	}
err:
	if (sensor_info->sen_devfd > 0) {
		close(sensor_info->sen_devfd);
		sensor_info->sen_devfd = -1;
	}
	hb_vin_close(sensor_info->entry_num);
	return -RET_ERROR;
}

void camera_deinit_utility(sensor_info_t *sensor_info)
{
	int init_cnt;

	/*lock */
	camera_init_deinit_lock(sensor_info, &init_cnt);
	pr_info("camera_deinit dev_port %d init_cnt %d\n",
			sensor_info->dev_port, init_cnt);
	if(init_cnt == 1) {
		hb_vin_deinit(sensor_info->entry_num);
		hb_cam_utility(CAM_DEINIT, sensor_info);
	}
	/*unlock*/
	camera_init_deinit_put_unlock(sensor_info);
	camera_res_release(sensor_info);
}

int camera_start_utility(sensor_info_t *sensor_info)
{
	int ret = 0;
	int start_cnt;

	ret = camera_start_stop_lock(sensor_info, &start_cnt);
	if (ret < 0) {
		pr_err("camera_start_stop_lock fail port %d start_cnt %d\n",
				sensor_info->port, start_cnt);
		return -RET_ERROR;
	}
	pr_info("camera_start_lock dev_port %d start_cnt %d \n",
		sensor_info->dev_port, start_cnt);
	if (start_cnt == 0) {
		ret = hb_cam_utility(CAM_START, sensor_info);
		if(ret < 0) {
			pr_err("!!!hb_cam_utility CAM_START error %d\n", ret);
			ret = ioctl(sensor_info->sen_devfd, SENSOR_USER_UNLOCK, 0);
			if (ret < 0) {
				pr_err("sen_devfd %d ioctl SENSOR_USER_UNLOCK fail(%s)",
					sensor_info->sen_devfd, strerror(errno));
			}
			return -RET_ERROR;
		}
	}
	ret = hb_vin_start(sensor_info->entry_num);
	if(ret < 0) {
		pr_err("!!!hb_vin_start fail %d\n", ret);
		ret = ioctl(sensor_info->sen_devfd, SENSOR_USER_UNLOCK, 0);
		if (ret < 0) {
			pr_err("sen_devfd %d ioctl SENSOR_USER_UNLOCK fail(%s)",
				sensor_info->sen_devfd, strerror(errno));
		}
		return -RET_ERROR;
	}
	ret = camera_start_stop_get_unlock(sensor_info);
	if (ret < 0) {
		pr_err("camera_start_stop_get_unlock fail\n");
	}
	return ret;
}

int camera_stop_utility(sensor_info_t *sensor_info)
{
	int ret = 0;
	int start_cnt;

	ret = camera_start_stop_lock(sensor_info, &start_cnt);
	if (ret < 0) {
		pr_err("camera_start_stop_lock fail port %d start_cnt %d\n",
			sensor_info->port, start_cnt);
		return -RET_ERROR;
	}
	pr_info("camera_start_stop_lock start_cnt %d dev_port %d\n",
		start_cnt, sensor_info->dev_port);
	if(start_cnt == 1) {
		ret = hb_cam_utility(CAM_STOP, sensor_info);
		if(ret < 0) {
			pr_err("!!!hb_cam_utility cam_stop ret %d port %d entry_num %d\n",
					ret, sensor_info->port, sensor_info->entry_num);
			ret = ioctl(sensor_info->sen_devfd, SENSOR_USER_UNLOCK, 0);
			if (ret < 0) {
				pr_err("sen_devfd %d ioctl SENSOR_USER_UNLOCK fail(%s)",
					sensor_info->sen_devfd, strerror(errno));
			}
			return -RET_ERROR;
		}
	}
	ret = hb_vin_stop(sensor_info->entry_num);
	if(ret < 0) {
		pr_err("!!!hb_vin_stop %d err port %d entry_num %d\n",
			ret, sensor_info->port, sensor_info->entry_num);
		ret = ioctl(sensor_info->sen_devfd, SENSOR_USER_UNLOCK, 0);
		if (ret < 0) {
			pr_err("sen_devfd %d ioctl SENSOR_USER_UNLOCK fail(%s)",
				sensor_info->sen_devfd, strerror(errno));
		}
		return -RET_ERROR;
	}
	ret = camera_start_stop_put_unlock(sensor_info);
	if (ret < 0) {
		pr_err("camera_start_stop_put_unlock fail port %d\n", sensor_info->port);
	}
	return ret;
}

// -- ctrl sensor in user-space
typedef struct sensor_ctrl_model_s {
	uint32_t port;
	pthread_t dev_ptr;
	int dev_fd;
	uint8_t ctrl_runing;
	void *sensor_ops;
	// pthread_mutex_t mutex;
	uint32_t func_flag;
	hal_control_info_t info;
} sensor_ctrl_model_t;

sensor_ctrl_model_t model[8];

typedef struct sensor_ctrl_info_s {
	uint32_t port;
	uint32_t gain_num;
	uint32_t gain_buf[4];
	uint32_t dgain_num;
	uint32_t dgain_buf[4];
	uint32_t en_dgain;
	uint32_t line_num;
	uint32_t line_buf[4];
	uint32_t rgain;
	uint32_t bgain;
	uint32_t grgain;
	uint32_t gbgain;
	uint32_t af_pos;
	uint32_t zoom_pos;
	uint32_t mode;
} sensor_ctrl_info_t;

#define CAMERA_CTRL_IOC_MAGIC	'x'
#define SENSOR_CTRL_INFO_SYNC	_IOWR(CAMERA_CTRL_IOC_MAGIC, 20, sensor_ctrl_info_t)

sensor_ctrl_info_t ctrl_data[8];

int sensor_ctrl_init(uint32_t port)
{
	if ((model[port].dev_fd = open("/dev/sensor_ctrl", O_RDWR | O_CLOEXEC)) < 0) {
		pr_err("sensor_ctrl port %d open fail, err %s \n", port, strerror(errno));
		return -RET_ERROR;
	}
	return 0;
}

void sensor_ctrl_deinit(uint32_t port)
{
	close(model[port].dev_fd);
	model[port].dev_fd = 0;
}

void *sensor_ctrl_pthread(void *input)
{
	int ret = 0;
	uint32_t i = 0;
	uint32_t j = 0;
	char send[4] = {0};
	struct timespec time;

	sensor_ctrl_model_t *arg = (sensor_ctrl_model_t *)(input);

	pr_info(" start fe pthread %d\n", arg->port);
	ret = sensor_ctrl_init(arg->port);
	if (ret) {
		model[arg->port].ctrl_runing = 0;
		pr_err(" start fe pthread %d failed!\n", arg->port);
	} else {
		ctrl_data[arg->port].port = arg->port;
		while (arg->ctrl_runing) {
			// clock_gettime(CLOCK_REALTIME, &time);
			// pr_err("time---%d %d\n", time.tv_sec, time.tv_nsec);
			ret = ioctl(arg->dev_fd, SENSOR_CTRL_INFO_SYNC, &ctrl_data[arg->port]);
			if ((!ret) && (arg->sensor_ops)) {
				// write gain
				if ((NULL != ((sensor_module_t *)(arg->sensor_ops))->aexp_gain_control) && (arg->func_flag & HAL_GAIN_CONTROL)) {
					((sensor_module_t *)(arg->sensor_ops))->aexp_gain_control(
						&arg->info, ctrl_data[arg->port].mode,
						ctrl_data[arg->port].gain_buf, ctrl_data[arg->port].dgain_buf, ctrl_data[arg->port].gain_num);
				}
				// write line
				if ((NULL != ((sensor_module_t *)(arg->sensor_ops))->aexp_line_control) && (arg->func_flag & HAL_LINE_CONTROL)) {
					((sensor_module_t *)(arg->sensor_ops))->aexp_line_control(
						&arg->info, ctrl_data[arg->port].mode,
						ctrl_data[arg->port].line_buf, ctrl_data[arg->port].line_num);
				}
				// write awb
				if ((NULL != ((sensor_module_t *)(arg->sensor_ops))->awb_control) && (arg->func_flag & HAL_AWB_CONTROL)) {
					((sensor_module_t *)(arg->sensor_ops))->awb_control(
						&arg->info, ctrl_data[arg->port].mode,
						ctrl_data[arg->port].rgain, ctrl_data[arg->port].bgain,
						ctrl_data[arg->port].grgain, ctrl_data[arg->port].gbgain);
				}
				// af control
				if ((NULL != ((sensor_module_t *)(arg->sensor_ops))->af_control) && (arg->func_flag & HAL_AF_CONTROL)) {
					((sensor_module_t *)(arg->sensor_ops))->af_control(
						&arg->info, ctrl_data[arg->port].mode,
						ctrl_data[arg->port].af_pos);
				}
				// zoom control
				if ((NULL != ((sensor_module_t *)(arg->sensor_ops))->zoom_control) && (arg->func_flag & HAL_ZOOM_CONTROL)) {
					((sensor_module_t *)(arg->sensor_ops))->zoom_control(
						&arg->info, ctrl_data[arg->port].mode,
						ctrl_data[arg->port].zoom_pos);
				}
			}
		}
	}

	sensor_ctrl_deinit(arg->port);
	return NULL;
}

int sensor_ctrl_start_i2cbus(uint32_t port, uint32_t ops_flag, void *ops)
{
	int ret = 0;

	if (port >= 8 || !ops) {
		pr_err("port %d or ops %p is err!\n", port, ops);
		return -1;
	}

	if (model[port].dev_ptr) {
		pr_info("port is init success!\n");
		return 0;
	}
	sensor_info_t *info = (sensor_info_t *)ops;

	// init info
	memset(&model[port].info, 0, sizeof(hal_control_info_t));
	model[port].func_flag = ops_flag;
	model[port].port = port;
	model[port].info.port = port;
	model[port].info.sensor_mode = info->sensor_mode;
	model[port].info.bus_type = info->bus_type;
	model[port].info.bus_num = info->bus_num;
	model[port].info.sensor_addr = info->sensor_addr;
	model[port].info.sensor1_addr = info->sensor1_addr;
	model[port].info.serial_addr = info->serial_addr;
	model[port].info.serial_addr1 = info->serial_addr1;
	model[port].info.eeprom_addr = info->eeprom_addr;
	memcpy(&model[port].info.sensor_spi_info, &info->spi_info, sizeof(spi_data_t));
	// init info end
	model[port].ctrl_runing = 1;
	ret = pthread_create(&model[port].dev_ptr, NULL, sensor_ctrl_pthread, (void *)(&model[port]));
	if(ret != 0){
		model[port].ctrl_runing = 0;
		model[port].dev_ptr = 0;
		pr_err("can't create thread: %s\n",strerror(ret));
		return -1;
	} else {
		pr_info("start ptf %ld \n", model[port].dev_ptr);
		model[port].sensor_ops = info->sensor_ops;
	}

	return ret;
}

void sensor_ctrl_stop_i2cbus(uint32_t port)
{
	model[port].ctrl_runing = 0;
	if (model[port].dev_ptr) {
		pr_info("start stop ptf %d \n", (uint32_t)model[port].dev_ptr);
		pthread_join(model[port].dev_ptr, NULL);
	}
	memset(&model[port], 0, sizeof(sensor_ctrl_model_t));
	pr_info(" stop fe pthread %d\n", port);
}

