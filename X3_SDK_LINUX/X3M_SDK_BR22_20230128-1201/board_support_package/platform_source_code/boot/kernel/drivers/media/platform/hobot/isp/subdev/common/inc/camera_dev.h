/*    Copyright (C) 2018 Horizon Inc.
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 */

#ifndef DRIVERS_MEDIA_PLATFORM_HOBOT_ISP_SUBDEV_COMMON_INC_CAMERA_DEV_H_
#define DRIVERS_MEDIA_PLATFORM_HOBOT_ISP_SUBDEV_COMMON_INC_CAMERA_DEV_H_

#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/spi/spi.h>

#include "inc/camera_subdev.h"
#define CHAR_DEVNAME_LEN  20
#define I2C_BUS  0
#define SPI_BUS  1

#define GPIO_MAX_NUM	256 /*max gpio number*/

#define CAMERA_IOC_MAGIC    'x'
#define SENSOR_TURNING_PARAM  _IOW(CAMERA_IOC_MAGIC, 0, sensor_turning_data_t)
#define SENSOR_OPEN_CNT       _IOR(CAMERA_IOC_MAGIC, 1, int)
#define SENSOR_SET_START_CNT  _IOW(CAMERA_IOC_MAGIC, 2, int)
#define SENSOR_GET_START_CNT  _IOR(CAMERA_IOC_MAGIC, 3, int)
#define SENSOR_USER_LOCK      _IOW(CAMERA_IOC_MAGIC, 4, int)
#define SENSOR_USER_UNLOCK    _IOW(CAMERA_IOC_MAGIC, 5, int)
#define SENSOR_AE_SHARE	      _IOW(CAMERA_IOC_MAGIC, 6, int)
#define SENSOR_TURNING_PARAM_EX   _IOW(CAMERA_IOC_MAGIC, 7, sensor_turning_data_ex_t)
#define SENSOR_SET_INIT_CNT  _IOW(CAMERA_IOC_MAGIC, 8, int)
#define SENSOR_GET_INIT_CNT  _IOR(CAMERA_IOC_MAGIC, 9, int)
#define SENSOR_GPIO_CONTROL  _IOW(CAMERA_IOC_MAGIC, 10, gpio_info_t)

#define CAMERA_REG_PARAM      _IOW(CAMERA_IOC_MAGIC, 11, camera_state_register_t)

typedef enum _mipi_pre_state_t {
	SENSOR_PRE_STATE_LOCK = 0,
	SENSOR_PRE_STATE_UNLOCK,
} sensor_pre_state_t;

struct sensor_ctrl_ops {
	char ctrl_name[20];
	void (*camera_gain_control)(uint32_t port, sensor_priv_t *priv_param,
		uint32_t *a_gain, uint32_t *d_gain, uint32_t *a_line);
	void (*camera_line_control)(uint32_t port, sensor_priv_t *priv_param,
		uint32_t *a_line);
	void (*camera_alloc_again)(uint32_t port, uint32_t *a_gain);
	void (*camera_alloc_dgain)(uint32_t port, uint32_t *a_gain);
};

typedef struct register_info {
	uint8_t reg_width;
	uint8_t value_width;
	uint16_t *register_table;
	uint32_t register_table_size;
}register_info_t;

typedef struct camera_state_register_info {
	uint8_t  bus_num;
	uint8_t  mipi_index;
	uint8_t  deserial_addr;
	uint8_t  serial_addr;
	uint8_t  sensor_addr;
	register_info_t deserial_register_info;
	register_info_t serial_register_info;
	register_info_t sensor_register_info;
}camera_state_register_t;

int camera_register_status_print(int port);

typedef struct _camera_charmod_s {
	char name[CHAR_DEVNAME_LEN];
	uint32_t devflag;
	struct mutex slock;
	uint32_t user_num;
	int dev_minor_id;
	struct miscdevice camera_chardev;
	uint32_t port;
	struct i2c_client *client;
    struct i2c_board_info board_info;
	struct spi_device *spidev;
	struct spi_board_info spi_board;
    sensor_turning_data_t camera_param;
    sensor_turning_data_ex_t camera_param_ex;
    camera_state_register_t camera_state_register_info;
	int camera_check_flag;
	struct file *mst_file;
	struct mutex user_mutex;
	uint32_t start_num;
	uint32_t init_num;
	uint32_t ae_share_flag;
	// wait queue
	uint32_t pre_state;
	bool pre_done;
	wait_queue_head_t pre_wq;
	uint32_t write_flag;
	DECLARE_BITMAP(gpio_req_mask, GPIO_MAX_NUM);
} camera_charmod_s;

extern camera_charmod_s *camera_mod[CAMERA_TOTAL_NUMBER];

int camera_gpio_info_config(camera_charmod_s * camera_cdev,
		gpio_info_t *gpio_info);
void camera_gpio_all_free(camera_charmod_s * camera_cdev);

int camera_state_register_set(uint32_t port,
		camera_state_register_t *camera_register_data);
int camera_cdev_init(void);
void camera_cdev_exit(void);
#endif // DRIVERS_MEDIA_PLATFORM_HOBOT_ISP_SUBDEV_COMMON_INC_CAMERA_DEV_H_
