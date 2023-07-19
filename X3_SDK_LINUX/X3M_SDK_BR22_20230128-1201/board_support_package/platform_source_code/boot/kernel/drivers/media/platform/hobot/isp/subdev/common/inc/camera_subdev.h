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

#ifndef DRIVERS_MEDIA_PLATFORM_HOBOT_ISP_SUBDEV_COMMON_INC_CAMERA_SUBDEV_H_
#define DRIVERS_MEDIA_PLATFORM_HOBOT_ISP_SUBDEV_COMMON_INC_CAMERA_SUBDEV_H_
#define CAMERA_TOTAL_NUMBER  8
#define V4L2_CAMERA_NAME "camera"
#define CAMERA_SENSOR_NAME  20

#define GPIO_HIGH	1
#define GPIO_LOW	0
#define GPIO_NAME_LENGTH 20

#include <linux/list.h>
#include <linux/workqueue.h>

typedef struct sensor_priv {
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
	  uint8_t  mode;
}sensor_priv_t;

typedef struct sensor_data {
	uint32_t  turning_type;
	uint32_t  step_gain;
	uint32_t  again_prec;
	uint32_t  dgain_prec;
	uint32_t  conversion;
	uint32_t  VMAX;
	uint32_t  HMAX;
	uint32_t  FSC_DOL2;
	uint32_t  FSC_DOL3;
	uint32_t  RHS1;
	uint32_t  RHS2;
	uint32_t  lane;
	uint32_t  clk;
	uint32_t  fps;
	uint32_t  gain_max;
	uint32_t  lines_per_second;
	uint32_t  analog_gain_max;
	uint32_t  digital_gain_max;
	uint32_t  exposure_time_max;
	uint32_t  exposure_time_min;
	uint32_t  exposure_time_long_max;
	uint32_t  active_width;
	uint32_t  active_height;
}sensor_data_t;

struct sensor_arg {
	 uint32_t port;
	 sensor_priv_t *sensor_priv;
	 sensor_data_t *sensor_data;
	 uint32_t *a_gain;
	 uint32_t *d_gain;
	 uint32_t  address;
	 uint32_t  w_data;
	 uint32_t  *r_data;
	 uint32_t *integration_time;
};

typedef struct ctrlp_s {
        int32_t ratio;
        uint32_t offset;
        uint32_t max;
} ctrlp_t;

typedef struct dol3_s {
	uint32_t param_hold;
	uint32_t param_hold_length;
	ctrlp_t  line_p[3];
	uint32_t s_line;
	uint32_t s_line_length;
	uint32_t m_line;
	uint32_t m_line_length;
	uint32_t l_line;
	uint32_t l_line_length;
	uint32_t again_control_num;
	uint32_t again_control[4];
	uint32_t again_control_length[4];
	uint32_t dgain_control_num;
	uint32_t dgain_control[4];
	uint32_t dgain_control_length[4];
	uint32_t *again_lut;
	uint32_t *dgain_lut;
} dol3_t;

typedef struct dol2_s {
	uint32_t param_hold;
	uint32_t param_hold_length;
	ctrlp_t  line_p[2];
	uint32_t s_line;
	uint32_t s_line_length;
	uint32_t m_line;
	uint32_t m_line_length;
	uint32_t again_control_num;
	uint32_t again_control[4];
	uint32_t again_control_length[4];
	uint32_t dgain_control_num;
	uint32_t dgain_control[4];
	uint32_t dgain_control_length[4];
	uint32_t *again_lut;
	uint32_t *dgain_lut;
}dol2_t;

typedef struct normal_s {
	uint32_t param_hold;
	uint32_t param_hold_length;
	ctrlp_t  line_p;
	uint32_t s_line;
	uint32_t s_line_length;
	uint32_t again_control_num;
	uint32_t again_control[4];
	uint32_t again_control_length[4];
	uint32_t dgain_control_num;
	uint32_t dgain_control[4];
	uint32_t dgain_control_length[4];
	uint32_t *again_lut;
	uint32_t *dgain_lut;
}normal_t;

typedef struct pwl_s {
	uint32_t param_hold;
	uint32_t param_hold_length;
	ctrlp_t  line_p;
	uint32_t line;
	uint32_t line_length;
	uint32_t again_control_num;
	uint32_t again_control[4];
	uint32_t again_control_length[4];
	uint32_t dgain_control_num;
	uint32_t dgain_control[4];
	uint32_t dgain_control_length[4];
	uint32_t *again_lut;
	uint32_t *dgain_lut;
}pwl_t;

typedef struct stream_ctrl_s {
	uint32_t stream_on[10];
	uint32_t stream_off[10];
	uint32_t data_length;
}stream_ctrl_t;

typedef struct sensor_awb_ctrl_s {
	uint32_t rgain_addr[4];
	uint32_t rgain_length[4];
	uint32_t bgain_addr[4];
	uint32_t bgain_length[4];
	uint32_t grgain_addr[4];
	uint32_t grgain_length[4];
	uint32_t gbgain_addr[4];
	uint32_t gbgain_length[4];
	uint32_t rb_prec;
} sensor_awb_ctrl_t;

typedef struct sensor_turning_data {
	uint32_t  port;
	char      sensor_name[CAMERA_SENSOR_NAME];
	uint32_t  sensor_addr;
    uint32_t  bus_num;
	uint32_t  bus_type;
	uint32_t  reg_width;
	uint32_t  chip_id;
	uint32_t  mode;
	uint32_t  cs;
	uint32_t  spi_mode;
	uint32_t  spi_speed;
	normal_t normal;
	dol2_t   dol2;
	dol3_t   dol3;
	pwl_t    pwl;  // ar0233
	sensor_awb_ctrl_t sensor_awb;
	stream_ctrl_t stream_ctrl;
	sensor_data_t sensor_data;
}sensor_turning_data_t;

typedef struct sensor_turning_data_ex {
	uint32_t  ratio_en;
	uint32_t  ratio_value;
	uint32_t  l_line;
	uint32_t  l_line_length;
	uint32_t lexposure_time_min;
}sensor_turning_data_ex_t;

enum camera_IOCTL {
    SENSOR_UPDATE = 0,
    SENSOR_WRITE,
    SENSOR_READ,
    SENSOR_GET_PARAM,
    SENSOR_STREAM_ON,
    SENSOR_STREAM_OFF,
	SENSOR_ALLOC_ANALOG_GAIN,
	SENSOR_ALLOC_DIGITAL_GAIN,
	SENSOR_ALLOC_INTEGRATION_TIME,
	SENSOR_AWB_UPDATE,
};

typedef struct {
	struct list_head list_free;
	struct list_head list_busy;
	spinlock_t lock;
	struct work_struct updata_work;
} event_header_t;

typedef struct {
	struct list_head  list_node;
	uint32_t port;
	uint32_t cmd;
	sensor_priv_t priv_param;
} event_node_t;

typedef struct camera_gpio_info_t {
	uint32_t gpio;
	uint32_t gpio_level;
} gpio_info_t;

#endif // DRIVERS_MEDIA_PLATFORM_HOBOT_ISP_SUBDEV_COMMON_INC_CAMERA_SUBDEV_H_

