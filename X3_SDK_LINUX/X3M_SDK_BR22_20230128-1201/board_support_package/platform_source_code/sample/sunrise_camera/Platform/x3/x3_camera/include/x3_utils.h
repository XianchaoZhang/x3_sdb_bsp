/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef X3_UTILS_H_
#define X3_UTILS_H_

typedef enum {
	E_CHIP_X3M,
	E_CHIP_X3E,
	E_CHIP_UNKNOW,
} E_CHIP_TYPE;

E_CHIP_TYPE x3_get_chip_type(void);

#define SENSOR_F37_SUPPORT			1
#define SENSOR_IMX415_SUPPORT		2
#define SENSOR_OS8A10_SUPPORT		4
#define SENSOR_OV8856_SUPPORT		8
#define SENSOR_SC031GS_SUPPORT		16
#define SENSOR_GC4663_SUPPORT		32
#define SENSOR_IMX415_BV_SUPPORT		64

typedef struct {
	E_CHIP_TYPE m_chip_type; // 芯片类型
	unsigned int m_sensor_list; // 每个bit对应一种sensor
}hard_capability_t; // 硬件能力

int x3_get_hard_capability(hard_capability_t *capability);

#endif // X3_UTILS_H_