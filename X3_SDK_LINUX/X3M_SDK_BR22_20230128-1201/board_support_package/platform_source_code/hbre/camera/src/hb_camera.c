/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2016 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/msg.h>
#include <sys/time.h>
#include <ctype.h>
#include <logging.h>


#ifndef NODIAG
#include "./diag_lib.h"
#endif
#include "cJSON.h"
#include "inc/hb_cam_interface.h"
#include "inc/cam_common.h"
#include "utility/hb_cam_utility.h"
#include "utility/hb_i2c.h"
#include "inc/hb_vin_mipi_dev.h"
#include "inc/hb_vin_mipi_host.h"
#include "inc/hb_vin.h"
#include "utility/hb_vcam.h"
#include "../utility/hb_pwm.h"

#define CameraDiagModule 0x9003
#define CameraEventId 0x0001
static board_info_t g_board_cfg;
static char cam_config_file[MAX_NUM_LENGTH];
static cJSON *root = NULL;
static cJSON *board_info = NULL;

// PRQA S ALL ++

int hb_cam_diag(int ret)
{
#ifndef NODIAG
	if(ret >= 0) {
		if(diag_send_event_stat_and_env_data(DIAG_MSG_PRIO_MID,
							 CameraDiagModule,
							 CameraEventId,
							 DIAG_EVENT_SUCCESS,
							 DIAG_GEN_ENV_DATA_WHEN_SUCCESS,
							 NULL, 0) < 0) {
			pr_info("camera send success error\n");
		}
	}
	if(ret < 0) {
		if(diag_send_event_stat_and_env_data(DIAG_MSG_PRIO_MID,
							 CameraDiagModule,
							 CameraEventId,
							 DIAG_EVENT_FAIL,
							 DIAG_GEN_ENV_DATA_WHEN_ERROR,
							 (uint8_t *)&ret, 4) < 0) {
			pr_info("camera send fail error\n");
		}
	}
#endif
	return 0;
}

/*Convert a hex string to an integer*/
int hb_cam_htoi(char s[])
{
	int i;
	int n = 0;

	if (s[0] == '0' && (s[1] =='x' || s[1] =='X')) {
		i = 2;
	} else {
		i = 0;
	}
	for (; (s[i] >= '0' && s[i] <= '9') || (s[i] >= 'a' && s[i] <= 'z') || (s[i] >='A' && s[i] <= 'Z'); ++i) {
		if (tolower(s[i]) > '9') {
			n = 16 * n + (10 + tolower(s[i]) - (uint8_t)'a');
		} else {
			n = 16 * n + (tolower(s[i]) - (uint8_t)'0');
		}
	}
	return n;
}

void hb_cam_struct_data_init()
{
	int i;
	sensor_info_t *si = NULL;

	for (i = 0; i < g_board_cfg.port_number; i++) {
		si = &g_board_cfg.sensor_info[i];
		if (si->deserial_index >=0 && si->deserial_index < g_board_cfg.deserial_num &&
	      si->deserial_port >=0 && si->deserial_port < CAM_MAX_NUM) {
	      g_board_cfg.deserial_info[si->deserial_index].sensor_info[si->deserial_port] = si;
	      si->deserial_info = &g_board_cfg.deserial_info[si->deserial_index];
	     }
	}
	return;
}

int hb_cam_deserial_parse_cfg(uint32_t deserial_index)
{
	int ret = RET_OK;
	char tmp[28] = {0};
	cJSON *deserial_node = NULL, *node = NULL, *arrayitem = NULL;
	int array_size = 0, array_index;

	snprintf(tmp, sizeof(tmp), "deserial_%d", deserial_index);
	deserial_node = cJSON_GetObjectItem(board_info, tmp);
	node = cJSON_GetObjectItem(deserial_node, "bus_type");
	if(NULL != node) {
		g_board_cfg.deserial_info[deserial_index].bus_type = node->valueint;
		pr_info("deserial_index %d bus_type = %d\n", deserial_index, g_board_cfg.deserial_info[deserial_index].bus_type);
	}
	node = cJSON_GetObjectItem(deserial_node, "bus_num");
	if(NULL != node) {
		g_board_cfg.deserial_info[deserial_index].bus_num = node->valueint;
		pr_info("deserial_index %d bus_num = %d\n", deserial_index, g_board_cfg.deserial_info[deserial_index].bus_num);
	}
	node = cJSON_GetObjectItem(deserial_node, "deserial_addr");
	if(NULL != node) {
		int deserial_addr = hb_cam_htoi(node->valuestring);
		if (deserial_addr > 0xFF) {
			pr_err("invalid value! deserial_addr should be less then 0xFF\n");
			return -RET_ERROR;
		}
		g_board_cfg.deserial_info[deserial_index].deserial_addr = deserial_addr;
		pr_info("deserial_index %d serial_addr = 0x%x\n", deserial_index, g_board_cfg.deserial_info[deserial_index].deserial_addr);
	}
	node = cJSON_GetObjectItem(deserial_node, "deserial_name");
	if(NULL != node) {
		g_board_cfg.deserial_info[deserial_index].deserial_name = node->valuestring;
		pr_info("deserial_index %d deserial_name = %s\n", deserial_index, g_board_cfg.deserial_info[deserial_index].deserial_name);
	}
	node = cJSON_GetObjectItem(deserial_node, "physical_entry");
	if(NULL != node) {
		g_board_cfg.deserial_info[deserial_index].physical_entry = node->valueint;
		pr_info("deserial_index %d physical_entry = %d\n", deserial_index, g_board_cfg.deserial_info[deserial_index].physical_entry);
	}
	node = cJSON_GetObjectItem(deserial_node, "power_mode");
	if(NULL != node) {
		g_board_cfg.deserial_info[deserial_index].power_mode = node->valueint;
		pr_info("deserial_index %d power_mode = %d\n",
		deserial_index, g_board_cfg.deserial_info[deserial_index].power_mode);
	}
	node = cJSON_GetObjectItem(deserial_node, "power_delay");
	if(NULL != node) {
		g_board_cfg.deserial_info[deserial_index].power_delay = node->valueint;
		pr_info("deserial_index %d power_delay = %d\n",
		deserial_index, g_board_cfg.deserial_info[deserial_index].power_delay);
	}
	node = cJSON_GetObjectItem(deserial_node, "serdes_index");
	if(NULL != node) {
		g_board_cfg.deserial_info[deserial_index].serdes_index = node->valueint;
		pr_info("deserial_index %d serdes_index = %d\n",
		deserial_index, g_board_cfg.deserial_info[deserial_index].serdes_index);
	}
	node = cJSON_GetObjectItem(deserial_node, "gpio_pin");
	if(NULL != node) {
		array_size = cJSON_GetArraySize(node);
		if (array_size > GPIO_NUMBER) {
			pr_err("invalid value! gpio_pin num exceed the maximum %d\n", GPIO_NUMBER);
			return -RET_ERROR;
		}
		g_board_cfg.deserial_info[deserial_index].gpio_num = array_size;
		for(array_index = 0; array_index < array_size; array_index++) {
			arrayitem = cJSON_GetArrayItem(node, array_index);
			pr_debug("arrayitem->valueint %d \n", arrayitem->valueint);
			g_board_cfg.deserial_info[deserial_index].gpio_pin[array_index] = arrayitem->valueint;
			pr_info("array_index %d gpio_pin %d", array_index, g_board_cfg.deserial_info[deserial_index].gpio_pin[array_index]);
		}
	}
	node = cJSON_GetObjectItem(deserial_node, "gpio_level");
	if(NULL != node) {
		array_size = cJSON_GetArraySize(node);
		if (array_size > GPIO_NUMBER) {
			pr_err("invalid value! gpio_level num exceed the maximum %d\n", GPIO_NUMBER);
			return -RET_ERROR;
		}
		g_board_cfg.deserial_info[deserial_index].gpio_num = array_size;
		for(array_index = 0; array_index < array_size; array_index++) {
			arrayitem = cJSON_GetArrayItem(node, array_index);
			pr_debug("arrayitem->valueint %d\n", arrayitem->valueint);
			if ((arrayitem->valueint != 0) && (arrayitem->valueint != 1)) {
				pr_err("invalid value! gpio_level must be 0 or 1!\n");
				return -RET_ERROR;
			}
			g_board_cfg.deserial_info[deserial_index].gpio_level[array_index] = arrayitem->valueint;
			pr_info("deserial_index %d array_index %d gpio_level %d\n", deserial_index, array_index, g_board_cfg.deserial_info[deserial_index].gpio_level[array_index]);
		}
	}
	node = cJSON_GetObjectItem(deserial_node, "deserial_config_path");
	if(NULL != node) {
		g_board_cfg.deserial_info[deserial_index].deserial_config_path = node->valuestring;
		pr_info("deserial_index %d deserial_config_path = %s\n", deserial_index, g_board_cfg.deserial_info[deserial_index].deserial_config_path);
	}
	pr_debug("deserial_index %d board_name %s deserial_name %s\n", deserial_index, g_board_cfg.board_name, g_board_cfg.deserial_info[deserial_index].deserial_name);
	return ret;
}

int hb_cam_port_parse_cfg(uint32_t port, int fps, int resolution)
{
	int ret = RET_OK;
	char tmp[16] = {0};
	cJSON *port_node = NULL, *node = NULL, *arrayitem = NULL;
	int array_size = 0, array_index;

	g_board_cfg.sensor_info[port].port = port;
	snprintf(tmp, sizeof(tmp), "port_%d", port);
	port_node = cJSON_GetObjectItem(board_info, tmp);
	node = cJSON_GetObjectItem(port_node, "bus_type");
	if(NULL != node) {
		g_board_cfg.sensor_info[port].bus_type = node->valueint;
		pr_info("port_index %d bus_type = %d\n", port, g_board_cfg.sensor_info[port].bus_type);
	}
	node = cJSON_GetObjectItem(port_node, "bus_num");
	if(NULL != node) {
		g_board_cfg.sensor_info[port].bus_num = node->valueint;
		pr_info("port %d bus_num = %d\n", port, g_board_cfg.sensor_info[port].bus_num);
	}
	node = cJSON_GetObjectItem(port_node, "dev_port");
	if(NULL != node) {
		g_board_cfg.sensor_info[port].dev_port = node->valueint;
		pr_info("port %d dev_port = %d\n", port, g_board_cfg.sensor_info[port].dev_port);
	}
	node = cJSON_GetObjectItem(port_node, "isp_addr");
	if(NULL != node) {
		int isp_addr = hb_cam_htoi(node->valuestring);
		g_board_cfg.sensor_info[port].isp_addr = isp_addr;
		pr_info("port_index %d isp_addr = 0x%x\n", port, g_board_cfg.sensor_info[port].isp_addr);
	}
	node = cJSON_GetObjectItem(port_node, "sensor_addr");
	if(NULL != node) {
		int sensor_addr = hb_cam_htoi(node->valuestring);
		if (sensor_addr > 0xFF) {
			pr_err("invalid value! sensor_addr should be less then 0xFF\n");
			return -RET_ERROR;
		}
		g_board_cfg.sensor_info[port].sensor_addr = sensor_addr;
		pr_info("port_index %d sensor_addr = 0x%x\n", port, g_board_cfg.sensor_info[port].sensor_addr);
	}
	node = cJSON_GetObjectItem(port_node, "sensor1_addr");
	if(NULL != node) {
		int sensor1_addr = hb_cam_htoi(node->valuestring);
		g_board_cfg.sensor_info[port].sensor1_addr = sensor1_addr;
		pr_info("port_index %d sensor1_addr = 0x%x\n", port, g_board_cfg.sensor_info[port].sensor1_addr);
	}
	node = cJSON_GetObjectItem(port_node, "serial_addr");
	if(NULL != node) {
		int serial_addr = hb_cam_htoi(node->valuestring);
		if (serial_addr > 0xFF) {
			pr_err("invalid value! serial_addr should be less then 0xFF\n");
			return -RET_ERROR;
		}
		g_board_cfg.sensor_info[port].serial_addr = serial_addr;
		pr_info("port_index %d serial_addr = 0x%x\n", port, g_board_cfg.sensor_info[port].serial_addr);
	}
	node = cJSON_GetObjectItem(port_node, "serial_addr1");
	if(NULL != node) {
		int serial_addr1 = hb_cam_htoi(node->valuestring);
		g_board_cfg.sensor_info[port].serial_addr1 =  serial_addr1;
		pr_info("port_index %d serial_addr1 = 0x%x\n", port, g_board_cfg.sensor_info[port].serial_addr1);
	}
	node = cJSON_GetObjectItem(port_node, "imu_addr");
	if(NULL != node) {
		int imu_addr = hb_cam_htoi(node->valuestring);
		g_board_cfg.sensor_info[port].imu_addr =  imu_addr;
		pr_info("port_index %d imu_addr = 0x%x\n", port, g_board_cfg.sensor_info[port].imu_addr);
	}
	node = cJSON_GetObjectItem(port_node, "eeprom_addr");
	if(NULL != node) {
		int eeprom_addr = hb_cam_htoi(node->valuestring);
		g_board_cfg.sensor_info[port].eeprom_addr =  eeprom_addr;
		pr_info("port_index %d eeprom_addr = 0x%x\n", port, g_board_cfg.sensor_info[port].eeprom_addr);
	}
	node = cJSON_GetObjectItem(port_node, "sensor_name");
	if(NULL != node) {
		g_board_cfg.sensor_info[port].sensor_name = node->valuestring;
		pr_info("port_index %d sensor_name = %s\n", port, g_board_cfg.sensor_info[port].sensor_name);
	}
	node = cJSON_GetObjectItem(port_node, "power_mode");
	if(NULL != node) {
		g_board_cfg.sensor_info[port].power_mode = node->valueint;
		pr_info("port_index %d power_mode = %d\n", port, g_board_cfg.sensor_info[port].power_mode);
	}
	node = cJSON_GetObjectItem(port_node, "sensor_mode");
	if(NULL != node) {
		g_board_cfg.sensor_info[port].sensor_mode = node->valueint;
		pr_info("port_index %d sensor_mode = %d\n", port, g_board_cfg.sensor_info[port].sensor_mode);
	}
	node = cJSON_GetObjectItem(port_node, "config_index");
	if(NULL != node) {
		g_board_cfg.sensor_info[port].config_index = node->valueint;
		pr_info("port_index %d config_index = %d\n", port, g_board_cfg.sensor_info[port].config_index);
	}
	node = cJSON_GetObjectItem(port_node, "stream_control");
	if(NULL != node) {
		g_board_cfg.sensor_info[port].stream_control = node->valueint;
		pr_info("port_index %d stream_control = %d\n", port, g_board_cfg.sensor_info[port].stream_control);
	}
	node = cJSON_GetObjectItem(port_node, "entry_num");
	if(NULL != node) {
		g_board_cfg.sensor_info[port].entry_num = node->valueint;
		pr_info("port_index %d entry_num = %d\n", port, g_board_cfg.sensor_info[port].entry_num);
	}
	node = cJSON_GetObjectItem(port_node, "recovery_op");
	if(NULL != node) {
		g_board_cfg.sensor_info[port].recovery_op = node->valueint;
		pr_info("port_index %d recovery_op = %d\n", port, g_board_cfg.sensor_info[port].recovery_op);
	}
	node = cJSON_GetObjectItem(port_node, "spi_cs");
	if(NULL != node) {
		g_board_cfg.sensor_info[port].spi_info.spi_cs = node->valueint;
		pr_info("port_index %d spi_info.spi_cs = %d\n", port, g_board_cfg.sensor_info[port].spi_info.spi_cs);
	}
	node = cJSON_GetObjectItem(port_node, "spi_mode");
	if(NULL != node) {
		g_board_cfg.sensor_info[port].spi_info.spi_mode = node->valueint;
		pr_info("port_index %d spi_info.spi_mode = %d\n", port, g_board_cfg.sensor_info[port].spi_info.spi_mode);
	}
	node = cJSON_GetObjectItem(port_node, "spi_speed");
	if(NULL != node) {
		g_board_cfg.sensor_info[port].spi_info.spi_speed = node->valueint;
		pr_info("port_index %d spi_info.spi_speed = %d\n", port, g_board_cfg.sensor_info[port].spi_info.spi_speed);
	}
	node = cJSON_GetObjectItem(port_node, "reg_width");
	if(NULL != node) {
		g_board_cfg.sensor_info[port].reg_width = node->valueint;
		pr_info("port_index %d reg_width = %d\n",
		port, g_board_cfg.sensor_info[port].reg_width);
	}
	node = cJSON_GetObjectItem(port_node, "config_path");
	if(NULL != node) {
		g_board_cfg.sensor_info[port].config_path = node->valuestring;
		pr_info("port_index %d config_path = %s\n", port, g_board_cfg.sensor_info[port].config_path);
	}
	if(fps || g_board_cfg.sensor_info[port].fps) {
		if(fps) {
			g_board_cfg.sensor_info[port].fps = fps;
		}
	} else {
		node = cJSON_GetObjectItem(port_node, "fps");
		if(NULL != node) {
			g_board_cfg.sensor_info[port].fps = node->valueint;
			pr_info("port_index %d fps = %d\n", port, g_board_cfg.sensor_info[port].fps);
		}
	}

	/* add width && height  */
	node = cJSON_GetObjectItem(port_node, "width");
	if(NULL != node) {
		g_board_cfg.sensor_info[port].width = node->valueint;
		pr_info("port_index %d width = %d\n", port, g_board_cfg.sensor_info[port].width);
	}
	node = cJSON_GetObjectItem(port_node, "height");
	if(NULL != node) {
		g_board_cfg.sensor_info[port].height = node->valueint;
		pr_info("port_index %d height = %d\n", port, g_board_cfg.sensor_info[port].height);
	}
	node = cJSON_GetObjectItem(port_node, "format");
	if(NULL != node) {
		g_board_cfg.sensor_info[port].format = node->valueint;
		pr_info("port_index %d format = %d\n", port, g_board_cfg.sensor_info[port].format);
	}

	if(resolution || g_board_cfg.sensor_info[port].resolution) {
		if(resolution) {
			g_board_cfg.sensor_info[port].resolution = resolution;
		}
	} else {
		node = cJSON_GetObjectItem(port_node, "resolution");
		if(NULL != node) {
			g_board_cfg.sensor_info[port].resolution = node->valueint;
			pr_info("port_index %d resolution = %d\n", port, g_board_cfg.sensor_info[port].resolution);
		}
	}
	node = cJSON_GetObjectItem(port_node, "extra_mode");
	if(NULL != node) {
		g_board_cfg.sensor_info[port].extra_mode = node->valueint;
		pr_info("port_index %d extra_mode = %d\n", port, g_board_cfg.sensor_info[port].extra_mode);
	}
	node = cJSON_GetObjectItem(port_node, "power_delay");
	if(NULL != node) {
		g_board_cfg.sensor_info[port].power_delay = node->valueint;
		pr_info("port_index %d power_delay = %d\n", port, g_board_cfg.sensor_info[port].power_delay);
	}
	node = cJSON_GetObjectItem(port_node, "deserial_index");
	if(NULL != node) {
		g_board_cfg.sensor_info[port].deserial_index = node->valueint;
		pr_info("port_index %d deserial_index = %d\n", port, g_board_cfg.sensor_info[port].deserial_index);
	} else {
			g_board_cfg.sensor_info[port].deserial_index = -1;
	}
	node = cJSON_GetObjectItem(port_node, "deserial_port");
	if(NULL != node) {
		g_board_cfg.sensor_info[port].deserial_port = node->valueint;
		pr_info("port_index %d deserial_port = %d\n", port, g_board_cfg.sensor_info[port].deserial_port);
	} else {
			g_board_cfg.sensor_info[port].deserial_port = -1;
	}
	node = cJSON_GetObjectItem(port_node, "gpio_pin");
	if(NULL != node) {
		array_size = cJSON_GetArraySize(node);
		if (array_size > GPIO_NUMBER) {
			pr_err("invalid value! gpio_pin num exceed the maximum %d\n", GPIO_NUMBER);
			return -RET_ERROR;
		}
		g_board_cfg.sensor_info[port].gpio_num = array_size;
		for(array_index = 0; array_index < array_size; array_index++) {
			arrayitem = cJSON_GetArrayItem(node, array_index);
			// pr_debug("arrayitem->valueint %d\n", arrayitem->valueint);
			g_board_cfg.sensor_info[port].gpio_pin[array_index] = arrayitem->valueint;
			// pr_info("array_index %d gpio_pin %d\n", array_index, g_board_cfg.sensor_info[port].gpio_pin[array_index]);
		}
	}
	node = cJSON_GetObjectItem(port_node, "gpio_level");
	if(NULL != node) {
		array_size = cJSON_GetArraySize(node);
		if (array_size > GPIO_NUMBER) {
			pr_err("invalid value! gpio_level num exceed the maximum %d\n", GPIO_NUMBER);
			return -RET_ERROR;
		}
		g_board_cfg.sensor_info[port].gpio_num = array_size;
		for(array_index = 0; array_index < array_size; array_index++) {
			arrayitem = cJSON_GetArrayItem(node, array_index);
			pr_debug("arrayitem->valueint %d\n", arrayitem->valueint);
			if ((arrayitem->valueint != 0) && (arrayitem->valueint != 1)) {
				pr_err("invalid value! gpio_level must be 0 or 1!\n");
				return -RET_ERROR;
			}
			g_board_cfg.sensor_info[port].gpio_level[array_index] = arrayitem->valueint;
			pr_info("port_index %d array_index %d gpio_level %d\n", port, array_index, g_board_cfg.sensor_info[port].gpio_level[array_index]);
		}
	}
	ret = hb_cam_mipi_parse_cfg(g_board_cfg.sensor_info[port].config_path, g_board_cfg.sensor_info[port].fps, g_board_cfg.sensor_info[port].resolution, g_board_cfg.sensor_info[port].entry_num);
	if(ret < 0) {
		pr_err("cam hb_mipi_parse_cfg %s failed\n", g_board_cfg.sensor_info[port].config_path);
		return ret;
	}
	return ret;
}

int hb_cam_board_parse_cfg(uint32_t cfg_index, const char *filename)
{
	char *filebuf=NULL;
	char tmp[16] = {0};
	FILE *fp=NULL;
	int ret = RET_OK;
	int fread_ret;
	struct stat statbuf;
	cJSON *num = NULL;
	cJSON *board_name = NULL, *node = NULL, *lpwm_node = NULL;
	cJSON *arrayitem = NULL;
	int port_index, deserial_index;
	int array_size = 0, array_index;

	if(filename == NULL) {
		pr_err("cam config file is null !!\n");
		return -RET_ERROR;
	}
	ret = stat(filename, &statbuf);
	if(ret < 0) {
		pr_err("file %s stat is fail!\n", filename);
		return ret;
	}
	pr_debug("filename = %s \n", filename);
	if(0 == statbuf.st_size) {
		pr_err("cam config file size is zero !!\n");
		return -RET_ERROR;
	}
	fp = fopen(filename,"r");
	if(fp == NULL) {
		pr_err("open %s fail!!\n", filename);
		return -RET_ERROR;
	}
	filebuf = (char *)malloc(statbuf.st_size + 1);
	if(NULL == filebuf) {
		pr_err("malloc buff fail !!\n");
		goto err_close;
	}
	memset(filebuf, 0, statbuf.st_size + 1);
	fread_ret = fread(filebuf, statbuf.st_size, 1, fp);
	if(fread_ret < 1) {
		pr_err("fread is error! read size is %d\n", fread_ret);
		goto err1;
	}
	root=cJSON_Parse((const char *)filebuf);
	if(NULL == root) {
		pr_err("parse json fail\n");
		goto err1;
	}
	num = cJSON_GetObjectItem(root, "config_number");
	if(NULL != num) {
		g_board_cfg.config_number  = num->valueint;
		pr_info("config_number = %d\n", g_board_cfg.config_number);
	}
	if(g_board_cfg.config_number < 1) {
		pr_err("cam cfg number shoud be at least one");
		goto err;
	}
	if(cfg_index > ((uint32_t)g_board_cfg.config_number - 1)) {
		pr_err("cfg_index:%d not support\n", cfg_index);
		goto err;
	}
	board_name = cJSON_GetObjectItem(root, "board_name");
	if(NULL != board_name) {
		g_board_cfg.board_name  = board_name->valuestring;
		pr_info("board_name = %s\n", g_board_cfg.board_name);
	}
	snprintf(tmp, sizeof(tmp), "config_%d", cfg_index);
	pr_info("node is %s\n", tmp);
	board_info = cJSON_GetObjectItem(root,tmp);

	/* add for support more interface input s */
	node = cJSON_GetObjectItem(board_info, "interface_type");
	if (NULL != node) {
		g_board_cfg.interface_type = node->valuestring;
		pr_info("interface_type = %s\n", g_board_cfg.interface_type);
	}
	if (g_board_cfg.interface_type == NULL) {
		pr_err("need set interface_type in cfg json\n");
		goto err;
	}

	lpwm_node = cJSON_GetObjectItem(board_info, "lpwm_info");
	if(NULL != lpwm_node) {
		node = cJSON_GetObjectItem(lpwm_node, "ext_timer_en");
		if(NULL != node) {
			int ext_timer_en = node->valueint;
			g_board_cfg.lpwm_info.ext_timer_en = ext_timer_en;
			pr_info("pwm_info.ext_timer_en = %d \n", g_board_cfg.lpwm_info.ext_timer_en);
		}
		node = cJSON_GetObjectItem(lpwm_node, "ext_trig_en");
		if(NULL != node) {
			int ext_trig_en = node->valueint;
			g_board_cfg.lpwm_info.ext_trig_en = ext_trig_en;
			pr_info("pwm_info.ext_trig_en = %d \n", g_board_cfg.lpwm_info.ext_trig_en);
		}
		node = cJSON_GetObjectItem(lpwm_node, "lpwm_enable");
		if(NULL != node) {
			int lpwm_enable = hb_cam_htoi(node->valuestring);
			g_board_cfg.lpwm_info.lpwm_enable = lpwm_enable;
			pr_info("pwm_info.lpwm_enable = %d \n", g_board_cfg.lpwm_info.lpwm_enable);
		}
		node = cJSON_GetObjectItem(lpwm_node, "offset_us");
		if(NULL != node) {
			array_size = cJSON_GetArraySize(node);
			for(array_index = 0; array_index < array_size; array_index++) {
				arrayitem = cJSON_GetArrayItem(node, array_index);
				 g_board_cfg.lpwm_info.offset_us[array_index] = arrayitem->valueint;
				 pr_info("array_index %d offset_us %d\n",
				 	array_index, g_board_cfg.lpwm_info.offset_us[array_index]);
			}
		}
		node = cJSON_GetObjectItem(lpwm_node, "period_us");
		if(NULL != node) {
			array_size = cJSON_GetArraySize(node);
			for(array_index = 0; array_index < array_size; array_index++) {
				arrayitem = cJSON_GetArrayItem(node, array_index);
				 g_board_cfg.lpwm_info.period_us[array_index] = arrayitem->valueint;
				 pr_info("array_index %d period_us %d\n",
				 	array_index, g_board_cfg.lpwm_info.period_us[array_index]);
			}
		}
		node = cJSON_GetObjectItem(lpwm_node, "duty_us");
		if(NULL != node) {
			array_size = cJSON_GetArraySize(node);
			for(array_index = 0; array_index < array_size; array_index++) {
				arrayitem = cJSON_GetArrayItem(node, array_index);
				 g_board_cfg.lpwm_info.duty_us[array_index] = arrayitem->valueint;
				 pr_info("array_index %d duty_us %d\n",
				 	array_index, g_board_cfg.lpwm_info.duty_us[array_index]);
			}
		}
	}

	node = cJSON_GetObjectItem(board_info, "deserial_num");
	if(NULL != node) {
		if (node->valueint > SERDES_NUMBER) {
			pr_err("invalid value! deserial_num exceed the maximum %d\n", SERDES_NUMBER);
			goto err;
		}
		g_board_cfg.deserial_num = node->valueint;
		pr_info("deserial_num = %d\n", g_board_cfg.deserial_num);
	}
	for (deserial_index = 0; deserial_index < g_board_cfg.deserial_num; deserial_index++) {
		hb_cam_deserial_parse_cfg(deserial_index);
	}
	node = cJSON_GetObjectItem(board_info, "port_number");
	if(NULL != node) {
		g_board_cfg.port_number = node->valueint;
		if (node->valueint > CAM_MAX_NUM) {
			pr_err("invalid value! port_number exceed the maximum %d\n", CAM_MAX_NUM);
			goto err;
		}
		pr_info("port_number = %d\n", g_board_cfg.port_number);
	}
	for (port_index = 0; port_index < g_board_cfg.port_number; port_index++) {
		ret = hb_cam_port_parse_cfg(port_index, 0, 0);
		if(ret < 0) {
			pr_err("hb_port_parse_cfg port %d failed\n", port_index);
			goto err;
		}
	}
	hb_cam_struct_data_init();
	free(filebuf);
	fclose(fp);
	return RET_OK;

err:
	cJSON_Delete(root);
err1:
	free(filebuf);
err_close:
	fclose(fp);
	return -HB_CAM_PARSE_BOARD_CFG_FAIL;
}

int hb_cam_check_alive(int *num)
{
	int ret = RET_OK;
	pr_info("not support now\n");
	return ret;
}

int hb_cam_share_ae(uint32_t sharer_dev_port, uint32_t user_dev_port)
{
	int ret = 0;
	uint32_t v = 0;

	v = (0xA0 + sharer_dev_port) << 16 | user_dev_port;
	ret = hb_cam_ae_share_init(sharer_dev_port, v);

	return ret;
}

int hb_cam_init(uint32_t cfg_index, const char *cfg_file)
{
	int ret = RET_OK;
	int port = 0, deserial_index;
	hb_vcam_msg_t init_msg;
	char *config_file = NULL;
	char *config_index = NULL;

	if (NULL == cfg_file) {
		config_file = getenv("CAM_CONFIG_FILE_PATH");
		pr_info("config_file %s\n", config_file);
		if(NULL != config_file) {
			cfg_file = config_file;
			config_index = getenv("CAM_INDEX");
			if(NULL != config_index) {
				cfg_index = atoi(config_index);
			} else {
				cfg_index = 0;
			}
		}
	}
	if(NULL == cfg_file) {
		pr_err("input error, cam config file is NULL!!!\n");
		return -HB_CAM_INIT_FAIL;
	}

	if(g_board_cfg.sensor_info[port].init_state == CAM_INIT) {
		pr_err("camera has already inited\n");
		return ret;
	}

	strncpy(cam_config_file, cfg_file, MAX_NUM_LENGTH-1);
	pr_info("cam init begin cam_index %d  cfg_file %s\n", cfg_index, cfg_file);
	memset(&g_board_cfg, 0, sizeof(g_board_cfg));
	ret = hb_cam_board_parse_cfg(cfg_index, cfg_file);
	if(ret < 0) {
		pr_err("cam parser %s failed\n", cfg_file);
		hb_cam_diag(ret);
		return -HB_CAM_PARSE_BOARD_CFG_FAIL;
	}
	if (g_board_cfg.lpwm_info.lpwm_enable != 0) {
		ret = hb_lpwm_config(PWM_PIN_NUM, g_board_cfg.lpwm_info.offset_us,
			g_board_cfg.lpwm_info.period_us, g_board_cfg.lpwm_info.duty_us);
		if (ret < 0) {
			pr_err("hb_lpwm_config fail ret %d \n", ret);
			goto err1;
		}
	}
	if (!strcmp(g_board_cfg.interface_type, INTERFACE_SDIO)) {
		init_msg.slot_info.img_info.width = g_board_cfg.sensor_info[port].width;
		init_msg.slot_info.img_info.height = g_board_cfg.sensor_info[port].height;
		init_msg.slot_info.img_info.format = g_board_cfg.sensor_info[port].format;
		ret = hb_vcam_init(&init_msg);
		if (ret < 0) {
			pr_err("cam start fail %s\n", g_board_cfg.interface_type);
			goto err1;
		}
		pr_info("interface type %s\n", g_board_cfg.interface_type);
	} else {
		for(deserial_index = 0; deserial_index < g_board_cfg.deserial_num; deserial_index++) {
			ret = hb_deserial_init(&g_board_cfg.deserial_info[deserial_index]);
			if(ret < 0) {
				pr_err("hb_deserial_init error! ret %d \n", ret);
				goto err1;
			}
		}
		for(port = 0; port < g_board_cfg.port_number; port++) {
			ret = camera_init_utility(&g_board_cfg.sensor_info[port]);
			if (ret < 0) {
				pr_err("camera_init_utility fail port %d\n", port);
				hb_cam_diag(ret);
				goto err;
			}
			g_board_cfg.sensor_info[port].init_state = CAM_INIT;
		}
	}
	return ret;
err:
	for(deserial_index = 0; deserial_index < g_board_cfg.deserial_num; deserial_index++) {
		hb_deserial_deinit(&g_board_cfg.deserial_info[deserial_index]);
	}
err1:
	cJSON_Delete(root);
	return -HB_CAM_INIT_FAIL;
}
int hb_cam_deinit(uint32_t cfg_index)
{
	int ret = RET_OK;
	int port = 0, deserial_index;

	pr_info("cam deinit begin\n");
	if(g_board_cfg.sensor_info[port].init_state != CAM_INIT) {
		pr_err("need cam init before cam deinit\n");
		return -HB_CAM_DEINIT_FAIL;
	}
	if (!strcmp(g_board_cfg.interface_type, INTERFACE_SDIO)) {
		hb_vcam_deinit();
		pr_err("cam start interface type %s\n", g_board_cfg.interface_type);
	} else {
		if(g_board_cfg.sensor_info[port].start_state == CAM_START) {
			for(port = 0; port < g_board_cfg.port_number; port++) {
				camera_stop_utility(&g_board_cfg.sensor_info[port]);
			}
		}
		for(deserial_index = 0; deserial_index < g_board_cfg.deserial_num; deserial_index++) {
			 hb_deserial_deinit(&g_board_cfg.deserial_info[deserial_index]);
		}
		for(port = 0; port < g_board_cfg.port_number; port++) {
			camera_deinit_utility(&g_board_cfg.sensor_info[port]);
			g_board_cfg.sensor_info[port].init_state = CAM_DEINIT;
		}
	}

	cJSON_Delete(root);
	return ret;
}
int hb_cam_start(uint32_t port)
{
	int ret = RET_OK;
	char *x2_port = NULL;

	if(port >= (uint32_t)g_board_cfg.port_number) {
		pr_err("not support port%d, max port %d\n", port, g_board_cfg.port_number - 1);
		x2_port = getenv("CAM_PORT");
		if(NULL != x2_port) {
			pr_info("force port %d to CAM_PORT %d\n", port, atoi(x2_port));
			port = atoi(x2_port);
			if(port >= (uint32_t)g_board_cfg.port_number) {
					pr_err("not spport CAM_PORT %d, max port is %d \n", port,
						g_board_cfg.port_number - 1);
					return -HB_CAM_INVALID_PARAM;
			}
		} else {
			return -HB_CAM_INVALID_PARAM;
		}
	}
	if(g_board_cfg.sensor_info[port].init_state != CAM_INIT) {
		pr_err("need cam init before cam start\n");
		return -HB_CAM_START_FAIL;
	}
	if(g_board_cfg.sensor_info[port].start_state == CAM_START) {
		pr_err("cam have been started\n");
		return ret;
	}
	/* add for vcam */
	if (!strcmp(g_board_cfg.interface_type, INTERFACE_SDIO)) {
		ret = hb_vcam_start();
		if (ret < 0) {
			pr_err("cam start fail %s\n", g_board_cfg.interface_type);
			return ret;
		}
		pr_err("cam start interface type %s\n", g_board_cfg.interface_type);
	} else {
		if (g_board_cfg.lpwm_info.lpwm_enable != 0) {
			if (g_board_cfg.lpwm_info.lpwm_start == 0) {
				ret = hb_lpwm_start(&g_board_cfg.lpwm_info);
				if (ret < 0) {
					pr_err("hb_lpwm_start fail ret %d \n", ret);
					return ret;
				}
				g_board_cfg.lpwm_info.lpwm_start = 1;
			}
		}
		ret = camera_start_utility(&g_board_cfg.sensor_info[port]);
		if(ret < 0) {
			pr_err("!!!camera_start_utility port %d ret %d\n", port, ret);
			hb_cam_diag(ret);
			return -HB_CAM_START_FAIL;
		}
	}
	g_board_cfg.sensor_info[port].start_state = CAM_START;
	pr_info("cam start end\n");
	hb_cam_diag(ret);
	return ret;
}

int hb_cam_stop(uint32_t port)
{
	int ret = RET_OK;
	char *x2_port = NULL;

	pr_info("camera stop begin port %d\n", port);
	if(port >= (uint32_t)g_board_cfg.port_number) {
		pr_err("not support port%d, max port %d\n", port, g_board_cfg.port_number - 1);
		x2_port = getenv("CAM_PORT");
		if(NULL != x2_port) {
			pr_info("force port %d to CAM_PORT %d\n", port, atoi(x2_port));
			port = atoi(x2_port);
			if(port >= g_board_cfg.port_number) {
					pr_err("not spport CAM_PORT %d, max port is %d \n", port,
						g_board_cfg.port_number - 1);
					return -HB_CAM_STOP_FAIL;
			}
		} else {
			return -HB_CAM_STOP_FAIL;
		}
	}
	if(g_board_cfg.sensor_info[port].init_state != CAM_INIT) {
		pr_err("need cam init before cam stop\n");
		return -HB_CAM_STOP_FAIL;
	}
	if(g_board_cfg.sensor_info[port].start_state != CAM_START) {
		pr_err("need cam start before cam stop\n");
		return ret;
	}
	if(g_board_cfg.sensor_info[port].start_state == CAM_STOP) {
		pr_err("cam have been stopped\n");
		return ret;
	}
	/* add for vcam */
	if (!strcmp(g_board_cfg.interface_type, INTERFACE_SDIO)) {
		ret = hb_vcam_stop();
		if (ret < 0) {
			pr_err("cam stop fail %s\n", g_board_cfg.interface_type);
			return ret;
		}
		pr_err("cam stop interface type %s\n", g_board_cfg.interface_type);
	} else {
		if (g_board_cfg.lpwm_info.lpwm_enable != 0) {
		  if (g_board_cfg.lpwm_info.lpwm_stop == 0) {
			hb_lpwm_stop(&g_board_cfg.lpwm_info);
			g_board_cfg.lpwm_info.lpwm_stop = 1;
		  }
		}
		ret = camera_stop_utility(&g_board_cfg.sensor_info[port]);
		if(ret < 0) {
			pr_err("!!!camera_stop_utility port %d ret %d\n", port, ret);
			return -HB_CAM_STOP_FAIL;
		}
	}
	g_board_cfg.sensor_info[port].start_state = CAM_STOP;
	pr_info("camera stop end\n");
	return ret;
}

int hb_cam_ipi_reset(uint32_t entry_num, uint32_t ipi_index, uint32_t enable)
{
	int ret = RET_OK;
	entry_t *entry_info = NULL;

	if (entry_num >= ENTRY_NUM)
		return -RET_ERROR;
	pr_info("entry_num %d ipi_index %d enable %d--begin-\n",
		entry_num, ipi_index, enable);
	ret = hb_vin_ipi_reset(entry_num, ipi_index, enable);
	if(ret < 0) {
		pr_err("hb_vin_ipi_reset fail\n");
		return -HB_CAM_IPI_RESET_FAIL;
	}
	pr_info("entry_num %d ipi_index %d enable %d--end---\n",
		entry_num, ipi_index, enable);
	return ret;
}
int hb_cam_mipi_stop_deinit(uint32_t entry_num)
{
	int ret = RET_OK;
	uint32_t port;
	int recovery_op = 0;

	for(port = 0; port < g_board_cfg.port_number; port++) {
		if(g_board_cfg.sensor_info[port].entry_num == entry_num) {
			pr_err("hb_cam_mipi_stop_deinit entry_num %d port %d\n",
				entry_num, port);
			recovery_op = g_board_cfg.sensor_info[port].recovery_op;
			if(((sensor_module_t *)(g_board_cfg.sensor_info[port].sensor_ops))->stream_off != NULL) {
				((sensor_module_t *)(g_board_cfg.sensor_info[port].sensor_ops))->stream_off(&g_board_cfg.sensor_info[port]);
			}
			if(recovery_op == 1) {
				hb_vin_ipi_reset(entry_num, g_board_cfg.sensor_info[port].deserial_port, 0);
			}
			if(recovery_op == 0) {
				hb_vin_stop(entry_num);
			}
		}
	}
	for(port = 0; port < g_board_cfg.port_number; port++) {
		if(g_board_cfg.sensor_info[port].entry_num == entry_num) {
			recovery_op = g_board_cfg.sensor_info[port].recovery_op;
			if(recovery_op == 0) {
				hb_vin_deinit(entry_num);
			}
		}
	}
	return ret;
}

int hb_cam_mipi_start(uint32_t entry_num)
{
	int ret = RET_OK;
	uint32_t port;
	int recovery_op = 0;

	for(port = 0; port < g_board_cfg.port_number; port++) {
		if(g_board_cfg.sensor_info[port].entry_num == entry_num) {
			recovery_op = g_board_cfg.sensor_info[port].recovery_op;
			if(recovery_op == 1) {
				hb_vin_ipi_reset(entry_num, g_board_cfg.sensor_info[port].deserial_port, 1);
			}
			if(((sensor_module_t *)(g_board_cfg.sensor_info[port].sensor_ops))->stream_on != NULL) {
			((sensor_module_t *)(g_board_cfg.sensor_info[port].sensor_ops))->stream_on(&g_board_cfg.sensor_info[port]);
			}
			if(recovery_op  == 0) {
				ret = hb_vin_start(entry_num);
				if(ret < 0) {
					pr_err("hb_cam_mipi_start ret %d entry_num %d\n", ret, entry_num);
					return -1;
				}
			}
		}
	}
	pr_err("hb_cam_mipi_start success\n");
	return ret;
}

int hb_cam_mipi_init(uint32_t entry_num)
{
	int ret = RET_OK;
	uint32_t port;
	int recovery_op = 0;

	for(port = 0; port < g_board_cfg.port_number; port++) {
		if(g_board_cfg.sensor_info[port].entry_num == entry_num) {
			recovery_op = g_board_cfg.sensor_info[port].recovery_op;
			if(recovery_op == 0) {
				ret = hb_vin_init(entry_num);
				if(ret < 0) {
					pr_err("mipi init error port %d ret %d entry_num %d\n",
						port, ret, entry_num);
					return -1;
				}
			}
		}
	}
	pr_err("hb_cam_mipi_init success \n");
	return ret;
}

int hb_cam_get_fps(uint32_t port, uint32_t *fps)
{
	int ret = RET_OK;
	char *x2_port = NULL;

	if (fps == NULL) {
		return -HB_CAM_INVALID_PARAM;
	}
	if(g_board_cfg.sensor_info[port].init_state != CAM_INIT) {
		pr_err("need cam init before get fps\n");
		return -HB_CAM_OPS_NOT_SUPPORT;
	}
	if(port >= g_board_cfg.port_number) {
		pr_err("not support port%d, max port %d\n", port,
					g_board_cfg.port_number - 1);
		x2_port = getenv("CAM_PORT");
		if(NULL != x2_port) {
			pr_info("force port %d to CAM_PORT %d\n", port, atoi(x2_port));
			port = atoi(x2_port);
			if(port >= g_board_cfg.port_number) {
					pr_err("not spport CAM_PORT %d, max port is %d \n", port,
						g_board_cfg.port_number - 1);
					return -HB_CAM_INVALID_PARAM;
			}
		} else {
			return -HB_CAM_INVALID_PARAM;
		}
	}
	*fps = g_board_cfg.sensor_info[port].fps;
	return ret;
}

int hb_cam_get_img(cam_img_info_t *cam_img_info)
{
	int ret = RET_OK;

	if (cam_img_info == NULL) {
		pr_err("%d img_info is null\n", __LINE__);
		return -RET_ERROR;
	}
	ret = hb_vcam_get_img(cam_img_info);
	if (ret < 0) {
		pr_err("cam get img fail %s\n", g_board_cfg.interface_type);
		return ret;
	}
	return ret;
}

int hb_cam_free_img(cam_img_info_t *cam_img_info)
{
	int ret = RET_OK;

	if (cam_img_info == NULL) {
		pr_err("%d img_info is null\n", __LINE__);
		return -RET_ERROR;
	}
	ret = hb_vcam_free_img(cam_img_info);
	if (ret < 0) {
		pr_err("cam free img fail %s\n", g_board_cfg.interface_type);
		return ret;
	}
	return ret;
}

int hb_cam_clean_img(cam_img_info_t *cam_img_info)
{
	int ret = RET_OK;

	if (cam_img_info == NULL) {
		pr_err("%d img_info is null\n", __LINE__);
		return -RET_ERROR;
	}
	ret = hb_vcam_clean(cam_img_info);
	if (ret < 0) {
		pr_err("cam clean img fail %s\n", g_board_cfg.interface_type);
		return ret;
	}
	return ret;
}
int hb_cam_enable_mclk(uint32_t entry_num)
{
	int ret = RET_OK;
	entry_t *entry_info = NULL;

	if (entry_num >= ENTRY_NUM)
		return -RET_ERROR;
	pr_info("entry_num %d hb_cam_enable_mclk begin\n", entry_num);
	ret = hb_vin_snrclk_set_en(entry_num, 1);
	if(ret < 0) {
		pr_err("hb_vin_snrclk_set_en fail\n");
		return -HB_CAM_ENABLE_CLK_FAIL;
	}
	pr_info("hb_cam_enable_mclk end\n");
	return ret;
}

int hb_cam_disable_mclk(uint32_t entry_num)
{
	int ret = RET_OK;
	entry_t *entry_info = NULL;

	if (entry_num >= ENTRY_NUM)
		return -RET_ERROR;
	pr_info("hb_cam_disable_mclk begin\n");
	ret = hb_vin_snrclk_set_en(entry_num, 0);
	if(ret < 0) {
		pr_err("hb_vin_snrclk_set_en fail\n");
		return -HB_CAM_DISABLE_CLK_FAIL;
	}
	pr_info("hb_cam_disable_mclk end\n");
	return ret;
}

int hb_cam_set_mclk(uint32_t entry_num, uint32_t mclk)
{
	int ret = RET_OK;
	entry_t *entry_info = NULL;

	if (entry_num >= ENTRY_NUM)
		return -RET_ERROR;
	pr_info("hb_cam_set_mclk begin\n");
	ret = hb_vin_snrclk_set_freq(entry_num, mclk);
	if(ret < 0) {
		pr_err("hb_vin_snrclk_set_freq fail\n");
		return -HB_CAM_SET_CLK_FAIL;
	}
	pr_info("hb_cam_set_mclk end\n");
	return ret;
}

extern int isp_sns_mode_switch(uint8_t ctx_id, camera_info_table_t *sns_table);
int hb_cam_dynamic_switch_mode(uint32_t port, camera_info_table_t *sns_table)
{
	int ret = RET_OK;
	char *x2_port = NULL;
	int old_mode;
	int process = 0;
	int i = 0;
#define	IPI_RESTORE_RETRY_TIME	10

	if(port >= g_board_cfg.port_number) {
		pr_err("not support port%d, max port %d\n", port, g_board_cfg.port_number - 1);
		x2_port = getenv("CAM_PORT");
		if(NULL != x2_port) {
			pr_info("force port %d to CAM_PORT %d\n", port, atoi(x2_port));
			port = atoi(x2_port);
			if(port >= g_board_cfg.port_number) {
					pr_err("not spport CAM_PORT %d, max port is %d \n", port,
						g_board_cfg.port_number - 1);
					return -HB_CAM_INVALID_PARAM;
			}
		} else {
			return -HB_CAM_INVALID_PARAM;
		}
	}
	if(NULL == sns_table) {
		pr_err("sns_table is NULL\n");
		return -HB_CAM_DYNAMIC_SWITCH_MODE_FAIL;
	}
	if(sns_table->mode >= HB_INVALID_MOD || sns_table->mode == 0) {
		pr_err("mode is invalid\n");
		return -HB_CAM_DYNAMIC_SWITCH_MODE_FAIL;
	}
	if(g_board_cfg.sensor_info[port].init_state != CAM_INIT) {
		pr_err("need cam init before cam_dynamic_switch mode");
		return -HB_CAM_DYNAMIC_SWITCH_MODE_FAIL;
	}
	old_mode = g_board_cfg.sensor_info[port].sensor_mode;
	if(g_board_cfg.sensor_info[port].sensor_mode == sns_table->mode) {
		pr_err("pipe %d is mode %d now, no need switch\n", port, (int)sns_table->mode);
		return -HB_CAM_DYNAMIC_SWITCH_MODE_FAIL;
	}

	ret = ((sensor_module_t *)(g_board_cfg.sensor_info[port].sensor_ops))->stop(&g_board_cfg.sensor_info[port]);
	if (ret < 0) {
		pr_err("sensor_stop fail ret %d\n", ret);
		return -HB_CAM_DYNAMIC_SWITCH_MODE_FAIL;
	}

	sleep(1);	//wait sensor stable and isp knl-evt process done then switch

	hb_vin_ipi_reset(g_board_cfg.sensor_info[port].entry_num,
		g_board_cfg.sensor_info[port].deserial_port, 0);

	ret = isp_sns_mode_switch((uint8_t)g_board_cfg.sensor_info[port].dev_port,
				sns_table);
	if(ret < 0) {
		return -1;
	}
	sleep(1);	//wait isp switch done
	g_board_cfg.sensor_info[port].sensor_mode = sns_table->mode;
	ret = ((sensor_module_t *)(g_board_cfg.sensor_info[port].sensor_ops))->init(&g_board_cfg.sensor_info[port]);
	if (ret < 0) {
		pr_err("sensor_init fail ret %d\n", ret);
		g_board_cfg.sensor_info[port].sensor_mode = old_mode;
		return -HB_CAM_DYNAMIC_SWITCH_MODE_FAIL;
	}

	hb_vin_ipi_reset(g_board_cfg.sensor_info[port].entry_num,
		g_board_cfg.sensor_info[port].deserial_port, 1);

	ret = ((sensor_module_t *)(g_board_cfg.sensor_info[port].sensor_ops))->start(&g_board_cfg.sensor_info[port]);
	if (ret < 0) {
		pr_err("sensor_start fail ret %d\n", ret);
	    g_board_cfg.sensor_info[port].sensor_mode = old_mode;
		return -HB_CAM_DYNAMIC_SWITCH_MODE_FAIL;
	}

	ret = hb_vin_ipi_fatal(g_board_cfg.sensor_info[port].entry_num,
					g_board_cfg.sensor_info[port].deserial_port);
	if (ret < 0) {
		pr_err("ipi status read fail ret %d\n", ret);
		return -HB_CAM_DYNAMIC_SWITCH_MODE_FAIL;
	}

	while (ret > 0 && ++i <= IPI_RESTORE_RETRY_TIME) {
		process = 1;

		pr_err("[IPI OVERFLOW] ctx %d, reset cnt %d, host%d, ipi%d, ipi st %x\n",
				port, i,
				g_board_cfg.sensor_info[port].entry_num,
				g_board_cfg.sensor_info[port].deserial_port, ret);

		ret = ((sensor_module_t *)(g_board_cfg.sensor_info[port].sensor_ops))->stop(&g_board_cfg.sensor_info[port]);
		if (ret < 0) {
			pr_err("sensor_stop fail ret %d\n", ret);
			return -HB_CAM_DYNAMIC_SWITCH_MODE_FAIL;
		}
		usleep(50*1000);
		hb_vin_ipi_reset(g_board_cfg.sensor_info[port].entry_num,
		g_board_cfg.sensor_info[port].deserial_port, 0);

		usleep(50*1000);

		hb_vin_ipi_reset(g_board_cfg.sensor_info[port].entry_num,
		g_board_cfg.sensor_info[port].deserial_port, 1);
		usleep(50*1000);
		ret = ((sensor_module_t *)(g_board_cfg.sensor_info[port].sensor_ops))->start(&g_board_cfg.sensor_info[port]);
		if (ret < 0) {
			pr_err("sensor_start fail ret %d\n", ret);
			g_board_cfg.sensor_info[port].sensor_mode = old_mode;
			return -HB_CAM_DYNAMIC_SWITCH_MODE_FAIL;
		}

		sleep(1);
		ret = hb_vin_ipi_fatal(g_board_cfg.sensor_info[port].entry_num,
					g_board_cfg.sensor_info[port].deserial_port);
		if (ret < 0) {
			pr_err("ipi status read fail ret %d\n", ret);
			return -HB_CAM_DYNAMIC_SWITCH_MODE_FAIL;
		}
	}

	if (process) {
		if (i < IPI_RESTORE_RETRY_TIME) {
			pr_err("[DONE] host%d, ipi%d, ipi st %x\n",
					g_board_cfg.sensor_info[port].entry_num,
					g_board_cfg.sensor_info[port].deserial_port, ret);
		} else if (i > IPI_RESTORE_RETRY_TIME) {
			pr_err("[FAILED] host%d, ipi%d, ipi st %x\n",
					g_board_cfg.sensor_info[port].entry_num,
					g_board_cfg.sensor_info[port].deserial_port, ret);
		}
	}

	return ret;
}

int  hb_cam_i2c_read(uint32_t port, uint32_t reg_addr)
{
	int value = 0;
	char *x2_port = NULL;

	if(port >= g_board_cfg.port_number) {
		pr_err("not support port%d, max port %d\n", port, g_board_cfg.port_number - 1);
		x2_port = getenv("CAM_PORT");
		if(NULL != x2_port) {
			pr_info("force port %d to CAM_PORT %d\n", port, atoi(x2_port));
			port = atoi(x2_port);
			if(port >= g_board_cfg.port_number) {
					pr_err("not spport CAM_PORT %d, max port is %d \n", port,
						g_board_cfg.port_number - 1);
					return -HB_CAM_INVALID_PARAM;
			}
		} else {
			return -HB_CAM_INVALID_PARAM;
		}
	}
	value = camera_i2c_read16(g_board_cfg.sensor_info[port].bus_num,
		g_board_cfg.sensor_info[port].reg_width,
		(uint8_t)g_board_cfg.sensor_info[port].sensor_addr, reg_addr);
	if(value < 0) {
		pr_err("value 0x%x is invalid\n", value);
		return -HB_CAM_I2C_READ_FAIL;
	}
	return value;
}

int  hb_cam_i2c_read_byte(uint32_t port, uint32_t reg_addr)
{
	int value = 0;
	char *x2_port = NULL;

	if(port >= g_board_cfg.port_number) {
		pr_err("not support port%d, max port %d\n", port, g_board_cfg.port_number - 1);
		x2_port = getenv("CAM_PORT");
		if(NULL != x2_port) {
			pr_info("force port %d to CAM_PORT %d\n", port, atoi(x2_port));
			port = atoi(x2_port);
			if(port >= g_board_cfg.port_number) {
					pr_err("not spport CAM_PORT %d, max port is %d \n", port,
						g_board_cfg.port_number - 1);
					return -HB_CAM_INVALID_PARAM;
			}
		} else {
			return -HB_CAM_INVALID_PARAM;
		}
	}
	value = camera_i2c_read8(g_board_cfg.sensor_info[port].bus_num,
		g_board_cfg.sensor_info[port].reg_width,
		(uint8_t)g_board_cfg.sensor_info[port].sensor_addr, reg_addr);
	if(value < 0) {
		pr_err("value 0x%x is invalid\n", value);
		return -HB_CAM_I2C_READ_BYTE_FAIL;
	}
	return value;
}

int  hb_cam_i2c_write(uint32_t port, uint32_t reg_addr, uint16_t value)
{
	int ret = RET_OK;
	char *x2_port = NULL;

	if(port >= g_board_cfg.port_number) {
		pr_err("not support port%d, max port %d\n", port, g_board_cfg.port_number - 1);
		x2_port = getenv("CAM_PORT");
		if(NULL != x2_port) {
			pr_info("force port %d to CAM_PORT %d\n", port, atoi(x2_port));
			port = atoi(x2_port);
			if(port >= g_board_cfg.port_number) {
					pr_err("not spport CAM_PORT %d, max port is %d \n", port,
						g_board_cfg.port_number - 1);
					return -HB_CAM_INVALID_PARAM;
			}
		} else {
			return -HB_CAM_INVALID_PARAM;
		}
	}
	ret = camera_i2c_write16(g_board_cfg.sensor_info[port].bus_num,
		g_board_cfg.sensor_info[port].reg_width,
		(uint8_t)g_board_cfg.sensor_info[port].sensor_addr, reg_addr, value);
	if(ret < 0) {
		pr_err("camera: write %x = %x fail\n", reg_addr, value);
		return -HB_CAM_I2C_WRITE_FAIL;
	}
	return RET_OK;
}

int  hb_cam_i2c_write_byte(uint32_t port, uint32_t reg_addr, uint8_t value)
{
	int ret = RET_OK;
	char *x2_port = NULL;

	if(port >= g_board_cfg.port_number) {
		pr_err("not support port%d, max port %d\n", port, g_board_cfg.port_number - 1);
		x2_port = getenv("CAM_PORT");
		if(NULL != x2_port) {
			pr_info("force port %d to CAM_PORT %d\n", port, atoi(x2_port));
			port = atoi(x2_port);
			if(port >= g_board_cfg.port_number) {
					pr_err("not spport CAM_PORT %d, max port is %d \n", port,
						g_board_cfg.port_number - 1);
					return -HB_CAM_INVALID_PARAM;
			}
		} else {
			return -HB_CAM_INVALID_PARAM;
		}
	}
	ret = camera_i2c_write8(g_board_cfg.sensor_info[port].bus_num,
		g_board_cfg.sensor_info[port].reg_width,
		(uint8_t)g_board_cfg.sensor_info[port].sensor_addr, reg_addr, value);
	if(ret < 0) {
		pr_err("camera: write %x = %x\n", reg_addr, value);
		return -HB_CAM_I2C_WRITE_BYTE_FAIL;
	}
	return RET_OK;
}

int  hb_cam_i2c_block_write(uint32_t port, uint32_t subdev, uint32_t reg_addr, char *buffer, uint32_t size)
{
	int ret = RET_OK;
	char *x2_port = NULL;

	if(NULL == buffer) {
		pr_err("%d buffer is null\n", __LINE__);
		return -HB_CAM_INVALID_PARAM;
	}
	if(port >= g_board_cfg.port_number) {
		pr_err("not support port%d, max port %d\n", port, g_board_cfg.port_number - 1);
		x2_port = getenv("CAM_PORT");
		if(NULL != x2_port) {
			pr_info("force port %d to CAM_PORT %d\n", port, atoi(x2_port));
			port = atoi(x2_port);
			if(port >= g_board_cfg.port_number) {
					pr_err("not spport CAM_PORT %d, max port is %d \n", port,
						g_board_cfg.port_number - 1);
					return -HB_CAM_INVALID_PARAM;
			}
		} else {
			return -HB_CAM_INVALID_PARAM;
		}
	}
	if(subdev >= DEVICE_INVALID) {
		pr_err("not support subdev type, max subdev is %d\n", IMU_DEVICE);
		return -HB_CAM_INVALID_PARAM;
	}
	if(subdev == SENSOR_DEVICE) {
		ret = camera_i2c_write_block(g_board_cfg.sensor_info[port].bus_num, g_board_cfg.sensor_info[port].reg_width, g_board_cfg.sensor_info[port].sensor_addr, reg_addr, buffer, size);
	}
	if(subdev == ISP_DEVICE) {
		ret = camera_i2c_write_block(g_board_cfg.sensor_info[port].bus_num, g_board_cfg.sensor_info[port].reg_width, g_board_cfg.sensor_info[port].isp_addr, reg_addr, buffer, size);
	}
	if((subdev == EEPROM_DEVICE) || (subdev == IMU_DEVICE)) {
		pr_err("not support subdev type \n");
		return -HB_CAM_INVALID_PARAM;
	}
	if(ret < 0) {
		pr_err("camera: write 0x%x block fail\n", reg_addr);
		return -HB_CAM_I2C_WRITE_BLOCK_FAIL;
	}
	return RET_OK;
}

int  hb_cam_i2c_block_read(uint32_t port, uint32_t subdev, uint32_t reg_addr, char *buffer, uint32_t size)
{
	int ret = RET_OK;
	char *x2_port = NULL;

	if(NULL == buffer) {
		pr_err("%d buffer is null\n", __LINE__);
		return -HB_CAM_INVALID_PARAM;
	}
	if(port >= g_board_cfg.port_number) {
		pr_err("not support port%d, max port %d\n", port, g_board_cfg.port_number - 1);
		x2_port = getenv("CAM_PORT");
		if(NULL != x2_port) {
			pr_info("force port %d to CAM_PORT %d\n", port, atoi(x2_port));
			port = atoi(x2_port);
			if(port >= g_board_cfg.port_number) {
					pr_err("not spport CAM_PORT %d, max port is %d \n", port,
						g_board_cfg.port_number - 1);
					return -HB_CAM_INVALID_PARAM;
			}
		} else {
			return -HB_CAM_INVALID_PARAM;
		}
	}
	if(subdev >= DEVICE_INVALID) {
		pr_err("not support subdev type, max subdev is %d\n", IMU_DEVICE);
		return -HB_CAM_INVALID_PARAM;
	}
	if(subdev == SENSOR_DEVICE) {
		ret = camera_i2c_read_block(g_board_cfg.sensor_info[port].bus_num, g_board_cfg.sensor_info[port].reg_width, g_board_cfg.sensor_info[port].sensor_addr, reg_addr, buffer, size);
	}
	if(subdev == ISP_DEVICE) {
		ret = camera_i2c_read_block(g_board_cfg.sensor_info[port].bus_num, g_board_cfg.sensor_info[port].reg_width, g_board_cfg.sensor_info[port].isp_addr, reg_addr, buffer, size);
	}
	if((subdev == EEPROM_DEVICE) || (subdev == IMU_DEVICE)) {
		pr_err("not support subdev type \n");
		return -HB_CAM_INVALID_PARAM;
	}
	if(ret < 0) {
		pr_err("camera: read 0x%x block fail\n", reg_addr);
		return -HB_CAM_I2C_READ_BLOCK_FAIL;
	}
	return RET_OK;
}

int  hb_cam_spi_block_write(uint32_t port, uint32_t subdev,          uint32_t reg_addr, char *buffer, uint32_t size)
{
	int ret = RET_OK;
	char *x2_port = NULL;

	if(NULL == buffer) {
		pr_err("%d buffer is null\n", __LINE__);
		return -HB_CAM_INVALID_PARAM;
	}
	if(subdev >= DEVICE_INVALID) {
		pr_err("not support subdev type, max subdev is %d\n", IMU_DEVICE);
		return -HB_CAM_INVALID_PARAM;
	}
	if(port >= g_board_cfg.port_number) {
		pr_err("not support port%d, max port %d\n", port, g_board_cfg.port_number - 1);
		x2_port = getenv("CAM_PORT");
		if(NULL != x2_port) {
			pr_info("force port %d to CAM_PORT %d\n", port, atoi(x2_port));
			port = atoi(x2_port);
			if(port >= g_board_cfg.port_number) {
					pr_err("not spport CAM_PORT %d, max port is %d \n", port,
						g_board_cfg.port_number - 1);
					return -HB_CAM_INVALID_PARAM;
			}
		} else {
			return -HB_CAM_INVALID_PARAM;
		}
	}
	if(subdev == SENSOR_DEVICE) {
		if(((sensor_module_t *)(g_board_cfg.sensor_info[port].sensor_ops))->spi_write == NULL) {
	 	   pr_err("sensor not suuport spi_block_write ops\n");
	 	   return -HB_CAM_OPS_NOT_SUPPORT;
	     }
		ret = ((sensor_module_t *)(g_board_cfg.sensor_info[port].sensor_ops))->spi_write(&g_board_cfg.sensor_info[port], reg_addr, buffer, size);
		if (ret < 0) {
			pr_err("spi_write fail\n");
			ret = -HB_CAM_SPI_WRITE_BLOCK_FAIL;
		}
	}
	if((subdev == EEPROM_DEVICE) || (subdev == IMU_DEVICE) || (subdev == ISP_DEVICE)) {
		pr_err("not support subdev type \n");
		return -HB_CAM_INVALID_PARAM;
	}
	return RET_OK;
}

int  hb_cam_spi_block_read(uint32_t port, uint32_t subdev, uint32_t reg_addr, char *buffer, uint32_t size)
{
	int ret = RET_OK;
	char *x2_port = NULL;

	if(NULL == buffer) {
		pr_err("%d buffer is null\n", __LINE__);
		return -HB_CAM_INVALID_PARAM;
	}
	if(subdev >= DEVICE_INVALID) {
		pr_err("not support subdev type, max subdev is %d\n", IMU_DEVICE);
		return -HB_CAM_INVALID_PARAM;
	}
	if(port >= g_board_cfg.port_number) {
		pr_err("not support port%d, max port %d\n", port, g_board_cfg.port_number - 1);
		x2_port = getenv("CAM_PORT");
		if(NULL != x2_port) {
			pr_info("force port %d to CAM_PORT %d\n", port, atoi(x2_port));
			port = atoi(x2_port);
			if(port >= g_board_cfg.port_number) {
					pr_err("not spport CAM_PORT %d, max port is %d \n", port,
						g_board_cfg.port_number - 1);
					return -HB_CAM_INVALID_PARAM;
			}
		} else {
			return -HB_CAM_INVALID_PARAM;
		}
	}
	if(subdev == SENSOR_DEVICE) {
	 	if(((sensor_module_t *)(g_board_cfg.sensor_info[port].sensor_ops))->spi_read == NULL) {
	 	  pr_err("sensor not suuport spi_block_read ops\n");
	 	  return -HB_CAM_OPS_NOT_SUPPORT;
	     }
	 	ret = ((sensor_module_t *)(g_board_cfg.sensor_info[port].sensor_ops))->spi_read(&g_board_cfg.sensor_info[port], reg_addr, buffer, size);
	 	if (ret < 0) {
	 		pr_err("spi_write fail\n");
	 		ret = -HB_CAM_SPI_READ_BLOCK_FAIL;
	 	}
	}
	if((subdev == EEPROM_DEVICE) || (subdev == IMU_DEVICE) || (subdev == ISP_DEVICE)) {
		pr_err("not support subdev type \n");
		return -HB_CAM_INVALID_PARAM;
	}
	return RET_OK;
}

int hb_cam_dynamic_switch(uint32_t port, uint32_t fps, uint32_t resolution)
{
 	int ret = RET_OK;
	char *x2_port = NULL;

	if(port >= g_board_cfg.port_number) {
		pr_err("not support port%d, max port %d\n", port, g_board_cfg.port_number - 1);
		x2_port = getenv("CAM_PORT");
		if(NULL != x2_port) {
			pr_info("force port %d to CAM_PORT %d\n", port, atoi(x2_port));
			port = atoi(x2_port);
			if(port >= g_board_cfg.port_number) {
					pr_err("not spport CAM_PORT %d, max port is %d \n", port,
						g_board_cfg.port_number - 1);
					return -HB_CAM_INVALID_PARAM;
			}
		} else {
			return -HB_CAM_INVALID_PARAM;
		}
	}
	if(g_board_cfg.sensor_info[port].start_state != CAM_STOP) {
		pr_err("need cam stop before cam_dynamic_switch");
		return -HB_CAM_DYNAMIC_SWITCH_FAIL;
	}
	ret = hb_cam_port_parse_cfg(port, fps, resolution);
	if(ret < 0) {
		pr_err("hb_port_parse_cfg port %d fail \n", port);
		return -HB_CAM_DYNAMIC_SWITCH_FAIL;
	}
	ret = ((sensor_module_t *)(g_board_cfg.sensor_info[port].sensor_ops))->init(&g_board_cfg.sensor_info[port]);
	if (ret < 0) {
		pr_err("sensor_init fail ret %d\n", ret);
		return -HB_CAM_DYNAMIC_SWITCH_FAIL;
	}
	ret = hb_vin_init(g_board_cfg.sensor_info[port].entry_num);
	if(ret < 0) {
		pr_err("hb_vin_init error! ret %d \n", ret);
		return -HB_CAM_DYNAMIC_SWITCH_FAIL;
	}

	return ret;
}

int hb_cam_dynamic_switch_fps(uint32_t port, uint32_t fps)
{
	int ret = RET_OK;
	char *x2_port = NULL;
	int time_ms = -1;
	struct timeval time_start = { 0 };
	struct timeval time_end = { 0 };

	if(port >= g_board_cfg.port_number) {
		pr_err("not support port%d, max port %d\n", port, g_board_cfg.port_number - 1);
		x2_port = getenv("CAM_PORT");
		if(NULL != x2_port) {
			pr_info("force port %d to CAM_PORT %d\n", port, atoi(x2_port));
			port = atoi(x2_port);
			if(port >= g_board_cfg.port_number) {
					pr_err("not spport CAM_PORT %d, max port is %d \n", port,
						g_board_cfg.port_number - 1);
					return -HB_CAM_INVALID_PARAM;
			}
		} else {
			return -HB_CAM_INVALID_PARAM;
		}
	}
	if(g_board_cfg.sensor_info[port].init_state != CAM_INIT) {
		pr_err("need cam init before switch_fps\n");
		return -HB_CAM_DYNAMIC_SWITCH_FPS_FAIL;
	}

	pr_info("hb_cam_dynamic_switch_fps begin\n");
	gettimeofday(&time_start, NULL);
	if(((sensor_module_t *)(g_board_cfg.sensor_info[port].sensor_ops))->dynamic_switch_fps == NULL) {
	 	  pr_err("sensor not suuport dynamic_switch_fps ops\n");
	 	  return -HB_CAM_OPS_NOT_SUPPORT;
	}
	ret = ((sensor_module_t *)(g_board_cfg.sensor_info[port].sensor_ops))->dynamic_switch_fps(&g_board_cfg.sensor_info[port], fps);
	if (ret < 0) {
		pr_err("dynamic_switch_fps fail ret %d\n", ret);
		return -HB_CAM_DYNAMIC_SWITCH_FPS_FAIL;
	}
	gettimeofday(&time_end, NULL);
	pr_info("hb_cam_dynamic_switch_fps end\n");
	time_ms = (int32_t)((time_end.tv_sec * 1000 + time_end.tv_usec /1000) -
		(time_start.tv_sec * 1000 + time_start.tv_usec /1000));
	pr_info("dynamic_switch_fps cost %d ms\n", time_ms);
	return ret;
}

int hb_cam_set_ex_gain(uint32_t port, uint32_t exposure_setting, uint32_t gain_setting_0, uint16_t gain_setting_1)
{
	int ret = RET_OK;
	char *x2_port = NULL;

	if(port >= g_board_cfg.port_number) {
		pr_err("not support port%d, max port %d\n", port, g_board_cfg.port_number - 1);
		x2_port = getenv("CAM_PORT");
		if(NULL != x2_port) {
			pr_info("force port %d to CAM_PORT %d\n", port, atoi(x2_port));
			port = atoi(x2_port);
			if(port >= g_board_cfg.port_number) {
					pr_err("not spport CAM_PORT %d, max port is %d \n", port,
						g_board_cfg.port_number - 1);
					return -HB_CAM_INVALID_PARAM;
			}
		} else {
			return -HB_CAM_INVALID_PARAM;
		}
	}
	if(((sensor_module_t *)(g_board_cfg.sensor_info[port].sensor_ops))->set_ex_gain == NULL) {
	 	  pr_err("sensor not suuport set_ex_gain ops\n");
	 	  return -HB_CAM_OPS_NOT_SUPPORT;
	}
	ret = ((sensor_module_t *)(g_board_cfg.sensor_info[port].sensor_ops))->set_ex_gain(g_board_cfg.sensor_info[port].bus_num, g_board_cfg.sensor_info[port].sensor_addr, exposure_setting, gain_setting_0, gain_setting_1);
	if (ret < 0) {
		pr_err("hb_cam_set_ex_gain fail ret %d\n", ret);
		return -HB_CAM_SET_EX_GAIN_FAIL;
	}
	return ret;
}

int hb_cam_set_awb_data(uint32_t port, AWB_DATA_s awb_data, float rg_gain, float b_gain)
{
	int ret = RET_OK;
	char *x2_port = NULL;

	if(port >= g_board_cfg.port_number) {
		pr_err("not support port%d, max port %d\n", port, g_board_cfg.port_number - 1);
		x2_port = getenv("CAM_PORT");
		if(NULL != x2_port) {
			pr_info("force port %d to CAM_PORT %d\n", port, atoi(x2_port));
			port = atoi(x2_port);
			if(port >= g_board_cfg.port_number) {
					pr_err("not spport CAM_PORT %d, max port is %d \n", port,
						g_board_cfg.port_number - 1);
					return -HB_CAM_INVALID_PARAM;
			}
		} else {
			return -HB_CAM_INVALID_PARAM;
		}
	}
	if(((sensor_module_t *)(g_board_cfg.sensor_info[port].sensor_ops))->set_awb == NULL) {
	 	pr_err("sensor not suuport set_awb ops\n");
	 	return -HB_CAM_OPS_NOT_SUPPORT;
	}
	ret = ((sensor_module_t *)(g_board_cfg.sensor_info[port].sensor_ops))->set_awb(g_board_cfg.sensor_info[port].bus_num, g_board_cfg.sensor_info[port].sensor_addr, rg_gain, b_gain);
	if (ret < 0) {
		pr_err("hb_cam_set_awb_data fail ret %d \n", ret);
		return -HB_CAM_SET_AWB_FAIL;
	}
	return ret;
}

int hb_cam_start_physical(uint32_t entry)
{
	int ret = RET_OK;

	if(entry > SERDES_NUMBER-1) {
		pr_err("invalid input param\n");
		return -HB_CAM_INVALID_PARAM;
	}
	ret = hb_deserial_start_physical(&g_board_cfg.deserial_info[entry]);
	if(ret < 0) {
		pr_err("hb_deserial_start_physical error!\n");
		return -HB_CAM_START_PHYSICAL_FAIL;
	}
	return ret;
}

int hb_cam_stop_all()
{
	int ret = RET_OK, i;

	if(g_board_cfg.sensor_info[0].init_state != CAM_INIT) {
		pr_err("need cam init before cam stop all\n");
		return -HB_CAM_OPS_NOT_SUPPORT;
	}
	for(i = 0; i < g_board_cfg.port_number; i++) {
		ret = hb_cam_stop(i);
		if(ret < 0) {
			pr_err("hb_cam_stop port_%d fail\n", i);
			ret = -HB_CAM_STOP_FAIL;
		}
	}
	return ret;
}

int hb_cam_start_all()
{
	int ret = RET_OK, i;

	if(g_board_cfg.sensor_info[0].init_state != CAM_INIT) {
		pr_err("need cam init before cam start all\n");
		return -HB_CAM_OPS_NOT_SUPPORT;
	}
	for(i = 0; i < g_board_cfg.port_number; i++) {
		ret = hb_cam_start(i);
		if(ret < 0) {
			pr_err("hb_cam_start port_%d fail\n", i);
			ret = -HB_CAM_START_FAIL;
		}
	}
	return ret;
}

int hb_cam_power_on(uint32_t port)
{
	int ret = RET_OK;
	char *x2_port = NULL;

	if(port >= g_board_cfg.port_number) {
		pr_err("not support port%d, max port %d\n", port, g_board_cfg.port_number - 1);
		x2_port = getenv("CAM_PORT");
		if(NULL != x2_port) {
			pr_info("force port %d to CAM_PORT %d\n", port, atoi(x2_port));
			port = atoi(x2_port);
			if(port >= g_board_cfg.port_number) {
					pr_err("not spport CAM_PORT %d, max port is %d \n", port,
						g_board_cfg.port_number - 1);
					return -HB_CAM_INVALID_PARAM;
			}
		} else {
			return -HB_CAM_INVALID_PARAM;
		}
	}
	if(g_board_cfg.sensor_info[port].sensor_ops == NULL) {
		pr_err("need cam init before cam power_on\n");
		return -HB_CAM_INVALID_PARAM;
	}
	if(((sensor_module_t *)(g_board_cfg.sensor_info[port].sensor_ops))->power_on == NULL) {
		pr_err("sensor not suuport power_on ops\n");
		return -HB_CAM_OPS_NOT_SUPPORT;
	}
	ret = ((sensor_module_t *)(g_board_cfg.sensor_info[port].sensor_ops))->power_on(&g_board_cfg.sensor_info[port]);
	if (ret < 0) {
		pr_err("hb_cam_power_on fail ret %d\n", ret);
		return -HB_CAM_SENSOR_POWERON_FAIL;
	}

	return ret;
}

int hb_cam_power_off(uint32_t port)
{
	int ret = RET_OK;
	char *x2_port = NULL;

	if(port >= g_board_cfg.port_number) {
		pr_err("not support port%d, max port %d\n", port, g_board_cfg.port_number - 1);
		x2_port = getenv("CAM_PORT");
		if(NULL != x2_port) {
			pr_info("force port %d to CAM_PORT %d\n", port, atoi(x2_port));
			port = atoi(x2_port);
			if(port >= g_board_cfg.port_number) {
					pr_err("not spport CAM_PORT %d, max port is %d \n", port,
						g_board_cfg.port_number - 1);
					return -HB_CAM_INVALID_PARAM;
			}
		} else {
			return -HB_CAM_INVALID_PARAM;
		}
	}
	if(g_board_cfg.sensor_info[port].sensor_ops == NULL) {
		pr_err("need cam init before cam power_off\n");
		return -HB_CAM_INVALID_PARAM;
	}
	if(((sensor_module_t *)(g_board_cfg.sensor_info[port].sensor_ops))->power_off == NULL) {
		pr_err("sensor not suuport power_off ops\n");
		return -HB_CAM_OPS_NOT_SUPPORT;
	}
	ret = ((sensor_module_t *)(g_board_cfg.sensor_info[port].sensor_ops))->power_off(&g_board_cfg.sensor_info[port]);
	if (ret < 0) {
		pr_err("hb_cam_power_off fail ret %d\n", ret);
		return -HB_CAM_SENSOR_POWEROFF_FAIL;
	}

	return ret;
}

int hb_cam_reset(uint32_t port)
{
	int ret = RET_OK;
	char *x2_port = NULL;

	if(port >= g_board_cfg.port_number) {
		pr_err("not support port%d, max port %d\n", port, g_board_cfg.port_number - 1);
		x2_port = getenv("CAM_PORT");
		if(NULL != x2_port) {
			pr_info("force port %d to CAM_PORT %d\n", port, atoi(x2_port));
			port = atoi(x2_port);
			if(port >= g_board_cfg.port_number) {
					pr_err("not spport CAM_PORT %d, max port is %d \n", port,
						g_board_cfg.port_number - 1);
					return -HB_CAM_INVALID_PARAM;
			}
		} else {
			return -HB_CAM_INVALID_PARAM;
		}
	}
	if(g_board_cfg.sensor_info[port].init_state != CAM_INIT) {
		pr_err("need cam init before cam reset\n");
		return -HB_CAM_RESET_FAIL;
	}
	if(g_board_cfg.sensor_info[port].sensor_ops == NULL) {
		pr_err("need cam init before cam reset\n");
		return -HB_CAM_INVALID_PARAM;
	}
	if(((sensor_module_t *)(g_board_cfg.sensor_info[port].sensor_ops))->power_reset == NULL) {
		pr_err("sensor not suuport power_reset ops\n");
		return -HB_CAM_OPS_NOT_SUPPORT;
	}
	ret = ((sensor_module_t *)(g_board_cfg.sensor_info[port].sensor_ops))->power_reset(&g_board_cfg.sensor_info[port]);
	if (ret < 0) {
		pr_err("hb_cam_power_on fail ret %d \n", ret);
		return -HB_CAM_RESET_FAIL;
	}
	return ret;
}

int hb_cam_extern_isp_poweron(uint32_t port)
{
	int ret = RET_OK;
	char *x2_port = NULL;

	if(port >= g_board_cfg.port_number) {
		pr_err("not support port%d, max port %d\n", port, g_board_cfg.port_number - 1);
		x2_port = getenv("CAM_PORT");
		if(NULL != x2_port) {
			pr_info("force port %d to CAM_PORT %d\n", port, atoi(x2_port));
			port = atoi(x2_port);
			if(port >= g_board_cfg.port_number) {
					pr_err("not spport CAM_PORT %d, max port is %d \n", port,
						g_board_cfg.port_number - 1);
					return -HB_CAM_INVALID_PARAM;
			}
		} else {
			return -HB_CAM_INVALID_PARAM;
		}
	}
	if(g_board_cfg.sensor_info[port].sensor_ops == NULL) {
		return -HB_CAM_INVALID_PARAM;
	}
	if(((sensor_module_t *)(g_board_cfg.sensor_info[port].sensor_ops))->extern_isp_poweron == NULL) {
		pr_err("sensor not suuport extern_isp_poweron ops\n");
		return -HB_CAM_OPS_NOT_SUPPORT;
	}
	ret = ((sensor_module_t *)(g_board_cfg.sensor_info[port].sensor_ops))->extern_isp_poweron(&g_board_cfg.sensor_info[port]);
	if (ret < 0) {
		pr_err("hb_cam_external_isp_poweron fail ret %d\n", ret);
		return -HB_CAM_ISP_POWERON_FAIL;
	}

	return ret;
}

int hb_cam_extern_isp_poweroff(uint32_t port)
{
	int ret = RET_OK;
	char *x2_port = NULL;

	if(port >= g_board_cfg.port_number) {
		pr_err("not support port%d, max port %d\n", port, g_board_cfg.port_number - 1);
		x2_port = getenv("CAM_PORT");
		if(NULL != x2_port) {
			pr_info("force port %d to CAM_PORT %d\n", port, atoi(x2_port));
			port = atoi(x2_port);
			if(port >= g_board_cfg.port_number) {
					pr_err("not spport CAM_PORT %d, max port is %d \n", port,
						g_board_cfg.port_number - 1);
					return -HB_CAM_INVALID_PARAM;
			}
		} else {
			return -HB_CAM_INVALID_PARAM;
		}
	}
	if(g_board_cfg.sensor_info[port].sensor_ops == NULL) {
		return -HB_CAM_INVALID_PARAM;
	}
	if(((sensor_module_t *)(g_board_cfg.sensor_info[port].sensor_ops))->power_off == NULL) {
		pr_err("sensor not suuport extern_isp_poweroff ops\n");
		return -HB_CAM_OPS_NOT_SUPPORT;
	}
	ret = ((sensor_module_t *)(g_board_cfg.sensor_info[port].sensor_ops))->extern_isp_poweroff(&g_board_cfg.sensor_info[port]);
	if (ret < 0) {
		pr_err("hb_cam_external_isp_poweroff fail ret %d\n", ret);
		return -HB_CAM_ISP_POWEROFF_FAIL;
	}

	return ret;
}

int hb_cam_extern_isp_reset(uint32_t port)
{
	int ret = RET_OK;
	char *x2_port = NULL;

	if(port >= g_board_cfg.port_number) {
		pr_err("not support port%d, max port %d\n", port, g_board_cfg.port_number - 1);
		x2_port = getenv("CAM_PORT");
		if(NULL != x2_port) {
			pr_info("force port %d to CAM_PORT %d\n", port, atoi(x2_port));
			port = atoi(x2_port);
			if(port >= g_board_cfg.port_number) {
					pr_err("not spport CAM_PORT %d, max port is %d \n", port,
						g_board_cfg.port_number - 1);
					return -HB_CAM_INVALID_PARAM;
			}
		} else {
			return -HB_CAM_INVALID_PARAM;
		}
	}
	if(g_board_cfg.sensor_info[port].sensor_ops == NULL) {
		return -HB_CAM_INVALID_PARAM;
	}
	if(((sensor_module_t *)(g_board_cfg.sensor_info[port].sensor_ops))->extern_isp_reset == NULL) {
		pr_err("sensor not suuport extern_isp_reset ops\n");
		return -HB_CAM_OPS_NOT_SUPPORT;
	}
	ret = ((sensor_module_t *)(g_board_cfg.sensor_info[port].sensor_ops))->extern_isp_reset(&g_board_cfg.sensor_info[port]);
	if (ret < 0) {
		pr_err("hb_cam_external_isp_reset fail ret %d \n", ret);
		return -HB_CAM_ISP_RESET_FAIL;
	}
	return ret;
}

int hb_cam_get_sns_info(uint32_t dev_port, cam_parameter_t *sp, uint8_t type)
{
	int i = 0;
	int ret = 0;
	sensor_info_t *si;

	if (dev_port >= CAM_MAX_NUM) {
		pr_err("port %d is invalid.\n", dev_port);
		return -1;
	}
	if (sp == NULL) {
		return -HB_CAM_INVALID_PARAM;
	}
	for (i = 0; i < CAM_MAX_NUM; i++) {
		si = &g_board_cfg.sensor_info[i];
		if (si->init_state == CAM_INIT && si->dev_port == dev_port) {
			if (((sensor_module_t *)(si->sensor_ops))->get_sns_params) {
				ret = ((sensor_module_t *)(si->sensor_ops))->get_sns_params(si, sp, type);
				if (ret < 0) {
					pr_err("get sns params fail ret %d\n", ret);
					return -HB_CAM_INVALID_PARAM;
				}
			}
		}
	}

	return 0;
}

// PRQA S ALL --
