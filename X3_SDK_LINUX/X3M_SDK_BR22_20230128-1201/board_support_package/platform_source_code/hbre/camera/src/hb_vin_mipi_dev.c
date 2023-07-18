/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <linux/netlink.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/poll.h>
#include <linux/socket.h>
#include "logging.h"
#include "cJSON.h"
#include "inc/hb_vin_common.h"
#include "inc/hb_vin_mipi_dev.h"

#define MIPIDEVIOC_MAGIC 'v'
#define MIPIDEVIOC_INIT             _IOW(MIPIDEVIOC_MAGIC, 0, mipi_dev_cfg_t)
#define MIPIDEVIOC_DEINIT           _IO(MIPIDEVIOC_MAGIC,  1)
#define MIPIDEVIOC_START            _IO(MIPIDEVIOC_MAGIC,  2)
#define MIPIDEVIOC_STOP             _IO(MIPIDEVIOC_MAGIC,  3)
#define MIPIDEVIOC_IPI_GET_INFO     _IOR(MIPIDEVIOC_MAGIC, 4, mipi_dev_ipi_info_t)
#define MIPIDEVIOC_IPI_SET_INFO     _IOW(MIPIDEVIOC_MAGIC, 5, mipi_dev_ipi_info_t)

int hb_vin_mipi_dev_parser_config(void *root, entry_t *e)
{
	cJSON       *base = NULL;
	cJSON       *dev = NULL;
	cJSON       *dev_param = NULL;
	mipi_dev_cfg_t *cfg;

	if (!e) {
		pr_err("entry error\n");
		return -HB_VIN_MIPI_DEV_PARSER_FAIL;
	}
	cfg = &e->mipi_dev_cfg;
	dev = cJSON_GetObjectItem(root, "dev");
	if (NULL == dev) {
		pr_err("no dev cfg node found\n");
		return -HB_VIN_MIPI_DEV_PARSER_FAIL;
	} else {
		cJSON       *node = NULL;
		node = cJSON_GetObjectItem(dev, "enable");
		if (NULL != node) {
			if (0 == node->valueint) {
				e->dev_enable = 0;
				pr_warn("dev not enable\n");
				return RET_OK;
			} else {
				e->dev_enable = node->valueint;
			}
		} else {
			pr_err("dev enable cfg not found\n");
			return -HB_VIN_MIPI_DEV_PARSER_FAIL;
		}
		node = cJSON_GetObjectItem(dev, "vpg");
		if (NULL != node)
			cfg->vpg = (uint16_t)node->valueint;
		node = cJSON_GetObjectItem(dev, "ipi_lines");
		if (NULL != node)
			cfg->ipi_lines = (uint16_t)node->valueint;
		node = cJSON_GetObjectItem(dev, "vc_num");
		if (NULL != node)
			cfg->channel_num = (uint16_t)node->valueint;
		node = cJSON_GetObjectItem(dev, "vc0_index");
		if ((NULL != node) && (node->valueint < MIPIDEV_CHANNEL_NUM))
			cfg->channel_sel[node->valueint] = MIPIDEV_CHANNEL_0;
		node = cJSON_GetObjectItem(dev, "vc1_index");
		if ((NULL != node) && (node->valueint < MIPIDEV_CHANNEL_NUM))
			cfg->channel_sel[node->valueint] = MIPIDEV_CHANNEL_1;
		node = cJSON_GetObjectItem(dev, "vc2_index");
		if ((NULL != node) && (node->valueint < MIPIDEV_CHANNEL_NUM))
			cfg->channel_sel[node->valueint] = MIPIDEV_CHANNEL_2;
		node = cJSON_GetObjectItem(dev, "vc3_index");
		if ((NULL != node) && (node->valueint < MIPIDEV_CHANNEL_NUM))
			cfg->channel_sel[node->valueint] = MIPIDEV_CHANNEL_3;
	}
	base = cJSON_GetObjectItem((cJSON *)dev, "dev_base");
	if (NULL == base)
		base = cJSON_GetObjectItem((cJSON *)root, "base");
	if (NULL == base) {
		pr_err("no base cfg node found\n");
		return -HB_VIN_MIPI_DEV_PARSER_FAIL;
	} else {
		cJSON       *node = NULL;
		node = cJSON_GetObjectItem(base, "lane");
		if (NULL != node)
			cfg->lane = (uint16_t)node->valueint;
		node = cJSON_GetObjectItem(base, "datatype");
		if (NULL != node)
			cfg->datatype = (uint16_t)strtoul(node->valuestring, NULL, 0);
		node = cJSON_GetObjectItem(base, "mclk");
		if (NULL != node)
			cfg->mclk = (uint16_t)node->valueint;
		node = cJSON_GetObjectItem(base, "mipiclk");
		if (NULL != node)
			cfg->mipiclk = (uint16_t)node->valueint;
		node = cJSON_GetObjectItem(base, "fps");
		if (NULL != node)
			cfg->fps = (uint16_t)node->valueint;
		node = cJSON_GetObjectItem(base, "width");
		if (NULL != node)
			cfg->width = (uint16_t)node->valueint;
		node = cJSON_GetObjectItem(base, "height");
		if (NULL != node)
			cfg->height = (uint16_t)node->valueint;
		node = cJSON_GetObjectItem(base, "linelenth");
		if (NULL != node)
			cfg->linelenth = (uint16_t)node->valueint;
		node = cJSON_GetObjectItem(base, "framelenth");
		if (NULL != node)
			cfg->framelenth = (uint16_t)node->valueint;
		node = cJSON_GetObjectItem(base, "settle");
		if (NULL != node)
			cfg->settle = (uint16_t)node->valueint;
	}
	memset(&e->dev_params, 0, sizeof(e->dev_params));
	dev_param = cJSON_GetObjectItem(root, "param");
	if (NULL != dev_param)
		dev_param = cJSON_GetObjectItem(dev_param, "dev");
	if (NULL != dev_param) {
		cJSON       *node = NULL, *arrayitem = NULL;
		int         array_size = 0, array_index;
		node = cJSON_GetObjectItem(dev_param, "name");
		if (NULL != node) {
			array_size = cJSON_GetArraySize(node);
			if (array_size > MIPI_PARAM_MAX) {
				pr_info("mipi dev param name overflow %d\n", array_size);
				array_size = MIPI_PARAM_MAX;
			}
			for(array_index = 0; array_index < array_size; array_index++) {
				arrayitem = cJSON_GetArrayItem(node, array_index);
				strncpy(e->dev_params[array_index].name,
					arrayitem->valuestring, MIPI_PARAM_NAME_LEN);
			}
		}
		node = cJSON_GetObjectItem(dev_param, "value");
		if (NULL != node) {
			array_size = cJSON_GetArraySize(node);
			if (array_size > MIPI_PARAM_MAX) {
				pr_info("mipi dev param value overflow %d\n", array_size);
				array_size = MIPI_PARAM_MAX;
			}
			for(array_index = 0; array_index < array_size; array_index++) {
				arrayitem = cJSON_GetArrayItem(node, array_index);
				e->dev_params[array_index].value = arrayitem->valueint;
			}
		}
	}
	return 0;
}

int hb_vin_mipi_dev_start(entry_t *e)
{
	int ret = RET_OK;

	if(!e)
		return RET_ERROR;
	pr_info("mipi dev%d start begin\n", e->dev_enable - 1);
	ret = ioctl(e->dev_fd, MIPIDEVIOC_START, 0);
	if (ret < 0) {
		pr_err("!!! dev%d MIPIDEVIOC_START error, ret = %d\n",
			e->dev_enable - 1, ret);
		return -HB_VIN_MIPI_DEV_START_FAIL;
	}
	pr_info("mipi dev%d start end\n", e->dev_enable - 1);
	return ret;
}

int hb_vin_mipi_dev_stop(entry_t *e)
{
	int ret = RET_OK;

	if(!e)
		return -HB_VIN_MIPI_DEV_STOP_FAIL;
	pr_info("mipi dev%d stop begin\n", e->dev_enable - 1);
	ret = ioctl(e->dev_fd, MIPIDEVIOC_STOP, 0);
	if (ret < 0) {
		pr_err("!!! dev%d MIPIDEVIOC_STOP error, ret = %d\n",
			e->dev_enable - 1, ret);
		return -HB_VIN_MIPI_DEV_STOP_FAIL;
	}
	pr_info("mipi dev%d stop end\n", e->dev_enable - 1);
	return ret;
}

static char* hb_vin_mipi_dev_path(entry_t *e)
{
	if (e->dev_path[0] == '\0')
		snprintf(e->dev_path, sizeof(e->dev_path), "%s%d",
			HB_VIN_MIPI_DEV_PATH, (e->dev_enable - 1));
	return e->dev_path;
}

static int hb_vin_mipi_dev_params(entry_t *e)
{
	FILE *pf_sys;
	mipi_param_t *param;
	char param_sys_path[128];
	char *dev_name;
	int i;

	dev_name = strrchr(hb_vin_mipi_dev_path(e), '/');
	if (dev_name == NULL)
		return -RET_ERROR;

	for (i = 0; i < MIPI_PARAM_MAX; i++) {
		param = &e->dev_params[i];
		if (param->name[0] == '\0')
			break;
		if (param->name[0] == '#')
			continue;
		snprintf(param_sys_path, sizeof(param_sys_path),
			MIPI_SYSFS_PATH_PRE "%s/" MIPI_SYSFS_DIR_PARAM "/%s",
			dev_name, param->name);
		pf_sys = fopen(param_sys_path, "r+");
		if (pf_sys == NULL) {
			pr_err("fopen %s error\n", param_sys_path);
			return -RET_ERROR;
		}
		if (fprintf(pf_sys, "%d\n", param->value) <= 0) {
			pr_err("set %s %d error\n", param_sys_path, param->value);
			fclose(pf_sys);
			return -RET_ERROR;
		}
		pr_debug("set %s %d\n", param_sys_path, param->value);
		fclose(pf_sys);
	}

	return RET_OK;
}

int hb_vin_mipi_dev_init(entry_t *e)
{
	int ret = RET_OK;
	mipi_dev_cfg_t *cfg;

	if(!e)
		return -HB_VIN_MIPI_DEV_INIT_FAIL;
	pr_info("mipi dev%d init begin\n", e->dev_enable - 1);
	ret = hb_vin_mipi_dev_params(e);
	if (ret < 0) {
		pr_err("!!!init params error, ret = %d\n", ret);
		return -HB_VIN_MIPI_HOST_INIT_FAIL;
	}
	cfg = &e->mipi_dev_cfg;
	if(e->dev_fd > 0) {
		pr_info("mipi dev%d have been open\n", e->dev_enable - 1);
	}else{
		e->dev_fd = open(hb_vin_mipi_dev_path(e), O_RDWR | O_CLOEXEC);
		if(e->dev_fd < 0) {
			pr_err("!!!Can't open %s, ret = %d\n", hb_vin_mipi_dev_path(e), e->dev_fd);
			return -HB_VIN_MIPI_DEV_INIT_FAIL;
		}
	}
	ret = ioctl(e->dev_fd, MIPIDEVIOC_INIT, cfg);
	if (ret < 0) {
		pr_err("!!! dev%d MIPIDEVIOC_INIT error, ret = %d\n",
			e->dev_enable - 1, ret);
		close(e->dev_fd);
		e->dev_fd = 0;
		return -HB_VIN_MIPI_DEV_INIT_FAIL;
	}
	pr_info("mipi dev%d init end\n", e->dev_enable - 1);
	return ret;
}

int hb_vin_mipi_dev_deinit(entry_t *e)
{
	if(!e)
		return -RET_ERROR;
	int ret = RET_OK;

	pr_info("mipi dev%d deinit begin\n", e->dev_enable - 1);
	if(e->dev_fd > 0) {
		ret = ioctl(e->dev_fd, MIPIDEVIOC_DEINIT, 0);
		if (ret < 0) {
			pr_err("ioctl MIPIDEVIOC_DEINIT is error! ret = %d\n", ret);
		}
		close(e->dev_fd);
		e->dev_fd = 0;
	}
	pr_info("mipi dev%d deinit end\n", e->dev_enable - 1);
	return RET_OK;
}

int hb_vin_mipi_dev_ipi_fatal(entry_t *e, int32_t ipi)
{
	int ret = RET_OK;
	int fd = -1;
	mipi_dev_ipi_info_t ipi_info;

	if(!e)
		return -HB_VIN_MIPI_DEV_IPI_FATAL_FAIL;
	memset(&ipi_info, 0, sizeof(ipi_info));
	ipi_info.index = (uint16_t)ipi;
	if (e->dev_fd > 0) {
		ret = ioctl(e->dev_fd, MIPIDEVIOC_IPI_GET_INFO, &ipi_info);
	} else {
		fd = open(hb_vin_mipi_dev_path(e), O_RDWR | O_CLOEXEC);
		if (fd >= 0) {
			ret = ioctl(e->dev_fd, MIPIDEVIOC_IPI_GET_INFO, &ipi_info);
			close(fd);
		} else {
			ret = -1;
		}
	}
	if (ret < 0) {
		pr_err("!!! dev%d MIPIDEVIOC_IPI_GET_INFO error, ret = %d\n",
			e->dev_enable - 1, ret);
		return -HB_VIN_MIPI_DEV_IPI_FATAL_FAIL;
	}
	return (int)(ipi_info.fatal);
}

int hb_vin_mipi_dev_ipi_get_info(entry_t *e, int32_t ipi, mipi_dev_ipi_info_t *info)
{
	int ret = RET_OK;
	int fd = -1;
	mipi_dev_ipi_info_t ipi_info;

	if(!e || !info)
		return -HB_VIN_MIPI_DEV_IPI_GET_INFO_FAIL;
	memset(&ipi_info, 0, sizeof(ipi_info));
	ipi_info.index = (uint16_t)ipi;
	if (e->dev_fd > 0) {
		ret = ioctl(e->dev_fd, MIPIDEVIOC_IPI_GET_INFO, &ipi_info);
	} else {
		fd = open(hb_vin_mipi_dev_path(e), O_RDWR | O_CLOEXEC);
		if (fd >= 0) {
			ret = ioctl(e->dev_fd, MIPIDEVIOC_IPI_GET_INFO, &ipi_info);
			close(fd);
		} else {
			ret = -1;
		}
	}
	if (ret < 0) {
		pr_err("!!! dev%d MIPIDEVIOC_IPI_GET_INFO error, ret = %d\n",
			e->dev_enable - 1, ret);
		return -HB_VIN_MIPI_DEV_IPI_GET_INFO_FAIL;
	}
	memcpy(info, &ipi_info, sizeof(ipi_info));
	return ret;
}

int hb_vin_mipi_dev_ipi_set_info(entry_t *e, int32_t ipi, mipi_dev_ipi_info_t *info)
{
	int ret = RET_OK;
	int fd = -1;
	mipi_dev_ipi_info_t ipi_info;

	if(!e || !info)
		return -HB_VIN_MIPI_DEV_IPI_SET_INFO_FAIL;
	pr_info("mipi dev%d set %d:ipi%d as mode=0x%x,vc=0x%x,dt=0x%x,maxfnum=%d,pixels=%d,lines=%d\n",
		e->dev_enable - 1, ipi, ipi + 1, info->mode, info->vc, info->datatype,
		info->maxfnum, info->pixels, info->lines);
	memcpy(&ipi_info, info, sizeof(ipi_info));
	ipi_info.index = (uint16_t)ipi;
	if (e->dev_fd > 0) {
		ret = ioctl(e->dev_fd, MIPIDEVIOC_IPI_SET_INFO, &ipi_info);
	} else {
		fd = open(hb_vin_mipi_dev_path(e), O_RDWR | O_CLOEXEC);
		if (fd >= 0) {
			ret = ioctl(e->dev_fd, MIPIDEVIOC_IPI_SET_INFO, &ipi_info);
			close(fd);
		} else {
			ret = -1;
		}
	}
	if (ret < 0) {
		pr_err("!!! dev%d MIPIDEVIOC_IPI_SET_INFO error, ret = %d\n",
			e->dev_enable - 1, ret);
		return -HB_VIN_MIPI_DEV_IPI_SET_INFO_FAIL;
	}
	pr_info("mipi dev%d set %d:ipi%d end\n", e->dev_enable - 1, ipi, ipi + 1);
	return ret;
}
