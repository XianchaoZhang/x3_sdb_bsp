/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2016 Horizon Robotics, Inc.
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
#include "inc/hb_vin_mipi_host.h"

#define MIPIHOSTIOC_MAGIC 'v'
#define MIPIHOSTIOC_INIT             _IOW(MIPIHOSTIOC_MAGIC, 0, mipi_host_cfg_t)
#define MIPIHOSTIOC_DEINIT           _IO(MIPIHOSTIOC_MAGIC,  1)
#define MIPIHOSTIOC_START            _IO(MIPIHOSTIOC_MAGIC,  2)
#define MIPIHOSTIOC_STOP             _IO(MIPIHOSTIOC_MAGIC,  3)
#define MIPIHOSTIOC_SNRCLK_SET_EN    _IOW(MIPIHOSTIOC_MAGIC, 4, uint32_t)
#define MIPIHOSTIOC_SNRCLK_SET_FREQ  _IOW(MIPIHOSTIOC_MAGIC, 5, uint32_t)
#define MIPIHOSTIOC_PRE_INIT_REQUEST _IOW(MIPIHOSTIOC_MAGIC, 6, uint32_t)
#define MIPIHOSTIOC_PRE_START_REQUEST _IOW(MIPIHOSTIOC_MAGIC, 7, uint32_t)
#define MIPIHOSTIOC_PRE_INIT_RESULT  _IOW(MIPIHOSTIOC_MAGIC, 8, uint32_t)
#define MIPIHOSTIOC_PRE_START_RESULT _IOW(MIPIHOSTIOC_MAGIC, 9, uint32_t)
#define MIPIHOSTIOC_IPI_RESET        _IOW(MIPIHOSTIOC_MAGIC, 10, mipi_host_ipi_reset_t)
#define MIPIHOSTIOC_IPI_GET_INFO     _IOR(MIPIHOSTIOC_MAGIC, 11, mipi_host_ipi_info_t)
#define MIPIHOSTIOC_IPI_SET_INFO     _IOW(MIPIHOSTIOC_MAGIC, 12, mipi_host_ipi_info_t)

int hb_vin_mipi_host_parser_config(void *root, entry_t *e)
{
	cJSON       *base = NULL;
	cJSON       *host = NULL;
	cJSON       *host_param = NULL;
	mipi_host_cfg_t *cfg;

	if (!e) {
		pr_err("entry error\n");
		return -HB_VIN_MIPI_DEV_PARSER_FAIL;
	}
	cfg = &e->mipi_host_cfg;
	host = cJSON_GetObjectItem(root, "host");
	if (NULL == host) {
		pr_err("no host cfg node found\n");
		return -HB_VIN_MIPI_HOST_PARSER_FAIL;
	} else {
		cJSON       *node = NULL;
		node = cJSON_GetObjectItem(host, "enable");
		if (NULL != node) {
			if (0 == node->valueint) {
				e->host_enable = 0;
				pr_warn("host not enable\n");
				return RET_OK;
			} else {
				e->host_enable = node->valueint;
			}
		} else {
			pr_err("host enable cfg not found\n");
			return -HB_VIN_MIPI_HOST_PARSER_FAIL;
		}
		node = cJSON_GetObjectItem(host, "hsa");
		if (NULL != node)
			cfg->hsaTime = (uint16_t)node->valueint;
		node = cJSON_GetObjectItem(host, "hbp");
		if (NULL != node)
			cfg->hbpTime = (uint16_t)node->valueint;
		node = cJSON_GetObjectItem(host, "hsd");
		if (NULL != node)
			cfg->hsdTime = (uint16_t)node->valueint;
		node = cJSON_GetObjectItem(host, "vc_num");
		if (NULL != node)
			cfg->channel_num = (uint16_t)node->valueint;
		node = cJSON_GetObjectItem(host, "vc0_index");
		if ((NULL != node) && (node->valueint < MIPIHOST_CHANNEL_NUM))
			cfg->channel_sel[node->valueint] = MIPIHOST_CHANNEL_0;
		node = cJSON_GetObjectItem(host, "vc1_index");
		if ((NULL != node) && (node->valueint < MIPIHOST_CHANNEL_NUM))
			cfg->channel_sel[node->valueint] = MIPIHOST_CHANNEL_1;
		node = cJSON_GetObjectItem(host, "vc2_index");
		if ((NULL != node) && (node->valueint < MIPIHOST_CHANNEL_NUM))
			cfg->channel_sel[node->valueint] = MIPIHOST_CHANNEL_2;
		node = cJSON_GetObjectItem(host, "vc3_index");
		if ((NULL != node) && (node->valueint < MIPIHOST_CHANNEL_NUM))
			cfg->channel_sel[node->valueint] = MIPIHOST_CHANNEL_3;
	}
	base = cJSON_GetObjectItem((cJSON *)root, "base");
	if (NULL == base) {
		pr_err("no base cfg node found\n");
		return -HB_VIN_MIPI_HOST_PARSER_FAIL;
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
	memset(&e->host_params, 0, sizeof(e->host_params));
	host_param = cJSON_GetObjectItem(root, "param");
	if (NULL != host_param)
		host_param = cJSON_GetObjectItem(host_param, "host");
	if (NULL != host_param) {
		cJSON       *node = NULL, *arrayitem = NULL;
		int         array_size = 0, array_index;
		node = cJSON_GetObjectItem(host_param, "name");
		if (NULL != node) {
			array_size = cJSON_GetArraySize(node);
			if (array_size > MIPI_PARAM_MAX) {
				pr_info("mipi host param name overflow %d\n", array_size);
				array_size = MIPI_PARAM_MAX;
			}
			for(array_index = 0; array_index < array_size; array_index++) {
				arrayitem = cJSON_GetArrayItem(node, array_index);
				strncpy(e->host_params[array_index].name,
					arrayitem->valuestring, MIPI_PARAM_NAME_LEN);
			}
		}
		node = cJSON_GetObjectItem(host_param, "value");
		if (NULL != node) {
			array_size = cJSON_GetArraySize(node);
			if (array_size > MIPI_PARAM_MAX) {
				pr_info("mipi host param value overflow %d\n", array_size);
				array_size = MIPI_PARAM_MAX;
			}
			for(array_index = 0; array_index < array_size; array_index++) {
				arrayitem = cJSON_GetArrayItem(node, array_index);
				e->host_params[array_index].value = arrayitem->valueint;
			}
		}
	}
	return 0;
}

int hb_vin_mipi_host_start(entry_t *e)
{
	int ret = RET_OK;

	if(!e)
		return -HB_VIN_MIPI_HOST_START_FAIL;
	pr_info("mipi host%d start begin\n", e->entry_num);
	ret = ioctl(e->host_fd, MIPIHOSTIOC_START, 0);
	if (ret < 0) {
		pr_err("!!! host%d MIPIHOSTIOC_START error, ret = %d\n",
			e->entry_num, ret);
		return -HB_VIN_MIPI_HOST_START_FAIL;
	}
	pr_info("mipi host%d start end\n", e->entry_num);
	return ret;
}

int hb_vin_mipi_host_stop(entry_t *e)
{
	int ret = RET_OK;

	if(!e)
		return -HB_VIN_MIPI_HOST_STOP_FAIL;
	pr_info("mipi host%d stop begin\n", e->entry_num);
	ret = ioctl(e->host_fd, MIPIHOSTIOC_STOP, 0);
	if (ret < 0) {
		pr_err("!!! host%d MIPIHOSTIOC_STOP error, ret = %d\n",
			e->entry_num, ret);
		return -HB_VIN_MIPI_HOST_STOP_FAIL;
	}
	pr_info("mipi host%d stop end\n", e->entry_num);
	return ret;
}

static char* hb_vin_mipi_host_path(entry_t *e)
{
	if (e->host_path[0] == '\0')
		snprintf(e->host_path, sizeof(e->host_path), "%s%d",
			HB_VIN_MIPI_HOST_PATH, e->entry_num);
	return e->host_path;
}

static int hb_vin_mipi_host_params(entry_t *e)
{
	FILE *pf_sys;
	mipi_param_t *param;
	char param_sys_path[128];
	char *dev_name;
	int i;

	dev_name = strrchr(hb_vin_mipi_host_path(e), '/');
	if (dev_name == NULL)
		return -RET_ERROR;

	for (i = 0; i < MIPI_PARAM_MAX; i++) {
		param = &e->host_params[i];
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
			pr_err("set %s param %s %d error\n", dev_name, param->name, param->value);
			fclose(pf_sys);
			return -RET_ERROR;
		}
		pr_debug("set %s %d\n", param_sys_path, param->value);
		fclose(pf_sys);
	}

	return RET_OK;
}

int hb_vin_mipi_host_open(entry_t *e)
{
	int ret = RET_OK;

	if (e->host_fd <= 0)
		e->host_fd = open(hb_vin_mipi_host_path(e), O_RDWR | O_CLOEXEC);
	if (e->host_fd < 0) {
		pr_err("!!!Can't open %s, ret = %d\n",
			hb_vin_mipi_host_path(e), e->host_fd);
		return -HB_VIN_MIPI_HOST_OPEN_FAIL;
	}
	return ret;
}

int hb_vin_mipi_host_init(entry_t *e)
{
	int ret = RET_OK;
	mipi_host_cfg_t *cfg;

	if(!e)
		return -HB_VIN_MIPI_HOST_INIT_FAIL;
	pr_info("mipi host%d init begin\n", e->entry_num);
	ret = hb_vin_mipi_host_params(e);
	if (ret < 0) {
		pr_err("!!!init params error, ret = %d\n", ret);
		return -HB_VIN_MIPI_HOST_INIT_FAIL;
	}
	cfg = &e->mipi_host_cfg;
	if (e->host_fd <= 0)
		e->host_fd = open(hb_vin_mipi_host_path(e), O_RDWR | O_CLOEXEC);
	if (e->host_fd < 0) {
		pr_err("!!!Can't open %s, ret = %d\n",
			hb_vin_mipi_host_path(e), e->host_fd);
		return -HB_VIN_MIPI_HOST_INIT_FAIL;
	}
	ret = ioctl(e->host_fd, MIPIHOSTIOC_INIT, cfg);
	if (ret < 0) {
		pr_err("!!! host%d MIPIHOSTIOC_INIT error, ret = %d\n",
			e->entry_num, ret);
		close(e->host_fd);
		e->host_fd = 0;
		return -HB_VIN_MIPI_HOST_INIT_FAIL;
	}
	pr_info("mipi host%d init end\n", e->entry_num);
	return ret;
}

int hb_vin_mipi_host_deinit(entry_t *e)
{
	if(!e)
		return -RET_ERROR;
	int ret = RET_OK;

	pr_info("mipi host%d deinit begin\n", e->entry_num);
	if(e->host_fd > 0) {
		ret = ioctl(e->host_fd, MIPIHOSTIOC_DEINIT, 0);
		if (ret < 0) {
			pr_err("ioctl MIPIDEVIOC_DEINIT is error! ret = %d\n", ret);
		}
		close(e->host_fd);
		e->host_fd = 0;
	}
	pr_info("mipi host%d deinit end\n", e->entry_num);
	return RET_OK;
}

int hb_vin_mipi_host_snrclk_set_en(entry_t *e, uint32_t enable)
{
	int ret = RET_OK;
	int fd = -1;

	if(!e)
		return -HB_VIN_MIPI_HOST_SNRCLK_SET_EN_FAIL;
	pr_info("mipi host%d snrclk set %s begin\n", e->entry_num,
		(enable) ? "enable" : "disable");
	if (e->host_fd > 0) {
		ret = ioctl(e->host_fd, MIPIHOSTIOC_SNRCLK_SET_EN, &enable);
	} else {
		fd = open(hb_vin_mipi_host_path(e), O_RDWR | O_CLOEXEC);
		if (fd >= 0) {
			ret = ioctl(fd, MIPIHOSTIOC_SNRCLK_SET_EN, &enable);
			if (enable && e->host_fd <= 0) {
				e->host_fd = fd;
			} else {
				close(fd);
			}
		} else {
			ret = -1;
		}
	}
	if (ret < 0) {
		pr_err("!!! host%d MIPIHOSTIOC_SNRCLK_SET_EN error, ret = %d\n",
			e->entry_num, ret);
		return -HB_VIN_MIPI_HOST_SNRCLK_SET_EN_FAIL;
	}
	pr_info("mipi host%d snrclk set %s end\n", e->entry_num,
		(enable) ? "enable" : "disable");
	return ret;
}

int hb_vin_mipi_host_snrclk_set_freq(entry_t *e, uint32_t freq)
{
	int ret = RET_OK;
	int fd = -1;

	if(!e)
		return -HB_VIN_MIPI_HOST_SNRCLK_SET_FREQ_FAIL;
	pr_info("mipi host%d snrclk set %u begin\n", e->entry_num, freq);
	if (e->host_fd > 0) {
		ret = ioctl(e->host_fd, MIPIHOSTIOC_SNRCLK_SET_FREQ, &freq);
	} else {
		fd = open(hb_vin_mipi_host_path(e), O_RDWR | O_CLOEXEC);
		if (fd >= 0) {
			ret = ioctl(fd, MIPIHOSTIOC_SNRCLK_SET_FREQ, &freq);
			if (freq && e->host_fd <= 0) {
				e->host_fd = fd;
			} else {
				close(fd);
			}
		} else {
			ret = -1;
		}
	}
	if (ret < 0) {
		pr_err("!!! host%d MIPIHOSTIOC_SNRCLK_SET_FREQ error, ret = %d\n",
			e->entry_num, ret);
		return -HB_VIN_MIPI_HOST_SNRCLK_SET_FREQ_FAIL;
	}
	pr_info("mipi host%d snrclk set %u end\n", e->entry_num, freq);
	return ret;
}

int hb_vin_mipi_host_pre_init_request(entry_t *e, uint32_t timeout)
{
	int ret = -HB_VIN_MIPI_HOST_PPE_INIT_REQUEST_FAIL;

	if(!e)
		return ret;

	if (e->host_fd <= 0)
		e->host_fd = open(hb_vin_mipi_host_path(e), O_RDWR | O_CLOEXEC);
	if (e->host_fd > 0 &&
		ioctl(e->host_fd, MIPIHOSTIOC_PRE_INIT_REQUEST, &timeout) == 0)
		ret = RET_OK;

	return ret;
}

int hb_vin_mipi_host_pre_start_request(entry_t *e, uint32_t timeout)
{
	int ret = -HB_VIN_MIPI_HOST_PRE_START_REQUEST_FAIL;

	if(!e)
		return ret;

	if (e->host_fd <= 0)
		e->host_fd = open(hb_vin_mipi_host_path(e), O_RDWR | O_CLOEXEC);
	if (e->host_fd > 0 &&
		ioctl(e->host_fd, MIPIHOSTIOC_PRE_START_REQUEST, &timeout) == 0)
		ret = RET_OK;

	return ret;
}

int hb_vin_mipi_host_pre_init_result(entry_t *e, uint32_t result)
{
	int ret = -HB_VIN_MIPI_HOST_PRE_INIT_RESULT_FAIL;

	if(!e)
		return ret;

	if (e->host_fd <= 0)
		e->host_fd = open(hb_vin_mipi_host_path(e), O_RDWR | O_CLOEXEC);
	if (e->host_fd > 0 &&
		ioctl(e->host_fd, MIPIHOSTIOC_PRE_INIT_RESULT, &result) == 0)
		ret = RET_OK;

	return ret;
}

int hb_vin_mipi_host_pre_start_result(entry_t *e, uint32_t result)
{
	int ret = -HB_VIN_MIPI_HOST_PRE_START_RESULT_FAIL;

	if(!e)
		return ret;

	if (e->host_fd <= 0)
		e->host_fd = open(hb_vin_mipi_host_path(e), O_RDWR | O_CLOEXEC);
	if (e->host_fd > 0 &&
		ioctl(e->host_fd, MIPIHOSTIOC_PRE_START_RESULT, &result) == 0)
		ret = RET_OK;

	return ret;
}

int hb_vin_mipi_host_ipi_reset(entry_t *e, int32_t ipi, uint32_t enable)
{
	int ret = RET_OK;
	int fd = -1;
	mipi_host_ipi_reset_t ipi_reset;

	if(!e)
		return -HB_VIN_MIPI_HOST_IPI_RESET_FAIL;
	pr_info("mipi host%d reset %d:ipi%d as %s\n", e->entry_num,
		ipi, ipi + 1, (enable) ? "enable" : "disable");
	if (ipi < 0)
		ipi_reset.mask = 0xff;
	else
		ipi_reset.mask = (uint16_t)(0x1 << ipi);
	ipi_reset.enable = (uint16_t)enable;
	if (e->host_fd > 0) {
		ret = ioctl(e->host_fd, MIPIHOSTIOC_IPI_RESET, &ipi_reset);
	} else {
		fd = open(hb_vin_mipi_host_path(e), O_RDWR | O_CLOEXEC);
		if (fd >= 0) {
			ret = ioctl(e->host_fd, MIPIHOSTIOC_IPI_RESET, &ipi_reset);
			close(fd);
		} else {
			ret = -1;
		}
	}
	if (ret < 0) {
		pr_err("!!! host%d MIPIHOSTIOC_IPI_RESET error, ret = %d\n",
			e->entry_num, ret);
		return -HB_VIN_MIPI_HOST_IPI_RESET_FAIL;
	}
	pr_info("mipi host%d reset %d:ipi%d end\n", e->entry_num, ipi, ipi + 1);
	return ret;
}

int hb_vin_mipi_host_ipi_fatal(entry_t *e, int32_t ipi)
{
	int ret = RET_OK;
	int fd = -1;
	mipi_host_ipi_info_t ipi_info;

	if(!e)
		return -HB_VIN_MIPI_HOST_IPI_FATAL_FAIL;
	memset(&ipi_info, 0, sizeof(ipi_info));
	ipi_info.index = (uint8_t)ipi;
	if (e->host_fd > 0) {
		ret = ioctl(e->host_fd, MIPIHOSTIOC_IPI_GET_INFO, &ipi_info);
	} else {
		fd = open(hb_vin_mipi_host_path(e), O_RDWR | O_CLOEXEC);
		if (fd >= 0) {
			ret = ioctl(e->host_fd, MIPIHOSTIOC_IPI_GET_INFO, &ipi_info);
			close(fd);
		} else {
			ret = -1;
		}
	}
	if (ret < 0) {
		pr_err("!!! host%d MIPIHOSTIOC_IPI_GET_INFO error, ret = %d\n",
			e->entry_num, ret);
		return -HB_VIN_MIPI_HOST_IPI_FATAL_FAIL;
	}
	return (int)(ipi_info.fatal);
}

int hb_vin_mipi_host_ipi_get_info(entry_t *e, int32_t ipi, mipi_host_ipi_info_t *info)
{
	int ret = RET_OK;
	int fd = -1;
	mipi_host_ipi_info_t ipi_info;

	if(!e || !info)
		return -HB_VIN_MIPI_HOST_IPI_GET_INFO_FAIL;
	memset(&ipi_info, 0, sizeof(ipi_info));
	ipi_info.index = (uint8_t)ipi;
	if (e->host_fd > 0) {
		ret = ioctl(e->host_fd, MIPIHOSTIOC_IPI_GET_INFO, &ipi_info);
	} else {
		fd = open(hb_vin_mipi_host_path(e), O_RDWR | O_CLOEXEC);
		if (fd >= 0) {
			ret = ioctl(e->host_fd, MIPIHOSTIOC_IPI_GET_INFO, &ipi_info);
			close(fd);
		} else {
			ret = -1;
		}
	}
	if (ret < 0) {
		pr_err("!!! host%d MIPIHOSTIOC_IPI_GET_INFO error, ret = %d\n",
			e->entry_num, ret);
		return -HB_VIN_MIPI_HOST_IPI_GET_INFO_FAIL;
	}
	memcpy(info, &ipi_info, sizeof(ipi_info));
	return ret;
}

int hb_vin_mipi_host_ipi_set_info(entry_t *e, int32_t ipi, mipi_host_ipi_info_t *info)
{
	int ret = RET_OK;
	int fd = -1;
	mipi_host_ipi_info_t ipi_info;

	if(!e || !info)
		return -HB_VIN_MIPI_HOST_IPI_SET_INFO_FAIL;
	pr_info("mipi host%d set %d:ipi%d as mode=0x%x,vc=0x%x,dt=0x%x,hsa=%d,hbp=%d,hsd=%d,adv=0x%x\n",
		e->entry_num, ipi, ipi + 1, info->mode, info->vc, info->datatype,
		info->hsa, info->hbp, info->hsd, info->adv);
	memcpy(&ipi_info, info, sizeof(ipi_info));
	ipi_info.index = (uint8_t)ipi;
	if (e->host_fd > 0) {
		ret = ioctl(e->host_fd, MIPIHOSTIOC_IPI_SET_INFO, &ipi_info);
	} else {
		fd = open(hb_vin_mipi_host_path(e), O_RDWR | O_CLOEXEC);
		if (fd >= 0) {
			ret = ioctl(e->host_fd, MIPIHOSTIOC_IPI_SET_INFO, &ipi_info);
			close(fd);
		} else {
			ret = -1;
		}
	}
	if (ret < 0) {
		pr_err("!!! host%d MIPIHOSTIOC_IPI_SET_INFO error, ret = %d\n",
			e->entry_num, ret);
		return -HB_VIN_MIPI_HOST_IPI_SET_INFO_FAIL;
	}
	pr_info("mipi host%d set %d:ipi%d end\n", e->entry_num, ipi, ipi + 1);
	return ret;
}
