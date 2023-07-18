/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2016 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/msg.h>
#include <logging.h>

#include "cJSON.h"
#include "inc/hb_vin.h"
#include "inc/hb_vin_common.h"
#include "inc/hb_vin_mipi_dev.h"
#include "inc/hb_vin_mipi_host.h"

entry_t entry[ENTRY_NUM];

int hb_cam_mipi_parse_cfg(char *filename, int fps, int resolution, int entry_num)
{
	char *filebuf = NULL;
	FILE *fp = NULL;
	struct stat statbuf;
	cJSON *mipi = NULL, *mipi_root = NULL, *node;
	int ret = RET_OK;
	int fread_ret = RET_OK;
	char file_buff[256] = {0};
	entry_t *e = NULL;

	if(filename == NULL) {
		pr_err("cam config file is null !!\n");
		return ret;
	}
	pr_debug("filename %s fps %d resolution %d\n", filename, fps, resolution);
	snprintf(file_buff, sizeof(file_buff), filename, fps, resolution);
	ret = stat(file_buff, &statbuf);
	if(ret < 0) {
		pr_err("file %s stat is fail!\n", file_buff);
		return ret;
	}
	pr_info("file_buff = %s\n", file_buff);
	if(0 == statbuf.st_size) {
		pr_err("mipi config file size is zero !!\n");
		return -RET_ERROR;
	}
	fp = fopen(file_buff, "r");
	if(fp == NULL) {
		pr_err("open %s fail!!\n", file_buff);
		return -RET_ERROR;
	}
	filebuf = (char *)malloc(statbuf.st_size + 1); /* PRQA S 5118 */
	if(NULL == filebuf) {
		pr_err("malloc buff fail !!\n");
		goto err1;
	}
	memset(filebuf, 0, statbuf.st_size + 1);
	fread_ret = fread(filebuf, statbuf.st_size, 1, fp);
	if(fread_ret < 1) {
		pr_err("fread mipi file is error!\n");
		goto err2;
	}
	mipi_root = cJSON_Parse((const char *)filebuf);
	if(NULL == mipi_root) {
		pr_err("parse %s json fail\n", file_buff);
		goto err2;
	}
	mipi = cJSON_GetObjectItem(mipi_root, "mipi");
	if (NULL == mipi) {
		pr_info("no mipi node found!\n");
	}

	if (entry_num >= ENTRY_NUM) {
		pr_err("entry_num %d > %d overflow\n", entry_num, ENTRY_NUM);
		goto err;
	}
	e = &entry[entry_num];
	ret = hb_vin_mipi_host_parser_config(mipi, e);
	if(ret < 0) {
		pr_err("mipi_host parser config file error!\n");
		goto err;
	}
	ret = hb_vin_mipi_dev_parser_config(mipi, e);
	if(ret < 0) {
		pr_err("mipi_dev parser config file error!\n");
		goto err;
	}
	e->entry_num = entry_num;
	free((void *)filebuf); /* PRQA S 5118 */ /* memery api */
	fclose(fp);
	cJSON_Delete(mipi_root);
	return RET_OK;

err:
	cJSON_Delete(mipi_root);
err2:
	free((void *)filebuf); /* PRQA S 5118 */ /* memery api */
err1:
	fclose(fp);
	return -RET_ERROR;
}

int hb_vin_init(uint32_t entry_num)
{
	int ret = RET_OK;
	entry_t *e;

	if (entry_num >= ENTRY_NUM)
		return -RET_ERROR;
	e = &entry[entry_num];
	e->entry_num = entry_num;
	if (e->init_state >= VIN_INIT) {
		pr_info("vin %u have been inited\n", entry_num);
		return RET_OK;
	}
	if(e->host_enable) {
		ret = hb_vin_mipi_host_init(e);
		if(ret < 0) {
			pr_err("mipi_host %u init error!\n", entry_num);
			return ret;
		}
	}
	if(e->dev_enable) {
		ret = hb_vin_mipi_dev_init(e);
		if(ret < 0) {
			pr_err("mipi_dev init error!\n");
			if(e->host_enable)
				hb_vin_mipi_host_deinit(e);
			return ret;
		}
	}
//	usleep(100*1000);
	e->init_state = VIN_INIT;
	pr_info("hb_vin_init %u end\n", entry_num);
	return ret;
}

int hb_vin_deinit(uint32_t entry_num)
{
	int ret = RET_OK;
	entry_t *e;

	if (entry_num >= ENTRY_NUM)
		return -RET_ERROR;
	e = &entry[entry_num];
	e->entry_num = entry_num;

	if(e->init_state == VIN_DEINIT) {
		pr_info("vin %u has been deinited\n", entry_num);
		return RET_OK;
	}

	if(e->start_state == VIN_START)
		hb_vin_stop(entry_num);

	if(e->dev_enable)
		hb_vin_mipi_dev_deinit(e);

	if(e->host_enable)
		hb_vin_mipi_host_deinit(e);

	e->init_state = VIN_DEINIT;
	pr_info("hb_vin_deinit %u end\n", entry_num);
	return ret;
}

int hb_vin_stop(uint32_t entry_num)
{
    int ret = RET_OK;
    entry_t *e;

    if (entry_num >= ENTRY_NUM)
        return -RET_ERROR;
    e = &entry[entry_num];
	e->entry_num = entry_num;

    if (e->start_state <= VIN_INIT) {
        pr_err("need vin %u start before stop\n", entry_num);
        return -RET_ERROR;
    }

    if (e->dev_enable && e->mipi_dev_cfg.vpg)
        ret = hb_vin_mipi_dev_stop(e);

    if (ret == RET_OK && e->host_enable)
        ret = hb_vin_mipi_host_stop(e);

    if (ret == RET_OK && e->dev_enable && !e->mipi_dev_cfg.vpg)
        ret = hb_vin_mipi_dev_stop(e);

    if (ret == RET_OK) {
        pr_info("hb_vin_stop %u end\n", entry_num);
        e->start_state = VIN_STOP;
    } else {
        pr_err("hb_vin_stop %u error %d\n", entry_num, ret);
    }
    return ret;
}

int hb_vin_start(uint32_t entry_num)
{
	int ret = RET_OK;
	entry_t *e;

	if (entry_num >= ENTRY_NUM)
		return -RET_ERROR;
	e = &entry[entry_num];
	e->entry_num = entry_num;

	if(e->init_state == VIN_DEINIT) {
		pr_err("need vin %u init before start\n", entry_num);
		return -RET_ERROR;
	}

	if(e->dev_enable && e->mipi_dev_cfg.vpg) {
		ret = hb_vin_mipi_dev_start(e);
		if(ret < 0) {
			pr_err("!!!Can't start mipi dev %d\n", ret);
			return ret;
		}
	}

	if(e->host_enable) {
		ret = hb_vin_mipi_host_start(e);
		if(ret < 0) {
			pr_err("!!!Can't start mipi host%u %d\n", entry_num, ret);
			if(e->dev_enable && e->mipi_dev_cfg.vpg)
				hb_vin_mipi_dev_stop(e);
			return ret;
		}
	}

	if(e->dev_enable && !e->mipi_dev_cfg.vpg) {
		ret = hb_vin_mipi_dev_start(e);
		if(ret < 0) {
			pr_err("!!!Can't start mipi dev %d\n", ret);
			if(e->host_enable)
				hb_vin_mipi_host_stop(e);
			return ret;
		}
	}

	e->start_state = VIN_START;
	pr_info("hb_vin_start %u end\n", entry_num);
	return ret;
}

int hb_vin_snrclk_set_en(uint32_t entry_num, uint32_t enable)
{
	int ret = RET_OK;
	entry_t *e;

	if (entry_num >= ENTRY_NUM)
		return -RET_ERROR;
	e = &entry[entry_num];
	e->entry_num = entry_num;
	ret = hb_vin_mipi_host_snrclk_set_en(e, enable);
	if(ret < 0) {
		pr_err("!!!Can't set %u snrclk en %d\n", entry_num, ret);
		return ret;
	}

	pr_info("hb_vin_snrclk_set_en %u end\n", entry_num);
	return ret;
}

int hb_vin_snrclk_set_freq(uint32_t entry_num, uint32_t freq)
{
	int ret = RET_OK;
	entry_t *e;

	if (entry_num >= ENTRY_NUM)
		return -RET_ERROR;
	e = &entry[entry_num];
	e->entry_num = entry_num;
	if(freq == 0) {
		pr_err("invalid freq value\n");
		return -RET_ERROR;
	}
	ret = hb_vin_mipi_host_snrclk_set_freq(e, freq);
	if(ret < 0) {
		pr_err("!!!Can't set host%u snrclk freq %d\n", entry_num, ret);
		return ret;
	}

	pr_info("hb_vin_snrclk_set_freq %u end\n", entry_num);
	return ret;
}

int hb_vin_pre_request(uint32_t entry_num, uint32_t type, uint32_t timeout)
{
	int ret = RET_OK;
	entry_t *e;

	if (entry_num >= ENTRY_NUM)
		return -RET_ERROR;
	e = &entry[entry_num];
	e->entry_num = entry_num;

	if (type)
		ret = hb_vin_mipi_host_pre_start_request(e, timeout);
	else
		ret = hb_vin_mipi_host_pre_init_request(e, timeout);

	return ret;
}

int hb_vin_pre_result(uint32_t entry_num, uint32_t type, uint32_t result)
{
	int ret = RET_OK;
	entry_t *e;

	if (entry_num >= ENTRY_NUM)
		return -RET_ERROR;
	e = &entry[entry_num];
	e->entry_num = entry_num;

	if (type)
		ret = hb_vin_mipi_host_pre_start_result(e, result);
	else
		ret = hb_vin_mipi_host_pre_init_result(e, result);
	return ret;
}

int hb_vin_chn_bypass(uint32_t port, uint32_t enable,
		uint32_t mux_sel, uint32_t chn_mask)
{
#ifndef NOSIF
	int ret = RET_OK;
	int sif_fd;
	uint32_t mipi_opt = (enable & 0x2) ? 0 : 1;
	uint32_t enable_x = enable & 0x1;
	entry_t *e;
	sif_input_bypass_t bypass_ctrl;

	if (port >= ENTRY_NUM || mux_sel == 2 || mux_sel > 4 || chn_mask > 0xf)
		return -RET_ERROR;
	e = &entry[port];
	sif_fd = open(HB_VIN_SIF_PATH,  O_RDWR | O_CLOEXEC);
	if (sif_fd < 0) {
		pr_info("!!!Can't open the sif device, ret = %d\n", sif_fd);
		return -HB_VIN_SIF_OPEN_DEV_FAIL;
	}
	bypass_ctrl.enable_bypass = enable_x;
	bypass_ctrl.enable_frame_id = 1;
	bypass_ctrl.init_frame_id = 1;
	bypass_ctrl.set_bypass_channels = chn_mask * 10 + mux_sel;
	ret = ioctl(sif_fd, SIF_IOC_BYPASS, &bypass_ctrl);
	if (ret < 0) {
		pr_err("!!! SIF_IOC_BYPASS error, ret = %d\n", ret);
		close(sif_fd);
		return -HB_VIN_SIF_BYPASS_FAIL;
	}
	close(sif_fd);
	if (mipi_opt) {
		if (enable_x) {
			usleep(50*1000);
			ret = hb_vin_mipi_dev_stop(e);
			ret = hb_vin_mipi_dev_start(e);
			if(ret < 0)
				pr_err("!!!hb_vin_mipi_dev_start error%d\n", ret);
		} else {
			ret = hb_vin_mipi_dev_stop(e);
			if(ret < 0)
				pr_err("!!!hb_vin_mipi_dev_stop error%d\n", ret);
		}
	}
#else
	int ret = -RET_ERROR;
#endif

	return ret;
}

int hb_vin_iar_bypass(uint32_t port, uint32_t enable,
		uint32_t enable_frame_id, uint32_t init_frame_id)
{
#ifndef NOSIF
	int ret = RET_OK;
	int sif_fd;
	uint32_t mipi_opt = (enable & 0x2) ? 0 : 1;
	uint32_t enable_x = enable & 0x1;
	entry_t *e;
	sif_input_bypass_t bypass_ctrl;

	if (port >= ENTRY_NUM)
		return -RET_ERROR;
	e = &entry[port];
	sif_fd = open(HB_VIN_SIF_PATH,  O_RDWR | O_CLOEXEC);
	if (sif_fd < 0) {
		pr_info("!!!Can't open the sif device, ret = %d\n", sif_fd);
		return -HB_VIN_SIF_OPEN_DEV_FAIL;
	}
	bypass_ctrl.enable_bypass = enable_x;
	bypass_ctrl.enable_frame_id = enable_frame_id;
	bypass_ctrl.init_frame_id = init_frame_id;
	bypass_ctrl.set_bypass_channels = 0x1 * 10 + 0x2;
	ret = ioctl(sif_fd, SIF_IOC_BYPASS, &bypass_ctrl);
	if (ret < 0) {
		pr_err("!!! SIF_IOC_BYPASS error, ret = %d\n", ret);
		close(sif_fd);
		sif_fd = -1;
		return -HB_VIN_SIF_BYPASS_FAIL;
	}
	if (sif_fd >= 0) {
		close(sif_fd);
		sif_fd = -1;
	}
	if (mipi_opt) {
		if (enable_x) {
			usleep(50*1000);
			ret = hb_vin_mipi_dev_stop(e);
			ret = hb_vin_mipi_dev_start(e);
			if(ret < 0)
				pr_err("!!!hb_vin_mipi_dev_start error%d\n", ret);
		} else {
			ret = hb_vin_mipi_dev_stop(e);
			if(ret < 0)
				pr_err("!!!hb_vin_mipi_dev_stop error%d\n", ret);
		}
	}
#else
	int ret = -RET_ERROR;
#endif
	return ret;
}

int hb_vin_set_bypass(uint32_t port, uint32_t enable)
{
	int ret = RET_OK;
	int sif_fd;
	uint32_t enable_x = enable & 0x1;
	entry_t *e;
#ifndef NOSIF
	uint32_t mipi_opt = (enable & 0x2) ? 0 : 1;
	sif_input_bypass_t bypass_ctrl;
#endif

	if (port >= ENTRY_NUM)
		return -RET_ERROR;
	e = &entry[port];

#ifndef NOSIF
	sif_fd = open(HB_VIN_SIF_PATH,  O_RDWR | O_CLOEXEC);
	if (sif_fd < 0) {
		pr_info("!!!Can't open the sif device, ret = %d\n", sif_fd);
		return -HB_VIN_SIF_OPEN_DEV_FAIL;
	}
	bypass_ctrl.enable_bypass = enable_x;
	bypass_ctrl.enable_frame_id = 0;
	bypass_ctrl.init_frame_id = 0;
	bypass_ctrl.set_bypass_channels = 0x1 * 1000 + 0x1 * 10;
	ret = ioctl(sif_fd, SIF_IOC_BYPASS, &bypass_ctrl);
	if (ret < 0) {
		pr_err("!!! SIF_IOC_BYPASS error, ret = %d\n", ret);
		close(sif_fd);
		sif_fd = -1;
		return -HB_VIN_SIF_BYPASS_FAIL;
	}
	if (sif_fd >= 0) {
		close(sif_fd);
		sif_fd = -1;
	}
	if (mipi_opt) {
#endif
		if (enable_x) {
			usleep(50*1000);
			ret = hb_vin_mipi_dev_stop(e);
			ret = hb_vin_mipi_dev_start(e);
			if(ret < 0)
				pr_err("!!!hb_vin_mipi_dev_start error%d\n", ret);
		} else {
			ret = hb_vin_mipi_dev_stop(e);
			if(ret < 0)
				pr_err("!!!hb_vin_mipi_dev_stop error%d\n", ret);
		}
#ifndef NOSIF
	}
#endif

	return ret;
}

int hb_vin_mipi_reset(uint32_t entry_num)
{
	int ret = RET_OK;
	entry_t *e;

	if (entry_num >= ENTRY_NUM)
		return -RET_ERROR;
	e = &entry[entry_num];
	e->entry_num = entry_num;
	ret = hb_vin_mipi_host_stop(e);
	if(ret < 0) {
		pr_err("!!!hb_vin_mipi_host_stop error%d\n", ret);
		return ret;
	}
	ret = hb_vin_mipi_dev_stop(e);
	if(ret < 0) {
		pr_err("!!!hb_vin_mipi_dev_stop error%d\n", ret);
		return ret;
	}
	e->start_state = VIN_STOP;
	ret = hb_vin_mipi_dev_deinit(e);
	if(ret < 0) {
			pr_err("hb_vin_mipi_dev_deinit err\n");
			return ret;
	}
	ret = hb_vin_mipi_host_deinit(e);
	if(ret < 0) {
			pr_err("hb_vin_mipi_host_deinit err\n");
			return ret;
	}
	e->init_state = VIN_DEINIT;
	ret = hb_vin_mipi_host_init(e);
	if(ret < 0) {
		pr_err("mipi_host init error!\n");
		hb_vin_mipi_host_deinit(e);
		return ret;
	}
	ret = hb_vin_mipi_dev_init(e);
	if(ret < 0) {
		pr_err("mipi_dev init error!\n");
		hb_vin_mipi_dev_deinit(e);
		return ret;
	}
	e->init_state = VIN_INIT;
	ret = hb_vin_mipi_host_start(e);
	if(ret < 0) {
		pr_err("!!!Can't start mipi host %d\n", ret);
		return ret;
	}
	e->start_state = VIN_START;

	pr_info("hb_vin_mipi_reset end\n");
	return ret;
}

int hb_vin_ipi_reset(uint32_t entry_num, int32_t ipi, uint32_t enable)
{
	int ret = RET_OK;
	entry_t *e;

	if (entry_num >= ENTRY_NUM)
		return -RET_ERROR;
	e = &entry[entry_num];
	e->entry_num = entry_num;
	ret = hb_vin_mipi_host_ipi_reset(e, ipi, enable);
	if(ret < 0) {
		pr_err("!!!hb_vin_mipi_host_ipi_reset error%d\n", ret);
		return ret;
	}
	pr_info("hb_vin_mipi_host_ipi_reset end\n");
	return ret;
}

int hb_vin_ipi_fatal(uint32_t entry_num, int32_t ipi)
{
	int ret = RET_OK;
	entry_t *e;

	if (entry_num >= ENTRY_NUM)
		return -RET_ERROR;
	e = &entry[entry_num];
	e->entry_num = entry_num;
	ret = hb_vin_mipi_host_ipi_fatal(e, ipi);
	if(ret < 0) {
		pr_err("!!!hb_vin_mipi_host_ipi_fatal error%d\n", ret);
		return ret;
	}
	pr_info("hb_vin_mipi_host_ipi_fatal end\n");
	return ret;
}

int hb_vin_reset(uint32_t entry_num)
{
	int ret = RET_OK;

	ret = hb_vin_mipi_reset(entry_num);
	if(ret < 0) {
		pr_err("hb_vin_mipi_reset failed\n");
		return ret;
	}
	return ret;
}

int hb_vin_open(uint32_t entry_num)
{
	int ret = RET_OK;
	entry_t *e;

	if (entry_num >= ENTRY_NUM)
		return -RET_ERROR;
	e = &entry[entry_num];
	e->entry_num = entry_num;

	if(e->host_enable) {
		ret = hb_vin_mipi_host_open(e);
		if (ret < 0) {
			pr_err("hb_vin_mipi_host_open failed\n");
		}
	}
	return ret;
}

int hb_vin_close(uint32_t entry_num)
{
	int ret = RET_OK;
	entry_t *e;

	if (entry_num >= ENTRY_NUM)
		return -RET_ERROR;
	e = &entry[entry_num];
	e->entry_num = entry_num;

	if(e->host_enable) {
		if(e->host_fd > 0) {
			close(e->host_fd);
			e->host_fd = 0;
		}
	}
	return ret;
}

