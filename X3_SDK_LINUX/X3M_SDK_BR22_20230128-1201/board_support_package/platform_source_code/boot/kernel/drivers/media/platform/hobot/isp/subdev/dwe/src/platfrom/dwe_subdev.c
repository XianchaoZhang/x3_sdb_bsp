/*
 *
 *   Copyright (C) 2018 Horizon Inc.
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

#define pr_fmt(fmt) "[ldc_drv]: %s: " fmt, __func__

#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/dmaengine.h>
#include <linux/compiler.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/mman.h>
#include <linux/device.h>
#include <linux/module.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-async.h>

#ifdef CONFIG_HOBOT_DIAG
#include <soc/hobot/diag.h>
#endif
#include "acamera_logger.h"

#include "vio_config.h"
#include "vio_group_api.h"
#include "system_dwe_api.h"
#include "dwe_dev.h"
#include "dwe_subdev.h"

#if defined(CUR_MOD_NAME)
#undef CUR_MOD_NAME
#define CUR_MOD_NAME LOG_MODULE_SOC_DWE
#else
#define CUR_MOD_NAME LOG_MODULE_SOC_DWE
#endif

#define ARGS_TO_PTR(arg) ((struct soc_dwe_ioctl_args *)arg)

/*PRQA S 2996,3238,0685 ++*/

/* global variable define */
typedef struct _subdev_dwe_ctx {
	struct v4l2_subdev soc_dwe;
	dwe_param_t *ptr_param;  /* param used for ldc/dis */
	//void *ptr_mem;
	struct dwe_dev_s *dev_ctx;
	spinlock_t ldclock; /* spin lock to protect ldc data */
	spinlock_t dislock; /* spin lock to protect dis data */
	dwe_context_t ctx;  /* data used for ctx */
	uint32_t ldc_irqstatus;
	uint32_t dis_irqstatus;
	struct work_struct ldc_work;
	struct work_struct dis_work;
} subdev_dwe_ctx;

static subdev_dwe_ctx *dwe_ctx;
static int ldc_error_sts = 0;

void ldc_printk(void)
{
	// no func
}

void dis_printk(void)
{
	// no func
}

int ldc_status_check(void)
{
	return ldc_error_sts;
}
EXPORT_SYMBOL(ldc_status_check);

/*
 * update ldc param
 * note : only update shadow info ldc_hwparam_set
 * ldc_hwpath_set update after fe
 */
static void update_ldc_param(void)
{
	//set param when next port is setting
	if ((dwe_ctx->ctx.ldc_next_port >= 0) && (dwe_ctx->ctx.ldc_next_port < 0xff)) {
		ldc_hwparam_set(&dwe_ctx->ctx, dwe_ctx->ctx.ldc_next_port);
		dwe_ctx->ctx.ldc_next_port = dwe_ctx->ctx.ldc_next_port + 0x100;
	}

	//set setting when ldc is not running
	if (dwe_ctx->ctx.ldc_running == 0) {
		if (dwe_ctx->ctx.ldc_next_port >= 0) {
			ldc_hwpath_set(&dwe_ctx->ctx, (dwe_ctx->ctx.ldc_next_port - 0x100));
		}
	}
}

/*
 * update dis param
 * note : only update shadow info in dis_hwparam_set
 * dis_hwpath_set update after fe
 */
void update_dis_param(void)
{
	//set param when next port is setting
	if ((dwe_ctx->ctx.dis_next_port >= 0) && (dwe_ctx->ctx.dis_next_port < 0xff)) {
		dis_hwparam_set(&dwe_ctx->ctx, dwe_ctx->ctx.dis_next_port);
		dwe_ctx->ctx.dis_next_port = dwe_ctx->ctx.dis_next_port + 0x100;
	}

	//set setting when dis is not running
	if (dwe_ctx->ctx.dis_running == 0) {
		if (dwe_ctx->ctx.dis_next_port >= 0) {
			dis_hwpath_set(&dwe_ctx->ctx, (dwe_ctx->ctx.dis_next_port - 0x100));
		}
	}
}

static void ldc_task_work(struct work_struct *work)
{
	unsigned long flags;

	LOG(LOG_INFO, "run %d , port %d, ldc_irq 0x%x !\n", dwe_ctx->ctx.ldc_running, dwe_ctx->ctx.ldc_next_port, dwe_ctx->ldc_irqstatus);

	spin_lock_irqsave(&dwe_ctx->ldclock, flags);
	update_ldc_param();
	spin_unlock_irqrestore(&dwe_ctx->ldclock, flags);
	//debug
	ldc_printk_info();
}

static void dis_task_work(struct work_struct *work)
{
	unsigned long flags;

	LOG(LOG_INFO, "run %d , port %d, dis_irq 0x%x !\n", dwe_ctx->ctx.dis_running, dwe_ctx->ctx.dis_next_port, dwe_ctx->dis_irqstatus);

	spin_lock_irqsave(&dwe_ctx->dislock, flags);
	update_dis_param();
	spin_unlock_irqrestore(&dwe_ctx->dislock, flags);
	//debug info
	dwe_printk_info();
}

/*
 * reset ctx param
 */
void reset_dwe_ctx(void)
{
	dwe_ctx->ctx.dis_running = 0;
	dwe_ctx->ctx.ldc_running = 0;
	dwe_ctx->ctx.ldc_curr_port = 0;
	dwe_ctx->ctx.ldc_next_port = 0;
	dwe_ctx->ctx.dis_curr_port = 0;
	dwe_ctx->ctx.dis_next_port = 0;
	dwe_ctx->ctx.online_enable = 1;
	dwe_ctx->ctx.online_port = 0;
	dwe_ctx->dis_irqstatus = 0;
	dwe_ctx->ldc_irqstatus = 0;
}
EXPORT_SYMBOL(reset_dwe_ctx);

// switch ldc port
int ldc_set_ioctl(uint32_t port, uint32_t online)
{
	unsigned long flags;
	int ret = 0;

	if (online == 1) {
		dwe_ctx->ctx.online_enable = 1;
		dwe_ctx->ctx.online_port = port;
	} else {
		dwe_ctx->ctx.online_enable = 0;
	}

	spin_lock_irqsave(&dwe_ctx->ldclock, flags);
	dwe_ctx->ctx.ldc_next_port = port;
	update_ldc_param();
	spin_unlock_irqrestore(&dwe_ctx->ldclock, flags);
	// schedule_work(&dwe_ctx->ldc_work);

	pr_debug("----set port 0x%x----", port);
	return ret;
}
EXPORT_SYMBOL(ldc_set_ioctl);

// switch dis port
int dis_set_ioctl(uint32_t port, uint32_t online)
{
	unsigned long flags;
	int ret = 0;

	if (online == 1) {
		dwe_ctx->ctx.online_enable = 1;
		dwe_ctx->ctx.online_port = port;
	} else {
		dwe_ctx->ctx.online_enable = 0;
	}

	spin_lock_irqsave(&dwe_ctx->dislock, flags);
	dwe_ctx->ctx.dis_next_port = port;
	dwe_ctx->ctx.dis_curr_port = port;
	update_dis_param();
	spin_unlock_irqrestore(&dwe_ctx->dislock, flags);
	// schedule_work(&dwe_ctx->dis_work);

	return ret;
}
EXPORT_SYMBOL(dis_set_ioctl);

/* dis model irq
 * pg_done:
 * error :
 * frame_done: change setting & buffer
 * note:
 *   if  FIRMWARE_CONTEXT_NUMBER < 4
 *       using setting[0-4]
 *   else
 *       using setting[0-1]
 *
 */

static irqreturn_t x3_dis_irq(int this_irq, void *data)
{
	int ret = 0;
	uint32_t addr = 0;
	uint32_t tmp = 0;
	dis_irqstatus_u tmp_irq;

	disable_irq_nosync(this_irq);

	spin_lock(&dwe_ctx->dislock);
	//dis irq
	get_dwe_int_status(dwe_ctx->dev_ctx->dis_dev->io_vaddr,
		&tmp_irq.status_g);
	set_dwe_int_status(dwe_ctx->dev_ctx->dis_dev->io_vaddr, &tmp_irq.status_g);

	if (tmp_irq.status_b.int_pg_done == 1) {
		pr_debug("dis pg_done");
		addr = 0x00;
		ldc_debug_info(dwe_ctx->dev_ctx->dis_dev->io_vaddr, addr, &tmp);
		pr_debug("[dump] addr 0x%x, data 0x%x", addr, tmp);
		addr = 0x10;
		ldc_debug_info(dwe_ctx->dev_ctx->dis_dev->io_vaddr, addr, &tmp);
		pr_debug("[dump] addr 0x%x, data 0x%x", addr, tmp);
		addr = 0x14;
		ldc_debug_info(dwe_ctx->dev_ctx->dis_dev->io_vaddr, addr, &tmp);
		pr_debug("[dump] addr 0x%x, data 0x%x", addr, tmp);
		LOG(LOG_DEBUG, "dis pg_done!");
	}

	if (tmp_irq.status_b.int_frame_done == 1) {
		//debug
		pr_debug("----dis done----");
		addr = 0x4;
		ldc_debug_info(dwe_ctx->dev_ctx->dis_dev->io_vaddr, addr, &tmp);
		pr_debug("[dump] addr 0x%x, data 0x%x", addr, tmp);
		addr = 0x44;
		ldc_debug_info(dwe_ctx->dev_ctx->dis_dev->io_vaddr, addr, &tmp);
		pr_debug("[dump] addr 0x%x, data 0x%x", addr, tmp);
		addr = 0x114;
		ldc_debug_info(dwe_ctx->dev_ctx->dis_dev->io_vaddr, addr, &tmp);
		pr_debug("[dump] addr 0x%x, data 0x%x", addr, tmp);
		addr = 0x214;
		ldc_debug_info(dwe_ctx->dev_ctx->dis_dev->io_vaddr, addr, &tmp);
		pr_debug("[dump] addr 0x%x, data 0x%x", addr, tmp);
		LOG(LOG_INFO, "----dis_irqstatus %x ----", dwe_ctx->dis_irqstatus);
		dwe_ctx->dis_irqstatus = tmp_irq.status_g;

		dwe_ctx->ctx.dis_running = 0;
		ret = dwe_stream_put_frame(dwe_ctx->ctx.dis_curr_port,
			&dwe_ctx->ctx.dframes[dwe_ctx->ctx.dis_curr_port]);
		//if online
		if (dwe_ctx->ctx.online_enable == 1) {
			dwe_ctx->ctx.dis_next_port = dwe_ctx->ctx.online_port;
		}
		update_dis_param();
		// schedule_work(&dwe_ctx->dis_work);
	}

	if ((tmp_irq.status_b.int_dis_h_ratio_err == 1) ||
		(tmp_irq.status_b.int_dis_v_ratio_err == 1)) {
		dwe_ctx->dis_irqstatus |= tmp_irq.status_g;

		dwe_ctx->ctx.dis_running = 0;
		LOG(LOG_DEBUG, "----over_flow!----");
	}

	spin_unlock(&dwe_ctx->dislock);

	enable_irq(this_irq);
	return IRQ_HANDLED;
}

#ifdef CONFIG_HOBOT_DIAG
static void ldc_diag_report(uint8_t errsta, uint32_t status)
{
	uint8_t env_data[8];

#if IS_ENABLED(CONFIG_HOBOT_DIAG_INJECT)
	ldc_irqstatus_u irq_stat = {0};
	diag_inject_val(ModuleDiag_VIO, EventIdVioLdcErr, &irq_stat.status_g);
	if (irq_stat.status_b.frame_overwrite
					|| irq_stat.status_b.line_overwrite
					|| irq_stat.status_b.isp_in_overwrite
					|| irq_stat.status_b.overflow) {
		errsta = 1;
		status = irq_stat.status_g;
	}
#endif
	env_data[0] = 0xff;
	env_data[1] = 0xff;
	env_data[2] = 0xff;
	env_data[3] = (uint8_t)(sizeof(uint32_t));
	env_data[4] = (uint8_t)(status & 0xff);
	env_data[5] = (uint8_t)((status >> 8) & 0xff);
	env_data[6] = (uint8_t)((status >> 16) & 0xff);
	env_data[7] = (uint8_t)((status >> 24) & 0xff);

	if (errsta) {
		diag_send_event_stat_and_env_data(
				DiagMsgPrioHigh,
				ModuleDiag_VIO,
				EventIdVioLdcErr,
				DiagEventStaFail,
				DiagGenEnvdataWhenErr,
				env_data,
				sizeof(env_data));
	}

	return;
}
#endif

/* ldc model irq
 * frame start: clear ldc_update
 * overflow:  debug ldc param info
 * output_frame_done: change setting
 * input_fram_done:
 *
 * if ldc == bypass
 *    set bypass_ldc == 1
 * else
 *    set bypass_ldc == 0
 *    set setting
 * note:
 *   if  FIRMWARE_CONTEXT_NUMBER < 4
 *       using setting[0-4]
 *   else
 *       using setting[0-1]
 */
static irqreturn_t x3_ldc_irq(int this_irq, void *data)
{
	ldc_irqstatus_u tmp_irq;

	disable_irq_nosync(this_irq);

	spin_lock(&dwe_ctx->ldclock);
	get_ldc_int_status(dwe_ctx->dev_ctx->ldc_dev->io_vaddr, &tmp_irq.status_g);
	set_ldc_int_status(dwe_ctx->dev_ctx->ldc_dev->io_vaddr, &tmp_irq.status_g);

	if (tmp_irq.status_b.frame_start == 1) {
		//debug
		uint32_t addr;
		uint32_t tmp;
		pr_debug("----ldc start----");
		pr_debug("----next port 0x%x----", dwe_ctx->ctx.ldc_next_port);
		pr_debug("----ldc_irqstatus 0x%x----", dwe_ctx->ldc_irqstatus);
		addr = 0x44;
		ldc_debug_info(dwe_ctx->dev_ctx->ldc_dev->io_vaddr, addr, &tmp);
		pr_debug("[dump] addr 0x%x, data 0x%x", addr, tmp);
		addr = 0x140;
		ldc_debug_info(dwe_ctx->dev_ctx->ldc_dev->io_vaddr, addr, &tmp);
		pr_debug("[dump] addr 0x%x, data 0x%x", addr, tmp);
		addr = 0x240;
		ldc_debug_info(dwe_ctx->dev_ctx->ldc_dev->io_vaddr, addr, &tmp);
		pr_debug("[dump] addr 0x%x, data 0x%x", addr, tmp);

		ldc_error_sts = 0;

		dwe_ctx->ldc_irqstatus = tmp_irq.status_g;

		if (dwe_ctx->ctx.ldc_running == 0) {
			dwe_ctx->ctx.ldc_running = 1;
		} else {
			dwe_ctx->ctx.ldc_running = 0;
		}
		if ((dwe_ctx->ctx.dis_running == 0) && (get_dis_status(dwe_ctx->ctx.dis_next_port))) {
			dwe_ctx->ctx.dis_running = 1;
		} else {
			dwe_ctx->ctx.dis_running = 0;
		}
		if ( dwe_ctx->ctx.dis_next_port > 0)
			dwe_ctx->ctx.dis_next_port = -1;

		if ( dwe_ctx->ctx.ldc_next_port > 0)
			dwe_ctx->ctx.ldc_next_port = -1;
	}

	if (tmp_irq.status_b.output_frame_done == 1) {
		dwe_ctx->ldc_irqstatus |= tmp_irq.status_g;
		dwe_ctx->ctx.ldc_running = 0;
		pr_debug("----ldc done----");
		//if online
		if (dwe_ctx->ctx.online_enable == 1) {
			dwe_ctx->ctx.ldc_next_port = dwe_ctx->ctx.online_port;
		}
		update_ldc_param();
		// schedule_work(&dwe_ctx->ldc_work);
	}

	if (tmp_irq.status_b.input_frame_done == 1) {
		dwe_ctx->ldc_irqstatus |= tmp_irq.status_g;
		//LOG(LOG_DEBUG, "----over_flow!----");
	}

	if ((tmp_irq.status_b.overflow == 1) ||
		(tmp_irq.status_b.frame_overwrite == 1) ||
		(tmp_irq.status_b.line_overwrite == 1) ||
		(tmp_irq.status_b.isp_in_overwrite == 1) ||
		(tmp_irq.status_b.line_buf_woi_error == 1)) {

		//set for ipu jdugement, mute woi_error
		if (tmp_irq.status_g & 0x3c)
			ldc_error_sts = 1;

		dwe_ctx->ldc_irqstatus |= tmp_irq.status_g;
		dwe_ctx->ctx.ldc_running = 0;
		//LOG(LOG_DEBUG, "----over_flow!----");
#ifdef CONFIG_HOBOT_DIAG
		ldc_diag_report((uint8_t)ldc_error_sts, tmp_irq.status_g);
#endif
	}

	spin_unlock(&dwe_ctx->ldclock);

	enable_irq(this_irq);
	return IRQ_HANDLED;
}

int check_dev(struct dwe_dev_s *check)
{
	int ret = 0;

	if (check == NULL) {
		LOG(LOG_ERR, "dwe_dev_s is null! \n");
		ret = -EINVAL;
	} else {
		if ((check->ldc_dev == NULL) || (check->dis_dev == NULL)) {
			LOG(LOG_ERR, "ldc_dev %p, dis_dev %p!\n", check->ldc_dev, check->dis_dev);
			ret = -EINVAL;
		}
	}

	return ret;
}

/*
 * init dwe hardware
 * 1. request irq
 * 2. request ion buffer
 */
int dwe_hw_init(void)
{
	int ret = 0;
	unsigned int irq = 0;
	
	ret = check_dev(dwe_ctx->dev_ctx);
	if (ret < 0) {
		LOG(LOG_ERR, "dwe_ctx->dev_ctx is error! \n");
		return ret;
	} else {
		irq = dwe_ctx->dev_ctx->ldc_dev->irq_num;
		ret = request_irq(irq, x3_ldc_irq, IRQF_TRIGGER_HIGH, "ldc", NULL);
		if (ret < 0) {
			LOG(LOG_ERR, "ldc irq %d register failed!\n", irq);
			goto irqldc_err;
		}

		vio_irq_affinity_set(irq, MOD_LDC, 0, 0);

		irq = dwe_ctx->dev_ctx->dis_dev->irq_num;
		ret = request_irq(irq, x3_dis_irq, IRQF_TRIGGER_HIGH, "dis", NULL);
		if (ret < 0) {
			LOG(LOG_ERR, "dis irq %d register failed!\n", irq);
			goto irqdis_err;
		}

		vio_irq_affinity_set(irq, MOD_DIS, 0, 0);
	}

	/* init workqueue */
	INIT_WORK(&dwe_ctx->ldc_work, ldc_task_work);
	INIT_WORK(&dwe_ctx->dis_work, dis_task_work);
	/* init spin_lock */
	spin_lock_init(&dwe_ctx->ldclock);
	spin_lock_init(&dwe_ctx->dislock);

	ret = dwe_init_api(&dwe_ctx->ctx, dwe_ctx->dev_ctx, &dwe_ctx->ptr_param);
	if (ret < 0) {
		LOG(LOG_ERR, "dwe_init_api is failed!\n");
		goto irq_err;
	} else {
		LOG(LOG_INFO, "dwe_ctx->ptr_param is %p\n", dwe_ctx->ptr_param);
	}

	return ret;

irq_err:
	free_irq(dwe_ctx->dev_ctx->dis_dev->irq_num, NULL);
irqdis_err:
	free_irq(dwe_ctx->dev_ctx->ldc_dev->irq_num, NULL);
irqldc_err:
	return ret;
}

/*
 * deinit dwe hardware
 * 1. free irq
 * 2. free ion buffer
 */
void dwe_hw_deinit(void)
{
	int ret = 0;
	unsigned int irq = 0;

	ret = check_dev(dwe_ctx->dev_ctx);
	if (ret < 0) {
		LOG(LOG_INFO, "dwe_ctx->dev_ctx is error! \n");
	} else {
		irq = dwe_ctx->dev_ctx->ldc_dev->irq_num;
		free_irq(irq, NULL);
		
		irq = dwe_ctx->dev_ctx->dis_dev->irq_num;
		free_irq(irq, NULL);
	}

	dwe_deinit_api(&dwe_ctx->ctx);
	dwe_ctx->ptr_param = NULL;
}

static int soc_dwe_log_status(struct v4l2_subdev *sd)
{
	LOG(LOG_DEBUG, "log status called");
	return 0;
}

static int soc_dwe_init(struct v4l2_subdev *sd, u32 val)
{
	int rc = 0;

	if (val >= FIRMWARE_CONTEXT_NUMBER) {
		rc = -EINVAL;
	}

	return rc;
}

static int soc_dwe_reset(struct v4l2_subdev *sd, u32 val)
{
	int rc = 0;
	if (val >= FIRMWARE_CONTEXT_NUMBER) {
		rc = -EINVAL;
	}

	return rc;
}

static long soc_dwe_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	int rc = 0;

	if (ARGS_TO_PTR(arg)->ctx_num >= FIRMWARE_CONTEXT_NUMBER) {
		LOG(LOG_ERR, "Failed to control dwe_ioctl :%d\n", ARGS_TO_PTR(arg)->ctx_num);
		return -1;
	}

	switch (cmd) {
	case SOC_DWE_SET_LDC:
		rc = ldc_set_ioctl(ARGS_TO_PTR(arg)->ctx_num, 0);
		break;
	case SOC_DWE_GET_LDC:
		break;
	case SOC_DWE_SET_DIS:
		rc = dis_set_ioctl(ARGS_TO_PTR(arg)->ctx_num, 0);
		break;
	case SOC_DWE_GET_DIS:
		break;
	default:
		LOG(LOG_WARNING, "Unknown lens ioctl cmd %d", cmd);
		rc = -1;
		break;
	};
	return (long)rc;
}

static const struct v4l2_subdev_core_ops core_ops = {
	.log_status = soc_dwe_log_status,
	.init = soc_dwe_init,
	.reset = soc_dwe_reset,
	.ioctl = soc_dwe_ioctl,
};

static const struct v4l2_subdev_ops dwe_ops = {
	.core = &core_ops,
};

static int dwe_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	int ret = 0;
	LOG(LOG_ERR, "+++++start dwe subdev control+++++!");

	return ret;
}

static int dwe_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	int ret = 0;
	LOG(LOG_ERR, "-----stop dwe subdev control-----!");

	return ret;
}

static const struct v4l2_subdev_internal_ops dwe_int_ops = {
        .open = dwe_open,
        .close = dwe_close,
};

static int32_t soc_dwe_probe(struct platform_device *pdev)
{
	int32_t rc = 0;

	dwe_ctx = kzalloc(sizeof(subdev_dwe_ctx), GFP_KERNEL);
	if (dwe_ctx == NULL) {
		LOG(LOG_ERR, "kzalloc is failed!");
		return -ENOMEM;
	}

	v4l2_subdev_init(&dwe_ctx->soc_dwe, &dwe_ops);

	dwe_ctx->soc_dwe.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	snprintf(dwe_ctx->soc_dwe.name, V4L2_SUBDEV_NAME_SIZE,
		"%s", V4L2_SOC_DWE_NAME);

	dwe_ctx->soc_dwe.dev = &pdev->dev;
	dwe_ctx->soc_dwe.internal_ops = &dwe_int_ops;
	rc = v4l2_async_register_subdev(&dwe_ctx->soc_dwe);

#ifdef CONFIG_HOBOT_DIAG
	if (diag_register(ModuleDiag_VIO, EventIdVioLdcErr,
					4, 74, DIAG_MSG_INTERVAL_MAX, NULL) < 0) {
		vio_err("LDC diag register fail\n");
	}
#endif

	LOG(LOG_DEBUG, "register v4l2 lens device. result %d", rc);

	return rc;
}

static int soc_dwe_remove(struct platform_device *pdev)
{
	v4l2_async_unregister_subdev(&dwe_ctx->soc_dwe);
	kfree(dwe_ctx);
	dwe_ctx = NULL;

	return 0;
}

static struct platform_device *soc_dwe_dev;

static int dwe_suspend(struct device *dev)
{
	int ret = 0;

	pr_info("enter\n");
	ret = check_dev(dwe_ctx->dev_ctx);
	if (ret < 0) {
		pr_err("dwe_ctx->dev_ctx is error! \n");
		return 0;
	} else {
		vio_irq_affinity_set(dwe_ctx->dev_ctx->ldc_dev->irq_num, MOD_LDC, 1, 0);
		vio_irq_affinity_set(dwe_ctx->dev_ctx->dis_dev->irq_num, MOD_DIS, 1, 0);
	}

	return ret;
}

static int dwe_resume(struct device *dev)
{
	int ret = 0;

	pr_info("enter\n");
	ret = check_dev(dwe_ctx->dev_ctx);
	if (ret < 0) {
		pr_err("dwe_ctx->dev_ctx is error! \n");
		return 0;
	} else {
		vio_irq_affinity_set(dwe_ctx->dev_ctx->ldc_dev->irq_num, MOD_LDC, 0, 0);
		vio_irq_affinity_set(dwe_ctx->dev_ctx->dis_dev->irq_num, MOD_DIS, 0, 0);
	}

	return ret;
}

static int dwe_runtime_suspend(struct device *dev)
{
	int ret = 0;

	pr_info("enter\n");

	return ret;
}

static int dwe_runtime_resume(struct device *dev)
{
	int ret = 0;

	pr_info("enter\n");

	return ret;
}

static const struct dev_pm_ops dwe_pm_ops = {
	.suspend = dwe_suspend,
	.resume = dwe_resume,
	.runtime_suspend = dwe_runtime_suspend,
	.runtime_resume = dwe_runtime_resume,
};

static struct platform_driver soc_dwe_driver = {
	.probe = soc_dwe_probe,
	.remove = soc_dwe_remove,
	.driver = {
		.name = "soc_dwe_v4l2",
		.owner = THIS_MODULE,
		.pm = &dwe_pm_ops,
	},
};

extern int __init system_dwe_init(struct dwe_dev_s **ptr);
extern void __exit system_dwe_exit(void);

extern int __init dwe_dev_init(uint32_t port);
extern void __exit dwe_dev_exit(int port);

static void chardevs_exit(void)
{
	uint32_t tmp = 0;
	
	for (tmp = 0; tmp < FIRMWARE_CONTEXT_NUMBER; tmp++) {
		dwe_dev_exit(tmp);
	}
}

static int chardevs_init(void)
{
	int ret = 0;
	uint32_t tmp = 0;
	
	for (tmp = 0; tmp < FIRMWARE_CONTEXT_NUMBER; tmp++) {
		ret = dwe_dev_init(tmp);
		if (ret < 0) {
			LOG(LOG_ERR, "dwe_dev_init %d is failed\n", tmp);
			goto devinit_err;
		}
	}

	return ret;
devinit_err:
	chardevs_exit();
	return ret;
}

int __init acamera_soc_dwe_init(void)
{
	int rc = 0;

	LOG(LOG_DEBUG, "[KeyMsg] dwe subdevice init");

	soc_dwe_dev = platform_device_register_simple(
		"soc_dwe_v4l2", -1, NULL, 0);
	rc = platform_driver_register(&soc_dwe_driver);
	if (rc == 0) {
		//get dev info
		rc = system_dwe_init(&dwe_ctx->dev_ctx);
		if (rc == 0) {
			//dwe hardward init
			LOG(LOG_INFO, "system_dwe_init is success!\n");
			rc = dwe_hw_init();
			if (rc < 0) {
				LOG(LOG_ERR, "dwe_hw_init is failed\n");
				goto init_err;
			}
			rc = chardevs_init();
			if (rc < 0) {
				LOG(LOG_ERR, "dwe_dev_init is failed\n");
				goto devinit_err;
			}
		}
	}
	LOG(LOG_DEBUG, "[KeyMsg] dwe subdevice init done: %d", rc);

	return rc;
devinit_err:
	dwe_hw_deinit();
init_err:
	system_dwe_exit();
	platform_driver_unregister(&soc_dwe_driver);
	platform_device_unregister(soc_dwe_dev);
	return rc;
}

void __exit acamera_soc_dwe_exit(void)
{
	LOG(LOG_DEBUG, "[KeyMsg] dwe subdevice exit");

	dwe_hw_deinit();
	chardevs_exit();
	platform_driver_unregister(&soc_dwe_driver);
	platform_device_unregister(soc_dwe_dev);
	system_dwe_exit();
	LOG(LOG_DEBUG, "[KeyMsg] dwe subdevice exit done");
}

late_initcall(acamera_soc_dwe_init); /*PRQA S 0605 ++*/
module_exit(acamera_soc_dwe_exit); /*PRQA S 0605 ++*/

MODULE_AUTHOR("IE&E");
MODULE_LICENSE("GPL v2");
