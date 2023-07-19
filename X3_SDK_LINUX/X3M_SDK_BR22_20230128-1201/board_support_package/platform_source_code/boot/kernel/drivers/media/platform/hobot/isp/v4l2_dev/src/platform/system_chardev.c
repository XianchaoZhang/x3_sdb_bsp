/*
*
* SPDX-License-Identifier: GPL-2.0
*
* Copyright (C) 2011-2018 ARM or its affiliates
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; version 2.
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
* or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
* for more details.
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*
*/

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/rtmutex.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/kfifo.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/string.h>
#include <linux/ioctl.h>
#include <linux/uaccess.h>
#include <linux/ion.h>
#include <linux/crc16.h>
#include "system_chardev.h"
#include "acamera_logger.h"
#include "acamera_tuning.h"
#include "acamera_fw.h"
#include "acamera_command_api.h"
#include "general_fsm.h"
#include "isp_ctxsv.h"


#define SYSTEM_CHARDEV_FIFO_SIZE (50*1024)
#define SYSTEM_CHARDEV_NAME "ac_isp"

#define ISPIOC_REG_RW   _IOWR('P', 0, struct metadata_t)
#define ISPIOC_LUT_RW   _IOWR('P', 1, struct metadata_t)
#define ISPIOC_COMMAND  _IOWR('P', 2, struct metadata_t)
#define ISPIOC_BUF_PACTET  _IOWR('P', 3, isp_packet_s)
#define ISPIOC_GET_CTX _IOWR('P', 4, isp_ctx_r_t)
#define ISPIOC_PUT_CTX _IOWR('P', 5, isp_ctx_r_t)
#define ISPIOC_FILL_CTX _IOWR('P', 6, isp_ctx_w_t)
#define ISPIOC_REG_MEM_RW _IOWR('P', 7, struct regs_mem_t)
#define ISPIOC_IRQ_WAIT _IOWR('P', 8, isp_irq_wait_s)
#define ISPIOC_STA_CTRL _IOWR('P', 9, isp_sta_ctrl_t)
#define ISPIOC_GET_CTX_CONDITIONAL _IOWR('P', 10, isp_ctx_r_t)

#define CHECK_CODE	0xeeff

typedef struct {
	isp_ctx_r_t isp_ctx;
	int flag;
} isp_sta_ctrl_t;

struct metadata_t {
        void *ptr;
        uint8_t chn;
        uint8_t dir;
        uint8_t id;
        uint32_t elem;
};

// for register
struct regs_t {
        uint32_t addr;
        uint8_t m;
        uint8_t n;
        uint32_t v;
};

// for register memory
struct regs_mem_t {
	uint32_t addr;	// register address
	uint8_t chn;	// pipeline id
	uint8_t dir;	// read or write
	uint32_t size;	//sizeof data
	uint32_t *ptr;	// pointer to data
};

// for command
struct kv_t {
        uint32_t k;
        uint32_t v;
};

typedef struct _isp_packet_s {
        uint32_t buf[5];
        void *pdata;
} isp_packet_s;

typedef struct _isp_irq_wait_s {
	uint8_t ctx_id;
	uint8_t irq_type;
	uint64_t time_out;
} isp_irq_wait_s;

#define BUF_LENGTH  1024

struct isp_dev_context {
    uint8_t dev_inited;
    int dev_minor_id;
    char *dev_name;
    int dev_opened;

    // for ctx dump
    struct ion_client *client;
    struct ion_handle *handle;
    void *vir_addr;
    phys_addr_t phy_addr;
    size_t mem_size;

    struct miscdevice isp_dev;
    struct rt_mutex fops_lock;

    struct kfifo isp_kfifo_in;
    struct kfifo isp_kfifo_out;

    wait_queue_head_t kfifo_in_queue;
    wait_queue_head_t kfifo_out_queue;
};

extern struct ion_device *hb_ion_dev;
extern int32_t acamera_set_api_context(uint32_t ctx_num);
extern void system_reg_rw(struct regs_t *rg, uint8_t dir);
extern void *acamera_get_ctx_ptr(uint32_t ctx_id);
extern int isp_temper_set_addr(general_fsm_ptr_t p_fsm);

/* First item for ACT Control, second for user-FW */
static struct isp_dev_context isp_dev_ctx;
int isp_dev_mem_alloc(void);

int system_chardev_lock(void)
{
	int rc = 0;

    rc = rt_mutex_lock_interruptible( &isp_dev_ctx.fops_lock );
    if ( rc ) {
        LOG( LOG_ERR, "Error: lock failed of dev: %s.", isp_dev_ctx.dev_name );
        return rc;
    }

	return rc;
}

void system_chardev_unlock(void)
{
	rt_mutex_unlock( &isp_dev_ctx.fops_lock );
}

static int isp_fops_open( struct inode *inode, struct file *f )
{
    int rc = 0;
    struct isp_dev_context *p_ctx = NULL;
    int minor = iminor( inode );

    LOG( LOG_INFO, "client is opening..., minor: %d.", minor );

    if ( !isp_dev_ctx.dev_inited ) {
        LOG( LOG_ERR, "dev is not inited, failed to open." );
        return -ERESTARTSYS;
    }

    if ( minor == isp_dev_ctx.dev_minor_id )
        p_ctx = &isp_dev_ctx;

    if ( !p_ctx ) {
        LOG( LOG_ERR, "Fatal error, isp dev contexts is wrong, contents dump:" );
        LOG( LOG_ERR, "    minor_id: %d, name: %s.", isp_dev_ctx.dev_minor_id, isp_dev_ctx.dev_name );

        return -ERESTARTSYS;
    }

    rc = rt_mutex_lock_interruptible( &p_ctx->fops_lock );
    if ( rc ) {
        LOG( LOG_ERR, "Error: lock failed of dev: %s.", p_ctx->dev_name );
        goto lock_failure;
    }

	if (p_ctx->dev_opened == 0) {
		rc = isp_dev_mem_alloc();
		if (rc) {
			pr_err("ac_isp alloc mem failed. rc %d\n", rc);
			rt_mutex_unlock( &p_ctx->fops_lock );
			return rc;
		}

		isp_ctx_queue_init();
	}

    f->private_data = p_ctx;
    p_ctx->dev_opened++;

    LOG( LOG_INFO, "open(%s) succeed.", p_ctx->dev_name );

#if 0
    if ( p_ctx->dev_opened ) {
        LOG( LOG_ERR, "open(%s) failed, already opened.", p_ctx->dev_name );
        rc = -EBUSY;
    } else {
        p_ctx->dev_opened = 1;
        rc = 0;
        LOG( LOG_INFO, "open(%s) succeed.", p_ctx->dev_name );

        LOG( LOG_INFO, "Bf set, private_data: %p.", f->private_data );
        f->private_data = p_ctx;
        LOG( LOG_INFO, "Af set, private_data: %p.", f->private_data );
    }
#endif
    rt_mutex_unlock( &p_ctx->fops_lock );

lock_failure:
    return rc;
}

static int isp_fops_release( struct inode *inode, struct file *f )
{
    int i;
    struct isp_dev_context *p_ctx = (struct isp_dev_context *)f->private_data;
	acamera_context_t *ptr;

    if ( p_ctx != &isp_dev_ctx ) {
        LOG( LOG_ERR, "Inalid paramter: %p.", p_ctx );
        return -EINVAL;
    }

    rt_mutex_lock(&p_ctx->fops_lock);

	//free memory later, disable all contexts isp info dump
	for (i = 0; i < FIRMWARE_CONTEXT_NUMBER; i++) {
		ptr = acamera_get_ctx_ptr(i);
		ptr->isp_ctxsv_on = 0;
		ptr->isp_awb_stats_on = 0;
		ptr->isp_ae_stats_on = 0;
		ptr->isp_af_stats_on = 0;
		ptr->isp_ae_5bin_stats_on = 0;
		ptr->isp_lumvar_stats_on = 0;
	}

    p_ctx->dev_opened--;

    if ( p_ctx->dev_opened <= 0 ) {
        p_ctx->dev_opened = 0;
        f->private_data = NULL;
        kfifo_reset( &p_ctx->isp_kfifo_in );
        kfifo_reset( &p_ctx->isp_kfifo_out );

		//free ion memory
		if (IS_ERR(isp_dev_ctx.client) == 0) {
			if (IS_ERR(isp_dev_ctx.handle) == 0) {
				ion_unmap_kernel(isp_dev_ctx.client, isp_dev_ctx.handle);
				ion_free(isp_dev_ctx.client, isp_dev_ctx.handle);
				isp_dev_ctx.handle = NULL;
				p_ctx->phy_addr = 0;
				p_ctx->mem_size = 0;
				p_ctx->vir_addr = 0;
			}
			ion_client_destroy(isp_dev_ctx.client);
			isp_dev_ctx.client = NULL;
		}
    } else {
        pr_info("device name is %s, dev_opened reference count: %d.\n", p_ctx->dev_name, p_ctx->dev_opened);
    }

    rt_mutex_unlock( &p_ctx->fops_lock );

    return 0;
}

static ssize_t isp_fops_write( struct file *file, const char __user *buf, size_t count, loff_t *ppos )
{
    int rc;
    unsigned int copied;
    struct isp_dev_context *p_ctx = (struct isp_dev_context *)file->private_data;

    /* isp_dev_uf device only support read at the moment */
    if ( p_ctx != &isp_dev_ctx ) {
        LOG( LOG_ERR, "Inalid paramter: %p.", p_ctx );
        return -EINVAL;
    }

    if ( rt_mutex_lock_interruptible( &p_ctx->fops_lock ) ) {
        LOG( LOG_ERR, "Fatal error: access lock failed." );
        return -ERESTARTSYS;
    }

    rc = kfifo_from_user( &p_ctx->isp_kfifo_in, buf, count, &copied );

    /* awake any reader */
    wake_up_interruptible( &p_ctx->kfifo_in_queue );

    LOG( LOG_DEBUG, "wake up reader." );

    rt_mutex_unlock( &p_ctx->fops_lock );

    return rc ? rc : copied;
}

static ssize_t isp_fops_read( struct file *file, char __user *buf, size_t count, loff_t *ppos )
{
    int rc;
    unsigned int copied;
    struct isp_dev_context *p_ctx = (struct isp_dev_context *)file->private_data;

    if ( p_ctx != &isp_dev_ctx ) {
        LOG( LOG_ERR, "Inalid paramter: %p.", p_ctx );
        return -EINVAL;
    }

    if ( rt_mutex_lock_interruptible( &p_ctx->fops_lock ) ) {
        LOG( LOG_ERR, "Fatal error: access lock failed." );
        return -ERESTARTSYS;
    }

    if ( kfifo_is_empty( &p_ctx->isp_kfifo_out ) ) {
        long time_out_in_jiffies = 1;      /* jiffies is depend on HW, in x86 Ubuntu, it's 4 ms, 1 is 4ms. */
        rt_mutex_unlock( &p_ctx->fops_lock ); /* unlock before we return or go sleeping */

        /* return if it's a non-blocking reading  */
        if ( file->f_flags & O_NONBLOCK )
            return -EAGAIN;

        /* wait for the event */
        LOG( LOG_DEBUG, "input FIFO is empty, wait for data, timeout_in_jiffies: %ld, HZ: %d.", time_out_in_jiffies, HZ );
	rc = wait_event_interruptible(p_ctx->kfifo_out_queue,
		!kfifo_is_empty(&p_ctx->isp_kfifo_out));
        LOG( LOG_DEBUG, "data is coming or timeout, kfifo_out size: %u, rc: %d.", kfifo_len( &p_ctx->isp_kfifo_out ), rc );

        /* after wake up, we need to re-gain the mutex */
        if ( rt_mutex_lock_interruptible( &p_ctx->fops_lock ) ) {
            LOG( LOG_ERR, "Fatal error: access lock failed." );
            return -ERESTARTSYS;
        }
    }

    rc = kfifo_to_user( &p_ctx->isp_kfifo_out, buf, count, &copied );
    rt_mutex_unlock( &p_ctx->fops_lock );

    return rc ? rc : copied;
}

#if 0
/** move to isp_fops_ioctl*/
static long isp_fops_ioctl_bak(struct file *pfile, unsigned int cmd,
	unsigned long arg)
{
	long ret = 0;
	isp_packet_s packet;
	uint32_t buf[BUF_LENGTH];
	uint32_t *buf_m = NULL;

	LOG(LOG_DEBUG, "---[%s-%d]---\n", __func__, __LINE__);

	switch (cmd) {
	case DISP_BUF_PACTET: {
		if (arg == 0) {
			LOG(LOG_ERR, "arg is null !\n");
			return -1;
		}
		if (copy_from_user((void *)&packet, (void __user *)arg,
			sizeof(isp_packet_s))) {
			LOG(LOG_ERR, "copy is err !\n");
			return -EINVAL;
		}
		if (packet.buf[0] < BUF_LENGTH * 4) {
			buf_m = buf;
		} else {
			buf_m = kzalloc(sizeof(uint32_t) * packet.buf[0], GFP_KERNEL);
			if (buf_m == NULL) {
				LOG(LOG_ERR, "kzalloc is failed!\n");
				return -EINVAL;
			}
		}
		memcpy(buf_m, packet.buf, sizeof(packet.buf));
		if (packet.pdata != NULL) {
			if (copy_from_user((void *)(buf_m + (sizeof(packet.buf)
				/ sizeof(uint32_t))), (void __user *)packet.pdata, packet.buf[4])) {
				LOG(LOG_ERR, "copy is err !\n");
				ret = -EINVAL;
				goto err_flag;
			}
		}
		process_ioctl_buf(buf_m);
		if (packet.pdata != NULL) {
			if (copy_to_user((void __user *)packet.pdata,
				(void *)(buf_m + (sizeof(packet.buf)
				/ sizeof(uint32_t))), packet.buf[4])) {
				LOG(LOG_ERR, "copy is err !\n");
				ret = -EINVAL;
				goto err_flag;
			}
		}
		if (copy_to_user((void __user *)arg, (void *)buf_m, sizeof(isp_packet_s))) {
			LOG(LOG_ERR, "copy is err !\n");
			ret = -EINVAL;
		}
err_flag:
		if (packet.buf[0] >= (BUF_LENGTH * 4)) {
			kzfree(buf_m);
		}
	}
	break;
	default: {
		LOG(LOG_ERR, "---cmd is err---\n");
		ret = -1;
	}
	break;
	}

	return ret;
}
#endif

static long isp_fops_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	uint32_t ret_value = 0;
	struct metadata_t md = {0};

	// to store metadata_t from user space and will be assigned later
	struct metadata_t md_user;
	struct metadata_t *pmd = (struct metadata_t *)&md_user;

	if (!isp_dev_ctx.dev_inited) {
		LOG(LOG_ERR, "dev is not inited, failed to ioctl.");
		return -1;
	}

	if (cmd != ISPIOC_IRQ_WAIT) {
		rt_mutex_lock(&isp_dev_ctx.fops_lock);
	}

	switch (cmd) {
	case ISPIOC_REG_RW:
	case ISPIOC_LUT_RW:
	case ISPIOC_COMMAND:
	{
		acamera_context_t *p_ctx;

		if (copy_from_user(&md, (void __user *)arg, sizeof(md))) {
			pr_err("copy_from_user error\n");
			ret = -EFAULT;
			break;
		}

		md_user = md;
		if (md.chn >= FIRMWARE_CONTEXT_NUMBER) {
			pr_err("ctx id %d exceed valid range\n", md.chn);
			ret = -EFAULT;
			break;
		}

		p_ctx = (acamera_context_t *)acamera_get_ctx_ptr(md.chn);
		if (p_ctx->initialized == 0) {
			pr_err("%d ctx is not inited.\n", md.chn);
			ret = -EFAULT;
		} else {
			acamera_set_api_context(md.chn);
		}
	}
		break;
	case ISPIOC_BUF_PACTET:
	case ISPIOC_GET_CTX:
	case ISPIOC_PUT_CTX:
	case ISPIOC_FILL_CTX:
	case ISPIOC_REG_MEM_RW:
	case ISPIOC_IRQ_WAIT:
	case ISPIOC_STA_CTRL:
	case ISPIOC_GET_CTX_CONDITIONAL:
		break;
	default:
		pr_err("command %d not support.\n", cmd);
		break;
	}

	if (ret < 0)
		goto out;

	switch (cmd) {
	case ISPIOC_REG_RW:
	{
        uint8_t i = 0;
		uint32_t s = 0;
		struct regs_t *rg;

		s = (uint32_t)(md.elem * sizeof(struct regs_t));
		md.ptr = kzalloc(s, GFP_KERNEL);
		if (md.ptr == NULL) {
			pr_err("ctx id %d, elem alloc mem failed\n", md.chn);
			ret = -ENOMEM;
			break;
		}

		if (copy_from_user(md.ptr, pmd->ptr, s)) {
			pr_err("ctx id %d, copy_from_user error\n", md.chn);
			ret = -EFAULT;
			kfree(md.ptr);
			break;
		}

		rg = md.ptr;

		for (i = 0; i < md.elem; i++) {
			system_reg_rw(&rg[i], md.dir);
			pr_debug("ctx id %d, dir %d, elem %d, addr %x, m %d, n %d, v %d\n",
					md.chn, md.dir, md.elem, rg[i].addr, rg[i].m, rg[i].n, rg[i].v);
		}

		if (md.dir == COMMAND_GET) {
			if (copy_to_user(pmd->ptr, md.ptr, s)) {
				pr_err("ctx id %d, copy_to_user error\n", md.chn);
				ret = -EFAULT;
			}
		}

		kfree(md.ptr);
	}
		break;

	case ISPIOC_LUT_RW:
	{
		uint32_t s = 0;

		// s = md.elem & 0xffff;
		s = _GET_SIZE((acamera_context_t *)acamera_get_ctx_ptr(md.chn), md.id);
		if (s == 0) {
			pr_err("calibration id %d is not exist.\n", md.id);
			goto out;
		}
		md.ptr = kzalloc(s, GFP_KERNEL);
		if (md.ptr == NULL) {
			pr_err("ctx id %d, md ptr alloc mem failed\n", md.chn);
			ret = -ENOMEM;
			break;
		}

		if (copy_from_user(md.ptr, pmd->ptr, s)) {
			pr_err("ctx id %d, copy_from_user error\n", md.chn);
			ret = -EFAULT;
			kfree(md.ptr);
			break;
		}

		ret = acamera_api_calibration(md.chn, 0, md.id, md.dir, md.ptr, s, &ret_value);
		if (ret == SUCCESS && md.dir == COMMAND_GET) {
			if (copy_to_user(pmd->ptr, md.ptr, s)) {
				pr_err("ctx id %d, copy_to_user error\n", md.chn);
				ret = -EFAULT;
			}
		}

		kfree(md.ptr);
	}
		break;
	case ISPIOC_REG_MEM_RW:
	{
        uint16_t i = 0;
		acamera_context_ptr_t p_ctx;
		struct regs_mem_t rgm;
		struct regs_mem_t *prgm = (struct regs_mem_t *)arg;

		if (copy_from_user(&rgm, (void __user *)arg, sizeof(rgm))) {
			pr_err("ISPIOC_REG_MEM_RW copy_from_user error\n");
			ret = -EFAULT;
			break;
		}

		if (rgm.chn >= FIRMWARE_CONTEXT_NUMBER) {
			pr_err("ctx id %d exceed valid range\n", rgm.chn);
			ret = -EFAULT;
			break;
		}

		p_ctx = (acamera_context_t *)acamera_get_ctx_ptr(rgm.chn);
		if (p_ctx->initialized == 0) {
			pr_err("%d ctx is not inited.\n", rgm.chn);
			ret = -EFAULT;
			break;
		}

		acamera_set_api_context(rgm.chn);

		rgm.ptr = vzalloc(rgm.size);
		if (rgm.ptr == NULL) {
			pr_err("ctx id %d, rgm ptr alloc mem failed\n", rgm.chn);
			ret = -ENOMEM;
			break;
		}

		if (copy_from_user(rgm.ptr, prgm->ptr, rgm.size)) {
			pr_err("ctx id %d, copy_from_user error\n", rgm.chn);
			ret = -EFAULT;
			vfree(rgm.ptr);
			break;
		}

		uintptr_t sw_addr = p_ctx->settings.isp_base + rgm.addr;
		for (i = 0; i < rgm.size / sizeof(uint32_t); i++) {
			if (rgm.dir == COMMAND_SET) {
				system_sw_write_32(sw_addr + i * 4, rgm.ptr[i]);
			} else {
				rgm.ptr[i] = system_sw_read_32(sw_addr + i * 4);
			}
			pr_debug("ctx id %d, dir %d, addr %p, val %x\n", rgm.chn, rgm.dir, (uint32_t *)(sw_addr + i * 4), rgm.ptr[i]);
		}

		if (rgm.dir == COMMAND_GET) {
			if (copy_to_user(prgm->ptr, rgm.ptr, rgm.size)) {
				pr_err("ctx id %d, copy_to_user error\n", rgm.chn);
				ret = -EFAULT;
			}
		}

		vfree(rgm.ptr);
	}
		break;
	case ISPIOC_COMMAND:
	{
        	uint8_t i = 0;
		uint8_t type = 0xff;
		uint32_t s = 0;
		struct kv_t *kv;

		s = (uint32_t)(md.elem * sizeof(struct kv_t));
		md.ptr = kzalloc(s, GFP_KERNEL);
		if (md.ptr == NULL) {
			pr_err("ctx id %d, md ptr alloc mem failed\n", md.chn);
			ret = -ENOMEM;
			break;
		}

		if (copy_from_user(md.ptr, pmd->ptr, s)) {
			pr_err("ctx id %d, copy_from_user error\n", md.chn);
			ret = -EFAULT;
			kfree(md.ptr);
			break;
		}

		kv = md.ptr;

		for (i = 0; i < md.elem; i++) {
			if (kv[i].v == CHECK_CODE) {
				type = (uint8_t)kv[i].k;
				continue;
			}

			ret = acamera_command(md.chn, type, (uint8_t)kv[i].k, kv[i].v, md.dir, &ret_value);
			if (ret == SUCCESS && md.dir == COMMAND_GET)
				kv[i].v = ret_value;

			pr_debug("chn %d, k %d, v %d, dir %d, ret_value %d\n",
				md.chn, kv[i].k, kv[i].v, md.dir, ret_value);
		}

		if (md.dir == COMMAND_GET) {
			if (copy_to_user(pmd->ptr, md.ptr, s)) {
				pr_err("ctx id %d, copy_to_user error\n", md.chn);
				ret = -EFAULT;
			}
		}

		kfree(md.ptr);
	}
		break;

	case ISPIOC_GET_CTX:
	{
		isp_ctx_r_t ctx;
		isp_ctx_node_t *cn = NULL;
		acamera_context_t *p_ctx;

		if (copy_from_user(&ctx, (void __user *)arg, sizeof(ctx))) {
			pr_err("ISPIOC_GET_CTX copy_from_user error\n");
			ret = -EFAULT;
			break;
		}

		// isp context save on
		p_ctx = (acamera_context_t *)acamera_get_ctx_ptr(ctx.ctx_id);
		if (p_ctx->initialized == 0) {
			pr_err("%d ctx is not inited.\n", ctx.ctx_id);
			ret = -EFAULT;
			break;
		}
		if (ctx.type == ISP_CTX)
			p_ctx->isp_ctxsv_on = 1;
		else if (ctx.type == ISP_AE)
			p_ctx->isp_ae_stats_on = 1;
		else if (ctx.type == ISP_AWB)
			p_ctx->isp_awb_stats_on = 1;
		else if (ctx.type == ISP_AF)
			p_ctx->isp_af_stats_on = 1;
		else if (ctx.type == ISP_AE_5BIN)
			p_ctx->isp_ae_5bin_stats_on = 1;
		else if (ctx.type == ISP_LUMVAR)
			p_ctx->isp_lumvar_stats_on = 1;

		cn = isp_ctx_get(ctx.ctx_id, ctx.type, ctx.time_out, ctx.latest_flag);
		if (cn) {
			if (copy_to_user((void __user *)arg, (void *)&cn->ctx, sizeof(ctx))) {
				pr_err("ctx id %d, copy_to_user error\n", ctx.ctx_id);
				ret = -EFAULT;
			}
		} else {
			pr_err("ctx id %d, type %d, node is null\n", ctx.ctx_id, ctx.type);
			ret = -EFAULT;
		}
	}
		break;
	case ISPIOC_GET_CTX_CONDITIONAL:
	{
		isp_ctx_r_t ctx;
		isp_ctx_node_t *cn = NULL;
		acamera_context_t *p_ctx;

		if (copy_from_user(&ctx, (void __user *)arg, sizeof(ctx))) {
			pr_err("ISPIOC_GET_CTX_CONDITIONAL copy_from_user error\n");
			ret = -EFAULT;
			break;
		}

		// isp context save on
		p_ctx = (acamera_context_t *)acamera_get_ctx_ptr(ctx.ctx_id);
		if (p_ctx->initialized == 0) {
			pr_err("%d ctx is not inited.\n", ctx.ctx_id);
			ret = -EFAULT;
			break;
		}

		cn = isp_ctx_get_conditional(ctx.ctx_id, ctx.type, ctx.frame_id, ctx.time_out);
		if (cn) {
			if (copy_to_user((void __user *)arg, (void *)&cn->ctx, sizeof(ctx))) {
				pr_err("ctx id %d, copy_to_user error\n", ctx.ctx_id);
				ret = -EFAULT;
			}
		} else {
			pr_err("ctx id %d, type %d, node is null\n", ctx.ctx_id, ctx.type);
			ret = -EFAULT;
		}
	}
		break;

	case ISPIOC_PUT_CTX:
	{
		isp_ctx_r_t ctx;
		acamera_context_t *p_ctx;
		if (copy_from_user(&ctx, (void __user *)arg, sizeof(ctx))) {
			pr_err("ISPIOC_PUT_CTX copy_from_user error\n");
			ret = -EFAULT;
			break;
		}
		p_ctx = (acamera_context_t *)acamera_get_ctx_ptr(ctx.ctx_id);
		if (p_ctx->initialized == 0) {
			pr_err("%d ctx is not inited.\n", ctx.ctx_id);
			ret = -EFAULT;
			break;
		}
		isp_ctx_put(ctx.ctx_id, ctx.type, ctx.idx);
	}
		break;

	case ISPIOC_FILL_CTX:
	{
		volatile void *ptr;
		uint32_t crc = 0;
		isp_ctx_w_t ctx;
		acamera_context_t *p_ctx;

		if (copy_from_user(&ctx, (void __user *)arg, sizeof(ctx))) {
			ret = -EFAULT;
			break;
		}

		crc = crc16(~0, ctx.ptr, CTX_SIZE);
		if (crc != ctx.crc16) {
			LOG(LOG_ERR, "isp context set failed.");
			ret = -EFAULT;
			break;
		}

		p_ctx = (acamera_context_t *)acamera_get_ctx_ptr(ctx.ctx_id);
		if (p_ctx->initialized == 0) {
			pr_err("%d ctx is not inited.\n", ctx.ctx_id);
			ret = -EFAULT;
			break;
		}
		ptr = p_ctx->sw_reg_map.isp_sw_config_map + CTX_OFFSET;
		memcpy_toio(ptr, ctx.ptr, CTX_SIZE);
		isp_temper_set_addr((general_fsm_ptr_t)(p_ctx->fsm_mgr.fsm_arr[FSM_ID_GENERAL]->p_fsm));
	}
		break;

	case ISPIOC_BUF_PACTET: {
		isp_packet_s packet;
		uint32_t *buf = NULL;
		uint32_t *buf_m = NULL;

		buf = vzalloc(BUF_LENGTH * sizeof(uint32_t));
		if (buf == NULL) {
			LOG(LOG_ERR, "ISPIOC_BUF_PACTET buf alloc failed!\n");
			break;
		}

		if (arg == 0) {
			LOG(LOG_ERR, "arg is null !\n");
			ret = -1;
			goto buf_free;
		}
		if (copy_from_user((void *)&packet, (void __user *)arg,
			sizeof(isp_packet_s))) {
			LOG(LOG_ERR, "copy is err !\n");
			ret = -EINVAL;
			goto buf_free;
		}
		if (packet.buf[0] < BUF_LENGTH * 4) {
			buf_m = buf;
		} else {
			buf_m = kzalloc(sizeof(uint32_t) * packet.buf[0], GFP_KERNEL);
			if (buf_m == NULL) {
				LOG(LOG_ERR, "kzalloc is failed!\n");
				ret = -EINVAL;
				goto buf_free;
			}
		}
		memcpy(buf_m, packet.buf, sizeof(packet.buf));
		if (packet.pdata != NULL) {
			if (copy_from_user((void *)(buf_m + (sizeof(packet.buf)
				/ sizeof(uint32_t))), (void __user *)packet.pdata, packet.buf[4])) {
				LOG(LOG_ERR, "copy is err !\n");
				ret = -EINVAL;
				goto err_flag;
			}
		}
		process_ioctl_buf(buf_m);
		if (packet.pdata != NULL) {
			if (copy_to_user((void __user *)packet.pdata,
				(void *)(buf_m + (sizeof(packet.buf)
				/ sizeof(uint32_t))), packet.buf[4])) {
				LOG(LOG_ERR, "copy is err !\n");
				ret = -EINVAL;
				goto err_flag;
			}
		}
		if (copy_to_user((void __user *)arg, (void *)buf_m, sizeof(isp_packet_s))) {
			LOG(LOG_ERR, "copy is err !\n");
			ret = -EINVAL;
		}
err_flag:
		if (packet.buf[0] >= (BUF_LENGTH * 4)) {
			kzfree(buf_m);
		}
buf_free:
		vfree(buf);
	}
	break;
	case ISPIOC_IRQ_WAIT: {
		isp_irq_wait_s isp_wait_info;
		if (copy_from_user((void *)&isp_wait_info, (void __user *)arg,
			sizeof(isp_irq_wait_s))) {
			LOG(LOG_ERR, "copy is err !\n");
			ret = -EINVAL;
			break;
		}
		ret = isp_irq_wait_for_completion(isp_wait_info.ctx_id, isp_wait_info.irq_type, isp_wait_info.time_out);
	}
	break;
	case ISPIOC_STA_CTRL:
	{
		isp_sta_ctrl_t ctx_ctrl;
		acamera_context_t *p_ctx;

		if (copy_from_user(&ctx_ctrl, (void __user *)arg, sizeof(ctx_ctrl))) {
			ret = -EFAULT;
			break;
		}

		// isp context save on
		p_ctx = (acamera_context_t *)acamera_get_ctx_ptr(ctx_ctrl.isp_ctx.ctx_id);
		if (p_ctx->initialized == 0) {
			pr_err("%d ctx is not inited.\n", ctx_ctrl.isp_ctx.ctx_id);
			ret = -EFAULT;
			break;
		}
		if (ctx_ctrl.isp_ctx.type == ISP_CTX)
			p_ctx->isp_ctxsv_on = (uint8_t)ctx_ctrl.flag;
		else if (ctx_ctrl.isp_ctx.type == ISP_AE)
			p_ctx->isp_ae_stats_on = (uint8_t)ctx_ctrl.flag;
		else if (ctx_ctrl.isp_ctx.type == ISP_AWB)
			p_ctx->isp_awb_stats_on = (uint8_t)ctx_ctrl.flag;
		else if (ctx_ctrl.isp_ctx.type == ISP_AF)
			p_ctx->isp_af_stats_on = (uint8_t)ctx_ctrl.flag;
		else if (ctx_ctrl.isp_ctx.type == ISP_AE_5BIN)
			p_ctx->isp_ae_5bin_stats_on = (uint8_t)ctx_ctrl.flag;
		else if (ctx_ctrl.isp_ctx.type == ISP_LUMVAR)
			p_ctx->isp_lumvar_stats_on = (uint8_t)ctx_ctrl.flag;
	}
	break;
	default:
		LOG(LOG_ERR, "command %d not support.\n", cmd);
		break;
	}

out:
	if (cmd != ISPIOC_IRQ_WAIT) {
		rt_mutex_unlock(&isp_dev_ctx.fops_lock);
	}

	return ret;
}

int isp_fops_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct isp_dev_context *p_ctx = (struct isp_dev_context *)file->private_data;

	if ((vma->vm_end - vma->vm_start) > p_ctx->mem_size) {
		LOG(LOG_ERR, "mmap size exceed valid range %d.", p_ctx->mem_size);
		return -ENOMEM;
	}
	vma->vm_flags |= VM_IO;
	vma->vm_flags |= VM_LOCKED;
	vma->vm_pgoff = p_ctx->phy_addr >> PAGE_SHIFT;
	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		LOG(LOG_ERR, "mmap phy addr %d failed.", p_ctx->phy_addr);
		return -EAGAIN;
	}

	return 0;
}

static struct file_operations isp_fops = {
	.owner = THIS_MODULE,
	.open = isp_fops_open,
	.release = isp_fops_release,
	.read = isp_fops_read,
	.write = isp_fops_write,
	.llseek = noop_llseek,
	.unlocked_ioctl = isp_fops_ioctl,
	.compat_ioctl = isp_fops_ioctl,
	.mmap = isp_fops_mmap,
};

void *isp_dev_get_vir_addr(void)
{
	return isp_dev_ctx.vir_addr;
}
EXPORT_SYMBOL(isp_dev_get_vir_addr);

int isp_dev_mem_alloc(void)
{
	int ret;
	struct isp_dev_context *p_ctx = NULL;

	p_ctx = &isp_dev_ctx;

	p_ctx->client = ion_client_create(hb_ion_dev, "ac_isp");
	if (IS_ERR(p_ctx->client)) {
		LOG(LOG_ERR, "ac_isp ion client create failed.");
		ret = -ENOMEM;
		goto out1;
	}

	p_ctx->handle = ion_alloc(p_ctx->client,
			TOTAL_MEM_SIZE, 0, ION_HEAP_CARVEOUT_MASK, ION_FLAG_CACHED);
	if (IS_ERR(p_ctx->handle)) {
		LOG(LOG_ERR, "ac_isp ion handle create failed.");
		ret = -ENOMEM;
		goto out1;
	}

	ret = ion_phys(p_ctx->client, p_ctx->handle->id,
			&p_ctx->phy_addr, &p_ctx->mem_size);
	if (ret) {
		LOG(LOG_ERR, "ion_phys get phy address failed.");
		goto out2;
	}
	p_ctx->vir_addr = ion_map_kernel(p_ctx->client, p_ctx->handle);
	if (IS_ERR(p_ctx->vir_addr)) {
		LOG(LOG_ERR, "ion_map failed.");
		ret = -ENOMEM;
		goto out2;
	}

	return 0;

out2:
	ion_free(p_ctx->client, p_ctx->handle);
	p_ctx->phy_addr = 0;
	p_ctx->mem_size = 0;
	ion_client_destroy(p_ctx->client);
out1:
	return ret;
}

static int isp_dev_context_init( struct isp_dev_context *p_ctx )
{
    int rc;

    p_ctx->isp_dev.minor = MISC_DYNAMIC_MINOR;
    p_ctx->isp_dev.fops = &isp_fops;

    rc = misc_register( &p_ctx->isp_dev );
    if ( rc ) {
        LOG( LOG_ERR, "Error: register ISP device failed, ret: %d.", rc );
        return rc;
    }

    p_ctx->dev_minor_id = p_ctx->isp_dev.minor;

    rc = kfifo_alloc( &p_ctx->isp_kfifo_in, SYSTEM_CHARDEV_FIFO_SIZE, GFP_KERNEL );
    if ( rc ) {
        LOG( LOG_ERR, "Error: kfifo_in alloc failed, ret: %d.", rc );
        goto failed_kfifo_in_alloc;
    }

    rc = kfifo_alloc( &p_ctx->isp_kfifo_out, SYSTEM_CHARDEV_FIFO_SIZE, GFP_KERNEL );
    if ( rc ) {
        LOG( LOG_ERR, "Error: kfifo_out alloc failed, ret: %d.", rc );
        goto failed_kfifo_out_alloc;
    }

    rt_mutex_init( &p_ctx->fops_lock );
    init_waitqueue_head( &p_ctx->kfifo_in_queue );
    init_waitqueue_head( &p_ctx->kfifo_out_queue );

    p_ctx->dev_inited = 1;

    LOG( LOG_INFO, "isp_dev_context(%s) init OK.", p_ctx->dev_name );

    return 0;

failed_kfifo_out_alloc:
    kfifo_free( &p_ctx->isp_kfifo_in );
failed_kfifo_in_alloc:
    misc_deregister( &p_ctx->isp_dev );

    return rc;
}

int system_chardev_init( void )
{
    int rc;

    struct isp_dev_context *p_ctx0 = NULL;

    LOG( LOG_INFO, "system init" );

    p_ctx0 = &isp_dev_ctx;
    memset( p_ctx0, 0, sizeof( *p_ctx0 ) );
    p_ctx0->isp_dev.name = SYSTEM_CHARDEV_NAME;
    p_ctx0->dev_name = SYSTEM_CHARDEV_NAME;
    rc = isp_dev_context_init( p_ctx0 );
    if ( rc ) {
        LOG( LOG_ERR, "Error: isp_dev_context_init failed for dev: %s.", p_ctx0->isp_dev.name );
        return rc;
    }

    return 0;
}

extern void *acamera_get_api_ctx_ptr( void );
extern int isp_open_check(void);
int system_chardev_read( char *data, int size )
{
    int rc;

    if ( !isp_dev_ctx.dev_inited ) {
        LOG( LOG_ERR, "dev is not inited, failed to read." );
        return -1;
    }

    rt_mutex_lock( &isp_dev_ctx.fops_lock );

    if ( kfifo_is_empty( &isp_dev_ctx.isp_kfifo_in ) ) {
        long time_out_in_jiffies = 2;           /* jiffies is depend on HW, in x86 Ubuntu, it's 4 ms, 2 is 8ms. */
        rt_mutex_unlock( &isp_dev_ctx.fops_lock ); /* unlock before we return or go sleeping */

        /* wait for the event */
        LOG( LOG_DEBUG, "input FIFO is empty, wait for data, timeout_in_jiffies: %ld, HZ: %d.", time_out_in_jiffies, HZ );
        // rc = wait_event_interruptible_timeout( isp_dev_ctx.kfifo_in_queue, !kfifo_is_empty( &isp_dev_ctx.isp_kfifo_in ), time_out_in_jiffies );
		rc = wait_event_interruptible( isp_dev_ctx.kfifo_in_queue, !kfifo_is_empty( &isp_dev_ctx.isp_kfifo_in ) );
        LOG( LOG_DEBUG, "data is coming or timeout, kfifo_in size: %u, rc: %d.", kfifo_len( &isp_dev_ctx.isp_kfifo_in ), rc );

        /* after wake up, we need to re-gain the mutex */
        rt_mutex_lock( &isp_dev_ctx.fops_lock );
    }

	acamera_context_ptr_t context_ptr = (acamera_context_ptr_t)acamera_get_api_ctx_ptr();
	if (context_ptr->initialized == 0 || isp_open_check() == 0) {
		rc = -1;
	} else {
		rc = kfifo_out( &isp_dev_ctx.isp_kfifo_in, data, size );
	}

    rt_mutex_unlock( &isp_dev_ctx.fops_lock );
    return rc;
}

int system_chardev_write( const char *data, int size )
{
    int rc;

    if ( !isp_dev_ctx.dev_inited ) {
        LOG( LOG_ERR, "dev is not inited, failed to write." );
        return -1;
    }

    rt_mutex_lock( &isp_dev_ctx.fops_lock );

    rc = kfifo_in( &isp_dev_ctx.isp_kfifo_out, data, size );

    /* awake any reader */
    wake_up_interruptible( &isp_dev_ctx.kfifo_out_queue );
    LOG( LOG_DEBUG, "wake up reader who wait on kfifo out." );

    rt_mutex_unlock( &isp_dev_ctx.fops_lock );
    return rc;
}

int system_chardev_destroy( void )
{
    if ( isp_dev_ctx.dev_inited ) {
        kfifo_free( &isp_dev_ctx.isp_kfifo_in );

        kfifo_free( &isp_dev_ctx.isp_kfifo_out );

        misc_deregister( &isp_dev_ctx.isp_dev );

        LOG( LOG_INFO, "misc_deregister dev: %s.", isp_dev_ctx.dev_name );
    } else {
        LOG( LOG_INFO, "dev not inited, do nothing." );
    }

    return 0;
}
