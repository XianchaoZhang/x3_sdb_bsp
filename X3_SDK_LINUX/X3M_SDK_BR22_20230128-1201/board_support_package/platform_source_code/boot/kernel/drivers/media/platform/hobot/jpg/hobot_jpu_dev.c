/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/version.h>
#include <linux/poll.h>
#include <linux/eventpoll.h>
#include <linux/debugfs.h>
#include <linux/sched/signal.h>
#ifdef CONFIG_HOBOT_XJ3
#ifdef CONFIG_ARM_HOBOT_DMC_DEVFREQ
#include <linux/devfreq.h>
#endif
#endif
#include "hobot_jpu_ctl.h"
#include "hobot_jpu_debug.h"
#include "hobot_jpu_pm.h"
#include "hobot_jpu_reg.h"
#include "hobot_jpu_utils.h"

int jpu_debug_flag = 5;
int jpu_debug_info = 0;

#ifdef JPU_SUPPORT_RESERVED_VIDEO_MEMORY
#define JPU_INIT_VIDEO_MEMORY_SIZE_IN_BYTE (16*1024*1024)
#define JPU_DRAM_PHYSICAL_BASE (0x63E00000)	//(0x8AA00000)
static jpu_mm_t s_jmem;
static hb_jpu_drv_buffer_t s_video_memory = { 0 };
#endif

/* this definition is only for chipsnmedia FPGA board env */
/* so for SOC env of customers can be ignored */

/*for kernel up to 3.7.0 version*/
#ifndef VM_RESERVED
#define VM_RESERVED   (VM_DONTEXPAND | VM_DONTDUMP)
#endif

#ifdef CONFIG_PM
/* implement to power management functions */
#endif

DECLARE_BITMAP(jpu_inst_bitmap, MAX_NUM_JPU_INSTANCE);

module_param(jpu_debug_info, int, 0644);

static int jpu_alloc_dma_buffer(hb_jpu_dev_t *dev,
			hb_jpu_drv_buffer_t * jb)
{
	if (!jb || !dev)
		return -1;
#ifdef JPU_SUPPORT_RESERVED_VIDEO_MEMORY
	jb->phys_addr = (unsigned long long)jmem_alloc(&s_jmem, jb->size, 0);
	if ((unsigned long)jb->phys_addr == (unsigned long)-1) {
		jpu_debug(5, "Physical memory allocation error size=%d\n",
			  jb->size);
		return -1;
	}

	jb->base = (unsigned long)(s_video_memory.base + (jb->phys_addr
							  -
							  s_video_memory.phys_addr));
#else
	jb->base = (unsigned long)dma_alloc_coherent(dev->device, PAGE_ALIGN(jb->size),
						     (dma_addr_t
						      *) (&jb->phys_addr),
						     GFP_DMA | GFP_KERNEL);
	if ((void *)(jb->base) == NULL) {
		jpu_debug(5, "Physical memory allocation error size=%d\n",
			  jb->size);
		return -1;
	}
#endif /* JPU_SUPPORT_RESERVED_VIDEO_MEMORY */
	return 0;
}

static void jpu_free_dma_buffer(hb_jpu_dev_t *dev,
				hb_jpu_drv_buffer_t * jb)
{
	if (!jb || !dev) {
		return;
	}

#ifdef JPU_SUPPORT_RESERVED_VIDEO_MEMORY
	if (jb->base)
		jmem_free(&s_jmem, jb->phys_addr, 0);
#else
	if (jb->base)
		dma_free_coherent(dev->device, PAGE_ALIGN(jb->size), (void *)jb->base,
				  jb->phys_addr);
#endif /* JPUR_SUPPORT_RESERVED_VIDEO_MEMORY */
}

static void jpu_clear_status(hb_jpu_dev_t *dev, uint32_t idx)
{
	uint32_t val;

	val = JPU_READL(MJPEG_PIC_STATUS_REG(idx));
	JPU_WRITEL(MJPEG_PIC_STATUS_REG(idx), val);
	JPU_WRITEL(MJPEG_PIC_START_REG(idx), 0);
	val = JPU_READL(MJPEG_INST_CTRL_STATUS_REG);
	val &= ~(1UL << idx);
	JPU_WRITEL(MJPEG_INST_CTRL_STATUS_REG, val);
}

#ifdef USE_SHARE_SEM_BT_KERNEL_AND_USER
typedef enum hb_jpu_mutex {
	JPUDRV_MUTEX_JPU,
	JPUDRV_MUTEX_JMEM,
	JPUDRV_MUTEX_MAX
} hb_jpu_mutex_t;

static void *get_mutex_base(hb_jpu_dev_t *dev, u32 core, unsigned int type)
{
	int instance_pool_size_per_core;
	void *vip_base;
	void *vdi_mutexes_base;
	void *mutex;

	if (dev->instance_pool.base == 0) {
		return NULL;
	}

	/* s_instance_pool.size  assigned to the size of all core once call
	VDI_IOCTL_GET_INSTANCE_POOL by user. */
	instance_pool_size_per_core = (dev->instance_pool.size/MAX_NUM_JPU_CORE);
	vip_base = (void *)(dev->instance_pool.base + (instance_pool_size_per_core
		* core));
	vdi_mutexes_base = (vip_base + (instance_pool_size_per_core -
		(sizeof(void *) * JDI_NUM_LOCK_HANDLES)));
	if (type == JPUDRV_MUTEX_JPU) {
		mutex = (void *)(vdi_mutexes_base + 0 * (sizeof(void *)));
	} else if (type == JPUDRV_MUTEX_JMEM) {
		mutex = (void *)(vdi_mutexes_base + 1 * (sizeof(void *)));
	} else {
		jpu_err("unknown MUTEX_TYPE type=%d\n", type);
		return NULL;
	}
	return mutex;
}

static void jdi_lock_release(hb_jpu_priv_t *priv, u32 core, unsigned int type)
{
	void *mutex;
	volatile int *sync_lock_ptr = NULL;
	hb_jpu_dev_t *dev;
	int32_t i, sync_val;
	unsigned long flags_mp;
	unsigned long timeout = jiffies + HZ;
	uint32_t val;
	//jpu_err("[JPUDRV]+%s, type=%d\n", __FUNCTION__, type);

	dev = priv->jpu_dev;
	sync_val = priv->tgid;
	mutex = get_mutex_base(dev, core, type);
	if (mutex == NULL) {
		jpu_err("Fail to get mutex base, core=%d, type=%d\n", 0, type);
		return;
	}

	sync_lock_ptr = (volatile int *)mutex;
	//if (*sync_lock_ptr == sync_val ||
	//	*sync_lock_ptr == (current->tgid + MAX_VPU_LOCK_INSTANCE)) {
	jpu_debug(5, "Free core=%d, type=%d, sync_lock=%d, current_pid=%d, "
		"tgid=%d, sync_val=%d, \n", 0, type,
		(volatile int)*sync_lock_ptr, current->pid, current->tgid,
		sync_val);
	if (*sync_lock_ptr == sync_val) {
		spin_lock_irqsave(&dev->irq_spinlock, flags_mp);
		for (i = 0; i < MAX_HW_NUM_JPU_INSTANCE; i++) {
			if (dev->interrupt_flag[i] == 1) {
				if (dev->irq_trigger == 1) {
					jpu_clear_status(dev, i);
					enable_irq(dev->irq);
					dev->irq_trigger = 0;
					dev->poll_int_event--;
					jpu_debug(5, "enable irq\n");
				}
				dev->interrupt_flag[i] = 0;
				break;
			} else {
				if (JPU_READL(MJPEG_PIC_STATUS_REG(i)) != 0) {
					dev->ignore_irq = 1;
				}
			}
		}
		if (i == MAX_HW_NUM_JPU_INSTANCE) {
			val = JPU_READL(MJPEG_INST_CTRL_STATUS_REG);
			if ((val >> 4) == INST_CTRL_ENC) {
				dev->ignore_irq = 1;
				jpu_debug(5, "ctrl_status ignore\n");
			}
		}
		spin_unlock_irqrestore(&dev->irq_spinlock, flags_mp);
		while (JPU_READL(MJPEG_INST_CTRL_STATUS_REG) >> 4 == INST_CTRL_ENC) {
			if (time_after(jiffies, timeout)) {
				jpu_debug(5, "Wait Interrupt BUSY timeout pid %d\n", current->pid);
				break;
			}
		}
		spin_lock_irqsave(&dev->irq_spinlock, flags_mp);
		for (i = 0; i < MAX_HW_NUM_JPU_INSTANCE; i++) {
			jpu_err("%d, %d\n", dev->interrupt_flag[i], dev->irq_trigger);
			if (JPU_READL(MJPEG_PIC_STATUS_REG(i)) != 0) {
				jpu_debug(5, "Clear status(pid %d).\n", current->pid);
				jpu_clear_status(dev, 0);
				dev->ignore_irq = 0;
				break;
			}
		}
		spin_unlock_irqrestore(&dev->irq_spinlock, flags_mp);
		__sync_lock_release(sync_lock_ptr);
	}
	//jpu_err("[JPUDRV]-%s\n", __FUNCTION__);
}
#endif

static int jpu_free_instances(struct file *filp)
{
	hb_jpu_drv_instance_list_t *vil, *n;
	hb_jpu_drv_instance_pool_t *vip;
	void *vip_base;
	int instance_pool_size_per_core;
#ifdef USE_SHARE_SEM_BT_KERNEL_AND_USER
	int32_t i = 0;
#else
	void *jdi_mutexes_base;
	const int PTHREAD_MUTEX_T_DESTROY_VALUE = 0xdead10cc;
#endif
	hb_jpu_dev_t *dev;
	hb_jpu_priv_t *priv;
	int core_idx = 0;

	jpu_debug_enter();
	if (!filp) {
		jpu_err("failed to free jpu buffers, filp is null.");
		return -1;
	}
	priv = filp->private_data;
	dev = priv->jpu_dev;
	if (!dev) {
		jpu_err("failed to free jpu buffers, dev is null.");
		return -1;
	}
	/* s_instance_pool.size  assigned to the size of all core once call 
	   JDI_IOCTL_GET_INSTANCE_POOL by user. */
	instance_pool_size_per_core =
	    (dev->instance_pool.size / MAX_NUM_JPU_CORE);

	list_for_each_entry_safe(vil, n, &dev->inst_list_head, list) {
		if (vil->filp == filp) {
			// core_idx need set when multi-core (todo zhaojun.li)
			vip_base = (void *)(dev->instance_pool.base +
					    instance_pool_size_per_core * core_idx);
			jpu_debug(5,
				  "jpu_free_instances detect instance crash "
				  "instIdx=%d, vip_base=%p, instance_pool_size_per_core=%d\n",
				  (int)vil->inst_idx, vip_base,
				  (int)instance_pool_size_per_core);
			vip = (hb_jpu_drv_instance_pool_t *) vip_base;
			if (vip) {
#ifdef USE_SHARE_SEM_BT_KERNEL_AND_USER
				for (i = 0; i < JPUDRV_MUTEX_MAX; i++) {
					jdi_lock_release(priv, (u32)0, i);
				}
#endif
				/* only first 4 byte is key point(inUse of CodecInst in jpuapi)
				   to free the corresponding instance. */
				memset(&vip->codecInstPool[vil->inst_idx], 0x00,
				       4);
#ifdef USE_SHARE_SEM_BT_KERNEL_AND_USER
#else
#define PTHREAD_MUTEX_T_HANDLE_SIZE 4
				jdi_mutexes_base =
				    (vip_base +
				     (instance_pool_size_per_core -
				      PTHREAD_MUTEX_T_HANDLE_SIZE * 4));
				jpu_debug(5,
					  "jpu_free_instances : force to destroy "
					  "jdi_mutexes_base=%p in userspace \n",
					  jdi_mutexes_base);
				if (jdi_mutexes_base) {
					int i;
					for (i = 0; i < 4; i++) {
						memcpy(jdi_mutexes_base,
						       &PTHREAD_MUTEX_T_DESTROY_VALUE,
						       PTHREAD_MUTEX_T_HANDLE_SIZE);
						jdi_mutexes_base +=
						    PTHREAD_MUTEX_T_HANDLE_SIZE;
					}
				}
#endif
			}
			dev->jpu_open_ref_count--;
			list_del(&vil->list);
			clear_bit(vil->inst_idx, jpu_inst_bitmap);
			dev->poll_event[vil->inst_idx] = LLONG_MIN;
			wake_up_interruptible(&dev->poll_wait_q[vil->inst_idx]);
			wake_up_interruptible(&dev->poll_int_wait);
			memset(&dev->jpu_ctx[vil->inst_idx], 0x00,
				sizeof(dev->jpu_ctx[vil->inst_idx]));
			memset(&dev->jpu_status[vil->inst_idx], 0x00,
				sizeof(dev->jpu_status[vil->inst_idx]));
			kfree(vil);
		}
	}

	jpu_debug_leave();

	return 0;
}

static int jpu_free_buffers(struct file *filp)
{
	hb_jpu_drv_buffer_pool_t *pool, *n;
	hb_jpu_drv_buffer_t jb;
	hb_jpu_dev_t *dev;
	hb_jpu_priv_t *priv;
	jpu_debug_enter();
	if (!filp) {
		jpu_err("failed to free jpu buffers, filp is null.");
		return -1;
	}
	priv = filp->private_data;
	dev = priv->jpu_dev;

	if (!dev) {
		jpu_err("failed to free jpu buffers, dev is null.");
		return -1;
	}

	list_for_each_entry_safe(pool, n, &dev->jbp_head, list) {
		if (pool->filp == filp) {
			jb = pool->jb;
			if (jb.base) {
				jpu_free_dma_buffer(dev, &jb);
				list_del(&pool->list);
				kfree(pool);
			}
		}
	}
	jpu_debug_leave();

	return 0;
}

static irqreturn_t jpu_irq_handler(int irq, void *dev_id)
{
	hb_jpu_dev_t *dev = (hb_jpu_dev_t *) dev_id;
	int i;
	u32 flag;

	spin_lock(&dev->irq_spinlock);
#ifdef JPU_IRQ_CONTROL
	disable_irq_nosync(dev->irq);
	dev->irq_trigger = 1;
#endif

	for (i = 0; i < MAX_HW_NUM_JPU_INSTANCE; i++) {
		flag = JPU_READL(MJPEG_PIC_STATUS_REG(i));
		if (flag != 0) {
			break;
		}
	}
	if (i >= MAX_HW_NUM_JPU_INSTANCE) {
		jpu_debug(7, "unknow INTERRUPT FLAG\n");
		dev->irq_trigger = 0;
		dev->interrupt_flag[0] = 0;
		dev->ignore_irq = 0;

#ifdef JPU_IRQ_CONTROL
		enable_irq(dev->irq);
#endif
		return IRQ_HANDLED;
	}

	// clear interrupt status register
	// JPU_WRITEL(MJPEG_PIC_STATUS_REG(i), flag);

	// notify the interrupt to userspace
	if (dev->async_queue)
		kill_fasync(&dev->async_queue, SIGIO, POLL_IN);

	if (dev->ignore_irq != 1) {
		dev->interrupt_reason[i] = flag;
		dev->interrupt_flag[i] = 1;
		// jpu_debug(7, "[%d] INTERRUPT FLAG: %08x, %08x\n", i,
		// dev->interrupt_reason[i], MJPEG_PIC_STATUS_REG(i));

		wake_up_interruptible(&dev->interrupt_wait_q[i]);
		spin_lock(&dev->poll_int_wait.lock);
		dev->poll_int_event++;
		// jpu_debug(7, "jpu_irq_handler poll_int_event[%d]=%lld.\n",
		// 	i, dev->poll_int_event);
		dev->total_poll++;
		// jpu_debug(7, "jpu_irq_handler total_poll=%lld.\n", dev->total_poll);
		spin_unlock(&dev->poll_int_wait.lock);
		wake_up_interruptible(&dev->poll_int_wait);
	} else {
		jpu_debug(7, "jpu_irq_handler ignore irq.\n");
		enable_irq(dev->irq);
		dev->interrupt_flag[i] = 0;
		dev->ignore_irq = 0;
		dev->irq_trigger = 0;
	}

	spin_unlock(&dev->irq_spinlock);
	return IRQ_HANDLED;
}

static void jpu_parse_dts(struct device_node *np, hb_jpu_dev_t * jpu_dev)
{
	hb_jpu_platform_data_t *pdata = jpu_dev->plat_data;

	if (!np)
		return;

	of_property_read_u32(np, "ip_ver", &pdata->ip_ver);
	of_property_read_u32(np, "clock_rate", &pdata->clock_rate);
	of_property_read_u32(np, "min_rate", &pdata->min_rate);
}

static int jpu_open(struct inode *inode, struct file *filp)
{
	hb_jpu_dev_t *dev;
	hb_jpu_priv_t *priv;
	u32 open_count = 0;
	jpu_debug_enter();

	priv = kzalloc(sizeof(hb_jpu_priv_t), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	dev = container_of(inode->i_cdev, hb_jpu_dev_t, cdev);
	if (!dev) {
		jpu_err("failed to get jpu dev data");
		return -1;
	}

	spin_lock(&dev->jpu_spinlock);
	open_count = dev->open_count;
	dev->open_count++;
	priv->jpu_dev = dev;
	priv->inst_index = -1;
	priv->is_irq_poll = 0;
	priv->tgid = current->tgid;
	filp->private_data = (void *)priv;
	spin_unlock(&dev->jpu_spinlock);
#ifdef CONFIG_HOBOT_XJ3
#ifdef CONFIG_ARM_HOBOT_DMC_DEVFREQ
	if (open_count == 0) {
		pm_qos_add_request(&dev->jpu_pm_qos_req, PM_QOS_DEVFREQ, 10000);
		if (!hobot_dmcfreq_checkup_max()) {
			jpu_err("set ddr freq max failed\n");
			dev->open_count--;
			kfree(priv);
			return -1;
		}
	}
#endif
#endif
	hb_jpu_clk_enable(dev);

	jpu_debug_leave();

	return 0;
}

static long jpu_ioctl(struct file *filp, u_int cmd, u_long arg)
{
	int ret = 0;
	int inst_index;
	hb_jpu_dev_t *dev;
	hb_jpu_priv_t *priv;
	unsigned long flags_mp;

	priv = filp->private_data;
	dev = priv->jpu_dev;
	if (!dev) {
		jpu_err("failed to get jpu dev data");
		return -1;
	}

	switch (cmd) {
	case JDI_IOCTL_ALLOCATE_PHYSICAL_MEMORY:
		{
			hb_jpu_drv_buffer_pool_t *jbp;

			jpu_debug(5, "[+]JDI_IOCTL_ALLOCATE_PHYSICAL_MEMORY\n");
			if ((ret = down_interruptible(&dev->jpu_sem)) == 0) {
				jbp =
				    kzalloc(sizeof(hb_jpu_drv_buffer_pool_t),
					    GFP_KERNEL);
				if (!jbp) {
					up(&dev->jpu_sem);
					return -ENOMEM;
				}

				ret =
				    copy_from_user(&(jbp->jb),
						   (hb_jpu_drv_buffer_t *) arg,
						   sizeof(hb_jpu_drv_buffer_t));
				if (ret) {
					kfree(jbp);
					up(&dev->jpu_sem);
					return -EFAULT;
				}

				ret = jpu_alloc_dma_buffer(dev, &(jbp->jb));
				if (ret == -1) {
					ret = -ENOMEM;
					kfree(jbp);
					up(&dev->jpu_sem);
					break;
				}
				ret =
				    copy_to_user((void __user *)arg, &(jbp->jb),
						 sizeof(hb_jpu_drv_buffer_t));
				if (ret) {
					kfree(jbp);
					ret = -EFAULT;
					up(&dev->jpu_sem);
					break;
				}

				jbp->filp = filp;
				spin_lock(&dev->jpu_spinlock);
				list_add(&jbp->list, &dev->jbp_head);
				spin_unlock(&dev->jpu_spinlock);

				up(&dev->jpu_sem);
			}
			jpu_debug(5, "[-]JDI_IOCTL_ALLOCATE_PHYSICAL_MEMORY\n");
		}
		break;

	case JDI_IOCTL_FREE_PHYSICAL_MEMORY:
		{
			hb_jpu_drv_buffer_pool_t *jbp, *n;
			hb_jpu_drv_buffer_t jb;

			jpu_debug(5, "[+]VDI_IOCTL_FREE_PHYSICALMEMORY\n");
			if ((ret = down_interruptible(&dev->jpu_sem)) == 0) {
				ret =
				    copy_from_user(&jb,
						   (hb_jpu_drv_buffer_t *) arg,
						   sizeof(hb_jpu_drv_buffer_t));
				if (ret) {
					up(&dev->jpu_sem);
					return -EACCES;
				}

				if (jb.base)
					jpu_free_dma_buffer(dev, &jb);

				spin_lock(&dev->jpu_spinlock);
				list_for_each_entry_safe(jbp, n, &dev->jbp_head,
							 list) {
					if (jbp->jb.base == jb.base) {
						list_del(&jbp->list);
						kfree(jbp);
						break;
					}
				}
				spin_unlock(&dev->jpu_spinlock);

				up(&dev->jpu_sem);
			}
			jpu_debug(5, "[-]VDI_IOCTL_FREE_PHYSICALMEMORY\n");
		}
		break;

	case JDI_IOCTL_GET_RESERVED_VIDEO_MEMORY_INFO:
		{
#ifdef JPU_SUPPORT_RESERVED_VIDEO_MEMORY
			if (s_video_memory.base != 0) {
				ret =
				    copy_to_user((void __user *)arg,
						 &s_video_memory,
						 sizeof(hb_jpu_drv_buffer_t));
				if (ret != 0)
					ret = -EFAULT;
			} else {
				ret = -EFAULT;
			}
#endif /* JPU_SUPPORT_RESERVED_VIDEO_MEMORY */
		}
		break;

	case JDI_IOCTL_WAIT_INTERRUPT:
		{
			hb_jpu_drv_intr_t info;
			u32 instance_no;
			unsigned long flags_mp;

			jpu_debug(7, "[+]JDI_IOCTL_WAIT_INTERRUPT\n");
			ret = copy_from_user(&info, (hb_jpu_drv_intr_t *) arg,
					     sizeof(hb_jpu_drv_intr_t));
			if (ret != 0)
				return -EFAULT;

			instance_no = info.inst_idx;
			// jpu_debug(7, "INSTANCE NO: %d Timeout %d:\n", instance_no, info.timeout);
			ret =
			    wait_event_interruptible_timeout
			    (dev->interrupt_wait_q[instance_no],
			     dev->interrupt_flag[instance_no] != 0,
			     msecs_to_jiffies(info.timeout));
			if (!ret) {
				jpu_debug(7, "INSTANCE NO: %d ETIME\n",
					  instance_no);
				ret = -ETIME;
				break;
			}
#if 1
			if (signal_pending(current)) {
				ret = -ERESTARTSYS;
				jpu_debug(5, "INSTANCE NO: %d ERESTARTSYS, s_interrupt_flag(%d)\n",
					  instance_no, dev->interrupt_flag[instance_no]);
				break;
			}
#endif

			jpu_debug(7,
				  "INST(%d) s_interrupt_flag(%d), reason(0x%08x)\n",
				  instance_no, dev->interrupt_flag[instance_no],
				  dev->interrupt_reason[instance_no]);

			info.intr_reason = dev->interrupt_reason[instance_no];
			dev->interrupt_flag[instance_no] = 0;
			dev->interrupt_reason[instance_no] = 0;
			ret = copy_to_user((void __user *)arg, &info,
					   sizeof(hb_jpu_drv_intr_t));
#ifdef JPU_IRQ_CONTROL
			spin_lock_irqsave(&dev->irq_spinlock, flags_mp);
			if (dev->irq_trigger == 1) {
				enable_irq(dev->irq);
				dev->irq_trigger = 0;
			}
			spin_unlock_irqrestore(&dev->irq_spinlock, flags_mp);
#endif
			spin_lock_irqsave(&dev->poll_int_wait.lock, flags_mp);
			dev->poll_int_event--;
			dev->total_release++;
			jpu_debug(7, "ioctl poll event %lld total_release=%lld.\n",
				dev->poll_int_event, dev->total_release);
			spin_unlock_irqrestore(&dev->poll_int_wait.lock, flags_mp);

			jpu_debug(7, "[-]VDI_IOCTL_WAIT_INTERRUPT\n");
			if (ret != 0)
				return -EFAULT;
		}
		break;

	case JDI_IOCTL_SET_CLOCK_GATE:
		{
			u32 clkgate;
			jpu_debug(5, "[+]JDI_IOCTL_SET_CLOCK_GATE\n");

			if (get_user(clkgate, (u32 __user *) arg))
				return -EFAULT;

#ifdef JPU_SUPPORT_CLOCK_CONTROL
			if (clkgate)
				hb_jpu_clk_enable(dev);
			else
				hb_jpu_clk_disable(dev);
#endif
			jpu_debug(5, "[-]VDI_IOCTL_SET_CLOCK_GATE\n");
		}
		break;

	case JDI_IOCTL_GET_INSTANCE_POOL:
		jpu_debug(5, "[+]JDI_IOCTL_GET_INSTANCE_POOL\n");

		if ((ret = down_interruptible(&dev->jpu_sem)) == 0) {
			if (dev->instance_pool.base != 0) {
				ret =
				    copy_to_user((void __user *)arg,
						 &dev->instance_pool,
						 sizeof(hb_jpu_drv_buffer_t));
					if (ret != 0)
						ret = -EFAULT;
			} else {
				ret = copy_from_user(&dev->instance_pool,
						     (hb_jpu_drv_buffer_t *)
						     arg,
						     sizeof
						     (hb_jpu_drv_buffer_t));
				if (ret == 0) {
					dev->instance_pool.size =
					    PAGE_ALIGN(dev->instance_pool.size);
					dev->instance_pool.base =
					    (unsigned long)
					    vmalloc(dev->instance_pool.size);
					dev->instance_pool.phys_addr =
					    dev->instance_pool.base;

					if (dev->instance_pool.base != 0) {
						/*clearing memory */
						memset((void *)
						       dev->instance_pool.base,
						       0x0,
						       dev->instance_pool.size);
						ret =
						    copy_to_user((void __user *)
								 arg,
								 &dev->instance_pool,
								 sizeof
								 (hb_jpu_drv_buffer_t));
						if (ret == 0) {
							/* success to get memory for instance pool */
							jpu_debug(5,
								  "[-]JDI_IOCTL_GET_INSTANCE_POOL");
							up(&dev->jpu_sem);
							break;
						}
					}
					ret = -EFAULT;
				}
			}
			up(&dev->jpu_sem);
		}

		jpu_debug(5,
			  "[-]JDI_IOCTL_GET_INSTANCE_POOL: %s base: %lx, size: %d\n",
			  (ret == 0 ? "OK" : "NG"), dev->instance_pool.base,
			  dev->instance_pool.size);
		break;

	case JDI_IOCTL_OPEN_INSTANCE:
		{
			hb_jpu_drv_inst_t inst_info;
			hb_jpu_drv_instance_list_t *jil, *jil_tmp, *n;

			jpu_debug(5, "[+]JDI_IOCTL_OPEN_INSTANCE\n");

			jil = kzalloc(sizeof(*jil), GFP_KERNEL);
			if (!jil)
				return -ENOMEM;
			if (copy_from_user
			    (&inst_info, (hb_jpu_drv_inst_t *) arg,
			     sizeof(hb_jpu_drv_inst_t))) {

				kfree(jil);
				return -EFAULT;
			}
			if (inst_info.inst_idx >= MAX_NUM_JPU_INSTANCE) {
				kfree(jil);
				return -EINVAL;
			}

			jil->inst_idx = inst_info.inst_idx;
			jil->filp = filp;

			spin_lock(&dev->jpu_spinlock);

			list_for_each_entry_safe(jil_tmp, n,
						 &dev->inst_list_head, list) {
				if (jil_tmp->inst_idx == inst_info.inst_idx) {
					kfree(jil);
					jpu_err
					    ("Failed to open instance due to same id(%d)",
					     (int)inst_info.inst_idx);
					spin_unlock(&dev->jpu_spinlock);
					return -EINVAL;
				}
			}

			list_add(&jil->list, &dev->inst_list_head);

			/* counting the current open instance number */
			inst_info.inst_open_count = 0;
			list_for_each_entry_safe(jil, n, &dev->inst_list_head,
						 list) {
				inst_info.inst_open_count++;
			}
			/* flag just for that jpu is in opened or closed */
			dev->jpu_open_ref_count++;
			spin_unlock(&dev->jpu_spinlock);

			if (copy_to_user((void __user *)arg, &inst_info,
					 sizeof(hb_jpu_drv_inst_t))) {
				kfree(jil);
				return -EFAULT;
			}

			jpu_debug(5, "[-]JDI_IOCTL_OPEN_INSTANCE inst_idx=%d, "
				  "s_jpu_open_ref_count=%d, inst_open_count=%d\n",
				  (int)inst_info.inst_idx,
				  dev->jpu_open_ref_count,
				  inst_info.inst_open_count);
		}
		break;

	case JDI_IOCTL_CLOSE_INSTANCE:
		{
			hb_jpu_drv_inst_t inst_info;
			hb_jpu_drv_instance_list_t *jil, *n;
			u32 found = 0;

			jpu_debug(5, "[+]JDI_IOCTL_CLOSE_INSTANCE\n");
			if (copy_from_user
			    (&inst_info, (hb_jpu_drv_inst_t *) arg,
			     sizeof(hb_jpu_drv_inst_t)))
				return -EFAULT;
			if (inst_info.inst_idx >= MAX_NUM_JPU_INSTANCE) {
				return -EINVAL;
			}

			spin_lock(&dev->jpu_spinlock);
			list_for_each_entry_safe(jil, n, &dev->inst_list_head,
						 list) {
				if (jil->inst_idx == inst_info.inst_idx) {
					list_del(&jil->list);
					kfree(jil);
					found = 1;
					break;
				}
			}

			if (0 == found) {
				spin_unlock(&dev->jpu_spinlock);
				return -EINVAL;
			}

			/* counting the current open instance number */
			inst_info.inst_open_count = 0;
			list_for_each_entry_safe(jil, n, &dev->inst_list_head,
						 list) {
				inst_info.inst_open_count++;
			}

			dev->poll_event[inst_info.inst_idx] = LLONG_MIN;
			wake_up_interruptible(&dev->poll_wait_q[inst_info.inst_idx]);
			wake_up_interruptible(&dev->poll_int_wait);


			/* flag just for that jpu is in opened or closed */
			dev->jpu_open_ref_count--;
			spin_unlock(&dev->jpu_spinlock);

			if (copy_to_user((void __user *)arg, &inst_info,
					 sizeof(hb_jpu_drv_inst_t)))
				return -EFAULT;

			jpu_debug(5, "[-]JDI_IOCTL_CLOSE_INSTANCE inst_idx=%d, "
				  "s_jpu_open_ref_count=%d, inst_open_count=%d\n",
				  (int)inst_info.inst_idx,
				  dev->jpu_open_ref_count,
				  inst_info.inst_open_count);
		}
		break;

	case JDI_IOCTL_GET_INSTANCE_NUM:
		{
			hb_jpu_drv_inst_t inst_info;
			hb_jpu_drv_instance_list_t *jil, *n;

			jpu_debug(5, "[+]JDI_IOCTL_GET_INSTANCE_NUM\n");

			ret =
			    copy_from_user(&inst_info,
					   (hb_jpu_drv_inst_t *) arg,
					   sizeof(hb_jpu_drv_inst_t));
			if (ret != 0) {
				ret = -EFAULT;
				break;
			}
			spin_lock(&dev->jpu_spinlock);
			inst_info.inst_open_count = 0;
			list_for_each_entry_safe(jil, n, &dev->inst_list_head,
						 list) {
				inst_info.inst_open_count++;
			}
			spin_unlock(&dev->jpu_spinlock);

			ret = copy_to_user((void __user *)arg, &inst_info,
					   sizeof(hb_jpu_drv_inst_t));

			jpu_debug(5,
				  "[-]JDI_IOCTL_GET_INSTANCE_NUM inst_idx=%d, "
				  "open_count=%d\n", (int)inst_info.inst_idx,
				  inst_info.inst_open_count);
		}
		break;

	case JDI_IOCTL_RESET:
		jpu_debug(5, "[+]JDI_IOCTL_RESET\n");
		hb_jpu_hw_reset();
		jpu_debug(5, "[-]JDI_IOCTL_RESET\n");
		break;

	case JDI_IOCTL_GET_REGISTER_INFO:
		{
			hb_jpu_drv_buffer_t reg_buf;
			memset(&reg_buf, 0, sizeof(hb_jpu_drv_buffer_t));
			jpu_debug(5, "[+]JDI_IOCTL_GET_REGISTER_INFO\n");
			reg_buf.phys_addr = dev->jpu_mem->start;
			reg_buf.virt_addr = (unsigned long)dev->regs_base;
			reg_buf.size = resource_size(dev->jpu_mem);
			ret = copy_to_user((void __user *)arg, &reg_buf,
					   sizeof(hb_jpu_drv_buffer_t));
			if (ret != 0)
				ret = -EFAULT;
			jpu_debug(5, "[-]JDI_IOCTL_GET_REGISTER_INFO "
				  "jpu_register.phys_addr==0x%lx, s_jpu_register.virt_addr=0x%lx,"
				  "s_jpu_register.size=%d\n", reg_buf.phys_addr,
				  reg_buf.virt_addr, reg_buf.size);
		}
		break;
	case JDI_IOCTL_ALLOCATE_INSTANCE_ID:
		{
			jpu_debug(5, "[+]JDI_IOCTL_ALLOCATE_INSTANCE_ID\n");
			spin_lock(&dev->jpu_spinlock);
			inst_index =
			    find_first_zero_bit(jpu_inst_bitmap,
						MAX_NUM_JPU_INSTANCE);
			if (inst_index < MAX_NUM_JPU_INSTANCE) {
				set_bit(inst_index, jpu_inst_bitmap);
				dev->poll_event[inst_index] = 0;
			} else {
				inst_index = -1;
			}
			priv->inst_index = inst_index; // it's useless
			spin_unlock(&dev->jpu_spinlock);

			ret =
			    copy_to_user((void __user *)arg, &inst_index,
					 sizeof(int));
			if (ret != 0)
				ret = -EFAULT;
			jpu_debug(5,
				  "[-]JDI_IOCTL_ALLOCATE_INSTANCE_ID id = %d\n",
				  inst_index);
		}
		break;
	case JDI_IOCTL_FREE_INSTANCE_ID:
		{
			int inst_index;
			jpu_debug(5, "[+]JDI_IOCTL_FREE_INSTANCE_ID\n");
			ret =
			    copy_from_user(&inst_index, (int *)arg,
					   sizeof(int));
			if (ret != 0
			    || (inst_index < 0
				|| inst_index >= MAX_NUM_JPU_INSTANCE)) {
				jpu_err
				    ("JDI_IOCTL_FREE_INSTANCE_ID invalid instance id.");
				return -EFAULT;
			}
			spin_lock(&dev->jpu_spinlock);
			clear_bit(inst_index, jpu_inst_bitmap);
			priv->inst_index = -1;
			memset(&dev->jpu_ctx[inst_index], 0x00,
				sizeof(dev->jpu_ctx[inst_index]));
			memset(&dev->jpu_status[inst_index], 0x00,
				sizeof(dev->jpu_status[inst_index]));
			spin_unlock(&dev->jpu_spinlock);

			jpu_debug(5,
				  "[-]JDI_IOCTL_FREE_INSTANCE_ID clear id = %d\n",
				  inst_index);
		}
		break;
	case JDI_IOCTL_POLL_WAIT_INSTANCE: {
		hb_jpu_drv_intr_t info;
		u32 inst_no;

		//jpu_debug(5, "[+]JDI_IOCTL_POLL_WAIT_INSTANCE\n");
		if ((ret = down_interruptible(&dev->jpu_sem)) == 0) {
		ret = copy_from_user(&info, (hb_jpu_drv_intr_t*)arg,
			sizeof(hb_jpu_drv_intr_t));
		if (ret != 0) {
			jpu_err
				("JDI_IOCTL_POLL_WAIT_INSTANCE copy from user fail.\n");
			up(&dev->jpu_sem);
			return -EFAULT;
		}
		inst_no = info.inst_idx;
		if (inst_no < MAX_NUM_JPU_INSTANCE) {
			if (info.intr_reason == 0) {
				spin_lock(&dev->poll_wait_q[inst_no].lock);
				priv->inst_index = inst_no;
				priv->is_irq_poll = 0;
				spin_unlock(&dev->poll_wait_q[inst_no].lock);
			} else if (info.intr_reason == JPU_INST_INTERRUPT) {
				spin_lock_irqsave(&dev->poll_int_wait.lock, flags_mp);
				priv->inst_index = inst_no;
				priv->is_irq_poll = 1;
				spin_unlock_irqrestore(&dev->poll_int_wait.lock, flags_mp);
			} else if (info.intr_reason == JPU_PIC_DONE ||
				info.intr_reason == JPU_INST_CLOSED) {
				spin_lock(&dev->poll_wait_q[inst_no].lock);
				dev->poll_event[inst_no]++;
				jpu_debug(7, "ioctl poll wait event dev->poll_event[%u]=%lld.\n",
					inst_no, dev->poll_event[inst_no]);
				spin_unlock(&dev->poll_wait_q[inst_no].lock);
				wake_up_interruptible(&dev->poll_wait_q[inst_no]);
			} else {
				jpu_err
					("JDI_IOCTL_POLL_WAIT_INSTANCE invalid instance reason"
					"(%d) or index(%d).", info.intr_reason, inst_no);
				ret = -EINVAL;
			}
		} else {
			ret = -EINVAL;
		}
		up(&dev->jpu_sem);
		}
		//jpu_debug(5, "[-]JDI_IOCTL_POLL_WAIT_INSTANCE\n");
	}
	break;
	case JDI_IOCTL_SET_CTX_INFO: {
			hb_jpu_ctx_info_t info;
			jpu_debug(5, "[+]JDI_IOCTL_SET_CTX_INFO\n");
			if ((ret = down_interruptible(&dev->jpu_sem)) == 0) {
			ret = copy_from_user(&info, (hb_jpu_ctx_info_t *) arg,
						 sizeof(hb_jpu_ctx_info_t));
			if (ret != 0) {
				jpu_err
					("JDI_IOCTL_SET_CTX_INFO copy from user fail.\n");
				up(&dev->jpu_sem);
				return -EFAULT;
			}
			inst_index = info.context.instance_index;
			if (inst_index < 0 || inst_index >= MAX_NUM_JPU_INSTANCE) {
				jpu_err
					("Invalid instance index %d.\n", inst_index);
				up(&dev->jpu_sem);
				return -EINVAL;
			}
			spin_lock(&dev->jpu_info_spinlock);
			dev->jpu_ctx[inst_index] = info;
			spin_unlock(&dev->jpu_info_spinlock);
			up(&dev->jpu_sem);
			}
			jpu_debug(5, "[-]JDI_IOCTL_SET_CTX_INFO\n");
			break;
		}
	case JDI_IOCTL_SET_STATUS_INFO: {
			//jpu_debug(5, "[+]JDI_IOCTL_SET_STATUS_INFO\n");
			hb_jpu_status_info_t info;
			if ((ret = down_interruptible(&dev->jpu_sem)) == 0) {
			ret = copy_from_user(&info, (hb_jpu_status_info_t *) arg,
						 sizeof(hb_jpu_status_info_t));
			if (ret != 0) {
				jpu_err
					("JDI_IOCTL_SET_STATUS_INFO copy from user fail.\n");
				up(&dev->jpu_sem);
				return -EFAULT;
			}
			inst_index = info.inst_idx;
			if (inst_index < 0 || inst_index >= MAX_NUM_JPU_INSTANCE) {
				jpu_err
					("Invalid instance index %d.\n", inst_index);
				up(&dev->jpu_sem);
				return -EINVAL;
			}
			spin_lock(&dev->jpu_info_spinlock);
			dev->jpu_status[inst_index] = info;
			spin_unlock(&dev->jpu_info_spinlock);
			up(&dev->jpu_sem);
			}
			//jpu_debug(5, "[-]JDI_IOCTL_SET_STATUS_INFO\n");
			break;
		}
	default:
		{
			jpu_err("No such IOCTL, cmd is %d\n", cmd);
			ret = -EINVAL;
		}
		break;
	}
	return ret;
}

static ssize_t jpu_read(struct file *filp, char __user * buf, size_t len,
			loff_t * ppos)
{
	jpu_debug_enter();
	jpu_debug_leave();
	return -1;
}

static ssize_t jpu_write(struct file *filp, const char __user * buf,
			 size_t len, loff_t * ppos)
{
	jpu_debug_enter();
	/* DPRINTK("jpu_write len=%d\n", (int)len); */
	if (!buf) {
		jpu_debug(5, "jpu_write buf = NULL error \n");
		return -EFAULT;
	}
	jpu_debug_leave();

	return -1;
}

static int jpu_release(struct inode *inode, struct file *filp)
{
	int ret = 0;
	u32 open_count;
	int i;

	hb_jpu_dev_t *dev;
	hb_jpu_priv_t *priv;
	jpu_debug_enter();
	dev = container_of(inode->i_cdev, hb_jpu_dev_t, cdev);
	if (!dev) {
		jpu_err("failed to get jpu dev data");
		return -1;
	}
	priv = filp->private_data;

	if ((ret = down_interruptible(&dev->jpu_sem)) == 0) {
		/* found and free the not handled buffer by user applications */
		spin_lock(&dev->jpu_spinlock);	//check this place
		jpu_free_buffers(filp);

		/* found and free the not closed instance by user applications */
		jpu_free_instances(filp);
		jpu_debug(5, "open_count: %d\n", dev->open_count);
		dev->open_count--;
		open_count = dev->open_count;
		spin_unlock(&dev->jpu_spinlock);
		if (open_count == 0) {
			if (dev->instance_pool.base) {
				jpu_debug(5, "free instance pool\n");
#ifdef USE_SHARE_SEM_BT_KERNEL_AND_USER
				for (i = 0; i < JPUDRV_MUTEX_MAX; i++) {
					jdi_lock_release(priv, (u32)0, i);
				}
#endif
				vfree((const void *)dev->instance_pool.base);
				dev->instance_pool.base = 0;
			}
			for (i = 0; i < MAX_NUM_JPU_INSTANCE; i++)
				clear_bit(i, jpu_inst_bitmap);
#ifdef CONFIG_ARM_HOBOT_DMC_DEVFREQ
			pm_qos_remove_request(&dev->jpu_pm_qos_req);
#endif
			dev->total_poll = 0;
			dev->total_release = 0;
		}
	}
	kfree(priv);
	up(&dev->jpu_sem);
	hb_jpu_clk_disable(dev);
	jpu_debug_leave();

	return 0;
}

static int jpu_fasync(int fd, struct file *filp, int mode)
{
	int ret = 0;
	hb_jpu_dev_t *dev;
	hb_jpu_priv_t *priv;
	jpu_debug_enter();
	priv = filp->private_data;
	dev = priv->jpu_dev;
	if (!dev) {
		jpu_err("failed to get jpu dev data");
		return -1;
	}

	ret = fasync_helper(fd, filp, mode, &dev->async_queue);
	jpu_debug_leave();
	return ret;
}

static int jpu_map_to_register(struct file *filp, struct vm_area_struct *vm)
{
	unsigned long pfn;
	hb_jpu_dev_t *dev;
	hb_jpu_priv_t *priv;
	int ret;
	jpu_debug_enter();

	if (!filp || !vm) {
		jpu_err("failed to map register, filp or vm is null.");
		return -1;
	}

	priv = filp->private_data;
	dev = priv->jpu_dev;

	if (!dev) {
		jpu_err("failed to map register, dev is null.");
		return -1;
	}

	vm->vm_flags |= VM_IO | VM_RESERVED;
	vm->vm_page_prot = pgprot_noncached(vm->vm_page_prot);
	pfn = dev->jpu_mem->start >> PAGE_SHIFT;
	ret = remap_pfn_range(vm, vm->vm_start, pfn, vm->vm_end - vm->vm_start,
			      vm->vm_page_prot) ? -EAGAIN : 0;
	jpu_debug_leave();
	return ret;
}

static int jpu_map_to_physical_memory(struct file *filp,
				      struct vm_area_struct *vm)
{
	hb_jpu_dev_t *dev;
	hb_jpu_priv_t *priv;
	int ret;
	jpu_debug_enter();

	if (!filp || !vm) {
		jpu_err("failed to map register, filp or vm is null.");
		return -1;
	}

	priv = filp->private_data;
	dev = priv->jpu_dev;

	if (!dev) {
		jpu_err("failed to map register, dev is null.");
		return -1;
	}

	vm->vm_flags |= VM_IO | VM_RESERVED;
	vm->vm_page_prot = pgprot_noncached(vm->vm_page_prot);
	ret = remap_pfn_range(vm, vm->vm_start, vm->vm_pgoff,
			      vm->vm_end - vm->vm_start,
			      vm->vm_page_prot) ? -EAGAIN : 0;
	jpu_debug_leave();
	return ret;
}

static int jpu_map_to_instance_pool_memory(struct file *filp,
					   struct vm_area_struct *vm)
{
	int ret;
	long length;
	unsigned long start;
	char *vmalloc_area_ptr;
	unsigned long pfn;
	hb_jpu_dev_t *dev;
	hb_jpu_priv_t *priv;

	jpu_debug_enter();

	if (!filp || !vm) {
		jpu_err("failed to map instances, filp or vm is null.");
		return -1;
	}

	priv = filp->private_data;
	dev = priv->jpu_dev;

	if (!dev) {
		jpu_err("failed to map  instances, dev is null.");
		return -1;
	}

	length = vm->vm_end - vm->vm_start;
	start = vm->vm_start;
	vmalloc_area_ptr = (char *)dev->instance_pool.base;

	vm->vm_flags |= VM_RESERVED;

	/* loop over all pages, map it page individually */
	while (length > 0) {
		pfn = vmalloc_to_pfn(vmalloc_area_ptr);
		if ((ret =
		     remap_pfn_range(vm, start, pfn, PAGE_SIZE,
				     PAGE_SHARED)) < 0) {
			return ret;
		}
		start += PAGE_SIZE;
		vmalloc_area_ptr += PAGE_SIZE;
		length -= PAGE_SIZE;
	}

	jpu_debug_leave();

	return 0;
}

/*!
* @brief memory map interface for jpu file operation
* @return  0 on success or negative error code on error
*/
static int jpu_mmap(struct file *filp, struct vm_area_struct *vm)
{
	hb_jpu_dev_t *dev;
	hb_jpu_priv_t *priv;
	jpu_debug_enter();
	priv = filp->private_data;
	dev = priv->jpu_dev;

	if (vm->vm_pgoff == 0)
		return jpu_map_to_instance_pool_memory(filp, vm);

	if (vm->vm_pgoff == (dev->jpu_mem->start >> PAGE_SHIFT))
		return jpu_map_to_register(filp, vm);

	return jpu_map_to_physical_memory(filp, vm);
}

static unsigned int jpu_poll(struct file *filp, struct poll_table_struct *wait)
{
	hb_jpu_dev_t *dev;
	hb_jpu_priv_t *priv;
	unsigned int mask = 0;
	int64_t count, count2;

	priv = filp->private_data;
	dev = priv->jpu_dev;
	if (priv->inst_index >= MAX_NUM_JPU_INSTANCE) {
		return EPOLLERR;
	}

	if (priv->is_irq_poll == 0) {
		poll_wait(filp, &dev->poll_wait_q[priv->inst_index], wait);
		spin_lock(&dev->poll_wait_q[priv->inst_index].lock);
		if (dev->poll_event[priv->inst_index] > 0) {
			mask = EPOLLIN | EPOLLET;
			dev->poll_event[priv->inst_index]--;
		} else if (dev->poll_event[priv->inst_index] == LLONG_MIN) {
			mask = EPOLLHUP;
		}
		spin_unlock(&dev->poll_wait_q[priv->inst_index].lock);
	} else {
		poll_wait(filp, &dev->poll_int_wait, wait);
		count = dev->poll_int_event;
		// here we check the poll event state for user stopping case
		count2 = dev->poll_event[priv->inst_index];
		if (count > 0) {
			mask = EPOLLIN | EPOLLET;
		} else if (count2 == LLONG_MIN) {
			mask = EPOLLHUP;
		}
	}
	return mask;
}

struct file_operations jpu_fops = {
	.owner = THIS_MODULE,
	.open = jpu_open,
	.read = jpu_read,
	.write = jpu_write,
	.unlocked_ioctl = jpu_ioctl,
	.release = jpu_release,
	.fasync = jpu_fasync,
	.mmap = jpu_mmap,
  .poll = jpu_poll,
};

//////////////// jenc
static char *get_codec(hb_jpu_ctx_info_t *jpu_ctx)
{
	switch (jpu_ctx->context.codec_id) {
		case MEDIA_CODEC_ID_H264:
			return "h264";
		case MEDIA_CODEC_ID_H265:
			return "h265";
		case MEDIA_CODEC_ID_MJPEG:
			return "mjpg";
		case MEDIA_CODEC_ID_JPEG:
			return "jpeg";
		default:
			break;
	}
	return "---";
}

static void rcparam_show(struct seq_file *s, hb_jpu_ctx_info_t *jpu_ctx) {
	mc_rate_control_params_t *rc =
		&(jpu_ctx->context.video_enc_params.rc_params);
	if (rc->mode == MC_AV_RC_MODE_MJPEGFIXQP) {
		seq_printf(s, "%7d %9s %10d %14d\n",
			jpu_ctx->context.instance_index,
			"mjpgfixqp",
			rc->mjpeg_fixqp_params.frame_rate,
			rc->mjpeg_fixqp_params.quality_factor);
	}
	seq_printf(s, "\n");
}

static int jpu_jenc_show(struct seq_file *s, void *unused)
{
	int i;
	int output = 0;
	hb_jpu_dev_t *dev = (hb_jpu_dev_t *)s->private;
	if (dev == NULL)
		return 0;

	for (i = 0; i < MAX_NUM_JPU_INSTANCE; i++) {
		if (dev->jpu_ctx[i].valid && dev->jpu_ctx[i].context.encoder) {
			if (output == 0) {
				output = 1;
				seq_printf(s, "----encode param----\n");
				seq_printf(s, "%7s %7s %5s %6s %7s %10s %15s %11s "
					"%10s %6s %6s\n", "enc_idx", "enc_id", "width", "height",
					"pix_fmt", "fbuf_count", "extern_buf_flag", "bsbuf_count",
					"bsbuf_size", "mirror", "rotate");
			}
			seq_printf(s, "%7d %7s %5d %6d %7d %10d %15d %11d %10d %6d %6d\n",
				dev->jpu_ctx[i].context.instance_index,
				get_codec(&dev->jpu_ctx[i]),
				dev->jpu_ctx[i].context.video_enc_params.width,
				dev->jpu_ctx[i].context.video_enc_params.height,
				dev->jpu_ctx[i].context.video_enc_params.pix_fmt,
				dev->jpu_ctx[i].context.video_enc_params.frame_buf_count,
				dev->jpu_ctx[i].context.video_enc_params.external_frame_buf,
				dev->jpu_ctx[i].context.video_enc_params.bitstream_buf_count,
				dev->jpu_ctx[i].context.video_enc_params.bitstream_buf_size,
				dev->jpu_ctx[i].context.video_enc_params.mir_direction,
				dev->jpu_ctx[i].context.video_enc_params.rot_degree);
		}
	}

	output = 0;
	for (i = 0; i < MAX_NUM_JPU_INSTANCE; i++) {
		if (dev->jpu_ctx[i].valid && dev->jpu_ctx[i].context.encoder) {
			if (output == 0) {
				output = 1;
				seq_printf(s, "\n");
				seq_printf(s, "----encode rc param----\n");
				seq_printf(s, "%7s %9s %10s %14s\n",
					"enc_idx", "rc_mode", "frame_rate", "quality_factor");
			}
			rcparam_show(s, &(dev->jpu_ctx[i]));
		}
	}

	// seq_printf(s, "\n");
	// seq_printf(s, "----encode gop param:\n");
	// seq_printf(s, "enc_idx gop_preset_idx custom_gop_size\n");
	// for (i = 0; i < MAX_NUM_JPU_INSTANCE; i++) {
	// 	if (dev->jpu_ctx[i].valid && dev->jpu_ctx[i].context.encoder) {
	// 		mc_video_gop_params_t *gop =
	// 			&(dev->jpu_ctx[i].context.video_enc_params.gop_params);
	// 		seq_printf(s, "%6d %14d %10d\n",
	// 			dev->jpu_ctx[i].context.instance_index,
	// 			gop->gop_preset_idx,
	// 			gop->custom_gop_size);
	// 	}
	// }
	spin_lock(&dev->jpu_info_spinlock);
	output = 0;
	for (i = 0; i < MAX_NUM_JPU_INSTANCE; i++) {
		if (dev->jpu_ctx[i].valid && dev->jpu_ctx[i].context.encoder) {
			mc_inter_status_t *status =	&(dev->jpu_status[i].status);
			if (output == 0) {
				output = 1;
				seq_printf(s, "\n");
				seq_printf(s, "----encode status----\n");
				seq_printf(s, "%7s %7s %17s %18s %15s %14s %19s %20s\n",
					"enc_idx", "enc_id", "cur_input_buf_cnt",
					"cur_output_buf_cnt", "left_recv_frame", "left_enc_frame",
					"total_input_buf_cnt", "total_output_buf_cnt");
			}
			seq_printf(s, "%7d %7s %17d %18d %15d %14d %19d %20d\n",
				dev->jpu_ctx[i].context.instance_index,
				get_codec(&dev->jpu_ctx[i]),
				status->cur_input_buf_cnt,
				status->cur_output_buf_cnt,
				status->left_recv_frame,
				status->left_enc_frame,
				status->total_input_buf_cnt,
				status->total_output_buf_cnt);
		}
	}
	spin_unlock(&dev->jpu_info_spinlock);
	return 0;
}

static int jpu_jenc_open(struct inode *inode, struct file *file)
{
	return single_open(file, jpu_jenc_show, inode->i_private);
}

static const struct file_operations jpu_jenc_fops = {
	.open = jpu_jenc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

//////////////// jdec
static int jpu_jdec_show(struct seq_file *s, void *unused)
{
	int i;
	int output = 0;
	hb_jpu_dev_t *dev = (hb_jpu_dev_t *)s->private;

	if (dev == NULL)
		return 0;
	for (i = 0; i < MAX_NUM_JPU_INSTANCE; i++) {
		if (dev->jpu_ctx[i].valid && (dev->jpu_ctx[i].context.encoder == 0)) {
			if (output == 0) {
				output = 1;
				seq_printf(s, "\n");
				seq_printf(s, "----decode param----\n");
				seq_printf(s, "%7s %7s %9s %7s %18s %19s %15s %6s %6s\n",
					"dec_idx", "dec_id", "feed_mode", "pix_fmt",
					"bitstream_buf_size", "bitstream_buf_count",
					"frame_buf_count", "mirror", "rotate");
			}
			seq_printf(s, "%7d %7s %9d %7d %18d %19d %15d ",
				dev->jpu_ctx[i].context.instance_index,
				get_codec(&dev->jpu_ctx[i]),
				dev->jpu_ctx[i].context.video_dec_params.feed_mode,
				dev->jpu_ctx[i].context.video_dec_params.pix_fmt,
				dev->jpu_ctx[i].context.video_dec_params.bitstream_buf_size,
				dev->jpu_ctx[i].context.video_dec_params.bitstream_buf_count,
				dev->jpu_ctx[i].context.video_dec_params.frame_buf_count);
			if (dev->jpu_ctx[i].context.codec_id == MEDIA_CODEC_ID_MJPEG) {
				mc_mjpeg_dec_config_t *mjpeg =
					&dev->jpu_ctx[i].context.video_dec_params.mjpeg_dec_config;
				seq_printf(s, "%6d %6d\n",
					mjpeg->mir_direction, mjpeg->rot_degree);
			} else if (dev->jpu_ctx[i].context.codec_id ==
												MEDIA_CODEC_ID_JPEG) {
				mc_jpeg_dec_config_t *jpeg =
					&dev->jpu_ctx[i].context.video_dec_params.jpeg_dec_config;
				seq_printf(s, "%6d %6d\n",
					jpeg->mir_direction, jpeg->rot_degree);
			} else {
				seq_printf(s, "\n");
			}
		}
	}

	spin_lock(&dev->jpu_info_spinlock);
	output = 0;
	for (i = 0; i < MAX_NUM_JPU_INSTANCE; i++) {
		if (dev->jpu_ctx[i].valid && (dev->jpu_ctx[i].context.encoder == 0)) {
			mc_mjpeg_jpeg_output_frame_info_t *frameinfo
								= &(dev->jpu_status[i].frame_info);
			if (output == 0) {
				output = 1;
				seq_printf(s, "\n");
				seq_printf(s, "----decode frameinfo----\n");
				seq_printf(s, "%7s %7s %13s %14s\n", "dec_idx", "dec_id",
					"display_width", "display_height");
			}
			seq_printf(s, "%7d %7s %13d %14d\n",
				dev->jpu_ctx[i].context.instance_index,
				get_codec(&dev->jpu_ctx[i]),
				frameinfo->display_width,
				frameinfo->display_height);
		}
	}

	output = 0;
	for (i = 0; i < MAX_NUM_JPU_INSTANCE; i++) {
		if (dev->jpu_ctx[i].valid && (dev->jpu_ctx[i].context.encoder == 0)) {
			mc_inter_status_t *status =	&(dev->jpu_status[i].status);
			if (output == 0) {
				output = 1;
				seq_printf(s, "\n");
				seq_printf(s, "----decode status----\n");
				seq_printf(s, "%7s %7s %17s %18s %19s %20s\n", "dec_idx",
					"dec_id", "cur_input_buf_cnt", "cur_output_buf_cnt",
					"total_input_buf_cnt", "total_output_buf_cnt");
			}
			seq_printf(s, "%7d %7s %17d %18d %19d %20d\n",
				dev->jpu_ctx[i].context.instance_index,
				get_codec(&dev->jpu_ctx[i]),
				status->cur_input_buf_cnt,
				status->cur_output_buf_cnt,
				status->total_input_buf_cnt,
				status->total_output_buf_cnt);
		}
	}
	spin_unlock(&dev->jpu_info_spinlock);
	return 0;
}

static int jpu_jdec_open(struct inode *inode, struct file *file)
{
	return single_open(file, jpu_jdec_show, inode->i_private);
}

static const struct file_operations jpu_jdec_fops = {
	.open = jpu_jdec_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int jpu_probe(struct platform_device *pdev)
{
	hb_jpu_dev_t *dev = NULL;
	struct resource *res = NULL;
	int err = 0;
	int i;

	dev_dbg(&pdev->dev, "%s()\n", __func__);
	dev = devm_kzalloc(&pdev->dev, sizeof(hb_jpu_dev_t), GFP_KERNEL);
	if (!dev) {
		dev_err(&pdev->dev, "Not enough memory for JPU device.\n");
		err = -ENOMEM;
		goto ERR_RESOURSE;
	}
	dev->device = &pdev->dev;

	dev->plat_data = pdev->dev.platform_data;
	dev->plat_data =
	    devm_kzalloc(&pdev->dev, sizeof(hb_jpu_platform_data_t),
			 GFP_KERNEL);
	if (!dev->plat_data) {
		dev_err(&pdev->dev,
			"Not enough memory for JPU platform data\n");
		err = -ENOMEM;
		goto ERR_RESOURSE;
	}
	jpu_parse_dts(dev->device->of_node, dev);

	err = hb_jpu_init_pm(dev->device);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to setup jpu clock & power\n");
		goto ERR_INIT_PM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "failed to get memory resource\n");
		err = -ENOENT;
		goto ERR_RES_MEM;
	}
	dev->jpu_mem = request_mem_region(res->start, resource_size(res),
					  pdev->name);
	if (!dev->jpu_mem) {
		dev_err(&pdev->dev, "failed to get memory region\n");
		err = -ENOENT;
		goto ERR_REQ_MEM;
	}
	dev->regs_base = ioremap_nocache(dev->jpu_mem->start,
					 resource_size(dev->jpu_mem));
	if (!dev->regs_base) {
		dev_err(&pdev->dev, "failed to ioremap address region\n");
		err = -ENOENT;
		goto ERR_IO_REMAP;
	}
	dev_dbg(&pdev->dev,
		"jpu IO memory resource: physical base addr = 0x%llx,"
		"virtual base addr = %p\n", dev->jpu_mem->start,
		dev->regs_base);

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev, "failed to get irq resource\n");
		err = -ENOENT;
		goto ERR_RES_IRQ;
	}
	dev->irq = res->start;
	// TODO Add top half irq and bottom half irq?
	err = request_threaded_irq(dev->irq, jpu_irq_handler, NULL,
				   IRQF_TRIGGER_RISING, pdev->name, dev);
	if (err) {
		dev_err(&pdev->dev,
			"failed to install register interrupt handler\n");
		goto ERR_REQ_IRQ;
	}
	irq_set_affinity_hint(dev->irq, get_cpu_mask(1));
	dev_dbg(&pdev->dev, "jpu irq number: irq = %d\n", dev->irq);

	dev->jpu_class = class_create(THIS_MODULE, JPU_DEV_NAME);
	if (IS_ERR(dev->jpu_class)) {
		dev_err(&pdev->dev, "failed to create class\n");
		err = PTR_ERR(dev->jpu_class);
		goto ERR_CREATE_CLASS;
	}

	/* get the major number of the character device */
	err = alloc_chrdev_region(&dev->jpu_dev_num, 0, 1, JPU_DEV_NAME);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to allocate character device\n");
		goto ERR_ALLOCATE_CHR;
	} else {
		dev->major = MAJOR(dev->jpu_dev_num);
		dev->minor = MINOR(dev->jpu_dev_num);
	}
	dev_dbg(&pdev->dev, "jpu device number: major = %d, minor = %d\n",
		dev->major, dev->minor);

	/* initialize the device structure and register the device with the kernel */
	cdev_init(&dev->cdev, &jpu_fops);
	err = cdev_add(&dev->cdev, dev->jpu_dev_num, 1);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to add character device\n");
		goto ERR_ADD_CHR;
	}

	dev->jpu_dev = device_create(dev->jpu_class, NULL, dev->jpu_dev_num,
				     NULL, JPU_DEV_NAME);
	if (IS_ERR(dev->jpu_dev)) {
		err = PTR_ERR(dev->jpu_dev);
		dev->jpu_dev = NULL;
		goto ERR_CREATE_DEV;
	}

	platform_set_drvdata(pdev, dev);

	err = hb_jpu_clk_get(dev);
	if (err < 0) {
		goto ERR_GET_CLK;
	}
	hb_jpu_clk_put(dev);

#ifdef CONFIG_ION_HOBOT
	dev->jpu_ion_client = ion_client_create(ion_exynos, JPU_DEV_NAME);
	if (IS_ERR(dev->jpu_ion_client)) {
		dev_err(&pdev->dev, "failed to ion_client_create\n");
		err = PTR_ERR(dev->jpu_dev);
		goto ERR_ION_CLIENT;
	}
#endif

#ifdef JPU_SUPPORT_RESERVED_VIDEO_MEMORY
	if (s_jmem.base_addr == 0) {
		s_video_memory.size = JPU_INIT_VIDEO_MEMORY_SIZE_IN_BYTE;
		s_video_memory.phys_addr = JPU_DRAM_PHYSICAL_BASE;
		//s_video_memory.base = (unsigned long)ioremap_nocache(s_video_memory.phys_addr, PAGE_ALIGN(s_video_memory.size));
		s_video_memory.base = (unsigned long)__va(s_video_memory.phys_addr);	// pfn_to_kaddr(pfn) //__VA(s_video_memory.phys_addr);
		//s_video_memory.base = phys_to_virt(s_video_memory.phys_addr);
		if (!s_video_memory.base) {
			dev_err(&pdev->dev,
				"fail to remap video memory physical phys_addr=0x%lx,"
				"base==0x%lx, size=%d\n",
				s_video_memory.phys_addr, s_video_memory.base,
				(int)s_video_memory.size);
			err = -ENOMEM;
			goto ERR_RESERVED_MEM;
		}

		if (jmem_init
		    (&s_jmem, s_video_memory.phys_addr,
		     s_video_memory.size) < 0) {
			err = -ENOMEM;
			dev_err(&pdev->dev, "fail to init jmem system\n");
			goto ERROR_INIT_VMEM;
		}
		dev_dbg(&pdev->dev,
			"success to probe jpu device with reserved video memory"
			"phys_addr==0x%lx, base = =0x%lx\n",
			s_video_memory.phys_addr, s_video_memory.base);
	}
#endif
	for (i = 0; i < MAX_HW_NUM_JPU_INSTANCE; i++) {
		init_waitqueue_head(&dev->interrupt_wait_q[i]);
	}

	for (i = 0; i < MAX_NUM_JPU_INSTANCE; i++) {
		init_waitqueue_head(&dev->poll_wait_q[i]);
	}

	init_waitqueue_head(&dev->poll_int_wait);

	dev->async_queue = NULL;
	dev->open_count = 0;
	mutex_init(&dev->jpu_mutex);
	sema_init(&dev->jpu_sem, 1);
	spin_lock_init(&dev->jpu_spinlock);
	spin_lock_init(&dev->jpu_info_spinlock);
	spin_lock_init(&dev->irq_spinlock);
	dev->irq_trigger = 0;

	INIT_LIST_HEAD(&dev->jbp_head);
	INIT_LIST_HEAD(&dev->inst_list_head);

	dev->debug_root = debugfs_create_dir("jpu", NULL);
	if (!dev->debug_root) {
		pr_err("hai: failed to create debugfs root directory.\n");
		goto ERR_GET_CLK;
	}

	dev->debug_file_jenc = debugfs_create_file("jenc", 0664,
						dev->debug_root,
						dev, &jpu_jenc_fops);
	if (!dev->debug_file_jenc) {
		char buf[256], *path;

		path = dentry_path_raw(dev->debug_root, buf, 256);
		pr_err("Failed to create client debugfs at %s/%s\n",
			path, "jenc");
	}

	dev->debug_file_jdec = debugfs_create_file("jdec", 0664,
						dev->debug_root,
						dev, &jpu_jdec_fops);
	if (!dev->debug_file_jdec) {
		char buf[256], *path;

		path = dentry_path_raw(dev->debug_root, buf, 256);
		pr_err("Failed to create client debugfs at %s/%s\n",
			path, "jdec");
	}

	return 0;

#ifdef JPU_SUPPORT_RESERVED_VIDEO_MEMORY
	jmem_exit(&s_jmem);
ERROR_INIT_VMEM:
ERR_RESERVED_MEM:
#endif
#ifdef CONFIG_ION_HOBOT
	ion_client_destroy(dev->jpu_ion_client);
ERR_ION_CLIENT:
#endif
ERR_GET_CLK:
	device_destroy(dev->jpu_class, dev->jpu_dev_num);
ERR_CREATE_DEV:
	cdev_del(&dev->cdev);
ERR_ADD_CHR:
	unregister_chrdev_region(dev->jpu_dev_num, 1);
ERR_ALLOCATE_CHR:
	class_destroy(dev->jpu_class);
ERR_CREATE_CLASS:
	free_irq(dev->irq, dev);
ERR_REQ_IRQ:
ERR_RES_IRQ:
	iounmap(dev->regs_base);
ERR_IO_REMAP:
	release_mem_region(dev->jpu_mem->start, resource_size(dev->jpu_mem));
ERR_REQ_MEM:
ERR_RES_MEM:
	hb_jpu_final_pm(dev->device);
ERR_INIT_PM:
ERR_RESOURSE:
	return err;
}

static int jpu_remove(struct platform_device *pdev)
{
	hb_jpu_dev_t *dev;

	dev_dbg(&pdev->dev, "%s()\n", __func__);

	dev = platform_get_drvdata(pdev);

	debugfs_remove_recursive(dev->debug_file_jenc);
	debugfs_remove_recursive(dev->debug_file_jdec);
	debugfs_remove_recursive(dev->debug_root);

	if (dev->instance_pool.base) {
		vfree((const void *)dev->instance_pool.base);
		dev->instance_pool.base = 0;
	}
#ifdef JPU_SUPPORT_RESERVED_VIDEO_MEMORY
	if (s_video_memory.base) {
		iounmap((void *)s_video_memory.base);
		s_video_memory.base = 0;
		jmem_exit(&s_jmem);
	}
#endif

#ifdef CONFIG_ION_HOBOT
	ion_client_destroy(dev->jpu_ion_client);
#endif

	//hb_jpu_clk_disable(dev);
	//hb_jpu_clk_put(dev);
	device_destroy(dev->jpu_class, dev->jpu_dev_num);
	cdev_del(&dev->cdev);
	unregister_chrdev_region(dev->jpu_dev_num, 1);
	class_destroy(dev->jpu_class);
	irq_set_affinity_hint(dev->irq, NULL);
	free_irq(dev->irq, dev);
	iounmap(dev->regs_base);
	release_mem_region(dev->jpu_mem->start, resource_size(dev->jpu_mem));
	hb_jpu_final_pm(dev->device);

	return 0;
}

#ifdef CONFIG_PM
static int jpu_suspend(struct platform_device *pdev, pm_message_t state)
{
	hb_jpu_dev_t *dev;

	jpu_debug_enter();
	dev = (hb_jpu_dev_t *) platform_get_drvdata(pdev);
	if (!dev) {
		jpu_err("The jpu dev is NULL!");
		return -1;
	}
	irq_set_affinity_hint(dev->irq, NULL);
	jpu_debug_leave();
	return 0;

}

static int jpu_resume(struct platform_device *pdev)
{
	hb_jpu_dev_t *dev;

	jpu_debug_enter();
	dev = (hb_jpu_dev_t *) platform_get_drvdata(pdev);
	if (!dev) {
		jpu_err("The jpu dev is NULL!");
		return -1;
	}
	irq_set_affinity_hint(dev->irq, get_cpu_mask(1));
	jpu_debug_leave();
	return 0;
}
#else
#define    jpu_suspend    NULL
#define    jpu_resume    NULL
#endif /* !CONFIG_PM */

/* Power management */
/*static const struct dev_pm_ops jpu_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(jpu_suspend, jpu_resume)
};*/

static const struct of_device_id jpu_of_match[] = {
	{
	 .compatible = "hobot,hobot_jpu",
	 .data = NULL,
	 },
	{},
};

// static const struct attribute_group *jpu_attr_groups[] = {};
static struct platform_driver jpu_driver = {
	.probe = jpu_probe,
	.remove = jpu_remove,
	.suspend = jpu_suspend,
	.resume = jpu_resume,
	.driver = {
		   .name = JPU_PLATFORM_DEVICE_NAME,
		   .of_match_table = jpu_of_match,
		   //.pm = &jpu_pm_ops
       // .groups = jpu_attr_groups,
		   },
};

module_platform_driver(jpu_driver);

MODULE_AUTHOR("Hobot");
MODULE_DESCRIPTION("Hobot JPEG processing unit linux driver");
MODULE_LICENSE("GPL v2");
