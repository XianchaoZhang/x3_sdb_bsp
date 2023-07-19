/*
 * Horizon Robotics
 *
 *  Copyright (C) 2020 Horizon Robotics Inc.
 *  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/seq_file.h>
#include <linux/pwm.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_address.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>

#include "hobot_lt9211.h"

#define PWM_PERIOD_DEFAULT 1000
#define PWM_DUTY_DEFAULT 10
//int lt9211_reset_pin = 85;

#define LT9211_CDEV_MAGIC 'i'
#define LCD_BACKLIGHT_SET	_IOW(LT9211_CDEV_MAGIC, 0x11, unsigned int)
#define CONVERTER_OUTPUT_CTRL       _IOW(LT9211_CDEV_MAGIC, 0x12, unsigned int)

int lt9211_reset_pin;
int lcd_reset_pin;
//int display_type = HDMI_TYPE;

struct pwm_device *lcd_backlight_pwm;

struct x2_lt9211_s *g_x2_lt9211;
//int vio_sys_clk_trigger_init(struct device *dev, unsigned int select);
//int ips_set_iar_clk(void);

static int lt9211_open(struct inode *inode, struct file *filp)
{
	struct x2_lt9211_s  *lt9211_p;

	lt9211_p = container_of(inode->i_cdev, struct x2_lt9211_s, cdev);
	filp->private_data = lt9211_p;
	return 0;
}

static ssize_t lt9211_write(struct file *filp, const char __user *ubuf,
		size_t len, loff_t *ppos)
{
	return len;
}


static ssize_t lt9211_read(struct file *filp, char __user *ubuf,
		size_t len, loff_t *offp)
{
	unsigned int size;

	size = 0;
	return size;
}

static long lt9211_ioctl(struct file *filp, unsigned int cmd, unsigned long p)
{
	int ret;
	void __user *arg = (void __user *)p;

	mutex_lock(&g_x2_lt9211->lt9211_mutex);
	switch (cmd) {
	case LCD_BACKLIGHT_SET:
		{
			unsigned int duty_level;

			LT9211_DEBUG("%s: begin set lcd backlight\n", __func__);
			if (copy_from_user(&duty_level, arg,
						sizeof(unsigned int)))
				return -EFAULT;
			pr_info("%s: level is %d!!\n", __func__, duty_level);
			ret = set_lcd_backlight(duty_level);
		}
		break;
	case CONVERTER_OUTPUT_CTRL:
		{
			unsigned int ctrl_val;

			if (copy_from_user(&ctrl_val, arg,
						sizeof(unsigned int)))
				return -EFAULT;
			pr_info("%s: output is %d!!\n", __func__, ctrl_val);
			LT9211_DEBUG("%s: begin set lt9211 output\n", __func__);
			ret = set_lt9211_output_ctrl(ctrl_val);
		}
		break;
	default:
		ret = -EPERM;
		break;
		}
	mutex_unlock(&g_x2_lt9211->lt9211_mutex);
	return ret;
}


int lt9211_release(struct inode *inode, struct file *filp)
{
	filp->private_data = NULL;
	return 0;
}


static int lcd_backlight_init(void)
{
	int ret = 0;

	LT9211_DEBUG("initialize lcd backligbt!!!\n");

	lcd_backlight_pwm = pwm_request(0, "lcd-pwm");
	if (lcd_backlight_pwm == NULL) {
		pr_err("\nNo pwm device 0!!!!\n");
		return -ENODEV;
	}
	LT9211_DEBUG("pwm request 0 is okay!!!\n");
	/**
	 * pwm_config(struct pwm_device *pwm, int duty_ns,
			     int period_ns) - change a PWM device configuration
	 * @pwm: PWM device
	 * @duty_ns: "on" time (in nanoseconds)
	 * @period_ns: duration (in nanoseconds) of one cycle
	 *
	 * Returns: 0 on success or a negative error code on failure.
	 */
	ret = pwm_config(lcd_backlight_pwm, PWM_DUTY_DEFAULT,
			PWM_PERIOD_DEFAULT);
	// 50Mhz,20ns period, on = 20ns
	if (ret) {
		pr_err("\nError config pwm!!!!\n");
		return ret;
	}
	LT9211_DEBUG("pwm config is okay!!!\n");
/*
 *	ret = pwm_set_polarity(lcd_backlight_pwm,
 *			PWM_POLARITY_NORMAL);
 *	if (ret) {
 *		pr_err("\nError set pwm polarity!!!!\n");
 *		return ret;
 *	}
 *	pr_debug("pwm set polarity is okay!!!\n");
 */
	ret = pwm_enable(lcd_backlight_pwm);
	if (ret) {
		pr_err("\nError enable pwm!!!!\n");
		return ret;
	}
	LT9211_DEBUG("pwm enable is okay!!!\n");

	return 0;
}

int lcd_backlight_change(unsigned int duty)
{
	int ret = 0;

	if (lcd_backlight_pwm == NULL) {
		pr_err("pwm is not init!!\n");
		return -1;
	}

	pwm_disable(lcd_backlight_pwm);

	ret = pwm_config(lcd_backlight_pwm, duty, PWM_PERIOD_DEFAULT);

	if (ret) {
		pr_err("\nError config pwm!!!!\n");
		return ret;
	}

	ret = pwm_enable(lcd_backlight_pwm);
	if (ret) {
		pr_err("\nError enable pwm!!!!\n");
		return ret;
	}

	return 0;

}

int set_lcd_backlight(unsigned int backlight_level)
{
	int retval = 0;
	unsigned int duty = 0;

	if (display_type == LCD_7_TYPE) {
		if (backlight_level == 0) {
			duty = PWM_PERIOD_DEFAULT - 1;
		} else if (backlight_level <= 10 && backlight_level > 0) {
			duty = PWM_PERIOD_DEFAULT -
				PWM_PERIOD_DEFAULT / 10 * backlight_level;
		} else {
			pr_info("error backlight value, exit!!\n");
			return -1;
		}

		retval = lcd_backlight_change(duty);
		if (retval) {
			pr_info("error set lcd backlight!!\n");
			return retval;
		}
	}
	return 0;
}

static const struct file_operations x2_lt9211_fops = {
	.owner	= THIS_MODULE,
	.open	= lt9211_open,
	.read	= lt9211_read,
	.write	= lt9211_write,
	.release	= lt9211_release,
	.unlocked_ioctl	= lt9211_ioctl,
	.compat_ioctl = lt9211_ioctl
};

struct kobject *lt9211_kobj;
static ssize_t lt9211_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	char *s = buf;

	return (s - buf);
}

static ssize_t lt9211_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t n)
{
	char *tmp;
	int error = -EINVAL;
	int ret = 0;
	unsigned long value = 0;
	unsigned int duty = 0;

	tmp = (char *)buf;
	ret = kstrtoul(tmp, 0, &value);
	if (ret == 0) {
		if (value == 0) {
			duty = PWM_PERIOD_DEFAULT - 1;
		} else if (value <= 10 && value > 0) {
			duty = PWM_PERIOD_DEFAULT -
				PWM_PERIOD_DEFAULT / 10 * value;
		} else {
			pr_info("error backlight value, exit!!\n");
			return error;
		}
	} else {
		pr_info("error input, exit!!\n");
		return error;
	}

	ret = lcd_backlight_change(duty);

	return ret ? error : n;

}

static struct kobj_attribute lt9211_test_attr = {
	.attr	= {
		.name = __stringify(lt9211_test_attr),
		.mode = 0644,
	},
	.show	= lt9211_show,
	.store	= lt9211_store,
};

static struct attribute *attributes[] = {
	&lt9211_test_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attributes,
};

static int x2_lt9211_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = client->adapter;
	struct device *x2_lt9211_dev;
	dev_t devno;
	int ret = 0;
	//void __iomem *iar_clk_regaddr;
	//void __iomem *vio_refclk_regaddr;
	//int regvalue = 0;
	//int regvalue1 = 0;

	pr_debug("x2 lt9211 probe start.\n");

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;

	g_x2_lt9211 = devm_kzalloc(&client->dev, sizeof(struct x2_lt9211_s),
			GFP_KERNEL);
	if (!g_x2_lt9211)
		return -ENOMEM;
	g_x2_lt9211->client = client;

	mutex_init(&g_x2_lt9211->lt9211_mutex);
	ret = alloc_chrdev_region(&devno, 0, 1, "x2_lt9211");
	if (ret < 0) {
		pr_err("Error %d while alloc chrdev x2_fpgactrl", ret);
		goto err;
	}
	g_x2_lt9211->dev_num = devno;
	g_x2_lt9211->name = "x2_lt9211";

	g_x2_lt9211->major = MAJOR(devno);
	g_x2_lt9211->minor = MINOR(devno);

	cdev_init(&(g_x2_lt9211->cdev), &x2_lt9211_fops);
	g_x2_lt9211->cdev.owner = THIS_MODULE;
	ret = lt9211_chip_id();
	if (ret != 0) {
		pr_err("not found lt9211 device, exit probe!!!\n");
		return ret;
	}
	ret = cdev_add(&(g_x2_lt9211->cdev), devno, 1);
	if (ret) {
		pr_err("Error %d while adding x2 fpgactrl cdev", ret);
		goto err;
	}

	g_x2_lt9211->x2_lt9211_classes = class_create(THIS_MODULE, "x2_lt9211");
	if (IS_ERR(g_x2_lt9211->x2_lt9211_classes)) {
		pr_err("[%s:%d] class_create error\n",
				__func__, __LINE__);
		ret = PTR_ERR(g_x2_lt9211->x2_lt9211_classes);
		goto err;
	}
	x2_lt9211_dev = device_create(g_x2_lt9211->x2_lt9211_classes,
			NULL, MKDEV(g_x2_lt9211->major, 0),
			NULL, g_x2_lt9211->name);
	if (IS_ERR(x2_lt9211_dev)) {
		pr_err("[%s] deivce create error\n", __func__);
		ret = PTR_ERR(x2_lt9211_dev);
		goto err;
	}

/*	//---------------------------------------------------------------
 *	//config vio refclk as 408MHz
 *	vio_refclk_regaddr = ioremap_nocache(0xA1000000 + 0x40, 4);
 *	regvalue1 = readl(vio_refclk_regaddr);
 *	printk("vio_refclk_regaddr is 0x%p, value is 0x%x\n",
 *					vio_refclk_regaddr, regvalue1);
 *	regvalue1 = (regvalue1 & 0xf0ffffff) | 0x04000000;
 *	writel(regvalue1, vio_refclk_regaddr);
 *	printk("write vio_refclk_regaddr ok!\n");
 *	regvalue1 = readl(vio_refclk_regaddr);
 *	printk("vio_refclk_regaddr is 0x%p, value is 0x%x\n",
 *					vio_refclk_regaddr, regvalue1);
 *	//----------------------------------------------------------------
 *	//config iar clk as vio_refclk/14 = 29.2MHz
 *	iar_clk_regaddr = ioremap_nocache(0xA1000000 + 0x240, 4);
 *	regvalue = readl(iar_clk_regaddr);
 *	printk("iar_clk_regaddr is 0x%p, value is 0x%x\n", iar_clk_regaddr,
 *								regvalue);
 *	//regvalue = (regvalue & 0xff0fffff) | 0x00d00000;//29.2M
 *	//regvalue = (regvalue & 0xff0fffff) | 0x00b00000;//33.3M
 *	regvalue = (regvalue & 0xff0fffff) | 0x00c00000;//32M
 *	writel(regvalue, iar_clk_regaddr);
 *	printk("write iar_clk_regaddr ok!\n");
 *	regvalue = readl(iar_clk_regaddr);
 *	printk("iar_clk_regaddr is 0x%p, value is 0x%x\n", iar_clk_regaddr,
 *								regvalue);
 *	//-----------------------------------------------------------------
 */
	display_type = LCD_7_TYPE;
	ret = of_property_read_u32(client->dev.of_node, "x2_lt9211_reset_pin",
					&lt9211_reset_pin);
	if (ret) {
		dev_err(&client->dev, "Filed to get rst_pin %d\n", ret);
		return ret;
	}

	ret = of_property_read_u32(x2_lt9211_dev->of_node,
			"x2_lcd_reset_pin", &lcd_reset_pin);
	if (ret)
		dev_err(x2_lt9211_dev, "Filed to get x2_lcd_reset_pin\n");
//	ret = of_property_read_u32(x2_lt9211_dev->of_node,
//	"x2_lcd_pwm_pin", &lcd_pwm_pin);
//	if (ret) {
//		dev_err(x2_lt9211_dev, "Filed to get x2_lcd_pwm_pin\n");
//	}

	i2c_set_clientdata(client, g_x2_lt9211);

	client->flags = I2C_CLIENT_SCCB;
	LT9211_DEBUG("chip found @ 0x%02x (%s)\n",
			client->addr << 1, client->adapter->name);

	lt9211_kobj = kobject_create_and_add("x2_lt9211", NULL);
	if (!lt9211_kobj)
		return -ENOMEM;
	sysfs_create_group(lt9211_kobj, &attr_group);


	ret = lcd_backlight_init();
	if (ret)
		LT9211_DEBUG("\nlcd backlight init err!\n");

	ret = lt9211_dsi_lcd_init();
	if (ret) {
		pr_err("\nlt9211 and dsi panel init err!\n");
		return ret;
	}

	pr_debug("x2_lt9211 probe OK!!!\n");
	return 0;
err:
	class_destroy(g_x2_lt9211->x2_lt9211_classes);
	cdev_del(&g_x2_lt9211->cdev);
	unregister_chrdev_region(MKDEV(g_x2_lt9211->major, 0), 1);
	if (g_x2_lt9211)
		devm_kfree(&client->dev, g_x2_lt9211);
	return ret;
}

static int x2_lt9211_remove(struct i2c_client *client)
{
	struct x2_lt9211_s *x2_lt9211 = i2c_get_clientdata(client);

	class_destroy(x2_lt9211->x2_lt9211_classes);
	cdev_del(&x2_lt9211->cdev);
	unregister_chrdev_region(MKDEV(g_x2_lt9211->major, 0), 1);
	if (x2_lt9211)
		devm_kfree(&client->dev, x2_lt9211);
	return 0;
}

static const struct of_device_id x2_lt9211_of_match[] = {
	{ .compatible = "x2,lt9211", .data = NULL },
	{}
};
MODULE_DEVICE_TABLE(of, x2_lt9211_of_match);


static struct i2c_driver x2_lt9211_driver = {
	.driver = {
		.name = "x2_lt9211",
		.of_match_table = x2_lt9211_of_match,
	},
	.probe = x2_lt9211_probe,
	.remove = x2_lt9211_remove,
};
module_i2c_driver(x2_lt9211_driver);

MODULE_DESCRIPTION("x2_lt9211_converter_driver");
MODULE_AUTHOR("GuoRui <rui.guo@hobot.cc>");
MODULE_LICENSE("GPL v2");
