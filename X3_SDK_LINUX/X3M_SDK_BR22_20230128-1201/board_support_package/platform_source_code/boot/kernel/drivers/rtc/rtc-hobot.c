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

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/bcd.h>
#include <linux/rtc.h>
#include <linux/delay.h>
#include <linux/pm_wakeirq.h>

/* Hobot RTC register offsets and bits */
#define HOBOT_RTC_CTRL_REG           0x04
#define HOBOT_RTC_SEC_CFG_REG        0x0C
#define HOBOT_RTC_TIME_CFG_REG       0x10
#define HOBOT_RTC_DATE_CFG_REG       0x14
#define HOBOT_RTC_CUR_MSEC_REG       0x18
#define HOBOT_RTC_CUR_TIME_REG       0x1C
#define HOBOT_RTC_CUR_DATE_REG       0x20
#define HOBOT_RTC_ALARM_TIME_REG     0x24
#define HOBOT_RTC_ALARM_DATE_REG     0x28
#define HOBOT_RTC_TICK_CNT_REG       0x2C
#define HOBOT_RTC_CUR_TICK_REG       0x30
#define HOBOT_RTC_INT_STA_REG        0x34
#define HOBOT_RTC_INT_MASK_REG       0x38
#define HOBOT_RTC_INT_SETMASK_REG    0x3C
#define HOBOT_RTC_INT_UNMASK_REG     0x40

#define HOBOT_RTC_AL_SEC_EN          BIT(0)
#define HOBOT_RTC_AL_MIN_EN          BIT(1)
#define HOBOT_RTC_AL_HOUR_EN         BIT(2)
#define HOBOT_RTC_AL_WEEK_EN         BIT(3)
#define HOBOT_RTC_AL_DAY_EN          BIT(4)
#define HOBOT_RTC_AL_MON_EN          BIT(5)
#define HOBOT_RTC_AL_YEAR_EN         BIT(6)
#define HOBOT_RTC_AL_EN              BIT(7)
#define HOBOT_RTC_TICK_EN            BIT(8)
#define HOBOT_RTC_EN                 BIT(9)
#define HOBOT_RTC_AL_INT_EN          BIT(0)
#define HOBOT_RTC_TICK_INT_EN        BIT(1)

/* Macros to read fields in consolidated time (CT) registers */
/* rtc_time_cfg_reg */
#define RTC_TIME_CFG_SEC(n)       (((n) & 0x7F) << 0)
#define RTC_TIME_CFG_MIN(n)       (((n) & 0x7F) << 8)
#define RTC_TIME_CFG_HOUR(n)      (((n) & 0x3F) << 16)
#define RTC_TIME_CFG_WEEK(n)      (((n) & 0x7) << 24)
/* rtc_date_cfg_reg */
#define RTC_DATE_CFG_DAY(n)       (((n) & 0x3F) << 0)
#define RTC_DATE_CFG_MON(n)       (((n) & 0x1F) << 8)
#define RTC_DATE_CFG_YEAR_L(n)    (((n) & 0xFF) << 16)
#define RTC_DATE_CFG_YEAR_H(n)    (((n) & 0xFF) << 24)
/* rtc_cur_time_reg */
#define RTC_TIME_GET_SEC(n)       (((n) >> 0) & 0x7F)
#define RTC_TIME_GET_MIN(n)       (((n) >> 8) & 0x7F)
#define RTC_TIME_GET_HOUR(n)      (((n) >> 16) & 0x3F)
#define RTC_TIME_GET_WEEK(n)      (((n) >> 24) & 0x7)
/* rtc_cur_date_reg */
#define RTC_DATE_GET_DAY(n)       (((n) >> 0) & 0x3F)
#define RTC_DATE_GET_MON(n)       (((n) >> 8) & 0x1F)
#define RTC_DATE_GET_YEAR_L(n)    (((n) >> 16) & 0xFF)
#define RTC_DATE_GET_YEAR_H(n)    (((n) >> 24) & 0xFF)
/* rtc_alarm_time_reg */
#define RTC_ALARM_CFG_SEC(n)      (((n) & 0x7F) << 0)
#define RTC_ALARM_CFG_MIN(n)      (((n) & 0x7F) << 8)
#define RTC_ALARM_CFG_HOUR(n)     (((n) & 0x3F) << 16)
#define RTC_ALARM_CFG_WEEK(n)     (((n) & 0x7) << 24)
/* rtc_alarm_date_reg */
#define RTC_ALARM_CFG_DAY(n)      (((n) & 0x3F) << 0)
#define RTC_ALARM_CFG_MON(n)      (((n) & 0x1F) << 8)
#define RTC_ALARM_CFG_YEAR_L(n)   (((n) & 0xFF) << 16)
#define RTC_ALARM_CFG_YEAR_H(n)   (((n) & 0xFF) << 24)
/* rtc_alarm_get_time */
#define RTC_ALARM_GET_SEC(n)      (((n) >> 0) & 0x7F)
#define RTC_ALARM_GET_MIN(n)      (((n) >> 8) & 0x7F)
#define RTC_ALARM_GET_HOUR(n)     (((n) >>16) & 0x3F)
#define RTC_ALARM_GET_WEEK(n)     (((n) >>24) & 0x7)
/* rtc_alarm_get_date */
#define RTC_ALARM_GET_DAY(n)      (((n) >>0) & 0x3F)
#define RTC_ALARM_GET_MON(n)      (((n) >>8) & 0x1F)
#define RTC_ALARM_GET_YEAR_L(n)   (((n) >>16) & 0xFF)
#define RTC_ALARM_GET_YEAR_H(n)   (((n) >>24) & 0xFF)

#define DECIMAL_WIDTH	10
#define	KEEP_ALIVE_TIME	5000
#define UNLOCK	"unlock"

#define hobot_rtc_rd(dev, reg)       ioread32((dev)->rtc_base + (reg))
#define hobot_rtc_wr(dev, reg, val) \
	do { \
		iowrite32((val), (dev)->rtc_base + (reg)); \
		mdelay(2); \
	} while (0)

struct hobot_rtc {
	struct rtc_device *rtc;
	int irq;
	void __iomem *rtc_base;
	bool    wakeup;
	uint32_t keep_alive_time;
#ifdef CONFIG_PM
	u32 rtc_regs[4];
#endif
};

static int hobot_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	u32 val;
	struct hobot_rtc *rtc = dev_get_drvdata(dev);

	/* Disable RTC during update */
	val = hobot_rtc_rd(rtc, HOBOT_RTC_CTRL_REG);
	val &= (~HOBOT_RTC_EN);
	hobot_rtc_wr(rtc, HOBOT_RTC_CTRL_REG, val);

	val = RTC_TIME_CFG_SEC(bin2bcd(tm->tm_sec)) |
	      RTC_TIME_CFG_MIN(bin2bcd(tm->tm_min)) |
	      RTC_TIME_CFG_HOUR(bin2bcd(tm->tm_hour)) |
	      RTC_TIME_CFG_WEEK(bin2bcd(tm->tm_wday==0 ? 7:tm->tm_wday));
	hobot_rtc_wr(rtc, HOBOT_RTC_TIME_CFG_REG, val);

	val = RTC_DATE_CFG_DAY(bin2bcd(tm->tm_mday)) |
	      RTC_DATE_CFG_MON(bin2bcd(tm->tm_mon+1)) |
	      RTC_DATE_CFG_YEAR_L(bin2bcd((tm->tm_year+1900)%100)) |
	      RTC_DATE_CFG_YEAR_H(bin2bcd((tm->tm_year+1900)/100));
	hobot_rtc_wr(rtc, HOBOT_RTC_DATE_CFG_REG, val);

	/* Enable RTC */
	val = hobot_rtc_rd(rtc, HOBOT_RTC_CTRL_REG);
	val |= HOBOT_RTC_EN;
	hobot_rtc_wr(rtc, HOBOT_RTC_CTRL_REG, val);

	return 0;
}

static int hobot_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	u32 cur_time, cur_date;
	struct hobot_rtc *rtc = dev_get_drvdata(dev);

	cur_time = hobot_rtc_rd(rtc, HOBOT_RTC_CUR_TIME_REG);
	cur_date = hobot_rtc_rd(rtc, HOBOT_RTC_CUR_DATE_REG);

	tm->tm_sec  = bcd2bin(RTC_TIME_GET_SEC(cur_time));
	tm->tm_min  = bcd2bin(RTC_TIME_GET_MIN(cur_time));
	tm->tm_hour = bcd2bin(RTC_TIME_GET_HOUR(cur_time));
	tm->tm_wday = bcd2bin(RTC_TIME_GET_WEEK(cur_time))==7 ? 0:bcd2bin(RTC_TIME_GET_WEEK(cur_time));
	tm->tm_mday = bcd2bin(RTC_DATE_GET_DAY(cur_date));
	tm->tm_mon  = bcd2bin(RTC_DATE_GET_MON(cur_date))-1;
	tm->tm_year = bcd2bin(RTC_DATE_GET_YEAR_L(cur_date))+bcd2bin(RTC_DATE_GET_YEAR_H(cur_date))*100-1900;

	return rtc_valid_tm(tm);
}

static int hobot_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *wkalrm)
{
	u32 alarm_time, alarm_date;
	struct hobot_rtc *rtc = dev_get_drvdata(dev);
	struct rtc_time *tm = &wkalrm->time;

	alarm_time = hobot_rtc_rd(rtc, HOBOT_RTC_ALARM_TIME_REG);
	alarm_date = hobot_rtc_rd(rtc, HOBOT_RTC_ALARM_DATE_REG);
	tm->tm_sec  = bcd2bin(RTC_ALARM_GET_SEC(alarm_time));
	tm->tm_min  = bcd2bin(RTC_ALARM_GET_MIN(alarm_time));
	tm->tm_hour = bcd2bin(RTC_ALARM_GET_HOUR(alarm_time));
	tm->tm_wday = bcd2bin(RTC_ALARM_GET_WEEK(alarm_time))==7 ? 0:bcd2bin(RTC_ALARM_GET_WEEK(alarm_time));
	tm->tm_mday = bcd2bin(RTC_ALARM_GET_DAY(alarm_date));
	tm->tm_mon  = bcd2bin(RTC_ALARM_GET_MON(alarm_date))-1;
	tm->tm_year = bcd2bin(RTC_ALARM_GET_YEAR_L(alarm_date))+bcd2bin(RTC_ALARM_GET_YEAR_H(alarm_date))*100-1900;

	wkalrm->enabled = (hobot_rtc_rd(rtc, HOBOT_RTC_CTRL_REG) & HOBOT_RTC_AL_EN);
	wkalrm->pending = (hobot_rtc_rd(rtc, HOBOT_RTC_INT_STA_REG) & HOBOT_RTC_AL_INT_EN);

	return rtc_valid_tm(&wkalrm->time);
}

static int hobot_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *wkalrm)
{
	u32 val;
	struct hobot_rtc *rtc = dev_get_drvdata(dev);
	struct rtc_time *tm = &wkalrm->time;

	/* Disable alarm during update */
	val = hobot_rtc_rd(rtc, HOBOT_RTC_CTRL_REG);
	val &= (~HOBOT_RTC_AL_EN);
	hobot_rtc_wr(rtc, HOBOT_RTC_CTRL_REG, val);

	val = RTC_ALARM_CFG_SEC(bin2bcd(tm->tm_sec)) |
		  RTC_ALARM_CFG_MIN(bin2bcd(tm->tm_min)) |
		  RTC_ALARM_CFG_HOUR(bin2bcd(tm->tm_hour)) |
		  RTC_ALARM_CFG_WEEK(bin2bcd(tm->tm_wday==0 ? 7:tm->tm_wday));
	hobot_rtc_wr(rtc, HOBOT_RTC_ALARM_TIME_REG, val);

	val = RTC_ALARM_CFG_DAY(bin2bcd(tm->tm_mday)) |
          RTC_ALARM_CFG_MON(bin2bcd(tm->tm_mon+1)) |
          RTC_ALARM_CFG_YEAR_L(bin2bcd((tm->tm_year+1900)%100)) |
          RTC_ALARM_CFG_YEAR_H(bin2bcd((tm->tm_year+1900)/100));
	hobot_rtc_wr(rtc, HOBOT_RTC_ALARM_DATE_REG, val);

	if (wkalrm->enabled) {
		val = hobot_rtc_rd(rtc, HOBOT_RTC_CTRL_REG);
		val |= HOBOT_RTC_AL_SEC_EN  | HOBOT_RTC_AL_MIN_EN | HOBOT_RTC_AL_HOUR_EN |
		      HOBOT_RTC_AL_WEEK_EN | HOBOT_RTC_AL_DAY_EN | HOBOT_RTC_AL_MON_EN  |
		      HOBOT_RTC_AL_YEAR_EN | HOBOT_RTC_AL_EN;
		hobot_rtc_wr(rtc, HOBOT_RTC_CTRL_REG, val);

		val = hobot_rtc_rd(rtc, HOBOT_RTC_INT_UNMASK_REG);
		val |= HOBOT_RTC_AL_INT_EN;
		hobot_rtc_wr(rtc, HOBOT_RTC_INT_UNMASK_REG, val);
		val = hobot_rtc_rd(rtc, HOBOT_RTC_INT_SETMASK_REG);
		val &= (~HOBOT_RTC_AL_INT_EN);
		hobot_rtc_wr(rtc, HOBOT_RTC_INT_SETMASK_REG, val);
	}

	return 0;
}

static int hobot_rtc_alarm_irq_enable(struct device *dev, unsigned int enable)
{
	u32 val;
	struct hobot_rtc *rtc = dev_get_drvdata(dev);

	if (enable) {
		val = hobot_rtc_rd(rtc, HOBOT_RTC_INT_UNMASK_REG);
		val |= HOBOT_RTC_AL_INT_EN;
		hobot_rtc_wr(rtc, HOBOT_RTC_INT_UNMASK_REG, val);
		val = hobot_rtc_rd(rtc, HOBOT_RTC_INT_SETMASK_REG);
		val &= (~HOBOT_RTC_AL_INT_EN);
		hobot_rtc_wr(rtc, HOBOT_RTC_INT_SETMASK_REG, val);
	} else {
		val = hobot_rtc_rd(rtc, HOBOT_RTC_INT_UNMASK_REG);
		val &= (~HOBOT_RTC_AL_INT_EN);
		hobot_rtc_wr(rtc, HOBOT_RTC_INT_UNMASK_REG, val);
		val = hobot_rtc_rd(rtc, HOBOT_RTC_INT_SETMASK_REG);
		val |= HOBOT_RTC_AL_INT_EN;
		hobot_rtc_wr(rtc, HOBOT_RTC_INT_SETMASK_REG, val);
	}

	return 0;
}

static irqreturn_t hobot_rtc_int_handle(int irq, void *data)
{
	u32 status;
	unsigned long events = RTC_IRQF;
	struct hobot_rtc *rtc = (struct hobot_rtc *)data;

	/* Check interrupt cause */
	status = hobot_rtc_rd(rtc, HOBOT_RTC_INT_STA_REG);
	if (status & HOBOT_RTC_AL_INT_EN) {
		events |= RTC_AF;
	}
	if (status & HOBOT_RTC_TICK_INT_EN) {
		events |= RTC_PF;
	}

	/* Clear interrupt status and report event */
	hobot_rtc_wr(rtc, HOBOT_RTC_INT_STA_REG, status);
	rtc_update_irq(rtc->rtc, 1, events);

	return IRQ_HANDLED;
}

static const struct rtc_class_ops hobot_rtc_ops = {
	.read_time		  = hobot_rtc_read_time,
	.set_time		  = hobot_rtc_set_time,
	.read_alarm		  = hobot_rtc_read_alarm,
	.set_alarm		  = hobot_rtc_set_alarm,
	.alarm_irq_enable = hobot_rtc_alarm_irq_enable,
};

static ssize_t wake_unlock_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	if (!strncmp(buf, UNLOCK, strlen(UNLOCK)))
		pm_relax(dev);

	return len;
}
static ssize_t wake_active_time_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct hobot_rtc *rtc = dev_get_drvdata(dev);

	return snprintf(buf, DECIMAL_WIDTH + 1, "%d\n", rtc->keep_alive_time);
}

static ssize_t wake_active_time_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	int ret;
	unsigned int input;
	struct hobot_rtc *rtc = dev_get_drvdata(dev);

	ret = kstrtou32(buf, 10, &input);
	if (ret)
		return ret;

	if (input < 1000)
		return -EINVAL;

	rtc->keep_alive_time = input;

	return len;
}
static DEVICE_ATTR_WO(wake_unlock);
static DEVICE_ATTR_RW(wake_active_time);

static int hobot_rtc_probe(struct platform_device *pdev)
{
	int ret;
	struct hobot_rtc *rtc;
	struct resource *res;
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;

	rtc = devm_kzalloc(&pdev->dev, sizeof(*rtc), GFP_KERNEL);
	if (!rtc)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	rtc->rtc_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(rtc->rtc_base))
		return PTR_ERR(rtc->rtc_base);

	rtc->irq = platform_get_irq(pdev, 0);
	if (rtc->irq < 0) {
		dev_warn(&pdev->dev, "can't get interrupt resource\n");
		return rtc->irq;
	}

	platform_set_drvdata(pdev, rtc);

	/* Clear any pending interrupts */
	hobot_rtc_wr(rtc, HOBOT_RTC_INT_STA_REG, 0x3);
	hobot_rtc_wr(rtc, HOBOT_RTC_CTRL_REG, 0x0000);
	hobot_rtc_wr(rtc, HOBOT_RTC_DATE_CFG_REG, 0x19700101);
	hobot_rtc_wr(rtc, HOBOT_RTC_TIME_CFG_REG, 0x00000000);
	hobot_rtc_wr(rtc, HOBOT_RTC_CTRL_REG, 0x0200);

	ret = devm_request_irq(&pdev->dev, rtc->irq, hobot_rtc_int_handle, 0, pdev->name, rtc);
	if (ret < 0) {
		dev_warn(&pdev->dev, "can't request interrupt\n");
		return ret;
	}

	rtc->keep_alive_time = KEEP_ALIVE_TIME;
	rtc->wakeup = of_property_read_bool(node, "wakeup-source") ||
			of_property_read_bool(node, "linux,wakeup");
	device_init_wakeup(dev, rtc->wakeup);

	rtc->rtc = devm_rtc_device_register(&pdev->dev, "hobot-rtc", &hobot_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc->rtc)) {
		dev_err(&pdev->dev, "can't register rtc device\n");
		ret = PTR_ERR(rtc->rtc);
		return ret;
	}

	device_create_file(&pdev->dev, &dev_attr_wake_unlock);
	device_create_file(&pdev->dev, &dev_attr_wake_active_time);

	return 0;
}

static int hobot_rtc_remove(struct platform_device *pdev)
{
	struct hobot_rtc *rtc = platform_get_drvdata(pdev);

	free_irq(rtc->irq, rtc);
	/* Ensure all interrupt sources are masked */
	hobot_rtc_wr(rtc, HOBOT_RTC_CTRL_REG, 0x0);
	hobot_rtc_wr(rtc, HOBOT_RTC_INT_SETMASK_REG, 0x3);
	hobot_rtc_wr(rtc, HOBOT_RTC_INT_UNMASK_REG, 0X0);

	return 0;
}

#ifdef CONFIG_PM
int hobot_rtc_suspend(struct device *dev)
{
#if 0
	struct hobot_rtc *rtc = dev_get_drvdata(dev);

	pr_info("%s:%s, enter suspend...\n", __FILE__, __func__);

	rtc->rtc_regs[0] = hobot_rtc_rd(rtc, HOBOT_RTC_CTRL_REG);
	rtc->rtc_regs[1] = hobot_rtc_rd(rtc, HOBOT_RTC_ALARM_TIME_REG);
	rtc->rtc_regs[2] = hobot_rtc_rd(rtc, HOBOT_RTC_DATE_CFG_REG);
	rtc->rtc_regs[3] = hobot_rtc_rd(rtc, HOBOT_RTC_TIME_CFG_REG);

	/* Ensure all interrupt sources are masked */
	hobot_rtc_wr(rtc, HOBOT_RTC_CTRL_REG, 0x0);
	hobot_rtc_wr(rtc, HOBOT_RTC_INT_SETMASK_REG, 0x3);
	hobot_rtc_wr(rtc, HOBOT_RTC_INT_UNMASK_REG, 0X0);
#endif
	return 0;
}

int hobot_rtc_resume(struct device *dev)
{
	struct hobot_rtc *rtc = dev_get_drvdata(dev);
#if 0

	pr_info("%s:%s, enter resume...\n", __FILE__, __func__);

	/* Clear any pending interrupts */
	hobot_rtc_wr(rtc, HOBOT_RTC_INT_STA_REG, 0x3);
	hobot_rtc_wr(rtc, HOBOT_RTC_CTRL_REG, 0x0000);
	hobot_rtc_wr(rtc, HOBOT_RTC_DATE_CFG_REG, 0x19700101);
	hobot_rtc_wr(rtc, HOBOT_RTC_TIME_CFG_REG, 0x00000000);
	hobot_rtc_wr(rtc, HOBOT_RTC_CTRL_REG, 0x0200);

	hobot_rtc_wr(rtc, HOBOT_RTC_ALARM_TIME_REG, rtc->rtc_regs[1]);
	hobot_rtc_wr(rtc, HOBOT_RTC_DATE_CFG_REG, rtc->rtc_regs[2]);
	hobot_rtc_wr(rtc, HOBOT_RTC_TIME_CFG_REG, rtc->rtc_regs[3]);
	hobot_rtc_wr(rtc, HOBOT_RTC_CTRL_REG, rtc->rtc_regs[0]);
#endif
	pm_wakeup_dev_event(dev, rtc->keep_alive_time, true);
	return 0;
}
#endif

static const struct dev_pm_ops hobot_rtc_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(hobot_rtc_suspend,
			hobot_rtc_resume)
};

static const struct of_device_id hobot_rtc_match[] = {
	{ .compatible = "hobot,hobot-rtc" },
	{ }
};
MODULE_DEVICE_TABLE(of, hobot_rtc_match);

static struct platform_driver hobot_rtc_driver = {
	.probe	= hobot_rtc_probe,
	.remove	= hobot_rtc_remove,
	.driver	= {
		.name = "hobot-rtc",
		.of_match_table	= hobot_rtc_match,
		.pm = &hobot_rtc_dev_pm_ops,
	},
};
module_platform_driver(hobot_rtc_driver);

MODULE_AUTHOR("hobot, Inc.");
MODULE_DESCRIPTION("Hobot RTC driver");
MODULE_LICENSE("GPL v2");
