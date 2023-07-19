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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/extcon.h>
#include <soc/hobot/hobot_bus.h>

#include "core.h"

struct dwc3_powersave {
	struct device		*dev;
	struct device		*parent;	/* parent dwc3 device */
	struct device_node	*usb_wrap_node;
	struct device_node	*dwc3_node;

	/* notify device power management module event */
	struct hobot_dpm	dpm;
	/* notify extcon usb gpio event */
	struct notifier_block	edev_nb;
	struct extcon_dev	*edev;
	struct mutex		notifier_lock;

	u32			current_powersave_status;
	u32			last_powersave_status;
};

/*
 * Function: power_management_call
 * Description: callback function triggered by hobot bus system.
 * Some event info as below:
 * event:
 *	HB_BUS_SIGNAL_START >>> change system pll clock start
 *	HB_BUS_SIGNAL_END   >>> change system pll clock end
 *
 * state:
 *	POWERSAVE_STATE     >>> enter powersave state
 *	OTHER_STATE         >>> exit powersave state
 *
 * Just do correct things for above event!!
 */
static int power_management_call(struct hobot_dpm *self,
				unsigned long event, int state)
{
	struct dwc3_powersave *dwc_pwr = container_of(self,
			struct dwc3_powersave, dpm);
	struct device	*dev = dwc_pwr->dev;
	struct device	*usb_wrap_device = dwc_pwr->parent;

	if (!usb_wrap_device)
		return NOTIFY_BAD;

	if (event == HB_BUS_SIGNAL_START && state == POWERSAVE_STATE) {
		/* release driver when start enter to powersave state */
		mutex_lock(&dwc_pwr->notifier_lock);
		dwc_pwr->last_powersave_status = 1;
		if (dwc_pwr->current_powersave_status) {
			dev_info(dev, "%s: dwc3 already in powersave status\n", __func__);
			mutex_unlock(&dwc_pwr->notifier_lock);
		} else {
			/* before enter powersave state, release dwc3 driver */
			dev_dbg(dev, "%s: powersave state.\n", __func__);
			dev_dbg(dev, "%s: manually detach dwc3 driver.\n", __func__);
			device_release_driver(usb_wrap_device);

			dwc_pwr->current_powersave_status = 1;
			mutex_unlock(&dwc_pwr->notifier_lock);
		}
	} else if (event == HB_BUS_SIGNAL_END && state == OTHER_STATE) {
		/* attach driver after end of exit from powersave */
		mutex_lock(&dwc_pwr->notifier_lock);
		dwc_pwr->last_powersave_status = 0;
		if (!dwc_pwr->current_powersave_status) {
			dev_info(dev, "%s: dwc3 already in normal status\n",
					__func__);
			mutex_unlock(&dwc_pwr->notifier_lock);
		} else {
			/* before leave powersave state, attach device to dwc3 driver */
			dev_dbg(dev, "%s: otherstate state.\n", __func__);
			dev_dbg(dev, "%s: manually attach dwc3 driver.\n", __func__);
			if (device_attach(usb_wrap_device) < 0)
				dev_err(usb_wrap_device, "%s: attach device to driver failed\n", __func__);

			dwc_pwr->current_powersave_status = 0;
			mutex_unlock(&dwc_pwr->notifier_lock);
		}
	} else {
		dev_dbg(usb_wrap_device, "%s: No need to handle for event(%lu), state(%d)\n",
				__func__, event, state);
	}

	return 0;
}

static int extcon_usb_notifier(struct notifier_block *self,
		unsigned long action, void *data)
{
	struct dwc3_powersave *dwc_pwr = container_of(self,
			struct dwc3_powersave, edev_nb);
	struct device	*dev = dwc_pwr->dev;
	struct device	*usb_wrap_device = dwc_pwr->parent;

	dev_dbg(dev, "%s: action(%lu), last_powerstatus(%d)\n",
			__func__, action, dwc_pwr->last_powersave_status);

	if (action) {
		/* extcon otg usb device plug in */
		mutex_lock(&dwc_pwr->notifier_lock);
		if (!dwc_pwr->current_powersave_status) {
			dev_info(dev, "%s: current not powersave status. needn't attach dwc3 driver",
					__func__);
			mutex_unlock(&dwc_pwr->notifier_lock);
			return NOTIFY_DONE;
		}

		/* leave powersave state, attach device to dwc3 driver */
		dev_dbg(dev, "%s: manually attach dwc3 driver.\n", __func__);
		if (device_attach(usb_wrap_device) < 0)
			dev_err(usb_wrap_device, "%s: attach device to driver failed\n", __func__);

		dwc_pwr->current_powersave_status = 0;
		mutex_unlock(&dwc_pwr->notifier_lock);
	} else {
		/* extcon otg usb device plug out, restore previous
		 * powersave status if current not in powersave status */
		mutex_lock(&dwc_pwr->notifier_lock);
		if (dwc_pwr->last_powersave_status &&
				!dwc_pwr->current_powersave_status) {
			dev_info(dev, "%s: previous in powersave status, "
					"need to restore it\n", __func__);
			device_release_driver(usb_wrap_device);
			dwc_pwr->current_powersave_status = 1;
		}
		mutex_unlock(&dwc_pwr->notifier_lock);
	}

	return NOTIFY_DONE;
}

static const struct of_device_id of_dwc3_powersave_match[] = {
	{ .compatible = "hobot,dwc3-powersave", },
	{}
};

static int dwc3_powersave_probe(struct platform_device *pdev)
{
	struct device		*dev = &pdev->dev;
	struct dwc3_powersave	*dwc_pwr;
	struct platform_device	*usb_wrap_pdev = NULL;
	struct device_node	*usb_wrap_node = NULL;	/* parent, usbdrd3 node */
	struct platform_device	*dwc3_pdev = NULL;
	struct device_node	*dwc3_node = NULL;	/* parent, dwc3 device node */
	int	ret;

	dwc_pwr = devm_kzalloc(dev, sizeof(*dwc_pwr), GFP_KERNEL);
	if (!dwc_pwr)
		return -ENOMEM;

	/* get usb wrap node */
	usb_wrap_node = of_parse_phandle(dev->of_node, "usb", 0);
	if (!usb_wrap_node) {
		dev_err(dev, "%s of_parse_phandle usb failed\n", __func__);
		return -EPROBE_DEFER;
	}

	usb_wrap_pdev = of_find_device_by_node(usb_wrap_node);
	if (!usb_wrap_pdev) {
		dev_err(dev, "%s of_find_device_by_node usb_wrap_node failed\n",
				__func__);
		ret = -EPROBE_DEFER;
		goto find_usb_wrap_fail;
	}

	/* get dwc3 node */
	dwc3_node = of_get_child_by_name(usb_wrap_node, "dwc3");
	if (!dwc3_node) {
		dev_err(dev, "%s of_get_child_by_name \"dwc3\" failed\n",
				__func__);
		ret = -EPROBE_DEFER;
		goto find_usb_wrap_fail;
	}

	dwc3_pdev = of_find_device_by_node(dwc3_node);
	if (!dwc3_pdev) {
		dev_err(dev, "%s of_find_device_by_node dwc3_node failed\n",
				__func__);
		ret = -EPROBE_DEFER;
		goto find_dwc3_fail;
	}

	dwc_pwr->dev = dev;
	dwc_pwr->parent = &usb_wrap_pdev->dev;
	dwc_pwr->usb_wrap_node = usb_wrap_node;
	dwc_pwr->dwc3_node = dwc3_node;

	mutex_init(&dwc_pwr->notifier_lock);

	if (of_property_read_bool(dwc3_node, "extcon"))
		dwc_pwr->edev = extcon_get_edev_by_phandle(&dwc3_pdev->dev, 0);

	if (IS_ERR(dwc_pwr->edev)) {
		ret = PTR_ERR(dwc_pwr->edev);
		goto edev_err;
	}

	/* register extcon usb gpio notifier */
	dwc_pwr->edev_nb.notifier_call = extcon_usb_notifier;
	ret = extcon_register_notifier(dwc_pwr->edev, EXTCON_USB_HOST,
				       &dwc_pwr->edev_nb);
	if (ret < 0) {
		dev_err(dev, "couldn't register extcon usb notifier\n");
		goto edev_err;
	}

	/* register power management handler, and do correct things */
	dwc_pwr->dpm.dpm_call = power_management_call;
	hobot_dpm_register(&dwc_pwr->dpm, dev);

	platform_set_drvdata(pdev, dwc_pwr);

	return 0;

edev_err:
find_dwc3_fail:
	of_node_put(dwc3_node);

find_usb_wrap_fail:
	of_node_put(usb_wrap_node);

	return ret;
}

static int dwc3_powersave_remove(struct platform_device *pdev)
{
	struct dwc3_powersave	*dwc_pwr = platform_get_drvdata(pdev);

	/* unregister power management handler */
	hobot_dpm_unregister(&dwc_pwr->dpm);

	/* unregister extcon usb gpio notifier */
	extcon_unregister_notifier(dwc_pwr->edev, EXTCON_USB_HOST,
				   &dwc_pwr->edev_nb);

	of_node_put(dwc_pwr->dwc3_node);
	of_node_put(dwc_pwr->usb_wrap_node);

	return 0;
}

static struct platform_driver dwc3_powersave_driver = {
	.probe		= dwc3_powersave_probe,
	.remove		= dwc3_powersave_remove,
	.driver		= {
		.name	= "dwc3-powersave",
		.of_match_table	= of_match_ptr(of_dwc3_powersave_match),
	},
};

module_platform_driver(dwc3_powersave_driver);

MODULE_ALIAS("platform:dwc3-powersave");
MODULE_AUTHOR("Jianghe Xu<jianghe.xu@horizon.ai>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("dwc3 powersave driver");
