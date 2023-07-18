/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <logging.h>
#include <time.h>
#include <sys/time.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdbool.h>
#include <sys/ioctl.h>

#include "../utility/hb_pwm.h"

#define LPWM_NUM	 4
#define LPWM_PATH "/dev/lpwm_cdev"

struct lpwm_cdev_state {
	unsigned int lpwm_num;
	unsigned int period;
	unsigned int duty_cycle;
	bool enabled;
};

#define LPWM_CDEV_MAGIC 'L'
#define LPWM_CDEV_INIT	  _IOW(LPWM_CDEV_MAGIC, 0x12, unsigned int)
#define LPWM_CDEV_DEINIT	_IOW(LPWM_CDEV_MAGIC, 0x13, unsigned int)
#define LPWM_CONF_SINGLE	_IOW(LPWM_CDEV_MAGIC, 0x14, struct lpwm_cdev_state)
#define LPWM_ENABLE_SINGLE  _IOW(LPWM_CDEV_MAGIC, 0x15, unsigned int)
#define LPWM_DISABLE_SINGLE _IOW(LPWM_CDEV_MAGIC, 0x16, unsigned int)
#define LPWM_SWTRIG		 _IOW(LPWM_CDEV_MAGIC, 0x17, int)
#define LPWM_PPSTRIG		_IOW(LPWM_CDEV_MAGIC, 0x18, int)
#define LPWM_GET_STATE	  _IOWR(LPWM_CDEV_MAGIC, 0x19, struct lpwm_cdev_state)
#define LPWM_OFFSET_CONF	_IOW(LPWM_CDEV_MAGIC, 0x20, int)
#define LPWM_OFFSET_STATE   _IOR(LPWM_CDEV_MAGIC, 0x21, int)

static int pwm_num = LPWM_NUM;
static char *syspath = LPWM_PATH;

/*
 * export all usable pwm in sysfs
 * @pwmchip_path can be null or any path, e.g. /dev/lpwm_cdev
 */
int hb_pwm_init(const char *pwmchip_path)
{
	unsigned int i;
	int ret;
	int fd;

	fd = open(syspath, O_RDWR | O_CLOEXEC);
	if (fd < 0) {
		pr_err("Failed to open path %s\n", syspath);
		ret = -HB_PWM_ERR_FILE;
		goto out;
	}

	for (i = 0; i < pwm_num; i++) {
		ret = ioctl(fd, LPWM_CDEV_INIT, &i);
		if (ret < 0) {
			pr_err("Failed to get lpwm%u \n", i);
			ret = -HB_PWM_ERR_IO;
			goto out;
		}
	}

	ret = HB_PWM_OK;
	pr_info("%s: %d pwms usable\n", syspath, pwm_num);

out:
	if (fd >= 0)
		close(fd);

	return ret;
}

/*
 * disable and unexport all pwms
 */
int hb_pwm_deinit(void)
{
	unsigned int i;
	int ret;
	int fd;

	for (i = 0; i < pwm_num; i++) {
		ret = hb_pwm_disable_single(i);
		if (ret != HB_PWM_OK)
			return ret;
	}

	fd = open(syspath, O_RDWR | O_CLOEXEC);
	if (fd < 0) {
		pr_err("Failed to open path %s\n", syspath);
		ret = -HB_PWM_ERR_FILE;
		goto out;
	}

	for (i = 0; i < pwm_num; i++) {
		ret = ioctl(fd, LPWM_CDEV_DEINIT, &i);
		if (ret < 0) {
			pr_err("Failed to free lpwm%u \n", i);
			ret = -HB_PWM_ERR_IO;
			goto out;
		}
	}

	ret = HB_PWM_OK;
	pr_info("%s: %d pwms disabled\n", syspath, pwm_num);

out:
	if (fd >= 0)
		close(fd);

	return ret;
}

/*
 * config a pwm
 * @pwm: index of the pwm
 * @period_us, 10^9/period_ns is the frequency of PWM, range: [10us 40960us]
 * @duty_us, duty_cycle of pwm, aka high level time of in a pwm period, range [10us 160us]
 */

int hb_pwm_config_single(unsigned int pwm, int period_us, int duty_us)
{
	int ret;
	int fd;
	struct lpwm_cdev_state cdev_state;

	if (pwm >= pwm_num) {
		pr_err("lpwm %d is out of range\n", pwm);
		return -HB_PWM_ERR_PARAM;
	}

	if (period_us > 0 && (period_us < 10 || period_us > 40960)) {
		pr_err("lpwm only support period in [10us~40960us]\n");
		return -HB_PWM_ERR_PARAM;
	}

	if (duty_us > 0 && (duty_us < 10 || duty_us > 160)) {
		pr_err("lpwm only support duty_cycle in [10us~160us]\n");
		return -HB_PWM_ERR_PARAM;
	}

	cdev_state.lpwm_num = pwm;
	cdev_state.duty_cycle = duty_us*1000;
	cdev_state.period = period_us*1000;

	fd = open(syspath, O_RDWR | O_CLOEXEC);
	if (fd < 0) {
		pr_err("Failed to open path %s\n", syspath);
		ret = -HB_PWM_ERR_FILE;
		goto out;
	}

	ret = ioctl(fd, LPWM_CONF_SINGLE, &cdev_state);
	if (ret < 0) {
		pr_err("Failed to config lpwm%u \n", pwm);
		ret = -HB_PWM_ERR_IO;
		goto out;
	}

	ret = HB_PWM_OK;

out:
	if (fd >= 0)
		close(fd);

	return ret;
}

int hb_lpwm_config_offset(unsigned int *lpwm_offset, int num)
{
	int ret;
	int fd;

	if (num != pwm_num) {
		pr_err("lpwm offset should set togther\n");
		return -HB_PWM_ERR_PARAM;
	}

	fd = open(syspath, O_RDWR | O_CLOEXEC);
	if (fd < 0) {
		pr_err("Failed to open path %s\n", syspath);
		ret = -HB_PWM_ERR_FILE;
		goto out;
	}

	ret = ioctl(fd, LPWM_OFFSET_CONF, lpwm_offset);
	if (ret < 0) {
		pr_err("Failed to config lpwm offsets \n");
		ret = -HB_PWM_ERR_IO;
		goto out;
	}

	ret = HB_PWM_OK;

out:
	if (fd >= 0)
		close(fd);

	return ret;
}

/*
 * @enable, 1 to enable, 0 to disable
 */
static int hb_pwm_enable_disable_single(unsigned int pwm, int enable)
{
	int ret;
	int fd;
	unsigned int num;

	if (pwm >= pwm_num) {
		pr_err("lpwm %d is out of range\n", pwm);
		return -HB_PWM_ERR_PARAM;
	}

	num = pwm;

	fd = open(syspath, O_RDWR | O_CLOEXEC);
	if (fd < 0) {
		pr_err("Failed to open path %s\n", syspath);
		ret = -HB_PWM_ERR_FILE;
		goto out;
	}

	if (enable)
		ret = ioctl(fd, LPWM_ENABLE_SINGLE, &num);
	else
		ret = ioctl(fd, LPWM_DISABLE_SINGLE, &num);

	if (ret < 0) {
		pr_err("Failed to enable/disable lpwm%u \n", num);
		ret = -HB_PWM_ERR_IO;
		goto out;
	}

	ret = HB_PWM_OK;

out:
	if (fd >= 0)
		close(fd);

	return ret;
}

int hb_pwm_enable_single(unsigned int pwm)
{
	return hb_pwm_enable_disable_single(pwm, 1);
}

int hb_pwm_disable_single(unsigned int pwm)
{
	return hb_pwm_enable_disable_single(pwm, 0);
}

/*
 * lpwm signal won't run before sw/pps triggered
 */
int hb_lpwm_sw_trigger(void)
{
	int ret;
	int fd;
	int val = 1;

	fd = open(syspath, O_RDWR | O_CLOEXEC);
	if (fd < 0) {
		pr_err("Failed to open path %s\n", syspath);
		ret = -HB_PWM_ERR_FILE;
		goto out;
	}

	ret = ioctl(fd, LPWM_SWTRIG, &val);
	if (ret < 0) {
		pr_err("Failed to start swtrigger\n");
		ret = -HB_PWM_ERR_IO;
		goto out;
	}

	ret = HB_PWM_OK;

out:
	if (fd >= 0)
		close(fd);

	return ret;
}

/*
 * lpwm sw timer trigger
 */
void hb_lpwm_sw_timer_trigger(void)
{
	char cmd[128];

	snprintf(cmd, sizeof(cmd),
			"echo %d > /sys/module/pwm_hobot_lite/parameters/swtrig_period",
				1000);   /*1s timer*/
	system(cmd);
	return;
}

/*
 * lpwm signal won't run before sw/pps triggered
 */
int hb_lpwm_pps_trigger(void)
{
	int ret;
	int fd;
	int val = 1;

	fd = open(syspath, O_RDWR | O_CLOEXEC);
	if (fd < 0) {
		pr_err("Failed to open path %s\n", syspath);
		ret = -HB_PWM_ERR_FILE;
		goto out;
	}

	ret = ioctl(fd, LPWM_PPSTRIG, &val);
	if (ret < 0) {
		pr_err("Failed to start ppstrigger\n");
		ret = -HB_PWM_ERR_IO;
		goto out;
	}

	ret = HB_PWM_OK;

out:
	if (fd >= 0)
		close(fd);

	return ret;
}

/*
 * hb_lpwm_config configures all 4 lpwms
 * offset_us: should pass a offset[4] array, for offset of each lpwm channel
 * offset_us[i] is in [10us 40960us], step is 10us
 * period_us: period of lpwm signal, same for all lpwm channels
 * period_us is in [10us 40960us], step is 10us, e.g. 30fps is 33330
 * duty_us: high level time in each pwm period, same for all lpwm channels
 * duty_us is in [10us 160us], step is 10us
 */
int hb_lpwm_config(int num, int *offset_us, int *period_us, int *duty_us)
{
	int ret;
	int i;

	ret = hb_lpwm_config_offset((uint32_t *)offset_us, num);
	if (ret != HB_PWM_OK)
		return ret;

	ret = hb_pwm_init(syspath);
	if (ret != HB_PWM_OK)
		return ret;

	for (i = 0; i < num; i++) {
		ret = hb_pwm_config_single(i, period_us[i], duty_us[i]);
		if (ret != HB_PWM_OK)
			return ret;
	}

	return HB_PWM_OK;
}

int hb_lpwm_start(lpwm_info_t *lpwm_info)
{
	int i;
	int ret;

	for (i = 0; i < LPWM_NUM; i++) {
		if (lpwm_info->lpwm_enable & (1 << i)) {
			ret = hb_pwm_enable_disable_single(i, 1);
			if (ret != HB_PWM_OK) {
				pr_err("Failed to enable lpwm%d\n", i);
				return ret;
			}
		}
	}
	if (lpwm_info->ext_timer_en)
		hb_lpwm_sw_timer_trigger(); 	/* sw timer trigger */
	if (lpwm_info->ext_trig_en)
		ret = hb_lpwm_sw_trigger();		/* software trigger */
	else
		ret = hb_lpwm_pps_trigger();   /* pps trigger */
	return ret;
}

int hb_lpwm_stop(lpwm_info_t *lpwm_info)
{
	int i;
	int ret = 0;

	for (i = 0; i < LPWM_NUM; i++) {
		if (lpwm_info->lpwm_enable & (1 << i)) {
			ret = hb_pwm_enable_disable_single(i, 0);
			if (ret != HB_PWM_OK) {
				pr_err("Failed to enable lpwm%d\n", i);
				return ret;
			}
		}
	}
	return ret;
}

