/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef UTILITY_HB_PWM_H_
#define UTILITY_HB_PWM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "cam_common.h"

enum {
	HB_PWM_OK,
	HB_PWM_ERR_PARAM,
	HB_PWM_ERR_FILE,
	HB_PWM_ERR_IO,
	HB_PWM_ERR,
};

/* APIs to control signle pwm */
extern int hb_pwm_init(const char *pwmchip_path);
extern int hb_pwm_deinit(void);
extern int hb_pwm_config_single(unsigned int pwm, int period_us, int duty_us);
extern int hb_pwm_enable_single(unsigned pwm);
extern int hb_pwm_disable_single(unsigned pwm);
extern int hb_lpwm_sw_trigger(void);
extern int hb_lpwm_pps_trigger(void);

/*
 * APIs to control multiple lpwms
 * hb_lpwm_config configures all 4 lpwms
 * num should be 4 for lpwm
 * offset_us: should be offset_us[4] array, for offset of each lpwm channel
 * offset_us[i] is in [10us 40960us], step is 10us
 * period_us: should be period_us[4] array, for period of each lpwm channel
 * period_us[i] is in [10us 40960us], step is 10us, e.g. 30fps is 33330
 * duty_us: should be duty_us[4] array, duty_cycle of each lpwm channel
 * duty_us[i] is in [10us 160us], step is 10us
 */
extern
int hb_lpwm_config(int num, int *offset_us, int *period_us, int *duty_us);
/*
 * Use bit map to start/stop multile pwm
 * bit0-3 map to lpwm0-3, eg. 0x3: lpwm0,1, 0x5:lpwm0,2
 */
extern int hb_lpwm_start(lpwm_info_t *lpwm_info);
extern int hb_lpwm_stop(lpwm_info_t *lpwm_info);

#ifdef __cplusplus
}
#endif

#endif  // UTILITY_HB_PWM_H_
