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

//-------------------------------------------------------------------------------------
//STRUCTURE:
//  VARIABLE SECTION:
//        CONTROLS - dependence from preprocessor
//        DATA     - modulation
//        RESET     - reset function
//        MIPI     - mipi settings
//        FLASH     - flash support
//  CONSTANT SECTION
//        DRIVER
//-------------------------------------------------------------------------------------

#define pr_fmt(fmt) "[isp_drv]: %s: " fmt, __func__

#include <linux/delay.h>
#include <linux/videodev2.h>
#include "acamera_types.h"
#include "acamera_logger.h"
#include "acamera_command_api.h"
#include "acamera_sbus_api.h"
#include "acamera_command_api.h"
#include "acamera_sensor_api.h"
#include "acamera_math.h"
#include "acamera_firmware_config.h"
#include "sensor_i2c.h"


#if defined( CUR_MOD_NAME)
#undef CUR_MOD_NAME 
#define CUR_MOD_NAME LOG_MODULE_SOC_SENSOR
#else
#define CUR_MOD_NAME LOG_MODULE_SOC_SENSOR
#endif

typedef enum {
	NORMAL_M = 1,
	DOL2_M,
	DOL3_M,
	DOL4_M,
	PWL_M,
} sensor_modes_e;

typedef struct _sensor_context_t {
    uint8_t address; // Sensor address for direct write (not used currently)
    uint8_t channel;
    acamera_sbus_t sbus;
    sensor_param_t param;
    sensor_mode_t supported_modes[ISP_MAX_SENSOR_MODES];
} sensor_context_t;

static sensor_context_t s_ctx[FIRMWARE_CONTEXT_NUMBER];
static struct sensor_operations *sensor_ops[FIRMWARE_CONTEXT_NUMBER];
struct sensor_priv_old sensor_data[FIRMWARE_CONTEXT_NUMBER];

//--------------------RESET------------------------------------------------------------
#if 0
static void sensor_hw_reset_enable( void )
{
//-- TODO
        LOG( LOG_DEBUG, "IE&E %s", __func__);
//	if (sensor_ops[p_ctx->channel])
//		sensor_ops[p_ctx->channel]->sensor_hw_reset_enable();
}

static void sensor_hw_reset_disable( void )
{
//-- TODO
        LOG( LOG_DEBUG, "IE&E %s", __func__);
//	if (sensor_ops[p_ctx->channel])
//		sensor_ops[p_ctx->channel]->sensor_hw_reset_disable();
}
#endif
//--------------------FLASH------------------------------------------------------------

static void sensor_alloc_analog_gain( void *ctx, int32_t *gain_ptr, uint32_t gain_num )
{
        uint32_t i = 0;
//-- TODO
	sensor_context_t *p_ctx = ctx;

        if (!gain_ptr) {
	        LOG(LOG_ERR, "param is null");
                return;
        }

        for (i = 0; (i < gain_num) || (i < 4); i++) {
	        gain_ptr[i] = gain_ptr[i] >> (LOG2_GAIN_SHIFT - 5);
        }
	if (sensor_ops[p_ctx->channel]) {
		if (sensor_ops[p_ctx->channel]->param_enable)
			sensor_ops[p_ctx->channel]->sensor_alloc_analog_gain(p_ctx->channel, gain_ptr, gain_num);
	}
	sensor_data[p_ctx->channel].analog_gain = gain_ptr[0];

        for (i = 0; (i < gain_num) || (i < 4); i++) {
	        gain_ptr[i] = gain_ptr[i] << (LOG2_GAIN_SHIFT - 5);
        }
}

static void sensor_alloc_digital_gain( void *ctx, int32_t *gain_ptr, uint32_t gain_num)
{
        uint32_t i = 0;
	sensor_context_t *p_ctx = ctx;
//get param
    	sensor_param_t *param = &p_ctx->param;
    	struct _setting_param_t sensor_param;
        if (!gain_ptr) {
	        LOG(LOG_ERR, "param is null");
                return;
        }

	LOG(LOG_DEBUG, "sensor_type %d", param->sensor_type);
	LOG(LOG_DEBUG, "lines_per_second %d", param->lines_per_second);
	LOG(LOG_DEBUG, "integration_time_min %d", param->integration_time_min);
	LOG(LOG_DEBUG, "intergration_time_max %d", param->integration_time_max);
	LOG(LOG_DEBUG, "integration_time_limit %d", param->integration_time_limit);
	LOG(LOG_DEBUG, "integration_time_long_max %d", param->integration_time_long_max);
	LOG(LOG_DEBUG, "dgain_log2_max %d", param->dgain_log2_max);
	LOG(LOG_DEBUG, "again_log2_max %d", param->again_log2_max);

	if (sensor_ops[p_ctx->channel] != NULL) {
		if (sensor_ops[p_ctx->channel]->param_enable == 0) {
			sensor_ops[p_ctx->channel]->sesor_get_para(p_ctx->channel, &sensor_param);
			if (sensor_param.lines_per_second && sensor_param.exposure_time_max) {
				sensor_ops[p_ctx->channel]->param_enable = 1;
				param->lines_per_second = sensor_param.lines_per_second;
				param->integration_time_min = sensor_param.exposure_time_min;
				param->integration_time_max = sensor_param.exposure_time_max;
				param->integration_time_limit = sensor_param.exposure_time_max;
				param->integration_time_long_max = sensor_param.exposure_time_long_max;
				param->dgain_log2_max = sensor_param.digital_gain_max;
				param->again_log2_max = sensor_param.analog_gain_max;
			}
		}
	}
//get param end
        for (i = 0; (i < gain_num) || (i < 4); i++) {
	        gain_ptr[i] = gain_ptr[i] >> (LOG2_GAIN_SHIFT - 5);
        }
	if (sensor_ops[p_ctx->channel]) {
		if (sensor_ops[p_ctx->channel]->param_enable)
			sensor_ops[p_ctx->channel]->sensor_alloc_digital_gain(p_ctx->channel, gain_ptr, gain_num);
	}
	sensor_data[p_ctx->channel].digital_gain = gain_ptr[0];

        for (i = 0; (i < gain_num) || (i < 4); i++) {
	        gain_ptr[i] = gain_ptr[i] << (LOG2_GAIN_SHIFT - 5);
        }
}

static void sensor_alloc_integration_time( void *ctx, uint16_t *int_time, uint16_t *int_time_M, uint16_t *int_time_L )
{
//-- TODO
	sensor_context_t *p_ctx = ctx;

	LOG(LOG_INFO, "init s_t %d, M_t %d, L_t %d", *int_time, *int_time_M, *int_time_L);
	if (sensor_ops[p_ctx->channel]) {
		if (sensor_ops[p_ctx->channel]->param_enable)
			sensor_ops[p_ctx->channel]->sensor_alloc_integration_time(p_ctx->channel, int_time, int_time_M, int_time_L);
	}
	sensor_data[p_ctx->channel].int_time = *int_time;
	sensor_data[p_ctx->channel].int_time_M = *int_time_M;
	sensor_data[p_ctx->channel].int_time_L = *int_time_L;

	LOG(LOG_INFO, "end s_t %d, M_t %d, L_t %d", *int_time, *int_time_M, *int_time_L);
}

static void sensor_update( void *ctx )
{
//-- TODO 
	sensor_context_t *p_ctx = ctx;
        
//	 int32_t analog_gain;
//       int32_t digital_gain;
//        uint16_t int_time;
//        uint16_t int_time_M;
//        uint16_t int_time_L; 
#if 1
	LOG(LOG_INFO, "analog_gain %d, digital_gain %d, int_time %d ", sensor_data[p_ctx->channel].analog_gain, sensor_data[p_ctx->channel].digital_gain, sensor_data[p_ctx->channel].int_time);
	if (sensor_ops[p_ctx->channel]) {
		if (sensor_ops[p_ctx->channel]->param_enable)
			sensor_ops[p_ctx->channel]->sensor_update(p_ctx->channel, sensor_data[p_ctx->channel]);
	}
#endif
}

static void sensor_set_type( void *ctx, uint8_t sensor_type, uint8_t sensor_i2c_channel )
{
    sensor_context_t *p_ctx = ctx;
    sensor_param_t *param = &p_ctx->param;
    struct _setting_param_t sensor_param;
    uint32_t tmp = 0;

    pr_debug("sensor type %d, exp %d\n", sensor_type, param->sensor_exp_number);
    if ((sensor_ops[p_ctx->channel] == NULL) || (param->sensor_type != sensor_type) ||
		(param->sensor_i2c_channel != sensor_i2c_channel)) {
    	param->sensor_type = sensor_type;
    	param->sensor_i2c_channel = sensor_i2c_channel;
		sensor_ops[p_ctx->channel] = sensor_chn_open(p_ctx->channel, sensor_i2c_channel, sensor_type);
	if (sensor_ops[p_ctx->channel]) {
		if (param->modes_table[param->mode].wdr_mode == WDR_MODE_LINEAR) {
			//normal
			sensor_ops[p_ctx->channel]->sensor_init(p_ctx->channel, NORMAL_M);
		} else if (param->modes_table[param->mode].wdr_mode == WDR_MODE_FS_LIN) {
			if (param->sensor_exp_number == 2) {
				sensor_ops[p_ctx->channel]->sensor_init(p_ctx->channel, DOL2_M);
			} else if (param->sensor_exp_number == 3) {
				sensor_ops[p_ctx->channel]->sensor_init(p_ctx->channel, DOL3_M);
			}
		} else if (param->modes_table[param->mode].wdr_mode == WDR_MODE_NATIVE) {
			//pwl
			sensor_ops[p_ctx->channel]->sensor_init(p_ctx->channel, PWL_M);
		}
		tmp = 0;
		while (tmp < 5) {
			tmp++;
			sensor_ops[p_ctx->channel]->sesor_get_para(p_ctx->channel, &sensor_param);
			if (sensor_param.lines_per_second && sensor_param.exposure_time_max) {
				sensor_ops[p_ctx->channel]->param_enable = 1;
				param->lines_per_second = sensor_param.lines_per_second;
				param->integration_time_min = sensor_param.exposure_time_min;
				param->integration_time_max = sensor_param.exposure_time_max;
				param->integration_time_limit = sensor_param.exposure_time_max;
				param->integration_time_long_max = sensor_param.exposure_time_long_max;
				param->dgain_log2_max = sensor_param.digital_gain_max;
				param->again_log2_max = sensor_param.analog_gain_max;
				break;
			} else {
				sensor_ops[p_ctx->channel]->param_enable = 0;
				LOG( LOG_INFO, "num [%d]: get param failed!", tmp);
				usleep_range( 5000, 5000 + 100 ); // reset at least 1 ms
			}
		}
    	}
    }
}

static void sensor_set_mode(void *ctx, uint8_t mode)
{
    sensor_context_t *p_ctx = ctx;
    sensor_param_t *param = &p_ctx->param;
    //struct _setting_param_t sensor_param;

    if (mode >= array_size(p_ctx->supported_modes))
        mode = 0;

    p_ctx->param.sensor_exp_number = 1;
    p_ctx->param.again_log2_max = 2088960;//0 255*2^13 = 255*8192
    p_ctx->param.dgain_log2_max = 2088960;//0
    p_ctx->param.integration_time_apply_delay = 2;
    p_ctx->param.isp_exposure_channel_delay = 0;

    param->active.width = p_ctx->supported_modes[mode].resolution.width;
    param->active.height = p_ctx->supported_modes[mode].resolution.height;
    param->total.width = p_ctx->supported_modes[mode].resolution.width;
    param->total.height = p_ctx->supported_modes[mode].resolution.height;

    param->pixels_per_line = param->total.width;
    param->integration_time_min = 5;
    param->integration_time_max = 90000;
    param->integration_time_long_max = 90000 * 3;
    param->integration_time_limit = 90000;
    param->lines_per_second = 10000;
    param->sensor_exp_number = param->modes_table[mode].exposures;
    //sensor param init
    p_ctx->supported_modes[param->mode].fps = 2560;
    param->mode = mode;

	pr_debug("mode %d, w %d, h %d, exp %d\n",
		mode, param->active.width, param->active.height, param->sensor_exp_number);
}

static uint16_t sensor_get_id( void *ctx )
{
    return 0xFFFF;
}

static void sensor_get_parameters(void *ctx, sensor_param_t *param)
{
	sensor_context_t *p_ctx = ctx;

	sensor_param_t *info = &p_ctx->param;
	struct _setting_param_t sensor_param;

	if ((sensor_ops[p_ctx->channel] != NULL) && (info)) {
		sensor_ops[p_ctx->channel]->sesor_get_para(p_ctx->channel, &sensor_param);
		if (sensor_param.lines_per_second && sensor_param.exposure_time_max) {
			sensor_ops[p_ctx->channel]->param_enable = 1;
			info->lines_per_second = sensor_param.lines_per_second;
			info->integration_time_min = sensor_param.exposure_time_min;
			info->integration_time_max = sensor_param.exposure_time_max;
			info->integration_time_limit = sensor_param.exposure_time_max;
			info->integration_time_long_max = sensor_param.exposure_time_long_max;
			info->dgain_log2_max = sensor_param.digital_gain_max;
			info->again_log2_max = sensor_param.analog_gain_max;
			if (sensor_param.fps) {
				p_ctx->supported_modes[info->mode].fps = sensor_param.fps;
			} else {
				p_ctx->supported_modes[info->mode].fps = 2560;
			}
		} else {
			sensor_ops[p_ctx->channel]->param_enable = 0;
		}
	}

	if (info)
		memcpy(param, info, sizeof(sensor_param_t));
}

static void sensor_disable_isp( void *ctx )
{
//-- TODO
	sensor_context_t *p_ctx = ctx;

        LOG( LOG_INFO, "IE&E %s", __func__);
	if (sensor_ops[p_ctx->channel])
		sensor_ops[p_ctx->channel]->sensor_disable_isp(p_ctx->channel);
}

static uint32_t read_register( void *ctx, uint32_t address )
{
//-- TODO
	uint32_t data = 0;
	sensor_context_t *p_ctx = ctx;

        LOG( LOG_INFO, "IE&E %s", __func__);
	if (sensor_ops[p_ctx->channel])
		data = sensor_ops[p_ctx->channel]->read_register(p_ctx->channel, address);
	return data;
}

static void write_register( void *ctx, uint32_t address, uint32_t data )
{
//-- TODO
	sensor_context_t *p_ctx = ctx;

        LOG( LOG_INFO, "IE&E %s", __func__);
	if (sensor_ops[p_ctx->channel])
		sensor_ops[p_ctx->channel]->write_register(p_ctx->channel, address, data);
}

static void stop_streaming( void *ctx )
{
//-- TODO	
	sensor_context_t *p_ctx = ctx;

        LOG( LOG_INFO, "IE&E %s", __func__);
	if (sensor_ops[p_ctx->channel])
		sensor_ops[p_ctx->channel]->stop_streaming(p_ctx->channel);
}

static void start_streaming( void *ctx )
{
//-- TODO
	sensor_context_t *p_ctx = ctx;

        LOG( LOG_INFO, "IE&E %s", __func__);
	if (sensor_ops[p_ctx->channel])
		sensor_ops[p_ctx->channel]->start_streaming(p_ctx->channel);
}

static void sensor_awb_update( void *ctx, void *p_awb )
{
	sns_param_awb_cfg_t *awb = p_awb;
	sensor_context_t *p_ctx = ctx;
        LOG( LOG_INFO, "IE&E %s", __func__);

	pr_debug("r %d, gr %d, gb %d, b %d\n", awb->rgain, awb->grgain, awb->gbgain, awb->bgain);
	if (sensor_ops[p_ctx->channel])
		sensor_ops[p_ctx->channel]->sensor_awb_update(p_ctx->channel, awb->rgain, awb->grgain, awb->gbgain, awb->bgain);
}

void sensor_deinit_dummy(uint32_t ctx_id, void *ctx )
{
}

int sensor_info_check_valid(uint32_t ctx_id, struct v4l2_format *f)
{
	uint8_t exposures;
    uint8_t bits;

	exposures = f->fmt.pix_mp.reserved[0];
	bits = f->fmt.pix_mp.reserved[1];

	if (exposures != NORMAL_M && exposures != DOL2_M &&
		exposures != DOL3_M && exposures != DOL4_M && exposures != PWL_M)
		return 0;

	if (bits != 8 && bits != 10 && bits != 12 && bits != 14 && bits != 16 && bits != 20)
		return 0;

	return 1;
}
EXPORT_SYMBOL(sensor_info_check_valid);

int sensor_info_check_exist(uint32_t ctx_id, struct v4l2_format *f)
{
	int i;
	uint16_t w, h;
	uint8_t exposures;
	uint8_t bits;
	sensor_context_t *p_ctx = &s_ctx[ctx_id];

	for (i = 0; i < p_ctx->param.modes_num; i++) {
		w = p_ctx->supported_modes[i].resolution.width;
		h = p_ctx->supported_modes[i].resolution.height;
		bits = p_ctx->supported_modes[i].bits;
		exposures = p_ctx->supported_modes[i].exposures;
		pr_debug("[%d/%d] w %d, h %d, bits %d, exposures %d\n", i+1, p_ctx->param.modes_num, w, h, bits, exposures);
	}

	for (i = 0; i < p_ctx->param.modes_num; i++) {
		w = p_ctx->supported_modes[i].resolution.width;
		h = p_ctx->supported_modes[i].resolution.height;
		bits = p_ctx->supported_modes[i].bits;
		exposures = p_ctx->supported_modes[i].exposures;
		if (w == f->fmt.pix_mp.width && h == f->fmt.pix_mp.height
			&& exposures == f->fmt.pix_mp.reserved[0] && bits == f->fmt.pix_mp.reserved[1])
			return 1;
	}

	return 0;
}
EXPORT_SYMBOL(sensor_info_check_exist);

void sensor_info_fill(uint32_t ctx_id, struct v4l2_format *f)
{
	int i;
	uint8_t mode = 0;
	sensor_context_t *p_ctx = &s_ctx[ctx_id];

	if (f->fmt.pix_mp.reserved[0] == NORMAL_M)
		mode = WDR_MODE_LINEAR;
	else if (DOL2_M <= f->fmt.pix_mp.reserved[0] && f->fmt.pix_mp.reserved[0] <= DOL4_M)
		mode = WDR_MODE_FS_LIN;
	else if (f->fmt.pix_mp.reserved[0] == PWL_M)
		mode = WDR_MODE_NATIVE;

	i = p_ctx->param.modes_num;
	p_ctx->supported_modes[i].wdr_mode = mode;
	p_ctx->supported_modes[i].fps = 30 * 256;
	p_ctx->supported_modes[i].resolution.width = (uint16_t)f->fmt.pix_mp.width;
	p_ctx->supported_modes[i].resolution.height = (uint16_t)f->fmt.pix_mp.height;
	p_ctx->supported_modes[i].exposures = f->fmt.pix_mp.reserved[0];
	p_ctx->supported_modes[i].bits = f->fmt.pix_mp.reserved[1];	
	p_ctx->param.modes_num++;
}
EXPORT_SYMBOL(sensor_info_fill);

int sensor_info_get_idx(uint32_t ctx_id, struct v4l2_format *f)
{
	int i;
	uint16_t w, h;
    uint8_t exposures;
    uint8_t bits;
	sensor_context_t *p_ctx = &s_ctx[ctx_id];

	for (i = 0; i < p_ctx->param.modes_num; i++) {
		w = p_ctx->supported_modes[i].resolution.width;
		h = p_ctx->supported_modes[i].resolution.height;
		bits = p_ctx->supported_modes[i].bits;
		exposures = p_ctx->supported_modes[i].exposures;
		if (w == f->fmt.pix_mp.width && h == f->fmt.pix_mp.height
			&& exposures == f->fmt.pix_mp.reserved[0] && bits == f->fmt.pix_mp.reserved[1])
			return i;
	}

	return -1;
}
EXPORT_SYMBOL(sensor_info_get_idx);

//--------------------Initialization------------------------------------------------------------
void sensor_init_dummy(uint32_t ctx_id, void **ctx, sensor_control_t *ctrl )
{
    if ( ctx_id < FIRMWARE_CONTEXT_NUMBER ) {
        sensor_context_t *p_ctx = &s_ctx[ctx_id];
		sensor_ops[ctx_id] = NULL;

		p_ctx->channel = (uint8_t)ctx_id;
        p_ctx->param.sensor_exp_number = 1;
        p_ctx->param.again_log2_max = 2088960;//0 255*2^13 = 255*8196
        p_ctx->param.dgain_log2_max = 2088960;//0
        p_ctx->param.integration_time_apply_delay = 2;
        p_ctx->param.isp_exposure_channel_delay = 0;
		p_ctx->param.lines_per_second = 10000;
        p_ctx->param.modes_table = p_ctx->supported_modes;
        p_ctx->param.modes_num = 1;

        p_ctx->supported_modes[0].wdr_mode = WDR_MODE_LINEAR;
        p_ctx->supported_modes[0].fps = 30 * 256;
        p_ctx->supported_modes[0].resolution.width = 1920;
        p_ctx->supported_modes[0].resolution.height = 1080;
        p_ctx->supported_modes[0].bits = 12;
        p_ctx->supported_modes[0].exposures = 1;

        p_ctx->param.sensor_ctx = &s_ctx;
		p_ctx->param.sensor_type = SENSOR_NULL;
        *ctx = p_ctx;

        ctrl->alloc_analog_gain = sensor_alloc_analog_gain;
        ctrl->alloc_digital_gain = sensor_alloc_digital_gain;
        ctrl->alloc_integration_time = sensor_alloc_integration_time;
        ctrl->sensor_update = sensor_update;
        ctrl->set_mode = sensor_set_mode;
		ctrl->set_sensor_type = sensor_set_type;
        ctrl->get_id = sensor_get_id;
        ctrl->get_parameters = sensor_get_parameters;
        ctrl->disable_sensor_isp = sensor_disable_isp;
        ctrl->read_sensor_register = read_register;
        ctrl->write_sensor_register = write_register;
        ctrl->start_streaming = start_streaming;
        ctrl->stop_streaming = stop_streaming;
        ctrl->sensor_awb_update = sensor_awb_update;
    } else {
        LOG( LOG_ERR, "Attempt to initialize more sensor instances than was configured. Sensor initialization failed." );
        *ctx = NULL;
    }
}

//*************************************************************************************
