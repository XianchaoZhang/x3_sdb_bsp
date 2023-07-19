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

#include <linux/seqlock.h>
#include "acamera_types.h"
#include "acamera_logger.h"
#include "acamera_sensor_api.h"
#include "acamera_command_api.h"

#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-async.h>

#include "soc_sensor.h"

extern void *acamera_camera_v4l2_get_subdev_by_name( const char *name );

typedef struct _sensor_context_t {
    sensor_param_t param;
    sensor_mode_t supported_modes[ISP_MAX_SENSOR_MODES];
    struct v4l2_subdev *soc_sensor;
    sensor_param_t tuning_param;//add ie&e
    seqlock_t m_lock;
} sensor_context_t;

static sensor_context_t s_ctx[FIRMWARE_CONTEXT_NUMBER];

static uint32_t get_ctx_num( void *ctx )
{
    uint32_t ret;
    for ( ret = 0; ret < FIRMWARE_CONTEXT_NUMBER; ret++ ) {
        if ( &s_ctx[ret] == ctx ) {
            break;
        }
    }
    return ret;
}


static void sensor_print_params( void *ctx )
{
    sensor_context_t *p_ctx = ctx;

    LOG( LOG_INFO, "SOC SENSOR PARAMETERS" );
    LOG( LOG_INFO, "pixels_per_line: %d", p_ctx->param.pixels_per_line );
    LOG( LOG_INFO, "again_log2_max: %d", p_ctx->param.again_log2_max );
    LOG( LOG_INFO, "dgain_log2_max: %d", p_ctx->param.dgain_log2_max );
    LOG( LOG_INFO, "again_accuracy: %d", p_ctx->param.again_accuracy );
    LOG( LOG_INFO, "integration_time_min: %d", p_ctx->param.integration_time_min );
    LOG( LOG_INFO, "integration_time_max: %d", p_ctx->param.integration_time_max );
    LOG( LOG_INFO, "integration_time_long_max: %d", p_ctx->param.integration_time_long_max );
    LOG( LOG_INFO, "integration_time_limit: %d", p_ctx->param.integration_time_limit );
    LOG( LOG_INFO, "day_light_integration_time_max: %d", p_ctx->param.day_light_integration_time_max );
    LOG( LOG_INFO, "integration_time_apply_delay: %d", p_ctx->param.integration_time_apply_delay );
    LOG( LOG_INFO, "isp_exposure_channel_delay: %d", p_ctx->param.isp_exposure_channel_delay );
    LOG( LOG_INFO, "xoffset: %d", p_ctx->param.xoffset );
    LOG( LOG_INFO, "yoffset: %d", p_ctx->param.yoffset );
    LOG( LOG_INFO, "lines_per_second: %d", p_ctx->param.lines_per_second );
    LOG( LOG_INFO, "sensor_exp_number: %d", p_ctx->param.sensor_exp_number );
    LOG( LOG_INFO, "modes_num: %d", p_ctx->param.modes_num );
    LOG( LOG_INFO, "mode: %d", p_ctx->param.mode );
    int32_t idx = 0;
    for ( idx = 0; idx < p_ctx->param.modes_num; idx++ ) {
        LOG( LOG_INFO, "preset %d", idx );
        LOG( LOG_INFO, "    mode:   %d", p_ctx->param.modes_table[idx].wdr_mode );
        LOG( LOG_INFO, "    width:  %d", p_ctx->param.modes_table[idx].resolution.width );
        LOG( LOG_INFO, "    height: %d", p_ctx->param.modes_table[idx].resolution.height );
        LOG( LOG_INFO, "    fps:    %d", p_ctx->param.modes_table[idx].fps );
        LOG( LOG_INFO, "    exp:    %d", p_ctx->param.modes_table[idx].exposures );
    }
}

//add ie&e
static void sensor_init_tuning_parameters(void *ctx)
{
	sensor_context_t *p_ctx = ctx;

        LOG(LOG_DEBUG, "init tuning parameters.");
	if(p_ctx != NULL) {
		memset(&p_ctx->tuning_param, 0, sizeof(sensor_param_t));
		p_ctx->tuning_param.again_log2_max = 0;
		p_ctx->tuning_param.dgain_log2_max = 0;
		p_ctx->tuning_param.integration_time_min = 0xffffffff;
		p_ctx->tuning_param.integration_time_max = 0;
		p_ctx->tuning_param.integration_time_long_max = 0;
		p_ctx->tuning_param.integration_time_limit = 0;
	}
}

//add ie&e
static void sensor_update_tuning_parameters(void *ctx, sensor_param_t *param)
{
	sensor_context_t *p_ctx = ctx;

	LOG(LOG_INFO, "update tuning parameters.");

	if((p_ctx != NULL) && (param != NULL)) {
		if(p_ctx->tuning_param.again_log2_max > 0) {
			param->again_log2_max =
				p_ctx->tuning_param.again_log2_max;
		}
		if(p_ctx->tuning_param.dgain_log2_max > 0) {
			param->dgain_log2_max =
				p_ctx->tuning_param.dgain_log2_max;
		}
		if(p_ctx->tuning_param.integration_time_max > 0) {
			param->integration_time_max =
				p_ctx->tuning_param.integration_time_max;
		}
		if(p_ctx->tuning_param.integration_time_long_max > 0) {
			param->integration_time_long_max =
				p_ctx->tuning_param.integration_time_long_max;
		}
		if(p_ctx->tuning_param.integration_time_min < 0xffffffe) {
			param->integration_time_min =
				p_ctx->tuning_param.integration_time_min;
		}
		if(p_ctx->tuning_param.integration_time_limit > 0) {
			param->integration_time_limit =
				p_ctx->tuning_param.integration_time_limit;
		}
	} else {
		LOG(LOG_ERR, "Sensor context pointer is NULL");
	}
}


static void sensor_update_parameters(void *ctx, sensor_param_t *param)
{
    sensor_context_t *p_ctx = ctx;
    int32_t rc = 0;
    //int32_t result = 0;
    if ((p_ctx != NULL) && (param != NULL)) {
        struct soc_sensor_ioctl_args settings = {0};
        struct v4l2_subdev *sd = p_ctx->soc_sensor;
        uint32_t ctx_num = get_ctx_num( ctx );
        if ( sd != NULL && ctx_num < FIRMWARE_CONTEXT_NUMBER ) {
            LOG( LOG_INFO, "Found context num:%d", ctx_num );
            settings.ctx_num = ctx_num;
            // Initial local parameters
            rc = v4l2_subdev_call( sd, core, ioctl, SOC_SENSOR_GET_EXP_NUMBER, &settings );
            param->sensor_exp_number = settings.args.general.val_out;

            rc = v4l2_subdev_call( sd, core, ioctl, SOC_SENSOR_GET_ANALOG_GAIN_MAX, &settings );
            param->again_log2_max = settings.args.general.val_out;

            rc = v4l2_subdev_call( sd, core, ioctl, SOC_SENSOR_GET_DIGITAL_GAIN_MAX, &settings );
            param->dgain_log2_max = settings.args.general.val_out;

            rc = v4l2_subdev_call( sd, core, ioctl, SOC_SENSOR_GET_UPDATE_LATENCY, &settings );
            param->integration_time_apply_delay = (uint8_t)settings.args.general.val_out;

            rc = v4l2_subdev_call( sd, core, ioctl, SOC_SENSOR_GET_INTEGRATION_TIME_MIN, &settings );
            param->integration_time_min = settings.args.general.val_out;

            rc = v4l2_subdev_call( sd, core, ioctl, SOC_SENSOR_GET_INTEGRATION_TIME_MAX, &settings );
            param->integration_time_max = settings.args.general.val_out;

            rc = v4l2_subdev_call( sd, core, ioctl, SOC_SENSOR_GET_INTEGRATION_TIME_LONG_MAX, &settings );
            param->integration_time_long_max = settings.args.general.val_out;

            rc = v4l2_subdev_call( sd, core, ioctl, SOC_SENSOR_GET_INTEGRATION_TIME_LIMIT, &settings );
            param->integration_time_limit = settings.args.general.val_out;

            rc = v4l2_subdev_call( sd, core, ioctl, SOC_SENSOR_GET_LINES_PER_SECOND, &settings );
            param->lines_per_second = settings.args.general.val_out;

            rc = v4l2_subdev_call( sd, core, ioctl, SOC_SENSOR_GET_ACTIVE_HEIGHT, &settings );
            param->active.height = (uint16_t)settings.args.general.val_out;

            rc = v4l2_subdev_call( sd, core, ioctl, SOC_SENSOR_GET_ACTIVE_WIDTH, &settings );
            param->active.width = (uint16_t)settings.args.general.val_out;

            param->isp_exposure_channel_delay = 0;

            rc = v4l2_subdev_call( sd, core, ioctl, SOC_SENSOR_GET_PRESET_NUM, &settings );
            param->modes_num = settings.args.general.val_out;

            rc = v4l2_subdev_call( sd, core, ioctl, SOC_SENSOR_GET_PRESET_CUR, &settings );
            param->mode = (uint8_t)settings.args.general.val_out;

            rc = v4l2_subdev_call( sd, core, ioctl, SOC_SENSOR_GET_PRESET_TYPE, &settings );
            param->sensor_type = (uint8_t)settings.args.general.val_out;

	    if ( param->modes_num > ISP_MAX_SENSOR_MODES ) {
                param->modes_num = ISP_MAX_SENSOR_MODES;
                LOG( LOG_WARNING, "Exceed maximum supported presets. Sensor driver returned %d but maximum is %d", param->modes_num, ISP_MAX_SENSOR_MODES );
            }


            param->modes_table = p_ctx->supported_modes;

            int32_t idx = 0;
            for ( idx = 0; idx < param->modes_num; idx++ ) {
                settings.args.general.val_in = idx;
                rc = v4l2_subdev_call( sd, core, ioctl, SOC_SENSOR_GET_PRESET_WIDTH, &settings );
                param->modes_table[idx].resolution.width = (uint16_t)settings.args.general.val_out;

                rc = v4l2_subdev_call( sd, core, ioctl, SOC_SENSOR_GET_PRESET_HEIGHT, &settings );
                param->modes_table[idx].resolution.height = (uint16_t)settings.args.general.val_out;

                rc = v4l2_subdev_call( sd, core, ioctl, SOC_SENSOR_GET_PRESET_FPS, &settings );
                param->modes_table[idx].fps = settings.args.general.val_out;

                rc = v4l2_subdev_call( sd, core, ioctl, SOC_SENSOR_GET_PRESET_EXP, &settings );
                param->modes_table[idx].exposures = (uint8_t)settings.args.general.val_out;

                rc = v4l2_subdev_call( sd, core, ioctl, SOC_SENSOR_GET_PRESET_MODE, &settings );
                param->modes_table[idx].wdr_mode = (uint8_t)settings.args.general.val_out;

                rc = v4l2_subdev_call( sd, core, ioctl, SOC_SENSOR_GET_SENSOR_BITS, &settings );
                param->modes_table[idx].bits = (uint8_t)settings.args.general.val_out;
            }

            param->sensor_ctx = p_ctx;

        } else {
            LOG( LOG_ERR, "SOC sensor subdev pointer is NULL" );
        }
        //if(SYSTEM_LOG_LEVEL==LOG_DEBUG) sensor_print_params( p_ctx );
        if(acamera_logger_get_level()==LOG_DEBUG) sensor_print_params( p_ctx );
    } else {
        LOG( LOG_ERR, "Sensor context pointer is NULL" );
    }
}


static void sensor_alloc_analog_gain( void *ctx, int32_t *gain, uint32_t gain_num )
{
    sensor_context_t *p_ctx = ctx;
    if ( p_ctx != NULL ) {
        struct soc_sensor_ioctl_args settings;
        struct v4l2_subdev *sd = p_ctx->soc_sensor;
        uint32_t ctx_num = get_ctx_num( ctx );
        if ( sd != NULL && ctx_num < FIRMWARE_CONTEXT_NUMBER ) {
            settings.ctx_num = ctx_num;
            settings.args.gain.gain_ptr = gain;
            settings.args.gain.gain_num = gain_num;

            int rc = v4l2_subdev_call( sd, core, ioctl, SOC_SENSOR_ALLOC_AGAIN, &settings );
            if ( rc == 0 ) {
                //result = settings.args.general.val_out;
            } else {
                LOG( LOG_ERR, "Failed to alloc again. rc = %d", rc );
            }
        } else {
            LOG( LOG_ERR, "SOC sensor subdev pointer is NULL" );
        }
    } else {
        LOG( LOG_ERR, "Sensor context pointer is NULL" );
    }
}


static void sensor_alloc_digital_gain( void *ctx, int32_t *gain, uint32_t gain_num)
{
    sensor_context_t *p_ctx = ctx;
    if ( p_ctx != NULL ) {
        struct soc_sensor_ioctl_args settings;
        struct v4l2_subdev *sd = p_ctx->soc_sensor;
        uint32_t ctx_num = get_ctx_num( ctx );
        if ( sd != NULL && ctx_num < FIRMWARE_CONTEXT_NUMBER ) {
            settings.ctx_num = ctx_num;
            settings.args.gain.gain_ptr = gain;
            settings.args.gain.gain_num = gain_num;

            int rc = v4l2_subdev_call( sd, core, ioctl, SOC_SENSOR_ALLOC_DGAIN, &settings );
            if ( rc == 0 ) {
                //result = settings.args.general.val_out;
            } else {
                LOG( LOG_ERR, "Failed to digital again. rc = %d", rc );
            }
        } else {
            LOG( LOG_ERR, "SOC sensor subdev pointer is NULL" );
        }
    } else {
        LOG( LOG_ERR, "Sensor context pointer is NULL" );
    }
}


static void sensor_alloc_integration_time( void *ctx, uint16_t *int_time, uint16_t *int_time_M, uint16_t *int_time_L )
{
    sensor_context_t *p_ctx = ctx;
    //int32_t result = 0;
    if ( p_ctx != NULL ) {
        struct soc_sensor_ioctl_args settings;
        struct v4l2_subdev *sd = p_ctx->soc_sensor;
        uint32_t ctx_num = get_ctx_num( ctx );
        if ( sd != NULL && ctx_num < FIRMWARE_CONTEXT_NUMBER ) {
            settings.ctx_num = ctx_num;
            settings.args.integration_time.it_short = *int_time;
            settings.args.integration_time.it_medium = *int_time_M;
            settings.args.integration_time.it_long = *int_time_L;
            int rc = v4l2_subdev_call( sd, core, ioctl, SOC_SENSOR_ALLOC_IT, &settings );
            if ( rc == 0 ) {
                *int_time = settings.args.integration_time.it_short;
                *int_time_M = settings.args.integration_time.it_medium;
                *int_time_L = settings.args.integration_time.it_long;
            } else {
                LOG( LOG_ERR, "Failed to alloc integration time. rc = %d", rc );
            }
        } else {
            LOG( LOG_ERR, "SOC sensor subdev pointer is NULL" );
        }
    } else {
        LOG( LOG_ERR, "Sensor context pointer is NULL" );
    }
    return;
}


static void sensor_update( void *ctx )
{
    sensor_context_t *p_ctx = ctx;
    if ( p_ctx != NULL ) {
        struct soc_sensor_ioctl_args settings;
        struct v4l2_subdev *sd = p_ctx->soc_sensor;
        uint32_t ctx_num = get_ctx_num( ctx );
        if ( sd != NULL && ctx_num < FIRMWARE_CONTEXT_NUMBER ) {
            settings.ctx_num = ctx_num;
            int rc = v4l2_subdev_call( sd, core, ioctl, SOC_SENSOR_UPDATE_EXP, &settings );
            if ( rc != 0 ) {
                LOG( LOG_ERR, "Failed to update sensor exposure. rc = %d", rc );
            }
        } else {
            LOG( LOG_ERR, "SOC sensor subdev pointer is NULL" );
        }
    } else {
        LOG( LOG_ERR, "Sensor context pointer is NULL" );
    }
}


static void sensor_awb_update( void *ctx, void *p_awb)
{
    sensor_context_t *p_ctx = ctx;
    sns_param_awb_cfg_t *awb = p_awb;
    if ( p_ctx != NULL ) {
        struct soc_sensor_ioctl_args settings;
        struct v4l2_subdev *sd = p_ctx->soc_sensor;
        uint32_t ctx_num = get_ctx_num( ctx );
        if ( sd != NULL && ctx_num < FIRMWARE_CONTEXT_NUMBER ) {
            settings.ctx_num = ctx_num;
            settings.args.general.val_in = awb->rgain;
            settings.args.general.val_in2 = awb->grgain;
            settings.args.general.val_in3 = awb->gbgain;
            settings.args.general.val_in4 = awb->bgain;
            int rc = v4l2_subdev_call( sd, core, ioctl, SOC_SENSOR_AWB_UPDATE, &settings );
            if ( rc != 0 ) {
                LOG( LOG_ERR, "Failed to update sensor awb. rc = %d", rc );
            }
        } else {
            LOG( LOG_ERR, "SOC sensor subdev pointer is NULL" );
        }
    } else {
        LOG( LOG_ERR, "Sensor context pointer is NULL" );
    }
}


static uint16_t sensor_get_id( void *ctx )
{
    return 0;
}


static void sensor_set_mode( void *ctx, uint8_t mode )
{
    LOG( LOG_NOTICE, "[KeyMsg] start set sensor to mode: '%d'.", mode );

    sensor_context_t *p_ctx = ctx;
    if ( p_ctx != NULL ) {
        struct soc_sensor_ioctl_args settings;
        struct v4l2_subdev *sd = p_ctx->soc_sensor;
        uint32_t ctx_num = get_ctx_num( ctx );
        if ( sd != NULL && ctx_num < FIRMWARE_CONTEXT_NUMBER ) {
            settings.ctx_num = ctx_num;
            settings.args.general.val_in = mode;
            int rc = v4l2_subdev_call( sd, core, ioctl, SOC_SENSOR_SET_PRESET, &settings );
            if ( rc == 0 ) {
                sensor_update_parameters(p_ctx, &p_ctx->param);
		write_seqlock(&p_ctx->m_lock);
		sensor_update_tuning_parameters(p_ctx, &p_ctx->param);//add ie&e
		write_sequnlock(&p_ctx->m_lock);
            } else {
                LOG( LOG_ERR, "Failed to set mode. rc = %d", rc );
            }
        } else {
            LOG( LOG_ERR, "SOC sensor subdev pointer is NULL" );
        }
    } else {
        LOG( LOG_ERR, "Sensor context pointer is NULL" );
    }

    LOG( LOG_NOTICE, "[KeyMsg] set sensor to mode: '%d' done.", mode );
}


static void sensor_set_type( void *ctx, uint8_t sensor_type, uint8_t sensor_i2c_channel )
{
    LOG( LOG_NOTICE, "[KeyMsg] start set sensor to type: '%d', i2c channel is %d .", sensor_type, sensor_i2c_channel );

    sensor_context_t *p_ctx = ctx;
    if ( p_ctx != NULL ) {
        struct soc_sensor_ioctl_args settings;
        struct v4l2_subdev *sd = p_ctx->soc_sensor;
        uint32_t ctx_num = get_ctx_num( ctx );
        if ( sd != NULL && ctx_num < FIRMWARE_CONTEXT_NUMBER ) {
            settings.ctx_num = ctx_num;
            settings.args.general.val_in = sensor_type;
            settings.args.general.val_in2 = sensor_i2c_channel;
            int rc = v4l2_subdev_call( sd, core, ioctl, SOC_SENSOR_SET_TYPE, &settings );
            if ( rc == 0 ) {
		sensor_init_tuning_parameters(p_ctx);
                sensor_update_parameters(p_ctx, &p_ctx->param);
		write_seqlock(&p_ctx->m_lock);
		sensor_update_tuning_parameters(p_ctx, &p_ctx->param);//add ie&e
		write_sequnlock(&p_ctx->m_lock);
            } else {
                LOG( LOG_ERR, "Failed to set sensor type. rc = %d", rc );
            }
        } else {
            LOG( LOG_ERR, "SOC sensor subdev pointer is NULL" );
        }
    } else {
        LOG( LOG_ERR, "Sensor context pointer is NULL" );
    }

    LOG( LOG_NOTICE, "[KeyMsg] start set sensor to type: '%d', i2c channel is %d .", sensor_type, sensor_i2c_channel );
}


static void sensor_get_parameters(void *ctx, sensor_param_t *param )
{
    sensor_context_t *p_ctx = ctx;
    uint32_t seq;

    if (param) {
	//sensor_init_tuning_parameters(p_ctx);
        sensor_update_parameters(p_ctx, param);
	do {
		seq = read_seqbegin(&p_ctx->m_lock);
		sensor_update_tuning_parameters(p_ctx, param);//add ie&e
	} while (read_seqretry(&p_ctx->m_lock, seq));
    }
}

//add ie&e
static void sensor_set_parameters(void *ctx, uint32_t cmd, uint32_t data)
{
	sensor_context_t *p_ctx = ctx;

	if(ctx != NULL) {
                sensor_update_parameters(p_ctx, &p_ctx->param);

		write_seqlock(&p_ctx->m_lock);
		switch (cmd) {
		case SET_AGAIN_MAX: {
			if(data > p_ctx->param.again_log2_max) {
				p_ctx->tuning_param.again_log2_max
					= p_ctx->param.again_log2_max;
			} else {
				p_ctx->tuning_param.again_log2_max = data;
			}
		}
		break;
		case SET_DGAIN_MAX: {
			if(data > p_ctx->param.dgain_log2_max) {
				p_ctx->tuning_param.dgain_log2_max
					= p_ctx->param.dgain_log2_max;
			} else {
				p_ctx->tuning_param.dgain_log2_max = data;
			}
		}
		break;
		case SET_INTER_MIN: {
			// integration_time_min is 1
			if((data < 1) || (data > p_ctx->param.integration_time_limit)) {
				LOG(LOG_ERR, "integration_time_limit is valid!");
				p_ctx->tuning_param.integration_time_min = 1;
			} else {
				p_ctx->tuning_param.integration_time_min = data;
			}
		}
		break;
		case SET_INTER_MAX: {
			if(data > p_ctx->param.integration_time_max) {
				p_ctx->tuning_param.integration_time_max
					= p_ctx->param.integration_time_max;
			} else {
				p_ctx->tuning_param.integration_time_max = data;
			}
		}
		break;
		case SET_INTER_LONG_MAX: {
			if(data > p_ctx->param.integration_time_long_max) {
				p_ctx->tuning_param.integration_time_long_max
					= p_ctx->param.integration_time_long_max;
			} else {
				p_ctx->tuning_param.integration_time_long_max = data;
			}
		}
		break;
		case SET_INTER_LIMIT: {
			if((data > p_ctx->param.integration_time_limit) || (data < p_ctx->param.integration_time_min)) {
				p_ctx->tuning_param.integration_time_limit
					= p_ctx->param.integration_time_limit;
			} else {
				p_ctx->tuning_param.integration_time_limit = data;
			}
		}
		break;
		default:
		break;
		}
		sensor_update_tuning_parameters(p_ctx, &p_ctx->param);//add ie&e
		write_sequnlock(&p_ctx->m_lock);
		LOG(LOG_INFO, " set sensor tuning param");
	} else {
		LOG(LOG_ERR, "Sensor context pointer is NULL");
	}
}

static void sensor_disable_isp( void *ctx )
{
}

/*
static void sensor_get_lines_per_second( void *ctx )
{
}
*/

static uint32_t read_register( void *ctx, uint32_t address )
{
    sensor_context_t *p_ctx = ctx;
    int32_t result = 0;
    if ( p_ctx != NULL ) {
        struct soc_sensor_ioctl_args settings;
        struct v4l2_subdev *sd = p_ctx->soc_sensor;
        uint32_t ctx_num = get_ctx_num( ctx );
        if ( sd != NULL && ctx_num < FIRMWARE_CONTEXT_NUMBER ) {
            settings.ctx_num = ctx_num;
            settings.args.general.val_in = address;
            int rc = v4l2_subdev_call( sd, core, ioctl, SOC_SENSOR_READ_REG, &settings );
            if ( rc == 0 ) {
                result = settings.args.general.val_out;
            } else {
                LOG( LOG_ERR, "Failed to read register. rc = %d", rc );
            }
        } else {
            LOG( LOG_ERR, "SOC sensor subdev pointer is NULL" );
        }
    } else {
        LOG( LOG_ERR, "Sensor context pointer is NULL" );
    }
    return result;
}


static void write_register( void *ctx, uint32_t address, uint32_t data )
{
    sensor_context_t *p_ctx = ctx;
    if ( p_ctx != NULL ) {
        struct soc_sensor_ioctl_args settings;
        struct v4l2_subdev *sd = p_ctx->soc_sensor;
        uint32_t ctx_num = get_ctx_num( ctx );
        if ( sd != NULL && ctx_num < FIRMWARE_CONTEXT_NUMBER ) {
            settings.ctx_num = ctx_num;
            settings.args.general.val_in = address;
            settings.args.general.val_in2 = data;
            int rc = v4l2_subdev_call( sd, core, ioctl, SOC_SENSOR_WRITE_REG, &settings );
            if ( rc == 0 ) {
            } else {
                LOG( LOG_ERR, "Failed to write register. rc = %d", rc );
            }
        } else {
            LOG( LOG_ERR, "SOC sensor subdev pointer is NULL" );
        }
    } else {
        LOG( LOG_ERR, "Sensor context pointer is NULL" );
    }
}


static void stop_streaming( void *ctx )
{
    sensor_context_t *p_ctx = ctx;
    if ( p_ctx != NULL ) {
        struct soc_sensor_ioctl_args settings;
        struct v4l2_subdev *sd = p_ctx->soc_sensor;
        uint32_t ctx_num = get_ctx_num( ctx );
        if ( sd != NULL && ctx_num < FIRMWARE_CONTEXT_NUMBER ) {
            settings.ctx_num = ctx_num;
            int rc = v4l2_subdev_call( sd, core, ioctl, SOC_SENSOR_STREAMING_OFF, &settings );
            if ( rc != 0 ) {
                LOG( LOG_ERR, "Failed to stop streaming. rc = %d", rc );
            }
        } else {
            LOG( LOG_ERR, "SOC sensor subdev pointer is NULL" );
        }
    } else {
        LOG( LOG_ERR, "Sensor context pointer is NULL" );
    }
}


static void start_streaming( void *ctx )
{
    sensor_context_t *p_ctx = ctx;
    if ( p_ctx != NULL ) {
        struct soc_sensor_ioctl_args settings;
        struct v4l2_subdev *sd = p_ctx->soc_sensor;
        uint32_t ctx_num = get_ctx_num( ctx );
        if ( sd != NULL && ctx_num < FIRMWARE_CONTEXT_NUMBER ) {
            settings.ctx_num = ctx_num;
            int rc = v4l2_subdev_call( sd, core, ioctl, SOC_SENSOR_STREAMING_ON, &settings );
            if ( rc != 0 ) {
                LOG( LOG_ERR, "Failed to start streaming. rc = %d", rc );
            }
        } else {
            LOG( LOG_ERR, "SOC sensor subdev pointer is NULL" );
        }
    } else {
        LOG( LOG_ERR, "Sensor context pointer is NULL" );
    }
}

void sensor_deinit_v4l2( void *ctx, uint8_t ctx_id )
{
    sensor_context_t *p_ctx = ctx;
    if ( p_ctx != NULL ) {
        if ( ctx_id >= FIRMWARE_CONTEXT_NUMBER ) {
            LOG( LOG_ERR, "ctx num %d is invalid.", ctx_id);
            return;
        }
        int rc = v4l2_subdev_call( p_ctx->soc_sensor, core, reset, ctx_id );
        if ( rc != 0 ) {
            LOG( LOG_ERR, "Failed to reset soc_sensor. core->reset failed with rc %d", rc );
            return;
        }
    } else {
        LOG( LOG_ERR, "Sensor context pointer is NULL" );
    }
}

//--------------------Initialization------------------------------------------------------------
void sensor_init_v4l2( void **ctx, sensor_control_t *ctrl, uint8_t ctx_id )
{
    //struct soc_sensor_ioctl_args settings;
    int rc = 0;

    LOG( LOG_NOTICE, "[KeyMsg] init sensor start" );

    // Local sensor data structure
    if ( ctx_id < FIRMWARE_CONTEXT_NUMBER ) {
        sensor_context_t *p_ctx = &s_ctx[ctx_id];

        ctrl->alloc_analog_gain = sensor_alloc_analog_gain;
        ctrl->alloc_digital_gain = sensor_alloc_digital_gain;
        ctrl->alloc_integration_time = sensor_alloc_integration_time;
        ctrl->sensor_update = sensor_update;
        ctrl->set_mode = sensor_set_mode;
	ctrl->sensor_awb_update = sensor_awb_update;
	ctrl->set_sensor_type = sensor_set_type;
        ctrl->get_id = sensor_get_id;
        ctrl->get_parameters = sensor_get_parameters;
        ctrl->set_parameters = sensor_set_parameters;
        ctrl->disable_sensor_isp = sensor_disable_isp;
        ctrl->read_sensor_register = read_register;
        ctrl->write_sensor_register = write_register;
        ctrl->start_streaming = start_streaming;
        ctrl->stop_streaming = stop_streaming;

        p_ctx->param.modes_table = p_ctx->supported_modes;
        p_ctx->param.modes_num = 0;

        *ctx = p_ctx;
	seqlock_init(&p_ctx->m_lock);
        p_ctx->soc_sensor = acamera_camera_v4l2_get_subdev_by_name( V4L2_SOC_SENSOR_NAME );
        if ( p_ctx->soc_sensor == NULL ) {
            LOG( LOG_CRIT, "Sensor bridge cannot be initialized before soc_sensor subdev is available. Pointer is null" );
            return;
        }

        rc = v4l2_subdev_call( p_ctx->soc_sensor, core, init, ctx_id );
        if ( rc != 0 ) {
            LOG( LOG_CRIT, "Failed to initialize soc_sensor. core->init failed with rc %d", rc );
            return;
        }

        sensor_update_parameters(p_ctx, &p_ctx->param);
	sensor_init_tuning_parameters(p_ctx);//add ie&e
    } else {
        LOG( LOG_ERR, "Attempt to initialize more sensor instances than was configured. Sensor initialization failed." );
        *ctx = NULL;
    }

    LOG( LOG_NOTICE, "[KeyMsg] init sensor done" );
}
