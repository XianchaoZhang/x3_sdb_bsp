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

#include "acamera_lens_api.h"
#include "system_sensor.h"
#include "acamera_logger.h"
#include "acamera_sbus_api.h"
#include "./af_driver/lens_api.h"

#if defined( CUR_MOD_NAME)
#undef CUR_MOD_NAME 
#define CUR_MOD_NAME LOG_MODULE_SOC_LENS
#else
#define CUR_MOD_NAME LOG_MODULE_SOC_LENS
#endif


typedef struct _lens_context_t {
    acamera_sbus_t p_bus;

    uint32_t port;
    uint16_t pos;
    uint16_t prev_pos;
    uint16_t move_pos;

    uint32_t time;

    lens_param_t param;
} lens_context_t;

static lens_context_t lens_ctx[FIRMWARE_CONTEXT_NUMBER];


uint8_t lens_null_test( uint32_t lens_bus )
{
    return 1;//TODO
}

extern void set_sensor_zoom_pos_info(uint32_t port, uint32_t zoom_pos);
static void zoom_null_drv_move(void *ctx, uint16_t position)
{
	int ret = 0;
	lens_context_t *p_ctx = (lens_context_t *)ctx;

	LOG(LOG_INFO, "IE&E %s, position %d ", __func__, position);
	set_sensor_zoom_pos_info(p_ctx->port, position);
	ret = lens_api_zoom_move((uint16_t)p_ctx->port, position);
}

extern void set_sensor_af_pos_info(uint32_t port, uint32_t af_pos);
static void vcm_null_drv_move( void *ctx, uint16_t position )
{
	int ret = 0;
	uint32_t pos = 0;
	lens_context_t *p_ctx = (lens_context_t *)ctx;

	LOG( LOG_INFO, "IE&E %s, position %d ", __func__, position );
	pos = position / 16;
	set_sensor_af_pos_info(p_ctx->port, pos);
	ret = lens_api_af_move((uint16_t)p_ctx->port, pos);
}

static uint8_t vcm_null_drv_is_moving( void *ctx )
{
	//lens_context_t *p_ctx = (lens_context_t *)ctx;
	LOG( LOG_INFO, "IE&E %s  ", __func__ );
	return 0;
}

static void vcm_null_write_register( void *ctx, uint32_t address, uint32_t data )
{
	lens_context_t *p_ctx = (lens_context_t *)ctx;
	lens_api_af_write_reg((uint16_t)p_ctx->port, address, data);
}

static uint32_t vcm_null_read_register( void *ctx, uint32_t address )
{
	uint32_t data = 0;

	lens_context_t *p_ctx = (lens_context_t *)ctx;
	lens_api_af_read_reg((uint16_t)p_ctx->port, address, &data);
	return data;
}

static const lens_param_t *lens_get_parameters( void *ctx )
{
	lens_context_t *p_ctx = ctx;
	return (const lens_param_t *)&p_ctx->param;
}

void lens_null_deinit( void *ctx )
{
}

void lens_null_init( void **ctx, lens_control_t *ctrl, uint32_t lens_bus )
{
    lens_context_t *p_ctx = &lens_ctx[lens_bus];
    lens_ctx[lens_bus].port = lens_bus;
    *ctx = p_ctx;

    ctrl->is_moving = vcm_null_drv_is_moving;
    ctrl->move = vcm_null_drv_move;
    ctrl->move_zoom = zoom_null_drv_move;
    ctrl->write_lens_register = vcm_null_write_register;
    ctrl->read_lens_register = vcm_null_read_register;
    ctrl->get_parameters = lens_get_parameters;

    p_ctx->prev_pos = 0;

    memset( &p_ctx->param, 0, sizeof( lens_param_t ) );
    p_ctx->param.min_step = 1 << 6;
    p_ctx->param.lens_type = LENS_VCM_DRIVER_COMMON;
}
