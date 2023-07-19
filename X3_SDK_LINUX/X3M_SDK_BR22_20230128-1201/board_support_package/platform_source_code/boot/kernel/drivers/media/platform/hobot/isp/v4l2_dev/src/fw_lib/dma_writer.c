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
#define pr_fmt(fmt) "[isp_drv]: %s: " fmt, __func__
#include <linux/ion.h>
#include "acamera_logger.h"
#include "system_interrupts.h"
#include "system_stdlib.h"
#include "dma_writer_api.h"
#include "dma_writer.h"
#include "acamera_firmware_config.h"
#include "acamera_isp_config.h"
#include "acamera.h"
#include "acamera_fw.h"
#include "dma_writer_fsm.h"


#if defined( CUR_MOD_NAME)
#undef CUR_MOD_NAME 
#define CUR_MOD_NAME LOG_MODULE_DMA_WRITER
#else
#define CUR_MOD_NAME LOG_MODULE_DMA_WRITER
#endif



extern uint32_t acamera_get_api_context( void );
extern void acamera_dma_wr_check(void);

#define TAG "DMA_WRITER"

//#define DMA_printk(a) printk a
#define DMA_printk( a ) (void)0

#define DEBUG_FRAME2USR(fmt,...) //printk

static void dma_writer_initialize_reg_ops( dma_pipe *pipe )
{
    dma_writer_reg_ops_t *primary_ops = &pipe->primary;
    dma_writer_reg_ops_t *secondary_ops = &pipe->secondary;

    primary_ops->format_read = pipe->api[0].p_acamera_isp_dma_writer_format_read;
    primary_ops->format_write = pipe->api[0].p_acamera_isp_dma_writer_format_write;
    primary_ops->bank0_base_write = pipe->api[0].p_acamera_isp_dma_writer_bank0_base_write;
    primary_ops->line_offset_write = pipe->api[0].p_acamera_isp_dma_writer_line_offset_write;
    primary_ops->write_on_write = pipe->api[0].p_acamera_isp_dma_writer_frame_write_on_write;
    primary_ops->active_width_write = pipe->api[0].p_acamera_isp_dma_writer_active_width_write;
    primary_ops->active_height_write = pipe->api[0].p_acamera_isp_dma_writer_active_height_write;
    primary_ops->active_width_read = pipe->api[0].p_acamera_isp_dma_writer_active_width_read;
    primary_ops->active_height_read = pipe->api[0].p_acamera_isp_dma_writer_active_height_read;

    secondary_ops->format_read = pipe->api[0].p_acamera_isp_dma_writer_format_read_uv;
    secondary_ops->format_write = pipe->api[0].p_acamera_isp_dma_writer_format_write_uv;
    secondary_ops->bank0_base_write = pipe->api[0].p_acamera_isp_dma_writer_bank0_base_write_uv;
    secondary_ops->line_offset_write = pipe->api[0].p_acamera_isp_dma_writer_line_offset_write_uv;
    secondary_ops->write_on_write = pipe->api[0].p_acamera_isp_dma_writer_frame_write_on_write_uv;
    secondary_ops->active_width_write = pipe->api[0].p_acamera_isp_dma_writer_active_width_write_uv;
    secondary_ops->active_height_write = pipe->api[0].p_acamera_isp_dma_writer_active_height_write_uv;
    secondary_ops->active_width_read = pipe->api[0].p_acamera_isp_dma_writer_active_width_read_uv;
    secondary_ops->active_height_read = pipe->api[0].p_acamera_isp_dma_writer_active_height_read_uv;

    primary_ops->format_read_hw = pipe->api[1].p_acamera_isp_dma_writer_format_read;
    primary_ops->format_write_hw = pipe->api[1].p_acamera_isp_dma_writer_format_write;
    primary_ops->bank0_base_write_hw = pipe->api[1].p_acamera_isp_dma_writer_bank0_base_write;
    primary_ops->line_offset_write_hw = pipe->api[1].p_acamera_isp_dma_writer_line_offset_write;
    primary_ops->write_on_write_hw = pipe->api[1].p_acamera_isp_dma_writer_frame_write_on_write;
    primary_ops->active_width_write_hw = pipe->api[1].p_acamera_isp_dma_writer_active_width_write;
    primary_ops->active_height_write_hw = pipe->api[1].p_acamera_isp_dma_writer_active_height_write;
    primary_ops->active_width_read_hw = pipe->api[1].p_acamera_isp_dma_writer_active_width_read;
    primary_ops->active_height_read_hw = pipe->api[1].p_acamera_isp_dma_writer_active_height_read;

    secondary_ops->format_read_hw = pipe->api[1].p_acamera_isp_dma_writer_format_read_uv;
    secondary_ops->format_write_hw = pipe->api[1].p_acamera_isp_dma_writer_format_write_uv;
    secondary_ops->bank0_base_write_hw = pipe->api[1].p_acamera_isp_dma_writer_bank0_base_write_uv;
    secondary_ops->line_offset_write_hw = pipe->api[1].p_acamera_isp_dma_writer_line_offset_write_uv;
    secondary_ops->write_on_write_hw = pipe->api[1].p_acamera_isp_dma_writer_frame_write_on_write_uv;
    secondary_ops->active_width_write_hw = pipe->api[1].p_acamera_isp_dma_writer_active_width_write_uv;
    secondary_ops->active_height_write_hw = pipe->api[1].p_acamera_isp_dma_writer_active_height_write_uv;
    secondary_ops->active_width_read_hw = pipe->api[1].p_acamera_isp_dma_writer_active_width_read_uv;
    secondary_ops->active_height_read_hw = pipe->api[1].p_acamera_isp_dma_writer_active_height_read_uv;
}

static int dma_writer_stream_get_frame( dma_pipe *pipe, tframe_t *tframe )
{
    uint32_t ctx_id = pipe->settings.p_ctx->context_id;
    acamera_stream_type_t type = pipe->type == dma_fr ? ACAMERA_STREAM_FR : ACAMERA_STREAM_DS1;
    acamera_settings *settings = &pipe->settings.p_ctx->settings;
    aframe_t tmp_aframes[2];
    int rc;

    tmp_aframes[0] = tframe->primary;
    tmp_aframes[1] = tframe->secondary;

    rc = settings->callback_stream_get_frame( ctx_id, type, tmp_aframes, 2 );

    tframe->primary = tmp_aframes[0];
    tframe->secondary = tmp_aframes[1];

    return rc;
}

static int dma_writer_stream_put_frame( dma_pipe *pipe, tframe_t *tframe, uint8_t flag )
{
    uint32_t ctx_id = pipe->settings.p_ctx->context_id;
    acamera_stream_type_t type = pipe->type == dma_fr ? ACAMERA_STREAM_FR : ACAMERA_STREAM_DS1;
    acamera_settings *settings = &pipe->settings.p_ctx->settings;
    aframe_t tmp_aframes[2];
    int rc;

    LOG(LOG_INFO, "ISP output frame id: %d",tframe->primary.frame_id);

    tmp_aframes[0] = tframe->primary;
    tmp_aframes[1] = tframe->secondary;

    rc = settings->callback_stream_put_frame( ctx_id, type, tmp_aframes, 2, flag );

    tframe->primary = tmp_aframes[0];
    tframe->secondary = tmp_aframes[1];

    return rc;
}

#if 0
static int dma_writer_stream_release_frame( dma_pipe *pipe, tframe_t *tframe )
{
    uint32_t ctx_id = pipe->settings.p_ctx->context_id;
    acamera_stream_type_t type = pipe->type == dma_fr ? ACAMERA_STREAM_FR : ACAMERA_STREAM_DS1;
    acamera_settings *settings = &pipe->settings.p_ctx->settings;
    aframe_t tmp_aframes[2];
    int rc;

    rc = settings->callback_stream_release_frame( ctx_id, type, tmp_aframes, 2 );
/*
    LOG(LOG_INFO, "ISP output frame id: %d",tframe->primary.frame_id);

    tmp_aframes[0] = tframe->primary;
    tmp_aframes[1] = tframe->secondary;

    rc = settings->callback_stream_release_frame( ctx_id, type, tmp_aframes, 2 );

    tframe->primary = tmp_aframes[0];
    tframe->secondary = tmp_aframes[1];
*/

    return rc;
}
#endif

void dma_writer_fr_dma_disable(dma_pipe *pipe, int plane, int flip)
{
    uintptr_t base;
    dma_writer_reg_ops_t *primary_ops = &pipe->primary;
    dma_writer_reg_ops_t *secondary_ops = &pipe->secondary;

    if ( acamera_isp_isp_global_ping_pong_config_select_read( 0 ) == ISP_CONFIG_PING ) {
        if (flip) {
            base = ISP_CONFIG_PING_SIZE;
            pr_debug("disable next pong");
        } else {
            base = 0;
            pr_debug("disable cur ping\n");
        }
    } else {
        if (flip) {
            base = 0;
            pr_debug("disable next ping\n");
        } else {
            base = ISP_CONFIG_PING_SIZE;
            pr_debug("disable cur pong\n");
        }
    }

    if (plane == 0) {
        primary_ops->write_on_write( pipe->settings.isp_base, 0 );
        primary_ops->bank0_base_write( pipe->settings.isp_base, 0 );

        primary_ops->write_on_write_hw( base, 0 );
        primary_ops->bank0_base_write_hw( base, 0 );

        pr_debug("disable fr y dma write\n");
    } else if (plane == 1) {
        secondary_ops->write_on_write( pipe->settings.isp_base, 0 );
        secondary_ops->bank0_base_write( pipe->settings.isp_base, 0 );

        secondary_ops->write_on_write_hw( base, 0 );
        secondary_ops->bank0_base_write_hw( base, 0 );

        pr_debug("disable fr uv dma write\n");
    } else {
        pr_err("unknown plane no %d\n", plane);
    }
}

extern acamera_firmware_t *acamera_get_firmware_ptr(void);
extern void dma_writer_config_done(void);
extern int isp_stream_onoff_check(void);
static int dma_writer_configure_frame_writer( dma_pipe *pipe,
                                              aframe_t *aframe,
                                              dma_writer_reg_ops_t *reg_ops )
{
    uint32_t addr;
    uint32_t line_offset;
    uintptr_t base;
    struct _acamera_context_t *p_ctx = pipe->settings.p_ctx;
    acamera_firmware_t *fw_ptr = acamera_get_firmware_ptr();

    if ( acamera_isp_isp_global_ping_pong_config_select_read( 0 ) == ISP_CONFIG_PING ) {
	    base = ISP_CONFIG_PING_SIZE;
        uint32_t v = acamera_isp_fr_dma_writer_bank0_base_read_hw(0);
	    pr_debug("current is ping, addr is 0x%x, next pong\n", v);
    } else {
	    base = 0;
        uint32_t v = acamera_isp_fr_dma_writer_bank0_base_read_hw(ISP_CONFIG_PING_SIZE);
	    pr_debug("current is pong, addr is 0x%x, next ping\n", v);
    }

    /* config ping on first frame.
        in case of raw feedback case, cannot using frame count variable as a condition.
    */
    if (fw_ptr && fw_ptr->first_frame < 2) {
        base = 0;
        fw_ptr->first_frame++;
        pr_debug("this is first frame, config to ping\n");
    }

    if ( aframe->status != dma_buf_purge ) {
        /*
         * For now we don't change the settings, so we take them from the hardware
         * The reason is that they are configured through a different API
         */
        aframe->type = reg_ops->format_read( pipe->settings.isp_base );
        aframe->width = reg_ops->active_width_read( pipe->settings.isp_base );
        aframe->height = reg_ops->active_height_read( pipe->settings.isp_base );

        aframe->line_offset =  ALIGN_UP16(aframe->width);
        aframe->size = aframe->line_offset * aframe->height * 3 / 2;

        addr = aframe->address;
        line_offset = aframe->line_offset;

        if ( pipe->settings.vflip ) {
            addr += aframe->size - aframe->line_offset;
            line_offset = -aframe->line_offset;
        }

        reg_ops->format_write( pipe->settings.isp_base, (uint8_t)aframe->type );
        reg_ops->active_width_write( pipe->settings.isp_base, (uint16_t)aframe->width );
        reg_ops->active_height_write( pipe->settings.isp_base, (uint16_t)aframe->height );
        reg_ops->line_offset_write( pipe->settings.isp_base, line_offset );
        reg_ops->bank0_base_write( pipe->settings.isp_base, addr );
        reg_ops->write_on_write( pipe->settings.isp_base, 1 );

        if (p_ctx->p_gfw->sif_isp_offline == 0) {
            reg_ops->format_write_hw( base, (uint8_t)aframe->type );
            reg_ops->active_width_write_hw( base, (uint16_t)aframe->width );
            reg_ops->active_height_write_hw( base, (uint16_t)aframe->height );
            reg_ops->line_offset_write_hw( base, line_offset );
            reg_ops->bank0_base_write_hw( base, addr );
            reg_ops->write_on_write_hw( base, 1 );
        }
        pr_debug("[s%d] enable dma write, frame%d, %dx%d, stride=%d, phy_addr=0x%x, size=%d, type=%d",
            p_ctx->context_id, aframe->frame_id, aframe->width, aframe->height, aframe->line_offset,
            aframe->address, aframe->size, aframe->type);

        /* for offline, tell feed thread dma writer config done */
        if (aframe->type == DMA_FORMAT_NV12_UV && isp_stream_onoff_check() != 2) {
            p_ctx->p_gfw->handler_flag_interrupt_handle_completed = 1;
            dma_writer_config_done();
        }
    } else {
        reg_ops->write_on_write( pipe->settings.isp_base, 0 );
        reg_ops->bank0_base_write( pipe->settings.isp_base, 0 );
        if (p_ctx->p_gfw->sif_isp_offline == 0) {
            reg_ops->write_on_write_hw( base, 0 );
            reg_ops->bank0_base_write_hw( base, 0 );
        }
        pr_debug("[s%d] disable dma write, frame%d, %dx%d, stride=%d, phy_addr=0x%x, size=%d, type=%d",
            p_ctx->context_id, aframe->frame_id, aframe->width, aframe->height, aframe->line_offset,
            aframe->address, aframe->size, aframe->type);
    }

    return 0;
}

int dma_writer_configure_pipe( dma_pipe *pipe )
{
    struct _acamera_context_t *p_ctx = pipe->settings.p_ctx;
    metadata_t *meta = &pipe->settings.curr_metadata;
    tframe_t *curr_frame = &pipe->settings.curr_frame;
    dma_writer_reg_ops_t *primary_ops = &pipe->primary;
    dma_writer_reg_ops_t *secondary_ops = &pipe->secondary;
    int rc;
    LOG(LOG_INFO,"+: curr_frame_id = %d",curr_frame->primary.frame_id);
    if ( !p_ctx ) {
        LOG( LOG_INFO, "No context available." );
        return -1;
    }

    curr_frame->primary.frame_id = meta->frame_id;
    curr_frame->secondary.frame_id = meta->frame_id;

    /* try to get a new buffer from application (V4l2 for example) */
    rc = dma_writer_stream_get_frame( pipe, curr_frame );
    if ( rc ) {
        curr_frame->primary.status = dma_buf_purge;
        curr_frame->secondary.status = dma_buf_purge;
        dma_writer_fr_dma_disable(pipe, PLANE_Y, 1);
        dma_writer_fr_dma_disable(pipe, PLANE_UV, 1);
        pr_err("get buffer from v4l2 failed.\n");
        acamera_dma_wr_check();
        return 0;
    } else {
        if ( curr_frame->primary.status == dma_buf_empty )
            curr_frame->primary.status = dma_buf_busy;
        if ( curr_frame->secondary.status == dma_buf_empty )
            curr_frame->secondary.status = dma_buf_busy;
    }

    rc = ion_check_in_heap_carveout(curr_frame->primary.address, 0);
    rc |= ion_check_in_heap_carveout(curr_frame->secondary.address, 0);

    /* write the current frame in the software config */
    if (rc == 0) {
        dma_writer_configure_frame_writer( pipe, &curr_frame->primary, primary_ops );
        dma_writer_configure_frame_writer( pipe, &curr_frame->secondary, secondary_ops );
    } else {
        pr_err("y addr 0x%x or uv addr 0x%x is invalid.\n", curr_frame->primary.address, curr_frame->secondary.address);
    }

    return 0;
}

static int dma_writer_done_process( dma_pipe *pipe )
{
    struct _acamera_context_t *p_ctx = pipe->settings.p_ctx;
    tframe_t *curr_frame = &pipe->settings.curr_frame;

    LOG(LOG_INFO,"+: curr_frame_id = %d",curr_frame->primary.frame_id);

    if ( !p_ctx ) {
        LOG( LOG_ERR, "No context available." );
        return -1;
    }

    /* put back frame to the application (V4L2 for example) */
    dma_writer_stream_put_frame( pipe, curr_frame, 0 );

    return 0;
}

static int dma_writer_error_process( dma_pipe *pipe )
{
    struct _acamera_context_t *p_ctx = pipe->settings.p_ctx;
    tframe_t *curr_frame = &pipe->settings.curr_frame;

    LOG(LOG_INFO,"+: curr_frame_id = %d",curr_frame->primary.frame_id);
    LOG(LOG_INFO,"+: curr_frame_id = %d",curr_frame->primary.frame_id);

    if ( !p_ctx ) {
        LOG( LOG_ERR, "No context available." );
        return -1;
    }

    /* put back frame to the application (V4L2 for example) */
    dma_writer_stream_put_frame( pipe, curr_frame, 1 );

    /* release frame to free list */
    // dma_writer_stream_release_frame( pipe, curr_frame );

    return 0;
}

extern void *acamera_get_ctx_ptr(uint32_t ctx_id);
void dma_writer_clear(uint32_t ctx_id)
{
	acamera_context_t *ptr = (acamera_context_t *)acamera_get_ctx_ptr(ctx_id);
	acamera_fsm_mgr_t *fsm_mgr = &(ptr->fsm_mgr);
	dma_writer_fsm_t *dma_fsm;
	dma_handle *dh;

	dma_fsm = (dma_writer_fsm_t *)fsm_mgr->fsm_arr[FSM_ID_DMA_WRITER]->p_fsm;

	dh = (dma_handle *)dma_fsm->handle;

	dh->pipe[dma_fr].settings.curr_frame.primary.status = dma_buf_purge;
	dh->pipe[dma_fr].settings.curr_frame.secondary.status = dma_buf_purge;
	dh->pipe[dma_fr].settings.done_frame.primary.status = dma_buf_purge;
	dh->pipe[dma_fr].settings.done_frame.secondary.status = dma_buf_purge;
	dh->pipe[dma_fr].settings.delay_frame.primary.status = dma_buf_purge;
	dh->pipe[dma_fr].settings.delay_frame.secondary.status = dma_buf_purge;
}

void dma_writer_disable(uint32_t ctx_id)
{
	acamera_context_t *ptr = (acamera_context_t *)acamera_get_ctx_ptr(ctx_id);
	acamera_fsm_mgr_t *fsm_mgr = &(ptr->fsm_mgr);
	dma_writer_fsm_t *dma_fsm;
	dma_handle *dh;

	dma_fsm = (dma_writer_fsm_t *)fsm_mgr->fsm_arr[FSM_ID_DMA_WRITER]->p_fsm;

	dh = (dma_handle *)dma_fsm->handle;

    //ping/pong dma writer disable write
    dh->pipe[dma_fr].primary.write_on_write_hw(0, 0);
    dh->pipe[dma_fr].secondary.write_on_write_hw(0, 0);
    dh->pipe[dma_fr].primary.write_on_write_hw(ISP_CONFIG_PING_SIZE, 0);
    dh->pipe[dma_fr].secondary.write_on_write_hw(ISP_CONFIG_PING_SIZE, 0);
}

void dma_writer_exit( void *handle )
{
#if ISP_HAS_DS1
    dma_writer_free_default_frame( &p_dma->pipe[dma_ds1] );
#endif
}

dma_error dma_writer_update_state( dma_pipe *pipe )
{
    dma_error result = edma_ok;
    if ( pipe != NULL ) {
        LOG( LOG_INFO, "state update %dx%d\n", (int)pipe->settings.width, (int)pipe->settings.height );
        if ( pipe->settings.width == 0 || pipe->settings.height == 0 ) {
            result = edma_invalid_settings;
            pipe->settings.enabled = 0;
            LOG( LOG_ERR, "stop dma writter: %dx%d, enable=%d\n", (int)pipe->settings.width, (int)pipe->settings.height, pipe->settings.enabled );
            pipe->api[0].p_acamera_isp_dma_writer_frame_write_on_write( pipe->settings.isp_base, (uint8_t)pipe->settings.enabled ); //disable pipe on invalid settings
            pipe->api[0].p_acamera_isp_dma_writer_frame_write_on_write_uv( pipe->settings.isp_base, (uint8_t)pipe->settings.enabled );
            return result;
        }

        //  to bypass isp function for ds dma & rgbir (need to fixed)
        acamera_isp_top_isp_downscale_pipe_disable_write(pipe->settings.isp_base, 1 );
        acamera_isp_top_bypass_demosaic_rgbir_write(pipe->settings.isp_base, 1 );

        //set default settings here
        pipe->api[0].p_acamera_isp_dma_writer_max_bank_write( pipe->settings.isp_base, 0 );
        pipe->api[0].p_acamera_isp_dma_writer_active_width_write( pipe->settings.isp_base, (uint16_t)pipe->settings.width );
        pipe->api[0].p_acamera_isp_dma_writer_active_height_write( pipe->settings.isp_base, (uint16_t)pipe->settings.height );
        pipe->api[0].p_acamera_isp_dma_writer_max_bank_write_uv( pipe->settings.isp_base, 0 );
        pipe->api[0].p_acamera_isp_dma_writer_active_width_write_uv( pipe->settings.isp_base, (uint16_t)pipe->settings.width );
        pipe->api[0].p_acamera_isp_dma_writer_active_height_write_uv( pipe->settings.isp_base, (uint16_t)pipe->settings.height );
    } else {
        result = edma_fail;
    }
    return result;
}


dma_error dma_writer_get_settings( void *handle, dma_type type, dma_pipe_settings *settings )
{
    dma_error result = edma_ok;
    if ( handle != NULL ) {
        dma_handle *p_dma = (dma_handle *)handle;
        system_memcpy( (void *)settings, &p_dma->pipe[type].settings, sizeof( dma_pipe_settings ) );
    } else {
        result = edma_fail;
    }
    return result;
}


dma_error dma_writer_set_settings( void *handle, dma_type type, dma_pipe_settings *settings )
{
    dma_error result = edma_ok;
    if ( handle != NULL ) {
        dma_handle *p_dma = (dma_handle *)handle;
        system_memcpy( (void *)&p_dma->pipe[type].settings, settings, sizeof( dma_pipe_settings ) );
        result = dma_writer_update_state( &p_dma->pipe[type] );

    } else {
        LOG( LOG_ERR, "set settings handle null\n" );
        result = edma_fail;
    }
    return result;
}

dma_error dma_writer_get_ptr_settings( void *handle, dma_type type, dma_pipe_settings **settings )
{
    dma_error result = edma_ok;
    if ( handle != NULL ) {
        dma_handle *p_dma = (dma_handle *)handle;
        *settings = &p_dma->pipe[type].settings;
    } else {
        result = edma_fail;
    }
    return result;
}

void dma_writer_set_initialized( void *handle, dma_type type, uint8_t initialized )
{
    if ( handle != NULL ) {
        dma_handle *p_dma = (dma_handle *)handle;
        p_dma->pipe[type].state.initialized = initialized;
    }
}

dma_error dma_writer_init( void *handle, dma_type type, dma_pipe_settings *settings, dma_api *api )
{
    dma_error result = edma_ok;

    if ( handle != NULL ) {
        dma_handle *p_dma = (dma_handle *)handle;
        p_dma->pipe[type].type = type;

        system_memcpy( (void *)&p_dma->pipe[type].api, api, sizeof( dma_api ) * 2 );
        result = dma_writer_set_settings( handle, type, settings );

        dma_writer_initialize_reg_ops( &p_dma->pipe[type] );

        if ( result == edma_ok ) {
            LOG( LOG_INFO, "%s: initialized a dma writer output. %dx%d ", TAG, (int)settings->width, (int)settings->height );
        } else {
            LOG( LOG_ERR, "%s: Failed to initialize a dma writer type:%d output. %dx%d ", TAG, (int)type, (int)settings->width, (int)settings->height );
        }
    } else {
        result = edma_fail;
    }
    return result;
}

dma_error dma_writer_pipe_process_interrupt( dma_pipe *pipe, uint32_t irq_event )
{
    switch ( irq_event ) {
    case ACAMERA_IRQ_FRAME_WRITER_FR:
        if ( pipe->type == dma_fr ) {
            dma_writer_configure_pipe( pipe );
        }
        break;
    case ACAMERA_IRQ_FRAME_WRITER_FR_DONE:
        if ( pipe->type == dma_fr ) {
            dma_writer_done_process( pipe );
        }
        break;
    case ACAMERA_IRQ_FRAME_ERROR:
        if ( pipe->type == dma_fr ) {
	    dma_writer_error_process( pipe );
        }
	break;
    case ACAMERA_IRQ_FRAME_WRITER_DS:
        if ( pipe->type == dma_ds1 ) {
            dma_writer_configure_pipe( pipe );
        }
        break;
    }

    return 0;
}


dma_error dma_writer_process_interrupt( void *handle, uint32_t irq_event )
{
    dma_error result = edma_ok;

    dma_handle *p_dma = (dma_handle *)handle;
    if ( !p_dma ) {
        LOG( LOG_ERR, "p_dma is NULL of event: %u", (unsigned int)irq_event );
        return edma_fail;
    }

    dma_writer_pipe_process_interrupt( &p_dma->pipe[dma_fr], irq_event );

    return result;
}


metadata_t *dma_writer_return_metadata( void *handle, dma_type type )
{
    if ( handle != NULL ) {

        dma_handle *p_dma = (dma_handle *)handle;

        metadata_t *ret = &( p_dma->pipe[type].settings.curr_metadata );
        return ret;
    }
    return 0;
}
