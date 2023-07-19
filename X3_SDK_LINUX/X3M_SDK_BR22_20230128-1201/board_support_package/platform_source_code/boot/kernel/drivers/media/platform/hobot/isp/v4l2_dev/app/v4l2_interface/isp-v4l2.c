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
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/videodev2.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-vmalloc.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-event.h>
#include <linux/pm_qos.h>
#ifdef CONFIG_HOBOT_XJ3
#ifdef CONFIG_ARM_HOBOT_DMC_DEVFREQ
#include <linux/devfreq.h>
#endif
#endif
#include "acamera_logger.h"

#include "isp-v4l2-common.h"
#include "isp-v4l2.h"
#include "isp-v4l2-ctrl.h"
#include "isp-v4l2-stream.h"
#include "isp-vb2.h"
#include "fw-interface.h"
#include "acamera_fw.h"
#include "general_fsm.h"
#include "vio_group_api.h"
#include "acamera.h"
#include "system_dma.h"

#define ISP_V4L2_NUM_INPUTS 1

#define ISPIOC_INPUT_PORT_CTRL _IOWR('@', 0x10, input_port_t)
#define ISPIOC_MIRROR_CTRL _IOWR('@', 0x11, uint8_t)
#define ISPIOC_WAKE_UP_CTRL _IO('@', 0x12)

/* isp_v4l2_dev_t to destroy video device */
static isp_v4l2_dev_t *g_isp_v4l2_devs[FIRMWARE_CONTEXT_NUMBER];
struct mutex init_lock;
extern int interrupt_line_ACAMERA_JUNO_IRQ;
extern int acamera_fw_isp_start(int ctx_id);
extern int acamera_fw_isp_stop(int ctx_id);
extern void acamera_fw_mem_free(void);
extern int acamera_isp_init_context(uint8_t idx);
extern int acamera_isp_deinit_context(uint8_t idx);
extern void acamera_isp_evt_thread_stop(uint8_t idx);
extern void *acamera_get_ctx_ptr(uint32_t ctx_id);
extern int ips_set_clk_ctrl(unsigned long module, bool enable);
extern void general_temper_disable(void);
static int _v4l2_stream_off(struct file *file);
static struct pm_qos_request isp_pm_qos_req;
/* ----------------------------------------------------------------
 * V4L2 file handle structures and functions
 * : implementing multi stream
 */
#define fh_to_private( __fh ) \
    container_of( __fh, struct isp_v4l2_fh, fh )

struct isp_v4l2_fh {
    struct v4l2_fh fh;
    unsigned int stream_id;
    struct vb2_queue vb2_q;
};

static int isp_v4l2_fh_open( struct file *file )
{
    isp_v4l2_dev_t *dev = video_drvdata( file );
    struct isp_v4l2_fh *sp;
    int i;

    sp = kzalloc( sizeof( struct isp_v4l2_fh ), GFP_KERNEL );
    if ( !sp )
        return -ENOMEM;

    unsigned int stream_opened = atomic_read( &dev->opened );
    if ( stream_opened >= V4L2_STREAM_TYPE_MAX ) {
        LOG( LOG_ERR, "too many open streams, stream_opened: %d, max: %d.", stream_opened, V4L2_STREAM_TYPE_MAX );
        kzfree( sp );
        return -EBUSY;
    }

    file->private_data = &sp->fh;

    for ( i = 0; i < V4L2_STREAM_TYPE_MAX; i++ ) {
        if ( ( dev->stream_mask & ( 1 << i ) ) == 0 ) {
            dev->stream_mask |= ( 1 << i );
            sp->stream_id = i;
            break;
        }
    }

    v4l2_fh_init( &sp->fh, &dev->video_dev );
    v4l2_fh_add( &sp->fh );

    return 0;
}

static int isp_v4l2_fh_release( struct file *file )
{
    struct isp_v4l2_fh *sp = fh_to_private( file->private_data );

    if ( sp ) {
        v4l2_fh_del( &sp->fh );
        v4l2_fh_exit( &sp->fh );
    }
    kzfree( sp );

    return 0;
}

int isp_open_check(void)
{
    int i;
    int total_open = 0;
    for (i = 0; i < FIRMWARE_CONTEXT_NUMBER; i++) {
	    isp_v4l2_dev_t *d = isp_v4l2_get_dev(i);
	    total_open += atomic_read(&d->opened);
    }

    pr_debug("total_open count %d\n", total_open);

    return total_open;
}

int isp_stream_onoff_check(void)
{
    int i;
    int total_stream_on = 0;
    for (i = 0; i < FIRMWARE_CONTEXT_NUMBER; i++) {
	    isp_v4l2_dev_t *d = isp_v4l2_get_dev(i);
	    total_stream_on += atomic_read(&d->stream_on_cnt);
    }

    pr_debug("total_stream_on count %d\n", total_stream_on);

    return total_stream_on;
}

int isp_v4l2_update_ctx(int ctx_id)
{
    int rc = 0;
    acamera_context_t *p_ctx = acamera_get_ctx_ptr(ctx_id);

    if (isp_stream_onoff_check() == 0 && isp_open_check() <= 1) {
        if (p_ctx->dma_chn_idx >= 0 && p_ctx->dma_chn_idx < HW_CONTEXT_NUMBER) {
            acamera_update_cur_settings_to_isp(p_ctx->dma_chn_idx);
        }
    }

    return rc;
}

/* ----------------------------------------------------------------
 * V4L2 file operations
 */
static int isp_v4l2_fop_open( struct file *file )
{
    int rc = 0;
    isp_v4l2_dev_t *dev = video_drvdata( file );
    struct isp_v4l2_fh *sp;

    if (dev->ctx_id >= FIRMWARE_CONTEXT_NUMBER) {
        rc = -1;
        pr_err("ctx_id %d exceed valid range\n", dev->ctx_id);
        return rc;
    }

    mutex_lock(&init_lock);
    pr_debug("ctx_id %d +\n", dev->ctx_id);

    if (isp_open_check() == 0) {
#ifdef CONFIG_HOBOT_XJ3
#ifdef CONFIG_ARM_HOBOT_DMC_DEVFREQ
        pm_qos_add_request(&isp_pm_qos_req, PM_QOS_DEVFREQ, 10000);
        if (!hobot_dmcfreq_checkup_max()) {
            pr_err("set ddr freq max failed\n");
            rc = -1;
            mutex_unlock(&init_lock);
            return rc;
        }
#endif
#endif
        ips_set_clk_ctrl(ISP0_CLOCK_GATE, true);
        ips_set_module_reset(ISP0_RST);
    }

    /* update open counter */
    atomic_add( 1, &dev->opened );

    rc = acamera_isp_init_context((uint8_t)dev->ctx_id);
    if (rc != 0) {
        goto fh_open_fail;
    }

    /* open file header */
    rc = isp_v4l2_fh_open( file );
    if ( rc < 0 ) {
        LOG( LOG_ERR, "Error, file handle open fail (rc=%d)", rc );
        goto fh_open_fail;
    }
    sp = fh_to_private( file->private_data );

    LOG( LOG_INFO, "isp_v4l2 open: ctx_id: %d, called for sid:%d.", dev->ctx_id, sp->stream_id );
    /* init stream */
    isp_v4l2_stream_init( &dev->pstreams[sp->stream_id], sp->stream_id, dev->ctx_id );
    if ( sp->stream_id == 0 ) {
        // stream_id 0 is a full resolution
        dev->stream_id_index[V4L2_STREAM_TYPE_FR] = sp->stream_id;
    }
#if ISP_HAS_DS1
    else if ( sp->stream_id == V4L2_STREAM_TYPE_DS1 ) {
        dev->stream_id_index[V4L2_STREAM_TYPE_DS1] = sp->stream_id;
    }
#endif

    /* init vb2 queue */

    rc = isp_vb2_queue_init( &sp->vb2_q, &dev->mlock, dev->pstreams[sp->stream_id], dev->v4l2_dev->dev );
    if ( rc < 0 ) {
        LOG( LOG_ERR, "Error, vb2 queue init fail (rc=%d)", rc );
        goto vb2_q_fail;
    }

    /* init fh_ptr */
    if ( mutex_lock_interruptible( &dev->notify_lock ) ) {
        LOG( LOG_ERR, "mutex_lock_interruptible failed.\n" );
        goto vb2_q_deinit;
    }
    dev->fh_ptr[sp->stream_id] = &( sp->fh );
    mutex_unlock( &dev->notify_lock );

    pr_debug("ctx_id %d -\n", dev->ctx_id);
    mutex_unlock(&init_lock);

    return rc;

vb2_q_deinit:
    isp_vb2_queue_release(&sp->vb2_q);

vb2_q_fail:
    isp_v4l2_stream_deinit( dev->pstreams[sp->stream_id] );

    //too_many_stream:
    isp_v4l2_fh_release( file );

fh_open_fail:
	atomic_dec(&dev->opened);
	//isp hardware stop
	if (isp_open_check() == 0) {
#ifdef CONFIG_ARM_HOBOT_DMC_DEVFREQ
		pm_qos_remove_request(&isp_pm_qos_req);
#endif
		ips_set_clk_ctrl(ISP0_CLOCK_GATE, false);
		acamera_fw_mem_free();
	}
	mutex_unlock(&init_lock);
    return rc;
}

extern void isp_temper_free(general_fsm_ptr_t p_fsm);
extern void dma_writer_disable(uint32_t ctx_id);
static int isp_v4l2_fop_close( struct file *file )
{
    isp_v4l2_dev_t *dev = video_drvdata( file );
    struct isp_v4l2_fh *sp = fh_to_private( file->private_data );
    isp_v4l2_stream_t *pstream = dev->pstreams[sp->stream_id];

    pr_debug("ctx_id %d +\n", dev->ctx_id);

	mutex_lock(&init_lock);

    dev->stream_mask &= ~( 1 << sp->stream_id );
    atomic_dec( &dev->opened );

    //evt-thread will touch hardware when processing event, stop it first
    acamera_isp_evt_thread_stop((uint8_t)dev->ctx_id);

    //force stream off
    if (atomic_read(&dev->stream_on_cnt))
        _v4l2_stream_off(file);

    //isp hardware stop
    if (isp_open_check() == 0) {
        acamera_fw_isp_stop(dev->ctx_id);
        general_temper_disable();
        dma_writer_disable(dev->ctx_id);
        ips_set_clk_ctrl(ISP0_CLOCK_GATE, false);
        acamera_fw_mem_free();
#ifdef CONFIG_ARM_HOBOT_DMC_DEVFREQ
        pm_qos_remove_request(&isp_pm_qos_req);
#endif
    }
    acamera_isp_deinit_context((uint8_t)dev->ctx_id);

    /* deinit fh_ptr */
    mutex_lock( &dev->notify_lock );
    dev->fh_ptr[sp->stream_id] = NULL;
    mutex_unlock( &dev->notify_lock );

    /* deinit stream */
    if ( pstream ) {
        if ( pstream->stream_type < V4L2_STREAM_TYPE_MAX )
            dev->stream_id_index[pstream->stream_type] = -1;
        isp_v4l2_stream_deinit( pstream );
        dev->pstreams[sp->stream_id] = NULL;
    }

    /* release vb2 queue */
    if ( sp->vb2_q.lock )
        mutex_lock( sp->vb2_q.lock );

    isp_vb2_queue_release( &sp->vb2_q );

    if ( sp->vb2_q.lock )
        mutex_unlock( sp->vb2_q.lock );

    /* release file handle */
    isp_v4l2_fh_release( file );

    mutex_unlock(&init_lock);

    pr_debug("ctx_id %d -\n", dev->ctx_id);

    return 0;
}

static ssize_t isp_v4l2_fop_read( struct file *filep,
                                  char __user *buf, size_t count, loff_t *ppos )
{
    struct isp_v4l2_fh *sp = fh_to_private( filep->private_data );
    int rc = 0;

    rc = (int)vb2_read( &sp->vb2_q, buf, count, ppos, filep->f_flags & O_NONBLOCK );

    return rc;
}

static unsigned int isp_v4l2_fop_poll( struct file *filep,
                                       struct poll_table_struct *wait )
{
    struct isp_v4l2_fh *sp = fh_to_private( filep->private_data );
    int rc = 0;

    if ( sp->vb2_q.lock && mutex_lock_interruptible( sp->vb2_q.lock ) )
        return POLLERR;

    rc = vb2_poll( &sp->vb2_q, filep, wait );

    if ( sp->vb2_q.lock )
        mutex_unlock( sp->vb2_q.lock );

    return rc;
}

static int isp_v4l2_fop_mmap( struct file *file, struct vm_area_struct *vma )
{
    struct isp_v4l2_fh *sp = fh_to_private( file->private_data );
    int rc = 0;

    rc = vb2_mmap( &sp->vb2_q, vma );

    return rc;
}

static const struct v4l2_file_operations isp_v4l2_fops = {
    .owner = THIS_MODULE,
    .open = isp_v4l2_fop_open,
    .release = isp_v4l2_fop_close,
    .read = isp_v4l2_fop_read,
    .poll = isp_v4l2_fop_poll,
    .unlocked_ioctl = video_ioctl2,
    .mmap = isp_v4l2_fop_mmap,
};


/* ----------------------------------------------------------------
 * V4L2 ioctl operations
 */
static int isp_v4l2_querycap( struct file *file, void *priv, struct v4l2_capability *cap )
{
    isp_v4l2_dev_t *dev = video_drvdata( file );

    LOG( LOG_DEBUG, "dev: %p, file: %p, priv: %p.\n", dev, file, priv );

    strcpy( cap->driver, "x3-isp" );
    strcpy( cap->card, "juno R2" );
    snprintf( cap->bus_info, sizeof( cap->bus_info ), "platform:%s", dev->v4l2_dev->name );

    /* V4L2_CAP_VIDEO_CAPTURE_MPLANE */

    cap->device_caps = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING | V4L2_CAP_READWRITE;
    cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

    return 0;
}


/* format related, will be moved to isp-v4l2-stream.c */
static int isp_v4l2_g_fmt_vid_cap( struct file *file, void *priv, struct v4l2_format *f )
{
    isp_v4l2_dev_t *dev = video_drvdata( file );
    struct isp_v4l2_fh *sp = fh_to_private( file->private_data );
    isp_v4l2_stream_t *pstream = dev->pstreams[sp->stream_id];

    LOG( LOG_DEBUG, "isp_v4l2_g_fmt_vid_cap sid:%d", sp->stream_id );

    isp_v4l2_stream_get_format( pstream, f );

    LOG( LOG_DEBUG, "v4l2_format: type: %u, w: %u, h: %u, pixelformat: 0x%x, field: %u, colorspace: %u, sizeimage: %u, bytesperline: %u, flags: %u.\n",
         f->type,
         f->fmt.pix.width,
         f->fmt.pix.height,
         f->fmt.pix.pixelformat,
         f->fmt.pix.field,
         f->fmt.pix.colorspace,
         f->fmt.pix.sizeimage,
         f->fmt.pix.bytesperline,
         f->fmt.pix.flags );

    return 0;
}

static int isp_v4l2_enum_fmt_vid_cap( struct file *file, void *priv, struct v4l2_fmtdesc *f )
{
    isp_v4l2_dev_t *dev = video_drvdata( file );
    struct isp_v4l2_fh *sp = fh_to_private( file->private_data );
    isp_v4l2_stream_t *pstream = dev->pstreams[sp->stream_id];

    return isp_v4l2_stream_enum_format( pstream, f );
}

static int isp_v4l2_try_fmt_vid_cap( struct file *file, void *priv, struct v4l2_format *f )
{
    isp_v4l2_dev_t *dev = video_drvdata( file );
    struct isp_v4l2_fh *sp = fh_to_private( file->private_data );
    isp_v4l2_stream_t *pstream = dev->pstreams[sp->stream_id];
    LOG( LOG_INFO, "isp_v4l2_try_fmt_vid_cap" );
    return isp_v4l2_stream_try_format( pstream, f );
}

static int isp_v4l2_s_fmt_vid_cap( struct file *file, void *priv, struct v4l2_format *f )
{
    isp_v4l2_dev_t *dev = video_drvdata( file );
    struct isp_v4l2_fh *sp = fh_to_private( file->private_data );
    isp_v4l2_stream_t *pstream = dev->pstreams[sp->stream_id];
    struct vb2_queue *q = &sp->vb2_q;
    int rc = 0;
    LOG( LOG_INFO, "isp_v4l2_s_fmt_vid_cap sid:%d", sp->stream_id );

    //linear/pwl switch will call this func, move this if case to isp_v4l2_stream_set_format
    // if ( vb2_is_busy( q ) )
    //     return -EBUSY;

    rc = isp_v4l2_stream_set_format( pstream, f, q );
    if ( rc < 0 ) {
        LOG( LOG_ERR, "set format failed." );
        return rc;
    }

    /* update stream pointer index */
    dev->stream_id_index[pstream->stream_type] = pstream->stream_id;

    return 0;
}

static int isp_v4l2_enum_framesizes( struct file *file, void *priv, struct v4l2_frmsizeenum *fsize )
{
    isp_v4l2_dev_t *dev = video_drvdata( file );
    struct isp_v4l2_fh *sp = fh_to_private( file->private_data );
    isp_v4l2_stream_t *pstream = dev->pstreams[sp->stream_id];

    return isp_v4l2_stream_enum_framesizes( pstream, fsize );
}


/* Per-stream control operations */
static inline bool isp_v4l2_is_q_busy( struct vb2_queue *queue, struct file *file )
{
    return queue->owner && queue->owner != file->private_data;
}

static int isp_v4l2_streamon( struct file *file, void *priv, enum v4l2_buf_type i )
{
    isp_v4l2_dev_t *dev = video_drvdata( file );
    struct isp_v4l2_fh *sp = fh_to_private( priv );
    isp_v4l2_stream_t *pstream = dev->pstreams[sp->stream_id];
    int rc = 0;

    pr_info("ctx_id %d+\n", dev->ctx_id);
    if (dev->ctx_id >= FIRMWARE_CONTEXT_NUMBER) {
        rc = -1;
        pr_err("ctx_id %d exceed valid range\n", dev->ctx_id);
        return rc;
    }

    if (isp_v4l2_is_q_busy(&sp->vb2_q, file)) {
        LOG(LOG_ERR, "ctx_id %d, isp_v4l2_is_q_busy", dev->ctx_id);
        return -EBUSY;
    }

    acamera_fsm_mgr_t *instance = &(((acamera_context_ptr_t)acamera_get_ctx_ptr(pstream->ctx_id))->fsm_mgr);
    if (instance->reserved) { //dma writer on
        rc = vb2_streamon( &sp->vb2_q, i );
        if ( rc != 0 ) {
            LOG( LOG_ERR, "fail to vb2_streamon. (rc=%d)", rc );
            return rc;
        }
    }

    mutex_lock(&init_lock);
    if (isp_stream_onoff_check() == 0) {
            rc = acamera_fw_isp_start(dev->ctx_id);
            if (rc != 0) {
                mutex_unlock(&init_lock);
                return rc;
            }
    }
    mutex_unlock(&init_lock);

    /* Start hardware */
    rc = isp_v4l2_stream_on( pstream );
    if ( rc != 0 ) {
        LOG( LOG_ERR, "fail to isp_stream_on. (stream_id = %d, rc=%d)", sp->stream_id, rc );
        isp_v4l2_stream_off( pstream );
        return rc;
    }

	acamera_context_ptr_t p_ctx = acamera_get_ctx_ptr(pstream->ctx_id);
	if (atomic_inc_return(&dev->stream_on_cnt) == 1) {
		//irq bind cpu-1 when sif-online-isp
		if(p_ctx->p_gfw->sif_isp_offline == 0)
			vio_irq_affinity_set(interrupt_line_ACAMERA_JUNO_IRQ, MOD_ISP, 0, 1);
	}
    pr_info("ctx_id %d-\n", dev->ctx_id);

    return rc;
}

static int _v4l2_stream_off(struct file *file)
{
    int rc = 0;
    isp_v4l2_dev_t *dev = video_drvdata(file);
    struct isp_v4l2_fh *sp = fh_to_private(file->private_data);
    isp_v4l2_stream_t *pstream = dev->pstreams[sp->stream_id];

    pr_info("ctx_id %d+\n", dev->ctx_id);
    if (isp_v4l2_is_q_busy(&sp->vb2_q, file))
        return -EBUSY;

    atomic_dec(&dev->stream_on_cnt);

    /* Stop hardware */
    if (isp_stream_onoff_check() == 0) {
        acamera_fw_isp_stop(dev->ctx_id);
        general_temper_disable();
        dma_writer_disable(dev->ctx_id);
#if FW_USE_HOBOT_DMA
        system_dma_desc_flush();
#endif
    }

    isp_v4l2_stream_off(pstream);

    /* vb streamoff */
    acamera_fsm_mgr_t *instance = &(((acamera_context_ptr_t)acamera_get_ctx_ptr(pstream->ctx_id))->fsm_mgr);
    if (instance->reserved) { //dma writer on
        rc = vb2_streamoff(&sp->vb2_q, sp->vb2_q.type);
    }

    pr_info("ctx_id %d-\n", dev->ctx_id);

    return rc;
}

static int isp_v4l2_streamoff( struct file *file, void *priv, enum v4l2_buf_type i )
{
    return _v4l2_stream_off(file);
}


/* input control */
static int isp_v4l2_enum_input( struct file *file, void *fh, struct v4l2_input *input )
{
    /* currently only support general camera input */
    if ( input->index > 0 )
        return -EINVAL;

    strlcpy( input->name, "camera", sizeof( input->name ) );
    input->type = V4L2_INPUT_TYPE_CAMERA;

    return 0;
}

static int isp_v4l2_g_input( struct file *file, void *fh, unsigned int *input )
{
    /* currently only support general camera input */
    *input = 0;

    return 0;
}

static int isp_v4l2_s_input( struct file *file, void *fh, unsigned int input )
{
    /* currently only support general camera input */
    return input == 0 ? 0 : -EINVAL;
}


/* vb2 customization for multi-stream support */
static int isp_v4l2_reqbufs( struct file *file, void *priv,
                             struct v4l2_requestbuffers *p )
{
    struct isp_v4l2_fh *sp = fh_to_private( file->private_data );
    int rc = 0;

    LOG( LOG_DEBUG, "(stream_id = %d, ownermatch=%d)", sp->stream_id, isp_v4l2_is_q_busy( &sp->vb2_q, file ) );
    if ( isp_v4l2_is_q_busy( &sp->vb2_q, file ) )
        return -EBUSY;

    rc = vb2_reqbufs( &sp->vb2_q, p );
    if ( rc == 0 )
        sp->vb2_q.owner = p->count ? file->private_data : NULL;

    LOG( LOG_DEBUG, "sid:%d reqbuf p->type:%d p->memory %d p->count %d rc %d", sp->stream_id, p->type, p->memory, p->count, rc );
    return rc;
}

static int isp_v4l2_expbuf( struct file *file, void *priv, struct v4l2_exportbuffer *p )
{
    struct isp_v4l2_fh *sp = fh_to_private( file->private_data );
    int rc = 0;

    if ( isp_v4l2_is_q_busy( &sp->vb2_q, file ) )
        return -EBUSY;

    rc = vb2_expbuf( &sp->vb2_q, p );
    LOG( LOG_DEBUG, "expbuf sid:%d type:%d index:%d plane:%d rc: %d",
         sp->stream_id, p->type, p->index, p->plane, rc );

    return rc;
}

static int isp_v4l2_querybuf( struct file *file, void *priv, struct v4l2_buffer *p )
{
    struct isp_v4l2_fh *sp = fh_to_private( file->private_data );
    int rc = 0;

    rc = vb2_querybuf( &sp->vb2_q, p );
    LOG( LOG_DEBUG, "sid:%d querybuf p->type:%d p->index:%d , rc %d", sp->stream_id, p->type, p->index, rc );
    return rc;
}

static int isp_v4l2_qbuf( struct file *file, void *priv, struct v4l2_buffer *p )
{
    struct isp_v4l2_fh *sp = fh_to_private( file->private_data );
    isp_v4l2_dev_t *dev = video_drvdata( file );
    acamera_context_t *p_ctx;
    int rc = 0;

    LOG( LOG_DEBUG, "(stream_id = %d, ownermatch=%d)", sp->stream_id, isp_v4l2_is_q_busy( &sp->vb2_q, file ) );
    if ( isp_v4l2_is_q_busy( &sp->vb2_q, file ) )
        return -EBUSY;

    p_ctx = acamera_get_ctx_ptr(dev->ctx_id);
    p_ctx->sts.qbuf_cnt++;

    rc = vb2_qbuf( &sp->vb2_q, p );
    LOG( LOG_DEBUG, "sid:%d qbuf p->type:%d p->index:%d, rc %d", sp->stream_id, p->type, p->index, rc );
    return rc;
}

static int isp_v4l2_dqbuf( struct file *file, void *priv, struct v4l2_buffer *p )
{
    struct isp_v4l2_fh *sp = fh_to_private( file->private_data );
    isp_v4l2_dev_t *dev = video_drvdata( file );
    acamera_context_t *p_ctx;
    int rc = 0;

    LOG( LOG_DEBUG, "(stream_id = %d, ownermatch=%d)", sp->stream_id, isp_v4l2_is_q_busy( &sp->vb2_q, file ) );
    if ( isp_v4l2_is_q_busy( &sp->vb2_q, file ) )
        return -EBUSY;

    p_ctx = acamera_get_ctx_ptr(dev->ctx_id);
    p_ctx->sts.dqbuf_cnt++;

    rc = vb2_dqbuf( &sp->vb2_q, p, file->f_flags & O_NONBLOCK );
    LOG( LOG_DEBUG, "sid:%d qbuf p->type:%d p->index:%d, rc %d", sp->stream_id, p->type, p->index, rc );
    return rc;
}

static void isp_wake_up_poll(struct vb2_queue *q)
{
	vb2_queue_error(q);
	return;
}

long isp_v4l2_ioc_default(struct file *file, void *fh, bool valid_prio, unsigned int cmd, void *arg)
{
    int ret = 0;
    acamera_context_t *p_ctx;
    isp_v4l2_dev_t *dev = video_drvdata( file );
    struct isp_v4l2_fh *sp = fh_to_private( file->private_data );

    p_ctx = acamera_get_ctx_ptr(dev->ctx_id);

	switch (cmd) {
	case ISPIOC_INPUT_PORT_CTRL:
        memcpy(&p_ctx->inport, (input_port_t *)arg, sizeof(input_port_t));
        break;
	case ISPIOC_WAKE_UP_CTRL:
			isp_wake_up_poll(&sp->vb2_q);
			break;
	case ISPIOC_MIRROR_CTRL: {
        uint8_t v1 = 0, v2 = 0;
        v1 = *((uint8_t *)arg);
		if (p_ctx->timestamps == 0) {
			v2 = acamera_isp_rggb_start_pre_mirror_read_hw();
			if (v1) //mirror on
				acamera_isp_rggb_start_post_mirror_write_hw(v2 % 2 ? (uint8_t)(v2 - 1) : (uint8_t)(v2 + 1));
			else
				acamera_isp_rggb_start_post_mirror_write_hw(v2);

			acamera_isp_bypass_mirror_write_hw(!v1);
		}

		v2 = acamera_isp_top_rggb_start_pre_mirror_read(p_ctx->settings.isp_base);
        if (v1) //mirror on
            acamera_isp_top_rggb_start_post_mirror_write(p_ctx->settings.isp_base, v2 % 2 ? (uint8_t)(v2 - 1) : (uint8_t)(v2 + 1));
        else
            acamera_isp_top_rggb_start_post_mirror_write(p_ctx->settings.isp_base, v2);

        acamera_isp_top_bypass_mirror_write(p_ctx->settings.isp_base, !v1);
        break;
    }
	default:
		return -ENOTTY;
	}

    pr_debug("xtotal %d, ytotal %d, xoffset %d, yoffset %d\n",
        p_ctx->inport.xtotal, p_ctx->inport.ytotal,
        p_ctx->inport.xoffset, p_ctx->inport.yoffset);

    return ret;
}

static const struct v4l2_ioctl_ops isp_v4l2_ioctl_ops = {
    .vidioc_querycap = isp_v4l2_querycap,

    /* Per-stream config operations */
    .vidioc_g_fmt_vid_cap_mplane = isp_v4l2_g_fmt_vid_cap,
    .vidioc_enum_fmt_vid_cap_mplane = isp_v4l2_enum_fmt_vid_cap,
    .vidioc_try_fmt_vid_cap_mplane = isp_v4l2_try_fmt_vid_cap,
    .vidioc_s_fmt_vid_cap_mplane = isp_v4l2_s_fmt_vid_cap,
    .vidioc_enum_framesizes = isp_v4l2_enum_framesizes,

    /* Per-stream control operations */
    .vidioc_streamon = isp_v4l2_streamon,
    .vidioc_streamoff = isp_v4l2_streamoff,

    /* input control */
    .vidioc_enum_input = isp_v4l2_enum_input,
    .vidioc_g_input = isp_v4l2_g_input,
    .vidioc_s_input = isp_v4l2_s_input,

    /* vb2 customization for multi-stream support */
    .vidioc_reqbufs = isp_v4l2_reqbufs,
    .vidioc_expbuf = isp_v4l2_expbuf,

    .vidioc_querybuf = isp_v4l2_querybuf,
    .vidioc_qbuf = isp_v4l2_qbuf,
    .vidioc_dqbuf = isp_v4l2_dqbuf,

    /* v4l2 event ioctls */
    .vidioc_log_status = v4l2_ctrl_log_status,
    .vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
    .vidioc_unsubscribe_event = v4l2_event_unsubscribe,

    /* private ioctls */
    .vidioc_default = isp_v4l2_ioc_default,
};

static int isp_v4l2_init_dev( uint32_t ctx_id, struct v4l2_device *v4l2_dev )
{
    isp_v4l2_dev_t *dev;
    struct video_device *vfd;
    v4l2_std_id tvnorms_cap = 0;
    int rc = 0;
    int i;

    if ( ctx_id >= FIRMWARE_CONTEXT_NUMBER ) {
        LOG( LOG_ERR, "Invalid ctx numbr: %d, max: %d.", ctx_id, FIRMWARE_CONTEXT_NUMBER - 1 );
        return -EINVAL;
    }

    /* allocate main isp_v4l2 state structure */
    dev = kzalloc( sizeof( *dev ), GFP_KERNEL );
    if ( !dev )
        return -ENOMEM;

    memset( dev, 0x0, sizeof( isp_v4l2_dev_t ) );

    /* register v4l2_device */

    dev->v4l2_dev = v4l2_dev;
    dev->ctx_id = ctx_id;

    /* init v4l2 controls */
    dev->isp_v4l2_ctrl.v4l2_dev = dev->v4l2_dev;
    dev->isp_v4l2_ctrl.video_dev = &dev->video_dev;
    rc = isp_v4l2_ctrl_init( ctx_id, &dev->isp_v4l2_ctrl );
    if ( rc )
        goto free_dev;

    /* initialize locks */
    mutex_init( &dev->mlock );
    mutex_init( &dev->notify_lock );
    mutex_init(&init_lock);

    /* initialize stream id table */
    for ( i = 0; i < V4L2_STREAM_TYPE_MAX; i++ ) {
        dev->stream_id_index[i] = -1;
    }

    /* initialize open counter */
    atomic_set( &dev->stream_on_cnt, 0 );
    atomic_set( &dev->opened, 0 );

    /* finally start creating the device nodes */
    vfd = &dev->video_dev;
    strlcpy( vfd->name, "isp_v4l2-vid-cap", sizeof( vfd->name ) );
    vfd->fops = &isp_v4l2_fops;
    vfd->ioctl_ops = &isp_v4l2_ioctl_ops;
    vfd->release = video_device_release_empty;
    vfd->v4l2_dev = dev->v4l2_dev;
    vfd->queue = NULL; // queue will be customized in file handle
    vfd->tvnorms = tvnorms_cap;
    vfd->vfl_type = VFL_TYPE_GRABBER;

    /*
     * Provide a mutex to v4l2 core. It will be used to protect
     * all fops and v4l2 ioctls.
     */
    vfd->lock = &dev->mlock;
    video_set_drvdata( vfd, dev );

    /* videoX start number, -1 is autodetect */
    rc = video_register_device( vfd, VFL_TYPE_GRABBER, -1 );
    if ( rc < 0 )
        goto unreg_dev;

    LOG( LOG_INFO, "V4L2 capture device registered as %s.",
         video_device_node_name( vfd ) );

    /* store dev pointer to destroy later and find stream */
    g_isp_v4l2_devs[ctx_id] = dev;

    return rc;

unreg_dev:
    video_unregister_device( &dev->video_dev );
    isp_v4l2_ctrl_deinit( &dev->isp_v4l2_ctrl );

free_dev:
    kfree( dev );

    return rc;
}


static void isp_v4l2_destroy_dev( int ctx_id )
{
    if ( g_isp_v4l2_devs[ctx_id] ) {
        LOG( LOG_INFO, "unregistering %s.", video_device_node_name( &g_isp_v4l2_devs[ctx_id]->video_dev ) );

        /* unregister video device */
        video_unregister_device( &g_isp_v4l2_devs[ctx_id]->video_dev );

        isp_v4l2_ctrl_deinit( &g_isp_v4l2_devs[ctx_id]->isp_v4l2_ctrl );

        kfree( g_isp_v4l2_devs[ctx_id] );
        g_isp_v4l2_devs[ctx_id] = NULL;
    } else {
        LOG( LOG_INFO, "isp_v4l2_dev ctx_id: %d is NULL, skip.", ctx_id );
    }
}


/* ----------------------------------------------------------------
 * V4L2 external interface for probe
 */
int isp_v4l2_create_instance( struct v4l2_device *v4l2_dev, uint32_t hw_isp_addr )
{
    int rc = 0;
    uint32_t ctx_id;

    if ( v4l2_dev == NULL ) {
        LOG( LOG_ERR, "Invalid parameter" );
        return -EINVAL;
    }

    ips_set_clk_ctrl(ISP0_CLOCK_GATE, true);

    /* initialize v4l2 layer devices */
    for ( ctx_id = 0; ctx_id < FIRMWARE_CONTEXT_NUMBER; ctx_id++ ) {
        rc = isp_v4l2_init_dev( ctx_id, v4l2_dev );
        if ( rc ) {
            LOG( LOG_ERR, "isp_v4l2_init ctx_id: %d failed.", ctx_id );
            goto unreg_dev;
        }
    }

    /* initialize isp */
    rc = fw_intf_isp_init( hw_isp_addr );
    if ( rc < 0 )
        goto unreg_dev;

    /* initialize stream related resources to prepare for streaming.
     * It should be called after sensor initialized.
     */
    for ( ctx_id = 0; ctx_id < FIRMWARE_CONTEXT_NUMBER; ctx_id++ ) {
        rc = isp_v4l2_stream_init_static_resources( ctx_id );
        if ( rc < 0 )
            goto deinit_fw_intf;
    }

    ips_set_clk_ctrl(ISP0_CLOCK_GATE, false);

    return 0;

deinit_fw_intf:
    fw_intf_isp_deinit();

unreg_dev:
    for ( ctx_id = 0; ctx_id < FIRMWARE_CONTEXT_NUMBER; ctx_id++ ) {
        isp_v4l2_destroy_dev( ctx_id );
    }

    return rc;
}

void isp_v4l2_destroy_instance( void )
{
    LOG( LOG_INFO, "%s: Enter.\n", __func__ );

    if ( g_isp_v4l2_devs[0] ) {
        int ctx_id;

        /* deinitialize firmware & stream resources */
        fw_intf_isp_deinit();
        isp_v4l2_stream_deinit_static_resources();

        for ( ctx_id = 0; ctx_id < FIRMWARE_CONTEXT_NUMBER; ctx_id++ ) {
            isp_v4l2_destroy_dev( ctx_id );
        }
    }
}


/* ----------------------------------------------------------------
 * stream finder utility function
 */
int isp_v4l2_find_stream( isp_v4l2_stream_t **ppstream,
                          int ctx_idber, isp_v4l2_stream_type_t stream_type )
{
    int stream_id;

    *ppstream = NULL;

    if ( stream_type >= V4L2_STREAM_TYPE_MAX || stream_type < 0 || ctx_idber >= FIRMWARE_CONTEXT_NUMBER ) {
        return -EINVAL;
    }

    if ( g_isp_v4l2_devs[ctx_idber] == NULL ) {
        return -EBUSY;
    }

    stream_id = g_isp_v4l2_devs[ctx_idber]->stream_id_index[stream_type];
    if ( stream_id < 0 || stream_id >= V4L2_STREAM_TYPE_MAX || g_isp_v4l2_devs[ctx_idber]->pstreams[stream_id] == NULL ) {
        return -ENODEV;
    }

    *ppstream = g_isp_v4l2_devs[ctx_idber]->pstreams[stream_id];

    return 0;
}

isp_v4l2_stream_type_t fw_to_isp_v4l2_stream_type( acamera_stream_type_t type )
{
    isp_v4l2_stream_type_t v4l2_type;

    switch ( type ) {
    case ACAMERA_STREAM_FR:
        v4l2_type = V4L2_STREAM_TYPE_FR;
        break;

#if ISP_HAS_META_CB
    case ACAMERA_STREAM_META:
        v4l2_type = V4L2_STREAM_TYPE_META;
        break;
#endif
#if ISP_HAS_RAW_CB
    case ACAMERA_STREAM_RAW:
        v4l2_type = V4L2_STREAM_TYPE_RAW;
        break;
#endif
#if ISP_HAS_DS1
    case ACAMERA_STREAM_DS1:
        v4l2_type = V4L2_STREAM_TYPE_DS1;
        break;
#endif
    default:
        LOG( LOG_CRIT, "bad type: %d", type );
        v4l2_type = V4L2_STREAM_TYPE_MAX;
        break;
    };

    return v4l2_type;
}

isp_v4l2_dev_t *isp_v4l2_get_dev( uint32_t ctx_number )
{
    return g_isp_v4l2_devs[ctx_number];
}

/* ----------------------------------------------------------------
 * event notifier utility function
 */
int isp_v4l2_notify_event( int ctx_id, int stream_id, uint32_t event_type )
{
    struct v4l2_event event;

    if ( g_isp_v4l2_devs[ctx_id] == NULL ) {
        return -EBUSY;
    }

    if ( mutex_lock_interruptible( &g_isp_v4l2_devs[ctx_id]->notify_lock ) )
        LOG( LOG_ERR, "mutex_lock_interruptible failed.\n" );

    if ( g_isp_v4l2_devs[ctx_id]->fh_ptr[stream_id] == NULL ) {
        LOG( LOG_ERR, "Error, no fh_ptr exists for stream id %d (event_type = %d)", stream_id, event_type );
        mutex_unlock( &g_isp_v4l2_devs[ctx_id]->notify_lock );
        return -EINVAL;
    }

    memset( &event, 0, sizeof( event ) );
    event.type = event_type;

    v4l2_event_queue_fh( g_isp_v4l2_devs[ctx_id]->fh_ptr[stream_id], &event );
    mutex_unlock( &g_isp_v4l2_devs[ctx_id]->notify_lock );

    return 0;
}

int isp_init_iridix(uint32_t ctx_id, uint32_t ctrl_val)
{
	int i;
	int iridix_no;
	int ret = 0;
	acamera_context_t *ptr_tmp = NULL;
	acamera_context_t *ptr = acamera_get_ctx_ptr(ctx_id);
    if (ptr == NULL) {
        pr_err("ctx %d not inited\n", ctx_id);
        return -1;
    }
	mutex_lock(&ptr->p_gfw->ctx_chg_lock);
	if (ptr->initialized == 1 && ptr->iridix_chn_idx == -1) {
		for (i = 0; i < HW_CONTEXT_NUMBER; i++) {
			 if (!test_bit(i, &ptr->p_gfw->iridix_chn_bitmap))
				 break;
		 }
		if (i < HW_CONTEXT_NUMBER) {
            /* two way to config iridix:
                first - isp_config_seq.h, bypass iridix 0, iridix enable 1
                second - dynamic_context_settings set by user
            */
            if (acamera_isp_top_bypass_iridix_read(ptr->settings.isp_base) == 0) {
                ptr->iridix_chn_idx = i;
                pr_debug("ctx_id = %d, iridix_chn_idx =%d\n", ctx_id, ptr->iridix_chn_idx);
                set_bit(i, &ptr->p_gfw->iridix_chn_bitmap);
                acamera_isp_iridix_context_no_write(ptr->settings.isp_base, i);
            }
		} else if (ctrl_val == WDR_MODE_NATIVE) {
			for (i = FIRMWARE_CONTEXT_NUMBER - 1; i >= 0; i--) {
				ptr_tmp = acamera_get_ctx_ptr(i);
				if (ptr_tmp && ptr_tmp->initialized == 1 &&
					ptr_tmp->isp_sensor_mode == WDR_MODE_LINEAR &&
					ptr_tmp->iridix_chn_idx != -1) {
					pr_debug("giver_id = %d, accpter_id = %d, iridix_no = %d\n",
							i, ctx_id, ptr_tmp->iridix_chn_idx);
					iridix_no = ptr_tmp->iridix_chn_idx;
					// take iridix off linear ctx
					ptr_tmp->iridix_chn_idx = -1;
					acamera_isp_top_bypass_iridix_write(ptr_tmp->settings.isp_base, 1);
					acamera_isp_iridix_enable_write(ptr_tmp->settings.isp_base, 0);
					// give iridix to pwl ctx
					ptr->iridix_chn_idx = iridix_no;
					acamera_isp_iridix_context_no_write(ptr->settings.isp_base, (uint8_t)iridix_no);
					acamera_isp_top_bypass_iridix_write(ptr->settings.isp_base, 0);
					acamera_isp_iridix_enable_write(ptr->settings.isp_base, 1);
					break;
				}
			}
		}
		ptr->isp_sensor_mode = ctrl_val;
	}
	mutex_unlock(&ptr->p_gfw->ctx_chg_lock);
    return ret;
}

