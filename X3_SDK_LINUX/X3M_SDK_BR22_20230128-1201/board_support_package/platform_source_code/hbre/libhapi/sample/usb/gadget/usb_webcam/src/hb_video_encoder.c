/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * hb_video_encoder.c
 *	hobot video encoder api functions
 *
 * Copyright (C) 2019 Horizon Robotics, Inc.
 *
 * Contact: jianghe xu<jianghe.xu@horizon.ai>
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "hb_video_encoder.h"
#include "hb_vio_interface.h"
#include "hb_vps_api.h"
#include "utils.h"

static void *pym_to_venc_thread(void *param)
{
	venc_context *vctx = (venc_context *)param;
	VIDEO_FRAME_S pstFrame;
	pym_buffer_t out_pym_buf;
	int pts = 0;
	int ret;

	trace_in();

	if (!vctx)
		return (void *)0;

	while (!vctx->quit) {
		ret = HB_VPS_GetChnFrame(0, 6, &out_pym_buf, 2000);
		if (ret != 0) {
			printf("HB_VPS_GetChnFrame error!!!ret 0x%x \n", ret);
			continue;
		}

		// 3840x2160
		memset(&pstFrame, 0, sizeof(VIDEO_FRAME_S));
		pstFrame.stVFrame.pix_format = HB_PIXEL_FORMAT_NV12;
		pstFrame.stVFrame.width = out_pym_buf.pym[0].width;
		pstFrame.stVFrame.height = out_pym_buf.pym[0].height;
		pstFrame.stVFrame.size = out_pym_buf.pym[0].width *
		    out_pym_buf.pym[0].height * 3 / 2;
		pstFrame.stVFrame.phy_ptr[0] = out_pym_buf.pym[0].paddr[0];
		pstFrame.stVFrame.phy_ptr[1] = out_pym_buf.pym[0].paddr[1];
		pstFrame.stVFrame.vir_ptr[0] = out_pym_buf.pym[0].addr[0];
		pstFrame.stVFrame.vir_ptr[1] = out_pym_buf.pym[0].addr[1];
		pstFrame.stVFrame.pts = pts;

		ret = HB_VENC_SendFrame(0, &pstFrame, 3000);
		if (ret < 0)
			printf("HB_VENC_SendFrame 0 error!!!ret 0x%x \n", ret);

		pts++;
		ret = HB_VPS_ReleaseChnFrame(0, 6, &out_pym_buf);
		if (ret) {
			printf("HB_VPS_ReleaseChnFrame error!!!\n");
			break;
		}
	}

	trace_out();

	return (void *)0;
}

static void *venc_to_uvc_thread(void *param)
{
	venc_context *vctx = (venc_context *)param;
	int r, venc_chn;

	trace_in();

	if (!vctx)
		return (void *)0;

	venc_chn = vctx->venc_chn;

	if (!vctx->mono_venc_q) {
		printf("mono_venc_q not ready...\n");
		return (void *)0;
	}

	venc_single_buffer *mono_venc_q = vctx->mono_venc_q;
	mono_venc_q->used = 0;
	memset(&mono_venc_q->vstream, 0, sizeof(VIDEO_STREAM_S));
	pthread_mutex_init(&mono_venc_q->mutex, NULL);

	while (!vctx->quit) {

		if (!mono_venc_q)
			break;

		/* check uvc single buffer */
		pthread_mutex_lock(&mono_venc_q->mutex);
		if (mono_venc_q->used) {
			pthread_mutex_unlock(&mono_venc_q->mutex);
			usleep(5*1000);		/* sleep 5ms and re-check mono_venc_q */
			continue;
		}

		r = HB_VENC_GetStream(venc_chn, &mono_venc_q->vstream, 300);
		if (r < 0) {
			pthread_mutex_unlock(&mono_venc_q->mutex);
			usleep(3*1000);		/* sleep 3ms and re-encode */
			continue;
		}

		mono_venc_q->used = 1;
		pthread_mutex_unlock(&mono_venc_q->mutex);
	}

	trace_out();

	return (void *)0;
}

static int sample_venc_setcrop(int venc_chn, int x, int y, int w, int h)
{
	int ret;
	VENC_JPEG_PARAM_S jpegparam;

	ret = HB_VENC_GetJpegParam(venc_chn, &jpegparam);
	if (ret != 0) {
		printf("HB_VENC_GetJpegParam failed.\n");
		return -1;
	}

	jpegparam.stCropCfg.bEnable = HB_TRUE;
	jpegparam.stCropCfg.stRect.s32X = x;
	jpegparam.stCropCfg.stRect.s32Y = y;
	jpegparam.stCropCfg.stRect.u32Width = w;
	jpegparam.stCropCfg.stRect.u32Height = h;

	ret = HB_VENC_SetJpegParam(venc_chn, &jpegparam);
	if (ret != 0) {
		printf("HB_VENC_GetJpegParam failed.\n");
		return -1;
	}

	return 0;
}

static int venc_channel_attr_init(VENC_CHN_ATTR_S *vch_attr,
		PAYLOAD_TYPE_E type, int width, int height,
		int crop_width, int crop_height,
		PIXEL_FORMAT_E pixel_format)
{
	trace_in();

	if (!vch_attr)
		return -1;

	vch_attr->stVencAttr.enType = type;

	vch_attr->stVencAttr.u32PicWidth = width;
	vch_attr->stVencAttr.u32PicHeight = height;

	vch_attr->stVencAttr.enMirrorFlip = DIRECTION_NONE;
	vch_attr->stVencAttr.enRotation = CODEC_ROTATION_0;
	vch_attr->stVencAttr.stCropCfg.bEnable = HB_TRUE;
	vch_attr->stVencAttr.stCropCfg.stRect.s32X = 0;
	vch_attr->stVencAttr.stCropCfg.stRect.s32Y = 0;
	vch_attr->stVencAttr.stCropCfg.stRect.u32Width = crop_width;
	vch_attr->stVencAttr.stCropCfg.stRect.u32Height = crop_height;
	vch_attr->stVencAttr.stAttrH264.h264_profile = 0;

	if (type == PT_JPEG || type == PT_MJPEG) {
		vch_attr->stVencAttr.enPixelFormat = pixel_format;
		vch_attr->stVencAttr.u32BitStreamBufferCount = 1;
		vch_attr->stVencAttr.u32FrameBufferCount = 2;
		vch_attr->stVencAttr.bExternalFreamBuffer = HB_TRUE;
		vch_attr->stVencAttr.stAttrJpeg.dcf_enable = HB_FALSE;
		vch_attr->stVencAttr.stAttrJpeg.quality_factor = 0;
		vch_attr->stVencAttr.stAttrJpeg.restart_interval = 0;

	} else {
		vch_attr->stVencAttr.enPixelFormat = pixel_format;
		vch_attr->stVencAttr.u32BitStreamBufferCount = 5;
		vch_attr->stVencAttr.u32FrameBufferCount = 5;
		vch_attr->stVencAttr.bExternalFreamBuffer = HB_TRUE;
	}

	if (type == PT_H265) {
		vch_attr->stRcAttr.enRcMode = VENC_RC_MODE_H265VBR;
		vch_attr->stRcAttr.stH265Vbr.bQpMapEnable = HB_TRUE;
		vch_attr->stRcAttr.stH265Vbr.u32IntraQp = 20;
		vch_attr->stRcAttr.stH265Vbr.u32IntraPeriod = 20;
		vch_attr->stRcAttr.stH265Vbr.u32FrameRate = 25;
	}

	if (type == PT_H264) {
		vch_attr->stRcAttr.enRcMode = VENC_RC_MODE_H264VBR;
		vch_attr->stRcAttr.stH264Vbr.bQpMapEnable = HB_TRUE;
		vch_attr->stRcAttr.stH264Vbr.u32IntraQp = 20;
		vch_attr->stRcAttr.stH264Vbr.u32IntraPeriod = 20;
		vch_attr->stRcAttr.stH264Vbr.u32FrameRate = 25;
	}

	vch_attr->stGopAttr.u32GopPresetIdx = 6;
	vch_attr->stGopAttr.s32DecodingRefreshType = 2;

	trace_out();

	return 0;
}

venc_context *hb_video_encoder_alloc_context()
{
	venc_context *vctx = calloc(1, sizeof(venc_context));

	if (!vctx) {
		printf("allocate venc_context failed\n");
		return NULL;
	}

	vctx->venc_chn = 0;
	vctx->type = VENC_MJPEG;	// mjpeg as default
	vctx->width = 1920;
	vctx->height = 1080;
	vctx->vps_grp = 0;
	vctx->vps_chn = 5;
	vctx->bitrate = 2000;
	vctx->quit = 0;

	vctx->mono_venc_q = malloc(sizeof(struct venc_single_buffer));
	if (!vctx->mono_venc_q) {
		free(vctx);
		return NULL;
	}

	return vctx;
}

int hb_video_encoder_init(venc_context *vctx)
{
	VENC_CHN_ATTR_S venc_chn_attr = {0, };
	VENC_RC_ATTR_S *venc_rc_attr;
	PAYLOAD_TYPE_E ptype;
	int need_crop = 0;
	int target_width, target_height;
	int ret;

	trace_in();

	if (!vctx)
		return -1;

	switch (vctx->type) {
	case VENC_H264:
		ptype = PT_H264;
		break;
	case VENC_H265:
		ptype = PT_H265;
		break;
	case VENC_MJPEG:
		ptype = PT_MJPEG;
		break;
	default:
		ptype = PT_JPEG;
		break;
	}

	ret = HB_VENC_Module_Init();
	if (ret)
		return -1;

	target_width = IS_ALIGNED(vctx->width, 8) ? vctx->width : ALIGN_UP(vctx->width, 8);
	target_height = IS_ALIGNED(vctx->height, 8) ? vctx->height: ALIGN_UP(vctx->height, 8);

	if (target_width != vctx->width || target_height != vctx->height) {
		printf("vctx->width or vctx->height not aligned. vctx->width(%d), vctx->height(%d), "
				"target_width(%d), target_height(%d)\n",
				vctx->width, vctx->height, target_width, target_height);
		need_crop = 1;
	}

	ret = venc_channel_attr_init(&venc_chn_attr, ptype,
			target_width, target_height, vctx->width, vctx->height,
			HB_PIXEL_FORMAT_NV12);
	if (ret) {
		printf("venc_channel_attr_init failed\n");
		return -1;
	}

	ret = HB_VENC_CreateChn(vctx->venc_chn, &venc_chn_attr);
	if (ret != 0) {
		printf("HB_VENC_CreateChn %d failed, %x.\n",
				vctx->venc_chn, ret);
		return -1;
	}

	if (ptype == PT_H264) {
		printf(">>>>venc h264 case\n");
		venc_rc_attr = &(venc_chn_attr.stRcAttr);
		venc_chn_attr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
		ret = HB_VENC_GetRcParam(vctx->venc_chn, venc_rc_attr);
		if (ret) {
			printf("HB_VENC_GetRcParam failed.\n");
			return -1;
		}

		venc_rc_attr->stH264Cbr.u32BitRate = vctx->bitrate;
		venc_rc_attr->stH264Cbr.u32FrameRate = 30;
		venc_rc_attr->stH264Cbr.u32IntraPeriod = 60;
		venc_rc_attr->stH264Cbr.u32VbvBufferSize = 3000;
	} else if (ptype == PT_H265) {
		printf(">>>>venc h265 case\n");
		venc_rc_attr = &(venc_chn_attr.stRcAttr);
		venc_chn_attr.stRcAttr.enRcMode = VENC_RC_MODE_H265CBR;
		ret = HB_VENC_GetRcParam(vctx->venc_chn, venc_rc_attr);
		if (ret) {
			printf("HB_VENC_GetRcParam failed.\n");
			return -1;
		}

		venc_rc_attr->stH265Cbr.u32BitRate = vctx->bitrate;
		venc_rc_attr->stH265Cbr.u32FrameRate = 30;
		venc_rc_attr->stH265Cbr.u32IntraPeriod = 30;
		venc_rc_attr->stH265Cbr.u32VbvBufferSize = 3000;
	} else if (ptype == PT_MJPEG) {
		printf(">>>>venc mjpeg case\n");
		venc_rc_attr = &(venc_chn_attr.stRcAttr);
		venc_chn_attr.stRcAttr.enRcMode = VENC_RC_MODE_MJPEGFIXQP;
		ret = HB_VENC_GetRcParam(vctx->venc_chn, venc_rc_attr);
		if (ret) {
			printf("HB_VENC_GetRcParam failed.\n");
			return -1;
		}

		/* frame rate and mjpeg quality factort.
		 * usb2.0 bandwidth couldn't handle mjpeg quality 100.
		 */
		venc_rc_attr->stMjpegFixQp.u32QualityFactort = 70;
		venc_rc_attr->stMjpegFixQp.u32FrameRate = 30;
		printf(">>>>set mjpeg framerate(30), QualityFactort(70)\n");
	}

	ret = HB_VENC_SetChnAttr(vctx->venc_chn, &venc_chn_attr);	// config
	if (ret != 0) {
		printf("HB_VENC_SetChnAttr failed\n");
		return -1;
	}

	/* set mjpeg crop property */
	if ((ptype == PT_MJPEG || ptype == PT_JPEG) && need_crop) {
		printf("do mjpeg crop operation. width(%d), height(%d)\n",
				vctx->width, vctx->height);
		sample_venc_setcrop(vctx->venc_chn,
				0, 0, vctx->width, vctx->height);
	}

	trace_out();

	return 0;
}

int hb_video_encoder_start(venc_context *vctx)
{
	int ret = 0;

	VENC_RECV_PIC_PARAM_S venc_rec_param;
	venc_rec_param.s32RecvPicNum = 0;	// unchangable

	trace_in();

	if (!vctx)
		return -1;

	ret = HB_VENC_StartRecvFrame(vctx->venc_chn, &venc_rec_param);
	if (ret != 0) {
		printf("HB_VENC_StartRecvFrame failed\n");
		return -1;
	}

	vctx->quit = 0;
	ret = pthread_create(&vctx->pym_to_venc_pid, NULL,
				pym_to_venc_thread, vctx);

	if (!ret && vctx->type > 0)
		ret = pthread_create(&vctx->venc_to_uvc_pid, NULL,
				venc_to_uvc_thread, vctx);

	trace_out();

	return ret;
}

int hb_video_encoder_stop(venc_context *vctx)
{
	int ret = 0;

	trace_in();

	if (!vctx)
		return -1;

	vctx->quit = 1;

	pthread_join(vctx->pym_to_venc_pid, NULL);
	pthread_join(vctx->venc_to_uvc_pid, NULL);

	printf("venc work thread join succeed...\n");

	ret = HB_VENC_StopRecvFrame(vctx->venc_chn);
	if (ret != 0) {
		printf("HB_VENC_StopRecvFrame failed\n");
		return -1;
	}

	trace_out();

	return 0;
}

void hb_video_encoder_deinit(venc_context *vctx)
{
	trace_in();

	if (!vctx)
		return;

	HB_VENC_DestroyChn(vctx->venc_chn);
	HB_VENC_Module_Uninit();

	if (vctx->mono_venc_q) {
		free(vctx->mono_venc_q);
		vctx->mono_venc_q = NULL;
	}

	if (vctx)
		free(vctx);

	trace_out();

	return;
}
