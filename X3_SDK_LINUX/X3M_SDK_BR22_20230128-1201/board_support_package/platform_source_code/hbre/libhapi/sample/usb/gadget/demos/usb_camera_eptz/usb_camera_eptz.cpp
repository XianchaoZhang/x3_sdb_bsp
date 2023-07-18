/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#include <thread>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>
#include <pthread.h>
#include <sys/stat.h>
#include <time.h>
#include <sys/time.h>
#include <signal.h>
#include <fcntl.h>
#include <math.h>
extern "C" {
#include "hb_vin_api.h"
#include "hb_vps_api.h"
#include "hb_mipi_api.h"
#include "hb_vio_interface.h"
#include "hb_sys.h"
}
#include "vio_venc.h"
#include "vio_vin.h"
#include "vio_vo.h"
#include "vio_vps.h"
#include "vio_sys.h"
#include "vio_cfg.h"
#include "utils/utils.h"
#include "uvc_gadget_api.h"
#define UVC_DUMP
#define DUMP_FILE "/userdata/uvc-dump.data"
#define EPTZ
#define QUEUE_LEN 2
#define VPS_GROUP_NUM 4
#define ROI_X_COORDINATE 1600
#define ROI_Y_COORDINATE 900
int pym_width = 3840;
int pym_height = 2160;
int crop_x_downscale = 0;
int crop_y_downscale = 0;
int pym_us_layer = 0;
int enabled_us_layer = 0;
int need_upscale = 0;
int ipu_to_pym = 0;
vio_cfg_t vio_cfg;
VPS_PYM_CHN_ATTR_S vps1_us_1080p_attr;
VPS_PYM_CHN_ATTR_S vps1_us_720p_attr;

extern int running;
static int uvc_stream_on = 0;
static int need_encoder = -1;
static int dump_fd = 0;
static std::mutex mutex_;

void signal_handler(int signal_number)
{
	running = 0;
}

static bool IsUvcStreamOn()
{
	std::lock_guard < std::mutex > lg(mutex_);
	if (uvc_stream_on == 0) {
		return false;
	} else {
		return true;
	}
}

static PAYLOAD_TYPE_E fcc_to_video_format(unsigned int fcc)
{
	PAYLOAD_TYPE_E format;
	switch (fcc) {
	case V4L2_PIX_FMT_NV12:
		format = PT_NV;
		break;
	case V4L2_PIX_FMT_MJPEG:
		format = PT_MJPEG;
		printf("format is MJPEG\n");
		break;
	case V4L2_PIX_FMT_H265:
		format = PT_H265;
		break;
	case V4L2_PIX_FMT_H264:
	default:
		format = PT_H264;
		printf("format is MJPEG\n");
	}

	return format;
}

int us_grp_attr_1080p[6][2] =
    { {1502, 846}, {1202, 676}, {962, 543}, {752, 424}, {602, 340}, {482,
								     272}
};

int us_grp_vps_chn_attr_init_1080p(VPS_PYM_CHN_ATTR_S * attr)
{
	memset(attr, 0x0, sizeof(VPS_PYM_CHN_ATTR_S));
	attr->frame_id = 0;
	attr->ds_uv_bypass = 0;
	attr->ds_layer_en = 5;
	attr->us_layer_en = 6;
	attr->us_uv_bypass = 0;
	attr->timeout = 2000;
	attr->frameDepth = 1;

	for (auto i = 0; i < 6; i++) {
		attr->us_info[i].factor =
		    vio_cfg.pym_cfg[0][5].us_info[i].factor;
		attr->us_info[i].roi_x =
		    ceil(vio_cfg.pym_cfg[0][5].us_info[i].roi_x / 2.0);
		attr->us_info[i].roi_y =
		    ceil(vio_cfg.pym_cfg[0][5].us_info[i].roi_y / 2.0);
		//    attr->us_info[i].roi_width = ceil(vio_cfg.pym_cfg[0][5].us_info[i].roi_width / ratio);
		//    attr->us_info[i].roi_height = ceil(vio_cfg.pym_cfg[0][5].us_info[i].roi_height / ratio);
		//    attr->us_info[i].roi_width = ((attr->us_info[i].roi_width >> 2) + 1) << 2;
		//    attr->us_info[i].roi_height = ((attr->us_info[i].roi_height >> 1) + 1) << 1;
		attr->us_info[i].roi_width = us_grp_attr_1080p[i][0];
		attr->us_info[i].roi_height = us_grp_attr_1080p[i][1];
		printf("vps1 us%d factor:%d x:%d y:%d w:%d h:%d\n", i,
		       attr->us_info[i].factor, attr->us_info[i].roi_x,
		       attr->us_info[i].roi_y, attr->us_info[i].roi_width,
		       attr->us_info[i].roi_height);
	}
}

int us_grp_attr_720p[6][2] =
    { {1002, 564}, {802, 452}, {642, 362}, {503, 284}, {402, 228}, {322, 182} };
int us_grp_vps_chn_attr_init_720p(VPS_PYM_CHN_ATTR_S * attr)
{
	memset(attr, 0x0, sizeof(VPS_PYM_CHN_ATTR_S));
	attr->frame_id = 0;
	attr->ds_uv_bypass = 0;
	attr->ds_layer_en = 5;
	attr->us_layer_en = 6;
	attr->us_uv_bypass = 0;
	attr->timeout = 2000;
	attr->frameDepth = 1;

	for (auto i = 0; i < 6; i++) {
		attr->us_info[i].factor =
		    vio_cfg.pym_cfg[0][5].us_info[i].factor;
		attr->us_info[i].roi_x =
		    ceil(vio_cfg.pym_cfg[0][5].us_info[i].roi_x / 3.0);
		attr->us_info[i].roi_y =
		    ceil(vio_cfg.pym_cfg[0][5].us_info[i].roi_y / 3.0);
		//    attr->us_info[i].roi_width = ceil(vio_cfg.pym_cfg[0][5].us_info[i].roi_width / ratio);
		//    attr->us_info[i].roi_height = ceil(vio_cfg.pym_cfg[0][5].us_info[i].roi_height / ratio);
		//    attr->us_info[i].roi_width = ((attr->us_info[i].roi_width >> 2) + 1) << 2;
		//    attr->us_info[i].roi_height = ((attr->us_info[i].roi_height >> 1) + 1) << 1;
		attr->us_info[i].roi_width = us_grp_attr_720p[i][0];
		attr->us_info[i].roi_height = us_grp_attr_720p[i][1];
		printf("vps1 us%d factor:%d x:%d y:%d w:%d h:%d\n", i,
		       attr->us_info[i].factor, attr->us_info[i].roi_x,
		       attr->us_info[i].roi_y, attr->us_info[i].roi_width,
		       attr->us_info[i].roi_height);
	}
}

int us_vps_grp_init(void)
{
	VPS_GRP_ATTR_S grp_attr;
	VPS_CHN_ATTR_S chn_attr;
	VPS_PYM_CHN_ATTR_S pym_chn_attr;
	int ret;

	memset(&grp_attr, 0, sizeof(VPS_GRP_ATTR_S));
	grp_attr.maxW = pym_width;
	grp_attr.maxH = pym_height;

	memset(&chn_attr, 0, sizeof(VPS_CHN_ATTR_S));
	chn_attr.enScale = 1;
	chn_attr.width = pym_width;
	chn_attr.height = pym_height;
	chn_attr.frameDepth = 0;

	memset(&pym_chn_attr, 0, sizeof(VPS_PYM_CHN_ATTR_S));
	if (pym_width == 1920) {
		memcpy(&pym_chn_attr, &vps1_us_1080p_attr,
		       sizeof(VPS_PYM_CHN_ATTR_S));
	} else {
		memcpy(&pym_chn_attr, &vps1_us_720p_attr,
		       sizeof(VPS_PYM_CHN_ATTR_S));
	}

	ret = HB_VPS_CreateGrp(1, &grp_attr);
	if (ret) {
		printf("HB_VPS_CreateGrp1 error!!!\n");
	}
	HB_SYS_SetVINVPSMode(1, VIN_OFFLINE_VPS_OFFINE);

	ret = HB_VPS_SetChnAttr(1, 5, &chn_attr);
	if (ret) {
		printf("HB_VPS_SetChnAttr error!!!\n");
	}

	ret = HB_VPS_EnableChn(1, 5);
	if (ret) {
		printf("HB_VPS_EnableChn error!!!\n");
	}

	ret = HB_VPS_SetPymChnAttr(1, 5, &pym_chn_attr);
	if (ret) {
		printf("HB_VPS_SetPymChnAttr grp1 chn5 error!!!\n");
	}
	HB_VPS_EnableChn(1, 5);

	for (auto i = 1; i < 6; i++) {
		ret = hb_vps_change_pym_us(1, i, 0);
		if (ret) {
			printf("hb_vps_change_pym_us grp1 layer%d enable%d\n",
			       i, 0);
		}
	}

	struct HB_SYS_MOD_S src_mod, dst_mod;
	src_mod.enModId = HB_ID_VPS;
	src_mod.s32DevId = 0;
	src_mod.s32ChnId = 2;
	dst_mod.enModId = HB_ID_VPS;
	dst_mod.s32DevId = 1;
	dst_mod.s32ChnId = 0;
	ret = HB_SYS_Bind(&src_mod, &dst_mod);
	if (ret) {
		printf("HB_SYS_Bind failed\n");
	}

	ret = HB_VPS_StartGrp(1);
	if (ret) {
		printf("HB_VPS_StartGrp error!!!\n");
	}

	return 0;
}

int us_vps_grp_deinit()
{
	int ret;

	struct HB_SYS_MOD_S src_mod, dst_mod;
	src_mod.enModId = HB_ID_VPS;
	src_mod.s32DevId = 0;
	src_mod.s32ChnId = 2;

	dst_mod.enModId = HB_ID_VPS;
	dst_mod.s32DevId = 1;
	dst_mod.s32ChnId = 0;
	ret = HB_SYS_UnBind(&src_mod, &dst_mod);
	if (ret) {
		printf("HB_SYS_UnBind vps 0/1 failed\n");
	}

	usleep(30000);

	ret = HB_VPS_StopGrp(1);
	if (ret) {
		printf("HB_VPS_StopGrp chn1 error!!!\n");
	}
	ret = HB_VPS_DestroyGrp(1);
	if (ret) {
		printf("HB_VPS_DestroyGrp chn1 error!!!\n");
	}
}

int us_vps_grp_update(int grp, int l, int t, int w, int h)
{
	VPS_CROP_INFO_S chn_crop_info;
	int ret, i;

	// printf("vps_upscale_update: l: %d t: %d w: %d h: %d\n", l, t, w, h);
	memset(&chn_crop_info, 0x0, sizeof(VPS_CROP_INFO_S));
	chn_crop_info.en = 1;
	chn_crop_info.cropRect.x = l;
	chn_crop_info.cropRect.y = t;
	chn_crop_info.cropRect.width = w;
	chn_crop_info.cropRect.height = h;

	ret = HB_VPS_SetChnCrop(grp, 5, &chn_crop_info);
	if (ret) {
		printf("dynamic set grp%d chn%d crop fail\n", i, 5);
	}
}

int ds_vps_grp_init()
{
	VPS_CHN_ATTR_S chn_attr;
	VPS_CROP_INFO_S chn_crop_info;
	int ret;

	memset(&chn_crop_info, 0x0, sizeof(VPS_CROP_INFO_S));
	chn_crop_info.en = 1;
	chn_crop_info.cropRect.width = 3840;
	chn_crop_info.cropRect.height = 2160;
	ret = HB_VPS_SetChnCrop(0, 2, &chn_crop_info);
	if (ret) {
		printf("HB_VPS_SetChnCrop chn2 error!!!\n");
	}

	memset(&chn_attr, 0, sizeof(VPS_CHN_ATTR_S));
	chn_attr.enScale = 1;
	chn_attr.width = pym_width;
	chn_attr.height = pym_height;
	chn_attr.frameDepth = 1;

	ret = HB_VPS_SetChnAttr(0, 2, &chn_attr);
	if (ret) {
		printf("HB_VPS_SetChnAttr chn2 error!!!\n");
	}
}

int ds_vps_grp_update(int l, int t, int w, int h)
{
	VPS_CROP_INFO_S chn_crop_info;
	int ret;

	// printf("downscale_update: l: %d t: %d w: %d h: %d\n", l, t, w, h);
	memset(&chn_crop_info, 0x0, sizeof(VPS_CROP_INFO_S));
	chn_crop_info.en = 1;
	chn_crop_info.cropRect.x = l;
	chn_crop_info.cropRect.y = t;
	chn_crop_info.cropRect.width = w;
	chn_crop_info.cropRect.height = h;
	ret = HB_VPS_SetChnCrop(0, 2, &chn_crop_info);
}

int camera_init(vio_cfg_t vio_cfg, IOT_VIN_ATTR_S * vin_attr)
{
	int ret;
	ret = hb_vin_init(0, 0, vio_cfg, vin_attr);
	if (ret) {
		printf("hb_vin_init failed, %d\n", ret);
		return -1;
	}
	ret = hb_vps_init(0, vio_cfg, vin_attr);
	if (ret) {
		hb_vin_deinit(0, vio_cfg);
		printf("hb_vps_init failed, %d\n", ret);
		return -1;
	}

	ret = hb_vin_bind_vps(0, 0, 0, vio_cfg);
	if (ret) {
		hb_vin_deinit(0, vio_cfg);
		printf("hb_vin_bind_vps failed, %d\n", ret);
		return -1;
	}

	ret = hb_vps_start(0);
	if (ret) {
		hb_vin_deinit(0, vio_cfg);
		printf("hb_vps_start failed, %d\n", ret);
		return -1;
	}

	for (auto i = 1; i < 6; i++) {
		ret = hb_vps_change_pym_us(0, i, 0);
		if (ret) {
			hb_vin_deinit(0, vio_cfg);
			printf("hb_vps_change_pym_us grp%d layer%d enable%d\n",
			       0, i, 0);
			return -1;
		}
	}

	us_grp_vps_chn_attr_init_1080p(&vps1_us_1080p_attr);
	us_grp_vps_chn_attr_init_720p(&vps1_us_720p_attr);
	ret = hb_vin_start(0, vio_cfg);
	if (ret) {
		printf("hb_vin_start failed, %d\n", ret);
		hb_vps_stop(0);
		hb_vin_deinit(0, vio_cfg);
		return -1;
	}
	return 0;
}

int camera_deinit(vio_cfg_t vio_cfg)
{
	hb_vin_stop(0, vio_cfg);
	hb_vps_stop(0);
	hb_venc_stop(0);
	hb_venc_deinit(0);
	HB_VENC_Module_Uninit();
	hb_vps_deinit(0);
	hb_vin_deinit(0, vio_cfg);
}

int VencInit(vencParam_t * param)
{
	auto ret = hb_venc_init(param);
	if (ret) {
		printf("hb_venc_init failed, %d\n", ret);
		return -1;
	}
	ret = hb_venc_start(0);
	if (ret) {
		printf("hb_venc_start failed, %d\n", ret);
		hb_venc_deinit(0);
		return -1;
	}

	return 0;
}

int VencDeinit()
{
	hb_venc_stop(0);
	hb_venc_deinit(0);
	return 0;
}

static void uvc_streamon_off(struct uvc_context *ctx, int is_on, void *userdata)
{
	struct uvc_device *dev;
	static vencParam_t vencparam;
	unsigned int fcc = 0;
	dev = ctx->udev;
	fcc = dev->fcc;

	printf("uvc_stream on/off value:%d\n");
	if (is_on) {
		if (dev->height == 2160) {
			pym_width = 3840;
			pym_height = 2160;
		} else if (dev->height == 1080) {
			pym_width = 1920;
			pym_height = 1080;
		} else if (dev->height == 720) {
			pym_width = 1280;
			pym_height = 720;
		} else {
			pym_width = 1280;
			pym_height = 720;
		}
		ds_vps_grp_init();
#ifdef EPTZ
		if (pym_width != 3840) {
			us_vps_grp_init();
		}
#endif
		if (fcc == V4L2_PIX_FMT_NV12) {
			std::lock_guard < std::mutex > lg(mutex_);
			vencparam.type = PT_NV;
			need_encoder = 0;
			uvc_stream_on = 1;
		} else {
			vencparam.width = pym_width;
			vencparam.height = pym_height;
			vencparam.veChn = 0;
			vencparam.vpsGrp = 0;
			vencparam.vpsChn = 6;
			vencparam.bitrate = 12000;
			vencparam.type = fcc_to_video_format(fcc);
			VencInit(&vencparam);
			std::lock_guard < std::mutex > lg(mutex_);
			need_encoder = 1;
			uvc_stream_on = 1;
		}
	} else {
		{
			std::lock_guard < std::mutex > lg(mutex_);
			uvc_stream_on = 0;
		}

		while (need_encoder != -1) {
			usleep(10000);
		}
		if (vencparam.type != PT_NV) {
			VencDeinit();
		}
#ifdef EPTZ
		if (pym_width != 3840) {
			us_vps_grp_deinit();
			enabled_us_layer = 0;
		}
#endif
		RingQueue < VIDEO_STREAM_S >::Instance().Clear();

#ifdef UVC_DUMP
		if (dump_fd > 0) {
			close(dump_fd);
			dump_fd = 0;
			printf("close uvc data dump file\n");
		}
#endif
	}
}

static int uvc_get_frame(struct uvc_context *ctx, void **buf_to,
			 int *buf_len, void **entity, void *userdata)
{
	if (!ctx || !ctx->udev) {
		printf("uvc_get_frame: input params is null\n");
		return -EINVAL;
	}
	if (IsUvcStreamOn() == 0) {
		printf("uvc_get_frame: stream is off\n");
		return -EFAULT;
	}

	VIDEO_STREAM_S *one_video =
	    (VIDEO_STREAM_S *) calloc(1, sizeof(VIDEO_STREAM_S));
	if (!one_video) {
		return -EINVAL;
	}

	if (!RingQueue < VIDEO_STREAM_S >::Instance().Pop(*one_video)) {
		return -EINVAL;
	}

	*buf_to = one_video->pstPack.vir_ptr;
	*buf_len = one_video->pstPack.size;
	*entity = one_video;

#ifdef UVC_DUMP
	if (!dump_fd && access("/tmp/uvc_dump_enable", F_OK) == 0) {
		/* create dump file in /userdata */
		dump_fd = open(DUMP_FILE, O_CREAT|O_RDWR, 0666);
		if (dump_fd < 0)
			fprintf(stderr, "open %s failed, errno(%d - %m)\n",
					DUMP_FILE, errno);

		printf("open %s for uvc data dump\n", DUMP_FILE);
	}

	if (dump_fd > 0 && *buf_to && *buf_len &&
			access("/tmp/uvc_dump_enable", F_OK) == 0) {
		if (write(dump_fd, *buf_to, *buf_len) < 0)
			fprintf(stderr, "uvc dump data failed. "
					"buf_to(%p), buf_len(%d)\n",
					*buf_to, *buf_len);
	}
#endif

	return 0;
}

static void uvc_release_frame(struct uvc_context *ctx, void **entity,
			      void *userdata)
{

	if (!ctx || !entity || !(*entity))
		return;
	auto video_buffer = static_cast < VIDEO_STREAM_S * >(*entity);
	if (video_buffer->pstPack.vir_ptr) {
		free(video_buffer->pstPack.vir_ptr);
	}

	free(video_buffer);
	*entity = nullptr;
}

int uvc_init(uvc_context * uvc_ctx, int bulk_mode)
{
	struct uvc_params params;
	char *uvc_devname = NULL;	/* uvc Null, lib will find one */
	char *v4l2_devname = NULL;	/* v4l2 Null is Null... */
	int ret;

	/* init uvc user params, just use default params and overlay
	 * some specify params.
	 */
	uvc_gadget_user_params_init(&params);

	if (bulk_mode) {
		/* bulk mode */
		params.bulk_mode = 1;
		params.h264_quirk = 0;
		params.burst = 9;
	} else {
		/* isoc mode */
		params.bulk_mode = 0;
		params.h264_quirk = 0;
		// params.burst = 10;
		params.mult = 2;
	}

	ret = uvc_gadget_init(&uvc_ctx, uvc_devname, v4l2_devname, &params);
	if (ret < 0) {
		printf("uvc_gadget_init error!\n");
		return ret;
	}

	uvc_set_streamon_handler(uvc_ctx, uvc_streamon_off, nullptr);
	/* prepare/release buffer with video queue */
	uvc_set_prepare_data_handler(uvc_ctx, uvc_get_frame, nullptr);
	uvc_set_release_data_handler(uvc_ctx, uvc_release_frame, nullptr);

	ret = uvc_gadget_start(uvc_ctx);
	if (ret < 0) {
		printf("uvc_gadget_start error!\n");
	}

	return ret;
}

void upscale_raito_calc(int ratio)
{
	if (ratio < 10 || ratio > 60) {
		printf("invalid us ratio %f\n", ratio);
		return;
	}

	if (ratio < 16) {
		pym_us_layer = 0;
	} else if (ratio < 24) {
		pym_us_layer = 1;
	} else if (ratio < 30) {
		pym_us_layer = 2;
	} else if (ratio < 38) {
		pym_us_layer = 3;
	} else if (ratio < 48) {
		pym_us_layer = 4;
	} else {
		pym_us_layer = 5;
	}
}

#ifdef EPTZ
float pym_us_ratio[6] = { 1.28, 1.6, 2, 2.56, 3.2, 4 };

int roi_region_calc()
{
	static int hold_time = 0;
	static int upscale_ratio_max;
	static int upscale_ratio = 13;
	pym_buffer_t out_pym_buf;
	hb_vio_buffer_t out_buf;
	int ret;

	if (need_upscale == 1) {
		float ipu_us_ratio;

		if (upscale_ratio >= upscale_ratio_max) {
			upscale_ratio = upscale_ratio_max;
			if (hold_time == 100) {
				need_upscale = 0;
				crop_x_downscale = 0;
				crop_y_downscale = 0;
				hold_time = 0;
			} else {
				hold_time++;
			}
		}
		if (ipu_to_pym == 1) {
			ret =
			    HB_VPS_GetChnFrame_Cond(1, 5, &out_pym_buf, 2000,
						    0);
			if (ret == 0)
				HB_VPS_ReleaseChnFrame(1, 5, &out_pym_buf);
			ipu_to_pym = 0;
		}

		if (upscale_ratio < 16) {
			pym_us_layer = 0;
		} else if (upscale_ratio < 24) {
			pym_us_layer = 1;
		} else if (upscale_ratio < 30) {
			pym_us_layer = 2;
		} else if (upscale_ratio < 38) {
			pym_us_layer = 3;
		} else if (upscale_ratio < 48) {
			pym_us_layer = 4;
		} else {
			pym_us_layer = 5;
		}
		ipu_us_ratio =
		    (float)upscale_ratio / pym_us_ratio[pym_us_layer] / 10;

		int w = (int)((float)pym_width / ipu_us_ratio);
		int h = (int)((float)pym_height / ipu_us_ratio);
		int w_align = ((w >> 2) + 1) << 2;
		int h_align = ((h >> 1) + 1) << 1;
		w_align = w_align > pym_width ? pym_width : w_align;
		h_align = h_align > pym_height ? pym_height : h_align;
		auto x = (pym_width - w_align) / 2;
		auto y = (pym_height - h_align) / 2;
		// printf("a: %d b: %d c: %f x: %d y: %d w: %d h: %d\n",
		// pym_us_layer, upscale_ratio, ipu_us_ratio, x, y, w_align, h_align);
		if (pym_width == 3840) {
			us_vps_grp_update(0, x, y, w_align, h_align);
		} else {
			us_vps_grp_update(1, x, y, w_align, h_align);
		}
		upscale_ratio += 1;
		return 1;
	} else {
		crop_x_downscale += 48;	//32;
		crop_y_downscale += 27;	//18;
		if (pym_width >= 3840 - crop_x_downscale * 2) {
			crop_x_downscale = (3840 - pym_width) / 2;
			crop_y_downscale = (2160 - pym_height) / 2;
			auto crop_x_tgt_upscale =
			    ROI_X_COORDINATE - crop_x_downscale;
			// auto crop_y_tgt_upscale = ROI_Y_COORDINATE - crop_y_downscale;
			auto crop_w_tgt_upscale =
			    pym_width - crop_x_tgt_upscale * 2;
			// auto crop_h_tgt_upscale = pym_height - crop_y_tgt_upscale * 2;
			upscale_ratio_max = pym_width / crop_w_tgt_upscale * 10;
			if (upscale_ratio_max > 60) {
				upscale_ratio_max = 60;
				printf("max scale ratio is 6\n");
			}
			upscale_ratio = 13;
			need_upscale = 1;
			pym_us_layer = -1;
			ipu_to_pym = 1;

			if (pym_width == 1920) {
				auto W = 1920 * 128 / 100;
				W = ((W >> 2) + 1) << 2;
				auto H = 1080 * 128 / 100;
				H = ((H >> 1) + 1) << 1;
				auto crop_X_downscale = (3840 - W) / 2;
				auto crop_Y_downscale = (2160 - H) / 2;
				ds_vps_grp_update(crop_X_downscale,
						  crop_Y_downscale, W, H);
			} else if (pym_width == 1280) {
				auto W = 1280 * 128 / 100;
				W = ((W >> 2) + 1) << 2;
				auto H = 720 * 128 / 100;
				H = ((H >> 1) + 1) << 1;
				auto crop_X_downscale = (3840 - W) / 2;
				auto crop_Y_downscale = (2160 - H) / 2;
				ds_vps_grp_update(crop_X_downscale,
						  crop_Y_downscale, W, H);
			}
			return 0;
		} else {
			if (crop_x_downscale > ROI_X_COORDINATE ||
			    crop_y_downscale > ROI_Y_COORDINATE) {
				crop_x_downscale = ROI_X_COORDINATE;
				crop_y_downscale = ROI_Y_COORDINATE;
				if (hold_time == 100) {
					crop_x_downscale = 0;
					crop_y_downscale = 0;
					hold_time = 0;
				} else {
					hold_time++;
				}
			}
		}
		auto w = 3840 - 2 * crop_x_downscale;
		auto h = 2160 - 2 * crop_y_downscale;
		ds_vps_grp_update(crop_x_downscale, crop_y_downscale, w, h);
		return 0;
	}
}
#endif

void get_frame_thread(void *param)
{
	VIDEO_FRAME_S pstFrame;
	VIDEO_STREAM_S pstStream;
	struct timeval ts0, ts1;
	int pts = 0, idx = 0;
	time_t start, last = time(NULL);
	int vps_grp;
	int vps_chn;
	int ret;
	int zoom_in = 0;

	auto *pvio_image =
	    reinterpret_cast <
	    pym_buffer_t * >(std::calloc(1, sizeof(pym_buffer_t)));
	while (running) {
		memset(&pstFrame, 0, sizeof(VIDEO_FRAME_S));
		memset(&pstStream, 0, sizeof(VIDEO_STREAM_S));
		if (!uvc_stream_on) {
			usleep(10000);
			need_upscale = 0;
			crop_x_downscale = 0;
			crop_y_downscale = 0;
			continue;
		}
#ifdef EPTZ
		zoom_in = roi_region_calc();
		if (zoom_in) {
			if (pym_width == 3840) {
				vps_grp = 0;
			} else {
				vps_grp = 1;
				if (pym_us_layer != enabled_us_layer) {
					hb_vps_change_pym_us(vps_grp,
							     enabled_us_layer,
							     0);
					hb_vps_change_pym_us(vps_grp,
							     pym_us_layer, 1);
					enabled_us_layer = pym_us_layer;
				}
			}
			vps_chn = 5;
		} else {
			vps_grp = 0;
			vps_chn = 2;
		}
		auto ret = HB_VPS_GetChnFrame_Cond(vps_grp, vps_chn,
						   reinterpret_cast <
						   void *>(pvio_image), 2000,
						   0);
		if (ipu_to_pym)
			goto vps_release;
#else
		vps_grp = 0;
		vps_chn = 2;
		ret = HB_VPS_GetChnFrame(vps_grp, vps_chn,
					 reinterpret_cast < void *>(pvio_image),
					 2000);
#endif
		/*skip the last ipu frame when ipu to pym */
		if (ret != 0) {
			usleep(10000);
			continue;
		}
#ifdef EPTZ
		if (zoom_in) {
			pstFrame.stVFrame.vir_ptr[0] =
			    pvio_image->us[enabled_us_layer].addr[0];
			pstFrame.stVFrame.vir_ptr[1] =
			    pvio_image->us[enabled_us_layer].addr[1];
			pstFrame.stVFrame.phy_ptr[0] =
			    pvio_image->us[enabled_us_layer].paddr[0];
			pstFrame.stVFrame.phy_ptr[1] =
			    pvio_image->us[enabled_us_layer].paddr[1];
			if (pym_us_layer != enabled_us_layer
			    && pym_width == 3840) {
				hb_vps_change_pym_us(0, enabled_us_layer, 0);
				hb_vps_change_pym_us(0, pym_us_layer, 1);
				enabled_us_layer = pym_us_layer;
			}
		} else {
			pstFrame.stVFrame.vir_ptr[0] =
			    pvio_image->pym[0].addr[0];
			pstFrame.stVFrame.vir_ptr[1] =
			    pvio_image->pym[0].addr[1];
			pstFrame.stVFrame.phy_ptr[0] =
			    pvio_image->pym[0].paddr[0];
			pstFrame.stVFrame.phy_ptr[1] =
			    pvio_image->pym[0].paddr[1];
			if (enabled_us_layer != 0 && pym_width != 3840) {
				hb_vps_change_pym_us(1, enabled_us_layer, 0);
				hb_vps_change_pym_us(1, 0, 1);
				enabled_us_layer = 0;
			}
		}
#else
		pstFrame.stVFrame.vir_ptr[0] = pvio_image->pym[0].addr[0];
		pstFrame.stVFrame.vir_ptr[1] = pvio_image->pym[0].addr[1];
		pstFrame.stVFrame.phy_ptr[0] = pvio_image->pym[0].paddr[0];
		pstFrame.stVFrame.phy_ptr[1] = pvio_image->pym[0].paddr[1];
#endif
		{
			std::lock_guard < std::mutex > lg(mutex_);
			if (!uvc_stream_on) {
				HB_VPS_ReleaseChnFrame(vps_grp, vps_chn,
						       reinterpret_cast <
						       void *>(pvio_image));
				need_encoder = -1;
				continue;
			}
		}

		pstFrame.stVFrame.width = pym_width;
		pstFrame.stVFrame.height = pym_height;
		pstFrame.stVFrame.size = pym_width * pym_height * 3 / 2;
		pstFrame.stVFrame.pix_format = HB_PIXEL_FORMAT_NV12;

		if (RingQueue < VIDEO_STREAM_S >::Instance().IsValid()) {
			if (!need_encoder) {
				auto buffer_size = pstFrame.stVFrame.size;
				pstStream.pstPack.vir_ptr =
				    (char *)calloc(1, buffer_size);
				pstStream.pstPack.size = buffer_size;

				memcpy(pstStream.pstPack.vir_ptr,
				       pstFrame.stVFrame.vir_ptr[0],
				       pstFrame.stVFrame.height *
				       pstFrame.stVFrame.width);
				memcpy(pstStream.pstPack.vir_ptr +
				       pstFrame.stVFrame.height *
				       pstFrame.stVFrame.width,
				       pstFrame.stVFrame.vir_ptr[1],
				       pstFrame.stVFrame.height *
				       pstFrame.stVFrame.width / 2);

				RingQueue <
				    VIDEO_STREAM_S >::
				    Instance().Push(pstStream);
			} else {
				HB_VENC_SendFrame(0, &pstFrame, 0);
				ret = HB_VENC_GetStream(0, &pstStream, 2000);
				if (ret) {
					HB_VPS_ReleaseChnFrame(vps_grp, vps_chn,
							       reinterpret_cast
							       < void
							       *>(pvio_image));
					printf("HB_VENC_GetStream failed\n");
					continue;
				}
				auto video_buffer = pstStream;
				auto buffer_size = video_buffer.pstPack.size;
				video_buffer.pstPack.vir_ptr =
				    (char *)calloc(1, buffer_size);
				if (video_buffer.pstPack.vir_ptr) {
					memcpy(video_buffer.pstPack.vir_ptr,
					       pstStream.pstPack.vir_ptr,
					       buffer_size);
					RingQueue <
					    VIDEO_STREAM_S >::
					    Instance().Push(video_buffer);
				}
				HB_VENC_ReleaseStream(0, &pstStream);
			}
			gettimeofday(&ts1, NULL);
			start = time(NULL);
			idx++;
			if (start > last) {
				gettimeofday(&ts1, NULL);
				printf("[%d.%06d]venc chn %d fps %d\n",
				       ts1.tv_sec, ts1.tv_usec, 0, idx);
				last = start;
				idx = 0;
			}
		}
vps_release:
		HB_VPS_ReleaseChnFrame(vps_grp, vps_chn,
				       reinterpret_cast < void *>(pvio_image));
	}
	RingQueue < VIDEO_STREAM_S >::Instance().Exit();
	free(pvio_image);
}

static void usage(const char *argv0)
{
	fprintf(stderr, "Usage: %s [options]\n", argv0);
	fprintf(stderr, "Available options are\n");
	fprintf(stderr, " -b		Use bulk mode\n");
}

int main(int argc, char *argv[])
{
	struct uvc_context *uvc_ctx = NULL;
	struct uvc_params params;
	IOT_VIN_ATTR_S vin_attr;
	std::thread * t;
	int ret, opt;
	int bulk_mode = 0;	// isoc mode as default

	while ((opt = getopt(argc, argv, "hb")) != -1) {
		switch (opt) {
		case 'b':
			bulk_mode = 1;
			break;

		case 'h':
			usage(argv[0]);
			return -1;

		default:
			printf("Invalid option '-%c'\n", opt);
			usage(argv[0]);
			return -1;
		}
	}

	signal(SIGINT, signal_handler);

	std::string cfg_path("./config/vin_vps_config_usb_cam.json");
	auto config = std::make_shared < VioConfig > (cfg_path);
	if (!config || !config->LoadConfig()) {
		printf("falied to load config file: %s\n", cfg_path);
		return -1;
	}
	config->ParserConfig();
	vio_cfg = config->GetConfig();

	ret = camera_init(vio_cfg, &vin_attr);
	if (ret) {
		printf("camera_init failed\n");
		return -1;
	}

	RingQueue < VIDEO_STREAM_S >::Instance().Init(2,
						      [](VIDEO_STREAM_S & elem)
						      {
						      if (elem.pstPack.vir_ptr) {
						      free(elem.
							   pstPack.vir_ptr);
						      elem.pstPack.vir_ptr =
						      nullptr;}
						      }
	) ;
	HB_VENC_Module_Init();
	ret = uvc_init(uvc_ctx, bulk_mode);
	if (ret) {
		printf("uvc_init failed\n");
		goto error;
	}

	running = 1;
	t = new std::thread(get_frame_thread, nullptr);
	while (running) {
		sleep(2);
		printf("runing\n");
	}
	if (uvc_gadget_stop(uvc_ctx)) {
		printf("uvc_gadget_stop failed\n");
	}

	t->join();
	uvc_gadget_deinit(uvc_ctx);
	camera_deinit(vio_cfg);
	HB_VENC_Module_Uninit();
	return 0;

error:
	camera_deinit(vio_cfg);
	HB_VENC_Module_Uninit();
	return -1;
}
