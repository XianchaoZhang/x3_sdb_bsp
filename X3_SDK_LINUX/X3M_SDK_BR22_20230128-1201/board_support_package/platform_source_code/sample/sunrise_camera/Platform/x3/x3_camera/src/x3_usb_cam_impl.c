#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <signal.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>

#include "communicate/sdk_common_cmd.h"
#include "communicate/sdk_common_struct.h"
#include "communicate/sdk_communicate.h"

#include "utils/utils_log.h"
#include "utils/cqueue.h"
#include "utils/common_utils.h"
#include "utils/stream_define.h"
#include "utils/stream_manager.h"
#include "utils/mthread.h"

#include "guvc/uvc_gadget_api.h"
#include "guvc/uvc_gadget.h"

#include "x3_vio_venc.h"
#include "x3_vio_vin.h"
#include "x3_vio_vps.h"
#include "x3_vio_bind.h"
#include "x3_sdk_wrap.h"
#include "x3_vio_vdec.h"
#include "x3_vio_vot.h"
#include "x3_vio_vp.h"
#include "x3_vio_rgn.h"
#include "x3_bpu.h"
#include "x3_utils.h"
#include "x3_config.h"

#include "camera_handle.h"
#include "x3_preparam.h"
#include "x3_usb_cam_impl.h"

// 4K 直接通过VPS输出分辨率列表：
// 3840x2160 2560x1620 1920x1080 1280x720 960x540 640x360

// 1080P 直接通过VPS输出分辨率列表：
// 1920x1080 1280x720 960x540 640x360

// PTZ 功能设置 CROP roi 区域后通过bpu rezise放大处理

typedef struct {
	x3_modules_info_t* 	m_infos; // X3 各媒体模块配置
	tsThread		m_rgn_thread;
	tsThread		m_vot_thread;

	struct uvc_context	*uvc_ctx;

	int			m_dst_width; // 输出给usb host的图像宽度
	int			m_dst_height; // 输出给usb host的图像高度
	int			m_dst_stride; // 输出给usb host的图像的字节宽度

	int			m_payload_type; // 输出给usb host的数据格式，NV12、h264、mjpeg
	int			m_venc_chn; // 如果是h264、mjpeg编码数据，使用到的编码通道
	int			m_venc_width; // 编码器有宽高输出的字节对齐需求，H264 8字节对齐，mjpeg
	int			m_venc_height;

	int			m_ptz_test_enable; // 测试 vps crop 左上角一个小图，再放大到usb host要求的目标图像
} x3_usb_cam_t;

static x3_usb_cam_t g_x3_usb_cam;
static x3_modules_info_t g_x3_modules_info;

// hid 模式或者usb 虚拟网络模式发送算法结果给主机
static int send_bpu_result_to_usb_host(char *result) {
	printf("%s\n", result);
	return 0;
}

#define ALIGN_8(v) ((v + (8 - 1)) / 8 * 8)

static int x3_uvc_venc_init(x3_usb_cam_t *x3_usb_cam)
{
	int ret = 0;
	PIXEL_FORMAT_E pixFmt = HB_PIXEL_FORMAT_NV12;
	VENC_RC_ATTR_S *pstRcParam;
	VENC_CHN_ATTR_S vencChnAttr;

	int payload_type = x3_usb_cam->m_payload_type;
	int width = x3_usb_cam->m_dst_width;
	int height = x3_usb_cam->m_dst_height;
	int stride = x3_usb_cam->m_dst_stride;
	VENC_CHN venc_chn = x3_usb_cam->m_venc_chn;

	height = ALIGN_8(height);

	// 处理输入对齐问题
	/*if (payload_type == PT_H264) {*/

	LOGI_print("width:%d height:%d payload_type:%d", width, height, payload_type);

	memset(&vencChnAttr, 0, sizeof(VENC_CHN_ATTR_S));
	vencChnAttr.stVencAttr.enType = payload_type;
	vencChnAttr.stVencAttr.u32PicWidth = width;
	vencChnAttr.stVencAttr.u32PicHeight = height;
	vencChnAttr.stVencAttr.enMirrorFlip = DIRECTION_NONE;
	vencChnAttr.stVencAttr.enRotation = CODEC_ROTATION_0;
	vencChnAttr.stVencAttr.stCropCfg.bEnable = HB_FALSE;
	vencChnAttr.stVencAttr.bEnableUserPts = HB_TRUE;
	vencChnAttr.stVencAttr.s32BufJoint = 0;
	vencChnAttr.stVencAttr.s32BufJointSize = 8000000;
	vencChnAttr.stVencAttr.enPixelFormat = pixFmt;
	vencChnAttr.stVencAttr.u32BitStreamBufferCount = 3;
	vencChnAttr.stVencAttr.u32FrameBufferCount = 3;
	vencChnAttr.stGopAttr.u32GopPresetIdx = 6;
	vencChnAttr.stGopAttr.s32DecodingRefreshType = 2;

	int size = width * height;
	int streambuf = size & 0xfffff000;
	if (size > 2688 * 1522) {
		vencChnAttr.stVencAttr.vlc_buf_size = 7900*1024;
	} else if (size > 1920 * 1080) {
		vencChnAttr.stVencAttr.vlc_buf_size = 4*1024*1024;
	} else if (size > 1280 * 720) {
		vencChnAttr.stVencAttr.vlc_buf_size = 2100*1024;
	} else if (size > 704 * 576) {
		vencChnAttr.stVencAttr.vlc_buf_size = 2100*1024;
	} else {
		vencChnAttr.stVencAttr.vlc_buf_size = 2048*1024;
	}

	if (payload_type == PT_MJPEG) {
		if (x3_usb_cam->m_dst_stride) {
			vencChnAttr.stVencAttr.u32PicWidth = x3_usb_cam->m_dst_stride;
		}
		vencChnAttr.stVencAttr.bExternalFreamBuffer = HB_TRUE;
		vencChnAttr.stVencAttr.stAttrJpeg.dcf_enable = HB_FALSE;
		vencChnAttr.stVencAttr.stAttrJpeg.quality_factor = 0;
		vencChnAttr.stVencAttr.stAttrJpeg.restart_interval = 0;
		vencChnAttr.stVencAttr.u32BitStreamBufSize = streambuf;
		vencChnAttr.stVencAttr.stCropCfg.bEnable = HB_TRUE;
		vencChnAttr.stVencAttr.stCropCfg.stRect.s32X = 0;
		vencChnAttr.stVencAttr.stCropCfg.stRect.s32Y = 0;
		vencChnAttr.stVencAttr.stCropCfg.stRect.u32Width = width;
		vencChnAttr.stVencAttr.stCropCfg.stRect.u32Height = height;
	} else if (payload_type == PT_H264) {
		vencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
		vencChnAttr.stVencAttr.bExternalFreamBuffer = HB_TRUE;
		vencChnAttr.stRcAttr.stH264Vbr.bQpMapEnable = HB_TRUE;
		vencChnAttr.stRcAttr.stH264Vbr.u32IntraQp = 20;
		vencChnAttr.stRcAttr.stH264Vbr.u32IntraPeriod = 20;
		vencChnAttr.stRcAttr.stH264Vbr.u32FrameRate = 30;
		vencChnAttr.stVencAttr.stAttrH264.h264_profile = HB_H264_PROFILE_UNSPECIFIED;
		vencChnAttr.stVencAttr.stAttrH264.h264_level = HB_H264_LEVEL_UNSPECIFIED;
	}

	ret = HB_VENC_CreateChn(venc_chn, &vencChnAttr);
	if (ret != 0) {
		printf("HB_VENC_CreateChn %d failed, %d.\n", 0, ret);
		return -1;
	}

	pstRcParam = &(vencChnAttr.stRcAttr);
	if (payload_type == PT_MJPEG) {
		vencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_MJPEGFIXQP;
		ret = HB_VENC_GetRcParam(venc_chn, pstRcParam);
		if (ret != 0) {
			printf("HB_VENC_GetRcParam failed.\n");
			return -1;
		}
		pstRcParam->stMjpegFixQp.u32FrameRate = 30;
		pstRcParam->stMjpegFixQp.u32QualityFactort = 70;
	} else if (payload_type == PT_H264) {
		vencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
		ret = HB_VENC_GetRcParam(venc_chn, pstRcParam);
		if (ret != 0) {
			printf("HB_VENC_GetRcParam failed.\n");
			return -1;
		}
		pstRcParam->stH264Cbr.u32BitRate = 12000;
		pstRcParam->stH264Cbr.u32FrameRate = 30;
		pstRcParam->stH264Cbr.u32IntraPeriod = 60;
		pstRcParam->stH264Cbr.u32VbvBufferSize = 3000;
		pstRcParam->stH264Cbr.u32IntraQp = 30;
		pstRcParam->stH264Cbr.u32InitialRcQp = 30;
		pstRcParam->stH264Cbr.bMbLevelRcEnable = HB_FALSE;
		pstRcParam->stH264Cbr.u32MaxIQp = 51;
		pstRcParam->stH264Cbr.u32MinIQp = 10;
		pstRcParam->stH264Cbr.u32MaxPQp = 51;
		pstRcParam->stH264Cbr.u32MinPQp = 10;
		pstRcParam->stH264Cbr.u32MaxBQp = 51;
		pstRcParam->stH264Cbr.u32MinBQp = 10;
		pstRcParam->stH264Cbr.bHvsQpEnable = HB_FALSE;
		pstRcParam->stH264Cbr.s32HvsQpScale = 2;
		pstRcParam->stH264Cbr.u32MaxDeltaQp = 3;
		pstRcParam->stH264Cbr.bQpMapEnable = HB_FALSE;
	}

	ret = HB_VENC_SetChnAttr(venc_chn, &vencChnAttr); // config
	if (ret != 0) {
		printf("HB_VENC_SetChnAttr failed\n");
		return -1;
	}

	VENC_RECV_PIC_PARAM_S pstRecvParam;
	pstRecvParam.s32RecvPicNum = 0;  // unchangable

	ret = HB_VENC_StartRecvFrame(venc_chn, &pstRecvParam);
	if (ret != 0) {
		printf("HB_VENC_StartRecvFrame failed\n");
		return -1;
	}

	/*print_file("/sys/kernel/debug/vpu/venc");*/
	return ret;
}

static int x3_uvc_venc_uninit(x3_usb_cam_t* x3_usb_cam)
{
	int ret = 0;
	ret = HB_VENC_StopRecvFrame(x3_usb_cam->m_venc_chn);
	if (ret != 0) {
		printf("HB_VENC_StopRecvFrame failed\n");
		return -1;
	}
	ret = HB_VENC_DestroyChn(x3_usb_cam->m_venc_chn);
	if (ret != 0) {
		printf("HB_VENC_DestroyChn failed\n");
		return -1;
	}

	return ret;
}

static int x3_uvc_vps_set_chn_attr(int vps_grp_id, int vps_chn_id, int width, int height)
{
	int ret = 0;
	VPS_CHN_ATTR_S chn_attr;

	memset(&chn_attr, 0, sizeof(VPS_CHN_ATTR_S));
	ret = HB_VPS_GetChnAttr(vps_grp_id, vps_chn_id, &chn_attr);
	if (ret) {
		LOGE_print("HB_VPS_GetChnAttr error, ret:%d\n", ret);
	}
	chn_attr.width = width;
	chn_attr.height = height;

	ret = HB_VPS_SetChnAttr(vps_grp_id, vps_chn_id, &chn_attr);
	if (ret) {
		LOGE_print("HB_VPS_SetChnAttr error, ret:%d\n", ret);
	}

	print_vps_chn_attr(&chn_attr);
	print_file("/sys/devices/platform/soc/a4040000.ipu/info/pipeline0_info");
	return ret;
}

static int x3_uvc_vps_set_crop(int vps_grp_id, int vps_chn_id, int x1, int y1, int x2, int y2)
{
	int ret = 0;
	VPS_CROP_INFO_S cropInfo;

	memset(&cropInfo, 0, sizeof(VPS_CROP_INFO_S));
	ret = HB_VPS_GetChnCrop(vps_grp_id, vps_chn_id, &cropInfo);
	if (ret) {
		LOGE_print("HB_VPS_GetChnCrop error, ret:%d\n", ret);
	}
	cropInfo.en = 1;
	cropInfo.cropRect.x = x1;
	cropInfo.cropRect.y = y1;
	cropInfo.cropRect.width = x2 - x1;
	cropInfo.cropRect.height = y2 - y1;

	ret = HB_VPS_SetChnCrop(vps_grp_id, vps_chn_id, &cropInfo);
	if (ret) {
		LOGE_print("HB_VPS_SetChnCrop error, ret:%d\n", ret);
	}

	return ret;
}


static PAYLOAD_TYPE_E fcc_to_video_format(unsigned int fcc) {
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
		printf("format is H264\n");
	}

	return format;
}

static void uvc_streamon_off(struct uvc_context *ctx, int is_on,
		void *userdata) {
	struct uvc_device *dev;
	unsigned int fcc = 0;
	x3_usb_cam_t *x3_usb_cam = (x3_usb_cam_t *)userdata;

	dev = ctx->udev;
	fcc = dev->fcc;

	int vps_grp_id = x3_usb_cam->m_infos->m_vps_infos.m_vps_info[0].m_vps_grp_id;
	int vps_chn_id = x3_usb_cam->m_infos->m_vps_infos.m_vps_info[0].m_vps_chn_attrs[0].m_chn_id;

	printf("uvc_stream on/off value:%d\n", is_on);

	// 不支持在原始图像上再放大
	if(dev->height > x3_usb_cam->m_infos->m_vin_info.mipi_attr.mipi_host_cfg.height) {
		LOGI_print("Scale up the original image is not supported");
		return;
	}

	if (is_on) {
		if (dev->height >= 240) {
			x3_usb_cam->m_dst_width = dev->width;
			x3_usb_cam->m_dst_height = dev->height;
		} else {
			printf("uvc_stream on/off fatal error, unsupport pixel solution: %d \n",
					dev->height);
			return;
		}

		// 重新设置vps的输出分辨率
		x3_uvc_vps_set_chn_attr(vps_grp_id, vps_chn_id, dev->width, dev->height);

		x3_usb_cam->m_payload_type = fcc_to_video_format(fcc);
		if (x3_usb_cam->m_payload_type == PT_H264 || x3_usb_cam->m_payload_type == PT_MJPEG) {
			x3_uvc_venc_init(x3_usb_cam);

			// vps bind venc
			x3_vps_bind_venc(vps_grp_id, vps_chn_id, x3_usb_cam->m_venc_chn);
		}
		printf("##STREAMON is_on(%d)## %s(%ux%u)\n", is_on,
				fcc_to_string(fcc), dev->width, dev->height);
	} else {
		if (x3_usb_cam->m_payload_type == PT_H264 || x3_usb_cam->m_payload_type == PT_MJPEG) {
			// vps unbind venc
			x3_vps_unbind_venc(vps_grp_id, vps_chn_id, x3_usb_cam->m_venc_chn);
			x3_uvc_venc_uninit(x3_usb_cam);
		}
		printf("##STREAMOFF is_on(%d)## %s(%ux%u)\n", is_on,
				fcc_to_string(fcc), dev->width, dev->height);
	}
}

static void uvc_streamon_off_eptz(struct uvc_context *ctx, int is_on,
		void *userdata) {
	struct uvc_device *dev;
	unsigned int fcc = 0;
	x3_usb_cam_t *x3_usb_cam = (x3_usb_cam_t *)userdata;

	dev = ctx->udev;
	fcc = dev->fcc;

	int vps_grp_id = x3_usb_cam->m_infos->m_vps_infos.m_vps_info[0].m_vps_grp_id;
	int vps_chn_id = x3_usb_cam->m_infos->m_vps_infos.m_vps_info[0].m_vps_chn_attrs[0].m_chn_id;

	printf("uvc_stream on/off value:%d\n", is_on);
	if (is_on) {
		if (dev->height == 2160) {
			x3_usb_cam->m_dst_width = 3840;
			x3_usb_cam->m_dst_height = 2160;
		} else if (dev->height == 1080) {
			x3_usb_cam->m_dst_width = 1920;
			x3_usb_cam->m_dst_height = 1080;
		} else if (dev->height == 720) {
			x3_usb_cam->m_dst_width = 1280;
			x3_usb_cam->m_dst_height = 720;
		} else {
			printf("uvc_stream on/off fatal error, unsupport pixel solution: %d \n",
					dev->height);
			return;
		}

		// 重新设置vps的输出分辨率
		x3_uvc_vps_set_chn_attr(vps_grp_id, vps_chn_id, dev->width, dev->height);

		x3_usb_cam->m_payload_type = fcc_to_video_format(fcc);
		if (x3_usb_cam->m_payload_type == PT_H264 || x3_usb_cam->m_payload_type == PT_MJPEG) {
			x3_uvc_venc_init(x3_usb_cam);

			// vps bind venc
			x3_vps_bind_venc(vps_grp_id, vps_chn_id, x3_usb_cam->m_venc_chn);
		}
		printf("##STREAMON is_on(%d)## %s(%ux%u)\n", is_on,
				fcc_to_string(fcc), dev->width, dev->height);
	} else {
		if (x3_usb_cam->m_payload_type == PT_H264 || x3_usb_cam->m_payload_type == PT_MJPEG) {
			// vps unbind venc
			x3_vps_unbind_venc(vps_grp_id, vps_chn_id, x3_usb_cam->m_venc_chn);
			x3_uvc_venc_uninit(x3_usb_cam);
		}
		printf("##STREAMOFF is_on(%d)## %s(%ux%u)\n", is_on,
				fcc_to_string(fcc), dev->width, dev->height);
	}
}

static int uvc_get_frame(struct uvc_context *ctx, void **buf_to,
		int *buf_len, void **entity, void *userdata) {
	x3_usb_cam_t *x3_usb_cam = (x3_usb_cam_t *)userdata;
	VIDEO_FRAME_S pstFrame;
	int ret = 0;
	static int pts = 0;
	static time_t start = 0;
	static time_t last = 0;
	static int idx = 0;

	if (!ctx || !ctx->udev) {
		printf("uvc_get_frame: input params is null\n");
		return -EINVAL;
	}

	if (!ctx->udev->is_streaming) // usb host是否在请求数据
		return 0;

	if (x3_usb_cam->m_payload_type == PT_H264 || x3_usb_cam->m_payload_type == PT_MJPEG) {
		VIDEO_STREAM_S *pstStream = (VIDEO_STREAM_S *)malloc(sizeof(VIDEO_STREAM_S));
		if (!pstStream) {
			return -ENOMEM;
		};

		memset(pstStream, 0, sizeof(VIDEO_STREAM_S));
		// 获取编码数据流
		ret = HB_VENC_GetStream(x3_usb_cam->m_venc_chn, pstStream, 2000);
		if (ret < 0) {
			printf("HB_VENC_GetStream error!!!\n");
			return -EINVAL;
		}

		/*LOGI_print("HB_VENC_GetStream succeed, pstStream:%p", pstStream);*/
		*buf_to =  pstStream->pstPack.vir_ptr;
		*buf_len = pstStream->pstPack.size;
		*entity = pstStream;

#if 0
		start = time(NULL);
		idx++;
		if (start > last) {
			printf("guvc venc fps %d\n", idx);
			last = start;
			idx = 0;
		}
#endif
	} else if (x3_usb_cam->m_payload_type == PT_NV) {
		hb_vio_buffer_t *vps_out_buf = (hb_vio_buffer_t *)malloc(sizeof(hb_vio_buffer_t));
		if (!vps_out_buf) {
			return -ENOMEM;
		};
		int vps_grp_id = x3_usb_cam->m_infos->m_vps_infos.m_vps_info[0].m_vps_grp_id;
		int vps_chn_id = x3_usb_cam->m_infos->m_vps_infos.m_vps_info[0].m_vps_chn_attrs[0].m_chn_id;
		memset(vps_out_buf, 0, sizeof(hb_vio_buffer_t));

		ret = HB_VPS_GetChnFrame(vps_grp_id, vps_chn_id, vps_out_buf, 2000);
		if (ret) {
			LOGE_print("HB_VPS_GetChnFrame error!!!");
			return -EINVAL;
		}

#if 1 // 内存拷贝的方式
		char *mem_yuv = (char *)malloc(vps_out_buf->img_addr.height * vps_out_buf->img_addr.width * 3 / 2);

		memcpy(mem_yuv, vps_out_buf->img_addr.addr[0],
				vps_out_buf->img_addr.height * vps_out_buf->img_addr.width);
		memcpy(mem_yuv + vps_out_buf->img_addr.height * vps_out_buf->img_addr.width,
				vps_out_buf->img_addr.addr[1],
				vps_out_buf->img_addr.height * vps_out_buf->img_addr.width / 2);

		HB_VPS_ReleaseChnFrame(vps_grp_id, vps_chn_id, vps_out_buf);
		*buf_to =  mem_yuv;
		*buf_len = vps_out_buf->img_addr.height * vps_out_buf->img_addr.width * 3 / 2;
		*entity = mem_yuv;

		free(vps_out_buf);
#else // 让 y 和 uv分量存在连续的内存上，避免内存拷贝
		// 在程序运行前配置 export IPU_YUV_CONSECTIVE=1 让获取出来的y uv 地址保持连续
		// 如果不进行动态重新配置输出分辨率，这样用可以减少一次内存拷贝，但是如果修改了输出分辨率，y 和 uv就不连续了。
		/*LOGI_print("HB_VPS_GetChnFrame width: %d height:%d stride_size: %d",*/
		/*vps_out_buf->img_addr.width, vps_out_buf->img_addr.height, vps_out_buf->img_addr.stride_size);*/

		/*LOGI_print("addr[0]:%p addr[1]:%p", vps_out_buf->img_addr.addr[0], vps_out_buf->img_addr.addr[1]);*/
		*buf_to =  vps_out_buf->img_addr.addr[0];
		*buf_len = vps_out_buf->img_addr.width * vps_out_buf->img_addr.height * 3 / 2;
		*entity = vps_out_buf;
#endif
	}

	return 0;
}

static void uvc_release_frame(struct uvc_context *ctx, void **entity,
		void *userdata) {

	x3_usb_cam_t *x3_usb_cam = (x3_usb_cam_t *)userdata;

	if (!ctx || !entity || !(*entity)) return;

	if (x3_usb_cam->m_payload_type == PT_H264 || x3_usb_cam->m_payload_type == PT_MJPEG) {
		VIDEO_STREAM_S *pstStream = (VIDEO_STREAM_S *)(*entity);
		if (pstStream) {
			int ret = HB_VENC_ReleaseStream(x3_usb_cam->m_venc_chn, pstStream);
			if (ret)
				LOGE_print("HB_VENC_ReleaseStream error");
		}

		free(pstStream);
		*entity = NULL;
	} else if (x3_usb_cam->m_payload_type == PT_NV) {
#if 1 // 内存拷贝方式
		free(*entity);
		*entity = NULL;

#else
		int vps_grp_id = x3_usb_cam->m_infos->m_vps_infos.m_vps_info[0].m_vps_grp_id;
		int vps_chn_id = x3_usb_cam->m_infos->m_vps_infos.m_vps_info[0].m_vps_chn_attrs[0].m_chn_id;
		hb_vio_buffer_t *vps_out_buf = (hb_vio_buffer_t *)(*entity);
		HB_VPS_ReleaseChnFrame(vps_grp_id, vps_chn_id, vps_out_buf);

		free(vps_out_buf);
		*entity = NULL;
#endif
	}
}

int x3_guvc_init(x3_usb_cam_t *x3_usb_cam)
{
	struct uvc_params params;
	char *uvc_devname = NULL;  /* uvc Null, lib will find one */
	char *v4l2_devname = NULL; /* v4l2 Null is Null... */
	int ret = 0;

	/* init uvc user params, just use default params and overlay
	 * some specify params.
	 */
	uvc_gadget_user_params_init(&params);
#if 0
	// bulk mode
	params.bulk_mode = 1;
	params.h264_quirk = 0;
	params.burst = 9;
#else
	// isoc
	params.bulk_mode = 0; /* use bulk mode  */
	params.h264_quirk = 0;
	// params.burst = 9;
	params.mult = 2;
#endif

	ret = uvc_gadget_init(&x3_usb_cam->uvc_ctx, uvc_devname, v4l2_devname, &params);
	if (ret < 0) {
		LOGE_print("uvc_gadget_init error!");
		return -1;
	}

#ifdef EPTZ
	uvc_set_streamon_handler(x3_usb_cam->uvc_ctx, uvc_streamon_off_eptz, x3_usb_cam);
#else
	uvc_set_streamon_handler(x3_usb_cam->uvc_ctx, uvc_streamon_off, x3_usb_cam);
#endif
	/* prepare/release buffer with video queue */
	uvc_set_prepare_data_handler(x3_usb_cam->uvc_ctx, uvc_get_frame, x3_usb_cam);
	uvc_set_release_data_handler(x3_usb_cam->uvc_ctx, uvc_release_frame, x3_usb_cam);

	printf("x3_guvc_init ok\n");

	return 0;
}

void x3_guvc_uninit(x3_usb_cam_t* x3_usb_cam)
{
	if (x3_usb_cam == NULL)
		return;

	uvc_gadget_deinit(x3_usb_cam->uvc_ctx);

}

int x3_guvc_start(x3_usb_cam_t* x3_usb_cam)
{
	int ret = 0;
	ret = uvc_gadget_start(x3_usb_cam->uvc_ctx);
	if (ret < 0) {
		LOGE_print("uvc_gadget_start error!");
	}

	printf("x3_guvc_start ok\n");
	return ret;
}

int x3_guvc_stop(x3_usb_cam_t* x3_usb_cam)
{
	int ret = 0;
	ret = uvc_gadget_stop(x3_usb_cam->uvc_ctx);
	if (ret < 0) {
		LOGE_print("x3_guvc_stop error!");
	}
	return ret;
}

static void *send_yuv_to_vot(void *ptr) {
	tsThread *privThread = (tsThread*)ptr;
	int ret = 0;
	hb_vio_buffer_t vps_out_buf;
	VOT_FRAME_INFO_S pstVotFrame;

	x3_modules_info_t *info = (x3_modules_info_t *)privThread->pvThreadData;

	mThreadSetName(privThread, __func__);

	while(privThread->eState == E_THREAD_RUNNING) {
		memset(&vps_out_buf, 0, sizeof(hb_vio_buffer_t));
		ret = HB_VPS_GetChnFrame(info->m_vps_infos.m_vps_info[0].m_vps_grp_id,
				3, &vps_out_buf, 300);
		if (ret != 0) {
			printf("HB_VPS_GetChnFrame error %d!!!\n", ret);
			continue;
		}

		// 把yuv（1080P）数据发给vot显示
		// 数据结构转换
		memset(&pstVotFrame, 0, sizeof(VOT_FRAME_INFO_S));
		pstVotFrame.addr = vps_out_buf.img_addr.addr[0]; // y分量虚拟地址
		pstVotFrame.addr_uv = vps_out_buf.img_addr.addr[1]; // uv分量虚拟地址
		pstVotFrame.size = 1920*1080*3/2; // 帧大小 1920*1088的帧需要强制成1920*1080
		// 发送数据帧到vo模块
		x3_vot_sendframe(&pstVotFrame);

		HB_VPS_ReleaseChnFrame(info->m_vps_infos.m_vps_info[0].m_vps_grp_id,
				3, &vps_out_buf);
	}

	mThreadFinish(privThread);
	return NULL;
}


int x3_usb_cam_init_param(void)
{
	int ret = 0;
	char sensor_name[32] = {0};
	int width = 0, height = 0, fps = 0;
	int i2c_bus_mun = -1;

	memset(&g_x3_modules_info, 0, sizeof(g_x3_modules_info));
	// 根据ipc solution的配置设置vin、vps、venc、bpu模块的使能和参数

	// vot 配置
	ret |= vot_param_init(&g_x3_modules_info.m_vot_info, 0);

	// 1. 配置vin
	g_x3_modules_info.m_enable = 1; // 整个数据通路使能
	g_x3_modules_info.m_vin_enable = 1;
	memset(sensor_name, 0, sizeof(sensor_name));
	strcpy(sensor_name, g_x3_config.usb_cam_solution.pipeline.sensor_name);
	if (strcmp(sensor_name, "IMX415") == 0) {
		ret = imx415_linear_vin_param_init(&g_x3_modules_info.m_vin_info);
	} else if (strcmp(sensor_name, "OS8A10") == 0) {
		ret = os8a10_linear_vin_param_init(&g_x3_modules_info.m_vin_info);
	} else if (strcmp(sensor_name, "OS8A10_2K") == 0) {
		ret = os8a10_2k_linear_vin_param_init(&g_x3_modules_info.m_vin_info);
	} else if (strcmp(sensor_name, "F37") == 0) {
		ret = f37_linear_vin_param_init(&g_x3_modules_info.m_vin_info);
	} else if (strcmp(sensor_name, "GC4663") == 0) {
		ret = gc4663_linear_vin_param_init(&g_x3_modules_info.m_vin_info);
	} else if (strcmp(sensor_name, "OV8856") == 0) {
			ret = ov8856_linear_vin_param_init(&g_x3_modules_info.m_vin_info);
	} else if (strcmp(sensor_name, "SC031GS") == 0) {
			ret = sc031gs_linear_vin_param_init(&g_x3_modules_info.m_vin_info);
	} else {
		LOGE_print("sensor name not found(%s)", sensor_name);
		g_x3_modules_info.m_vin_enable = 0;
		return -1;
	}

	/* 对i2c bus做自动矫正配置 */
	i2c_bus_mun = x3_get_sensor_on_which_i2c_bus(sensor_name);
	if (i2c_bus_mun >= 0) {
		g_x3_modules_info.m_vin_info.snsinfo.sensorInfo.bus_num = i2c_bus_mun;
	}

	// 减少ddr带宽使用量
	g_x3_modules_info.m_vin_info.vin_vps_mode = VIN_ONLINE_VPS_ONLINE;

	// 2. 根据vin中的分辨率配置vps
	width = g_x3_modules_info.m_vin_info.mipi_attr.mipi_host_cfg.width;
	height = g_x3_modules_info.m_vin_info.mipi_attr.mipi_host_cfg.height;
	fps = g_x3_modules_info.m_vin_info.mipi_attr.mipi_host_cfg.fps;

	g_x3_modules_info.m_vps_enable = 1;
	g_x3_modules_info.m_vps_infos.m_group_num = 1;
	// 配置group的输入
	ret |= vps_grp_param_init(&g_x3_modules_info.m_vps_infos.m_vps_info[0],
			width, height);
	// 以下是配置group的每一个通道的参数
	g_x3_modules_info.m_vps_infos.m_vps_info[0].m_chn_num = 3;
	// chn2 给 usb gadget
	ret |= vps_chn_param_init(&g_x3_modules_info.m_vps_infos.m_vps_info[0].m_vps_chn_attrs[0],
			2, width, height, fps);
	if (width >= 1920 && height >= 1080) {
		// chn 1 给 bpu 运算算法
		ret |= vps_chn_param_init(&g_x3_modules_info.m_vps_infos.m_vps_info[0].m_vps_chn_attrs[1],
				1, 1920, 1080, fps);
		// chn 3 给 hdmi 显示
		ret |= vps_chn_param_init(&g_x3_modules_info.m_vps_infos.m_vps_info[0].m_vps_chn_attrs[2],
				3, 1920, 1080, fps);
		g_x3_modules_info.m_vot_enable = 1;
	}

	// 3. rgn 配置
	g_x3_modules_info.m_rgn_enable = 1;
	ret = x3_rgn_timestamp_param_init(&g_x3_modules_info.m_rgn_info, 0, 0);

	// 4. bpu算法模型
	if (g_x3_config.usb_cam_solution.pipeline.alog_id != 0) {
		g_x3_modules_info.m_bpu_enable = 1;
		g_x3_modules_info.m_bpu_info.m_alog_id = g_x3_config.usb_cam_solution.pipeline.alog_id;
	}

	return ret;
}

int x3_usb_cam_init(void) {
	int ret = 0;
	x3_modules_info_t *infos = &g_x3_modules_info;
	memset(&g_x3_usb_cam, 0, sizeof(x3_usb_cam_t));
	g_x3_usb_cam.m_infos = infos;

	/* sdb 生态开发板  ，使能sensor       mclk */
	HB_MIPI_EnableSensorClock(0);
	HB_MIPI_EnableSensorClock(1);

	// 编码模块初始化
	HB_VENC_Module_Init();

	ret = x3_vp_init();
	if (ret) {
		LOGE_print("hb_vp_init failed, ret: %d", ret);
		goto vp_err;
	}

	// 初始化USB Gadget 模块
	ret = x3_guvc_init(&g_x3_usb_cam);
	if (ret) {
		LOGE_print("x3_guvc_init failed");
		goto guvc_err;
	}

	if (infos->m_vot_enable) {
		ret = x3_vot_init(&infos->m_vot_info);
		if (ret) {
			LOGE_print("x3_vot_init failed, %d", ret);
			goto vot_err;
		}
		LOGI_print("x3_vot_init ok!");
	}

	// 初始化算法模块
	// 初始化bpu
	if (infos->m_bpu_enable) {
		infos->m_bpu_info.m_bpu_handle = x3_bpu_sample_init(infos->m_bpu_info.m_alog_id);
		if (NULL == infos->m_bpu_info.m_bpu_handle) {
			LOGE_print("x3_bpu_predict_init failed");
			goto bpu_err;
		}
		// 注册算法结果回调函数
		x3_bpu_predict_callback_register(infos->m_bpu_info.m_bpu_handle, send_bpu_result_to_usb_host, NULL);
	}

	// 1. 初始化 vin，此处调用失败，大概率是因为sensor没有接，或者没有接好，或者sensor库用的版本不配套
	if (infos->m_vin_enable) {
		ret = x3_vin_init(&infos->m_vin_info);
		if (ret) {
			LOGE_print("x3_vin_init failed, %d", ret);
			goto vin_err;
		}
		LOGI_print("x3_vin_init ok!\n");
	}

	// 2. 初始化 vps
	if (infos->m_vps_enable) {
		ret = x3_vps_init_wrap(&infos->m_vps_infos.m_vps_info[0]);
		if (ret) {
			LOGE_print("x3_vps_init failed, %d", ret);
			goto vps_err;
		}
		LOGI_print("x3_vps_init_wrap ok!\n");
	}

	// 4 vin bind vps
	if (infos->m_vin_enable && infos->m_vps_enable) {
		ret = x3_vin_bind_vps(infos->m_vin_info.pipe_id, infos->m_vps_infos.m_vps_info[0].m_vps_grp_id,
				infos->m_vin_info.vin_vps_mode);
		if (ret) {
			LOGE_print("x3_vin_bind_vps failed, %d", ret);
			goto vps_bind_err;
		}
	}

	// 初始化 rgn，显示时间戳osd
	if (infos->m_rgn_enable) {
		ret = x3_rgn_init(infos->m_rgn_info.m_rgn_handle,
				&infos->m_rgn_info.m_rgn_chn, &infos->m_rgn_info.m_rgn_attr,
				&infos->m_rgn_info.m_rgn_chn_attr);
		if (ret != 0) {
			printf("x3_rgn_init failed, %x\n", ret);
			goto rgn_err;
		}
		LOGI_print("x3_rgn_init ok, infos->m_rgn_info.m_rgn_handle = %d", infos->m_rgn_info.m_rgn_handle);
	}

	LOGI_print("x3_ipc_init success");
	return ret;

rgn_err:
	x3_vin_unbind_vps(infos->m_vin_info.pipe_id, infos->m_vps_infos.m_vps_info[0].m_vps_grp_id,
			infos->m_vin_info.vin_vps_mode);
vps_bind_err:
	if (infos->m_vps_enable) {
		x3_vps_uninit_wrap(&infos->m_vps_infos.m_vps_info[0]);
	}
vps_err:
	if (infos->m_vin_enable) {
		x3_vin_deinit(&infos->m_vin_info);
	}
vin_err:
	if (infos->m_bpu_enable) {
		x3_bpu_predict_callback_unregister(infos->m_bpu_info.m_bpu_handle);
		x3_bpu_predict_unint(infos->m_bpu_info.m_bpu_handle);
	}
bpu_err:
	if (infos->m_vot_enable)
		x3_vot_deinit();
vot_err:
	x3_guvc_uninit(&g_x3_usb_cam);
guvc_err:
	x3_vp_deinit();
vp_err:
	HB_VENC_Module_Uninit();

	return -1;
}

int x3_usb_cam_uninit(void) {
	int i = 0;
	x3_modules_info_t *infos = &g_x3_modules_info;

	if (infos->m_vin_enable && infos->m_vps_enable) {
		x3_vin_unbind_vps(infos->m_vin_info.pipe_id, infos->m_vps_infos.m_vps_info[0].m_vps_grp_id,
				infos->m_vin_info.vin_vps_mode);
	}

	if (infos->m_rgn_enable) {
		x3_rgn_uninit(infos->m_rgn_info.m_rgn_handle,
				&infos->m_rgn_info.m_rgn_chn);
	}

	if (infos->m_bpu_enable) {
		x3_bpu_predict_callback_unregister(infos->m_bpu_info.m_bpu_handle);
		x3_bpu_predict_unint(infos->m_bpu_info.m_bpu_handle);
	}

	if (infos->m_vps_enable) {
		for (i = 0; i < infos->m_vps_infos.m_group_num; i++)
			x3_vps_uninit_wrap(&infos->m_vps_infos.m_vps_info[i]);
	}

	if (infos->m_vin_enable) {
		x3_vin_deinit(&infos->m_vin_info);
	}

	if (infos->m_vot_enable)
		x3_vot_deinit();

	x3_guvc_uninit(&g_x3_usb_cam);

	x3_vp_deinit();

	HB_VENC_Module_Uninit();

	print_debug_infos();
	return 0;
}

int x3_usb_cam_start(void) {
	int i = 0, ret = 0;
	x3_modules_info_t * infos = &g_x3_modules_info;

	// 使能 vps
	if (infos->m_vps_enable) {
		for (i = 0; i < infos->m_vps_infos.m_group_num; i++) {
			ret = x3_vps_start(infos->m_vps_infos.m_vps_info[i].m_vps_grp_id);
			if (ret) {
				LOGE_print("x3_vps_start failed, %d", ret);
				return -3;
			}
		}
		LOGI_print("x3_vps_start ok!");
	}

	if (infos->m_vin_enable) {
		ret = x3_vin_start(&infos->m_vin_info);
		if (ret) {
			LOGE_print("x3_vin_start failed, %d", ret);
			return -3003;
		}
	}

	// 启动算法模块
	if (infos->m_bpu_enable) {
		x3_bpu_predict_start(infos->m_bpu_info.m_bpu_handle);
		LOGI_print("x3_bpu_predict_start ok!");
	}

	// 启动osd 时间戳线程
	if (infos->m_rgn_enable) {
		g_x3_usb_cam.m_rgn_thread.pvThreadData = (void*)&infos->m_rgn_info.m_rgn_handle;
		mThreadStart(x3_rgn_set_timestamp_thread, &g_x3_usb_cam.m_rgn_thread, E_THREAD_JOINABLE);
	}

	if (infos->m_vot_enable) {
		g_x3_usb_cam.m_vot_thread.pvThreadData = (void*)infos;
		mThreadStart(send_yuv_to_vot, &g_x3_usb_cam.m_vot_thread, E_THREAD_JOINABLE);
	}

	// 启动usb gadget模块
	x3_guvc_start(&g_x3_usb_cam);

	print_debug_infos();
	return 0;

}

int x3_usb_cam_stop(void)
{
	int i = 0, ret = 0;
	x3_modules_info_t * infos = &g_x3_modules_info;

	x3_guvc_stop(&g_x3_usb_cam);

	if (infos->m_vot_enable) {
		mThreadStop(&g_x3_usb_cam.m_vot_thread);
	}

	if (infos->m_bpu_enable) {
		x3_bpu_predict_stop(infos->m_bpu_info.m_bpu_handle);
	}

	// 停止osd线程
	if (infos->m_rgn_enable) {
		mThreadStop(&g_x3_usb_cam.m_rgn_thread);
	}

	if (infos->m_vin_enable) {
		x3_vin_stop(&infos->m_vin_info);
	}

	// stop vps
	if (infos->m_vps_enable) {
		for (i = 0; i < infos->m_vps_infos.m_group_num; i++) {
			x3_vps_stop(infos->m_vps_infos.m_vps_info[i].m_vps_grp_id);
			if (ret) {
				LOGE_print("x3_vps_stop failed, %d", ret);
				return -1;
			}
		}
	}
	return 0;
}

int x3_usb_cam_param_set(CAM_PARAM_E type, char* val, unsigned int length)
{
	switch(type)
	{
	case CAM_VENC_BITRATE_SET:
		{
			// 只设置主码流的比特率
			x3_venc_set_bitrate(0, *(int *)val);
			break;
		}
	default:
		break;
	}
	return 0;
}

int x3_usb_cam_param_get(CAM_PARAM_E type, char* val, unsigned int* length)
{
	int ret = 0;
	switch(type)
	{
	case CAM_GET_CHIP_TYPE:
		{
			E_CHIP_TYPE chip_id = x3_get_chip_type();
			sprintf(val, "%s", chip_id == E_CHIP_X3M ? "X3M" : "X3E");
			break;
		}
	default:
		{
			ret= -1;
			break;
		}
	}

	return ret;
}

