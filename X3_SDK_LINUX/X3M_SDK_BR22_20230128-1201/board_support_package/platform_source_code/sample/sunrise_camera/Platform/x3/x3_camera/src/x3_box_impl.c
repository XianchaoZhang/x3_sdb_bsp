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
#include "utils/mqueue.h"

#include "x3_vio_venc.h"
#include "x3_vio_vin.h"
#include "x3_vio_vps.h"
#include "x3_vio_bind.h"
#include "x3_sdk_wrap.h"
#include "x3_vio_vdec.h"
#include "x3_vio_vot.h"
#include "x3_vio_vp.h"
#include "x3_bpu.h"
#include "x3_utils.h"
#include "x3_config.h"

#include "camera_handle.h"
#include "x3_preparam.h"
#include "x3_box_impl.h"

typedef struct
{
	x3_modules_info_t*	infos;
	shm_stream_t 		*venc_shms[32]; /* H264 H265 码流，最大可能是32路 */
	tsThread 		m_venc_thread; /* 图像编码、输出给vo、算法图像前处理 */
	tsThread 		m_vdec_thread[8]; /* 读取h264视频文件解码 */
	tsThread 		m_yuv_merge_thread; /* 图像拼接线程，多路盒子方案需要 */
	tsQueue			m_yuv_merge_queue; /* yuv图像队列 */
}x3_box_t;

static x3_box_t g_x3_box;
static x3_modules_info_t g_x3_modules_info;

// 1路解码线程
void *vdec_send_thread(void *ptr) {
	tsThread *privThread = (tsThread*)ptr;
	int ret = 0;
	int mmz_index = 0;

	mThreadSetName(privThread, __func__);

	x3_vdec_chn_info_t *vdec_chn_info = (x3_vdec_chn_info_t *)privThread->pvThreadData;

	VIDEO_FRAME_S pstFrame;
	VIDEO_STREAM_S pstStream;
	memset(&pstFrame, 0, sizeof(VIDEO_FRAME_S));
	memset(&pstStream, 0, sizeof(VIDEO_STREAM_S));

	AVFormatContext *avContext = NULL;
	AVPacket avpacket = {0};

	av_param_t *av_param = &vdec_chn_info->av_param;
	vp_param_t *vp_param = &vdec_chn_info->vp_param;

	LOGI_print("vdec m_stream_src: %s", vdec_chn_info->m_stream_src);

restart:
	mmz_index = 0;
	// 使用ffmpeg库的接口来读取h264码流
	av_param->firstPacket = 1;
	av_param->videoIndex = AV_open_stream_file(vdec_chn_info->m_stream_src, &avContext, &avpacket);
	if (av_param->videoIndex < 0) {
		printf("AV_open_stream_file failed, ret = %d\n", av_param->videoIndex);
		goto err;
	}
	while(privThread->eState == E_THREAD_RUNNING) {
		VDEC_CHN_STATUS_S pstStatus;
		HB_VDEC_QueryStatus(vdec_chn_info->m_vdec_chn_id, &pstStatus);
		if (pstStatus.cur_input_buf_cnt >= (uint32_t)vp_param->mmz_cnt) {
			LOGE_print("vdec_chn_info->m_vdec_chn_id: %d continue");
			usleep(10000);
			continue;
		}
		usleep(33000); // 帧率控制
		mmz_index = AV_read_frame(avContext, &avpacket, av_param, vp_param);
		if (mmz_index == -1) {
			LOGI_print("AV_read_frame eos");
			pstStream.pstPack.phy_ptr = vp_param->mmz_paddr[0];
			pstStream.pstPack.vir_ptr = vp_param->mmz_vaddr[0];
			pstStream.pstPack.pts = av_param->count;
			pstStream.pstPack.src_idx = mmz_index;
			pstStream.pstPack.size = 0;
			pstStream.pstPack.stream_end = HB_FALSE;
			ret = HB_VDEC_SendStream(vdec_chn_info->m_vdec_chn_id, &pstStream, 200);
			if (ret) {
				LOGE_print("HB_VDEC_SendStream chn%d failed, ret: %d", vdec_chn_info->m_vdec_chn_id, ret);
			}
			if (avContext) {
				avformat_close_input(&avContext);
			}

			// 重新读取文件继续发送
			goto restart;
		}
		pstStream.pstPack.phy_ptr = vp_param->mmz_paddr[mmz_index];
		pstStream.pstPack.vir_ptr = vp_param->mmz_vaddr[mmz_index];
		pstStream.pstPack.pts = av_param->count;
		pstStream.pstPack.src_idx = mmz_index;
		pstStream.pstPack.size = av_param->bufSize;
		pstStream.pstPack.stream_end = HB_FALSE;

		/*printf("[%p] vdec chn%d pstStream.pstPack.size = %d\n",*/
		/*privThread, vdec_chn_info->m_vdec_chn_id, pstStream.pstPack.size);*/

		ret = HB_VDEC_SendStream(vdec_chn_info->m_vdec_chn_id, &pstStream, 200);
		if (ret) {
			LOGI_print("HB_VDEC_SendStream chn%d error, ret: %d", vdec_chn_info->m_vdec_chn_id, ret);
		}
	}

err:
	// 发送空帧，尝试解决解码概率性异常，导致vpu退出的问题
	pstStream.pstPack.phy_ptr = vp_param->mmz_paddr[mmz_index];
	pstStream.pstPack.vir_ptr = vp_param->mmz_vaddr[mmz_index];
	pstStream.pstPack.pts = av_param->count;
	pstStream.pstPack.src_idx = mmz_index;
	pstStream.pstPack.size = 0;
	pstStream.pstPack.stream_end = HB_FALSE;
	ret = HB_VDEC_SendStream(vdec_chn_info->m_vdec_chn_id, &pstStream, 200);
	if (ret) {
		LOGI_print("HB_VDEC_SendStream chn%d failed, ret: %d", vdec_chn_info->m_vdec_chn_id, ret);
	}
	if (avContext) {
		avformat_close_input(&avContext);
	}
	mThreadFinish(privThread);
	return NULL;
}

void x3_box_push_stream(VENC_CHN VeChn, VENC_CHN_ATTR_S *vencChnAttr, VIDEO_STREAM_S *pstStream)
{
	shm_stream_t* stm = NULL;
	PAYLOAD_TYPE_E enType = vencChnAttr->stVencAttr.enType;
	if (VeChn > 32) return;
	/*LOGI_print("in x3_venc_use_stream");*/
	stm = g_x3_box.venc_shms[VeChn]; // 这里先默认用h264，功能跑通之后需要区别对待h265和jpeg
	if(stm == NULL) {
		char shm_id[32] = {0}, shm_name[32] = {0};
		sprintf(shm_id, "cam_id_%s_chn%d", enType == PT_H264 ? "h264" :
				(enType == PT_H265 ? "h264" :
				 (enType == PT_JPEG) ? "jpeg" : "other"), VeChn);
		sprintf(shm_name, "name_%s_chn%d", enType == PT_H264 ? "h264" :
				(enType == PT_H265 ? "h264" :
				 (enType == PT_JPEG) ? "jpeg" : "other"), VeChn);
		g_x3_box.venc_shms[VeChn] = shm_stream_create(shm_id, shm_name,
				STREAM_MAX_USER, x3_venc_get_framerate(vencChnAttr),
				vencChnAttr->stVencAttr.u32BitStreamBufSize,
				SHM_STREAM_WRITE, SHM_STREAM_MALLOC);
	}

	frame_info info;
	info.type 		= enType;
	info.seq 		= pstStream->pstPack.src_idx;
	info.pts 		= pstStream->pstPack.pts;
	info.length 	= pstStream->pstPack.size;
	info.t_time 	= (unsigned int)time(0);
	info.framerate 	= x3_venc_get_framerate(vencChnAttr);
	info.width 		= vencChnAttr->stVencAttr.u32PicWidth;
	info.height 	= vencChnAttr->stVencAttr.u32PicHeight;

	if(enType == PT_H264)
	{
		info.key	= pstStream->stStreamInfo.nalu_type == 1 ? 0 : 1; /* MC_H264_NALU_TYPE_P */
	}
	else if(enType == PT_H265)
	{
		info.key	= pstStream->stStreamInfo.nalu_type == 1 ? 0 : 1; /* MC_H265_NALU_TYPE_P */
	}
	else
	{
		info.key	= 1;
	}
	/*LOGI_print("in shm_stream_put");*/
	shm_stream_put(stm, info, (unsigned char*)pstStream->pstPack.vir_ptr, pstStream->pstPack.size);
}


// 编码和输出图像到vo线程
// 因为需要输出给vo，所以不能解码后直接绑定输出给编码
static void *stream_output_1to1_thread(void *ptr) {
	tsThread *privThread = (tsThread*)ptr;
	int ret;
	VIDEO_FRAME_S pstFrame;
	VIDEO_STREAM_S pstStream;
	VOT_FRAME_INFO_S pstVotFrame;
	VENC_CHN_ATTR_S vencChnAttr;
	address_info_t bpu_input_data;
	memset(&pstFrame, 0, sizeof(VIDEO_FRAME_S));
	memset(&pstStream, 0, sizeof(VIDEO_STREAM_S));
	memset(&bpu_input_data, 0, sizeof(address_info_t));

	x3_box_t *x3_box = (x3_box_t *)privThread->pvThreadData;

	mThreadSetName(privThread, __func__);

/*#define SINGLE_IMAGE_TEST*/
#ifdef SINGLE_IMAGE_TEST
	// 1080P yuv单图送bpu运行，测试用
	address_info_t bpu_input_test_data;
	VIDEO_FRAME_S pstFrame_test;
	memset(&bpu_input_test_data, 0, sizeof(address_info_t));
	HB_SYS_Alloc(&bpu_input_test_data.paddr[0],
			(void **)&bpu_input_test_data.addr[0], 1920 * 1080);
	HB_SYS_Alloc(&bpu_input_test_data.paddr[1],
			(void **)&bpu_input_test_data.addr[1], 1920 * 1080 / 2);
	/*int img_in_fd = open("../test_data/1080P_dog_bike_car.yuv", O_RDWR | O_CREAT, 0666);*/
	int img_in_fd = open("../test_data/1920x1080_nv12.yuv", O_RDWR | O_CREAT, 0666);
	if (img_in_fd < 0) {
		printf("open image failed !\n");
		return;
	}

	read(img_in_fd, bpu_input_test_data.addr[0], 1920 * 1080);
	usleep(10 * 1000);
	read(img_in_fd, bpu_input_test_data.addr[1], 1920 * 1080 / 2);
	usleep(10 * 1000);
	close(img_in_fd);
#endif

	while (privThread->eState == E_THREAD_RUNNING) {
		// 从解码器获取解码好的yuv数据
		memset(&pstFrame, 0, sizeof(VIDEO_FRAME_S));
		ret = HB_VDEC_GetFrame(0, &pstFrame, 2000);
		if (ret) {
			LOGE_print("HB_VDEC_GetFrame error!!!");
			continue;
		}

		// 把yuv数据送进bpu进行算法运算
		memset(&bpu_input_data, 0, sizeof(address_info_t));
		bpu_input_data.height = pstFrame.stVFrame.height;
		bpu_input_data.width = pstFrame.stVFrame.width;
		bpu_input_data.stride_size = pstFrame.stVFrame.stride;
		bpu_input_data.addr[0] = pstFrame.stVFrame.vir_ptr[0];
		bpu_input_data.addr[1] = pstFrame.stVFrame.vir_ptr[1];
		bpu_input_data.paddr[0] = pstFrame.stVFrame.phy_ptr[0];
		bpu_input_data.paddr[1] = pstFrame.stVFrame.phy_ptr[1];

#ifdef SINGLE_IMAGE_TEST
		// 1080P yuv单图送bpu运行，测试用
		bpu_input_test_data.height = pstFrame.stVFrame.height;
		bpu_input_test_data.width = pstFrame.stVFrame.width;
		bpu_input_test_data.stride_size = pstFrame.stVFrame.stride;
		pstFrame_test = pstFrame;
		pstFrame_test.stVFrame.vir_ptr[0] = bpu_input_test_data.addr[0];
		pstFrame_test.stVFrame.vir_ptr[1] = bpu_input_test_data.addr[1];

		pstFrame_test.stVFrame.phy_ptr[0] = bpu_input_test_data.paddr[0];
		pstFrame_test.stVFrame.phy_ptr[1] = bpu_input_test_data.paddr[1];

		x3_bpu_input_feed(x3_box->infos->m_bpu_info.m_bpu_handle, &bpu_input_test_data);
#else
		x3_bpu_input_feed(x3_box->infos->m_bpu_info.m_bpu_handle, &bpu_input_data);
#endif

		// 数据结构转换
#ifdef SINGLE_IMAGE_TEST
		memset(&pstVotFrame, 0, sizeof(VOT_FRAME_INFO_S));
		pstVotFrame.addr = pstFrame_test.stVFrame.vir_ptr[0]; // y分量虚拟地址
		pstVotFrame.addr_uv = pstFrame_test.stVFrame.vir_ptr[1]; // uv分量虚拟地址
		pstVotFrame.size = 1920*1080*3/2; // 帧大小 1920*1088的帧需要强制成1920*1080
#else
		memset(&pstVotFrame, 0, sizeof(VOT_FRAME_INFO_S));
		pstVotFrame.addr = pstFrame.stVFrame.vir_ptr[0]; // y分量虚拟地址
		pstVotFrame.addr_uv = pstFrame.stVFrame.vir_ptr[1]; // uv分量虚拟地址
		pstVotFrame.size = 1920*1080*3/2; // 帧大小 1920*1088的帧需要强制成1920*1080

#endif
		// 发送数据帧到vo模块
		x3_vot_sendframe(&pstVotFrame);

		// 把yuv数据送进编码器编码
#ifdef SINGLE_IMAGE_TEST
		ret = HB_VENC_SendFrame(0, &pstFrame_test, 2000);
#else
		ret = HB_VENC_SendFrame(0, &pstFrame, 2000);
#endif
		if (ret < 0) {
			printf("HB_VENC_SendFrame error!!!\n");
			continue;
		}
		// 释放yuv buff
		HB_VDEC_ReleaseFrame(0, &pstFrame);
		// 获取编码数据流
		ret = HB_VENC_GetStream(0, &pstStream, 2000);
		if (ret < 0) {
			printf("HB_VENC_GetStream error!!!\n");
			continue;
		} else {
			// 把编码数据送进共享内存，rtsp server获取后推流
			ret = x3_venc_get_chn_attr(0, &vencChnAttr);
			if (ret) { // 报错就认为是通道没有使能
				continue;
			}
			x3_box_push_stream(0, &vencChnAttr, &pstStream);
			// 释放编码buff
			HB_VENC_ReleaseStream(0, &pstStream);
		}
	}
	mThreadFinish(privThread);
	return NULL;
}

// 参数说明：
// py: 目标内存Y分量地址
// puv: 目标内存UV分量地址
// width: 目标内存的宽
// height: 目标内存的高
// stFrameInfo: 数据源
// idx: 需要填到4分屏的哪个位置，左上0， 右上1，左下2，右下3
int memcpy_4in1(char *py, char *puv, int width, int height,
		VIDEO_FRAME_S *stFrameInfo, int idx)
{
	int w, h, i;
	char *pdsty = py, *pdstuv = puv;

	w = stFrameInfo->stVFrame.width;
	h = stFrameInfo->stVFrame.height;

	if (idx == 0) {
		pdsty = py;
		pdstuv = puv;
	} else if (idx == 1) {
		pdsty = py + width/2;
		pdstuv = puv + width/2;
	} else if (idx == 2) {
		pdsty = py + width*height/2;
		pdstuv = puv + width*height/4;
	} else {
		pdsty = py + width*height/2 + width/2;
		pdstuv = puv + width*height/4 + width/2;
	}

	for (i=0; i<h; i++) {
		memcpy(pdsty, stFrameInfo->stVFrame.vir_ptr[0]+i*stFrameInfo->stVFrame.stride, w);
		pdsty += width;
	}

	for (i=0; i<h/2; i++) {
		memcpy(pdstuv, stFrameInfo->stVFrame.vir_ptr[1]+i*stFrameInfo->stVFrame.stride, w);
		pdstuv += width;
	}

	return 0;
}

int memcpy_8in1(char *py, char *puv, int width, int height,
		VIDEO_FRAME_S *stFrameInfo, int idx)
{
	int w, h, i;
	char *pdsty = py, *pdstuv = puv;

	w = stFrameInfo->stVFrame.width;
	h = stFrameInfo->stVFrame.height;

	if (idx == 0) {
		pdsty = py;
		pdstuv = puv;
	} else if (idx == 1) {
		pdsty = py + width/4;
		pdstuv = puv + width/4;
	} else if (idx == 2) {
		pdsty = py + width*2/4;
		pdstuv = puv + width*2/4;
	} else if (idx == 3) {
		pdsty = py + width*3/4;
		pdstuv = puv + width*3/4;
	} else if (idx == 4) {
		pdsty = py + width*height/2;
		pdstuv = puv + width*height/4;
	} else if (idx == 5) {
		pdsty = py + width*height/2 + width/4;
		pdstuv = puv + width*height/4 + width/4;
	} else if (idx == 6) {
		pdsty = py + width*height/2 + width*2/4;
		pdstuv = puv + width*height/4 + width*2/4;
	} else if (idx == 7) {
		pdsty = py + width*height/2 + width*3/4;
		pdstuv = puv + width*height/4 + width*3/4;
	}

	for (i=0; i<h; i++) {
		memcpy(pdsty, stFrameInfo->stVFrame.vir_ptr[0]+i*stFrameInfo->stVFrame.stride, w);
		pdsty += width;
	}

	for (i=0; i<h/2; i++) {
		memcpy(pdstuv, stFrameInfo->stVFrame.vir_ptr[1]+i*stFrameInfo->stVFrame.stride, w);
		pdstuv += width;
	}

	return 0;
}

// 拼接成的4Kyuv进行编码和缩小成1080P输出图像到vo线程
// 因为需要输出给vo，所以不能解码后直接绑定输出给编码
// 拼接成的4K图像，需要使用vps创建一个group来缩小成1080P给到vot
// 缩小成1080P的图像经BPU rezise成模型输入大小后放到推理队列
static void *stream_output_4in1_thread(void *ptr)
{
	tsThread *privThread = (tsThread*)ptr;
	int ret = 0;
	vp_param_t *mix_yuv_buf;
	int pts = 0;
	VIDEO_FRAME_S pst4kFrame;
	VIDEO_STREAM_S pstStream;
	VOT_FRAME_INFO_S pstVotFrame;
	VENC_CHN_ATTR_S vencChnAttr;
	hb_vio_buffer_t vps_out_buf;
	memset(&pstStream, 0, sizeof(VIDEO_STREAM_S));

	mThreadSetName(privThread, __func__);

	// 送给bpu处理的数据机构
	address_info_t bpu_input_data;
	memset(&bpu_input_data, 0, sizeof(address_info_t));

	mThreadSetName(privThread, __func__);

	x3_box_t *x3_box = (x3_box_t *)privThread->pvThreadData;
	int height = x3_box->infos->m_venc_info.m_venc_chn_info[0].m_chn_attr.stVencAttr.u32PicHeight;
	int width = x3_box->infos->m_venc_info.m_venc_chn_info[0].m_chn_attr.stVencAttr.u32PicWidth;

	while (privThread->eState == E_THREAD_RUNNING) {
		if (mQueueDequeueTimed(&x3_box->m_yuv_merge_queue, 100, (void**)&mix_yuv_buf) != E_QUEUE_OK)
			continue;

		/*LOGI_print("m_4in1_yuv_queue Dequeue");*/

		// 把yuv数据送进编码器编码

		memset(&pst4kFrame, 0, sizeof(VIDEO_FRAME_S));
		pst4kFrame.stVFrame.phy_ptr[0] = mix_yuv_buf->mmz_paddr[0];
		pst4kFrame.stVFrame.phy_ptr[1] = mix_yuv_buf->mmz_paddr[0] + width * height;
		pst4kFrame.stVFrame.vir_ptr[0] = mix_yuv_buf->mmz_vaddr[0];
		pst4kFrame.stVFrame.vir_ptr[1] = mix_yuv_buf->mmz_vaddr[0] + width * height;
		pst4kFrame.stVFrame.pts = pts++;
		ret = HB_VENC_SendFrame(0, &pst4kFrame, 2000);
		if (ret < 0) {
			LOGE_print("HB_VENC_SendFrame error!!!");
			continue;
		}

		// X3M 是拼接成4K的到这里来编码的，所以需要通过vps缩小成1080P给vot和算法
		// vps 4K 缩小成 1080P 再获取出来，20ms
		// 把4K yuv数据送进vps缩小成1080P, 缩放输出到显示，如果在编码推流性能不足的情况下，最好另开线程去处理
		if (x3_get_chip_type() == E_CHIP_X3M) {
			hb_vio_buffer_t image_in = {0};
			memset(&image_in, '\0', sizeof(hb_vio_buffer_t));
			// 虚拟地址
			image_in.img_addr.addr[0] = mix_yuv_buf->mmz_vaddr[0];
			image_in.img_addr.addr[1] = mix_yuv_buf->mmz_vaddr[0] + width * height;
			// 物理地址
			image_in.img_addr.paddr[0] = mix_yuv_buf->mmz_paddr[0];
			image_in.img_addr.paddr[1] = mix_yuv_buf->mmz_paddr[0] + width * height;
			/*printf("vaddr0: %p, vaddr1: %p\n",*/
			/*image_in.img_addr.addr[0], image_in.img_addr.addr[1]);*/
			image_in.img_addr.width = width;
			image_in.img_addr.height = height;
			image_in.img_addr.stride_size = width;
			image_in.img_info.planeCount = 2;
			/*LOGI_print("Send yuv to vps, grp:%d m_chn_id:%d",*/
			/*g_x3_box.infos->m_vps_infos.m_vps_info[0].m_vps_grp_id,*/
			/*g_x3_box.infos->m_vps_infos.m_vps_info[0].m_vps_chn_attrs[0].m_chn_id);*/
			ret = x3_vps_input(x3_box->infos->m_vps_infos.m_vps_info[0].m_vps_grp_id, &image_in);
			if (ret) {
				LOGE_print("hb_vps_input failed, %d", ret);
				continue;
			}
			memset(&vps_out_buf, 0, sizeof(hb_vio_buffer_t));
			ret = HB_VPS_GetChnFrame(x3_box->infos->m_vps_infos.m_vps_info[0].m_vps_grp_id,
					x3_box->infos->m_vps_infos.m_vps_info[0].m_vps_chn_attrs[0].m_chn_id,
					&vps_out_buf, 2000);
			if (ret != 0) {
				LOGE_print("HB_VPS_GetChnFrame error %d!!!", ret);
				continue;
			}

			/*x3_vio_buf_info_print(&vps_out_buf);*/

			// 把yuv（1080P）数据送进bpu进行算法运算
			memset(&bpu_input_data, 0, sizeof(address_info_t));
			bpu_input_data = vps_out_buf.img_addr;
			// 前处理 7-9ms
			x3_bpu_input_feed(x3_box->infos->m_bpu_info.m_bpu_handle, &bpu_input_data);

			// 此时从vps中获取到的应该是1920 * 1080 的yuv数据，可以直接送给vot显示
			// 数据结构转换
			memset(&pstVotFrame, 0, sizeof(VOT_FRAME_INFO_S));
			pstVotFrame.addr = vps_out_buf.img_addr.addr[0]; // y分量虚拟地址
			pstVotFrame.addr_uv = vps_out_buf.img_addr.addr[1]; // uv分量虚拟地址
			pstVotFrame.size = vps_out_buf.img_addr.width * vps_out_buf.img_addr.height * 3 / 2;
			// 发送数据帧到vo模块
			ret = x3_vot_sendframe(&pstVotFrame);
			if (ret != 0) {
				LOGE_print("x3_vot_sendframe error %d!!!", ret);
			}
			// 释放VPS 帧
			HB_VPS_ReleaseChnFrame(x3_box->infos->m_vps_infos.m_vps_info[0].m_vps_grp_id,
					x3_box->infos->m_vps_infos.m_vps_info[0].m_vps_chn_attrs[0].m_chn_id,
					&vps_out_buf);
		}
		else {// X3E 拼接出来的就是1080P,直接送vot和算法
			// 把yuv数据送进bpu进行算法运算
			memset(&bpu_input_data, 0, sizeof(address_info_t));
			bpu_input_data.height = height;
			bpu_input_data.width = width;
			bpu_input_data.stride_size = width;
			bpu_input_data.addr[0] = mix_yuv_buf->mmz_vaddr[0];
			bpu_input_data.addr[1] = mix_yuv_buf->mmz_vaddr[0] + width * height;
			bpu_input_data.paddr[0] = mix_yuv_buf->mmz_paddr[0];
			bpu_input_data.paddr[1] = mix_yuv_buf->mmz_paddr[0] + width * height;

			x3_bpu_input_feed(x3_box->infos->m_bpu_info.m_bpu_handle, &bpu_input_data);

			// 数据结构转换
			memset(&pstVotFrame, 0, sizeof(VOT_FRAME_INFO_S));
			pstVotFrame.addr = mix_yuv_buf->mmz_vaddr[0]; // y分量虚拟地址
			pstVotFrame.addr_uv = mix_yuv_buf->mmz_vaddr[0] + width * height; // uv分量虚拟地址
			pstVotFrame.size = width * height * 3/2; // 帧大小 1920*1088的帧需要强制成1920*1080
			// 发送数据帧到vo模块
			ret = x3_vot_sendframe(&pstVotFrame);
			if (ret != 0) {
				LOGE_print("x3_vot_sendframe error %d!!!", ret);
			}
		}
		// 获取编码数据流
		ret = HB_VENC_GetStream(0, &pstStream, 2000);
		if (ret < 0) {
			printf("HB_VENC_GetStream error!!!\n");
			continue;
		} else {
			// 把编码数据送进共享内存，rtsp server获取后推流
			ret = x3_venc_get_chn_attr(0, &vencChnAttr);
			if (ret) { // 报错就认为是通道没有使能
				continue;
			}
			x3_box_push_stream(0, &vencChnAttr, &pstStream);
			// 释放编码buff
			HB_VENC_ReleaseStream(0, &pstStream);
		}

	}

	mThreadFinish(privThread);
	return NULL;
}

void *yuv_4in1_mix_thread(void *ptr) {
	tsThread *privThread = (tsThread*)ptr;
	int i, ret;
	VIDEO_FRAME_S pstFrame;
	vp_param_t mix_yuv_buf[5]; // 5个buf轮转
	int cur_buf_idx = 0;
	memset(&pstFrame, 0, sizeof(VIDEO_FRAME_S));

	x3_box_t *x3box = (x3_box_t *)privThread->pvThreadData;

	int height = x3box->infos->m_venc_info.m_venc_chn_info[0].m_chn_attr.stVencAttr.u32PicHeight;
	int width = x3box->infos->m_venc_info.m_venc_chn_info[0].m_chn_attr.stVencAttr.u32PicWidth;

	mThreadSetName(privThread, __func__);

	// 申请YUV的内存buff
	for (i = 0; i < 5; i++) {
		mix_yuv_buf[i].mmz_cnt = 1;
		// 先乘以3再除以2的性能会比直接乘以1.5高出3倍左右
		// 1088 多出来的8像素需要去掉
		// 根据编码通道的分辨率设置buf大小
		mix_yuv_buf[i].mmz_size = width * height * 3 / 2; // 编码图像分辨率
		x3_vp_alloc(&mix_yuv_buf[i]);
	}

	while (privThread->eState == E_THREAD_RUNNING) {
		// 依次获取4路解码器的数据，并且进行拼接
		// 获取解码数据拼接 12-14ms
		for (i = 0; i < 4 && privThread->eState == E_THREAD_RUNNING; i++) {
			memset(&pstFrame, 0, sizeof(VIDEO_FRAME_S));
			ret = HB_VDEC_GetFrame(i, &pstFrame, 1000);
			if (ret) {
				LOGE_print("HB_VDEC_GetFrame error!!!\n");
				i = -1; // 重新获取
				continue;
			}
			/*printf("get vdec chn%d stream\n", i);*/
			// 如果是X3E，那么要在这里把解码出来的1080P缩小成960*540,然后再拼接成1080P
			if (x3_get_chip_type() == E_CHIP_X3E) {
				// VIDEO_FRAME_S 数据类型转成 VPS 输入类型 hb_vio_buffer_t
				hb_vio_buffer_t image_in = {0};
				memset(&image_in, '\0', sizeof(hb_vio_buffer_t));
				// 虚拟地址
				image_in.img_addr.addr[0] = pstFrame.stVFrame.vir_ptr[0];
				image_in.img_addr.addr[1] = pstFrame.stVFrame.vir_ptr[1];
				// 物理地址
				image_in.img_addr.paddr[0] = pstFrame.stVFrame.phy_ptr[0];
				image_in.img_addr.paddr[1] = pstFrame.stVFrame.phy_ptr[1];
				/*printf("vaddr0: %p, vaddr1: %p\n",*/
				/*image_in.img_addr.addr[0], image_in.img_addr.addr[1]);*/
				image_in.img_addr.width = pstFrame.stVFrame.width;
				image_in.img_addr.height = pstFrame.stVFrame.height;
				image_in.img_addr.stride_size = pstFrame.stVFrame.stride;
				image_in.img_info.planeCount = 2; // 都是nv12，y 和 uv 所以是2

				// 送yuv进vps缩小
				x3_vps_input(x3box->infos->m_vps_infos.m_vps_info[0].m_vps_grp_id, &image_in);
				if (ret) {
					printf("hb_vps_input failed, %d\n", ret);
				}

				// 获取vps输出yuv
				hb_vio_buffer_t vps_out_buf;
				memset(&vps_out_buf, 0, sizeof(hb_vio_buffer_t));
				ret = HB_VPS_GetChnFrame(x3box->infos->m_vps_infos.m_vps_info[0].m_vps_grp_id,
						x3box->infos->m_vps_infos.m_vps_info[0].m_vps_chn_attrs[0].m_chn_id,
						&vps_out_buf, 2000);
				if (ret != 0) {
					printf("HB_VPS_GetChnFrame error %d!!!\n", ret);
					continue;
				}
				// hb_vio_buffer_t 数据类型转 VIDEO_FRAME_S
				VIDEO_FRAME_S pstFrameTemp;
				memset(&pstFrameTemp, '\0', sizeof(pstFrameTemp));
				pstFrameTemp.stVFrame.width = vps_out_buf.img_addr.width;
				pstFrameTemp.stVFrame.height = vps_out_buf.img_addr.height;
				pstFrameTemp.stVFrame.stride = vps_out_buf.img_addr.stride_size;
				pstFrameTemp.stVFrame.vir_ptr[0] = vps_out_buf.img_addr.addr[0];
				pstFrameTemp.stVFrame.vir_ptr[1] = vps_out_buf.img_addr.addr[1];
				// 拼接
				memcpy_4in1(mix_yuv_buf[cur_buf_idx].mmz_vaddr[0], mix_yuv_buf[cur_buf_idx].mmz_vaddr[0] + width * height, width, height,
						&pstFrameTemp, i);

				// 释放vps输出帧 buff
				HB_VPS_ReleaseChnFrame(x3box->infos->m_vps_infos.m_vps_info[0].m_vps_grp_id,
						x3box->infos->m_vps_infos.m_vps_info[0].m_vps_chn_attrs[0].m_chn_id, &vps_out_buf);
			} else {
				// 4个1080P拼成4K：一次调用拼接耗时 3ms
				memcpy_4in1(mix_yuv_buf[cur_buf_idx].mmz_vaddr[0], mix_yuv_buf[cur_buf_idx].mmz_vaddr[0] + width * height, width, height,
						&pstFrame, i);
			}
			// 释放解码器输出帧 buff
			HB_VDEC_ReleaseFrame(i, &pstFrame);
		}

		// 把拼接好的 yuv 图像 送进处理队列
		mQueueEnqueueEx(&x3box->m_yuv_merge_queue, &mix_yuv_buf[cur_buf_idx]);
		cur_buf_idx++;
		cur_buf_idx %= 5;

	}

	// 释放YUV的内存buff
	for (i = 0; i < 5; i++) {
		x3_vp_free(&mix_yuv_buf[i]);
	}

	mThreadFinish(privThread);
	return NULL;
}

int x3_box_init_param(void)
{
	int i, ret = 0;

	memset(&g_x3_modules_info, 0, sizeof(g_x3_modules_info));
	g_x3_modules_info.m_enable = 1;

	// 根据box solution的配置设置vps、vdec、venc、bpu模块的使能和参数
	if (g_x3_config.box_solution.box_chns == 1) { // 1路方案
		// 配置编码通道
		g_x3_modules_info.m_venc_enable = 1;
		g_x3_modules_info.m_venc_info.m_chn_num = 1;
		ret = venc_chn_param_init(&g_x3_modules_info.m_venc_info.m_venc_chn_info[0],
				0, 1920, 1080, 30, g_x3_config.box_solution.venc_bitrate);

		// 配置解码通道
		g_x3_modules_info.m_vdec_enable = 1;
		g_x3_modules_info.m_vdec_info.m_chn_num = 1;
		ret |= vdec_chn_param_init(&g_x3_modules_info.m_vdec_info.m_vdec_chn_info[0],
				0, 1920, 1088, "../test_data/1080P_test.h264");

		// 配置vot输出通道
		g_x3_modules_info.m_vot_enable = 1;
		ret |= vot_param_init(&g_x3_modules_info.m_vot_info, 0);

		// 配置算法模型
		if (g_x3_config.box_solution.alog_id != 0) {
			g_x3_modules_info.m_bpu_enable = 1;
			g_x3_modules_info.m_bpu_info.m_alog_id = g_x3_config.box_solution.alog_id;
		}
	} else if (g_x3_config.box_solution.box_chns == 4) { // 4路方案
		// 配置编码通道
		g_x3_modules_info.m_venc_enable = 1;
		g_x3_modules_info.m_venc_info.m_chn_num = 1;
		// 如果是X3E芯片，编码换成1080P分辨路，vps配置成1080P输入 960*540输出，vps用作缩小解码出来的yuv数据
		if (x3_get_chip_type() == E_CHIP_X3E) {
			ret = venc_chn_param_init(&g_x3_modules_info.m_venc_info.m_venc_chn_info[0],
					0, 1920, 1080, 30, g_x3_config.box_solution.venc_bitrate);
		} else {
			ret = venc_chn_param_init(&g_x3_modules_info.m_venc_info.m_venc_chn_info[0],
					0, 3840, 2160, 30, g_x3_config.box_solution.venc_bitrate);
		}

		// 配置解码通道
		g_x3_modules_info.m_vdec_enable = 1;
		g_x3_modules_info.m_vdec_info.m_chn_num = 4;
		for (i = 0; i < 4; i++) {
			ret |= vdec_chn_param_init(&g_x3_modules_info.m_vdec_info.m_vdec_chn_info[i],
					i, 1920, 1088, "../test_data/1080P_test.h264");
		}

		// 配置vot输出通道
		g_x3_modules_info.m_vot_enable = 1;
		ret |= vot_param_init(&g_x3_modules_info.m_vot_info, 0);

		// 4路方案需要新建一个vps通道来做缩放
		g_x3_modules_info.m_vps_enable = 1;
		g_x3_modules_info.m_vps_infos.m_group_num = 1;
		// 配置group的输入
		if (x3_get_chip_type() == E_CHIP_X3E) { // X3E 先对每一路缩小再拼接成1080P
			ret |= vps_grp_param_init(&g_x3_modules_info.m_vps_infos.m_vps_info[0],
					1920, 1080);
			// 以下是配置group的每一个通道的参数
			g_x3_modules_info.m_vps_infos.m_vps_info[0].m_chn_num = 1;
			ret |= vps_chn_param_init(&g_x3_modules_info.m_vps_infos.m_vps_info[0].m_vps_chn_attrs[0],
					2, 960, 540, 30);
		} else { // X3M 先拼接成4K，再缩小成1080P
			ret |= vps_grp_param_init(&g_x3_modules_info.m_vps_infos.m_vps_info[0],
					3840, 2160);
			// 以下是配置group的每一个通道的参数
			g_x3_modules_info.m_vps_infos.m_vps_info[0].m_chn_num = 1;
			ret |= vps_chn_param_init(&g_x3_modules_info.m_vps_infos.m_vps_info[0].m_vps_chn_attrs[0],
					2, 1920, 1080, 30);
		}

		// 配置算法模型
		if (g_x3_config.box_solution.alog_id != 0) {
			g_x3_modules_info.m_bpu_enable = 1;
			g_x3_modules_info.m_bpu_info.m_alog_id = g_x3_config.box_solution.alog_id;
		}
	} else {
		LOGE_print("%d-channel is not supported!", g_x3_config.box_solution.box_chns);
		return -1;
	}

	return ret;
}


int x3_box_init(void)
{
	int i = 0, ret = 0;
	x3_modules_info_t *infos = &g_x3_modules_info;
	memset(&g_x3_box, 0, sizeof(x3_box_t));
	g_x3_box.infos = infos;

	// 编码、解码模块初始化，整个应用中需要调用一次
	HB_VDEC_Module_Init();
	HB_VENC_Module_Init();

	ret = x3_vp_init();
	if (ret) {
		LOGE_print("hb_vp_init failed, ret: %d", ret);
		goto vp_err;
	}

	LOGI_print("x3_vp_init ok!");

	if (infos->m_vot_enable) {
		ret = x3_vot_init(&infos->m_vot_info);
		if (ret) {
			LOGE_print("x3_vot_init failed, %d", ret);
			goto vot_err;
		}
		LOGI_print("x3_vot_init ok!");
	}

	if (infos->m_vdec_enable) {
		ret = x3_vdec_init_wrap(&infos->m_vdec_info);
		if (ret) {
			LOGE_print("x3_vdec_init_wrap failed, %d", ret);
			goto vdec_err;
		}
		LOGI_print("x3_vdec_init_wrap ok!");
	}

	// 初始化视频码流编码器
	if (infos->m_venc_enable) {
		ret = x3_venc_init_wrap(&infos->m_venc_info); // 初始化编码器通道
		if (ret) {
			LOGE_print("x3_venc_init_wrap failed, %d", ret);
			goto venc_err;
		}
		LOGI_print("x3_venc_init_wrap ok!");
	}

	// 初始化vps
	if (infos->m_vps_enable) {
		// 修改VPS的输入模式为ddr输入模式
		HB_SYS_SetVINVPSMode(0, VIN_OFFLINE_VPS_OFFINE);

		for (i = 0; i < infos->m_vps_infos.m_group_num; i++) {
			ret = x3_vps_init_wrap(&infos->m_vps_infos.m_vps_info[i]);
			if (ret) {
				LOGE_print("x3_vps_init_wrap failed, %d", ret);
				goto vps_err;
			}
		}
		LOGI_print("x3_vps_init_wrap ok!");
	}

	if (infos->m_bpu_enable) {
		// 初始化bpu
		LOGI_print("DEBUG!");
		infos->m_bpu_info.m_bpu_handle = x3_bpu_sample_init(infos->m_bpu_info.m_alog_id);
		if (NULL == infos->m_bpu_info.m_bpu_handle) {
			LOGE_print("x3_bpu_predict_init failed");
			goto bpu_err;
		}
		// 注册算法结果回调函数
		x3_bpu_predict_callback_register(infos->m_bpu_info.m_bpu_handle,
			x3_bpu_predict_general_result_handle, NULL);

		LOGI_print("x3_bpu_predict_init ok!");
	}
	return 0;

bpu_err:
	if (infos->m_vps_enable) {
		for (i = 0; i < infos->m_vps_infos.m_group_num; i++)
			x3_vps_uninit_wrap(&infos->m_vps_infos.m_vps_info[i]);
	}
vps_err:
	if (infos->m_venc_enable)
		x3_venc_uninit_wrap(&infos->m_venc_info);
venc_err:
	if (infos->m_vdec_enable)
		x3_vdec_uninit_wrap(&infos->m_vdec_info);
vdec_err:
	if (infos->m_vot_enable)
		x3_vot_deinit();
vot_err:
	x3_vp_deinit();
vp_err:
	HB_VDEC_Module_Uninit();
	HB_VENC_Module_Uninit();
	return 0;
}

int x3_box_uninit(void)
{
	int i = 0;
	shm_stream_t* stm = NULL;
	x3_modules_info_t *infos = &g_x3_modules_info;
	if (infos->m_bpu_enable) {
		x3_bpu_predict_callback_unregister(infos->m_bpu_info.m_bpu_handle);
		x3_bpu_predict_unint(infos->m_bpu_info.m_bpu_handle);
	}

	if (infos->m_vps_enable) {
		for (i = 0; i < infos->m_vps_infos.m_group_num; i++)
			x3_vps_uninit_wrap(&infos->m_vps_infos.m_vps_info[i]);
	}
	if (infos->m_venc_enable)
		x3_venc_uninit_wrap(&infos->m_venc_info);
	if (infos->m_vdec_enable)
		x3_vdec_uninit_wrap(&infos->m_vdec_info);
	if (infos->m_vot_enable)
		x3_vot_deinit();

	x3_vp_deinit();

	HB_VDEC_Module_Uninit();
	HB_VENC_Module_Uninit();

	stm = g_x3_box.venc_shms[0];

	if (stm != NULL) {
		shm_stream_destory(stm); // 销毁共享内存读句柄
		g_x3_box.venc_shms[0] = NULL;
	}

	print_debug_infos();
	return 0;
}

int x3_box_start(void)
{
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

	// start 解码器
	if (infos->m_vdec_enable) {
		for (i = 0; i < infos->m_vdec_info.m_chn_num; i++){
			ret = x3_vdec_start(infos->m_vdec_info.m_vdec_chn_info[i].m_vdec_chn_id);
			if (ret) {
				LOGE_print("x3_vdec_start failed, %d", ret);
				return -1;
			}

			// 启动解码线程
			LOGI_print("m_vdec_chn_info: %p", &infos->m_vdec_info.m_vdec_chn_info[i]);
			g_x3_box.m_vdec_thread[i].pvThreadData = (void*)&infos->m_vdec_info.m_vdec_chn_info[i];
			mThreadStart(vdec_send_thread, &g_x3_box.m_vdec_thread[i], E_THREAD_JOINABLE);
		}
	}

	// start 编码模块
	if (infos->m_venc_enable) {
		for (i = 0; i < infos->m_venc_info.m_chn_num; i++){
			ret = x3_venc_start(infos->m_venc_info.m_venc_chn_info[i].m_venc_chn_id);
			if (ret) {
				LOGE_print("x3_venc_start failed, %d", ret);
				return -2;
			}
		}
	}

	if (g_x3_config.box_solution.box_chns == 1) { // 1路1080P
		// 数据从解码器来
		g_x3_box.m_venc_thread.pvThreadData = (void *)&g_x3_box;
		mThreadStart(stream_output_1to1_thread, &g_x3_box.m_venc_thread, E_THREAD_JOINABLE);
	} else if (g_x3_config.box_solution.box_chns == 4) { // 4路1080P
		// 初始化拼接图像队列
		mQueueCreate(&g_x3_box.m_yuv_merge_queue, 2);//the length of queue is 2

		// 启动拼接线程
		g_x3_box.m_yuv_merge_thread.pvThreadData = (void *)&g_x3_box;
		mThreadStart(yuv_4in1_mix_thread, &g_x3_box.m_yuv_merge_thread, E_THREAD_JOINABLE);

		usleep(1000);

		// 把拼接后的图像送给编码器推流、缩小成1080P给到vot显示，并完成算法数据前处理
		g_x3_box.m_venc_thread.pvThreadData = (void *)&g_x3_box;
		mThreadStart(stream_output_4in1_thread, &g_x3_box.m_venc_thread, E_THREAD_JOINABLE);

		// 需要设置bpu后处理的原始图像大小为推流视频分辨率
		x3_bpu_predict_set_ori_hw(infos->m_bpu_info.m_bpu_handle,
				infos->m_venc_info.m_venc_chn_info[0].m_chn_attr.stVencAttr.u32PicWidth,
				infos->m_venc_info.m_venc_chn_info[0].m_chn_attr.stVencAttr.u32PicHeight);
	}

	if (infos->m_bpu_enable) {
		x3_bpu_predict_start(infos->m_bpu_info.m_bpu_handle);
		LOGI_print("x3_bpu_predict_start ok!");
	}

	print_debug_infos();
	return 0;
}

int x3_box_stop(void)
{
	int i = 0, ret = 0;
	x3_modules_info_t * infos = &g_x3_modules_info;

	if (infos->m_bpu_enable) {
		x3_bpu_predict_stop(infos->m_bpu_info.m_bpu_handle);
	}

	// 结束解码、编码线程
	if (infos->m_venc_enable) {
		mThreadStop(&g_x3_box.m_venc_thread);
	}

	// 4 路的时候需要多停掉两个线程
	if (g_x3_config.box_solution.box_chns == 4) {
		mThreadStop(&g_x3_box.m_yuv_merge_thread);
		mQueueDestroy(&g_x3_box.m_yuv_merge_queue);
	}

	if (infos->m_vdec_enable) {
		for (i = 0; i < infos->m_vdec_info.m_chn_num; i++){
			mThreadStop(&g_x3_box.m_vdec_thread[i]);
		}
	}

	// stop 编码模块
	if (infos->m_venc_enable) {
		for (i = 0; i < infos->m_venc_info.m_chn_num; i++){
			ret = x3_venc_stop(infos->m_venc_info.m_venc_chn_info[i].m_venc_chn_id);
			if (ret) {
				LOGE_print("x3_venc_stop failed, %d", ret);
				return -1;
			}
		}
	}

	// stop 解码模块
	if (infos->m_vdec_enable) {
		for (i = 0; i < infos->m_vdec_info.m_chn_num; i++){
			ret = x3_vdec_stop(infos->m_vdec_info.m_vdec_chn_info[i].m_vdec_chn_id);
			if (ret) {
				LOGE_print("x3_vdec_stop failed, %d", ret);
				return -1;
			}
		}
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

int x3_box_param_set(CAM_PARAM_E type, char* val, unsigned int length)
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

int x3_box_param_get(CAM_PARAM_E type, char* val, unsigned int* length)
{
	int ret = 0;
	switch(type)
	{
	case CAM_VENC_CHN_PARAM_GET: // 获取某个编码通道的配置
		{
			venc_info_t* param = (venc_info_t*)val;
			VENC_CHN_ATTR_S vencChnAttr;
			LOGI_print("param->channle: %d", param->channle);
			ret = x3_venc_get_chn_attr(param->channle, &vencChnAttr);
			if (ret) { // 报错就认为是通道没有使能
				param->enable = 0;
				break;
			}
			// 填充对外的信息
			param->enable = 1;
			param->type = vencChnAttr.stVencAttr.enType;
			param->mirror = vencChnAttr.stVencAttr.enMirrorFlip;
			param->flip = vencChnAttr.stVencAttr.enMirrorFlip;
			param->framerate = x3_venc_get_framerate(&vencChnAttr);
			if (param->type == PT_H264 || param->type == PT_H265) {
				param->bitrate = x3_venc_get_bitrate(&vencChnAttr);
				param->gop = x3_venc_get_gop(&vencChnAttr); // I帧间隔
			}
			param->stream_buf_size = vencChnAttr.stVencAttr.u32BitStreamBufSize;
			if (param->type == PT_H264)
				param->profile = vencChnAttr.stVencAttr.stAttrH264.h264_profile;
			param->width = vencChnAttr.stVencAttr.u32PicWidth;
			param->height = vencChnAttr.stVencAttr.u32PicHeight;
			param->cvbr = vencChnAttr.stRcAttr.enRcMode;
			break;
		}
	case CAM_GET_VENC_CHN_STATUS: // 获取哪些编码通道被使能了
		{
			// 32位的整形，每个通道的状态占其中一个bit
			// 注： 64bit的值 位与会有异常，待查
			unsigned int *status = (unsigned int *)val;
			VENC_CHN_ATTR_S vencChnAttr;
			int i;
			*status = 0;
			for (i = 0; i < 32; i++) {
				if(x3_venc_get_chn_attr(i, &vencChnAttr) == 0)
					*status |= (1 << i);
			}
			LOGI_print("status: %u\n", *status);
			break;
		}
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

