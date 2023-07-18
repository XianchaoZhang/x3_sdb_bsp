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

// sunrise camare 封装的头文件
#include "x3_vio_venc.h"
#include "x3_vio_vin.h"
#include "x3_vio_vps.h"
#include "x3_vio_vot.h"
#include "x3_vio_bind.h"
#include "x3_sdk_wrap.h"
#include "x3_vio_rgn.h"
#include "x3_utils.h"
#include "x3_bpu.h"
#include "x3_config.h"

#include "camera_handle.h"
#include "x3_preparam.h"
#include "x3_ipc_impl.h"

#define MAX_FRAME_SIZE		(250*1024)
#define DEFAULT_FRAME_SIZE	(50*1024)

typedef struct
{
	x3_modules_info_t* 	infos;
	shm_stream_t 		*venc_shms[32]; /* H264 H265 码流，最大可能是32路 */
	shm_stream_t* 		shm_yuv;
	unsigned char* 		frame;
	unsigned int		frame_size;
	VENC_CHN		jpeg_venc_chn; // jpeg编码通道

	pthread_t		md_tid;
	int			md_stop;
	pthread_t		yuv_tid;
	int			yuv_stop;

	tsThread		m_venc_thread[MAX_DEV_NUM]; // 对应多个编码通道，实现web上的多码流切换
	tsThread		m_rgn_thread[MAX_DEV_NUM];
	tsThread		m_bpu_thread[MAX_DEV_NUM];
	tsThread		m_vot_thread;
}x3_ipc_t;

static x3_ipc_t g_x3_ipc;
static x3_modules_info_t g_x3_modules_info[MAX_DEV_NUM];

/* 创建编码通道，用来抓拍后编码成jpeg */
static int x3_jpeg_venc_init(void)
{
	int ret = 0;
	VENC_CHN_ATTR_S vencChnAttr;
	VENC_RECV_PIC_PARAM_S pstRecvParam;

	// jpeg编码通道固定设置为 32
	g_x3_ipc.jpeg_venc_chn = 32;
	// 创建一个jpeg编码通道，把yuv编码成jpg
	memset(&vencChnAttr, 0, sizeof(VENC_CHN_ATTR_S));
	vencChnAttr.stVencAttr.enType = PT_JPEG;

	// 因为该编码器的作用是编码所有的yuv成jpeg
	// 输入尺寸不确定，此处暂时使用4K来初始化
	// 在实际调用编码前需要根据yuv的尺寸来重新设置尺寸
	vencChnAttr.stVencAttr.u32PicWidth = 3840;
	vencChnAttr.stVencAttr.u32PicHeight = 2160;

	vencChnAttr.stVencAttr.enMirrorFlip = DIRECTION_NONE;
	vencChnAttr.stVencAttr.enRotation = CODEC_ROTATION_0;
	vencChnAttr.stVencAttr.stCropCfg.bEnable = HB_FALSE;

	vencChnAttr.stVencAttr.enPixelFormat = HB_PIXEL_FORMAT_NV12;
	vencChnAttr.stVencAttr.u32BitStreamBufferCount = 1;
	vencChnAttr.stVencAttr.u32FrameBufferCount = 2;
	vencChnAttr.stVencAttr.bExternalFreamBuffer = HB_TRUE;
	vencChnAttr.stVencAttr.stAttrJpeg.dcf_enable = HB_FALSE;
	vencChnAttr.stVencAttr.stAttrJpeg.quality_factor = 100;
	vencChnAttr.stVencAttr.stAttrJpeg.restart_interval = 0;
	vencChnAttr.stVencAttr.u32BitStreamBufSize = 4*1024*1024;

	ret = HB_VENC_CreateChn(g_x3_ipc.jpeg_venc_chn, &vencChnAttr);
	if (ret != 0) {
		printf("HB_VENC_CreateChn %d failed, %x.\n", g_x3_ipc.jpeg_venc_chn, ret);
		return -1;
	}

	ret = HB_VENC_SetChnAttr(g_x3_ipc.jpeg_venc_chn, &vencChnAttr);  // config
	if (ret != 0) {
		printf("HB_VENC_SetChnAttr failed\n");
		return -1;
	}

	// 这个地方的start编码启动先就放这里启动，顺便测试一下流程调用上是不是会有异常
	pstRecvParam.s32RecvPicNum = 0;  // unchangable
	ret = HB_VENC_StartRecvFrame(g_x3_ipc.jpeg_venc_chn, &pstRecvParam);
	if (ret != 0) {
		printf("HB_VENC_StartRecvFrame failed, %x\n", ret);
		return -1;
	}

	LOGI_print("x3_jpeg_venc_init ok!\n");

	return 0;
}

static int x3_jpeg_venc_uninit(void)
{
	int ret = 0;
	ret = HB_VENC_StopRecvFrame(g_x3_ipc.jpeg_venc_chn);
	if (ret != 0) {
		printf("HB_VENC_StopRecvFrame failed, %x\n", ret);
		return ret;
	}
	ret = x3_venc_deinit(g_x3_ipc.jpeg_venc_chn);
	if (ret) {
		printf("x3_venc_deinit failed, %d\n", ret);
		return ret;
	}

	return ret;
}

/******************************************************************************/

void x3_venc_use_stream(VENC_CHN VeChn, VENC_CHN_ATTR_S *vencChnAttr, VIDEO_STREAM_S *pstStream)
{
	shm_stream_t* stm = NULL;
	PAYLOAD_TYPE_E enType = vencChnAttr->stVencAttr.enType;
	if (VeChn > 32) return;
	stm = g_x3_ipc.venc_shms[VeChn]; // 这里先默认用h264，功能跑通之后需要区别对待h265和jpeg
	if(stm == NULL) {
		char shm_id[32] = {0}, shm_name[32] = {0};
		sprintf(shm_id, "cam_id_%s_chn%d", enType == PT_H264 ? "h264" :
				(enType == PT_H265 ? "h264" :
				 (enType == PT_JPEG) ? "jpeg" : "other"), VeChn);
		sprintf(shm_name, "name_%s_chn%d", enType == PT_H264 ? "h264" :
				(enType == PT_H265 ? "h264" :
				 (enType == PT_JPEG) ? "jpeg" : "other"), VeChn);
		g_x3_ipc.venc_shms[VeChn] = shm_stream_create(shm_id, shm_name,
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
	/*LOGI_print("in shm_stream_put, stm:%p pstStream->pstPack.vir_ptr:%p", stm, pstStream->pstPack.vir_ptr);*/
	shm_stream_put(stm, info, (unsigned char*)pstStream->pstPack.vir_ptr, pstStream->pstPack.size);
}


/******************************************************************************
 * funciton : get stream from each channels
 ******************************************************************************/
void* x3_venc_get_stream_proc(void *ptr)
{
	tsThread *privThread = (tsThread*)ptr;
	uint32_t i;
	uint32_t s32ChnTotal;
	VENC_CHN_ATTR_S stVencChnAttr;
	uint32_t maxfd = 0;
	struct timeval timeout_val;
	fd_set read_fds;
	int32_t VencFd[VENC_MAX_CHN_NUM];
	char aszFileName[VENC_MAX_CHN_NUM][64];
	FILE *pFile[VENC_MAX_CHN_NUM];
	char szFilePostfix[10];
	VIDEO_STREAM_S pstStream;
	int32_t s32Ret;
	VENC_CHN VencChn;
	PAYLOAD_TYPE_E enPayLoadType[VENC_MAX_CHN_NUM];

	mThreadSetName(privThread, __func__);

	x3_venc_info_t *venc_info = (x3_venc_info_t *)privThread->pvThreadData;
	s32ChnTotal = venc_info->m_chn_num;
	printf("[%s] Enabled venc total channles:%d\n", __func__, s32ChnTotal);

	/******************************************
	  step 1:  check & prepare save-file & venc-fd
	 ******************************************/
	if (s32ChnTotal >= VENC_MAX_CHN_NUM)
	{
		printf("venc input chn count invaild\n");
		return NULL;
	}
	for (i = 0; i < s32ChnTotal; i++)
	{
		/* decide the stream file name, and open file to save stream */
		VencChn = venc_info->m_venc_chn_info[i].m_venc_chn_id;
		s32Ret = HB_VENC_GetChnAttr(VencChn, &stVencChnAttr);
		if(s32Ret)
		{
			printf("HB_VENC_GetChnAttr chn[%d] failed with %d!\n", \
					VencChn, s32Ret);
			return NULL;
		}
		enPayLoadType[i] = stVencChnAttr.stVencAttr.enType;

		s32Ret = x3_venc_get_file_postfix(enPayLoadType[i], szFilePostfix);
		if(s32Ret)
		{
			printf("x3_venc_get_file_postfix [%d] failed with %d!\n", \
					stVencChnAttr.stVencAttr.enType, s32Ret);
			return NULL;
		}

		if(venc_info->m_venc_chn_info[i].m_is_save_to_file == 1)
		{
			sprintf(aszFileName[i], "stream_chn%d%s", VencChn, szFilePostfix);
			pFile[i] = fopen(aszFileName[i], "wb");
			if (!pFile[i])
			{
				printf("open file[%s] failed!\n",
						aszFileName[i]);
				return NULL;
			}
		}

		/* Set Venc Fd. */
		s32Ret = HB_VENC_GetFd(VencChn, &VencFd[i]);
		if (s32Ret)
		{
			printf("HB_VENC_GetFd failed with %d!\n", s32Ret);
			return NULL;
		}
		if (maxfd <= VencFd[i])
		{
			maxfd = VencFd[i];
		}
	}

	/******************************************
	  step 2:  Start to get streams of each channel.
	 ******************************************/
	while (privThread->eState == E_THREAD_RUNNING)
	{
		FD_ZERO(&read_fds);
		for (i = 0; i < s32ChnTotal; i++)
		{
			FD_SET(VencFd[i], &read_fds);
		}

		timeout_val.tv_sec  = 3;
		timeout_val.tv_usec = 0;
		s32Ret = select(maxfd + 1, &read_fds, NULL, NULL, &timeout_val);
		if (s32Ret < 0)
		{
			printf("select failed!\n");
			continue;
		}
		else if (s32Ret == 0)
		{
			printf("get venc stream time out, continue\n");
			continue;
		}
		else
		{
			for (i = 0; i < s32ChnTotal; i++)
			{
				if (FD_ISSET(VencFd[i], &read_fds))
				{
					/*******************************************************
					  step 2.1: call mpi to get one-frame stream
					 *******************************************************/
					s32Ret = HB_VENC_GetStream(venc_info->m_venc_chn_info[i].m_venc_chn_id,
							&pstStream, 0);
					if (s32Ret)
					{
						printf("HB_VENC_GetStream failed with %d!\n", \
								s32Ret);
						break;
					}

#if 0
					printf("pstStream.pstPack.size:%d\n",
							pstStream.pstPack.size);
#endif

					/*******************************************************
					  step 2.2 : save frame to file
					 *******************************************************/
					if(venc_info->m_venc_chn_info[i].m_is_save_to_file == 0)
					{
#if 0
						printf("[%s][%d] i = %d chn: %d stVencChnAttr:%p\n",
								__func__, __LINE__,
								i, venc_info->m_venc_chn_info[i].m_venc_chn_id,
								&stVencChnAttr);
#endif
						x3_venc_use_stream(venc_info->m_venc_chn_info[i].m_venc_chn_id,
								&stVencChnAttr, &pstStream);
					}
					else
					{
						s32Ret = x3_venc_save_stream(enPayLoadType[i], pFile[i], &pstStream);
						if (s32Ret)
						{
							printf("x3_venc_save_stream failed!\n");
							break;
						}
					}
					/*******************************************************
					  step 2.3 : release stream
					 *******************************************************/
					s32Ret = HB_VENC_ReleaseStream(venc_info->m_venc_chn_info[i].m_venc_chn_id, &pstStream);
				}
			}
		}
	}

	/*******************************************************
	 * step 3 : close save-file
	 *******************************************************/
	if(venc_info->m_venc_chn_info[i].m_is_save_to_file == 1)
	{
		for (i = 0; i < s32ChnTotal; i++)
		{
			fclose(pFile[i]);
		}
	}

	mThreadFinish(privThread);
	return NULL;
}

// 从vps的chn5获取yuv数据送给bpu进行算法运行
static void *send_yuv_to_bpu(void *ptr) {
	tsThread *privThread = (tsThread*)ptr;
	int ret = 0;
	hb_vio_buffer_t vps_out_buf;

	x3_modules_info_t *info = (x3_modules_info_t *)privThread->pvThreadData;

	// 送给bpu处理的数据机构
	address_info_t bpu_input_data;
	memset(&bpu_input_data, 0, sizeof(address_info_t));

	mThreadSetName(privThread, __func__);

	while(privThread->eState == E_THREAD_RUNNING) {
		memset(&vps_out_buf, 0, sizeof(hb_vio_buffer_t));
		ret = HB_VPS_GetChnFrame(info->m_vps_infos.m_vps_info[0].m_vps_grp_id,
				1, &vps_out_buf, 300);
		if (ret != 0) {
			printf("HB_VPS_GetChnFrame error %d!!!\n", ret);
			continue;
		}

		// 把yuv（1080P）数据送进bpu进行算法运算
		memset(&bpu_input_data, 0, sizeof(address_info_t));
		bpu_input_data = vps_out_buf.img_addr;
		// 前处理 7-9ms
		x3_bpu_input_feed(info->m_bpu_info.m_bpu_handle, &bpu_input_data);

		HB_VPS_ReleaseChnFrame(info->m_vps_infos.m_vps_info[0].m_vps_grp_id,
				1, &vps_out_buf);
	}

	mThreadFinish(privThread);
	return NULL;
}

static void *send_yuv_to_vot(void *ptr) {
	tsThread *privThread = (tsThread*)ptr;
	int ret = 0;
	hb_vio_buffer_t vps_out_buf;
	VOT_FRAME_INFO_S pstVotFrame;

	x3_modules_info_t *info = (x3_modules_info_t *)privThread->pvThreadData;

	mThreadSetName(privThread, __func__);

	if (g_x3_config.disp_dev == 1) {
		ret = HB_VPS_SetChnRotate(info->m_vps_infos.m_vps_info[0].m_vps_grp_id, 3, ROTATION_90);
		if (ret) {
			LOGE_print("HB_VPS_SetChnRotate failed, %d", ret);
		}
	}

	while(privThread->eState == E_THREAD_RUNNING) {
		memset(&vps_out_buf, 0, sizeof(hb_vio_buffer_t));
		ret = HB_VPS_GetChnFrame(info->m_vps_infos.m_vps_info[0].m_vps_grp_id,
				3, &vps_out_buf, 300);
		if (ret != 0) {
			printf("HB_VPS_GetChnFrame error %d!!!\n", ret);
			continue;
		}

		// 把yuv数据发给vot显示
		// 数据结构转换
		memset(&pstVotFrame, 0, sizeof(VOT_FRAME_INFO_S));
		pstVotFrame.addr = vps_out_buf.img_addr.addr[0]; // y分量虚拟地址
		pstVotFrame.addr_uv = vps_out_buf.img_addr.addr[1]; // uv分量虚拟地址
		pstVotFrame.size = info->m_vot_info.m_stLayerAttr.stImageSize.u32Width*info->m_vot_info.m_stLayerAttr.stImageSize.u32Height*3/2; // 帧大小
		// 发送数据帧到vo模块
		x3_vot_sendframe(&pstVotFrame);

		HB_VPS_ReleaseChnFrame(info->m_vps_infos.m_vps_info[0].m_vps_grp_id,
				3, &vps_out_buf);
	}

	mThreadFinish(privThread);
	return NULL;
}


int x3_ipc_init_param(void)
{
	int i, ret = 0;
	char sensor_name[32] = {0};
	int width = 0, height = 0, fps = 0;
	int i2c_bus_mun = -1;

	memset(g_x3_modules_info, 0, sizeof(g_x3_modules_info));
	// 根据ipc solution的配置设置vin、vps、venc、bpu模块的使能和参数

	// 配置vot输出通道, 第一路的视频送hdmi/LCD显示
	if (g_x3_config.ipc_solution.pipeline_num == 1) {
		ret |= vot_param_init(&g_x3_modules_info[0].m_vot_info, g_x3_config.disp_dev);
	}

	// 1. 配置vin
	for (i = 0; i < g_x3_config.ipc_solution.pipeline_num; i++) {
		g_x3_modules_info[i].m_enable = 1; // 整个数据通路使能
		g_x3_modules_info[i].m_vin_enable = 1;
		memset(sensor_name, 0, sizeof(sensor_name));
		strcpy(sensor_name, g_x3_config.ipc_solution.pipelines[i].sensor_name);
		if (strcmp(sensor_name, "IMX415") == 0) {
			ret = imx415_linear_vin_param_init(&g_x3_modules_info[i].m_vin_info);
		} else if (strcmp(sensor_name, "OS8A10") == 0) {
			ret = os8a10_linear_vin_param_init(&g_x3_modules_info[i].m_vin_info);
		} else if (strcmp(sensor_name, "OS8A10_2K") == 0) {
			ret = os8a10_2k_linear_vin_param_init(&g_x3_modules_info[i].m_vin_info);
		} else if (strcmp(sensor_name, "GC4663") == 0) {
			ret = gc4663_linear_vin_param_init(&g_x3_modules_info[i].m_vin_info);
		} else if (strcmp(sensor_name, "F37") == 0) {
			ret = f37_linear_vin_param_init(&g_x3_modules_info[i].m_vin_info);
		} else if (strcmp(sensor_name, "IMX415_BV") == 0) {
			ret = imx415_bv_vin_param_init(&g_x3_modules_info[i].m_vin_info);
		} else if (strcmp(sensor_name, "OV8856") == 0) {
			ret = ov8856_linear_vin_param_init(&g_x3_modules_info[i].m_vin_info);
		} else if (strcmp(sensor_name, "SC031GS") == 0) {
			ret = sc031gs_linear_vin_param_init(&g_x3_modules_info[i].m_vin_info);
		} else {
			LOGE_print("sensor name not found(%s)", sensor_name);
			g_x3_modules_info[i].m_vin_enable = 0;
			return -1;
		}

		/* 对i2c bus做自动矫正配置 */
		i2c_bus_mun = x3_get_sensor_on_which_i2c_bus(sensor_name);
		if (i2c_bus_mun >= 0) {
			g_x3_modules_info[i].m_vin_info.snsinfo.sensorInfo.bus_num = i2c_bus_mun;
		}
	}

	// 2. 根据vin中的分辨率配置vps 和 venc
	for (i = 0; i < g_x3_config.ipc_solution.pipeline_num; i++) {
		width = g_x3_modules_info[i].m_vin_info.mipi_attr.mipi_host_cfg.width;
		height = g_x3_modules_info[i].m_vin_info.mipi_attr.mipi_host_cfg.height;
		fps = g_x3_modules_info[i].m_vin_info.mipi_attr.mipi_host_cfg.fps;

		g_x3_modules_info[i].m_vps_enable = 1;
		g_x3_modules_info[i].m_vps_infos.m_group_num = 1;
		// 配置group的输入
		ret |= vps_grp_param_init(&g_x3_modules_info[i].m_vps_infos.m_vps_info[0],
				width, height);
		// 以下是配置group的每一个通道的参数
		g_x3_modules_info[i].m_vps_infos.m_vps_info[0].m_chn_num = 2;
		ret |= vps_chn_param_init(&g_x3_modules_info[i].m_vps_infos.m_vps_info[0].m_vps_chn_attrs[0],
				2, width, height, fps);
		if (width >= 1920 && height >= 1080) {
			ret |= vps_chn_param_init(&g_x3_modules_info[i].m_vps_infos.m_vps_info[0].m_vps_chn_attrs[1],
					1, 1920, 1080, fps);
			if (g_x3_config.ipc_solution.pipeline_num == 1) { // 单目时配置一个通道用于显示
				g_x3_modules_info[i].m_vps_infos.m_vps_info[0].m_chn_num++;
				if (g_x3_config.disp_dev) { // lcd 的竖屏
					ret |= vps_chn_param_init(&g_x3_modules_info[i].m_vps_infos.m_vps_info[0].m_vps_chn_attrs[2],
						3,
						g_x3_modules_info[0].m_vot_info.m_stLayerAttr.stImageSize.u32Height,
						g_x3_modules_info[0].m_vot_info.m_stLayerAttr.stImageSize.u32Width,
						fps);
				} else { // hdmi 的横屏
					ret |= vps_chn_param_init(&g_x3_modules_info[i].m_vps_infos.m_vps_info[0].m_vps_chn_attrs[2],
						3,
						g_x3_modules_info[0].m_vot_info.m_stLayerAttr.stImageSize.u32Width,
						g_x3_modules_info[0].m_vot_info.m_stLayerAttr.stImageSize.u32Height,
						fps);
				}
				g_x3_modules_info[0].m_vot_enable = 1;
			}
		}

		g_x3_modules_info[i].m_venc_enable = 1;
		g_x3_modules_info[i].m_venc_info.m_chn_num = 1;
		ret = venc_chn_param_init(&g_x3_modules_info[i].m_venc_info.m_venc_chn_info[0],
				0, width, height, fps, g_x3_config.ipc_solution.pipelines[i].venc_bitrate);
	}

	// 3. rgn 配置
	for (i = 0; i < g_x3_config.ipc_solution.pipeline_num; i++) {
		g_x3_modules_info[i].m_rgn_enable = 1;
		ret = x3_rgn_timestamp_param_init(&g_x3_modules_info[i].m_rgn_info, i, 2);
	}

	// 4. bpu算法模型
	for (i = 0; i < g_x3_config.ipc_solution.pipeline_num; i++) {
		if (g_x3_config.ipc_solution.pipelines[i].alog_id != 0) {
			g_x3_modules_info[i].m_bpu_enable = 1;
			g_x3_modules_info[i].m_bpu_info.m_pipeline_id = i;
			g_x3_modules_info[i].m_bpu_info.m_alog_id = g_x3_config.ipc_solution.pipelines[i].alog_id;
		}
	}

	// 如果是双目应用，需要修改pipeline的一些配置
	if (g_x3_config.ipc_solution.pipeline_num > 1) {
		// 修改配置，用于支持多通路
		/* 双目sif-isp之间必须是offline的
		 * 但是双目的时候因为运行了bpu，为了节省带宽，VPS要使用ONLINE模式
		 */
		for (i = 0; i < g_x3_config.ipc_solution.pipeline_num; i++) {
			g_x3_modules_info[i].m_vin_info.vin_vps_mode = VIN_OFFLINE_VPS_ONLINE;

			/* snsinfo 中的port id 必须等于pipeId     */
			g_x3_modules_info[i].m_vin_info.snsinfo.sensorInfo.port = i;
			g_x3_modules_info[i].m_vin_info.snsinfo.sensorInfo.dev_port = i;

			g_x3_modules_info[i].m_vin_info.dev_id = i;
			g_x3_modules_info[i].m_vin_info.pipe_id = i;

			g_x3_modules_info[i].m_vps_infos.m_vps_info[0].m_vps_grp_id = i;

			g_x3_modules_info[i].m_venc_info.m_venc_chn_info[0].m_vps_grp_id = i;
			g_x3_modules_info[i].m_venc_info.m_venc_chn_info[0].m_venc_chn_id = i;
		}
	}

	return ret;
}

static int x3_ipc_init_pipeline(x3_modules_info_t *info)
{
	int ret = 0, i = 0;
	// 1. 初始化 vin，此处调用失败，大概率是因为sensor没有接，或者没有接好，或者sensor库用的版本不配套
	if (info->m_vin_enable) {
		ret = x3_vin_init(&info->m_vin_info);
		if (ret) {
			LOGE_print("x3_vin_init failed, %d", ret);
			goto vin_err;
		}
		LOGI_print("x3_vin_init ok!\n");
	}
	// 2. 初始化 vps
	if (info->m_vps_enable) {
		ret = x3_vps_init_wrap(&info->m_vps_infos.m_vps_info[0]);
		if (ret) {
			LOGE_print("x3_vps_init failed, %d", ret);
			goto vps_err;
		}
		LOGI_print("x3_vps_init_wrap ok!\n");
	}

	// 3. 初始化视频码流编码器
	if (info->m_venc_enable) {
		ret = x3_venc_init_wrap(&info->m_venc_info); // 初始化编码器通道
		if (ret) {
			LOGE_print("x3_venc_init_wrap failed, %d", ret);
			goto venc_err;
		}
		LOGI_print("x3_venc_init_wrap ok!\n");
	}
	// 4. vin -> vps -> venc 进行绑定
	// 4.1 vin bind vps
	if (info->m_vin_enable && info->m_vps_enable) {
		ret = x3_vin_bind_vps(info->m_vin_info.pipe_id, info->m_vps_infos.m_vps_info[0].m_vps_grp_id,
				info->m_vin_info.vin_vps_mode);
		if (ret) {
			LOGE_print("x3_vin_bind_vps failed, %d", ret);
			goto vin_bind_err;
		}
	}
	// 4.2 vps chn bind venc if needed
	if (info->m_vps_enable && info->m_venc_enable) {
		for (i = 0; i < info->m_venc_info.m_chn_num; i++){
			if (info->m_venc_info.m_venc_chn_info[i].m_is_bind) {
				ret = x3_vps_bind_venc(info->m_venc_info.m_venc_chn_info[i].m_vps_grp_id,
						info->m_venc_info.m_venc_chn_info[i].m_vps_chn_id,
						info->m_venc_info.m_venc_chn_info[i].m_venc_chn_id);
				if (ret) {
					LOGE_print("x3_vps_bind_venc failed, %d", ret);
					goto vps_bind_err;
				}
			}
		}
	}

	// 初始化 rgn，显示时间戳osd
	if (info->m_rgn_enable) {
		ret = x3_rgn_init(info->m_rgn_info.m_rgn_handle,
				&info->m_rgn_info.m_rgn_chn, &info->m_rgn_info.m_rgn_attr,
				&info->m_rgn_info.m_rgn_chn_attr);
		if (ret != 0) {
			printf("x3_rgn_init failed, %x\n", ret);
			goto rgn_err;
		}
		LOGI_print("x3_rgn_init ok, info->m_rgn_info.m_rgn_handle = %d", info->m_rgn_info.m_rgn_handle);
	}

	// 初始化算法模块， 从vps的chn5通道get yuv数据，这个通道的数据输出 1920 * 1080数据
	// 初始化bpu
	if (info->m_bpu_enable) {
		info->m_bpu_info.m_bpu_handle = x3_bpu_sample_init(info->m_bpu_info.m_alog_id);
		if (NULL == info->m_bpu_info.m_bpu_handle) {
			LOGE_print("x3_bpu_predict_init failed");
			goto bpu_err;
		}
		// 注册算法结果回调函数
		x3_bpu_predict_callback_register(info->m_bpu_info.m_bpu_handle,
			x3_bpu_predict_general_result_handle, &info->m_bpu_info.m_pipeline_id);
	}

	return ret;

bpu_err:
	if (info->m_rgn_enable) {
		ret = x3_rgn_uninit(info->m_rgn_info.m_rgn_handle,
				&info->m_rgn_info.m_rgn_chn);
		if (ret != 0) {
			LOGE_print("x3_rgn_uninit failed, %d", ret);
			return ret;
		}
		LOGI_print("x3_rgn_uninit ok");
	}

rgn_err:
	if (info->m_vps_enable && info->m_venc_enable) {
		for (i = 0; i < info->m_venc_info.m_chn_num; i++){
			if (info->m_venc_info.m_venc_chn_info[i].m_is_bind) {
				ret = x3_vps_unbind_venc(info->m_venc_info.m_venc_chn_info[i].m_vps_grp_id,
						info->m_venc_info.m_venc_chn_info[i].m_vps_chn_id,
						info->m_venc_info.m_venc_chn_info[i].m_venc_chn_id);
				if (ret) {
					LOGE_print("x3_vps_unbind_venc failed, %d", ret);
					return -2006;
				}
			}
		}
	}

vps_bind_err:
	if (info->m_vin_enable && info->m_vps_enable) {
		ret = x3_vin_unbind_vps(info->m_vin_info.pipe_id, info->m_vps_infos.m_vps_info[0].m_vps_grp_id,
				info->m_vin_info.vin_vps_mode);
		if (ret) {
			LOGE_print("x3_vin_unbind_vps failed, %d", ret);
			return -2005;
		}
	}

vin_bind_err:
	if (info->m_venc_enable) {
		ret = x3_venc_uninit_wrap(&info->m_venc_info);
		if (ret) {
			LOGE_print("x3_venc_uninit_wrap failed, %d", ret);
			return -2004;
		}
	}

venc_err:
	if (info->m_vps_enable) {
		x3_vps_uninit_wrap(&info->m_vps_infos.m_vps_info[0]);
	}
vps_err:
	if (info->m_vin_enable) {
		x3_vin_deinit(&info->m_vin_info);
	}
vin_err:
	return -1;
}

static int x3_ipc_uninit_pipeline(x3_modules_info_t *info)
{
	int ret = 0, i = 0;

	if (info->m_bpu_enable) {
		x3_bpu_predict_callback_unregister(info->m_bpu_info.m_bpu_handle);
		x3_bpu_predict_unint(info->m_bpu_info.m_bpu_handle);
	}

	if (info->m_rgn_enable) {
		ret = x3_rgn_uninit(info->m_rgn_info.m_rgn_handle,
				&info->m_rgn_info.m_rgn_chn);
		if (ret) {
			LOGE_print("x3_rgn_uninit failed, %d", ret);
		}
	}

	if (info->m_vps_enable && info->m_venc_enable) {
		for (i = 0; i < info->m_venc_info.m_chn_num; i++){
			if (info->m_venc_info.m_venc_chn_info[i].m_is_bind) {
				ret = x3_vps_unbind_venc(info->m_venc_info.m_venc_chn_info[i].m_vps_grp_id,
						info->m_venc_info.m_venc_chn_info[i].m_vps_chn_id,
						info->m_venc_info.m_venc_chn_info[i].m_venc_chn_id);
				if (ret) {
					LOGE_print("x3_vps_unbind_venc failed, %d", ret);
				}
			}
		}
	}

	if (info->m_vin_enable && info->m_vps_enable) {
		ret = x3_vin_unbind_vps(info->m_vin_info.pipe_id, info->m_vps_infos.m_vps_info[0].m_vps_grp_id,
				info->m_vin_info.vin_vps_mode);
		if (ret) {
			LOGE_print("x3_vin_unbind_vps failed, %d", ret);
		}
	}

	if (info->m_venc_enable) {
		ret = x3_venc_uninit_wrap(&info->m_venc_info);
		if (ret) {
			LOGE_print("x3_venc_uninit_wrap failed, %d", ret);
		}
	}

	if (info->m_vps_enable) {
		x3_vps_uninit_wrap(&info->m_vps_infos.m_vps_info[0]);
	}

	if (info->m_vin_enable) {
		x3_vin_deinit(&info->m_vin_info);
	}

	return ret;
}

int x3_ipc_init(void)
{
	int ret = 0;
	int idx = 0, i = 0;
	x3_modules_info_t *infos = g_x3_modules_info;
	memset(&g_x3_ipc, 0, sizeof(x3_ipc_t));
	g_x3_ipc.infos = infos;

	/* sdb 生态开发板  ，使能sensor       mclk */
	HB_MIPI_EnableSensorClock(0);
	HB_MIPI_EnableSensorClock(1);

	if (infos[0].m_vot_enable) {
		ret = x3_vot_init(&infos[0].m_vot_info);
		if (ret) {
			LOGE_print("x3_vot_init failed, %d", ret);
			return -1;
		}
		LOGI_print("x3_vot_init ok!");
	}

	// 3. 初始化 venc
	// 这个模块不能多次调用，所以单拿出来执行一次
	ret = x3_venc_common_init(); // 编码模块整体初始化，不清楚是否可以重复调用
	if (ret) {
		LOGE_print("x3_venc_common_init failed(%d)", ret);
		goto venc_common_err;
	}

	// 初始化抓拍jpeg编码通道，jpeg编码默认使用32通道
	ret = x3_jpeg_venc_init(); // 使用isp的输出做为jpeg抓拍的源（VPS OFFILE），也可以使用主码流对应的vps通道的输出做为抓拍源
	if (ret) {
		LOGE_print("x3_jpeg_venc_init failed(%d)", ret);
		goto jpeg_err;
	}

	for(idx = 0; idx < MAX_DEV_NUM; idx++) {
		if (infos[idx].m_enable == 0) continue;

		LOGI_print("x3_ipc init pipeline: %d", idx);
		ret = x3_ipc_init_pipeline(&infos[idx]);
		if (ret) goto init_pipeline_err;
	}

	LOGI_print("x3_ipc_init success");
	return ret;

init_pipeline_err:
	for(i = idx-1; i >= 0; i--) {
		if (infos[i].m_enable == 0) continue;

		x3_ipc_uninit_pipeline(&infos[i]);
	}
jpeg_err:
	x3_venc_common_deinit();
venc_common_err:
	if (infos[0].m_vot_enable)
		x3_vot_deinit();
	return -1;
}

int x3_ipc_uninit(void)
{
	int idx = 0, i = 0;
	shm_stream_t* stm = NULL;
	x3_modules_info_t *infos = g_x3_modules_info;
	for(idx = 0; idx < MAX_DEV_NUM; idx++) {
		if (infos[idx].m_enable == 0) continue;

		x3_ipc_uninit_pipeline(&infos[idx]);
	}
	x3_jpeg_venc_uninit(); // 销毁jpeg编码通道
	x3_venc_common_deinit(); // 单独拿出来执行，只执行一次就行

	if (infos[0].m_vot_enable)
		x3_vot_deinit();

	for (i = 0; i < 32; i++) {
		stm = g_x3_ipc.venc_shms[i];
		if (stm != NULL) {
			shm_stream_destory(stm); // 销毁共享内存读句柄
			g_x3_ipc.venc_shms[i] = NULL;
		}
	}

	print_debug_infos();
	return 0;
}

int x3_ipc_start(void)
{
	int ret = 0, idx = 0, i = 0;
	x3_modules_info_t *infos = g_x3_modules_info;
	for(idx = 0; idx < MAX_DEV_NUM; idx++) {
		if (infos[idx].m_enable == 0) continue;
		// start vps, venc, vin

		if (infos[idx].m_vps_enable) {
			ret = x3_vps_start(infos[idx].m_vps_infos.m_vps_info[0].m_vps_grp_id);
			if (ret) {
				LOGE_print("x3_vps_start failed, %d", ret);
				return -3001;
			}
		}

		if (infos[idx].m_vin_enable) {
			ret = x3_vin_start(&infos[idx].m_vin_info);
			if (ret) {
				LOGE_print("x3_vin_start failed, %d", ret);
				return -3003;
			}
		}

		if (infos[idx].m_venc_enable) {
			for (i = 0; i < infos[idx].m_venc_info.m_chn_num; i++){
				ret = x3_venc_start(infos[idx].m_venc_info.m_venc_chn_info[i].m_venc_chn_id);
				if (ret) {
					LOGE_print("x3_venc_start failed, %d", ret);
					return -3002;
				}
			}
			// 启动获取编码线程
			g_x3_ipc.m_venc_thread[idx].pvThreadData = &infos[idx].m_venc_info;
			mThreadStart(x3_venc_get_stream_proc, &g_x3_ipc.m_venc_thread[idx], E_THREAD_JOINABLE);
		}

		// 启动osd 时间戳线程
		if (infos[idx].m_rgn_enable) {
			g_x3_ipc.m_rgn_thread[idx].pvThreadData = (void*)&infos[idx].m_rgn_info.m_rgn_handle;
			mThreadStart(x3_rgn_set_timestamp_thread, &g_x3_ipc.m_rgn_thread[idx], E_THREAD_JOINABLE);
		}

		if (infos[idx].m_bpu_enable) {
			// 设置bpu后处理的原始图像大小为推流图像大小
			x3_bpu_predict_set_ori_hw(infos[idx].m_bpu_info.m_bpu_handle,
					infos[idx].m_venc_info.m_venc_chn_info[0].m_chn_attr.stVencAttr.u32PicWidth,
					infos[idx].m_venc_info.m_venc_chn_info[0].m_chn_attr.stVencAttr.u32PicHeight);

			x3_bpu_predict_start(infos[idx].m_bpu_info.m_bpu_handle);
			LOGI_print("x3_bpu_predict_start ok!\n");

			// 启动一个线程从 vps    获取yuv数据给bpu运算
			g_x3_ipc.m_bpu_thread[idx].pvThreadData = (void*)&infos[idx];
			mThreadStart(send_yuv_to_bpu, &g_x3_ipc.m_bpu_thread[idx], E_THREAD_JOINABLE);
		}
	}

	if (infos[0].m_vot_enable) {
		g_x3_ipc.m_vot_thread.pvThreadData = (void*)&infos[0];
		mThreadStart(send_yuv_to_vot, &g_x3_ipc.m_vot_thread, E_THREAD_JOINABLE);
	}

	/* 打印个模块配置信息 */
	print_debug_infos();
	return 0;
}

int x3_ipc_stop(void)
{
	int ret = 0, idx = 0, i = 0;
	x3_modules_info_t * infos = g_x3_modules_info;

	if (infos[0].m_vot_enable) {
		mThreadStop(&g_x3_ipc.m_vot_thread);
	}

	for(idx = 0; idx < MAX_DEV_NUM; idx++) {
		if (infos[idx].m_enable == 0) continue;
		if (infos[idx].m_bpu_enable) {
			mThreadStop(&g_x3_ipc.m_bpu_thread[idx]);
			x3_bpu_predict_stop(infos[idx].m_bpu_info.m_bpu_handle);
		}

		// 停止osd线程
		if (infos[idx].m_rgn_enable) {
			mThreadStop(&g_x3_ipc.m_rgn_thread[idx]);
		}

		if (infos[idx].m_venc_enable) {
			mThreadStop(&g_x3_ipc.m_venc_thread[idx]);

			for (i = 0; i < infos[idx].m_venc_info.m_chn_num; i++){
				ret = x3_venc_stop(infos[idx].m_venc_info.m_venc_chn_info[i].m_venc_chn_id);
				if (ret) {
					LOGE_print("x3_venc_stop failed, %d", ret);
					return -4001;
				}
			}
		}
		if (infos[idx].m_vin_enable) {
			x3_vin_stop(&infos[idx].m_vin_info);
		}
		if (infos[idx].m_vps_enable) {
			x3_vps_stop(infos[idx].m_vps_infos.m_vps_info[0].m_vps_grp_id);
		}
	}

	return 0;
}

int x3_ipc_param_set(CAM_PARAM_E type, char* val, unsigned int length)
{
	//有修改的需要修改配置文件
	switch(type)
	{
	case CAM_PARAM_ALL_SET:
		{
			break;
		}
	case CAM_DEC_DATA_PUSH:
		{
			break;
		}
	case CAM_DEC_DATA_CLEAR:
		{

			break;
		}
	case CAM_DEC_FILE_PUSH:
		{
			break;
		}
	case CAM_MOTION_CALLBACK_REG:
		{
			break;
		}
	case CAM_MOTION_CALLBACK_UNREG:
		{
			break;
		}
	case CAM_VENC_FRAMERATE_SET:
		{
			break;
		}
	case CAM_VENC_BITRATE_SET:
		{
			// 只设置主码流的比特率
			x3_venc_set_bitrate(0, *(int *)val);
			break;
		}
	case CAM_VENC_GOP_SET:
		{
			break;
		}
	case CAM_VENC_CVBR_SET:
		{
			break;
		}
	case CAM_VENC_FORCEIDR:
		{
			break;
		}
	case CAM_VENC_MIRROR_SET:
		{
			break;
		}
	case CAM_AENC_VOLUME_SET:
		{
			break;
		}
	case CAM_AENC_MUTE_SET:
		{
			break;
		}
	case CAM_AENC_SAMPLERATE_SET:
		{
			break;
		}
	case CAM_AENC_BITRATE_SET:
		{
			break;
		}
	case CAM_AENC_CHANNLES_SET:
		{
			break;
		}
	case CAM_AENC_AEC_SET:
		{
			break;
		}
	case CAM_AENC_AGC_SET:
		{
			break;
		}
	case CAM_AENC_ANR_SET:
		{
			break;
		}
	case CAM_ADEC_VOLUME_SET:
		{
			break;
		}
	case CAM_ADEC_SAMPLERATE_SET:
		{
			break;
		}
	case CAM_ADEC_BITRATE_SET:
		{
			break;
		}
	case CAM_ADEC_CHANNLES_SET:
		{
			break;
		}
	case CAM_OSD_TIMEENABLE_SET:
		{
			break;
		}
	case CAM_SNAP_PATH_SET:
		{
			break;
		}
	case CAM_SNAP_ONCE:
		{
			break;
		}
	case CAM_MOTION_PARAM_SET:
		{
			break;
		}
	case CAM_YUV_BUFFER_START:
		{
			break;
		}
	case CAM_YUV_BUFFER_STOP:
		{
			break;
		}
	case CAM_MOTION_SENSITIVITY_SET:
		{
			break;
		}
	case CAM_MOTION_AREA_SET:
		{
			break;
		}
	case CAM_GPIO_CTRL_SET:
		{
			break;
		}
	case CAM_ISP_CTRL_MODE_SET:
		{
			break;
		}
	case CAM_GET_RAW_FRAME: {// 先获取0通道的raw和yuv，后面要支持多通道，vin模式需要是offline才行
					char raw_file_name[100] = {0};
					x3_vin_sif_raw_dump(0, raw_file_name);
					// 通知浏览器下载文件
					SDK_Cmd_Impl(SDK_CMD_WEBSOCKET_UPLOAD_FILE, (void*)raw_file_name);
					break;
				}
	case CAM_GET_YUV_FRAME: {
					char yuv_file_name[100] = {0};
					x3_vin_isp_yuv_dump(0, yuv_file_name);
					// 通知浏览器下载文件
					SDK_Cmd_Impl(SDK_CMD_WEBSOCKET_UPLOAD_FILE, (void*)yuv_file_name);
					break;
				}
	case CAM_JPEG_SNAP: {
				    char jpeg_file_name[100] = {0};
				    x3_vin_isp_yuv_dump_to_jpeg(g_x3_ipc.jpeg_venc_chn, 0, jpeg_file_name);
				    // 通知浏览器下载文件
				    SDK_Cmd_Impl(SDK_CMD_WEBSOCKET_UPLOAD_FILE, (void*)jpeg_file_name);
				    break;
			    }
	case CAM_START_RECORDER: // 保存所有编码通道的数据
			    break;
	case CAM_STOP_RECORDER:
			    break;
	default:
			    break;
	}

	return 0;
}

int x3_ipc_param_get(CAM_PARAM_E type, char* val, unsigned int* length)
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
	case CAM_PARAM_STATE_GET:
		{
			break;
		}
	case CAM_PARAM_ALL_GET:
		{
			break;
		}
	case CAM_GPIO_CTRL_GET:
		{
			break;
		}
	case CAM_ADC_CTRL_GET:
		{
			break;
		}
	case CAM_VENC_NTSC_PAL_GET:
		{
			break;
		}
	case CAM_VENC_MIRROR_SET:
		{
			break;
		}
	case CAM_AENC_PARAM_GET:
		{
			break;
		}
	case CAM_MOTION_PARAM_GET:
		{
			break;
		}
	case CAM_RINGBUFFER_CREATE:
		{
			break;
		}
	case CAM_RINGBUFFER_DATA_GET:
		{
			break;
		}
	case CAM_RINGBUFFER_DATA_SYNC:
		{
			break;
		}
	case CAM_RINGBUFFER_DESTORY:
		{
			break;
		}
	case CAM_VENC_CHN_PARAM_GET: // 获取某个编码通道的配置
		{
			venc_info_t* param = (venc_info_t*)val;
			VENC_CHN_ATTR_S vencChnAttr;
			printf("param->channle: %d\n", param->channle);
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
			printf("status: %u\n", *status);
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

