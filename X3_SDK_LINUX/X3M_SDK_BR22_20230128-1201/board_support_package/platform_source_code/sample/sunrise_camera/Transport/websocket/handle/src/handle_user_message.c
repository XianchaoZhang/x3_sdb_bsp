#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "communicate/sdk_common_cmd.h"
#include "communicate/sdk_common_struct.h"
#include "communicate/sdk_communicate.h"

#include "utils/nalu_utils.h"
#include "utils/mthread.h"
#include "utils/utils_log.h"
#include "utils/common_utils.h"
#include "utils/stream_define.h"
#include "utils/stream_manager.h"
#include "utils/cJSON.h"

#include "Handshake.h"
#include "Errors.h"
#include "handle_user_message.h"
#include "Communicate.h"
#include "WebsocketWrap.h"

typedef enum {
	WS_CMD_UNDEFINE = -1,
	WS_CMD_HEARTBEAT,
	WS_CMD_SWITCH_APP,
	WS_CMD_SNAP,
	WS_CMD_START_STREAM,
	WS_CMD_STOP_STREAM,
	WS_CMD_SYNC_TIME,
	WS_CMD_GET_DEV_INFO,
	WS_CMD_SET_BITRATE,
	WS_CMD_SAVE_CONFIG,
	WS_CMD_RECOVERY_CONFIG,
} WS_CMD_KIND;

void ws_send_respose(ws_list *l, ws_client *n, char *msg)
{
	ws_message *m = message_new();
	m->len = strlen(msg);
	m->msg = malloc(sizeof(char)*(m->len+1) );
	memset(m->msg, 0, m->len+1);
	memcpy(m->msg, msg, m->len);
	if ( (encodeMessage(m)) != CONTINUE) {
		message_free(m);
		free(m);
		return;
	}
	list_multicast_one(l, n, m);
	message_free(m);
	free(m);
}

static void *ws_push_stream_thread(void *ptr)
{
	tsThread *privThread = (tsThread*)ptr;
	ws_client *n = (ws_client*)privThread->pvThreadData;
	shm_stream_t *fShmSource = n->fShmSource;
	unsigned int fFrameSize = 0;
	unsigned int fNaluLen = 0;
	frame_info info;
	unsigned int length = 0;
	unsigned char* data = NULL;

	// 设置线程名，方便知道退出的是什么线程
	mThreadSetName(privThread, __func__);

	// 从共享内存中读取码流数据
	while (privThread->eState == E_THREAD_RUNNING) {
		if (shm_stream_front(fShmSource, &info, &data, &length) == 0) {
			NALU_t nalu;
			fFrameSize = length;
			int ret = get_annexb_nalu(data + fNaluLen, fFrameSize - fNaluLen, &nalu);
#if 1
			if (ret < 0) {
				printf("[%s][%d] fShmSource: %p data: %p length: %u fNaluLen: %d readers:%d\n",
						__func__, __LINE__, fShmSource, data, length, fNaluLen, shm_stream_readers(fShmSource));
				fNaluLen = 0;
			}
#endif

			if (ret > 0) fNaluLen += nalu.len + 4;    //记录nalu偏移总长

#if 0
			printf("nal_unit_type:%d data:%p buf:%p len:%u, fNaluLen:%d\n", nalu.nal_unit_type, data,
						   nalu.buf, nalu.len, fNaluLen);
#endif

			//只发送sps pps i p nalu, 其他抛弃
			if (nalu.nal_unit_type == 7 || nalu.nal_unit_type == 8
				|| nalu.nal_unit_type == 1 || nalu.nal_unit_type == 5)
			{
				fFrameSize = nalu.len;

				// 发送数据
				/*printf("send binary: %d\n", nalu.len);*/
				ws_send_binary(n, nalu.buf - 4, nalu.len + 4); // 需要发送带头信息的数据给 wfs
				if (nalu.nal_unit_type == 1 || nalu.nal_unit_type == 5)
				{
					fNaluLen = 0;
					int remains = shm_stream_remains(fShmSource);
					if(remains > 10)
						LOGI_print("fShmSource:%p, framer video pts:%llu length:%d fFrameSize:%d remains:%d",
							fShmSource, info.pts, length, fFrameSize, remains);

					//该帧发送完毕，包括sps pps等nalu拆分完毕，可以释放
					shm_stream_post(fShmSource);
				}
				// 会出现只有 7 和 8 类型的包
				if ((nalu.nal_unit_type == 7 || nalu.nal_unit_type == 8) && length == nalu.len + 4) {
					fNaluLen = 0;
					//该帧发送完毕，包括sps pps等nalu拆分完毕，可以释放
					shm_stream_post(fShmSource);
				}
			}
			// 调试过程中遇到出现 type == 23 的情况，不解析直接抛弃掉
			else {
				fNaluLen = 0;
				shm_stream_post(fShmSource);
			}
		}
		// 休眠
		usleep(20000);
	}
	if (fShmSource != NULL) {
		shm_stream_destory(fShmSource); // 销毁共享内存读句柄
		n->fShmSource = NULL;
	}
	mThreadFinish(privThread);
	return NULL;
}

static int _do_start_stream(ws_client *n, int channel)
{
	int ret = 0;
	T_SDK_VENC_INFO venc_chn_info;

	// 从camera模块获取chn0的码流配置
	venc_chn_info.channel = channel; // venc 的 channle 需要作为入参，这个应该通过camera模块的参数get出来，这里先写死好了
	ret = SDK_Cmd_Impl(SDK_CMD_CAMERA_VENC_CHN_PARAM_GET, (void*)&venc_chn_info);
	if(ret < 0)
	{
		printf("SDK_Cmd_Impl: SDK_CMD_CAMERA_VENC_CHN_PARAM_GET Error, ERRCODE: %d\n", ret);
		return -1;
	}

	printf("[%s][%d] venc chn %d id %s, type: %d, frameRate: %f\n", __func__, __LINE__, venc_chn_info.channel,
		venc_chn_info.enable == 1 ? "enable" : "disable", venc_chn_info.type,
		venc_chn_info.framerate);

	char shm_id[32] = {0}, shm_name[32] = {0};
	int type = venc_chn_info.type;
	sprintf(shm_id, "ws%d_id_%s_chn%d", n->socket_id, type == 96 ? "h264" : 
								(type == 265 ? "h264" : 
								(type == 26) ? "jpeg" : "other"), venc_chn_info.channel);
	sprintf(shm_name, "name_%s_chn%d", type == 96 ? "h264" : 
								(type == 265 ? "h264" : 
								(type == 26) ? "jpeg" : "other"), venc_chn_info.channel);
	n->fShmSource = shm_stream_create(shm_id, shm_name,
		STREAM_MAX_USER, venc_chn_info.framerate,
		venc_chn_info.stream_buf_size,
		SHM_STREAM_READ, SHM_STREAM_MALLOC);

	memset(&n->stream_thread, 0, sizeof(tsThread));
	n->stream_thread.pvThreadData = (void*)n;
	mThreadStart(ws_push_stream_thread, &n->stream_thread, E_THREAD_JOINABLE);
	return 0;
}

static int _do_add_sms(int channel)
{
	int ret = 0;
	T_SDK_VENC_INFO venc_chn_info;
	// 从camera模块获取chn0的码流配置
	venc_chn_info.channel = channel; // venc 的 channle 需要作为入参，这个应该通过camera模块的参数get出来，这里先写死好了
	ret = SDK_Cmd_Impl(SDK_CMD_CAMERA_VENC_CHN_PARAM_GET, (void*)&venc_chn_info);
	if(ret < 0)
	{
		printf("SDK_Cmd_Impl: SDK_CMD_CAMERA_VENC_CHN_PARAM_GET Error, ERRCODE: %d", ret);
		return -1;
	}

	printf("venc chn %d id %s, type: %d, frameRate: %f\n", venc_chn_info.channel,
		venc_chn_info.enable == 1 ? "enable" : "disable", venc_chn_info.type,
		venc_chn_info.framerate);

	T_SDK_RTSP_SRV_PARAM sms_param = { 0 };
	int type = venc_chn_info.type;

	sprintf(sms_param.prefix, "stream_chn%d.h264", venc_chn_info.channel);

	sms_param.audio.enable = 0;

	sms_param.video.enable = 1;
	if (type == 96)
		sms_param.video.type = T_SDK_RTSP_VIDEO_TYPE_H264;
	else
		sms_param.video.type = T_SDK_RTSP_VIDEO_TYPE_H264; // 目前只支持H264
	sms_param.video.framerate = 30;
	sprintf(sms_param.shm_id, "rtsp_id_%s_chn%d", type == 96 ? "h264" : 
								(type == 265 ? "h264" : 
								(type == 26) ? "jpeg" : "other"), venc_chn_info.channel);
	sprintf(sms_param.shm_name, "name_%s_chn%d", type == 96 ? "h264" : 
								(type == 265 ? "h264" : 
								(type == 26) ? "jpeg" : "other"), venc_chn_info.channel);
	sms_param.stream_buf_size = venc_chn_info.stream_buf_size;
	sms_param.video.framerate = venc_chn_info.framerate;
	/*printf("framerate:%d\n", sms_param.video.framerate);*/
	ret = SDK_Cmd_Impl(SDK_CMD_RTSP_SERVER_ADD_SMS, (void*)&sms_param);
	if(ret < 0)
	{
		printf("SDK_Cmd_Impl: SDK_CMD_RTSP_SERVER_START Error, ERRCODE: %d", ret);
		return -1;
	}
	return ret;
}

int handle_user_msg(ws_list *l, ws_client *n, char *msg)
{
	int i, ret = 0;
	cJSON *root = cJSON_Parse(msg);
	WS_CMD_KIND cmd_kind = WS_CMD_UNDEFINE;
	char cmd_context[2048] = {0};
	char ws_msg[2048 + 64] = {0};
	
	/*printf("%s\n", cJSON_Print(root));*/
	if (root == NULL) return -1;

	cmd_kind = cJSON_GetObjectItem(root, "kind")->valueint;

	switch (cmd_kind)
	{
		case WS_CMD_HEARTBEAT:
			// do nothing
			break;
		case WS_CMD_SWITCH_APP:
			strcpy(cmd_context, cJSON_GetObjectItem(root, "param")->valuestring);
			printf("cmd_context: %s\n", cmd_context);
			// 1. 先stop、反初始化vin 、isp、vps、 venc 和 rtps 删除sms
			printf("===================== DEL SMS =======================\n");
			SDK_Cmd_Impl(SDK_CMD_RTSP_SERVER_DEL_SMS, NULL);
			/*SDK_Cmd_Impl(SDK_CMD_RTSP_SERVER_STOP, NULL);*/
			printf("===================== STOP CAMERA =======================\n");
			SDK_Cmd_Impl(SDK_CMD_CAMERA_STOP, NULL);
			printf("===================== UNINIT CAMERA =======================\n");
			SDK_Cmd_Impl(SDK_CMD_CAMERA_UNINIT, NULL);

			printf("===================== Start new APP =======================\n");
			// 2. 更新配置结构体
			SDK_Cmd_Impl(SDK_CMD_CAMERA_SET_SOLUTION_CONFIG, (void *)cmd_context);

			// 3. 开始启动应用
			ret = SDK_Cmd_Impl(SDK_CMD_CAMERA_INIT, NULL);
			if(ret < 0)
			{
				printf("SDK_Cmd_Impl: SDK_CMD_CAMERA_INIT Error, ERRCODE: %d\n", ret);
				ws_send_respose(l, n, "{\"kind\":1,\"app_status\": \"请检查sensor是否连接正常\"}");
				return -1;
			}
			usleep(500*1000);
			ret = SDK_Cmd_Impl(SDK_CMD_CAMERA_START, NULL);
			if(ret < 0)
			{
				printf("SDK_Cmd_Impl: SDK_CMD_CAMERA_START Error, ERRCODE: %d\n", ret);
				ws_send_respose(l, n, "{\"kind\":1,\"app_status\": \"请检查sensor是否连接正常\"}");
				return -1;
			}
			usleep(500*1000);
			// 根据编码通道的配置添加推流
			unsigned int venc_chns_status = 0;
			SDK_Cmd_Impl(SDK_CMD_CAMERA_GET_VENC_CHN_STATUS, (void*)&venc_chns_status);
			printf("venc_chns_status: %u\n", venc_chns_status);
			for (i = 0; i < 32; i++) {
				if (venc_chns_status & (1 << i))
					_do_add_sms(i); // 给对应的编码数据建立rtsp推流sms
			}
			ws_send_respose(l, n, "{\"kind\":1,\"Status\":\"200\"}");
			/*_do_start_stream(n, 0);*/
			break;
		case WS_CMD_SNAP:
			strcpy(cmd_context, cJSON_GetObjectItem(root, "snap_type")->valuestring);
			printf("WS_CMD_SNAP type: %s\n", cmd_context);
			int pipe_dev_id = 0;
			if (strncmp(cmd_context, "raw", strlen(cmd_context)) == 0) {
				ret = SDK_Cmd_Impl(SDK_CMD_CAMERA_GET_RAW_FRAME, (void *)&pipe_dev_id);
			}
			else if (strncmp(cmd_context, "yuv", strlen(cmd_context)) == 0) {
				ret = SDK_Cmd_Impl(SDK_CMD_CAMERA_GET_YUV_FRAME, (void *)&pipe_dev_id);
			}
			else if (strncmp(cmd_context, "jpeg", strlen(cmd_context)) == 0) {
				ret = SDK_Cmd_Impl(SDK_CMD_CAMERA_JPEG_SNAP, (void *)&pipe_dev_id);
			}
			else {
				printf("WS cmder undefined!\n");
			}
			
			if(ret < 0)
			{
				printf("SDK_Cmd_Impl Error, ERRCODE: %d", ret);
				return -1;
			}
			break;
		case WS_CMD_START_STREAM:
			// 避免重复拉流，重复拉流的原因不太清楚，应该是web上有并发事件
			if (n->fShmSource)
				break;

			printf("start ws venc chn%d stream\n", cJSON_GetObjectItem(root, "stream_chn")->valueint);
			ret = _do_start_stream(n, cJSON_GetObjectItem(root, "stream_chn")->valueint);
			if (ret < 0) {
				printf("start websocket push stream failed\n");
			}
			break;
		case WS_CMD_STOP_STREAM:
			printf("stop ws venc chn%d stream\n", cJSON_GetObjectItem(root, "stream_chn")->valueint);
			mThreadStop(&n->stream_thread);
			break;
		case WS_CMD_SYNC_TIME:
			printf("sync pc time to : %d\n", cJSON_GetObjectItem(root, "time")->valueint);
			long int pc_t = cJSON_GetObjectItem(root, "time")->valueint;
#if __GLIBC_MINOR__ == 31
			struct timespec res;
			res.tv_sec = pc_t;
			clock_settime(CLOCK_REALTIME,&res);
#else
			stime(&pc_t);
#endif
			break;
		case WS_CMD_GET_DEV_INFO:
		{
			
			// 获取sensor list
			unsigned int sensor_list = 0;
			SDK_Cmd_Impl(SDK_CMD_CAMERA_GET_SENSOR_LIST, (void*)&sensor_list);
			printf("sensor_list: %u\n", sensor_list);
			memset(ws_msg, '\0', sizeof(ws_msg));
			sprintf(ws_msg, "{\"kind\":6,\"sensor_list\": %u}", sensor_list);
			ws_send_respose(l, n, ws_msg);

			// 获取场景配置
			memset(ws_msg, '\0', sizeof(ws_msg));
			char config_str[2048] = {0};
			SDK_Cmd_Impl(SDK_CMD_CAMERA_GET_SOLUTION_CONFIG, (void *)config_str);
			/*printf("config_str: %s\n", config_str);*/
			sprintf(ws_msg, "{\"kind\":6,\"solution_configs\": %s}", config_str);
			ws_send_respose(l, n, ws_msg);
			
			memset(ws_msg, '\0', sizeof(ws_msg));
			sprintf(ws_msg, "{\"kind\":6,\"app_version\": \"sunrise camera build time:%s %s\"}",
				__DATE__, __TIME__);
			ws_send_respose(l, n, ws_msg);
			T_SDK_CHIP_TYPE dev_info;
			ret = SDK_Cmd_Impl(SDK_CMD_CAMERA_GET_CHIP_TYPE, (void*)&dev_info);
			if(ret < 0)
			{
				printf("SDK_Cmd_Impl: SDK_CMD_CAMERA_GET_CHIP_TYPE Error, ERRCODE: %d\n", ret);
				return -1;
			}
			memset(ws_msg, '\0', sizeof(ws_msg));
			sprintf(ws_msg, "{\"kind\":6,\"chip_type\": \"%s\"}",dev_info.chip_type);
			ws_send_respose(l, n, ws_msg);

			break;
		}
		case WS_CMD_SET_BITRATE:
		{
			int bitrate = cJSON_GetObjectItem(root, "bitrate")->valueint;
			LOGI_print("bitrate = %d", bitrate);
			SDK_Cmd_Impl(SDK_CMD_CAMERA_VENC_BITRATE_SET, (void*)&bitrate);
			break;
		}
		case WS_CMD_SAVE_CONFIG:
		{
			char cfg_str[2048] = {0};
			strcpy(cfg_str, cJSON_GetObjectItem(root, "param")->valuestring);
			SDK_Cmd_Impl(SDK_CMD_CAMERA_SAVE_SOLUTION_CONFIG, (void *)cfg_str);
			break;
		}
		case WS_CMD_RECOVERY_CONFIG:
		{
			memset(ws_msg, '\0', sizeof(ws_msg));
			char config_str[2048] = {0};
			SDK_Cmd_Impl(SDK_CMD_CAMERA_RECOVERY_SOLUTION_CONFIG, (void *)config_str);
			/*printf("config_str: %s\n", config_str);*/
			sprintf(ws_msg, "{\"kind\":6,\"solution_configs\": %s}", config_str);
			ws_send_respose(l, n, ws_msg);
			break;
		}
		case WS_CMD_UNDEFINE:
		default:
			printf("WS cmder undefined!\n");
	}

	if (root)
		cJSON_free(root);

	return 0;
}

