#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "api_camera.h"
#include "camera_handle.h"
#include "communicate/sdk_communicate.h"
#include "communicate/sdk_common_cmd.h"
#include "communicate/sdk_common_struct.h"
#include "utils/utils_log.h"

int camera_cmd_impl(SDK_CMD_E cmd, void* param);

static sdk_cmd_reg_t cmd_reg[] = 
{
	{SDK_CMD_CAMERA_RINGBUFFER_CREATE,		camera_cmd_impl,				1},
	{SDK_CMD_CAMERA_RINGBUFFER_DATA_GET,	camera_cmd_impl,				1},
	{SDK_CMD_CAMERA_RINGBUFFER_DATA_SYNC,	camera_cmd_impl,				1},
	{SDK_CMD_CAMERA_RINGBUFFER_DESTORY,		camera_cmd_impl,				1},
	
	{SDK_CMD_CAMERA_INIT,			camera_cmd_impl,				1},
	{SDK_CMD_CAMERA_UNINIT,			camera_cmd_impl,				1},
	{SDK_CMD_CAMERA_START,			camera_cmd_impl,				1},
	{SDK_CMD_CAMERA_STOP,			camera_cmd_impl,				1},
	{SDK_CMD_CAMERA_PARAM_SET,		camera_cmd_impl,				1},
	{SDK_CMD_CAMERA_PARAM_GET,		camera_cmd_impl,				1},
	{SDK_CMD_CAMERA_MIC_SWITCH,		camera_cmd_impl,				1},
	{SDK_CMD_CAMERA_AENC_PARAM_GET,	camera_cmd_impl,				1},
	{SDK_CMD_CAMERA_AENC_PARAM_SET,	camera_cmd_impl,				1},
	{SDK_CMD_CAMERA_MOTION_PARAM_GET,camera_cmd_impl,				1},
	{SDK_CMD_CAMERA_MOTION_PARAM_SET,camera_cmd_impl,				1},
	{SDK_CMD_CAMERA_YUV_BUFFER_START,camera_cmd_impl,				1},
	{SDK_CMD_CAMERA_YUV_BUFFER_STOP,camera_cmd_impl,				1},
	{SDK_CMD_CAMERA_VENC_MIRROR_GET,camera_cmd_impl,				1},
	{SDK_CMD_CAMERA_VENC_MIRROR_SET,camera_cmd_impl,				1},
	{SDK_CMD_CAMERA_VENC_FORCE_IDR, camera_cmd_impl,				1},
	{SDK_CMD_CAMERA_VENC_SNAP, 		camera_cmd_impl,				1},
	{SDK_CMD_CAMERA_VENC_ALL_PARAM_GET,	camera_cmd_impl,			1},
	{SDK_CMD_CAMERA_VENC_CHN_PARAM_GET,	camera_cmd_impl,			1},
	{SDK_CMD_CAMERA_GET_VENC_CHN_STATUS,camera_cmd_impl,			1},
	{SDK_CMD_CAMERA_SENSOR_NTSC_PAL_GET,	camera_cmd_impl,		1},
	{SDK_CMD_CAMERA_GPIO_STATE_SET, camera_cmd_impl,				1},
	{SDK_CMD_CAMERA_GPIO_STATE_GET, camera_cmd_impl,				1},
	{SDK_CMD_CAMERA_ADC_VALUE_GET, 	camera_cmd_impl,				1},
	{SDK_CMD_CAMERA_ISP_MODE_SET, 	camera_cmd_impl,				1},
	{SDK_CMD_CAMERA_ADEC_DATA_PUSH, camera_cmd_impl,				1},
	{SDK_CMD_CAMERA_GET_RAW_FRAME, camera_cmd_impl,					1},
	{SDK_CMD_CAMERA_GET_YUV_FRAME, camera_cmd_impl,					1},
	{SDK_CMD_CAMERA_JPEG_SNAP, camera_cmd_impl,						1},
	{SDK_CMD_CAMERA_START_RECORD, camera_cmd_impl,					1},
	{SDK_CMD_CAMERA_STOP_RECORD, camera_cmd_impl,					1},
	{SDK_CMD_CAMERA_GET_CHIP_TYPE, camera_cmd_impl,					1},
	{SDK_CMD_CAMERA_VENC_BITRATE_SET,  camera_cmd_impl,				1},
	{SDK_CMD_CAMERA_GET_SENSOR_LIST,  camera_cmd_impl,				1},
	{SDK_CMD_CAMERA_GET_SOLUTION_CONFIG,  camera_cmd_impl,			1},
	{SDK_CMD_CAMERA_SET_SOLUTION_CONFIG,  camera_cmd_impl,			1},
	{SDK_CMD_CAMERA_SAVE_SOLUTION_CONFIG,  camera_cmd_impl,			1},
	{SDK_CMD_CAMERA_RECOVERY_SOLUTION_CONFIG,  camera_cmd_impl,		1},
};

int camera_cmd_register()
{
	int i;
	for(i=0; i<sizeof(cmd_reg)/sizeof(cmd_reg[0]); i++)
	{
		sdk_cmd_register(cmd_reg[i].cmd, cmd_reg[i].call, cmd_reg[i].enable);
	}

	return 0;
}

int camera_cmd_impl(SDK_CMD_E cmd, void* param)
{
	int ret = 0;
	switch(cmd)
	{
		case SDK_CMD_CAMERA_INIT:
		{
			ret = camera_handle_init();
			break;
		}
		case SDK_CMD_CAMERA_UNINIT:
		{
			ret = camera_handle_uninit();
			break;
		}
		case SDK_CMD_CAMERA_START:
		{
			ret = camera_handle_start();
			break;
		}
		case SDK_CMD_CAMERA_STOP:
		{
			ret = camera_handle_stop();
			break;
		}
		case SDK_CMD_CAMERA_GET_CHIP_TYPE:
		{
			T_SDK_CHIP_TYPE* p = (T_SDK_CHIP_TYPE*)param;
			unsigned int length = 16;
			ret = camera_handle_param_get(CAM_GET_CHIP_TYPE, (char*)p->chip_type, &length);
			break;
		}
		case SDK_CMD_CAMERA_GET_SENSOR_LIST:
		{
			*(unsigned int *)param = camera_handle_get_sensor_list();
			break;
		}
		case SDK_CMD_CAMERA_GET_SOLUTION_CONFIG:
		{
			ret = camera_handle_get_solution_config((char *)param);
			break;
		}
		case SDK_CMD_CAMERA_SET_SOLUTION_CONFIG:
		{
			ret = camera_handle_set_solution_config((char *)param);
			break;
		}
		case SDK_CMD_CAMERA_SAVE_SOLUTION_CONFIG:
		{
			ret = camera_handle_save_solution_config((char *)param);
			break;
		}
		case SDK_CMD_CAMERA_RECOVERY_SOLUTION_CONFIG:
		{
			ret = camera_handle_recovery_solution_config((char *)param);
			break;
		}
		case SDK_CMD_CAMERA_AENC_PARAM_GET:
		{
			T_SDK_CAM_AENC_PARAM* p = (T_SDK_CAM_AENC_PARAM*)param;
			aenc_info_t	info;
			unsigned int length = sizeof(aenc_info_t);
			ret = camera_handle_param_get(CAM_AENC_PARAM_GET, (char*)&info, &length);
			if(ret != -1)
			{
				memcpy(p, &info, sizeof(T_SDK_CAM_AENC_PARAM));
			}
			break;
		}
		case SDK_CMD_CAMERA_MOTION_PARAM_GET:
		{
			T_SDK_CAM_MD_PARAM* p = (T_SDK_CAM_MD_PARAM*)param;
			motion_info_t info;
			unsigned int length = sizeof(motion_info_t);
			ret = camera_handle_param_get(CAM_MOTION_PARAM_GET, (char*)&info, &length);
			if(ret != -1)
			{
				p->enable = info.enable;
				p->sensitivity = info.sensitivity;
				p->sustain = info.sustain;
				p->rect_enable = info.rect_enable;
				p->width = info.width;
				p->height = info.height;
			}
			break;
		}
		case SDK_CMD_CAMERA_MOTION_PARAM_SET:
		{
			T_SDK_CAM_MD_PARAM* p = (T_SDK_CAM_MD_PARAM*)param;
			motion_info_t info;
			unsigned int length = sizeof(motion_info_t);
			camera_handle_param_get(CAM_MOTION_PARAM_GET, (char*)&info, &length);
			
			info.enable = p->enable;
			info.sensitivity = p->sensitivity;
			info.sustain = p->sustain;
			info.rect_enable = p->rect_enable;
//			info.width = p->width;
//			info.height = p->height;
			ret = camera_handle_param_set(CAM_MOTION_PARAM_SET, (char*)&info, length);
			break;
		}
		case SDK_CMD_CAMERA_YUV_BUFFER_START:
		{
			T_SDK_CAM_YUV_BUFFER* p = (T_SDK_CAM_YUV_BUFFER*)param;
			cam_yuv_buffer_t yuv_buffer;
			unsigned int length = sizeof(cam_yuv_buffer_t);
			memcpy(&yuv_buffer, p, sizeof(T_SDK_CAM_YUV_BUFFER));
			ret = camera_handle_param_set(CAM_YUV_BUFFER_START, (char*)&yuv_buffer, length);
			break;
		}
		case SDK_CMD_CAMERA_YUV_BUFFER_STOP:
		{
			ret = camera_handle_param_set(CAM_YUV_BUFFER_STOP, NULL, 0);
			break;
		}
		case SDK_CMD_CAMERA_ADEC_FILE_PUSH:
		{
			T_SDK_DEC_FILE* pData = (T_SDK_DEC_FILE*)param;
			camera_handle_param_set(CAM_DEC_FILE_PUSH, pData->file, strlen(pData->file));
			break;
		}
		case SDK_CMD_CAMERA_VENC_FORCE_IDR:
		{
			T_SDK_FORCE_I_FARME* pData = (T_SDK_FORCE_I_FARME*)param;
			cam_forceidr_t idr;
			idr.channle = pData->channel;
			idr.count = pData->num;
			ret = camera_handle_param_set(CAM_VENC_FORCEIDR, (char*)&idr, sizeof(cam_forceidr_t));
			break;
		}
		case SDK_CMD_CAMERA_VENC_SNAP:
		{
			T_SDK_CAM_SNAP* pData = (T_SDK_CAM_SNAP*)param;
			ret = camera_handle_param_set(CAM_SNAP_ONCE, pData->file, strlen(pData->file));
			break;
		}
		case SDK_CMD_CAMERA_ADEC_DATA_PUSH:
		{
			T_SDK_DEC_DATA* pData = (T_SDK_DEC_DATA*)param;

			ret = camera_handle_param_set(CAM_DEC_DATA_PUSH, (char*)(pData->cp_data), pData->un_data_len);
			break;
		}
		case SDK_CMD_CAMERA_GPIO_STATE_SET:
		{
			T_SDK_GPIO*	gpio = (T_SDK_GPIO*)param;
//			LOGI_print("un_type:%d un_value:%d", gpio->un_type, gpio->un_value);
			cam_gpoi_ctrl_t gpoi_ctrl;
			gpoi_ctrl.type = gpio->un_type;
			gpoi_ctrl.val = gpio->un_value;
			ret = camera_handle_param_set(CAM_GPIO_CTRL_SET, (char*)&gpoi_ctrl, sizeof(cam_gpoi_ctrl_t));
			break;
		}
		case SDK_CMD_CAMERA_GPIO_STATE_GET:
		{
			T_SDK_GPIO*	gpio = (T_SDK_GPIO*)param;
//			LOGI_print("un_type:%d", gpio->un_type);
			cam_gpoi_ctrl_t gpoi_ctrl;
			unsigned int lenght = sizeof(cam_gpoi_ctrl_t);
			gpoi_ctrl.type = gpio->un_type;
			ret = camera_handle_param_get(CAM_GPIO_CTRL_GET, (char*)&gpoi_ctrl, &lenght);
			if(ret != -1)
			{
				gpio->un_value = gpoi_ctrl.val;
			}
				
			break;
		}
		case SDK_CMD_CAMERA_ADC_VALUE_GET:
		{
			T_SDK_ADC* adc = (T_SDK_ADC*)param;
//			LOGI_print("un_type:%d", gpio->un_type);
			cam_adc_ctrl_t adc_ctrl;
			unsigned int lenght = sizeof(cam_adc_ctrl_t);
			adc_ctrl.ch = adc->un_ch;
			ret = camera_handle_param_get(CAM_ADC_CTRL_GET, (char*)&adc_ctrl, &lenght);
			if(ret != -1)
			{
				adc->un_value = adc_ctrl.val;
			}
			break;
		}
		case SDK_CMD_CAMERA_ISP_MODE_SET:
		{
			T_SDK_ISP_MODE* p = (T_SDK_ISP_MODE*)param;
			ret = camera_handle_param_set(CAM_ISP_CTRL_MODE_SET, (char*)(&p->un_mode), sizeof(unsigned int));
			break;
		}
		case SDK_CMD_CAMERA_VENC_MIRROR_GET:
		{
			T_SDK_CAM_MIRROR* p = (T_SDK_CAM_MIRROR*)param;
			int mode;
			unsigned int length = sizeof(int);
			ret = camera_handle_param_get(CAM_VENC_MIRROR_SET, (char*)&mode, &length);
			if(ret == 0)
			{
				p->mode = mode;
			}
			break;
		}
		case SDK_CMD_CAMERA_VENC_MIRROR_SET:
		{
			T_SDK_CAM_MIRROR* p = (T_SDK_CAM_MIRROR*)param;
			int mode = p->mode;
			ret = camera_handle_param_set(CAM_VENC_MIRROR_SET, (char*)&mode, sizeof(int));
			break;
		}
		case SDK_CMD_CAMERA_VENC_CHN_PARAM_GET:
		{
			T_SDK_VENC_INFO* p = (T_SDK_VENC_INFO*)param;
			unsigned int length = sizeof(T_SDK_VENC_INFO);
			ret = camera_handle_param_get(CAM_VENC_CHN_PARAM_GET, (char*)p, &length);
			break;
		}
		case SDK_CMD_CAMERA_GET_VENC_CHN_STATUS:
		{
			unsigned int* p = (unsigned int*)param;
			unsigned int length = sizeof(unsigned int);
			ret = camera_handle_param_get(CAM_GET_VENC_CHN_STATUS, (char*)p, &length);
			break;
		}
		case SDK_CMD_CAMERA_SENSOR_NTSC_PAL_GET:
		{
			T_SDK_SENSOR_NTSC_PAL* p = (T_SDK_SENSOR_NTSC_PAL*)param;
			int ntsc_pal;
			unsigned int length = sizeof(int);
			ret = camera_handle_param_get(CAM_VENC_NTSC_PAL_GET, (char*)&ntsc_pal, &length);
			if(ret == 0)
			{
				p->hz = (unsigned int)ntsc_pal;
			}
			else
			{
				p->hz = 0;
			}
			break;
		}
		case SDK_CMD_CAMERA_MIC_SWITCH:
		{
			T_SDK_CAM_MIC_SWITCH* p = (T_SDK_CAM_MIC_SWITCH*)param;
			int mute = p->type;
			camera_handle_param_set(CAM_AENC_MUTE_SET, (char*)&mute, sizeof(int));
			break;
		}
		case SDK_CMD_CAMERA_RINGBUFFER_CREATE:
		{
			T_SDK_CAM_RINGBUFFER* p = (T_SDK_CAM_RINGBUFFER*)param;
			unsigned int length = sizeof(T_SDK_CAM_RINGBUFFER);
			ret = camera_handle_param_get(CAM_RINGBUFFER_CREATE, (char*)p, &length);
			break;
		}
		case SDK_CMD_CAMERA_RINGBUFFER_DATA_GET:
		{
			T_SDK_CAM_RINGBUFFER_DATA* p = (T_SDK_CAM_RINGBUFFER_DATA*)param;
			unsigned int length = sizeof(T_SDK_CAM_RINGBUFFER_DATA);
			ret = camera_handle_param_get(CAM_RINGBUFFER_DATA_GET, (char*)p, &length);
			break;
		}
		case SDK_CMD_CAMERA_RINGBUFFER_DATA_SYNC:
		{
			T_SDK_CAM_RINGBUFFER* p = (T_SDK_CAM_RINGBUFFER*)param;
			unsigned int length = sizeof(T_SDK_CAM_RINGBUFFER);
			ret = camera_handle_param_get(CAM_RINGBUFFER_DATA_SYNC, (char*)p, &length);
			break;
		}
		case SDK_CMD_CAMERA_RINGBUFFER_DESTORY:
		{
			T_SDK_CAM_RINGBUFFER* p = (T_SDK_CAM_RINGBUFFER*)param;
			unsigned int length = sizeof(T_SDK_CAM_RINGBUFFER);
			ret = camera_handle_param_get(CAM_RINGBUFFER_DESTORY, (char*)p, &length);
			break;
		}
		case SDK_CMD_CAMERA_GET_RAW_FRAME:
			ret = camera_handle_param_set(CAM_GET_RAW_FRAME, (char*)param, sizeof(int));
			break;
		case SDK_CMD_CAMERA_GET_YUV_FRAME:
			ret = camera_handle_param_set(CAM_GET_YUV_FRAME, (char*)param, sizeof(int));
			break;
		case SDK_CMD_CAMERA_JPEG_SNAP:
			ret = camera_handle_param_set(CAM_JPEG_SNAP, (char*)param, sizeof(int));
			break;
		case SDK_CMD_CAMERA_START_RECORD:
			ret = camera_handle_param_set(CAM_START_RECORDER, (char*)param, sizeof(int));
			break;
		case SDK_CMD_CAMERA_STOP_RECORD:
			ret = camera_handle_param_set(CAM_STOP_RECORDER, (char*)param, sizeof(int));
			break;
		case SDK_CMD_CAMERA_VENC_BITRATE_SET:
			ret = camera_handle_param_set(CAM_VENC_BITRATE_SET, (char*)param, sizeof(int));
			break;
		default:
			ret = -1;
			LOGE_print("unknow cmd:%d", cmd);
			break;
	}

	return ret;
}
