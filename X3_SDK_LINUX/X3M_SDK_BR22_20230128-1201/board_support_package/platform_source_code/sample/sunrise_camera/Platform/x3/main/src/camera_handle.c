#include <stdio.h>
#include <string.h>

#include "camera_handle.h"
#include "utils/utils_log.h"
#include "utils/stream_define.h"
#include "utils/stream_manager.h"

#include "communicate/sdk_communicate.h"
#include "communicate/sdk_common_cmd.h"
#include "utils/common_utils.h"

#include "x3_ipc_impl.h"
#include "x3_box_impl.h"
#include "x3_usb_cam_impl.h"
#include "x3_utils.h"
#include "x3_config.h"

#include "camera_handle.h"

// 这个文件中的接口后面都做成注册接口，实现对更底层sensor的封装
// 后面每个sensor或者应用示例实现一套接口就能对接上

typedef struct
{
	volatile enum
	{
		E_STATE_STARTED,
		E_STATE_RUNNING,
		E_STATE_STOPPING,
		E_STATE_STOPPED,
	} m_state;
	int				m_solution_id;
	camera_ops_t	*impl;
} camera_handle_t;

static hard_capability_t g_hard_capability;

static camera_handle_t *g_camera_handle = NULL;

static camera_ops_t *x3_ipc_generic_impl(void)
{
	camera_ops_t *impl = malloc(sizeof(camera_ops_t));

	memset(impl, 0, sizeof(camera_ops_t));
	impl->init_param = x3_ipc_init_param;
	impl->init = x3_ipc_init;
	impl->uninit = x3_ipc_uninit;
	impl->start = x3_ipc_start;
	impl->stop = x3_ipc_stop;
	impl->param_get = x3_ipc_param_get;
	impl->param_set = x3_ipc_param_set;
	return impl;
}

static camera_ops_t *x3_box_generic_impl(void)
{
	camera_ops_t *impl = malloc(sizeof(camera_ops_t));

	memset(impl, 0, sizeof(camera_ops_t));
	impl->init_param = x3_box_init_param;
	impl->init = x3_box_init;
	impl->uninit = x3_box_uninit;
	impl->start = x3_box_start;
	impl->stop = x3_box_stop;
	impl->param_get = x3_box_param_get;
	impl->param_set = x3_box_param_set;
	return impl;
}

static camera_ops_t *x3_usb_cam_generic_impl(void)
{
	camera_ops_t *impl = malloc(sizeof(camera_ops_t));

	memset(impl, 0, sizeof(camera_ops_t));
	impl->init_param = x3_usb_cam_init_param;
	impl->init = x3_usb_cam_init;
	impl->uninit = x3_usb_cam_uninit;
	impl->start = x3_usb_cam_start;
	impl->stop = x3_usb_cam_stop;
	impl->param_get = x3_usb_cam_param_get;
	impl->param_set = x3_usb_cam_param_set;
	return impl;
}

// 根据配置中的solution配置对应的接口函数
static int _camera_handle_init(camera_handle_t *handle)
{
	memset(handle, 0, sizeof(camera_handle_t));

	handle->m_solution_id = g_x3_config.solution_id;

	switch (handle->m_solution_id) {
		case 0: // ip camera
			handle->impl = x3_ipc_generic_impl();
			break;
		case 1: // usb camera
			handle->impl = x3_usb_cam_generic_impl();
			break;
		case 2: // video box
			handle->impl = x3_box_generic_impl();
			break;
		default:
			printf("Solution(%d) not implemented, Please look forward to it!\n", handle->m_solution_id);
			return -1;
	}

	LOGI_print("Start Solution: %d", handle->m_solution_id);

	return 0;
}

/* id：用来区分不同的应用方案，比如选择使用哪个sensor，使用什么样的vps、venc配置，或者只启用vps和venc
 * 根据id来设置适用于该方案的接口方法
*/
int camera_handle_init(void)
{
	int ret = 0;
	camera_handle_t *handle;
	if (g_camera_handle)
		return 0;

	// 获取芯片型号和接入的sensor型号
	memset(&g_hard_capability, 0, sizeof(g_hard_capability));
	x3_get_hard_capability(&g_hard_capability);

	// 初始化配置
	if (g_x3_cfg_is_load == 0) {
		x3_cfg_load();
		g_x3_cfg_is_load = 1;
	}

	handle = malloc(sizeof(camera_handle_t));
	ASSERT(handle);
	g_camera_handle = handle;
	ret = _camera_handle_init(handle);
	if (ret)
	{
		printf("_camera_handle_init failed!\n");
		goto err;
	}
	ASSERT(handle->impl);

	// 配置vin、 isp、vps、venc等各个模块的参数
	if (handle->impl->init_param && handle->impl->init_param())
	{
		printf("handle->impl->init_param failed!\n");
		goto err;
	}

	// init vin、isp、vps、venc...
	if (handle->impl->init && handle->impl->init())
	{
		printf("handle->impl->init failed!\n");
		goto err;
	}

	return 0;

err:
	if (handle)
		free(handle);
	g_camera_handle = NULL;

	return -1;
}

int camera_handle_uninit(void)
{
	camera_handle_t *handle = g_camera_handle;
	// 判断参数是否合法
	if (handle == NULL || handle->impl == NULL || handle->impl->uninit == NULL)
		return -1;
	if (handle->impl->uninit && handle->impl->uninit())
	{
		printf("handle->impl->uninit failed!\n");
		return -1;
	}
	if (handle)
		free(handle);
	g_camera_handle = NULL;

	return 0;
}

int camera_handle_start(void)
{
	camera_handle_t *handle = g_camera_handle;
	if (handle == NULL || handle->impl == NULL || handle->impl->start == NULL)
		return -1;
	return handle->impl->start();
}

int camera_handle_stop(void)
{
	camera_handle_t *handle = g_camera_handle;
	if (handle == NULL || handle->impl == NULL || handle->impl->stop == NULL)
		return -1;
	return handle->impl->stop();
}

int camera_handle_get_sensor_list(void)
{
	return g_hard_capability.m_sensor_list;
}

int camera_handle_get_solution_config(char *out_str)
{
	char *config_str = x3_cfg_obj2string();
	strcpy(out_str, config_str);
	free(config_str);
	return 0;
}

int camera_handle_set_solution_config(char *in_str)
{
	x3_cfg_string2obj(in_str);
	return 0;
}

int camera_handle_save_solution_config(char *in_str)
{
	x3_cfg_string2obj(in_str);
	x3_cfg_save();
	return 0;
}

int camera_handle_recovery_solution_config(char *out_str)
{
	x3_cfg_load_default_config();

	char *config_str = x3_cfg_obj2string();
	strcpy(out_str, config_str);
	free(config_str);
	return 0;
}

int camera_handle_param_set(CAM_PARAM_E type, char* val, unsigned int length)
{
	camera_handle_t *handle = g_camera_handle;
	if (handle == NULL || handle->impl == NULL || handle->impl->param_set == NULL)
		return -1;
	return handle->impl->param_set(type, val, length);
}

int camera_handle_param_get(CAM_PARAM_E type, char* val, unsigned int* length)
{
	camera_handle_t *handle = g_camera_handle;
	if (handle == NULL || handle->impl == NULL || handle->impl->param_get == NULL)
		return -1;
	return handle->impl->param_get(type, val, length);
}
