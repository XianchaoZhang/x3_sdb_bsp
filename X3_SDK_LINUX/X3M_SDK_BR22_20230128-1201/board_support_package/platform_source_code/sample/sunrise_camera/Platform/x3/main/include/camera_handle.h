#ifndef CAMERA_HANDLE_H
#define CAMERA_HANDLE_H

#include "camera_struct_define.h"
#include "communicate/sdk_common_struct.h"

#include "x3_sdk_wrap.h"

#define MAX_DEV_NUM  4  // 定义支持的最多数据通路，比如多sensor输入

typedef struct {
	int 				m_enable;
	int 				m_vin_enable; // 使能标志
	x3_vin_info_t		m_vin_info; // 包括 sensor、 mipi、isp、 ldc、dis的配置
	int 				m_vps_enable;
	x3_vps_infos_t		m_vps_infos; // vps的配置，支持多个vps group
	/*int 				m_pym_enable;*/
	/*x3_pmy_info_t		m_pym_info; // pym 各层配置*/
	int 				m_venc_enable;
	x3_venc_info_t		m_venc_info; // H264、H265、Jpeg、MJpeg编码通道配置
	int 				m_vdec_enable;
	x3_vdec_info_t		m_vdec_info; // H264、H265解码通道配置
	int 				m_vot_enable;
	x3_vot_info_t		m_vot_info; // x3的视频输出（vo、iar）配置
	int 				m_rgn_enable;
	x3_rgn_info_t		m_rgn_info; // 设置一个时间戳的osd
	int 				m_bpu_enable;
	x3_bpu_info_t		m_bpu_info; // 设置bpu运行的模型
} x3_modules_info_t;

typedef struct camera_ops {
	int (*init_param)(void); // 初始化VIN、VPS、VENC、BPU等模块的配置参数
	int (*init)(void); // x3 sdk 初始化，根据配置初始化
	int (*uninit)(void); // 反初始化
	int (*start)(void); // 启动x3 媒体相关的各个模块
	int (*stop)(void); // 停止
	// 本模块支持的CMD都通过以下两个接口简直实现
	int (*param_set)(CAM_PARAM_E type, char* val, unsigned int length);
	int (*param_get)(CAM_PARAM_E type, char* val, unsigned int* length);
} camera_ops_t;

int camera_handle_init(void);
int camera_handle_uninit(void);
int camera_handle_start(void);
int camera_handle_stop(void);
int camera_handle_get_sensor_list(void);
int camera_handle_get_solution_config(char *out_str);
int camera_handle_set_solution_config(char *in_str);
int camera_handle_save_solution_config(char *in_str);
int camera_handle_recovery_solution_config(char *out_str);
int camera_handle_param_set(CAM_PARAM_E type, char* val, unsigned int length);
int camera_handle_param_get(CAM_PARAM_E type, char* val, unsigned int* length);

#endif