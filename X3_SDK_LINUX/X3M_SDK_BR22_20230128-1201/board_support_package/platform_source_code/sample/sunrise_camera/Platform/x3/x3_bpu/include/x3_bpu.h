/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef X3_BPU_H_
#define X3_BPU_H_

#include "vio/hb_vio_interface.h"

#include "utils/mqueue.h"
#include "utils/mthread.h"

#include "dnn/hb_dnn.h"

typedef int (*bpu_post_process_callback)(char* result, void *userdata);

// 这个结构体中存储的数据用来后处理时进行坐标还原
typedef struct {
	int m_model_w;		// 模型输入宽
	int m_model_h;		// 模型输入高
	int m_ori_width;	// 用户看到的原始图像宽，比如web上显示的推流视频
	int m_ori_height;	// 用户看到的原始图像高，比如web上显示的推流视频
} bpu_image_info_t;

typedef struct {
	int 				m_alog_type; // 1:mobilenetV2 2: yolo5 3:personMultitask
	hbPackedDNNHandle_t	m_packed_dnn_handle;
	hbDNNHandle_t		m_dnn_handle;
	bpu_image_info_t	m_image_info;
	tsThread 			m_run_model_thread; // 运算模型的线程
	hbDNNTensor 		m_resized_tensors[5]; // 给bpu resize预分配内存，避免每一帧数据都进行内存的申请和释放
	int 				m_cur_resized_tensor; // 当前使用的bpu resize 内存
	tsQueue				m_input_queue; // 用于算法预测的yuv数据
	tsThread 			m_post_process_thread; // 算法后处理线程
	tsQueue				m_output_queue; // 算法输出结果队列，yolo5的后处理时间太长了，用线程分开处理
	bpu_post_process_callback	callback; // 算法结果处理后的回调，目前直接通过websocket发给web
	void				*m_userdata; // 回调函数中使用到的数据
} bpu_server_t;

bpu_server_t *x3_bpu_predict_init(char *model_file_name, int alog_id);
int x3_bpu_predict_unint(bpu_server_t *handle);

// 对x3_bpu_predict_init再次封装，主要是根据alog_id使用不同的模型文件
bpu_server_t *x3_bpu_sample_init(int alog_id);

void x3_bpu_predict_set_ori_hw(bpu_server_t *handle, int width, int height);

int x3_bpu_predict_start(bpu_server_t *handle);
int x3_bpu_predict_stop(bpu_server_t *handle);

int x3_bpu_input_feed(bpu_server_t *handle, address_info_t *input_data);

void x3_bpu_predict_callback_register(bpu_server_t* handle, bpu_post_process_callback callback, void *userdata);
void x3_bpu_predict_callback_unregister(bpu_server_t* handle);

int x3_bpu_run_mobilenetv2(hbPackedDNNHandle_t *packed_dnn_handle, address_info_t *input_data);

int x3_bpu_predict_general_result_handle(char *result, void *userdata);
#endif // X3_BPU_H_