/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2019 Horizon Robotics, Inc.
 * All rights reserved.
 ----------------------------------------------------------------------------
 * 该段代码用于实现 vin->vps->model(bpu_predict) 数据通路
 * 模型以 224*224 的 mobilenetv1 为例，主要用于理解 bpu_predict 的相关接口
 ***************************************************************************/
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
#include <stdbool.h>

#include "communicate/sdk_common_cmd.h"
#include "communicate/sdk_common_struct.h"
#include "communicate/sdk_communicate.h"

#include "utils/utils_log.h"
#include "utils/mthread.h"

#include "x3_bpu.h"
#include "yolov5_post_process.h"
#include "fcos_post_process.h"
#include "personMultitask_post_process.h"
#include "dnn/hb_dnn.h"

/**
 * Align by 16
 */
#define ALIGN_16(v) ((v + (16 - 1)) / 16 * 16)

#define HB_CHECK_SUCCESS(value, errmsg)                                \
	do {                                                               \
		/*value can be call of function*/                                  \
		int ret_code = value;                                           \
		if (ret_code != 0) {                                             \
			printf("[BPU ERROR] %s, error code:%d\n", errmsg, ret_code); \
			return ret_code;                                               \
		}                                                                \
	} while (0);


static void print_model_info(hbPackedDNNHandle_t *packed_dnn_handle)
{
	int i = 0, j = 0;
	hbDNNHandle_t dnn_handle;
	const char **model_name_list;
	int model_count = 0;
	hbDNNTensorProperties properties;

	HB_CHECK_SUCCESS(hbDNNGetModelNameList(
		&model_name_list, &model_count, packed_dnn_handle),
		"hbDNNGetModelNameList failed");
	if (model_count <= 0) {
		printf("Modle count <= 0\n");
		return -1;
	}
	HB_CHECK_SUCCESS(
		hbDNNGetModelHandle(&dnn_handle, packed_dnn_handle, model_name_list[0]),
		"hbDNNGetModelHandle failed");

	printf("Model info:\nmodel_name: %s\n", model_name_list[0]);

	int input_count = 0;
	int output_count = 0;
	HB_CHECK_SUCCESS(hbDNNGetInputCount(&input_count, dnn_handle),
                     "hbDNNGetInputCount failed");
    HB_CHECK_SUCCESS(hbDNNGetOutputCount(&output_count, dnn_handle),
                     "hbDNNGetInputCount failed");

	printf("Input count: %d\n", input_count);
	for (i = 0; i < input_count; i++) {
		HB_CHECK_SUCCESS(
			hbDNNGetInputTensorProperties(&properties, dnn_handle, i),
			"hbDNNGetInputTensorProperties failed");
		
		printf("input[%d]: tensorLayout: %d tensorType: %d validShape:(",
			i, properties.tensorLayout, properties.tensorType);
		for (j = 0; j < properties.validShape.numDimensions; j++)
			printf("%d, ", properties.validShape.dimensionSize[j]);
		printf("), ");
		printf("alignedShape:(");
		for (j = 0; j < properties.alignedShape.numDimensions; j++)
			printf("%d, ", properties.alignedShape.dimensionSize[j]);
		printf(")\n");
	}

	printf("Output count: %d\n", output_count);
	for (i = 0; i < output_count; i++) {
		HB_CHECK_SUCCESS(
			hbDNNGetOutputTensorProperties(&properties, dnn_handle, i),
			"hbDNNGetOutputTensorProperties failed");
		printf("Output[%d]: tensorLayout: %d tensorType: %d validShape:(",
			i, properties.tensorLayout, properties.tensorType);
		for (j = 0; j < properties.validShape.numDimensions; j++)
			printf("%d, ", properties.validShape.dimensionSize[j]);
		printf("), ");
		printf("alignedShape:(");
		for (j = 0; j < properties.alignedShape.numDimensions; j++)
			printf("%d, ", properties.alignedShape.dimensionSize[j]);
		printf(")\n");
	}
}

static int prepare_output_tensor(hbDNNTensor *output_tensor,
                           hbDNNHandle_t dnn_handle) {
  int ret = 0;

  int i = 0, j = 0;
  int output_count = 0;
  char mem_name[16] = {0};
  hbDNNTensorProperties properties;
  hbDNNTensor *output = output_tensor;

  hbDNNGetOutputCount(&output_count, dnn_handle);
  for (i = 0; i < output_count; ++i) {
	HB_CHECK_SUCCESS(
			hbDNNGetOutputTensorProperties(&output[i].properties, dnn_handle, i),
			"hbDNNGetOutputTensorProperties failed");
	HB_CHECK_SUCCESS(hbSysAllocCachedMem(&output[i].sysMem[0], output[i].properties.alignedByteSize),
			"hbSysAllocCachedMem failed");
	}

  return ret;
}

static void release_output_tensor(hbDNNTensor *output, int len)
{
	for (int i = 0; i < len; i++) {
		HB_CHECK_SUCCESS(hbSysFreeMem(&(output[i].sysMem[0])),
					   "hbSysFreeMem failed");
	}
}

static void *thread_yolov5_post_process(void *ptr)
{
	tsThread *privThread = (tsThread*)ptr;
	Yolov5PostProcessInfo_t *post_info;

	mThreadSetName(privThread, __func__);

	bpu_server_t *bpu_handle = (bpu_server_t *)privThread->pvThreadData;
	while (privThread->eState == E_THREAD_RUNNING) {
		if (mQueueDequeueTimed(&bpu_handle->m_output_queue, 100, (void**)&post_info) != E_QUEUE_OK)
			continue;

		char *results = Yolov5PostProcess(post_info);

		if (results) {
			if (NULL != bpu_handle->callback) {
				bpu_handle->callback(results, bpu_handle->m_userdata);
			} else {
				LOGI_print("%s", results);
			}
			free(results);
		}
		if (post_info) {
			free(post_info);
			post_info = NULL;
		}
	}
	mThreadFinish(privThread);
	return NULL;
}


static void *thread_run_yolov5(void *ptr)
{
	tsThread *privThread = (tsThread*)ptr;
	int i = 0, ret = 0;
	hbDNNTensor *input_tensor;
	int output_count = 0;

	bpu_server_t *bpu_handle = (bpu_server_t *)privThread->pvThreadData;

	mThreadSetName(privThread, __func__);

	if (bpu_handle == NULL)
		return NULL;

	hbPackedDNNHandle_t packed_dnn_handle = bpu_handle->m_packed_dnn_handle;
	hbDNNHandle_t dnn_handle = bpu_handle->m_dnn_handle;

	hbDNNGetOutputCount(&output_count, dnn_handle);

	// 准备模型输出节点tensor，5组输出buff轮转，简单处理，理论上后处理的速度是要比算法推理更快的
	hbDNNTensor output_tensors[5][3];
	int cur_ouput_buf_idx = 0;
	for (i = 0; i < 5; i++) {
		ret = prepare_output_tensor(output_tensors[i], dnn_handle);
		if (ret) {
			printf("prepare model output tensor failed\n");
			return NULL;
		}
	}

	hbDNNTaskHandle_t task_handle = NULL;

	while (privThread->eState == E_THREAD_RUNNING) {
		if (mQueueDequeueTimed(&bpu_handle->m_input_queue, 100, (void**)&input_tensor) != E_QUEUE_OK)
			continue;

		// make sure memory data is flushed to DDR before inference
		hbSysFlushMem(&input_tensor->sysMem[0], HB_SYS_MEM_CACHE_CLEAN);

		hbDNNTensor *output = &output_tensors[cur_ouput_buf_idx][0];

		// 模型推理infer
		hbDNNInferCtrlParam infer_ctrl_param;
		HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&infer_ctrl_param);
		HB_CHECK_SUCCESS(hbDNNInfer(&task_handle,
				&output,
				input_tensor,
				dnn_handle,
				&infer_ctrl_param),
				"hbDNNInfer failed");
		// wait task done
		HB_CHECK_SUCCESS(hbDNNWaitTaskDone(task_handle, 0),
			"hbDNNWaitTaskDone failed");

		// make sure CPU read data from DDR before using output tensor data
		for (int i = 0; i < output_count; i++) {
			hbSysFlushMem(&output_tensors[cur_ouput_buf_idx][i].sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE);
		}

		// release task handle
		HB_CHECK_SUCCESS(hbDNNReleaseTask(task_handle), "hbDNNReleaseTask failed");
		task_handle = NULL;

		// 如果后处理队列满的，直接返回
		if (mQueueIsFull(&bpu_handle->m_output_queue)) {
			LOGI_print("post process queue full, skip it");
			cur_ouput_buf_idx++;
			cur_ouput_buf_idx %= 5;
			continue;
		}

		// 后处理数据
		Yolov5PostProcessInfo_t *post_info;
		post_info = (Yolov5PostProcessInfo_t *)malloc(sizeof(Yolov5PostProcessInfo_t));
		if (NULL == post_info) {
			LOGE_print("Failed to allocate memory for post_info");
			continue;
		}
		post_info->is_pad_resize = 0;
		post_info->score_threshold = 0.3;
		post_info->nms_threshold = 0.45;
		post_info->nms_top_k = 500;
		post_info->width = bpu_handle->m_image_info.m_model_w;
		post_info->height = bpu_handle->m_image_info.m_model_h;
		post_info->ori_width = bpu_handle->m_image_info.m_ori_width;
		post_info->ori_height = bpu_handle->m_image_info.m_ori_height;
		post_info->output_tensor = output_tensors[cur_ouput_buf_idx];
		mQueueEnqueue(&bpu_handle->m_output_queue, post_info);
		cur_ouput_buf_idx++;
		cur_ouput_buf_idx %= 5;
		
	}
	
	for (i = 0; i < 5; i++)
		release_output_tensor(output_tensors[i], 3);	// 释放模型输出资源

	mThreadFinish(privThread);
	return NULL;
}


static void *thread_fcos_post_process(void *ptr)
{
	tsThread *privThread = (tsThread*)ptr;
	FcosPostProcessInfo_t *post_info;

	mThreadSetName(privThread, __func__);

	bpu_server_t *bpu_handle = (bpu_server_t *)privThread->pvThreadData;
	while (privThread->eState == E_THREAD_RUNNING) {
		if (mQueueDequeueTimed(&bpu_handle->m_output_queue, 100, (void**)&post_info) != E_QUEUE_OK)
			continue;

		char *results = FcosPostProcess(post_info);

		if (results) {
			if (NULL != bpu_handle->callback) {
				bpu_handle->callback(results, bpu_handle->m_userdata);
			} else {
				LOGI_print("%s", results);
			}
			free(results);
		}

		if (post_info) {
			free(post_info);
			post_info = NULL;
		}
	}
	mThreadFinish(privThread);
	return NULL;
}


static void *thread_run_fcos(void *ptr)
{
	tsThread *privThread = (tsThread*)ptr;
	int i = 0, ret = 0;
	hbDNNTensor *input_tensor;
	int output_count = 0;

	bpu_server_t *bpu_handle = (bpu_server_t *)privThread->pvThreadData;

	mThreadSetName(privThread, __func__);

	if (bpu_handle == NULL)
		return NULL;

	hbPackedDNNHandle_t packed_dnn_handle = bpu_handle->m_packed_dnn_handle;
	hbDNNHandle_t dnn_handle = bpu_handle->m_dnn_handle;

	hbDNNGetOutputCount(&output_count, dnn_handle);

	// 准备模型输出节点tensor，5组输出buff轮转，简单处理，理论上后处理的速度是要比算法推理更快的
	hbDNNTensor output_tensors[5][15];
	int cur_ouput_buf_idx = 0;
	for (i = 0; i < 5; i++) {
		ret = prepare_output_tensor(output_tensors[i], dnn_handle);
		if (ret) {
			printf("prepare model output tensor failed\n");
			return NULL;
		}
	}

	hbDNNTaskHandle_t task_handle = NULL;

	while (privThread->eState == E_THREAD_RUNNING) {
		if (mQueueDequeueTimed(&bpu_handle->m_input_queue, 100, (void**)&input_tensor) != E_QUEUE_OK)
			continue;

		// make sure memory data is flushed to DDR before inference
		hbSysFlushMem(&input_tensor->sysMem[0], HB_SYS_MEM_CACHE_CLEAN);

		hbDNNTensor *output = &output_tensors[cur_ouput_buf_idx][0];

		// 模型推理infer
		hbDNNInferCtrlParam infer_ctrl_param;
		HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&infer_ctrl_param);
		infer_ctrl_param.bpuCoreId = 0; // 单帧双核，量化编译的模型也要配置成单帧双核模型
		HB_CHECK_SUCCESS(hbDNNInfer(&task_handle,
				&output,
				input_tensor,
				dnn_handle,
				&infer_ctrl_param),
				"hbDNNInfer failed");
		// wait task done
		HB_CHECK_SUCCESS(hbDNNWaitTaskDone(task_handle, 0),
			"hbDNNWaitTaskDone failed");

		// make sure CPU read data from DDR before using output tensor data
		for (int i = 0; i < output_count; i++) {
			hbSysFlushMem(&output_tensors[cur_ouput_buf_idx][i].sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE);
		}

		// release task handle
		HB_CHECK_SUCCESS(hbDNNReleaseTask(task_handle), "hbDNNReleaseTask failed");
		task_handle = NULL;

		// 如果后处理队列满的，直接返回
		if (mQueueIsFull(&bpu_handle->m_output_queue)) {
			LOGI_print("post process queue full, skip it");
			cur_ouput_buf_idx++;
			cur_ouput_buf_idx %= 5;
			continue;
		}

		// 后处理数据
		FcosPostProcessInfo_t *post_info;
		post_info = (FcosPostProcessInfo_t *)malloc(sizeof(FcosPostProcessInfo_t));
		if (NULL == post_info) {
			LOGE_print("Failed to allocate memory for post_info");
			continue;
		}
		post_info->is_pad_resize = 0;
		post_info->score_threshold = 0.5;
		post_info->nms_threshold = 0.6;
		post_info->nms_top_k = 500;
		post_info->width = bpu_handle->m_image_info.m_model_w;
		post_info->height = bpu_handle->m_image_info.m_model_h;
		post_info->ori_width = bpu_handle->m_image_info.m_ori_width;
		post_info->ori_height = bpu_handle->m_image_info.m_ori_height;
		post_info->output_tensor = output_tensors[cur_ouput_buf_idx];
		mQueueEnqueue(&bpu_handle->m_output_queue, post_info);
		cur_ouput_buf_idx++;
		cur_ouput_buf_idx %= 5;
		
	}
	
	for (i = 0; i < 5; i++)
		release_output_tensor(output_tensors[i], 3);	// 释放模型输出资源

	mThreadFinish(privThread);
	return NULL;
}


static void *thread_personMultitask_post_process(void *ptr)
{
	tsThread *privThread = (tsThread*)ptr;
	PersonPostProcessInfo_t *post_info = NULL;

	mThreadSetName(privThread, __func__);

	bpu_server_t *bpu_handle = (bpu_server_t *)privThread->pvThreadData;
	while (privThread->eState == E_THREAD_RUNNING) {
		if (mQueueDequeueTimed(&bpu_handle->m_output_queue, 100, (void**)&post_info) != E_QUEUE_OK)
			continue;

		char *results = PersonPostProcess(post_info);

		if (results) {
			if (NULL != bpu_handle->callback) {
				bpu_handle->callback(results, bpu_handle->m_userdata);
			} else {
				LOGI_print("%s", results);
			}
			free(results);
		}
		if (post_info) {
			free(post_info);
			post_info = NULL;
		}
	}
	mThreadFinish(privThread);
	return NULL;
}

static void *thread_run_personMultitask(void *ptr)
{
	tsThread *privThread = (tsThread*)ptr;
	int i = 0, ret = 0;
	hbDNNTensor *input_tensor;
	int output_count = 0;

	bpu_server_t *bpu_handle = (bpu_server_t *)privThread->pvThreadData;

	mThreadSetName(privThread, __func__);

	if (bpu_handle == NULL)
		return NULL;

	hbPackedDNNHandle_t packed_dnn_handle = bpu_handle->m_packed_dnn_handle;
	hbDNNHandle_t dnn_handle = bpu_handle->m_dnn_handle;

	hbDNNGetOutputCount(&output_count, dnn_handle);

	// 准备模型输出节点tensor，5组输出buff轮转，简单处理，理论上后处理的速度是要比算法推理更快
	hbDNNTensor output_tensors[5][12];
	int cur_ouput_buf_idx = 0;
	for (i = 0; i < 5; i++) {
		ret = prepare_output_tensor(output_tensors[i], dnn_handle);
		if (ret) {
			printf("prepare model output tensor failed\n");
			return NULL;
		}
	}

	hbDNNTaskHandle_t task_handle = NULL;
	hbDNNTensor *output = NULL;

	while (privThread->eState == E_THREAD_RUNNING) {
		if (mQueueDequeueTimed(&bpu_handle->m_input_queue, 100, (void**)&input_tensor) != E_QUEUE_OK)
			continue;

		// make sure memory data is flushed to DDR before inference
		hbSysFlushMem(&input_tensor->sysMem[0], HB_SYS_MEM_CACHE_CLEAN);
		output = &output_tensors[cur_ouput_buf_idx][0];

		// 模型推理infer
		hbDNNInferCtrlParam infer_ctrl_param;
		HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&infer_ctrl_param);
		HB_CHECK_SUCCESS(hbDNNInfer(&task_handle,
				&output,
				input_tensor,
				dnn_handle,
				&infer_ctrl_param),
				"hbDNNInfer failed");
		// wait task done
		HB_CHECK_SUCCESS(hbDNNWaitTaskDone(task_handle, 0),
			"hbDNNWaitTaskDone failed");

		/*LOGI_print("04. run model ok.");*/

		// make sure CPU read data from DDR before using output tensor data
		for (int i = 0; i < output_count; i++) {
			hbSysFlushMem(&output_tensors[cur_ouput_buf_idx][i].sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE);
		}

		// release task handle
		HB_CHECK_SUCCESS(hbDNNReleaseTask(task_handle), "hbDNNReleaseTask failed");
		task_handle = NULL;

		// 如果后处理队列满的，直接返回
		if (mQueueIsFull(&bpu_handle->m_output_queue)) {
			LOGI_print("post process queue full, skip it");
			cur_ouput_buf_idx++;
			cur_ouput_buf_idx %= 5;
			continue;
		}

		// 后处理数据
		PersonPostProcessInfo_t *post_info = NULL;
		post_info = (PersonPostProcessInfo_t *)malloc(sizeof(PersonPostProcessInfo_t));
		if (NULL == post_info) {
			LOGE_print("Failed to allocate memory for post_info");
			continue;
		}
		post_info->m_dnn_handle = bpu_handle->m_dnn_handle;
		post_info->width = bpu_handle->m_image_info.m_model_w;
		post_info->height = bpu_handle->m_image_info.m_model_h;
		post_info->ori_width = bpu_handle->m_image_info.m_ori_width;
		post_info->ori_height = bpu_handle->m_image_info.m_ori_height;
		post_info->output_tensor = output_tensors[cur_ouput_buf_idx];
		mQueueEnqueue(&bpu_handle->m_output_queue, post_info);
		cur_ouput_buf_idx++;
		cur_ouput_buf_idx %= 5;
	}
	
	for (i = 0; i < 5; i++)
		release_output_tensor(output_tensors[i], output_count);	// 释放模型输出资源

	mThreadFinish(privThread);
	return NULL;
}


// 解析分类结果
static void parse_classification_result(
		hbDNNTensor *tensor,
		int *idx,
		float *score_top1) {

	float *scores = (float *) (tensor->sysMem[0].virAddr);
	int *shape = tensor->properties.validShape.dimensionSize;
	for (int i = 0; i < shape[1] * shape[2] * shape[3]; i++) {
		float score = scores[i];
		if (score > *score_top1){
			*idx =  i;
			*score_top1 = score;
		}
	}
}

static void *thread_run_mobilenetv2(void *ptr)
{
	tsThread *privThread = (tsThread*)ptr;
	int ret = 0;
	hbDNNTensor *input_tensor;
	int output_count = 0;

	bpu_server_t *bpu_handle = (bpu_server_t *)privThread->pvThreadData;

	mThreadSetName(privThread, __func__);

	if (bpu_handle == NULL)
		return NULL;

	hbPackedDNNHandle_t packed_dnn_handle = bpu_handle->m_packed_dnn_handle;
	hbDNNHandle_t dnn_handle = bpu_handle->m_dnn_handle;

	hbDNNGetOutputCount(&output_count, dnn_handle);

	// 准备模型输出节点tensor
	hbDNNTensor output_tensors[1];
	ret = prepare_output_tensor(output_tensors, dnn_handle);
	if (ret) {
		printf("prepare model output tensor failed\n");
		return NULL;
	}

	hbDNNTaskHandle_t task_handle = NULL;
	hbDNNTensor *output = &output_tensors[0];

	while (privThread->eState == E_THREAD_RUNNING) {
		if (mQueueDequeueTimed(&bpu_handle->m_input_queue, 100, (void**)&input_tensor) != E_QUEUE_OK)
			continue;

		// make sure memory data is flushed to DDR before inference
		hbSysFlushMem(&input_tensor->sysMem[0], HB_SYS_MEM_CACHE_CLEAN);

		// 模型推理infer
		hbDNNInferCtrlParam infer_ctrl_param;
		HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&infer_ctrl_param);
		HB_CHECK_SUCCESS(hbDNNInfer(&task_handle,
				&output,
				input_tensor,
				dnn_handle,
				&infer_ctrl_param),
				"hbDNNInfer failed");
		// wait task done
		HB_CHECK_SUCCESS(hbDNNWaitTaskDone(task_handle, 0),
			"hbDNNWaitTaskDone failed");

		/*LOGI_print("04. run model ok.");*/

		// make sure CPU read data from DDR before using output tensor data
		for (int i = 0; i < output_count; i++) {
			hbSysFlushMem(&output_tensors[i].sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE);
		}

		// release task handle
		HB_CHECK_SUCCESS(hbDNNReleaseTask(task_handle), "hbDNNReleaseTask failed");
		task_handle = NULL;

		// 同步模式下的后处理, 测试用，每个模型都要一份独立的后处理接口
		float score_top1 = 0.0;
		int idx = 0;
		char result[128] = {0};
		parse_classification_result(
				&output_tensors[0], &idx, &score_top1);
		/*printf("Classification top_1 results: { id: %d, score: %.3f }\n", idx, score_top1);*/
		/*printf("---------------------------\n\n");*/
	
		// 通过websocket把算法结果发送给web页面
		sprintf(result, "\"mobilenetv2_result\": \"id=%d, score=%.3f\"", idx, score_top1);

		if (NULL != bpu_handle->callback) {
			bpu_handle->callback(result, bpu_handle->m_userdata);
		} else {
			LOGI_print("%s", result);
		}
	}

	ret = hbSysFreeMem(&output_tensors[0]);			  // 释放模型输出资源
	if (ret)
		printf("output data free failed\n");

	mThreadFinish(privThread);
	return NULL;
}

bpu_server_t *x3_bpu_predict_init(char *model_file_name, int alog_id)
{
	int ret = 0, i = 0;
	bpu_server_t *bpu_handle = NULL; // 算法运行任务句柄
	const char **model_name_list;
	int model_count = 0;
	hbDNNTensorProperties properties;
	hbPackedDNNHandle_t packed_dnn_handle;
	hbDNNHandle_t dnn_handle;

	// 分配句柄空间
	bpu_handle = malloc(sizeof(bpu_server_t));
	if (NULL == bpu_handle)
		return NULL;

	memset(bpu_handle, 0, sizeof(bpu_server_t));

	LOGI_print("packed_dnn_handle: %p, dnn_handle: %p", packed_dnn_handle, dnn_handle);
	
	bpu_handle->m_alog_type = alog_id;

	LOGI_print("model_file_name:%s\n", model_file_name);

	// 加载模型
	HB_CHECK_SUCCESS(
		hbDNNInitializeFromFiles(&packed_dnn_handle, &model_file_name, 1),
		"hbDNNInitializeFromFiles failed"); // 从本地文件加载模型

	// 打印模型信息
	print_model_info(packed_dnn_handle);

	HB_CHECK_SUCCESS(hbDNNGetModelNameList(
		&model_name_list, &model_count, packed_dnn_handle),
		"hbDNNGetModelNameList failed");

	if (model_count <= 0) {
		printf("Modle count <= 0\n");
		return -1;
	}
	LOGI_print("model_name_list[0]:%s\n", model_name_list[0]);

	HB_CHECK_SUCCESS(
		hbDNNGetModelHandle(&dnn_handle, packed_dnn_handle, model_name_list[0]),
		"hbDNNGetModelHandle failed");

	bpu_handle->m_packed_dnn_handle = packed_dnn_handle;
	bpu_handle->m_dnn_handle = dnn_handle;
	LOGI_print("packed_dnn_handle: %p, dnn_handle: %p", packed_dnn_handle, dnn_handle);

	LOGI_print("Model info:\nmodel_name: %s\n", model_name_list[0]);

	// 获取模型相关信息
	// 目前模型输入的yuv都按照nv12格式处理，其他格式先不做考虑
	HB_CHECK_SUCCESS(
			hbDNNGetInputTensorProperties(&properties, dnn_handle, 0),
			"hbDNNGetInputTensorProperties failed");
	hbDNNTensorShape  *input_tensor_shape = &properties.validShape;	 // 获取模型输入shape
	bpu_handle->m_image_info.m_model_h = (input_tensor_shape->dimensionSize)[2];
	bpu_handle->m_image_info.m_model_w = (input_tensor_shape->dimensionSize)[3];
	/*printf("get model iinput_tensor_shape ok, NCHW = (1, 3, %d, %d).\n",
		bpu_handle->m_image_info.m_model_h, bpu_handle->m_image_info.m_model_w);		// 打印从模型中读取的宽高*/

	// 设置默认的原始图像宽高为 1920 * 1080
	// 如果原始图像时4K/2K或者其他分辨率，请调用x3_bpu_predict_set_ori_hw接口重新设置
	bpu_handle->m_image_info.m_ori_height = 1080;
	bpu_handle->m_image_info.m_ori_width = 1920;

	// 队列中存2个，解决算法结果延迟较大的问题
	mQueueCreate(&bpu_handle->m_input_queue, 2);//the length of queue is 2
	mQueueCreate(&bpu_handle->m_output_queue, 3);//the length of queue is 3

	// bpu resize使用的内存
	bpu_handle->m_cur_resized_tensor = 0;
	for (i = 0; i < 5; i++) {
		// 分配bpu resize 使用的内存
		HB_CHECK_SUCCESS(hbSysAllocCachedMem(&bpu_handle->m_resized_tensors[i].sysMem[0],
			bpu_handle->m_image_info.m_model_h * bpu_handle->m_image_info.m_model_w),
			"hbSysAllocCachedMem failed");
		HB_CHECK_SUCCESS(hbSysAllocCachedMem(&bpu_handle->m_resized_tensors[i].sysMem[1],
			bpu_handle->m_image_info.m_model_h * bpu_handle->m_image_info.m_model_w/2),
			"hbSysAllocCachedMem failed");
	}

	return bpu_handle;
}

int x3_bpu_predict_unint(bpu_server_t *handle)
{
	int ret = 0;
	int i = 0;

	if (handle == NULL)
		return 0;

	for (i = 0; i < 5; i++) {
		ret = hbSysFreeMem(&handle->m_resized_tensors[i].sysMem[0]);	   // 释放模型输入资源
		ret |= hbSysFreeMem(&handle->m_resized_tensors[i].sysMem[1]);
		if (ret)
			LOGE_print("input data free failed");
	}
	// 销毁队列
	mQueueDestroy(&handle->m_output_queue);
	mQueueDestroy(&handle->m_input_queue);
	
	// 释放模型资源
	HB_CHECK_SUCCESS(hbDNNRelease(handle->m_packed_dnn_handle), "hbDNNRelease failed");

	// 释放任务句柄
	free(handle);

	LOGI_print("ok!");

	return ret;
}

// 启动算法示例
// id代表需要启动的算法模型:
// 1 - mobilenetv2
// 2 - yolov5
// 3 - 地平线私有人体、人脸、人体骨骼多任务算法模型
// 4 - fcos
bpu_server_t *x3_bpu_sample_init(int alog_id)
{
	switch(alog_id) {
		case 1:
			return x3_bpu_predict_init("../model_zoom/mobilenetv2_224x224_nv12.bin", alog_id);
		case 2:
			return x3_bpu_predict_init("../model_zoom/yolov5_672x672_nv12.bin", alog_id);
		case 3:
			return x3_bpu_predict_init("../model_zoom/personMultitask.hbm", alog_id);
		case 4:
			return x3_bpu_predict_init("../model_zoom/fcos_512x512_nv12.bin", alog_id);
		default:
			LOGE_print("unsupport alog id");
			return NULL;
	}
	return NULL;
}

void x3_bpu_predict_set_ori_hw(bpu_server_t *handle, int width, int height)
{
	if (handle == NULL) return;
	handle->m_image_info.m_ori_height = height;
	handle->m_image_info.m_ori_width = width;
}


int x3_bpu_predict_start(bpu_server_t *handle)
{
	if (handle == NULL)
		return 0;

	if (handle->m_alog_type == 1) {
		handle->m_run_model_thread.pvThreadData = (void *)handle;
		mThreadStart(thread_run_mobilenetv2, &handle->m_run_model_thread, E_THREAD_JOINABLE);
	} else if (handle->m_alog_type == 2) {
		// yolo5 算法 + 后处理耗时较长，不仅要抽帧运算，后处理还要多线程流水线处理
		// 所以其他算法都建议流水线处理
		handle->m_run_model_thread.pvThreadData = (void *)handle;
		mThreadStart(thread_run_yolov5, &handle->m_run_model_thread, E_THREAD_JOINABLE);

		handle->m_post_process_thread.pvThreadData = (void *)handle;
		mThreadStart(thread_yolov5_post_process, &handle->m_post_process_thread, E_THREAD_JOINABLE);
	} else if (handle->m_alog_type == 3) {
		handle->m_run_model_thread.pvThreadData = (void *)handle;
		mThreadStart(thread_run_personMultitask, &handle->m_run_model_thread, E_THREAD_JOINABLE);

		handle->m_post_process_thread.pvThreadData = (void *)handle;
		mThreadStart(thread_personMultitask_post_process, &handle->m_post_process_thread, E_THREAD_JOINABLE);
	} else if (handle->m_alog_type == 4) {
		handle->m_run_model_thread.pvThreadData = (void *)handle;
		mThreadStart(thread_run_fcos, &handle->m_run_model_thread, E_THREAD_JOINABLE);

		handle->m_post_process_thread.pvThreadData = (void *)handle;
		mThreadStart(thread_fcos_post_process, &handle->m_post_process_thread, E_THREAD_JOINABLE);
	} 

	return 0;
}

int x3_bpu_predict_stop(bpu_server_t *handle)
{
	if (handle == NULL)
		return 0;

	mThreadStop(&handle->m_post_process_thread);
	mThreadStop(&handle->m_run_model_thread);

	return 0;
}

static int x3_dumpToFile2plane(char *filename, char *srcBuf, char *srcBuf1,
		unsigned int size, unsigned int size1)
{

	FILE *yuvFd = NULL;
	char *buffer = NULL;

	yuvFd = fopen(filename, "w+");

	if (yuvFd == NULL) {
		printf("open(%s) fail", filename);
		return -1;
	}

	buffer = (char *)malloc(size + size1);

	if (buffer == NULL) {
		printf("ERR:malloc file");
		fclose(yuvFd);
		return -1;
	}

	memcpy(buffer, srcBuf, size);
	memcpy(buffer + size, srcBuf1, size1);

	fflush(stdout);

	fwrite(buffer, 1, size + size1, yuvFd);

	fflush(yuvFd);

	if (yuvFd)
		fclose(yuvFd);
	if (buffer)
		free(buffer);

	printf("DEBUG:filedump(%s, size(%d) is successed!!", filename, size);

	return 0;
}

// 完成输入数据前处理，执行bpu_resize缩放到适合模型的输入大小
// 处理完的数据推入算法推理队列
int x3_bpu_input_feed(bpu_server_t *handle, address_info_t *input_data)
{
#if 0 // test
	char nv12_file_name[32];
	static int nv12_index = 0;
#endif

	if (handle == NULL || input_data == NULL)
		return 0;

	// 如果队列满的，直接返回
	if (mQueueIsFull(&handle->m_input_queue)) {
		/*LOGI_print("input queue full, skip it");*/
		return 0;
	}

	int model_h = handle->m_image_info.m_model_h;
	int model_w = handle->m_image_info.m_model_w;

#if 0
	printf("model_h=%d, model_w=%d\n", model_h, model_w);
#endif

	// 准备输入数据（用于存放yuv数据）
	hbDNNTensor input_tensor;
	// resize后送给bpu运行的图像
	hbDNNTensor *input_tensor_resized = &handle->m_resized_tensors[handle->m_cur_resized_tensor];

	memset(&input_tensor, '\0', sizeof(hbDNNTensor));
	input_tensor.properties.tensorLayout = HB_DNN_LAYOUT_NCHW;
	// 张量类型为Y通道及UV通道为输入的图片, 方便直接使用 vpu出来的y和uv分离的数据
	input_tensor.properties.tensorType = HB_DNN_IMG_TYPE_NV12_SEPARATE; // 用于Y和UV分离的场景，主要为我们摄像头数据通路场景

	// 填充 input_tensor.sysMem 成员变量 Y 分量
	input_tensor.sysMem[0].virAddr = input_data->addr[0];
	input_tensor.sysMem[0].phyAddr = input_data->paddr[0];
	input_tensor.sysMem[0].memSize = input_data->stride_size * input_data->height;

	// 填充 input_tensor.data_ext 成员变量， UV 分量
	input_tensor.sysMem[1].virAddr = input_data->addr[1];
	input_tensor.sysMem[1].phyAddr = input_data->paddr[1];
	input_tensor.sysMem[1].memSize = (input_data->stride_size * input_data->height) / 2;

#if 0 // test
	if (nv12_index++ % 100 == 0) {
		sprintf(nv12_file_name, "1920x1080_nv12_input_%d.yuv", nv12_index++);
		x3_dumpToFile2plane(nv12_file_name, input_data->addr[0], input_data->addr[1],
			input_tensor.sysMem[0].memSize, input_tensor.sysMem[1].memSize);
	}
#endif

	// HB_DNN_IMG_TYPE_NV12_SEPARATE 类型的 layout 为 (1, 3, h, w)
	input_tensor.properties.validShape.numDimensions = 4;
	input_tensor.properties.validShape.dimensionSize[0] = 1;						// N
	input_tensor.properties.validShape.dimensionSize[1] = 3;						// C
	input_tensor.properties.validShape.dimensionSize[2] = input_data->height;		// H
	input_tensor.properties.validShape.dimensionSize[3] = input_data->stride_size; 	// W
	input_tensor.properties.alignedShape = input_tensor.properties.validShape;		// 已满足跨距对齐要求，直接赋值

	// 准备模型输入数据（用于存放模型输入大小的数据）
	input_tensor_resized->properties.tensorLayout = HB_DNN_LAYOUT_NCHW;
	input_tensor_resized->properties.tensorType = HB_DNN_IMG_TYPE_NV12_SEPARATE;

	// NCHW
	input_tensor_resized->properties.validShape.numDimensions = 4;
	input_tensor_resized->properties.validShape.dimensionSize[0] = 1;
	input_tensor_resized->properties.validShape.dimensionSize[1] = 3;
	input_tensor_resized->properties.validShape.dimensionSize[2] = model_h;
	input_tensor_resized->properties.validShape.dimensionSize[3] = model_w;
	input_tensor_resized->properties.alignedShape = input_tensor_resized->properties.validShape;		// 已满足对齐要求

	// 将数据Resize到模型输入大小
	hbDNNResizeCtrlParam ctrl;
	HB_DNN_INITIALIZE_RESIZE_CTRL_PARAM(&ctrl);
	hbDNNTaskHandle_t task_handle;
	HB_CHECK_SUCCESS(
		hbDNNResize(&task_handle, input_tensor_resized, &input_tensor, NULL, &ctrl),
		"hbDNNResize failed");

#if 0
	LOGI_print("03. resize ok.\n");
#endif
	HB_CHECK_SUCCESS(hbDNNWaitTaskDone(task_handle, 0),
					"hbDNNWaitTaskDone failed");

	HB_CHECK_SUCCESS(hbDNNReleaseTask(task_handle), "hbDNNReleaseTask failed");

#if 0
	LOGI_print("input_tensor_resized(%p): tensorLayout: %d tensorType: %d validShape:(", input_tensor_resized,
			input_tensor_resized->properties.tensorLayout, input_tensor_resized->properties.tensorType);
		int j = 0;
		for (j = 0; j < input_tensor_resized->properties.validShape.numDimensions; j++)
			printf("%d, ", input_tensor_resized->properties.validShape.dimensionSize[j]);
		printf(")\n");
#endif

#if 0 // test
	if (nv12_index % 100 == 0) {
		// Save resized image to file disk
		sprintf(nv12_file_name, "224x224_nv12_resize_%d.yuv", nv12_index);
		x3_dumpToFile2plane(nv12_file_name, input_tensor_resized->sysMem[0].virAddr,
			input_tensor_resized->sysMem[1].virAddr,
			input_tensor_resized->sysMem[0].memSize, input_tensor_resized->sysMem[1].memSize);
	}
#endif

	// 数据缩放完成，接下来推送进处理队列
	if (mQueueEnqueueEx(&handle->m_input_queue, input_tensor_resized) == E_QUEUE_OK) {
		handle->m_cur_resized_tensor++;
		handle->m_cur_resized_tensor %= 5;
	} else {
		LOGI_print("m_input_queue full, skip it");
	}

	return 0;
}

void x3_bpu_predict_callback_register(bpu_server_t* handle, bpu_post_process_callback callback, void *userdata)
{
	if (handle == NULL)
		return;
	handle->callback = callback;
	handle->m_userdata = userdata;
}

void x3_bpu_predict_callback_unregister(bpu_server_t* handle)
{
	if (handle == NULL)
		return;
	handle->callback = NULL;
	handle->m_userdata = NULL;
	LOGI_print("ok!");
}


// 通用的算法回调函数，目前都是通过websocket想web上发送
int x3_bpu_predict_general_result_handle(char *result, void *userdata)
{
	int ret = 0;
	int pipeline_id = 0;
	char *ws_msg = NULL;

	if (userdata)
		pipeline_id = *(int*)userdata;

	// json 算法结果添加标志信息
	// 分配内存
	ws_msg = malloc(strlen(result) + 32);
	if (NULL == ws_msg) {
		LOGE_print("Failed to allocate memory for ws_msg");
		return -1;
	}
	sprintf(ws_msg, "{\"kind\":3, \"pipeline\":%d,", pipeline_id);
	strcat(ws_msg, result);
	strcat(ws_msg, "}");

	ret = SDK_Cmd_Impl(SDK_CMD_WEBSOCKET_SEND_MSG, (void*)ws_msg);
	free(ws_msg);
	return ret;
}

