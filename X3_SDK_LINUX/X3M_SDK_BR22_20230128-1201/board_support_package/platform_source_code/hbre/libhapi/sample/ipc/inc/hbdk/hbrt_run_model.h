//
// Created by zhenjiang.wang on 8/10/18.
//

#ifndef HBDK_HBRT_RUN_MODEL_H
#define HBDK_HBRT_RUN_MODEL_H

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_CORE_NUM 2

#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include "plat_cnn.h"

typedef struct {
  bpu_addr_t inst_addr;                // Model instruction base address
  uint32_t inst_num;                   // Model instruction number
  bpu_addr_t param_addr;               // Model param base address
  bpu_addr_t output_addr;              // Model output base address
  bpu_addr_t input_addr;               // Model input base address
  bpu_addr_t dynamic_addr;             // Model intermediate output base address
  enum cnn_core_type core_type;        // Core type to run a instruction sequence, enum type of CNN_CPRE_TYPE
  enum cnn_start_method start_method;  // Start method of a instruction sequence, enum type of CNN_START_METHOD
  uint32_t interrupt_num;
  bpu_addr_t inst_param_base[MAX_CORE_NUM];
  bpu_addr_t dynamic_base[MAX_CORE_NUM];
  int use_running_dynamic;
  int use_running_code_cache;
  bpu_addr_t extra_param[4];  // Extra params reserved for specific target
  /*
   * extra_param[0]:
   *      for Matrix, it represents the size of all outputs of ONE roi. this value is used to set output stride register
   *                  of 2PE core.
   *      for X2, it represents the physical addr. of pyramid UV channel data
   *
   * extra_param[1]:
   *      for X2, it represents the physical addr. for debug output.
   *
   * extra_param[2]:
   *      for X2, it represents the perf counter enable flag.
   *
   * extra_param[3]:
   *      for X2, it represents the resizer work mode. 0 means only do resizer computation. 1 means do both resizer and
   * CNN.
   *
   */
} hbrt_funccall_info;

extern void setExtraParamForX2(hbrt_funccall_info *fc, bpu_addr_t pym_uv_addr, bpu_addr_t debug_output_addr,
                               bpu_addr_t perf_counter_enable_flag, bpu_addr_t resizer_work_mode);

extern void setExtraParamForMatrix(hbrt_funccall_info *fc, uint64_t output_stride);

extern uint32_t cpu_buffer_size;  // CPU buffer is reserved in ARM memory for CPU operators.

/**
 * Call RiStart to generate a function call to run the first instruction sequnce of a model.
 * RiStart will also allocate resources to run this model.
 * @param model_name, name of a model to run
 * @param ri_id, input frame ID
 * @param output_mem_handle, output memory handle, got from API in system software library
 * @param input_mem_handle, input base address, got from API in system software library
 * @param funccall_info, address to save generated funccall info
 * @return
 */
extern int32_t hbrtRiStart(const char *model_name, uint32_t ri_id, bpu_addr_t bpu_output_addr,
                           bpu_addr_t bpu_input_addr, uint32_t interrupt_num, hbrt_funccall_info *funccall_info);

/**
 * Some models contain more than 1 instruction sequences, in which some instruction sequences
 * are JUST-IN-TIME compiled with a CPU operator. To generate function calls to execute all
 * these instructions, RiContinue should be called every time you receive a interrupt from
 * CNN core until it returns 0.
 * @param model_name, name of a model to run
 * @param ri_id, input frame ID
 * @param funccall_buffer, address to save generated funccall info
 * @return remaining function call to run
 */
extern int32_t hbrtRiContinue(uint32_t ri_id, uint32_t interrupt_num, hbrt_funccall_info *funccall_buffer);

extern int32_t hbrtRiNextSegmentIsOnCPU(uint32_t ri_id);

extern int32_t hbrtRiIsDone(uint32_t ri_id);

// extern void hbrtSyncOutput(uint32_t ri_id, bpu_addr_t cpu_dst_addr, bpu_addr_t bpu_src_addr, uint32_t output_idx);

/**
 * RiDestroy should be called when all outputs of an input frame are collected.
 * Resources allocated in RiStart will be released in RiDestroy
 * @param model_name, name of a model to run
 * @param ri_id, input frame ID
 */
extern void hbrtRiDestroy(uint32_t ri_id);

extern void hbrtPrintFunccall(hbrt_funccall_info *funccall);

extern void hbrtPrintResizerROI(roi_box_t *box);

#ifdef __cplusplus
}
#endif

#endif  // HBDK_HBRT_RUN_MODEL_H
