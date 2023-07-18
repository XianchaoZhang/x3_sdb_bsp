#ifndef _HBRT_UTIL_H_
#define _HBRT_UTIL_H_

#include "plat_cnn.h"
#include "stdint.h"
#include "stdlib.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HBDK_DEBUG 0
#define HBDK_LOG_LEVEL 0

#define ARM_MEM_BUFFER_SIZE (256 * 1024)
#define MAX_FINAL_OUTPUT_NUM 32
#define CNN_CODE_CACHE_SLOT_SIZE (4 * 1024 * 1024)
#define DMA_ALIGN_SIZE 64

#define CEIL_ALIGN(len) ((((len) + DMA_ALIGN_SIZE - 1) / DMA_ALIGN_SIZE) * DMA_ALIGN_SIZE)

#ifndef USE_SYS_LIB
extern uintptr_t cnn_mem_base;
#endif

extern uint64_t bpu_core0_program_table_addr;
extern uint64_t bpu_core1_program_table_addr;
extern uint64_t bpu_core0_model_dynamic_addr;
extern uint64_t bpu_core1_model_dynamic_addr;
extern uint64_t bpu_debug_perf_info_addr;
extern int debug_dump_flag;

extern int all_indata_info_initialized[32];
extern int all_outdata_info_initialized[32];
extern int final_indata_info_initialized[32];
extern int final_outdata_info_initialized[32];
extern int segment_info_initialized[32];

extern void bswap16bytes(char *src, size_t size);
extern void bswap4bytes(char *src, size_t size);

#if HBDK_DEBUG
#define WHEN_DEBUG(a) \
  { a; }
#else
#define WHEN_DEBUG(a)
#endif

inline char *strip_filename(const char *file) {
  char *name = NULL;
  char *ptr = (char *)file;
  if (file == NULL) return NULL;
  while (*ptr != '\0') {
    if (*ptr == '/') name = ptr;
    ptr++;
  }
  return name == NULL ? (char *)file : (name + 1);
}

#define RUNTIME_ASSERT(cond, ...)                                                                                 \
  if (!(cond)) {                                                                                                  \
    fprintf(stderr, "[RUNTIME ERROR] LINE %d in %s from %s\n", __LINE__, __FUNCTION__, strip_filename(__FILE__)); \
    fprintf(stderr, __VA_ARGS__);                                                                                 \
    fprintf(stderr, "\n");                                                                                        \
    abort();                                                                                                      \
  }

#define RUNTIME_LOG(log_level, header, ...) \
  if (log_level <= HBDK_LOG_LEVEL) {        \
    fflush(stdout);                         \
    if (header) {                           \
      printf("[RUNTIME INFO] ");            \
    }                                       \
    printf(__VA_ARGS__);                    \
  }

#define RUNTIME_LOG_FLUSH() fflush(stdout)

extern int GetFpgaInfo();

#ifdef __cplusplus
}
#endif

#endif
