/***************************************************************************
 *   Copyright (C) 2019 by horizon.                                        *
 *   zenghao.zhang@horizon.ai                                                  *
 *                                                                         *
 *   Diag share memory header file.                                                     *
 *                                                                         *
 ***************************************************************************/
#ifndef DIAGSHM_DIAG_SHM_H_
#define DIAGSHM_DIAG_SHM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <unistd.h>
#include <sys/mman.h>
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <diag_log.h>

#ifdef __cplusplus
}
#endif

#ifdef DIAG_SHM_GLOBALS
#define DIAG_SHM_EXT
#else
#define DIAG_SHM_EXT extern
#endif

struct dtc_id_base_index {
	uint16_t dtc_id_base;
	uint8_t dtc_id_offset;
};

typedef enum _dtc_type {
	DTC_KERNEL,
	DTC_MODEL,
	DTC_PERCEPTION,
	DTC_ADAS,
	DTC_AP,
	DTC_MAX
} dtc_type_e;

#define KERNEL_DTC_ID_OFFSET 0
#define MODEL_DTC_ID_OFFSET 4
#define PERCEPRION_DTC_OFFSET 6
#define ADAS_DTC_OFFSET 10
#define AP_DTC_OFFSET 12

#define KERNEL_DTC_ID_BASE 0x0000
#define MODEL_DTC_ID_BASE 0x1000
#define PERCEPTION_DTC_ID_BASE 0x2000
#define ADAS_DTC_ID_BASE 0x3000
#define AP_DTC_ID_BASE 0x4000

extern int g_shm_fd;
extern uint8_t *g_shm_data;
extern struct dtc_id_base_index dtc_index[];

typedef enum {
	DIAG_SHM_ERROR = -1,
	DIAG_SHM_OK,
} diag_shm_status;

typedef struct _diag_data_ {
	/* data */
	uint16_t dtc_id_base;
	uint8_t dtc_status_len;
	uint8_t *dtc_status;
} diag_data_t;

#ifdef __cplusplus
extern "C" {
#endif

int8_t get_diag_summary(uint8_t *ptr, uint8_t *len);
int8_t get_diag_by_dtcid(uint16_t dtc_id);
int parse_dtc_offset(uint16_t dtc_id);

#ifdef __cplusplus
}
#endif


DIAG_SHM_EXT diag_shm_status diag_shm_lib_init(void);
DIAG_SHM_EXT diag_shm_status diag_shm_lib_deinit(void);

#endif // DIAGSHM_DIAG_SHM_H_
