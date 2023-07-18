/***************************************************************************
 *   Copyright (C) 2019 by horizon.                                        *
 *                                                                         *
 *   diag_service header file.                                              *
 *                                                                         *
 ***************************************************************************/
#ifndef INCLUDE_DIAG_SERVICE_H_
#define INCLUDE_DIAG_SERVICE_H_

#ifdef __CPLUSPLUS
extern "C" {
#endif

#include <errno.h>
#include <fcntl.h>
#include <linux/netlink.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#ifdef __CPLUSPLUS
}
#endif

#ifdef SPI_SERVICE_GLOBALS
#define SPI_SERVICE_EXT
#else
#define SPI_SERVICE_EXT extern
#endif

#define DIAG_REQ_ID_FROM_MCU 0x0202
#define DIAG_RESP_ID_TO_MCU 0x0302
#define DIAG_COMM_TYPE_SET 0x0003

typedef enum {
	diag_spi_resp_null,
	diag_spi_resp_req_err,
	diag_spi_resp_uds_oping,
	diag_spi_resp_uds_res,
	diag_spi_resp_end
} diag_spi_resp_type;
SPI_SERVICE_EXT diag_spi_resp_type diag_spi_resp_map_type;

#define REQ_FROM_MCU_NUM_MAX 30
typedef struct {
	char req_from_mcu[REQ_FROM_MCU_NUM_MAX];
	char *resp_to_mcu;
} spi_il_data;
SPI_SERVICE_EXT spi_il_data spi_il_msg;
SPI_SERVICE_EXT char *period_to_mcu;

typedef struct {
	uint8_t comm_type[2];
} __attribute__((packed)) spi_diag_il_head;

typedef struct {
	spi_diag_il_head head;
	uint8_t *data;
} __attribute__((packed)) spi_diag_il_pack;
SPI_SERVICE_EXT spi_diag_il_pack spi_diag_req;
SPI_SERVICE_EXT spi_diag_il_pack spi_diag_resp;
SPI_SERVICE_EXT spi_diag_il_pack spi_diag_period_resp;

typedef enum {
	spi_service_sts_dis = 0,
	spi_service_sts_en,
	spi_service_sts_end
} enspi_service_sts;
SPI_SERVICE_EXT enspi_service_sts spi_service_sts;

SPI_SERVICE_EXT int spi_service_init(void);
SPI_SERVICE_EXT void spi_service_deinit(void);
SPI_SERVICE_EXT void spi_service_process(void);
SPI_SERVICE_EXT void spi_period_process(void);
#endif  // INCLUDE_DIAG_SERVICE_H_
