/***************************************************************************
 *   Copyright (C) 2019 by horizon.                                        *
 *   bo01.chen@horizon.ai                                                  *
 *                                                                         *
 *   Diag header file.                                                     *
 *                                                                         *
 ***************************************************************************/
#ifndef DIAG_LIB_H
#define DIAG_LIB_H

#ifdef __cplusplus
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
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <mqueue.h>
#include <diag_log.h>

#ifdef __cplusplus
}
#endif

#ifdef DIAG_LIB_GLOBALS
#define DIAG_LIB_EXT
#else
#define DIAG_LIB_EXT extern
#endif

#define IOCS_DIAG_DEV_SELF_TEST 0x01

extern int errno;

#define MSG_WITHOUT_ENV		0
#define MSG_WITH_ENV		1

// diag test msq
#define TEST_MSQ "/test_msq"

// netlink
#define NETLINK_DIAG 23
#define PORT 100  // diag process, netlink port
#define NETLINK_GROUP 1
#define DIAG_MSG_HEAD 0xAA
#define DIAG_MSG_VER 0x01
#define MESSAGE_EXTRA_SIZE 0x64  // extra data(eg. header)
#define DIAG_KERNEL_FRAGMENT_SIZE (0x4000 + MESSAGE_EXTRA_SIZE)
#define DIAG_AP_TO_AP_FRAGMENT_SIZE (0x1c00)  // 7kb

/* diag driver sta set */
#define IOCS_DIAG_DEV_UNMASK_ID 0x03
#define IOCS_DIAGDRIVER_STA 0x07
#define DIAGDRIVER_STARTWORK 0x01
#define DIAGDRIVER_STOPWORK 0x02

/* frame type */
#define DIAG_MSG_TYPE_REPORT_EVENT_STAT 1
#define DIAG_MSG_TYPE_SEND_ENV_DATA 2
#define DIAG_MSG_TYPE_READ_ENV_DATA 3

/*
 * module id and event id.
 */
#define MODULE_ID_MAX 0xFFFF
#define EVENT_ID_MAX 100

/*
 * event status.
 */
#define DIAG_EVENT_UNKNOWN 1
#define DIAG_EVENT_SUCCESS 2
#define DIAG_EVENT_FAIL 3
#define DIAG_EVENT_MAX 4

/*
 * env data generation timing.
 */
#define DIAG_GEN_ENV_DATA_WHEN_ERROR 1    // when error
#define DIAG_GEN_ENV_DATA_LAST_ERROR 2    // when chg to success
#define DIAG_GEN_ENV_DATA_WHEN_SUCCESS 3  // when snd general env data

/*
 * diag msg priority
 */
#define DIAG_MSG_PRIO_HIGH 1
#define DIAG_MSG_PRIO_MID 2
#define DIAG_MSG_PRIO_LOW 3
#define DIAG_MSG_PRIO_MAX 4

/*
 * diag app module id and event id
 * each module --- one entry.
 */
#define DIAG_APP_MODULE_SELFTEST 0x8001
#define DIAG_APP_EVENT_SELFTEST 0x0001

struct diag_unmask_id {
	uint16_t module_id;
	uint16_t event_id;
};

#define DIAGLIB_ERR_DATA_LEN	128

struct message {
	uint32_t msg_id;
	uint8_t msg_text[DIAG_AP_TO_AP_FRAGMENT_SIZE + MESSAGE_EXTRA_SIZE];
};

/* msg id: It consists of three parts.*/
struct diag_msg_id {
	uint16_t module_id;  // module id.
	uint16_t event_id;   // event id.
	uint8_t msg_pri;     // msg priroty.
};

typedef enum {
	DIAG_LIB_ERROR = -1,
	DIAG_LIB_OK,
	DIAG_LIB_NOT_EXIST,
} TDisg_lib_status;

#define NOT_SUPPORT -1;
#define DIAGLIB_ERR_DATA_LEN	128

typedef struct diaglib_err {
	uint8_t err_id;
	uint8_t err_level;
	uint8_t err_flag;
	uint8_t when;
	uint16_t module_id;
	uint8_t err_data[DIAGLIB_ERR_DATA_LEN];
	uint8_t env_len;
	uint8_t sys_err_level;
} diaglib_err_t;

typedef struct diaglib_err_msg {
	diaglib_err_t err;
	uint8_t msg_type;
	uint32_t checksum;
	uint32_t dummy;
} diaglib_err_msg_t;

typedef struct diag_alivetick {
    uint8_t step1;
    uint8_t step2;
    uint8_t step3;
    uint8_t step_invalid;
    uint16_t module_id;
    uint64_t time_stamp_ms;
    uint64_t checksum;
} diag_alivetick_t;

// extern TDisg_lib_status diag_msg_app_unmask_id_set(struct diag_unmask_id
// *unmask_id);

/*
 * send event sta and it's env data to the diag app.
 * @id module id, event id, msg priority
 * @event_sta event sta
 * @env_data_gen_timing When is the environmental data generated.
 * @env_data env data ptr
 * @env_len env data len.
 * @return -1:error, >=0:OK
 */

#ifdef __cplusplus
extern "C" {
#endif
int diag_exist();

int diag_send_event_stat_and_env_data(
		uint8_t msg_prio, uint16_t module_id, uint16_t event_id,
		uint8_t event_sta, uint8_t env_data_gen_timing, uint8_t *env_data,
		size_t env_len);

int diag_send_event_stat(uint8_t msg_prio, uint16_t module_id,
						uint16_t event_id, uint8_t event_sta);

#ifdef __cplusplus
}
#endif
DIAG_LIB_EXT TDisg_lib_status diag_ipc_lib_init(void);
DIAG_LIB_EXT TDisg_lib_status diag_ipc_lib_deinit(void);
DIAG_LIB_EXT TDisg_lib_status diag_ipc_lib_rcvmsg(struct message *pmsg);
DIAG_LIB_EXT TDisg_lib_status diag_ipc_lib_sndmsg(int32_t qid, uint8_t *buffer,
													uint32_t len);

DIAG_LIB_EXT TDisg_lib_status diag_netlink_lib_init(void);
DIAG_LIB_EXT TDisg_lib_status diag_netlink_lib_deinit(void);
// DIAG_LIB_EXT TDisg_lib_status diag_netlink_lib_rcvmsg(user_msg_info *pmsg);
DIAG_LIB_EXT TDisg_lib_status diag_netlink_lib_sndmsg(uint8_t *buff,
                                                      uint32_t len);
/* everything is ok, now start diag driver */
DIAG_LIB_EXT TDisg_lib_status
diag_netlink_lib_driver_sta_set(uint8_t ioctrlsta);
DIAG_LIB_EXT TDisg_lib_status diag_netlink_get_mod_event_sta(uint32_t moduleid,
                                                             uint32_t event_id);
extern int32_t send_sfmu_msg(diaglib_err_msg_t *msg, uint32_t if_block);
extern int32_t receive_sfmu_msg(diaglib_err_msg_t *msg);
extern int32_t diag_set_alivetick(diag_alivetick_t *alivetick);
extern int32_t diag_send_alivetick_msg(diag_alivetick_t alivetick);
extern int32_t diag_clear_alivetick(diag_alivetick_t *alivetick);

DIAG_LIB_EXT TDisg_lib_status diag_sfmu_lib_init(void);
DIAG_LIB_EXT TDisg_lib_status diag_sfmu_lib_deinit(void);
DIAG_LIB_EXT TDisg_lib_status diag_sfmu_lib_rcvmsg(diaglib_err_msg_t *msg);

#endif
