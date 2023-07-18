/*
 * Horizon Robotics
 *
 *  Copyright (C) 2020 Horizon Robotics Inc.
 *  All rights reserved.
 *  Author: zenghao.zhang<zenghao.zhang@horizon.ai>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __DIAG_LIB_H
#define __DIAG_LIB_H

#define DIAGLIB_ERR_DATA_LEN	128
#define MSG_WITHOUT_ENV		0
#define MSG_WITH_ENV		1

#define NON_BLOCK	0
#define BLOCK		1

#define DIAGLIB_ALIVETICK_INVALID_STEP	0
#define DIAGLIB_ALIVETICK_STEP1     	1
#define DIAGLIB_ALIVETICK_STEP2     	2
#define DIAGLIB_ALIVETICK_STEP3     	3

#define DIAGLIB_ALIVETICK_TIME_UPPER_LIMIT_MS   132
#define DIAGLIB_ALIVETICK_TIME_LOWER_LIMIT_MS   33

#define TOTAL_APP_MODULE_NUM 5

#ifndef FUSA_DEBUG
// #define FUSA_DEBUG
#endif

/*
 * @Description: alive tick for program sequence check
 * @step1: first step in program sequence
 * @step2: second step in program sequence
 * @step3: third step in program sequence
 * @step_invalid: intened to be INVALIDSTEP
 * @module_id: module index
 * @timestamp_ms: timestamp in millisecond
 * @checksum: checksum of diag alivetick message
 */
typedef struct diag_alivetick {
    uint8_t step1;
    uint8_t step2;
    uint8_t step3;
    uint8_t step_invalid;
    uint16_t module_id;
    uint64_t time_stamp_ms;
    uint64_t checksum;
} diag_alivetick_t;

/*
 * @Description: diaglib error definition
 * @err_id: error ID, each module may have several error IDs.
 * @err_level: error level, three levels.
 * @err_flag: error flag to mark whether it is an error or not.
 * @when: time of currence, maybe unsed.
 * @module_id: module ID.
 * @err_data: error data, maybe unused.
 * @env_len: data length of err_data.
 * @sys_err_level: current system err level,updated across message information
 */
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

/*
 * @Description: diaglib error message definition
 * @err: diaglib error information
 * @msg_type: MSG_WITHOUT_ENV or MSG_WITH_ENV
 * @checksum: checksum of error data
 * @dummy: placeholder for future use.
 */
typedef struct diaglib_err_msg {
	diaglib_err_t err;
	uint8_t msg_type;
	uint32_t checksum;
	uint32_t dummy;
} diaglib_err_msg_t;

/*
 * @Description: receive message from sfmu
 * @msg: message buffer pointer to store message to
 * @return: 0 on success, others value on failure
 */
int32_t hb_receive_sfmu_msg(diaglib_err_msg_t *msg);

/*
 * @Description: send message to sfmu
 * @msg: message to be send
 * @block: BLOCK or NON_BLOCK
 * @return: 0 on success, others value on failure
 */
int32_t hb_send_sfmu_msg(diaglib_err_msg_t *msg, uint32_t block);

/*
 * @Description: set alivetick message
 * alivetick: alivetick message
 * @return: 0 on success, others value on failure
 */
int32_t hb_diag_set_alivetick(diag_alivetick_t *alivetick);

/*
 * @Description: send alivetick message
 * alivetick: alivetick message
 * @return: 0 on success, others value on failure
 */
int32_t hb_diag_send_alivetick_msg(diag_alivetick_t alivetick);

/*
 * @Description: clear alivetick message
 * alivetick: alivetick message
 * @return: 0 on success, others value on failure
 */
int32_t hb_diag_clear_alivetick(diag_alivetick_t *alivetick);

/*
 * Description: receive alivetick data from user space
 * @alivetick: buffer to store message to
 * Return: 0 on success, -1 on fail
 */
int32_t diag_alivetick_rcvmsg(diag_alivetick_t *alivetick);

#endif //__DIAG_LIB_API__H
