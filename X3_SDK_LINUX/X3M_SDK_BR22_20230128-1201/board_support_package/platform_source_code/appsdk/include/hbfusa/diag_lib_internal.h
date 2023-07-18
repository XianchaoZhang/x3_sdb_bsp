/*
 * Horizon Robotics
 *
 *  Copyright (C) 2020 Horizon Robotics Inc.
 *  All rights reserved.
 *  Author: leye.wang<leye.wang@horizon.ai>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __DIAG_LIB_INTERNAL_H
#define __DIAG_LIB_INTERNAL_H

#include <unistd.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <mqueue.h>
#include <linux/netlink.h>
#include <sys/socket.h>
#include <errno.h>

#include "./diag_lib.h"
/* ============== Error code definition ============= */

/* ============== structure definition ============= */

#define  TESTLIB_MQ_NAME "/testlib_mq"
#define  SFMU_MQ_NAME "/sfmu_mq"

#define ERR_PIN_FATAL		1
#define ERR_PIN_NORMAL	0
#define FUSA_FATAL_MAGIC 'f'
#define FUSA_SET_ERROR_PIN             _IOW(FUSA_FATAL_MAGIC, 0, unsigned int)

/* ============== interfaces definition ============= */

/*
 * @Description: create netlink socket to interact with diagnose driver
 * @return: 0 on success, negative value on failure
 */
int32_t diaglib_netlink_init(void);

/*
 * @Description: destroy netlink socket
 * @sockfd: socket file descriptor
 * @return: 0 on success, negative value on failure
 */
int32_t diaglib_netlink_deinit(int32_t sockfd);

/*
 * @Description: receive message by netlink
 * @sockfd: socket file descriptor
 * @msg: message buffer pointer to store message to
 * @return: 0 on success, negative value on failure
 */
int32_t diaglib_netlink_recv_msg(int32_t sockfd, diaglib_err_msg_t *msg);

/*
 * @Description: send message by netlink
 * @sockfd: socket file descriptor
 * @msg: message to be send
 * @return: 0 on success, negative value on failure
 */
int32_t diaglib_netlink_send_msg(int32_t sockfd, diaglib_err_msg_t *msg);

/*
 * @Description: create posix message queue
 * @mq_name: message queue name
 * @if_block: BLOCK or NON_BLOCK
 * @return: mq descriptor on success, system defined error on failure
 */
mqd_t diaglib_mq_create(const char *mq_name, uint32_t if_block);

/*
 * @Description: destroy posix message queue
 * @mq_name: message queue name to remove
 * @mq: message queue descriptor
 * @return: 0 on success, negative value on failure
 */
int32_t diaglib_mq_destroy(const char *mq_name, mqd_t mq);

/*
 * @Description: receive message by posix message queue
 * @mq: mq file descriptor
 * @msg: message buffer pointor to store message to
 * @return: 0 on success, negative value on failure
 */
int32_t diaglib_mq_recv_msg(mqd_t mq, diaglib_err_msg_t *msg);

/*
 * @Description: send message by posix message queue
 * @mq: mq file descriptor
 * @msg: message to be send
 * @return: 0 on success, negative value on failure
 */
int32_t diaglib_mq_send_msg(mqd_t mq, diaglib_err_msg_t *msg);

/*
 * @Description: calculate the checksum of diaglib_err_t
 * @msg: message to be calculated
 */
void diaglib_calculate_checksum(diaglib_err_msg_t *msg);

#if 0 // obsolete API to be removed
/*
 * @Description: open file
 * @pin: pin number
 * @return: fd on success, negative value on failure
 */
int32_t diaglib_operate_gpio_init(uint16_t pin);

/*
 * @Description: close file
 * @fd: file descriptor
 * @pin: pin number
 * @return: 0 on success, negative value on failure
 */
int32_t diaglib_operate_gpio_deinit(int32_t fd, uint16_t pin);

/*
 * @Description: operate gpio pin, pull up or down.
 * @fd: file descriptor
 * @opcode: pull up or pull down
 * @return: 0 on success, negative value on failure
 */
int32_t diaglib_operate_gpio_pin(int32_t fd, uint32_t opcode);
#endif // obsolete API to be removed

/*
 * @Description: record diagnose information
 * @name: file name to store the information
 * @msg: message
 * @return: 0 on success, negative value on failure
 */
int32_t diaglib_record_diag_info(char *file_name, diaglib_err_msg_t *msg);

// /*
//  * @Description: verify program execute sequence
//  * @msg: alive tick message
//  * @return: 0 on success, negative value on failure
//  */
// int32_t diaglib_verify_program_seq(diaglib_alivetick_msg_t *msg);

/*
 * @Description: init error pin
 * @input:none
 * @return: fd on success, negative value on failure
 */
int32_t diaglib_error_pin_init();

/*
 * @Description: operate error pin, pull up or down.
 * @fd: file descriptor
 * @opcode: ERR_PIN_FATAL or ERR_PIN_NORMAL
 * @return: 0 on success, negative value on failure
 */
int32_t diaglib_operate_error_pin(int32_t fd, uint32_t opcode);

/*
 * @Description: close err pin
 * @fd: err pin descriptor to close
 * @return: 0 on success, negative value on failure
 */
int32_t diaglib_error_pin_deinit(int32_t fd);
#endif //__DIAG_LIB_INTERNAL_H
