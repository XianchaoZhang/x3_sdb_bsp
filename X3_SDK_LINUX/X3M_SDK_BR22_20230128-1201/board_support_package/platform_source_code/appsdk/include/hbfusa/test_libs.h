/*************************************************************************
 *                     COPYRIGHT NOTICE
 *            Copyright 2016-2022 Horizon Robotics, Inc.
 *                   All rights reserved.
 *************************************************************************/
#ifndef TEST_LIBS_H_
#define TEST_LIBS_H_
#include <mqueue.h>
#include <stdint.h>
#include <stdbool.h>

#define EVNT_DDR_ECC	(1U)
#define EVNT_DDR_ECC_TO	(2U)
#define EVNT_BPU_SRAM	(1U)
#define EVNT_RCORE_ALU	(2U)
#define EVNT_BPU_SRAM_TO	(3U)
#define EVNT_SAFE_REG	(1U)
#define EVNT_SAFE_REG_TO	(2U)
#define EVNT_DDR_DATALN	(1U)
#define EVNT_DDR_DATALN_TO	(2U)
#define EVNT_SOC_TEMP	(1U)
#define EVNT_SOC_TEMP_TO	(2U)
#define EVNT_NVM_CRC	(1U)
#define EVNT_QA_BTWN_ACORE	(1U)
#define EVNT_QA_BTWN_ACORE_TO	(2U)
#define EVNT_DDR_ECC_FUNC	(1U)

/*
 * @Description: test case struct
 * @case_init: case init function pointer
 * @case_exec: case executefunction pointer
 * @case_deinit: case deinit function pointer
 * @fd: file descriptor
 * @bypass: bypass flag
 * @mq: messge queue id
 * @private_data: private data
 * @timestamp: time stamp of last event report,filter event when report
 * @err_flag: err_flag of last event report,filter event when report
 * @event_last: event reported last time,filter event when report
 */
typedef struct test_case {
	int32_t (*case_init)(struct test_case*); /* PRQA S 1336 */
	int32_t (*case_exec)(mqd_t, struct test_case*); /* PRQA S 1336 */
	int32_t (*case_deinit)(struct test_case*); /* PRQA S 1336 */
	int32_t fd;
	int32_t bypass;
	mqd_t mq;
	void *private_data;
	int64_t timestamp;
	uint8_t err_flag;
	uint8_t timeout_flag_last;
	uint32_t event_last;
	uint8_t periodic;
} test_case_t;

/* ============== enumber definition ============== */

/*
 * @Description: test case type
 * @TC_DDR_ECC_CHECK: periodic DDR ECC check
 * @TC_BPU_SRAM_CHECK: periodic bpu sram check
 * @TC_DDR_DATALINE_CHECK: periodic ddr dataline check
 * @TC_SAFE_REG_CHECK: periodic safety register check
 * @TC_SOC_TEMP_CHECK: periodic SOC temperature check
 * @TC_FLASH_CHECK: oneshot flash crc data check
 * @TC_QA_BWTN_ACORE: periodic Q&A check between Acore
 * @TC_DDR_ECC_FUNC_CHECK: oneshot DDR ECC functional check
 */
#define TC_DDR_ECC_CHECK		(0U)
#define TC_BPU_SRAM_CHECK		(1U)
#define TC_DDR_DATALINE_CHECK	(2U)
#define TC_SAFE_REG_CHECK		(3U)
#define TC_SOC_TEMP_CHECK		(4U)
#define TC_FLASH_CHECK			(5U)
#define TC_QA_BTWN_ACORE		(6U)
#define TC_DDR_ECC_FUNC_CHECK	(7U)
#define MAX_TC		(8U)

/* bpu sram complete 1 time per 2 loops */
#define BPU_SRAM_CK_LOOP		(2U)

/*
 * periodic testlib TC monitor
 * @TESTLIB_TC_OK: actural exicution periodic within FHTI
 * @TESTLIB_TC_TO: actural exicution periodic exceed FHTI
 */
#define TESTLIB_TC_OK (128U)
#define TESTLIB_TC_TO (129U)

/* Safety mechanism diagnose flag */
#define DIAG_ERR_STATUS_UKNOWN (1U)
#define DIAG_ERR_STATUS_OK (2U)
#define DIAG_ERR_STATUS_FAILED (3U)
#define DIAG_ERR_STATUS_MAX (4U)

/* ============== interface definition ============== */

/*
 * @Description: testlibs  init
 * @indx: which case to init, -1--init all or tc_type
 * @mq: queue id to send message
 * @return: 0 on success, others value on failure
 */
int32_t testlib_cases_init(int32_t indx, mqd_t mq);

/*
 * @Description: test libs execute
 * @indx: which test case to init,use tc_type
 * @return: 0 on success, others value on failure
 */
int32_t testlib_cases_exec(int32_t indx);

/*
 * @Description: test libs deinit
 * @indx: which test case to deinit,use tc_type
 * @return: 0 on success, others value on failure
 */
int32_t testlib_cases_deinit(int32_t indx);

#endif
