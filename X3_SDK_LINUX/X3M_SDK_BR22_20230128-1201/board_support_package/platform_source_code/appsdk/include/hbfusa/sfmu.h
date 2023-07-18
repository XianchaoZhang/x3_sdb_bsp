/*************************************************************************
 *                     COPYRIGHT NOTICE
 *            Copyright 2016-2022 Horizon Robotics, Inc.
 *                   All rights reserved.
 *************************************************************************/
#ifndef SFMU_H
#define SFMU_H
#include <mqueue.h>
#include <pthread.h>
#include <stdint.h>
#include <stdbool.h>
#include "diag_lib_internal.h"
#include "fusa_dtc.h"

#define J3_SOC 1

/* ============== structure definition ============= */
#define LIBFUSA_VERSION  "V0.0.4"
#define IOCS_DIAGDRIVER_STA 0x07
#define IOCS_SEND_SFMU_ERROR 0x08 /* PRQA S 1534 */
#define DIAGDRIVER_STARTWORK 0x01
#define DIAGDRIVER_STOPWORK 0x02

/* to compensate the wake up dalay(millisecond) */
#define SCHED_TORLERANCE_TIME 50

#define STATIC_BUF_SIZE_MAX	(290U)
#define BUF_LEN_64	(64U)

/* SM fault flag */
#define FLAG_SM_FAULT (1U)

/*
 * Description: debounce count get
 * bit31               bit1 bit0
 * |<- debounce count ->|     |
 * @cnt: SM fault statistic data
 * Return: debounce count
 * */
static inline uint32_t DBC_CNT(uint32_t cnt)
{
	uint32_t dbc_cnt;
	dbc_cnt = cnt >> FLAG_SM_FAULT;

	return dbc_cnt;
}

/* range of pthread priority of SCHED_FIFO in safety library */
#define PRIO_DEF 1
#define PRIO_MAX 10

/* max camera number */
#define MAX_CAM_NUM 4
#define NAME_MAX_LEN  64

/* env_data protocol payload offset */
#define PROTO_ENV_DATA_CHNL		(0U)
#define PROTO_ENV_DATA_FTYPE	(1U)
#define PROTO_ENV_DATA_RSV		(2U)
#define PROTO_ENV_DATA_LEN		(3U)
#define PROTO_ENV_DATA_PAYLOAD	(4U)
#define PROTO_ENV_DATA_HDR_LEN	(4U)
#define PROTO_ENV_DATA_INVAL	(0xffU)

/* member offset of soc_temp_info_t */
#define SOC_TEMP_CUR_OFF	(PROTO_ENV_DATA_PAYLOAD + 0U)
#define SOC_TEMP_LAST_OFF	(SOC_TEMP_CUR_OFF + 4U)
#define SOC_TS_CUR_OFF		(SOC_TEMP_LAST_OFF + 4U)
#define SOC_TS_LAST_OFF		(SOC_TS_CUR_OFF + 8U)

#define I2C_SLV_ADDR_OFF		PROTO_ENV_DATA_PAYLOAD

#define BYTE_WIDTH		(8)
#define SOC_TEMP_RES	(1000)
#define MS_PER_SEC		(1000)
#define US_PER_MS		(1000)
#define NS_PER_MS		(1000000)
#define US_PER_SEC		(1000000)
#define NS_PER_US		(1000)

#define ERR_IPU_SIZE_W	(1U)
#define ERR_IPU_SIZE_H	(2U)
#define ERR_IPU_SIZE	(ERR_IPU_SIZE_H | ERR_IPU_SIZE_W)
#define ERR_TYPE_IPU_SIZE	(1U)
#define ERR_TYPE_IPU_DROP	(2U)

/*
 * Description: load uint16_t data from specified address
 * @addr: address data load from
 * Return: data of read
 * */
static inline uint16_t DATA_TO_UINT16(const uint8_t *addr)
{
	uint16_t ret = 0;
	uint16_t tmp;

	for (uint8_t i = 0; i < sizeof(tmp); i++) {
		tmp = (uint16_t)addr[i];
		ret |= (uint16_t)(tmp << (i * (uint8_t)BYTE_WIDTH));
	}

	return ret;
}

/*
 * Description: load uint32_t data from specified address
 * @addr: address data load from
 * Return: data of read
 * */
static inline uint32_t DATA_TO_UINT32(const uint8_t *addr)
{
	uint32_t ret = 0;
	uint32_t tmp;

	for (uint8_t i = 0; i < sizeof(tmp); i++) {
		tmp = (uint32_t)addr[i];
		ret |= tmp << (i * (uint8_t)BYTE_WIDTH);
	}

	return ret;
}

/*
 * Description: load uint64_t data from specified address
 * @addr: address data load from
 * Return: data of read
 * */
static inline uint64_t DATA_TO_UINT64(const uint8_t *addr)
{
	uint64_t ret = 0;
	uint64_t tmp;

	for (uint8_t i = 0; i < sizeof(tmp); i++) {
		tmp = (uint32_t)addr[i];
		ret |= tmp << (i * (uint8_t)BYTE_WIDTH);
	}

	return ret;
}

/*
 * Description: store uint16_t data to specified address
 * @addr: address to store to
 * @data: data to be stored
 * */
static inline void DATA_STR_UINT16(uint8_t *addr, uint16_t data)
{
	for (uint8_t i = 0; i < sizeof(uint16_t); i++) {
		addr[i] = (uint8_t)(data >> (i * 8U));
	}
}

/*
 * Description: store uint32_t data to specified address
 * @addr: address to store to
 * @data: data to be stored
 * */
static inline void DATA_STR_UINT32(uint8_t *addr, uint32_t data)
{
	for (uint8_t i = 0; i < sizeof(uint32_t); i++) {
		addr[i] = (uint8_t)(data >> (i * 8));
	}
}

/*
 * Description: store uint64_t data to specified address
 * @addr: address to store to
 * @data: data to be stored
 * */
static inline void DATA_STR_UINT64(uint8_t *addr, uint64_t data)
{
	for (uint8_t i = 0; i < sizeof(uint64_t); i++) {
		addr[i] = (uint8_t)(data >> (i * 8));
	}
}

#define max(a,b) \
		({__typeof__ (a) _a = (a); \
			__typeof__ (b) _b = (b); \
			(_a > _b) ? _a : _b;})

#define min(a,b) \
		({__typeof__ (a) _a = (a); \
			__typeof__ (b) _b = (b); \
			(_a < _b) ? _a : _b;})

/*
 * message process return code definition
 * RC_0:process OK,safety action followed
 * RC_1:valid module ID but none safety related,route followed
 */
#define RC_0 (0)
#define RC_1 (1)

/*
 * soc error level
 */
#define ERR_LEVEL1 (1U)
#define ERR_LEVEL2 (2U)
#define ERR_LEVEL3 (3U)

/*
 * Description: camera status
 * CAM_LOCK: lvds lock
 * CAM_UNLOCK: lvds unlock
 * */
enum cam_status {
	CAM_UNLOCK,
	CAM_LOCK
};

/*
 * all modules exist error level2
 */
typedef enum _module_type {
	/* testlib TC */
	MD_DDR_ECC_CK = 0,
	MD_BPU_SRAM_CK,
	MD_DDR_DATALINE_CK,
	MD_SAFE_REG_CK,
	MD_SOC_TEMP_CK,
	MD_ACORE_STAT,
	MD_DDR_ECC_FUNC_CK,
	/* periodic TC timeout */
	MD_DDR_ECC_CK_TO,
	MD_BPU_SRAM_CK_TO,
	MD_DDR_DATALINE_CK_TO,
	MD_SAFE_REG_CK_TO,
	MD_SOC_TEMP_CK_TO,
	MD_ACORE_STAT_CK_TO,
	/* kmodules */
	MD_I2C0,
	MD_I2C1,
	MD_I2C2,
	MD_I2C3,
	MD_I2C4,
	MD_I2C5,
	MD_UART0,
	MD_UART1,
	MD_UART2,
	MD_UART3,
	MD_SPI0,
	MD_SPI1,
	MD_SPI2,
	MD_EMMC,
	MD_NORFLASH,
	MD_SPI_FRAME_OUTOF_ORDER,
	MD_SPI_FRAME_LOSS,
	MD_SPI_CRC_ERROR,
	MD_SPI_TIMEOUT,

	MD_VIO_MIPI_HOST_0,
	MD_VIO_MIPI_HOST_1,
	MD_VIO_MIPI_HOST_2,
	MD_VIO_MIPI_HOST_3,
	MD_VIO_MIPI_DEV,
	MD_VIO_SIF,
	MD_VIO_IPU,
	MD_VIO_ISP_DROP,
	MD_VIO_FRAME_LOST,
	MD_VIO_GDC_0,
	MD_VIO_GDC_1,
	MD_VIO_PYM,
	MD_CAMERA_0,
	MD_CAMERA_1,
	MD_CAMERA_2,
	MD_CAMERA_3,
	MD_MPU,
	MD_ETH,
	MD_IMG_TO,
	MD_COM_SPI,
	MD_COM_ETH,
	MD_COM_UART,
	MAX_MD
} module_type_e;

struct mq_st {
	mqd_t testlib_mq;
	mqd_t sfmu_mq;
};

/* safety mechanism */
#define SM_CFG_FILE "/etc/fusa/j3_sm_conf.json"

enum safety_sm {
	/* kernel module */
	SM_MIPI_HOST_0,
	SM_MIPI_HOST_1,
	SM_MIPI_HOST_2,
	SM_MIPI_HOST_3,
	SM_MIPI_DEV,
	SM_SIF,
	SM_IPU,
	SM_IPU_SIZE,
	SM_ISP,
	SM_VIO_FL,
	SM_SPI_0,
	SM_SPI_1,
	SM_SPI_2,
	SM_I2C_0,
	SM_I2C_1,
	SM_I2C_2,
	SM_I2C_3,
	SM_I2C_4,
	SM_I2C_5,
	SM_EMMC,
	SM_PYM,
	SM_PMU,
	SM_ETH,
	SM_GDC_0,
	SM_GDC_1,
	SM_BPU0,
	SM_BPU1,
	SM_UART_0,
	SM_UART_1,
	SM_UART_2,
	SM_UART_3,
	SM_QSPI,
	SM_ACORE_CAL,
	SM_MPU,
	/* testlib tc */
	SM_DDR_ECC,
	SM_BPU_SRAM,
	SM_DDR_DL,
	SM_SAFETY_REG,
	SM_SOC_TEMP,
	SM_SOC_FLASH,
	SM_QA_BTWN_ACORE,
	SM_DDR_ECC_FUNC,
	/* external */
	SM_BIF_SPI,
	SM_PRG_FLOW,
	SM_PRG_ERR,
	SM_IMG_TO,
	SM_BPU_FC_TO,
	SM_SPI_COM,
	SM_ETH_COM,
	SM_UART_COM,
	/* camera */
	SM_CAMERA_0,
	SM_CAMERA_1,
	SM_CAMERA_2,
	SM_CAMERA_3,
	SM_MAX
};

typedef struct _thread_attr {
	int32_t priority;
	char policy[16];
} thread_attr_t;

/*
 * Description: Safety mechanism configuration
 * @safetylib: safety process on/off
 * @fhti: Fault Handling Time Interval,unit ms
 * @soc_temp_th1: SOC temperature threshold 1
 * @soc_temp_th2: SOC temperature threshold 2
 * @soc_temp_th3: SOC temperature threshold 3
 * @soc_temp_dur_th: SOC temperature duration threshold
 * @dbc_cnt: debounce count
 * @dbc_time: debouce time(ms)
 * @testlib: testlib all TC check on/off
 * @kmodules: kernel module event process on/off
 * @extern_tc: external TC process on/off
 * @sm: individual SM on/off
 * @testlib_exe: testlib TC execution thread attibution
 * @testlib_diag: testlib TC message receive & process thread attibution
 * @kernel_diag: kernel message receive & process thread attibution
 * @bpu_sram_check: bpu sram check execution thread attibution
 * @qa_master: Acore QA master thread attibution
 * @qa_slave: Acore QA slave thread attibution
 */
typedef struct _sm_switch {
	int8_t safetylib;
	int32_t fhti;
	uint8_t soc_temp_th1;
	uint8_t soc_temp_th2;
	uint8_t soc_temp_th3;
	uint16_t soc_temp_dur_th;
	uint8_t dbc_cnt;
	uint8_t dbc_time;
	int8_t testlib;
	int8_t kmodules;
	int8_t extern_tc;
	uint8_t sm[SM_MAX];
	thread_attr_t testlib_exe;
	thread_attr_t testlib_diag;
	thread_attr_t kernel_diag;
	thread_attr_t bpu_sram_check;
	thread_attr_t qa_master;
	thread_attr_t qa_slave;
} sm_switch_t;

/* Description: camera configuration
 * i2c_bus: i2c bus camera connected to
 * safety: safety related or not
 * ser_addr: serial address
 * des_addr: deserial address
 * cam_addr: camera address
 * id: camera identifier
 * name: camera name
 */
typedef struct _cam_info {
	uint8_t i2c_bus;
	uint8_t safety;
	uint16_t ser_addr;
	uint16_t des_addr;
	uint16_t cam_addr;
	uint16_t id;
	uint8_t name[NAME_MAX_LEN];
} cam_info_t;

/*
 * Description: Safety fault statistics
 * @lock: lock of data
 * @level: fault level of specific SM
 * @cnt: fault times occured
 * @ts: fault timestamp last time
 * @duration: fault duration
 */
typedef struct _sm_fault_stat {
	pthread_rwlock_t lock;
	uint8_t level[MAX_MD];
	uint32_t cnt[MAX_MD];
	int64_t ts[MAX_MD];
	int64_t duration[MAX_MD];
} sm_fault_stat_t;

/*
 * Discription: SOC termperature information
 * @temp_cur: current temperature
 * @temp_last: last temperature
 * @ts_cur: current timestamp
 * @ts_last: last timestamp
 */
typedef struct _soc_temp_info {
	int32_t temp_cur;
	int32_t temp_last;
	uint64_t ts_cur;
	uint64_t ts_last;
} soc_temp_info_t;

/* ============== interfaces definition ============= */

/*
 * @Description: sfmu init
 * @return: 0 on success, others value on failure
 */
int32_t sfmu_init(void);

/*
 * @Description: sfmu deinit
 * @return: 0 on success, others value on failure
 */
int32_t sfmu_deinit(void);

/*
 * @Description: execute sfmu
 * @return: 0 on success, others value on failure
 */
int32_t sfmu_exec(void);

/*
 * Description: update sys_err_level to target.Note that when level goes
 *              down,sys_err_level may not set to target,if some other SMs
 *              still at a higher level
 * @perr: update sys_err_level according to perr->err_level
 * Return: -1 on fail,0 on success
 */
int32_t update_sys_err(diaglib_err_t *perr);

/*
 * Description: update periodic testlib TC timestamp
 * @msg: update according to msg->err.module_id
 */
void intern_testlib_tc_ts_update(diaglib_err_msg_t *msg);

/*
 * Description: check whether internal periodic test case timeout or not
 * @type: module to be checked
 * Return: 1 true,0 false
 */
int32_t intern_testlib_tc_timeout(uint32_t type, uint16_t event);

/*
 * Description: create posix thread
 * @thread: address to store thread handle to
 * @attr: attribure pointer,NULL if no need
 * @func: thread function
 * @arg: thread argument,NULL if no need
 * return: 0 on success,-1 on fail
 * */
int32_t fusa_pthread_create(pthread_t *thread,
                const thread_attr_t *attr,
                void *(*func)(void *), /* PRQA S 1336 */
                void *arg);
#endif
