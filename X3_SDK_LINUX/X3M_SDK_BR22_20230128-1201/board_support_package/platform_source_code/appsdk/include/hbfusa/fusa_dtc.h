#ifndef _FUSA_DTC_H
#define _FUSA_DTC_H

/**
 * DTC code define detail refer to
 * hbre/diagnose/inclue/config_file_read.h(prj_mono)
 */

/* Module ID define start */

#define KERNEL_MODULE_ID_MIN	((uint16_t) 0x0001)
#define KERNEL_MODULE_ID_MAX	((uint16_t) 0x0FFF)

#define TESTTLIB_MODULE_ID_MIN	((uint16_t) 0x4001)
#define TESTTLIB_MODULE_ID_MAX	((uint16_t) 0x4FFF)

#define PERCEPTION_MODULE_ID_MIN	((uint16_t) 0x9001)
#define PERCEPTION_MODULE_ID_MAX	((uint16_t) 0x9FFF)

#define EXTERN_TC_MODULE_ID_MIN		((uint16_t) 0x1001)
#define EXTERN_TC_MODULE_ID_MAX		((uint16_t) 0x1fff)

#define ADAS_MODULE_ID_MIN	((uint16_t) 0xA001)
#define ADAS_MODULE_ID_MAX	((uint16_t) 0xAFFF)

#define MODULE_CAMERA_PATTERN	((uint16_t) 0x9003)
/* Module ID define end */

/* mipi host st_int_main fault safe exception */
#define ST_INT_ECC_COR (1 << 7)

/* isp fault safe exception */
#define ISP_TEMP_DMA_DROP 3

/* Event ID define start */

/*
 * module id for other app
 */
#define MODULE_BIF_SPI			((uint16_t) 0x4021)
#define MODULE_PRG_FLOW			((uint16_t) 0x4022)
#define MODULE_PRG_ERROR		((uint16_t) 0x4023)
#define MODULE_IMAGE_TIMEOUT	((uint16_t) 0x4024)
#define MODULE_BPU_FC_TO		((uint16_t) 0x4025)
#define MODULE_SPI_COM			((uint16_t) 0x4026)
#define MODULE_ETH_COM			((uint16_t) 0x4027)
#define MODULE_UART_COM			((uint16_t) 0x4028)
#define MODULE_APP_MAX			((uint16_t) 0x4029)

#define COM_E2E_DBC_MAX		(3U)

/*
 * bif spi event id
 */
#define BIF_SPI_OUTOF_ORDER	(1U)
#define BIF_SPI_LOSS_FRAME_COUNTER	(2U)
#define BIF_SPI_CRC_ERROR	(3U)
#define BIF_SPI_TIME_OUT	(4U)

/*
 * program error event id
 */
#define PRG_SIGABRT	(1U)
#define PRG_SIGBUS	(2U)
#define PRG_SIGFPE	(3U)
#define PRG_SIGILL	(4U)
#define PRG_SIGINFO	(5U)
#define PRG_SIGIOT	(6U)
#define PRG_SIGPWR	(7U)
#define PRG_SIGSEGV	(8U)
#define PRG_SIGSYS	(9U)
#define PRG_SIGTERM	(10U)
#define PRG_SIGUNUSED	(11U)

/*
 * driver module ID
 */
#define ModuleDiagDriver	(1U)
#define ModuleDiag_i2c		(2U)
#define ModuleDiag_VIO		(3U)
#define ModuleDiag_bpu		(4U)
#define ModuleDiag_sound	(5U)
#define ModuleDiag_bif		(6U)
#define ModuleDiag_eth		(7U)
#define ModuleDiag_spi		(8U)
#define ModuleDiag_emmc		(9U)
#define ModuleDiag_norflash		(10U)
#define ModuleDiag_cpu_cal		(11U)
#define ModuleDiag_mpu		(12U)
#define ModuleDiag_uart		(13U)
#define ModuleIdMax		(14U)

/* cpu cal module event id */
#define EventIdCpuCalTestErr	(1U)

/* spi module event id */
#define EventIdSpiErr	(1U)
#define EventIdQspiErr	(2U)

/* VIO module event id */
#define EventIdVioMipiHost0Err		(1U)
#define EventIdVioMipiHost1Err		(2U)
#define EventIdVioMipiHost2Err		(3U)
#define EventIdVioMipiHost3Err		(4U)
#define EventIdVioMipiDevErr		(5U)
#define EventIdVioSifErr		(6U)
#define EventIdVioIpuErr		(7U)
#define EventIdVioIspErr		(8U)
#define EventIdVioGdc0Err		(9U)
#define EventIdVioGdc1Err		(10U)
#define EventIdVioLdcErr		(11U)
#define EventIdVioPymErr		(12U)
#define EventIdVioFrameLost		(13U)

/* i2c module event id */
#define EventIdI2cController0Err	(1U)
#define EventIdI2cController1Err	(2U)
#define EventIdI2cController2Err	(3U)
#define EventIdI2cController3Err	(4U)

/* bpu module event id */
#define EventIdBpu0Err		(1U)
#define EventIdBpu1Err		(2U)

/* emmc module event id */
#define EventIdEmmcErr		(1U)

/* mpu module event id */
#define EventIdMpuCNN0FetchErr 		(12U)
#define EventIdMpuCNN1FetchErr		(13U)
#define EventIdMpuCNN0OtherErr		(14U)
#define EventIdMpuCNN1OtherErr		(15U)
#define EventIdMpuVioM0Err		(29U)
#define EventIdMpuVpuErr		(30U)
#define EventIdMpuVioM1Err		(31U)

/* norflash module event id */
#define EventIdNorflashErr		(1U)

/* camera pattern event id */
#define CAMERA_INIT		(1U)
#define SENSOR_TEMP		(2U)
#define SENSOR_TEST_PATTERN		(3U)
#define SENSOR_SYSTEM_CHECK		(4U)
#define CAMERA_LOCK		(5U)

#define EXTERN_TC_EVENT_ID_BASE		(0U)

/* Event ID define end */

#endif /* _FUSA_DTC_H */
