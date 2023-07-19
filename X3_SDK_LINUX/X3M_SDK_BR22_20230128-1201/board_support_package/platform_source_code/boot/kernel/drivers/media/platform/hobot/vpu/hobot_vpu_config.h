#ifndef __HOBOT_VPU_CONFIG_H_
/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/
#define __HOBOT_VPU_CONFIG_H_

/* definitions to be changed as customer  configuration */
/* if you want to have clock gating scheme frame by frame */
/* #define VPU_SUPPORT_CLOCK_CONTROL */

/* if the driver want to use interrupt service from kernel ISR */
#define VPU_SUPPORT_ISR

#define USE_SHARE_SEM_BT_KERNEL_AND_USER
#define VPU_LOCK_BUSY_CHECK_TIMEOUT          120000
//#define USE_MUTEX_IN_KERNEL_SPACE
#define USE_VPU_CLOSE_INSTANCE_ONCE_ABNORMAL_RELEASE

#ifdef VPU_SUPPORT_ISR
/* if the driver want to disable and enable IRQ whenever interrupt asserted. */
#define VPU_IRQ_CONTROL
#endif

#ifdef VPU_SUPPORT_ISR
#define VPU_IRQ_NUM 			(66+32)
#endif

#define SUPPORT_TIMEOUT_RESOLUTION

#define WAVE420_CODE                    0x4200	// Wave420
#define WAVE420L_CODE                   0x4201	// Wave420L
#define WAVE521_CODE                    0x5210
#define WAVE521C_CODE                   0x521c
#define WAVE521C_DUAL_CODE              0x521d	// wave521 dual core

#define MAX_NUM_VPU_INSTANCE            32	// 4 -> 32
#define MAX_VPU_INSTANCE_IDX            0xFFFFFFFF
#define MAX_INTERRUPT_QUEUE             (16*MAX_NUM_VPU_INSTANCE)
#define MAX_NUM_VPU_CORE                1
#define MAX_INST_HANDLE_SIZE            48	/* DO NOT CHANGE THIS VALUE */
#define PRODUCT_CODE_W_SERIES(x)	(x == WAVE521_CODE || x == WAVE521C_CODE \
									|| x == WAVE521C_DUAL_CODE)
#define SIZE_COMMON 				(2*1024*1024)

#endif /* HOBOT_VPU_CONFIG_H_ */
