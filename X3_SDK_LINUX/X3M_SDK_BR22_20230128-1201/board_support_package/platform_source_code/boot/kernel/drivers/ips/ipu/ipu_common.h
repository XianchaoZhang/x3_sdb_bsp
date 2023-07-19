#ifndef __IPU_COMMON_H__
#define __IPU_COMMON_H__

#define IPU_LOG_DEBUG	(1)
#define IPU_LOG_INFO	(2)
extern unsigned int ipu_debug_level;
#define ipu_info(fmt, args...)	\
	do {									\
		if((ipu_debug_level >= IPU_LOG_INFO))	\
			printk(KERN_INFO "[ipu][info]: "fmt, ##args);		\
	} while(0)

#define ipu_dbg(fmt, args...)	\
	do {									\
		if((ipu_debug_level >= IPU_LOG_DEBUG)) \
			printk(KERN_INFO "[ipu][debug]: "fmt, ##args);		\
	} while(0)

#define ipu_err(fmt, args...)       printk(KERN_ERR "[ipu][error]: "fmt, ##args)

#define IPU_SLOT_MAX_SIZE			0x1000000
#define IPU_SLOT_MIN_SIZE			0x600000

#define IPU_MAX_SLOT				64
#define IPU_DEF_SLOT				8
#define IPU_MIN_SLOT				3
#define IPU_SLOT_SIZE				(g_ipu->slot_size ? \
									 g_ipu->slot_size : \
									 IPU_SLOT_MAX_SIZE)

#define IPU_MAX_SLOT_DUAL			(IPU_MAX_SLOT / 2)
#define IPU_SLOT_DAUL_SIZE			(IPU_SLOT_SIZE *2)

#define ALIGN_4(d)          (((d) + 3) & ~0x3)
#define ALIGN_16(d)         (((d) + 15) & ~0xf)
#define ALIGN_64(d)         (((d) + 63) & ~0x3f)

#define IPU_SLOT_NOT_AVALIABLE	0
#define IPU_TRIGGER_ISR	1
#define IPU_RCV_NOT_AVALIABLE	2
#define IPU_PYM_NOT_AVALIABLE	3
#define IPU_PYM_STARTUP		4
#define IPU_RCV_CFG_UPDATE	5
#define IPU_PYM_CFG_UPDATE	6

#endif
