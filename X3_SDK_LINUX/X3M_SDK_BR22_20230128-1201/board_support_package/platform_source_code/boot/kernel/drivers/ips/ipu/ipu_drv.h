#ifndef __IPU_DRV_H__
#define __IPU_DRV_H__

#include <linux/ion.h>
#include "ipu_dev.h"
#include "ipu_common.h"

typedef enum {
	IPUC_SET_DDR = 1,
	IPUC_SET_CROP_DDR,
	IPUC_SET_SCALE_DDR,
	IPUC_SET_PYM_DDR,
	IPUC_SET_PYM_1ST_SRC_DDR,
	IPUC_SET_PYM_2ND_SRC_DDR,
	IPUC_SET_CROP,
	IPUC_SET_SCALE,
	IPUC_SET_PYMID,
	IPUC_SET_BASE,
	IPUC_SET_FRAME_ID,
} ipu_cmd_e;

enum {
	IPU_INVALID,
	IPU_ISP_SINGLE,
	IPU_DDR_SIGNLE,
	IPU_FEED_BACK,
	IPU_DDR_DUAL,
};

extern uint8_t g_slot_num;
extern struct class *ipu_class;

typedef void (*ipu_handle_t)(uint32_t status);

struct x2_ipu_data {
	/* if ipu mem from ion */
	struct ion_client *ipu_iclient;
	struct ion_handle *ipu_ihandle;
	int ion_cnt;
	/* if ipu mem from reserve mem */
	void __iomem *regbase;  /* read/write[bwl] */

	phys_addr_t paddr;
	void *vaddr;
	size_t memsize;

	/* the slot num will use to caculate use memory size */
	uint8_t slot_num;
	uint32_t slot_size;
	unsigned int vio_mode;
	struct task_struct *ipu_task;
	wait_queue_head_t wq_head;
	spinlock_t elock;
	bool stop;
	bool pymid_done;
	int8_t done_idx;
	uint32_t isr_data;
	//uint32_t err_status;
	ipu_cfg_t *cfg;
	struct resource *io_r;
	unsigned long runflags;
	unsigned int ipu_mode;
	ipu_handle_t ipu_handle[IPU_DDR_DUAL];
};

typedef struct {
	uint32_t paddr;
	uint32_t memsize;
	uint8_t slot_num;
	uint32_t slot_size;
} ipu_meminfo_t;

void ipu_tsin_reset(void);
int64_t ipu_tsin_get(int64_t sys_ts);
int8_t ipu_drv_start(void);
int8_t ipu_drv_stop(void);
int8_t ipu_cfg_ddrinfo_init(ipu_cfg_t *ipu);
int8_t ipu_set(ipu_cmd_e cmd, ipu_cfg_t *ipu_cfg, uint64_t data);
extern void ipu_handle_frame_done(void);
extern void ipu_handle_pym_frame_done(void);
extern void ipu_handle_frame_start(void);
int ipu_ion_alloc(void);
int ipu_ion_free(void);

#endif
