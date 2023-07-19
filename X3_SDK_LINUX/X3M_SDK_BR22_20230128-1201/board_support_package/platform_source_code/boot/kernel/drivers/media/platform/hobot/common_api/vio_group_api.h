/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef __VIO_GROUP_API_H__
#define __VIO_GROUP_API_H__

#include <linux/kthread.h>
#include <linux/semaphore.h>
#include <linux/wait.h>

#include "vio_framemgr.h"
#include "vio_config.h"

#define MAX_SUB_DEVICE  8
#define MAX_SHADOW_NUM 4

#define GROUP_ID_SIF_OUT	0
#define GROUP_ID_SIF_IN		1
#define GROUP_ID_IPU		2
#define GROUP_ID_PYM		3
#define GROUP_ID_NUMBER		4

#define X3_IAR_INTERFACE
#define SET_CPU_AFFINITY

#define SIF_OUT_TASK_PRIORITY    40
#define SIF_DDRIN_TASK_PRIORITY  37
#define IPU_TASK_PRIORITY        36
#define PYM_TASK_PRIORITY        35
#define OSD_TASK_PRIORITY        34

#define IPU0_IDLE    BIT(12)
#define PYM_IDLE	BIT(14)

#define SIF_CAP_FS		0
#define SIF_CAP_FE		1
#define SIF_IN_FS		2
#define SIF_IN_FE		3
#define IPU_FS			4
#define IPU_US_FE		5
#define IPU_DS0_FE		6
#define IPU_DS1_FE		7
#define IPU_DS2_FE		8
#define IPU_DS3_FE		9
#define IPU_DS4_FE		10
#define PYM_FS			11
#define PYM_FE			12
#define GDC_FS			13
#define GDC_FE			14
#define ISP_FS			15
#define ISP_FE			16
#define STAT_NUM		17

#define MAX_DELAY_FRAMES 5000
#define MAX_SHOW_FRAMES 5

#define VIO_RETRY_100	100
#define VIO_RETRY_50	50

enum vio_module {
	SIF_MOD,
	ISP_MOD,
	IPU_MOD,
	PYM_MOD,
	GDC_MOD,
	VIO_MOD_NUM,
};

enum vio_module_event {
	event_none,
	event_sif_cap_fs,
	event_sif_cap_fe,
	event_sif_in_fs,
	event_sif_in_fe,
	event_isp_fs,
	event_isp_fe,
	event_ipu_fs,
	event_ipu_us_fe,
	event_ipu_ds0_fe,
	event_ipu_ds1_fe,
	event_ipu_ds2_fe,
	event_ipu_ds3_fe,
	event_ipu_ds4_fe,
	event_pym_fs,
	event_pym_fe,
	event_gdc_fs,
	event_gdc_fe,

	event_sif_cap_dq,
	event_sif_in_dq,
	event_isp_dq,
	event_ipu_dq,
	event_ipu_us_dq,
	event_ipu_ds0_dq,
	event_ipu_ds1_dq,
	event_ipu_ds2_dq,
	event_ipu_ds3_dq,
	event_ipu_ds4_dq,
	event_pym_dq,
	event_gdc_dq,

	event_sif_cap_q,
	event_sif_in_q,
	event_isq_q,
	event_ipu_q,
	event_ipu_us_q,
	event_ipu_ds0_q,
	event_ipu_ds1_q,
	event_ipu_ds2_q,
	event_ipu_ds3_q,
	event_ipu_ds4_q,
	event_pym_q,
	event_gdc_q,

	event_err,
};

enum vio_group_task_state {
	VIO_GTASK_START,
	VIO_GTASK_REQUEST_STOP,
	VIO_GTASK_SHOT,
	VIO_GTASK_SHOT_STOP,
};

enum vio_group_state {
	VIO_GROUP_OPEN,
	VIO_GROUP_INIT,
	VIO_GROUP_START,
	VIO_GROUP_FORCE_STOP,
	VIO_GROUP_OTF_INPUT,
	VIO_GROUP_OTF_OUTPUT,
	VIO_GROUP_DMA_INPUT,
	VIO_GROUP_DMA_OUTPUT,
	VIO_GROUP_LEADER,
	VIO_GROUP_IPU_DS2_DMA_OUTPUT,
};

enum vio_group_scenarios_state {
	VIO_GROUP_SIF_OFF_ISP_ON_IPU_ON_PYM,
	VIO_GROUP_SIF_OFF_ISP_ON_IPU_OFF_PYM,
};

struct vio_group_task {
	struct task_struct		*task;
	struct kthread_worker	worker;
	unsigned long				state;
	atomic_t			refcount;
	struct semaphore    hw_resource;
	u32 id;
};

/**
 * struct vio_group used to describe processing modules on a pipeline
 * include GROUP_ID_SIF_OUT/GROUP_ID_SIF_IN/GROUP_ID_IPU/GROUP_ID_PYM
 * @id: identify different GROUP.
 * @instance: identify different pipeline.
 * @sema_flag: record which groups are done on the pipeline.
 * @target_sema: wake up leadr group task sema value.
 * @leader: leader group sequentially call next group->frame_work, and finally
 * call leader->frame_work. make sure the output buf is ready before the input
 * @gtask: pointer to dev(sif/ipu/pym) gtask
 */
struct vio_group {
	spinlock_t 			slock;
	void *sub_ctx[MAX_SUB_DEVICE];
	struct frame_id frameid;
	unsigned long state;
	atomic_t rcount; /* request count */
	u32 id;
	u32 instance;
	u32 sema_flag;
	u32 target_sema;
	bool get_timestamps;
	bool leader;
	u32 output_flag;
	atomic_t node_refcount;
	struct vio_group		*next;
	struct vio_group		*prev;
	struct vio_group		*head;
	struct vio_chain		*chain;
	struct vio_group_task *gtask;
	void (*frame_work)(struct vio_group *group);

	// check ipu/pym scenario, for frameid
	int group_scenario;
	atomic_t work_insert;

	unsigned int abnormal_fs;
	u32 shadow_reuse_check; /* for check once in one group */
};

struct vio_work {
	struct kthread_work work;
	struct vio_group *group;
};

struct vio_video_ctx {
	wait_queue_head_t		done_wq;
	struct vio_framemgr 	framemgr;
	struct vio_group		*group;
	unsigned long			state;

	u32 id;
	u32 event;
	bool leader;
};

struct statinfo {
	u32 framid;
	int event;
	struct timeval g_tv;
	u32 addr;
	u8 queued_count[NR_FRAME_STATE];
};

struct vio_chain {
	struct vio_group group[GROUP_ID_NUMBER];
	struct statinfo statinfo[MAX_DELAY_FRAMES][VIO_MOD_NUM];
	unsigned long statinfoidx[VIO_MOD_NUM];
	unsigned long state;
};

struct vio_core {
       struct vio_chain chain[VIO_MAX_STREAM];
       atomic_t rsccount;
       atomic_t gdc0_rsccount;
       atomic_t gdc1_rsccount;
};
struct vio_frame_id {
	u32 frame_id;
	u64 timestamps;
	struct timeval tv;
	u32 frame_id_bits;
	spinlock_t id_lock;
};

enum osd_chn {
    OSD_IPU_US,
    OSD_IPU_DS0,
    OSD_IPU_DS1,
    OSD_IPU_DS2,
    OSD_IPU_DS3,
    OSD_IPU_DS4,
    OSD_IPU_SRC,
    OSD_PYM_OUT,
    OSD_CHN_MAX,
};

struct vio_osd_info {
	atomic_t need_sw_osd;
	// osd chn number
	uint32_t id;
	// frame count which is processing by osd
	atomic_t frame_count;

	void (*return_frame)(struct vio_osd_info *osd_info, struct vio_frame *frame);
};

typedef int (*isp_callback)(int);
typedef int (*iar_get_type_callback)(u8 *pipeline, u8 *channel);
typedef int (*iar_set_addr_callback)(uint32_t disp_layer, u32 yaddr, u32 caddr);
typedef void (*osd_send_frame_callback)(struct vio_osd_info *osd_info, struct vio_frame *frame);
typedef int (*osd_get_sta_bin_callback)(void *subdev, uint16_t (*sta_bin)[4]);

int vio_group_task_start(struct vio_group_task *group_task);
int vio_group_task_stop(struct vio_group_task *group_task);
void vio_group_start_trigger(struct vio_group *group, struct vio_frame *frame);
void vio_group_start_trigger_mp(struct vio_group *group, struct vio_frame *frame);
void vio_group_insert_work(struct vio_group *group, struct kthread_work *work);

void vio_init_ldc_access_mutex(void);
void vio_ldc_access_mutex_lock(void);
void vio_ldc_access_mutex_unlock(void);
void vio_get_ldc_rst_flag(u32 *ldc_rst_flag);
void vio_set_ldc_rst_flag(u32 ldc_rst_flag);
void vio_get_sif_exit_flag(u32 *sif_exit);
void vio_set_sif_exit_flag(u32 sif_exit);

void vio_rst_mutex_init(void);
void vio_rst_mutex_lock(void);
void vio_rst_mutex_unlock(void);
struct vio_group *vio_get_chain_group(int instance, u32 group_id);
int vio_bind_chain_groups(struct vio_group *src_group, struct vio_group *dts_group);
int vio_init_chain(int instance);
void vio_bind_group_done(int instance);
void vio_get_frame_id(struct vio_group *group);
void vio_get_sif_frame_info(u32 instance, struct vio_frame_id *frame_info);
void vio_get_ipu_frame_info(u32 instance, struct vio_frame_id *frame_info);
void vio_get_sif_frame_id(struct vio_group *group);
void vio_get_ipu_frame_id(struct vio_group *group);
int vio_check_all_online_state(struct vio_group *group);
int vio_group_init_mp(u32 group_id);
void vio_reset_module(u32 module);
void vio_group_done(struct vio_group *group);
void vio_dwe_clk_enable(void);
void vio_dwe_clk_disable(void);
void vio_gdc_clk_enable(u32 hw_id);
void vio_gdc_clk_disable(u32 hw_id);
void vio_set_stat_info(u32 instance, u32 stat_type, u32 event, u32 frameid,
	u32 addr, u32 *queued_count);
void vio_print_stat_info(u32 instance);
int vio_print_delay(s32 instance, u8* buf, u32 size);
void vio_print_stack_by_name(char *name);
void vio_clear_stat_info(u32 instance);
void* vio_get_stat_info_ptr(u32 instance);
void voi_set_stat_info_update(s32 update);

extern int sif_get_frame_id(u32 instance, u32 *frame_id);
extern int sif_set_frame_id(u32 instance, u32 frame_id);
extern int sif_set_frame_id_nr(u32 instance, u32 nr);

extern iar_get_type_callback iar_get_type;
extern iar_set_addr_callback iar_set_addr;
extern isp_callback sif_isp_ctx_sync;
extern osd_send_frame_callback osd_send_frame;
extern osd_get_sta_bin_callback osd_get_sta_bin;

extern int isp_status_check(void);
extern int ldc_status_check(void);

extern void ips_set_module_reset(unsigned long module);
extern int ips_set_clk_ctrl(unsigned long module, bool enable);
extern int ips_set_bus_ctrl(unsigned int cfg);
extern int ips_get_bus_ctrl(void);

extern void ips_set_md_enable(void);
extern void ips_set_md_disable(void);

extern int ips_get_bus_status(void);
extern int ips_set_md_cfg(sif_output_md_t *cfg);
extern int ips_disable_md(void);
extern int ips_set_md_refresh(bool enable);
extern int ips_set_md_resolution(u32 width, u32 height);
extern int ips_get_md_event(void);
extern int ips_set_md_fmt(u32 fmt);
extern void ips_set_iram_size(u32 iram_size);

extern int vio_clk_enable(const char *name);
extern int vio_clk_disable(const char *name);
extern int vio_set_clk_rate(const char *name, ulong frequency);
extern ulong vio_get_clk_rate(const char *name);
extern int ion_check_in_heap_carveout(phys_addr_t start, size_t size);
extern void ion_dcache_invalid(phys_addr_t paddr, size_t size);
extern void ion_dcache_flush(phys_addr_t paddr, size_t size);

extern void vio_irq_affinity_set(int irq, enum MOD_ID id, int suspend,
		int input_online);

extern struct class *vps_class;
extern ulong sif_mclk_freq;
extern void ips_sif_mclk_set(u32 sif_mclk);

#endif
