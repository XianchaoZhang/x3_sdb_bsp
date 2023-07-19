/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef VIO_FRAME_MGR_H
#define VIO_FRAME_MGR_H

#include <linux/kthread.h>
#include "vio_config.h"

#define VIO_MAX_PLANES 32
#define VIO_MP_MAX_FRAMES 128
#define VIO_MAX_SUB_PROCESS	8
#define VIO_BUF_PLANE_0		0
#define VIO_BUF_PLANE_1		1
#define VIO_BUF_PLANE_2		2

#define framemgr_e_barrier_irqs(this, index, flag)		\
	do {							\
		this->sindex |= index;				\
		spin_lock_irqsave(&this->slock, flag);		\
	} while (0)
#define framemgr_x_barrier_irqr(this, index, flag)		\
	do {							\
		spin_unlock_irqrestore(&this->slock, flag);	\
		this->sindex &= ~index;				\
	} while (0)
#define framemgr_e_barrier_irq(this, index)			\
	do {							\
		this->sindex |= index;				\
		spin_lock_irq(&this->slock);			\
	} while (0)
#define framemgr_x_barrier_irq(this, index)			\
	do {							\
		spin_unlock_irq(&this->slock);			\
		this->sindex &= ~index;				\
	} while (0)
#define framemgr_e_barrier(this, index)				\
	do {							\
		this->sindex |= index;				\
		spin_lock(&this->slock);			\
	} while (0)
#define framemgr_x_barrier(this, index)				\
	do {							\
		spin_unlock(&this->slock);			\
		this->sindex &= ~index;				\
	} while (0)

enum vio_frame_state {
	FS_FREE,
	FS_REQUEST,
	FS_PROCESS,
	FS_COMPLETE,
	FS_USED,
	FS_INVALID
};

enum vio_frame_index_state {
	FRAME_IND_FREE = 0,
	FRAME_IND_USING,
	FRAME_IND_STREAMOFF
};

enum vio_hw_frame_state {
	FS_HW_FREE,
	FS_HW_REQUEST,
	FS_HW_CONFIGURE,
	FS_HW_WAIT_DONE,
	FS_HW_INVALID
};

enum buffer_owner {
	VIO_BUFFER_OTHER = 0,
	VIO_BUFFER_THIS = 1,
	VIO_BUFFER_OWN_INVALID = -1,
};

enum vio_framemgr_state {
	FRAMEMGR_CREAT = 0,
	FRAMEMGR_INIT,
};


#define NR_FRAME_STATE FS_INVALID

#define TRACE_ID		(1)

enum vio_frame_mem_state {
	/* initialized memory */
	FRAME_MEM_INIT,
	/* mapped memory */
	FRAME_MEM_MAPPED
};

#define MAX_FRAME_INFO		(4)
enum vio_frame_info_index {
	INFO_FRAME_START,
	INFO_CONFIG_LOCK,
	INFO_FRAME_END_PROC
};

struct vio_frame_info {
	int			cpu;
	int			pid;
	unsigned long long	when;
};

struct frame_id {
	u32 frame_id;
	u64 timestamps;
	struct timeval tv;
};

struct special_buffer {
	u32 ds_y_addr[24];
	u32 ds_uv_addr[24];
	u32 us_y_addr[6];
	u32 us_uv_addr[6];
};

struct frame_info {
	u32 frame_id;
	u64 timestamps;
	struct timeval tv;
	int format;
	int height;
	int width;
	u32 addr[8];
	int bufferindex;
	int pixel_length;
	u32 dynamic_flag;
	struct special_buffer spec;
	int ion_share_id[3];
	int32_t inbuf_ion_share_id[3];
	u32 addr_org[8];
};

struct vio_frame {
	struct list_head	list;
	struct kthread_work work;
	struct kthread_work *mp_work;
	void 		*data;
	/* common use */
	u32			planes; /* total planes include multi-buffers */
	u32			dvaddr_buffer[VIO_MAX_PLANES];
	ulong 		kvaddr_buffer[VIO_MAX_PLANES];

	struct vio_frame_info frame_info[MAX_FRAME_INFO];
	struct frame_info frameinfo;
	u32			instance; /* device instance */
	u32			state;
	u32			fcount;
	u32			index;

	u32			poll_mask;
	u32			dispatch_mask;

	u8			hw_idx;
};

/*
 * Y_UV_NON_CONTINUOUS_ALIGN_4K:
 * IPU default ION, Y and UV are indepent-allocated
 * and their begin address all align to 4K
 *
 * Y_UV_CONTINUOUS_ALIGN:
 * Y and UV are allocated in a continous buffer,
 * and begin address is align as user set
 */
enum yuv_buf_type {
	Y_UV_NON_CONTINUOUS_ALIGN_4K,
	Y_UV_CONTINUOUS_ALIGN
};

struct mp_vio_frame {
	// must be the first
	struct vio_frame common_frame;

	// ipu/pym multprocess share
	// support max 3 planar
	// phy addr for ion reserverd in xj3 is never greater than 32bit
	int plane_count;
	u64 addr[3];
	struct ion_handle *ion_handle[3];

	// which process this vio_frame belongs to
	// the process alloc how may ion buffers and it's first indx in vio_framemgr
	// for multiprocess alloc/free tracking
	int proc_id;
	int first_indx;
	int ion_bufffer_num;

	// if Y and UV are continous allocated
	// used only in ipu driver
	enum yuv_buf_type ion_alloc_type;
};

struct vio_framemgr {
	u32			id;
	/*
	 * slock for protect elements and buf list in struct vio_framemgr
	 * TODO every list have one spinlock
	 */
	spinlock_t		slock;
	ulong			sindex;

	u32			num_frames;
	u32			max_index;
	struct vio_frame	*frames;
	struct vio_frame	*frames_mp[VIO_MP_MAX_FRAMES];
	u32			dispatch_mask[VIO_MP_MAX_FRAMES];
	u32			ctx_mask;
	enum vio_frame_index_state	index_state[VIO_MP_MAX_FRAMES];
	enum vio_framemgr_state	state;

	u32			queued_count[NR_FRAME_STATE];
	struct list_head	queued_list[NR_FRAME_STATE];
};

static const char * const hw_frame_state_name[NR_FRAME_STATE] = {
	"Free",
	"Request",
	"Configure",
	"Wait_Done"
};

static const char * const frame_state_name[NR_FRAME_STATE] = {
	"Free",
	"Request",
	"Process",
	"Complete",
	"Used"
};

/* frame info statistic */
enum USER_STATISTIC {
	USER_STATS_NORM_FRM = 0,
	USER_STATS_SEL_TMOUT ,
	USER_STATS_DROP,
	USER_STATS_DQ_FAIL,
	USER_STATS_SEL_ERR,
	/* reserver: 5-7 */
	USER_STATS_NUM = 8
};

struct user_statistic {
	uint32_t cnt[USER_STATS_NUM];
};

struct user_seq_info {
	u64 timeout;
	uint8_t seq_num[VIO_MAX_STREAM];
};

#define HB_VIO_BUFFER_MAX_PLANES 3
#define HB_VIO_BUFFER_MAX_MP (128)
typedef struct kernel_ion_one {
	int planecount;
	size_t planeSize[HB_VIO_BUFFER_MAX_PLANES];
	uint64_t paddr[HB_VIO_BUFFER_MAX_PLANES];
	int share_id[HB_VIO_BUFFER_MAX_PLANES];
}kernel_ion_one_t;

typedef struct kernel_ion {
	int buf_num;
	uint32_t flag;
	struct kernel_ion_one one[HB_VIO_BUFFER_MAX_MP];
}kernel_ion_t;


int frame_fcount(struct vio_frame *frame, void *data);
int put_frame(struct vio_framemgr *this, struct vio_frame *frame,
			enum vio_frame_state state);
struct vio_frame *get_frame(struct vio_framemgr *this,
			enum vio_frame_state state);
int trans_frame(struct vio_framemgr *this, struct vio_frame *frame,
			enum vio_frame_state state);
struct vio_frame *peek_frame(struct vio_framemgr *this,
			enum vio_frame_state state);
struct vio_frame *peek_frame_tail(struct vio_framemgr *this,
			enum vio_frame_state state);
struct vio_frame *find_frame(struct vio_framemgr *this,
			enum vio_frame_state state,
			int (*fn)(struct vio_frame *, void *), void *data);
void print_frame_queue(struct vio_framemgr *this,
			enum vio_frame_state state);

int frame_manager_open(struct vio_framemgr *this, u32 buffers);
int frame_manager_close(struct vio_framemgr *this);
int frame_manager_flush(struct vio_framemgr *this);
void frame_manager_print_queues(struct vio_framemgr *this);
void frame_manager_print_info_queues(struct vio_framemgr *this);
int frame_manager_init_mp(struct vio_framemgr *this);
int frame_manager_open_mp(struct vio_framemgr *this, u32 buffers,
	u32 *index_start);

#if 0
int frame_manager_close_mp(struct vio_framemgr *this,
	u32 index_start, u32 buffers, u32 ctx_index);
#endif

int frame_manager_flush_mp(struct vio_framemgr *this,
	u32 index_start, u32 buffers, u8 proc_id);
#if 0
int frame_manager_flush_mp_prepare(struct vio_framemgr *this,
	u32 index_start, u32 buffers, u8 proc_id);
#endif
void frame_work_init(struct kthread_work *work);

#endif
