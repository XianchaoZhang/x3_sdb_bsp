/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef HB_X2A_VIO_HB_VIO_BUFFER_MGR_H
#define HB_X2A_VIO_HB_VIO_BUFFER_MGR_H

#include <stdbool.h>
#include <semaphore.h>
#include "hb_utils.h"
#include "../inc/hb_vio_interface.h"
#include "list.h"
#include "ion.h"

#define HB_VIO_BUFFER_MAX (16)
#define HB_VIO_BUFFER_MAX_MP (128)	// max buffer num for multi-process


//#define DEBUG_BUF_TRACE
#define DEBUG_INFO_TRACE
//#define DUMMY_BUF
#define DEBUG_KERNEL_GROUP_BIND


#define ALIGN_UP(a, size)    ((a+size-1) & (~ (size-1)))
#define ALIGN_DOWN(a, size)  (a & (~(size-1)) )

#define ALIGN_UP2(d)   ALIGN_UP(d, 2)
#define ALIGN_UP4(d)   ALIGN_UP(d, 4)
#define ALIGN_UP16(d)  ALIGN_UP(d, 16)
#define ALIGN_UP32(d)  ALIGN_UP(d, 32)
#define ALIGN_UP64(d)  ALIGN_UP(d, 64)

#define ALIGN_DOWN2(d)    ALIGN_DOWN(d, 2)
#define ALIGN_DOWN4(d)    ALIGN_DOWN(d, 4)
#define ALIGN_DOWN16(d)   ALIGN_DOWN(d, 16)
#define ALIGN_DOWN32(d)   ALIGN_DOWN(d, 32)
#define ALIGN_DOWN64(d)   ALIGN_DOWN(d, 64)
#define SCALE3_2(a)     ((a*3)>>1)

#define PAGE_SIZE	4096
#define PAGE_SHITF	12

typedef struct list_head queue_node;

#define bufmgr_en_lock(this)\
	do {\
		pthread_mutex_lock(&this->lock);\
	} while (0)

#define bufmgr_un_lock(this)\
	do {\
		pthread_mutex_unlock(&this->lock);\
	} while (0)

typedef enum buffer_owner {
	HB_VIO_BUFFER_OTHER = 0,
	HB_VIO_BUFFER_THIS = 1,
	HB_VIO_BUFFER_OWN_INVALID = -1,
}buffer_owner_e;

typedef enum img_fmt {
	FMT_RAW = 0,
	FMT_YUV422_YUYV = 8,
	FMT_YUV422_YVYU,
	FMT_YUV422_UYVY,
	FMT_YUV422_VYUY,
	FMT_YUV420,
	FMT_MAX = 0x7fffffff,
} img_fmt_e;

//kernel set
//8bit	0
//10bit 1
//12bit 2
//14bit 3
//16bit 4

typedef enum img_pix_len {
	PIX_LEN_8 = 0,
	PIX_LEN_10 = 1,
	PIX_LEN_12 = 2,
	PIX_LEN_14 = 3,
	PIX_LEN_16 = 4,
	PIX_LEN_MAX = 0x7fffffff,
} img_pix_len_e;

typedef enum mgr_lock_state {
	MGR_NO_LOCK = 0,
	MGR_LOCK
} mgr_lock_state_e;


typedef struct buf_offset_s {
	uint16_t width;
	uint16_t height;
	uint16_t stride_size;
	uint32_t offset[3];	//y,uv  or raw/ dol2 / dol3
} buf_offset_t;

typedef enum HB_VIO_BUF_MGR_TYPE {
	HB_VIO_BUFFER_TYPE_SIF = 0,
	HB_VIO_BUFFER_TYPE_ISP,
	HB_VIO_BUFFER_TYPE_GDC,
	HB_VIO_BUFFER_TYPE_IPU,
	HB_VIO_BUFFER_TYPE_PYM,
	HB_VIO_BUFFER_TYPE_INVALID
} HB_VIO_BUF_MGR_TYPE_E;

typedef enum buffer_position {
	HB_VIO_BUFFER_POSITION_NONE = 0,
	HB_VIO_BUFFER_POSITION_IN_DRIVER,
	HB_VIO_BUFFER_POSITION_IN_HAL,
	HB_VIO_BUFFER_POSITION_IN_USER,
	HB_VIO_BUFFER_POSITION_MAX
} buffer_position_e;

typedef enum buf_cache_type {
	HB_VIO_BUFFER_ION_NONCACHED_TYPE = 0,
	HB_VIO_BUFFER_ION_CACHED_TYPE = 1,
} buf_cache_type_e;

typedef struct hb_vio_buffer_info_s {
	buffer_position_e position;
	buffer_state_e state;
} hb_vio_buffer_info_t;

typedef struct buffer_node_s {
	queue_node node;
	hb_vio_buffer_t vio_buf;
} buf_node_t;

typedef struct pym_buffer_node_s {
	queue_node node;
	pym_buffer_t pym_buf;
} pym_buf_node_t;

typedef struct buffer_mgr_s {
	VIO_DATA_TYPE_E buffer_type;	//pym buffer is special
	uint32_t pipeline_id;
	uint16_t num_buffers;
	void *buf_nodes;	//buf_node_t or pym_buf_node_t * num_buffers
	uint32_t queued_count[BUFFER_INVALID];
	queue_node buffer_queue[BUFFER_INVALID];
	sem_t sem[BUFFER_INVALID];
	pthread_mutex_t lock;
	uint32_t first_idx;
	int dev_fd;
} buffer_mgr_t;

#if 0
typedef union buf_s {
	queue_node node;
	hb_vio_buffer_t normal_buf;
	pym_buffer_t pym_buf;
} buf_u;

typedef struct buffer_mgr_s {
	HB_VIO_BUF_MGR_TYPE_E buffer_type;	//pym buffer is special
	uint32_t pipeline_id;
	uint16_t num_buffers;
	buf_u *buf_nodes;	//buf_node_t or pym_buf_node_t * num_buffers
	uint32_t queued_count[BUFFER_INVALID];
	struct list_head buffer_queue[BUFFER_INVALID];
	pthread_mutex_t lock;
} buffer_mgr_t;
#endif

int buf_enqueue(buffer_mgr_t * this, queue_node * node, buffer_state_e state,
				mgr_lock_state_e lock);
queue_node *buf_dequeue(buffer_mgr_t * this, buffer_state_e state,
				mgr_lock_state_e lock);
int trans_buffer(buffer_mgr_t * this, queue_node * node, buffer_state_e state,
				mgr_lock_state_e lock);
queue_node *peek_buffer(buffer_mgr_t * this, buffer_state_e state,
				mgr_lock_state_e lock);
queue_node *peek_buffer_tail(buffer_mgr_t * this, buffer_state_e state,
				mgr_lock_state_e lock);
queue_node *find_pop_buffer(buffer_mgr_t *this, buffer_state_e state,
                            int buf_index, mgr_lock_state_e lock);

buffer_mgr_t *buffer_manager_create(uint32_t pipeline_id,
				    VIO_DATA_TYPE_E buffer_type);
void buffer_manager_destroy(buffer_mgr_t * this);

int buffer_manager_init(buffer_mgr_t * this, uint32_t buffer_num);
int buffer_manager_deinit(buffer_mgr_t * this);

int buffer_mgr_alloc(buffer_mgr_t * this, int planeCount, uint32_t * planeSize,
		     void *img_info);
int buffer_mgr_alloc_mp(buffer_mgr_t * this, int planeCount,
		uint32_t * planeSize, void *img_info, uint32_t first_idx, int fd);

int buffer_mgr_free(buffer_mgr_t * this, _Bool need_map);
buffer_owner_e buffer_index_owner(buffer_mgr_t * this, uint32_t index);
int buffer_other_index_init(buffer_mgr_t * this, uint32_t index);


int buf_mgr_flush_AlltoAvali(buffer_mgr_t *this, mgr_lock_state_e lock);
int buf_mgr_flush_Queueto(buffer_mgr_t * this, buffer_state_e src_queue,
                          buffer_state_e dst_queue, mgr_lock_state_e lock);

void buf_mgr_print_queues(buffer_mgr_t * this, mgr_lock_state_e lock);

void print_buffer_queue(buffer_mgr_t * this, buffer_state_e state,
				mgr_lock_state_e lock);

void buf_mgr_print_qcount(buffer_mgr_t * this, mgr_lock_state_e lock);

int ion_buffer_alloc(int *fd,
		     int size,
		     char **addr,
		     uint32_t ion_heap_mask,
		     uint32_t ion_flags, uint32_t ion_align, _Bool need_map);

int ion_buffer_free(int *fd, int size, char **addr, _Bool need_map);

int ion_alloc_phy(int size, int *fd, char **vaddr, unsigned long *paddr);

#endif //HB_X2A_VIO_HB_VIO_BUFFER_MGR_H
