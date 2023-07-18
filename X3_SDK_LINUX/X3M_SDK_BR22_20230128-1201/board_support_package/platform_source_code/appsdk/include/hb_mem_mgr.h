/*************************************************************************
 *                     COPYRIGHT NOTICE
 *            Copyright 2021 Horizon Robotics, Inc.
 *                   All rights reserved.
 *************************************************************************/
#ifndef HB_MEM_MGR_H
#define HB_MEM_MGR_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define MAX_GRAPHIC_BUF_COMP 3

/**
* Define the buffer format of graphic buffer.
**/
typedef enum mem_pixel_format_t {
	MEM_PIX_FMT_NONE = -1,

	MEM_PIX_FMT_RGB565,		//< packed RGB 5:6:5, 16bpp,
	MEM_PIX_FMT_RGB24,		//< packed RGB 8:8:8, 24bpp, RGBRGB...
	MEM_PIX_FMT_BGR24,		//< packed RGB 8:8:8, 24bpp, BGRBGR...
	MEM_PIX_FMT_ARGB,		//< packed ARGB 8:8:8:8, 32bpp, ARGBARGB...
	MEM_PIX_FMT_RGBA,		//< packed RGBA 8:8:8:8, 32bpp, RGBARGBA...
	MEM_PIX_FMT_ABGR,		//< packed ABGR 8:8:8:8, 32bpp, ABGRABGR...
	MEM_PIX_FMT_BGRA,		//< packed BGRA 8:8:8:8, 32bpp, BGRABGRA...
	/* planar YUV 4:2:0, 12bpp, (1 Cr & Cb sample per 2x2 Y samples) */
	MEM_PIX_FMT_YUV420P,
	/* 
	 * planar YUV 4:2:0, 12bpp, 1 plane for Y and 1 plane for the UV components,
	 * which are interleaved (first byte U and the following byte V).
	 */
	MEM_PIX_FMT_NV12,
	/* as above, but U and V bytes are swapped */
	MEM_PIX_FMT_NV21,
	/* planar YUV 4:2:2, 16bpp, (1 Cr & Cb sample per 2x1 Y samples) */
	MEM_PIX_FMT_YUV422P,
	/*
	 * interleaved chroma (first byte U and the following byte V)
	 * YUV 4:2:2, 16bpp, (1 Cr & Cb sample per 2x1 Y samples)
	 */
	MEM_PIX_FMT_NV16,
	/*
	 * interleaved chroma (first byte V and the following byte U)
	 * YUV 4:2:2, 16bpp, (1 Cr & Cb sample per 2x1 Y samples)
	 */
	MEM_PIX_FMT_NV61,
	/* packed YUV 4:2:2, 16bpp, Y0 Cb Y1 Cr */
	MEM_PIX_FMT_YUYV422,
	/* packed YUV 4:2:2, 16bpp, Y0 Cr Y1 Cb */
	MEM_PIX_FMT_YVYU422,
	/* packed YUV 4:2:2, 16bpp, Cb Y0 Cr Y1 */
	MEM_PIX_FMT_UYVY422,
	/* packed YUV 4:2:2, 16bpp, Cr Y0 Cb Y1 */
	MEM_PIX_FMT_VYUY422,
	/* packed YUV 4:4:4, 24bpp, (1 Cr & Cb sample per 1x1 Y samples) */
	MEM_PIX_FMT_YUV444,
	/* planar YUV 4:4:4, 24bpp, (1 Cr & Cb sample per 1x1 Y samples) */
	MEM_PIX_FMT_YUV444P,
	/*
	 * interleaved chroma (first byte U and the following byte V)
	 * YUV 4:4:4, 24bpp, (1 Cr & Cb sample per 1x1 Y samples)
	 */
	MEM_PIX_FMT_NV24,
	/*
	 * interleaved chroma (first byte V and the following byte U)
	 * YUV 4:4:4, 24bpp, (1 Cr & Cb sample per 1x1 Y samples)
	 */
	MEM_PIX_FMT_NV42,
	/* planar YUV 4:4:0 (1 Cr & Cb sample per 1x2 Y samples) */
	MEM_PIX_FMT_YUV440P,
	/* Gray Y, YUV 4:0:0 */
	MEM_PIX_FMT_YUV400,

	MEM_PIX_FMT_TOTAL,
} mem_pixel_format_t;

/**
* Define the usage of buffer. You can combine different usages as a final flags.
* Undefined usage will be ignored.
**/
typedef enum mem_usage_t {
	/* buffer is never read by cpu */
	HB_MEM_USAGE_CPU_READ_NEVER          = 0x00000000LL,
	/* buffer is often read by cpu */
	HB_MEM_USAGE_CPU_READ_OFTEN          = 0x00000001LL,
	/* mask for the software read and write flag */
	HB_MEM_USAGE_CPU_READ_MASK           = 0x0000000FLL,

	/* buffer is never write by cpu */
	HB_MEM_USAGE_CPU_WRITE_NEVER         = 0x00000000LL,
	/* buffer is often write by cpu, on arm platform *WRITE implies *READ*/
	HB_MEM_USAGE_CPU_WRITE_OFTEN         = 0x00000010LL,
	/* mask for the software read and write flag */
	HB_MEM_USAGE_CPU_WRITE_MASK          = 0x000000F0LL,

	/* buffer will be used as camera interface module buffer */
	HB_MEM_USAGE_HW_CIM                  = 0x00000100LL,
	/* buffer will be used as pyramid buffer */
	HB_MEM_USAGE_HW_PYRAMID              = 0x00000200ULL,
	/* buffer will be used as geometric distortion correction buffer */
	HB_MEM_USAGE_HW_GDC                  = 0x00000400LL,
	/* buffer will be used as stitch buffer */
	HB_MEM_USAGE_HW_STITCH               = 0x00000800LL,
	/* buffer will be used as optical flow buffer */
	HB_MEM_USAGE_HW_OPTICAL_FLOW         = 0x00001000ULL,
	/* buffer will be used as bpu buffer */
	HB_MEM_USAGE_HW_BPU                  = 0x00002000LL,
	/* buffer will be used as isp buffer */
	HB_MEM_USAGE_HW_ISP                  = 0x00004000LL,
	/* buffer will be used as display buffer */
	HB_MEM_USAGE_HW_DISPLAY              = 0x00008000LL,
	/* buffer will be used as video codec buffer */
	HB_MEM_USAGE_HW_VIDEO_CODEC          = 0x00010000LL,
	/* buffer will be used as jpeg codec buffer */
	HB_MEM_USAGE_HW_JPEG_CODEC           = 0x00020000LL,
	/* buffer will be used as vdsp buffer */
	HB_MEM_USAGE_HW_VDSP                 = 0x00040000LL,
	/* mask for the hw flag */
	HB_MEM_USAGE_HW_MASK                 = 0x00FFFF00LL,

	/* buffer will be initialized */
	HB_MEM_USAGE_MAP_INITIALIZED         = 0x01000000LL,
	/* buffer will not be initialized */
	HB_MEM_USAGE_MAP_UNINITIALIZED       = 0x02000000LL,
	/* buffer will not be initialized, unchangable */
	HB_MEM_USAGE_CACHED                  = 0x04000000LL,
	/* graphic buffer will be contiguous */
	HB_MEM_USAGE_GRAPHIC_CONTIGUOUS_BUF  = 0x08000000LL,
	/* It only indicates the buffer is a memory pool. Don't use this flag
	  to allocate buffer. It's useless.*/
	HB_MEM_USAGE_MEM_POOL				 = 0x10000000LL,
	/* mask for the trivial flag.
	  Here you can combine different usages as a final flags,
	  But the *MAP_INITIALIZED and *MAP_UNINITIALIZED are mutually exclusive,
	  *MAP_INITIALIZED has high priority. If you don't choose the tow flag,
	  DMA heap will defaultly be initialized, RESERVED and SRAM heap will
	  be uninitialized.*/
	HB_MEM_USAGE_TRIVIAL_MASK            = 0xFF000000LL,

	/* implementation-specific private usage flags */
	/* buffer will be allocated from CMA heap in linux system */
	HB_MEM_USAGE_PRIV_HEAP_DMA           = 0x000000000LL,
	/* buffer will be allocated from Carveout heap in linux system */
	HB_MEM_USAGE_PRIV_HEAP_RESERVERD     = 0x100000000LL, // deprecated
	HB_MEM_USAGE_PRIV_HEAP_RESERVED      = 0x100000000LL,
	/* buffer will be allocated from SRAM heap in linux system */
	HB_MEM_USAGE_PRIV_HEAP_SRAM          = 0x200000000LL,
	/* buffer will be allocated from another Carveout heap in linux system,
	  this heap can isolate bandwidth access of different hardwares in order to
	  decrease bandwidth preemption with other heaps.*/
	HB_MEM_USAGE_PRIV_HEAP_2_RESERVERD   = 0x400000000LL, // deprecated
	HB_MEM_USAGE_PRIV_HEAP_2_RESERVED    = 0x400000000LL,
	/* mask for the private flag */
	HB_MEM_USAGE_PRIV_MASK               = 0xF00000000LL,
} mem_usage_t;

/**
* Define the descriptor of common buffer.
**/
typedef struct hb_mem_common_buf_t {
/**
 * File descriptors of the buffer.
 */
	int32_t fd;
/**
 * Share id of the buffer.
 */
	int32_t share_id;
/**
 * Buffer flags for allocation.
 * @mem_usage_t
 *
 * - Default: 0
 */
	int64_t flags;
/**
 * Total Buffer size specified by user during allocation.
 */
	uint64_t size;
/**
 * Buffer starting virtual address.
 */
	uint8_t *virt_addr;
/**
 * Buffer starting physical address.
 * - Note: It's not recommended to use or transfer the physical address directly.
 *         There is no memory object to keep track of the physical address.
 */
	uint64_t phys_addr;
/**
 * Buffer offset.
 *
 * - Default: 0
 */
	uint64_t offset;
} hb_mem_common_buf_t;

/**
* Define the descriptor of graphic buffer.
**/
typedef struct hb_mem_graphic_buf_t {
/**
 * File descriptors of the buffer for each component.
 */
	int32_t fd[MAX_GRAPHIC_BUF_COMP];
/**
 * Plane count of this graphic buffer
 * Values [1, MAX_GRAPHIC_BUF_COMP]
 */
	int32_t plane_cnt;
/**
 * Buffer format for allocation.
 * @mem_pixel_format_t
 */
	int32_t format;
/**
 * Buffer width.
 */
	int32_t width;
/**
 * Buffer height.
 */
	int32_t height;
/**
 * Buffer horizontal stride.
 */
	int32_t stride;
/**
 * Buffer vertical stride.
 */
	int32_t vstride;
/**
 * Buffer physical memory is contiguous.
 * @mem_usage_t
 *
 * - Default: 0
 */
	int32_t is_contig;
/**
 * Share id of the buffer.
 */
	int32_t share_id[MAX_GRAPHIC_BUF_COMP];
/**
 * Buffer flags for allocation.
 * @mem_usage_t
 *
 * - Default: 0
 */
	int64_t flags;
/**
 * Total Buffer size for each component.
 */
	uint64_t size[MAX_GRAPHIC_BUF_COMP];
/**
 * Buffer virtual address for each component.
 */
	uint8_t *virt_addr[MAX_GRAPHIC_BUF_COMP];
/**
 * Buffer physical address for each component.
 * - Note: It's not recommended to use or transfer the physical address directly.
 *         There is no memory object to keep track of the physical address.
 */
	uint64_t phys_addr[MAX_GRAPHIC_BUF_COMP];
/**
 * Buffer offset.
 *
 * - Default: 0
 */
	uint64_t offset[MAX_GRAPHIC_BUF_COMP];
} hb_mem_graphic_buf_t;

/**
 * Define the descriptor of buffer queue.
 **/
typedef struct hb_mem_buf_queue_t {
/**
 * Uinque id specified by memory manager. Should no be modified.
 */
	uint64_t unique_id;
/**
 * Total items of the buffer queue.
 */
	uint32_t count;
/**
 * Size of each item.
 */
	uint32_t item_size;
} hb_mem_buf_queue_t;

/**
* Define the descriptor of memory pool.
**/
typedef struct hb_mem_pool_t {
/**
 * Buffer flags for allocation.
 * @mem_usage_t
 *
 * - Default: 0
 */
	int64_t flags;
/**
 * Total Buffer size specified by user during allocation.
 */
	uint64_t size;
/**
 * File descriptors of the pool.
 */
	int32_t fd;
/**
 * Page size in byte.
 */
	int32_t page_size;
/**
 * Total page count.
 */
	int32_t total_page_cnt;

/**
 * Available page count.
 */
	int32_t avail_page_cnt;
/**
 * Current pool client count.
 */
	int32_t cur_client_cnt;

	int32_t reserved;
} hb_mem_pool_t;

/**
* Get the memory module version.
*
* @return =0 on success, negative HB_MEM_ERROR in case of failure
*/
int32_t hb_mem_get_version(uint32_t *major, uint32_t *minor, uint32_t *patch);

/**
* Open the memory module.
* The memory module should be keep opened if you want to allocat or share memory.
*
* @return =0 on success, negative HB_MEM_ERROR in case of failure
*/
int32_t hb_mem_module_open();

/**
* Close the memory module.
* The memory module should be closed if you won't do any buffer operaion.
*
* @return =0 on success, negative HB_MEM_ERROR in case of failure
*/
int32_t hb_mem_module_close();

/**
* Allocate common buffer.
*
* @param[in]       size
* @param[in]       flags @see mem_usage_t
* @param[out]      buf, @see hb_mem_common_buf_t
*
* @return ==0 on success, negative HB_MEM_ERROR in case of failure
* @see mem_usage_t
* @see hb_mem_common_buf_t
*/
int32_t hb_mem_alloc_com_buf(uint64_t size, int64_t flags,
			hb_mem_common_buf_t * buf);

/**
* Get common buffer information.
*
* @param[in]       fd, file descriptor of the buffer
* @param[out]      buf, @see hb_mem_common_buf_t
*
* @return ==0 on success, negative HB_MEM_ERROR in case of failure
* @see hb_mem_common_buf_t
*/
int32_t hb_mem_get_com_buf(int32_t fd, hb_mem_common_buf_t *buf);

/**
* Allocate graphic buffer.
*
* @param[in]       w
* @param[in]       h
* @param[in]       format
* @param[in]       flags, @see mem_usage_t
* @param[in]       stride, horizontal stride
* @param[in]       vstride, vertical stride
* @param[out]      buf, @see hb_mem_graphic_buf_t
*
* @return ==0 on success, negative HB_MEM_ERROR in case of failure
* @see mem_usage_t
* @see hb_mem_graphic_buf_t
*/
int32_t hb_mem_alloc_graph_buf(int32_t w, int32_t h, int32_t format, int64_t flags,
			int32_t stride, int32_t vstride, hb_mem_graphic_buf_t * buf);

/**
* Get graphic buffer information.
*
* @param[in]       fd, file descriptor of the buffer
* @param[out]      buf, @see hb_mem_graphic_buf_t
*
* @return ==0 on success, negative HB_MEM_ERROR in case of failure
* @see hb_mem_graphic_buf_t
*/
int32_t hb_mem_get_graph_buf(int32_t fd, hb_mem_graphic_buf_t *buf);

/**
* Free buffer.
* The fd of common buffer or any fd of graphic buffer shouled be specified.
* Memory module will free all the related buffer of common or graphic buffer.
*
* @param[in]       fd, file descriptor of the buffer
*
* @return ==0 on success, negative HB_MEM_ERROR in case of failure
*/
int32_t hb_mem_free_buf(int32_t fd);

/**
* Invalidate cache lines associated with this buffer.
* When the flags HB_MEM_USAGE_CPU_READ_OFTEN or HB_MEM_USAGE_CPU_WRITE_OFTEN is specified,
* the buffer is cached. So invalidate-operaion shouled be done before read buffer operaion.
*
* @param[in]       fd, file descriptor of the buffer
* @param[in]       offset, buffer offset
* @param[in]       size, buffer size
*
* @return ==0 on success, negative HB_MEM_ERROR in case of failure
*/
int32_t hb_mem_invalidate_buf(int32_t fd, uint64_t offset, uint64_t size);

/**
* Flush cache lines associated with this buffer.
* When the flags HB_MEM_USAGE_CPU_READ_OFTEN or HB_MEM_USAGE_CPU_WRITE_OFTEN is specified,
* the buffer is cached. So flush-operaion shouled be done after write buffer operaion.
*
* @param[in]       fd, file descriptor of the buffer
* @param[in]       offset, buffer offset
* @param[in]       size, buffer size
*
* @return ==0 on success, negative HB_MEM_ERROR in case of failure
*/
int32_t hb_mem_flush_buf(int32_t fd, uint64_t offset, uint64_t size);

/**
* Check whether the buffer associated with the virtual address is allocatd from memory manager.
*
* @param[in]       virt_addr, virtual address
* @param[out]      valid, 1: buffer is valid, 0: buffer is invalid
*
* @return ==0 on success, negative HB_MEM_ERROR in case of failure
*/
int32_t hb_mem_is_valid_buf(uint64_t virt_addr, uint64_t size, int32_t *valid);

/**
* Get the buffer physical address associated with the virtual address.
*
* @param[in]       virt_addr, virtual address
* @param[out]      phys_addr, physical addresss
*
* @return ==0 on success, negative HB_MEM_ERROR in case of failure
*/
int32_t hb_mem_get_phys_addr(uint64_t virt_addr, uint64_t * phys_addr);

/**
* Get the buffer information associated with the virtual address.
*
* @param[in]       virt_addr, virtual address
* @param[out]      start, starting virtual addresss
* @param[out]      size, total buffer size
* @param[out]      flags, @mem_usage_t
*
* @return ==0 on success, negative HB_MEM_ERROR in case of failure
*/
int32_t hb_mem_get_buf_info(uint64_t virt_addr, uint64_t *start, uint64_t *size, int64_t *flags);

/**
* Invalidate cache lines associated with the virtual address.
* When the flags HB_MEM_USAGE_CPU_READ_OFTEN or HB_MEM_USAGE_CPU_WRITE_OFTEN is specified,
* the buffer is cached. So invalidate-operaion shouled be done before read buffer operaion.
*
* @param[in]       virt_addr, virtual address
* @param[in]       size, buffer size
*
* @return ==0 on success, negative HB_MEM_ERROR in case of failure
*/
int32_t hb_mem_invalidate_buf_with_vaddr(uint64_t virt_addr, uint64_t size);

/**
* Flush cache lines associated with the virtual address.
* When the flags HB_MEM_USAGE_CPU_READ_OFTEN or HB_MEM_USAGE_CPU_WRITE_OFTEN is specified,
* the buffer is cached. So flush-operaion shouled be done after write buffer operaion.
*
* @param[in]       virt_addr, virtual address
* @param[in]       size, buffer size
*
* @return ==0 on success, negative HB_MEM_ERROR in case of failure
*/
int32_t hb_mem_flush_buf_with_vaddr(uint64_t virt_addr, uint64_t size);

/**
* Get the common buffer information associated with the virtual address.
*
* @param[in]       virt_addr, virtual address
* @param[out]      buf, @see hb_mem_common_buf_t
*
* @return ==0 on success, negative HB_MEM_ERROR in case of failure
* @see hb_mem_common_buf_t
*/
int32_t hb_mem_get_com_buf_with_vaddr(uint64_t virt_addr,  hb_mem_common_buf_t *buf);

/**
* Get the graphic buffer information associated with the virtual address.
*
* @param[in]       virt_addr, virtual address
* @param[out]      buf, @see hb_mem_graphic_buf_t
*
* @return ==0 on success, negative HB_MEM_ERROR in case of failure
* @see hb_mem_graphic_buf_t
*/
int32_t hb_mem_get_graph_buf_with_vaddr(uint64_t virt_addr,  hb_mem_graphic_buf_t *buf);

/**
* Free buffer with virtual address.
* The vaddr of common buffer or any vaddr of graphic buffer shouled be specified.
* Memory module will free all the related buffer of common or graphic buffer.
*
* @param[in]       virt_addr, virtual address of the buffer
*
* @return ==0 on success, negative HB_MEM_ERROR in case of failure
*/
int32_t hb_mem_free_buf_with_vaddr(uint64_t virt_addr);

/**
* Import common buffer.
*
* @param[in]       buf, input common buffer, @hb_mem_common_buf_t
* @param[out]      out_buf, output common buffer, @hb_mem_common_buf_t
*
* @return ==0 on success, negative HB_MEM_ERROR in case of failure
*/
int32_t hb_mem_import_com_buf(hb_mem_common_buf_t * buf, hb_mem_common_buf_t * out_buf);

/**
* Import graphic buffer.
*
* @param[in]       buf, input graphic buffer, @hb_mem_graphic_buf_t
* @param[out]      out_buf, output graphic buffer, @hb_mem_graphic_buf_t
*
* @return ==0 on success, negative HB_MEM_ERROR in case of failure
*/
int32_t hb_mem_import_graph_buf(hb_mem_graphic_buf_t * buf, hb_mem_graphic_buf_t * out_buf);

/**
* Get the current share client count associated with the file description.
* The share client count will increase after hb_mem_import_* called.
*
* @param[in]       fd, file descriptor of the buffer
* @param[out]      share_client_cnt, share client count
*
* @return ==0 on success, negative HB_MEM_ERROR in case of failure
*/
int32_t hb_mem_get_share_info(int32_t fd, int32_t * share_client_cnt);

/**
* Get the current share client count associated with the virtual address.
* The share client count will increase after hb_mem_import_* called.
*
* @param[in]       virt_addr, virtual address
* @param[out]      share_client_cnt, share client count
*
* @return ==0 on success, negative HB_MEM_ERROR in case of failure
*/
int32_t hb_mem_get_share_info_with_vaddr(uint64_t virt_addr, int32_t * share_client_cnt);

/**
* Sleep until the share client count of the fd related buffer
* less than or equal to the input share_client_cnt.
*
* @param[in]       virt_addr, virtual address
* @param[in]       share_client_cnt, target share client count
* @param[in]       timeout in ms
*
* @return ==0 on success, negative HB_MEM_ERROR in case of failure
*/
int32_t hb_mem_wait_share_status(int32_t fd, int32_t share_client_cnt, int64_t timeout);

/**
* Sleep until the share client count of the virtual address related buffer
* less than or equal to the input share_client_cnt.
*
* @param[in]       virt_addr, virtual address
* @param[in]       share_client_cnt, share client count
* @param[in]       timeout in ms
*
* @return ==0 on success, negative HB_MEM_ERROR in case of failure
*/
int32_t hb_mem_wait_share_status_with_vaddr(uint64_t virt_addr, int32_t share_client_cnt, int64_t timeout);

/**
* Create buffer queue.
* The unique id of the queue will be specified if the queue is created successfully.
* And user should not modify it.
*
* @param[in]       queue, queue inforamtion
*
* @return ==0 on success, negative HB_MEM_ERROR in case of failure
* @see hb_mem_buf_queue_t
*/
int32_t hb_mem_create_buf_queue(hb_mem_buf_queue_t *queue);

/**
* Destroy buffer queue.
*
* @param[in]       queue, queue inforamtion
*
* @return ==0 on success, negative HB_MEM_ERROR in case of failure
* @see hb_mem_buf_queue_t
*/
int32_t hb_mem_destroy_buf_queue(hb_mem_buf_queue_t *queue);

/**
* Dequeue buffer slot from the buffer queue for producer.
*
* @param[in]       queue, queue inforamtion
* @param[out]      slot, available slot index
* @param[out]      buf, the buffer information, its size is queue item_size
* @param[in]       timeout in ms
*
* @return ==0 on success, negative HB_MEM_ERROR in case of failure
* @see hb_mem_buf_queue_t
*/
int32_t hb_mem_dequeue_buf(hb_mem_buf_queue_t * queue, int32_t *slot,
			void * buf, int64_t timeout);

/**
* Queue buffer slot into the buffer queue for producer.
*
* @param[in]       queue, queue inforamtion
* @param[in]       slot, slot index
* @param[in]       buf, the buffer information, its size is queue item_size
*
* @return ==0 on success, negative HB_MEM_ERROR in case of failure
* @see hb_mem_buf_queue_t
*/
int32_t hb_mem_queue_buf(hb_mem_buf_queue_t * queue, int32_t slot, const void * buf);

/**
* Request buffer slot from the buffer queue for consumer.
*
* @param[in]       queue, queue inforamtion
* @param[out]      slot, available slot index
* @param[out]      buf, the buffer information, its size is queue item_size
* @param[in]       timeout in ms
*
* @return ==0 on success, negative HB_MEM_ERROR in case of failure
* @see hb_mem_buf_queue_t
*/
int32_t hb_mem_request_buf(hb_mem_buf_queue_t * queue, int32_t *slot,
			void * buf, int64_t timeout);

/**
* Release buffer slot to the buffer queue for consumer.
*
* @param[in]       queue, queue inforamtion
* @param[in]       slot, slot index
*
* @return ==0 on success, negative HB_MEM_ERROR in case of failure
* @see hb_mem_buf_queue_t
*/
int32_t hb_mem_release_buf(hb_mem_buf_queue_t * queue, int32_t slot);

/**
* Cancel buffer slot to the buffer queue for producer or consumer.
* Producer can cancel the dequeued buffer.
* Consumer can cancle the requested buffer.
*
* @param[in]       queue, queue inforamtion
* @param[in]       slot, slot index
*
* @return ==0 on success, negative HB_MEM_ERROR in case of failure
* @see hb_mem_buf_queue_t
*/
int32_t hb_mem_cancel_buf(hb_mem_buf_queue_t * queue, int32_t slot);

/**
* Create memory pool.
* User can allocate momory from the pool more quickly.
*
* @param[in]       size, pool memory size
* @param[in]       flags, @see mem_usage_t
* @param[out]      pool, @see hb_mem_pool_t
*
* @return ==0 on success, negative HB_MEM_ERROR in case of failure
* @see mem_usage_t
* @see hb_mem_pool_t
*/
int32_t hb_mem_pool_create(uint64_t size, int64_t flags, hb_mem_pool_t * pool);

/**
* Destroy memory pool.
* User can use fd which is returned after creating to destory the memory pool.
* If buffers allocated from pool are not freed, the memory pool can't be destroyed
* and HB_MEM_ERR_POOL_BUSY is returned.
* User also can use hb_mem_free_buf to destroy the memory pool, but it's not recommeded.
*
* @param[in]       fd, file descriptor of the pool, @hb_mem_pool_t
*
* @return ==0 on success, negative HB_MEM_ERROR in case of failure
* @see hb_mem_pool_t
*/
int32_t hb_mem_pool_destroy(int32_t fd);

/**
* Allocate buffer from the memory pool.
* User can use fd which is returned after creating to destory the memory pool.
* If buffers allocated from pool are not freed, the memory pool can't be destroyed
* and HB_MEM_ERR_POOL_BUSY is returned.
* User also can use hb_mem_free_buf to destroy the memory pool, but it's not recommeded.
*
* @param[in]       fd, file descriptor of the pool, @hb_mem_pool_t
* @param[in]       size, pool memory size
* @param[out]      buf, @see hb_mem_common_buf_t
*
* @return ==0 on success, negative HB_MEM_ERROR in case of failure
* @see hb_mem_pool_t
* @see hb_mem_common_buf_t
*/
int32_t hb_mem_pool_alloc_buf(int32_t fd, uint64_t size, hb_mem_common_buf_t * buf);

/**
* Free memory pool buffer.
* User can use virtal address which is returned after pool buffer allocating to free memory pool buffer.
* If buffers is not allocated from pool, HB_MEM_ERR_INVALID_VADDR is returned.
* User also can use hb_mem_free_buf_with_vaddr to free the memory pool buffer, but it's not recommeded.
*
* @param[in]       virt_addr, virtal address, @hb_mem_common_buf_t
*
* @return ==0 on success, negative HB_MEM_ERROR in case of failure
* @see hb_mem_common_buf_t
*/
int32_t hb_mem_pool_free_buf(uint64_t virt_addr);

/**
* Get the current status of the memory pool.
*
* @param[in]       fd, file descriptor of the pool, @hb_mem_pool_t
* @param[out]      pool, @see hb_mem_pool_t
*
* @return ==0 on success, negative HB_MEM_ERROR in case of failure
* @see hb_mem_pool_t
*/
int32_t hb_mem_pool_get_info(int32_t fd, hb_mem_pool_t * pool);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* HB_MEM_MGR_H */
