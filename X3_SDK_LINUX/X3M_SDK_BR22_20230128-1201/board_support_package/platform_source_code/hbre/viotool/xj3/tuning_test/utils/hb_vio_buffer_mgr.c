/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#include <string.h>
#include <errno.h>
#include <pthread.h>
#include <sys/mman.h>
#include <unistd.h>
#include "hb_vio_buffer_mgr.h"
#include "../pym/pym.h"
#include "../utils/list.h"
#include "../sif/sif.h"


static const char *const buf_state_name[BUFFER_INVALID] = {
	"Available",
	"Process",
	"Done",
	"Reprocess",
	"User"
};

static const char *const buf_type_name[HB_VIO_DATA_TYPE_MAX] = {
	"ipu_ds0",
	"ipu_ds1",
	"ipu_ds2",
	"ipu_ds3",
	"ipu_ds4",
	"ipu_us",
	"pym_feed_src",
	"pym",
	"sif_feed_src",
	"sif",
	"isp",
	"gdc"
};


pthread_mutex_t ion_lock = PTHREAD_MUTEX_INITIALIZER;
int g_ion_opened = 0;
int m_ionClient = -1;

buffer_mgr_t *buffer_manager_create(uint32_t pipeline_id,
				    VIO_DATA_TYPE_E buffer_type)
{

	buffer_mgr_t *me = NULL;

	if ((0 <= buffer_type) && (buffer_type < HB_VIO_DATA_TYPE_MAX)) {
		me = (buffer_mgr_t *) malloc(sizeof(buffer_mgr_t));
		if (me != NULL) {
			memset(me, 0, sizeof(buffer_mgr_t));
			me->pipeline_id = pipeline_id;
			me->buffer_type = buffer_type;
			vio_log
			    ("mgr create done! pipeline_id = %u, buffer_type = %d \n",
			     me->pipeline_id, me->buffer_type);
		} else {
			vio_err("mgr create failed, malloc failed !!!\n");
		}
	} else {
		vio_err
		    ("mgr create failed !!! pipeline_id = %u, invaild buf type = %d\n",
		     me->pipeline_id, buffer_type);
	}

	return me;
}

void buffer_manager_destroy(buffer_mgr_t * this)
{

	if (this != NULL) {
		free(this);
		this = NULL;
	}
}

int buffer_manager_init(buffer_mgr_t * this, uint32_t buffer_num)
{

	int ret = -1;

	if (this == NULL) {
		vio_err("buf mgr init but null point set.\n");
		return ret;
	}

	if (this->buffer_type == HB_VIO_PYM_DATA) {
		this->buf_nodes = malloc(sizeof(pym_buf_node_t) * HB_VIO_BUFFER_MAX_MP);
		memset(this->buf_nodes, 0, sizeof(pym_buf_node_t) * HB_VIO_BUFFER_MAX_MP);
		for (int i = 0; i < HB_VIO_BUFFER_MAX_MP; ++i) {
			((pym_buf_node_t *) (this->buf_nodes) +
			 i)->pym_buf.pym_img_info.buf_index = -1;
		}
	} else {
		this->buf_nodes = malloc(sizeof(buf_node_t) * HB_VIO_BUFFER_MAX_MP);
		memset(this->buf_nodes, 0, sizeof(buf_node_t) * HB_VIO_BUFFER_MAX_MP);
		for (int i = 0; i < HB_VIO_BUFFER_MAX_MP; ++i) {
			((buf_node_t *) (this->buf_nodes) +
			 i)->vio_buf.img_info.buf_index = -1;
		}
	}

	this->num_buffers = buffer_num;

	vio_dbg("this->num_buffers = %d, buffer type = %d", this->num_buffers,
		this->buffer_type);
// should move to the start
// the avaliable value init should according to the real value
// avaliable buf - prepare = sem[avaliable]
/*
	for (int i = 0; i < BUFFER_INVALID; i++) {
		ret = sem_init(&this->sem[i], 0, 0);
		if (ret == -1) {
			vio_dbg("sem_init(%d) failed! err(%s)\n", i,
				strerror(errno));
		}
	}
*/
	pthread_mutex_init(&(this->lock), NULL);

#ifndef DUMMY_BUF
	pthread_mutex_lock(&ion_lock);
	if (!g_ion_opened) {
		m_ionClient = ion_open();
		if (m_ionClient < 0) {
			vio_err("ion allocator open failed ret:%d!\n",
				m_ionClient);
			pthread_mutex_unlock(&ion_lock);
			return -1;
		}
		vio_log("ion allocator open success m_ionClient :%d !\n",
			m_ionClient);
		g_ion_opened = 1;
	}
	pthread_mutex_unlock(&ion_lock);
#endif
	return 0;
}

int buffer_manager_deinit(buffer_mgr_t * this)
{

	int ret = 0;
	if (this != NULL) {
		if (this->buf_nodes)
			free(this->buf_nodes);
		pthread_mutex_destroy(&this->lock);
		/*
		for (int i = 0; i < BUFFER_INVALID; i++) {
			sem_destroy(&this->sem[i]);
		}
		*/
	}
	return ret;
}

int ion_buffer_map(int size, int fd, char **addr)
{

	int ret = 0;
	char *ionAddr = NULL;

	if (size == 0) {
		vio_dbg("size equals zero !");
		ret = -1;
		goto func_exit;
	}

	if (fd <= 0) {
		vio_dbg(":fd=%d failed", size);
		ret = -1;
		goto func_exit;
	}

	ionAddr =
	    (char *)mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

	if (ionAddr == (char *)MAP_FAILED || ionAddr == NULL) {
		vio_dbg("ERR(%s):ion_map(size=%d) failed", __FUNCTION__, size);
		close(fd);
		ionAddr = NULL;
		ret = -1;
		goto func_exit;
	}

func_exit:

	*addr = ionAddr;

	return ret;
}

int ion_buffer_alloc(int *fd,
		     int size,
		     char **addr,
		     uint32_t ion_heap_mask,
		     uint32_t ion_flags, uint32_t ion_align, _Bool need_map)
{

	int ret = 0;
	int ionFd = 0;
	char *ionAddr = NULL;

#ifdef DUMMY_BUF

	*fd = 1;
	printf("DUMMY_BUF ==> malloc size = %d \n", size);
	*addr = malloc(size);
	if (*addr != NULL) {
		printf("DUMMY_BUF vaddr %p \n", *addr);
	} else {
		printf("DUMMY_BUF malloc failed !!! \n");
	}
	return 0;
#endif

	if (m_ionClient == 0) {
		vio_dbg("allocator is not yet created");
		ret = -1;
		goto func_exit;
	}

	if (size == 0) {
		vio_dbg("size equals zero");
		ret = -1;
		goto func_exit;
	}

	ret =
	    ion_alloc_fd(m_ionClient, size, ion_align, ion_heap_mask, ion_flags,
			 &ionFd);

	if (ret < 0) {
		vio_dbg("ion_alloc_fd(fd=%d) failed(%s)", ionFd,
			strerror(errno));
		ionFd = -1;
		ret = -1;
		goto func_exit;
	}

	if (need_map == true) {
		if (ion_buffer_map(size, ionFd, &ionAddr) != 1) {
			vio_dbg("map failed");
		}
	}

func_exit:

	*fd = ionFd;
	*addr = ionAddr;

	return ret;
}

int ion_buffer_free(int *fd, int size, char **addr, _Bool need_map)
{

#ifdef DUMMY_BUF
	printf("DUMMY_BUF==> fd %d release addr %p, size %d \n", *fd, *addr,
	       size);
	free(*addr);
	return 0;
#endif
	int ret = 0;

	if((fd == NULL) || (addr == NULL)) {
		vio_err("fd or addr is null \n");
		return -1;
	}
	int ionFd = *fd;
	char *ionAddr = *addr;

	if (ionFd < 0) {
		vio_err("ion_fd is lower than zero");
		ret = -1;
		goto func_exit;
	}

	if (need_map == true) {
		if (ionAddr == NULL) {
			vio_err("ion_addr equals NULL");
			ret = -1;
			goto func_exit;
		}

		if (munmap(ionAddr, size) < 0) {
			vio_err("munmap failed");
			ret = -1;
			goto func_exit;
		}
	}

	ion_close(ionFd);

	ionFd = -1;
	ionAddr = NULL;

func_exit:
	*fd = ionFd;
	*addr = ionAddr;

	return ret;
}

int ion_alloc_phy(int size, int *fd, char **vaddr, uint64_t * paddr)
{
	int ret = -1;
	ion_user_handle_t handle = 0;
	unsigned long p_addr = 0;
	char *ion_addr = NULL;
	uint64_t leng = 0;
	int isCached = 0;
	size_t m_ionAlign = 0;
	unsigned int m_ionHeapMask = ION_HEAP_CARVEOUT_MASK;
	unsigned int m_ionFlags =
	    (isCached ==
	     1 ? (ION_FLAG_CACHED | ION_FLAG_CACHED_NEEDS_SYNC) : 0);

	if((vaddr == NULL) || (paddr == NULL)) {
		vio_err("vaddr or paddr is null \n");
		return ret;
	}
	ret =
	    ion_alloc(m_ionClient, size, m_ionAlign, m_ionHeapMask, m_ionFlags,
		      &handle);
	if (ret < 0) {
		vio_err("ion_alloc failed. \n");
		return ret;
	}
	ret = ion_share(m_ionClient, handle, fd);
	if (ret < 0) {
			ion_free(m_ionClient, handle);
			vio_err("Failed to share ion memory.\n");
			return -1;
	}
	if (ion_buffer_map(size, *fd, &ion_addr) != 0) {
		vio_dbg("m_ionClient (%d)  map failed.\n", m_ionClient);
		return ret;
	}

	*vaddr = ion_addr;

	ret = ion_phys(m_ionClient, handle, (void *)&p_addr, &leng);
	if (ret < 0) {
		ion_free(m_ionClient, handle);
		close(*fd);
		vio_err("ion_phys failed.\n");
		return ret;
	}
	vio_log("Alloc size(%d) fd (%d) vaddr %p paddr = %lx, length = %lu\n",
		size, *fd, *vaddr, p_addr, leng);
	*paddr = p_addr;
	ion_free(m_ionClient, handle);

	return ret;
}

void offset_conv_pym_address(pym_buf_offset_t * offset, pym_buffer_t * pym_buf)
{
	int i = 0;
	uint64_t base_paddr = 0;
	void* base_vaddr = 0;

	base_paddr = pym_buf->pym[0].paddr[0];
	base_vaddr = pym_buf->pym[0].addr[0];
	for (i = 0; i < 24; i++) {
		if (i % 4 == 0) {
			pym_buf->pym[i / 4].paddr[0] =
			    base_paddr + offset->pym[i / 4].offset[0];
			pym_buf->pym[i / 4].paddr[1] =
			    base_paddr + offset->pym[i / 4].offset[1];
			pym_buf->pym[i / 4].addr[0] =
					(base_vaddr + offset->pym[i / 4].offset[0]);
			pym_buf->pym[i / 4].addr[1] =
			    base_vaddr + offset->pym[i / 4].offset[1];
			pym_buf->pym[i / 4].width = offset->pym[i / 4].width;
			pym_buf->pym[i / 4].height = offset->pym[i / 4].height;
			pym_buf->pym[i / 4].stride_size = offset->pym[i / 4].stride_size;
		} else if (offset->pym_roi[i / 4][i % 4 - 1].offset[0] != 0) {
			pym_buf->pym_roi[i / 4][i % 4 - 1].paddr[0] =
			    base_paddr + offset->pym_roi[i / 4][i % 4 - 1].offset[0];
			pym_buf->pym_roi[i / 4][i % 4 - 1].paddr[1] =
			    base_paddr + offset->pym_roi[i / 4][i % 4 - 1].offset[1];
			pym_buf->pym_roi[i / 4][i % 4 - 1].addr[0] =
			    base_vaddr + offset->pym_roi[i / 4][i % 4 - 1].offset[0];
			pym_buf->pym_roi[i / 4][i % 4 - 1].addr[1] =
			    base_vaddr + offset->pym_roi[i / 4][i % 4 - 1].offset[1];

			pym_buf->pym_roi[i / 4][i % 4 - 1].width =
				offset->pym_roi[i / 4][i % 4 - 1].width;
			pym_buf->pym_roi[i / 4][i % 4 - 1].height =
				offset->pym_roi[i / 4][i % 4 - 1].height;
			pym_buf->pym_roi[i / 4][i % 4 - 1].stride_size =
				offset->pym_roi[i / 4][i % 4 - 1].stride_size;

			//vio_dbg("paddr[0] = 0x%x, ds_uv_addr[%d] = 0x%x",i,info->spec.ds_y_addr[i],i,info->spec.ds_uv_addr[i]);
		}
	}

	for (i = 0; i < 6; i++) {
		if (offset->pym_us[i].offset[0] != 0) {
			pym_buf->us[i].paddr[0] =
			    base_paddr + offset->pym_us[i].offset[0];
			pym_buf->us[i].paddr[1] =
			    base_paddr + offset->pym_us[i].offset[1];
			pym_buf->us[i].addr[0] =
			    base_vaddr + offset->pym_us[i].offset[0];
			pym_buf->us[i].addr[1] =
			    base_vaddr + offset->pym_us[i].offset[1];

			pym_buf->us[i].width =
				offset->pym_us[i].width;
			pym_buf->us[i].height =
				offset->pym_us[i].height;
			pym_buf->us[i].stride_size =
				offset->pym_us[i].stride_size;
		}
	}
}

int buffer_mgr_alloc_mp(buffer_mgr_t * this, int planeCount,
		uint32_t * planeSize, void *img_info, uint32_t first_idx, int fd)
{

	int ret = 0;
	int mask = 0;
	int flags = 0;		//TODO modify according to the platform

	if (this == NULL) {
		vio_err("NULL buffer mgr set  !!! \n");
		return -1;
	}

	if (planeCount > HB_VIO_BUFFER_MAX_PLANES) {
		vio_err("abnormal planeCount =%d \n", planeCount);
		return -1;
	}
	if (first_idx >= HB_VIO_BUFFER_MAX_MP) {
		vio_err("first index valuet %d err. \n", first_idx);
		return -1;
	}
	buf_offset_t *offset_info = NULL;
	pym_buf_offset_t *pym_offset_info = NULL;

	int reqBufCount = this->num_buffers;
	uint32_t pipe_id = this->pipeline_id;
	this->dev_fd = fd;

	if (this->buffer_type == HB_VIO_PYM_DATA) {
		pym_offset_info = (pym_buf_offset_t *) img_info;
		for (int bufIndex = first_idx;
			bufIndex < (reqBufCount + first_idx); bufIndex++) {
			for (int planeIndex = 0; planeIndex < planeCount; planeIndex++) {
				((pym_buf_node_t *) (this->buf_nodes) + bufIndex)->pym_buf
				.pym_img_info.size[planeIndex] = planeSize[planeIndex];
				ret = ion_alloc_phy(((pym_buf_node_t *) (this->buf_nodes) +
					       bufIndex)->pym_buf.pym_img_info.size[planeIndex],
					      &(((pym_buf_node_t *) (this->buf_nodes)
						 + bufIndex)->pym_buf.pym_img_info.fd[planeIndex]),
					      &(((pym_buf_node_t *) (this->buf_nodes)
						 + bufIndex)->pym_buf.pym[0].addr[planeIndex]),
					      &(((pym_buf_node_t *) (this->buf_nodes)
						 +bufIndex)->pym_buf.pym[0].paddr[planeIndex])
				    );
			}
			((pym_buf_node_t *) (this->buf_nodes) +
			 bufIndex)->pym_buf.pym_img_info.planeCount = planeCount;
			((pym_buf_node_t *) (this->buf_nodes) +
			 bufIndex)->pym_buf.pym_img_info.pipeline_id = pipe_id;
			((pym_buf_node_t *) (this->buf_nodes) +
			 bufIndex)->pym_buf.pym_img_info.data_type = this->buffer_type;
			offset_conv_pym_address(pym_offset_info,
						&(((pym_buf_node_t *) (this->buf_nodes)+ bufIndex)->pym_buf));
		}
		//TODO y, uv save in one block buffer, and y plane & uv plane addr need remap

		for (int i = 0; i < BUFFER_INVALID; i++) {
			this->queued_count[i] = 0;
			vio_init_list_head(&this->buffer_queue[i]);
		}

		this->first_idx = first_idx;
		for (int i = first_idx; i < (reqBufCount + first_idx); ++i) {
			((pym_buf_node_t *) (this->buf_nodes) +
			 i)->pym_buf.pym_img_info.buf_index = i;
			buf_enqueue(this,
				&(((pym_buf_node_t *) (this->buf_nodes) + i)->node),
				BUFFER_AVAILABLE, MGR_LOCK);
		}
	} else {
		offset_info = (buf_offset_t *) (img_info);
		for (int bufIndex = first_idx;
			bufIndex < (reqBufCount + first_idx); bufIndex++) {
			for (int planeIndex = 0; planeIndex < planeCount; planeIndex++) {
				((buf_node_t *) (this->buf_nodes) +
				 bufIndex)->vio_buf.img_info.size[planeIndex] =
 														planeSize[planeIndex];
				if (this->buffer_type != HB_VIO_ISP_YUV_DATA)
				ret = ion_alloc_phy(((buf_node_t *) (this->buf_nodes)
						       + bufIndex)->vio_buf.img_info.size[planeIndex],
						      &(((buf_node_t *) (this->buf_nodes)
							 + bufIndex)->vio_buf.img_info.fd[planeIndex]),
						      &(((buf_node_t *) (this->buf_nodes)
							 + bufIndex)->vio_buf.img_addr.addr[planeIndex]),
						      &(((buf_node_t *) (this->buf_nodes)
							 + bufIndex)->vio_buf.img_addr.paddr[planeIndex])
					    );
			}
			((buf_node_t *) (this->buf_nodes) +
				bufIndex)->vio_buf.img_info.planeCount = planeCount;
			((buf_node_t *) (this->buf_nodes) +
				bufIndex)->vio_buf.img_info.pipeline_id = pipe_id;
			((buf_node_t *) (this->buf_nodes) +
				bufIndex)->vio_buf.img_info.data_type = this->buffer_type;
			if (this->buffer_type == HB_VIO_SIF_YUV_DATA )
				((buf_node_t *) (this->buf_nodes) +
				bufIndex)->vio_buf.img_info.img_format = SIF_FORMAT_YUV;
			else if (this->buffer_type == HB_VIO_SIF_RAW_DATA )
				((buf_node_t *) (this->buf_nodes) +
				bufIndex)->vio_buf.img_info.img_format = SIF_FORMAT_RAW;
			((buf_node_t *) (this->buf_nodes) +
			 bufIndex)->vio_buf.img_addr.width = offset_info->width;
			((buf_node_t *) (this->buf_nodes) +
			 bufIndex)->vio_buf.img_addr.height = offset_info->height;
			((buf_node_t *) (this->buf_nodes) +
			 bufIndex)->vio_buf.img_addr.stride_size = offset_info->stride_size;
		}

		for (int i = 0; i < BUFFER_INVALID; i++) {
			this->queued_count[i] = 0;
			vio_init_list_head(&this->buffer_queue[i]);
		}

		this->first_idx = first_idx;
		for (int i = first_idx; i < (reqBufCount + first_idx); ++i) {
			((buf_node_t *) (this->buf_nodes) +
			 i)->vio_buf.img_info.buf_index = i;
			buf_enqueue(this,
				    &(((buf_node_t *) (this->buf_nodes) +
				       i)->node), BUFFER_AVAILABLE, MGR_LOCK);
		}
#ifdef DEBUG_BUF_TRACE
		print_buffer_queue(this, BUFFER_AVAILABLE);
#endif
	}
	if (ret < 0) {
		vio_log("Buffer alloc failed ! pipe(%u) buf type(%d) !\n",
			this->pipeline_id, this->buffer_type);
	} else {
		vio_log("Buffer alloc done ! pipe(%u) buf type(%d) !\n",
			this->pipeline_id, this->buffer_type);
	}
	return ret;

}

int buffer_mgr_alloc(buffer_mgr_t * this, int planeCount, uint32_t * planeSize,
					  void *img_info)
{
	return buffer_mgr_alloc_mp(this, planeCount, planeSize, img_info, 0, -1);
}

int buffer_mgr_free(buffer_mgr_t * this, _Bool need_map)
{
	int ret = 0;
	if (this == NULL) {
		vio_err("buf mgr null in free !!");
		return ret;
	}

	int planeCount = 0;
	int reqBufCount = this->num_buffers;
	int first_idx = this->first_idx;
	int bufIndex;
	int bufIndex_node;
	buf_node_t * buf_node;
	pym_buf_node_t * pym_buf_node;

	if (this->buffer_type == HB_VIO_PYM_DATA) {
		for (int bufIndex = first_idx;
			bufIndex < (reqBufCount + first_idx); bufIndex++) {
			planeCount =
			    ((pym_buf_node_t *) (this->buf_nodes) +
			     bufIndex)->pym_buf.pym_img_info.planeCount;
#ifdef DEBUG_BUF_TRACE
			vio_dbg
			    ("PYM Buf Mgr free info :: bufIndex (%d), planeCount(%d)\n",
			     bufIndex, planeCount);
#endif
			for (int planeIndex = 0; planeIndex < planeCount;
			     planeIndex++) {
				ret = ion_buffer_free(&
						(((pym_buf_node_t *) (this->buf_nodes)
						 +
						  bufIndex)->pym_buf.pym_img_info.fd[planeIndex]),
						((pym_buf_node_t *) (this->buf_nodes)
						 +
						 bufIndex)->pym_buf.pym_img_info.size[planeIndex],
						&(((pym_buf_node_t *) (this->buf_nodes)
						 +
						 bufIndex)->pym_buf.pym[0].addr[planeIndex]), need_map);
			}
		}

		/* unmap */
		for (bufIndex = 0; bufIndex < HB_VIO_BUFFER_MAX_MP; bufIndex++) {
			pym_buf_node = (pym_buf_node_t *)(this->buf_nodes) + bufIndex;
			bufIndex_node = pym_buf_node->pym_buf.pym_img_info.buf_index;
			if ( (bufIndex_node == -1) || (bufIndex != bufIndex_node))
				continue;
			if (buffer_index_owner(this, bufIndex) == HB_VIO_BUFFER_THIS)
				continue;
			if (pym_buf_node->pym_buf.pym[0].addr[0] == 0)
				continue;
			planeCount = pym_buf_node->pym_buf.pym_img_info.planeCount;
			for (int planeIndex = 0; planeIndex < planeCount;
			     planeIndex++) {
				ret = munmap((char*)pym_buf_node->pym_buf.pym[0].addr[planeIndex],
					pym_buf_node->pym_buf.pym_img_info.size[planeIndex]);
				vio_dbg("munmap ret%d type %d index %d addr %lx size %d",
					ret, this->buffer_type, bufIndex,
					pym_buf_node->pym_buf.pym[0].addr[planeIndex],
					pym_buf_node->pym_buf.pym_img_info.size[planeIndex]);
			}
		}
	} else {
		for (bufIndex = first_idx;
			bufIndex < (reqBufCount + first_idx); bufIndex++) {

			planeCount =
			    ((buf_node_t *) (this->buf_nodes) +
			     bufIndex)->vio_buf.img_info.planeCount;
#ifdef DEBUG_BUF_TRACE
			vio_dbg
			    ("Normal Buf Mgr free info :: bufIndex (%d), planeCount(%d)\n",
			     bufIndex, planeCount);
#endif
			for (int planeIndex = 0; planeIndex < planeCount;
			     planeIndex++) {
				ret = ion_buffer_free(&
						(((buf_node_t *) (this->buf_nodes) +
						  bufIndex)->vio_buf.img_info.fd[planeIndex]),
						((buf_node_t *) (this->buf_nodes) +
						 bufIndex)->vio_buf.img_info.size[planeIndex],
						&(((buf_node_t *) (this->buf_nodes) +
						   bufIndex)->vio_buf.img_addr.addr[planeIndex]),
						   need_map);
			}
		}

		/* unmap */
		for (bufIndex = 0; bufIndex < HB_VIO_BUFFER_MAX_MP; bufIndex++) {
			buf_node = (buf_node_t *)(this->buf_nodes) + bufIndex;
			bufIndex_node = buf_node->vio_buf.img_info.buf_index;
			if ( (bufIndex_node == -1) || (bufIndex != bufIndex_node))
				continue;
			if (buffer_index_owner(this, bufIndex) == HB_VIO_BUFFER_THIS)
				continue;
			if (buf_node->vio_buf.img_addr.addr[0] == 0)
				continue;
			planeCount = buf_node->vio_buf.img_info.planeCount;
			for (int planeIndex = 0; planeIndex < planeCount;
			     planeIndex++) {
				ret = munmap((char*)buf_node->vio_buf.img_addr.addr[planeIndex],
					buf_node->vio_buf.img_info.size[planeIndex]);
				vio_dbg("munmap ret%d type %d index %d addr %lx size %d",
					ret, this->buffer_type, bufIndex,
					buf_node->vio_buf.img_addr.addr[planeIndex],
					buf_node->vio_buf.img_info.size[planeIndex]);
			}
		}
	}
	return ret;
}

buffer_owner_e buffer_index_owner(buffer_mgr_t * this, uint32_t index)
{
	if (index >= HB_VIO_BUFFER_MAX_MP)
		return HB_VIO_BUFFER_OWN_INVALID;
	else if (this->num_buffers == 0)
		return HB_VIO_BUFFER_THIS;
	else if ((index >= this->first_idx)
		&&(index < (this->first_idx + this->num_buffers)))
		return HB_VIO_BUFFER_THIS;
	else
		return HB_VIO_BUFFER_OTHER;
}

int buffer_other_index_init(buffer_mgr_t * this, uint32_t index)
{
	int self_index;
	buf_node_t *buf_node_dst, *buf_node_src;
	pym_buf_node_t *pym_buf_node_dst, *pym_buf_node_src;

	self_index = this->first_idx;
	if (this->buffer_type == HB_VIO_PYM_DATA) {
		pym_buf_node_dst = (pym_buf_node_t*)this->buf_nodes + index;
		if (pym_buf_node_dst->pym_buf.pym_img_info.buf_index != -1)
			return 0;
		pym_buf_node_src = (pym_buf_node_t*)this->buf_nodes + self_index;
		memcpy(&pym_buf_node_dst->pym_buf, &pym_buf_node_src->pym_buf,
			sizeof(hb_vio_buffer_t));
		pym_buf_node_dst->pym_buf.pym_img_info.buf_index = -1;
		pym_buf_node_dst->pym_buf.pym_img_info.state = BUFFER_INVALID;

	} else {
		buf_node_dst = (buf_node_t*)this->buf_nodes + index;
		if (buf_node_dst->vio_buf.img_info.buf_index != -1)
			return 0;
		buf_node_src = (buf_node_t*)this->buf_nodes + self_index;
		memcpy(&buf_node_dst->vio_buf, &buf_node_src->vio_buf,
			sizeof(hb_vio_buffer_t));
		buf_node_dst->vio_buf.img_info.buf_index = -1;
		buf_node_dst->vio_buf.img_info.state = BUFFER_INVALID;
	}
	return 0;
}


int buf_enqueue(buffer_mgr_t * this, queue_node * node, buffer_state_e state,
				mgr_lock_state_e lock)
{

	if (this == NULL || node == NULL) {
		vio_err("buf mgr was NULL !!!");
		return -1;
	}

	/*
	 *       Type     Node
	 *       normal : buf_node_t
	 *       pym    : pym_buf_node_t
	 */
	 if (lock == MGR_LOCK)
	pthread_mutex_lock(&this->lock);
	if (this->buffer_type == HB_VIO_PYM_DATA) {
		if ((((pym_buf_node_t *) node)->pym_buf.pym_img_info.state == state)
			&& (state != BUFFER_AVAILABLE)) {
			vio_list_del(node);
			this->queued_count[state]--;
		}

		((pym_buf_node_t *) node)->pym_buf.pym_img_info.state = state;
		vio_list_add_tail(&(((pym_buf_node_t *) node)->node),
		                  &this->buffer_queue[state]);
		this->queued_count[state]++;
#ifdef DEBUG_BUF_TRACE
		vio_dbg
		    ("pipe(%d), pym put buf index(%d) to queue(%s),now left buf(%d).\n",
		     ((pym_buf_node_t *) node)->pym_buf.pym_img_info.pipeline_id,
		     ((pym_buf_node_t *) node)->pym_buf.pym_img_info.buf_index,
		     buf_state_name[state],
		     this->queued_count[state]);
#endif
	} else {
		/*
		 * driver may force use frame when frame too little
		 * so HAL may enqueue the same frame to the same list,
		 * then the list will err.
		 */
		if ((((buf_node_t *) node)->vio_buf.img_info.state == state)
			&& (state != BUFFER_AVAILABLE)) {
			vio_list_del(node);
			this->queued_count[state]--;
		}

		((buf_node_t *) node)->vio_buf.img_info.state = state;
		vio_list_add_tail(&(((buf_node_t *) node)->node),
		                  &this->buffer_queue[state]);
		this->queued_count[state]++;
#ifdef DEBUG_BUF_TRACE
		vio_dbg
		    ("pipe(%d),normal put buf type(%d)(%s) index(%d) to queue(%s), now"
	   " left buf(%d).\n",
		     ((buf_node_t *) node)->vio_buf.img_info.pipeline_id,
		     ((buf_node_t *) node)->vio_buf.img_info.data_type,
		     buf_type_name[this->buffer_type],
		     ((buf_node_t *) node)->vio_buf.img_info.buf_index,
		     buf_state_name[state],
		     this->queued_count[state]);
#endif
	}
	if(lock == MGR_LOCK)
	pthread_mutex_unlock(&this->lock);
	return 0;
}

queue_node *buf_dequeue(buffer_mgr_t * this, buffer_state_e state,
						mgr_lock_state_e lock)
{

	queue_node *node = NULL;

	if (this == NULL) {
		vio_err("buf mgr was NULL !!!\n");
		return NULL;
	}
	if (state == BUFFER_INVALID) {
		vio_err("buf state was BUFFER_INVALID !!!\n");
		return NULL;
	}
	if(lock == MGR_LOCK)
	pthread_mutex_lock(&this->lock);
	if (vio_list_empty(&this->buffer_queue[state])) {
		if(lock == MGR_LOCK)
		pthread_mutex_unlock(&this->lock);
		vio_dbg
		    ("buf_dequeue(type %d)--->There no item, queued_count[%s] = %d\n",
		this->buffer_type, buf_state_name[state], this->queued_count[state]);
		return NULL;
	}

	node = list_first(&this->buffer_queue[state]);
	if (node) {
		vio_list_del(node);
		this->queued_count[state]--;
		if (this->buffer_type == HB_VIO_PYM_DATA) {
			((pym_buf_node_t *) node)->pym_buf.pym_img_info.state =
			    BUFFER_INVALID;
#ifdef DEBUG_BUF_TRACE
			vio_dbg
			    ("pipe(%d), pym get buf index(%d) from queue(%s), now left buf"
		"(%d).\n",
			     ((pym_buf_node_t *) node)->pym_buf.pym_img_info.pipeline_id,
			     ((pym_buf_node_t *) node)->pym_buf.pym_img_info.buf_index,
			     buf_state_name[state],
			     this->queued_count[state]);
#endif
		} else {
			((buf_node_t *) node)->vio_buf.img_info.state =
			    BUFFER_INVALID;
#ifdef DEBUG_BUF_TRACE
			vio_dbg
			    ("pipe(%d), normal get buf type(%d)(%s) index(%d)from queue(%s)"
		         "now left total buf(%d).\n",
			     ((buf_node_t *) node)->vio_buf.img_info.pipeline_id,
			     ((buf_node_t *) node)->vio_buf.img_info.data_type,
			      buf_type_name[this->buffer_type],
			     ((buf_node_t *) node)->vio_buf.img_info.buf_index,
			     buf_state_name[state],
			     this->queued_count[state]);
#endif
		}
	}
	if(lock == MGR_LOCK)
	pthread_mutex_unlock(&this->lock);
	return node;
}

int trans_buffer(buffer_mgr_t * this, queue_node * node, buffer_state_e state,
				mgr_lock_state_e lock)
{
	if (this == NULL || node == NULL) {
		vio_err("buf mgr or node was NULL !!!");
		return -1;
	}
	buf_node_t *buf_node = NULL;
	pym_buf_node_t *pym_buf_node = NULL;

	buffer_state_e buf_state = 0;

	if (this->buffer_type == HB_VIO_PYM_DATA) {
		pym_buf_node = (pym_buf_node_t *) node;
		buf_state = pym_buf_node->pym_buf.pym_img_info.state;
	} else {
		buf_node = (buf_node_t *) node;
		buf_state = buf_node->vio_buf.img_info.state;
	}

	if (state == BUFFER_INVALID || buf_state == BUFFER_INVALID) {
		vio_err("buf state was BUFFER_INVALID, wrong usage! not allow "
		  " (deque -> trans).");
		return -1;
	}
	if(lock == MGR_LOCK)
	pthread_mutex_lock(&this->lock);
	if (vio_list_empty(&this->buffer_queue[buf_state])) {
		if(lock == MGR_LOCK)
		pthread_mutex_unlock(&this->lock);
		vio_log("trans_buffer-->There no item, queued_count[%s] = %d",
			buf_state_name[state], this->queued_count[state]);
		return 0;
	}

	vio_list_del(node);
	this->queued_count[buf_state]--;
	if(lock == MGR_LOCK)
	pthread_mutex_unlock(&this->lock);

	return buf_enqueue(this, node, state, lock);

}

queue_node *peek_buffer(buffer_mgr_t * this, buffer_state_e state,
						mgr_lock_state_e lock)
{
	queue_node *node = NULL;

	if (this == NULL) {
		vio_err("buf mgr was NULL !!!");
		return NULL;
	}
	if (state == BUFFER_INVALID) {
		vio_err("buf state was BUFFER_INVALID !!!");
		return NULL;
	}
	if(lock == MGR_LOCK)
	pthread_mutex_lock(&this->lock);
	if (vio_list_empty(&this->buffer_queue[state])) {
		if(lock == MGR_LOCK)
		pthread_mutex_unlock(&this->lock);
		return NULL;
	}

	node = list_first(&this->buffer_queue[state]);
	if(lock == MGR_LOCK)
	pthread_mutex_unlock(&this->lock);
	return node;
}

queue_node *peek_buffer_tail(buffer_mgr_t * this, buffer_state_e state,
							mgr_lock_state_e lock)
{
	queue_node *node = NULL;

	if (this == NULL) {
		vio_err("buf mgr was NULL !!!");
		return NULL;
	}
	if (state == BUFFER_INVALID) {
		vio_err("buf state was BUFFER_INVALID !!!");
		return NULL;
	}
	if(lock == MGR_LOCK)
	pthread_mutex_lock(&this->lock);
	if (vio_list_empty(&this->buffer_queue[state])) {
		if(lock == MGR_LOCK)
		pthread_mutex_unlock(&this->lock);
		return NULL;
	}
	node = list_last(&this->buffer_queue[state]);
	if(lock == MGR_LOCK)
	pthread_mutex_unlock(&this->lock);
	return node;
}

queue_node *find_pop_buffer(buffer_mgr_t *this, buffer_state_e state,
                            int buf_index, mgr_lock_state_e lock)
{
	queue_node *node_pos, *node_next;
	buf_node_t *normal_node;
	pym_buf_node_t *pym_node;

	if (this == NULL || state == BUFFER_INVALID) {
		vio_err("buf mgr was NULL or  BUFFER_INVALID !!! \n");
		return NULL;
	}
	if(lock == MGR_LOCK)
	pthread_mutex_lock(&this->lock);

	if (vio_list_empty(&this->buffer_queue[state])) {
		if(lock == MGR_LOCK)
		pthread_mutex_unlock(&this->lock);
		vio_log("find_pop_buffer-->There no item, queued_count[%s] = %d",
			buf_state_name[state], this->queued_count[state]);
		return NULL;
	}

	list_for_each_safe(node_pos, node_next, &this->buffer_queue[state]) {

		if (this->buffer_type == HB_VIO_PYM_DATA) {
			pym_node = (pym_buf_node_t *) node_pos;
			if (pym_node->pym_buf.pym_img_info.buf_index == buf_index) {
				vio_list_del(node_pos);
				this->queued_count[state]--;
				((pym_buf_node_t *) node_pos)->pym_buf.pym_img_info.state =
			    BUFFER_INVALID;
				if(lock == MGR_LOCK)
				pthread_mutex_unlock(&this->lock);
				return node_pos;
			}
		} else {
			normal_node = (buf_node_t *) node_pos;
			if (normal_node->vio_buf.img_info.buf_index == buf_index) {
				vio_list_del(node_pos);
				this->queued_count[state]--;
				((buf_node_t *) node_pos)->vio_buf.img_info.state =
				BUFFER_INVALID;
				if(lock == MGR_LOCK)
				pthread_mutex_unlock(&this->lock);
				return node_pos;
			}
		}
	}
	if(lock == MGR_LOCK)
	pthread_mutex_unlock(&this->lock);
	vio_err("sorry, There no buf (%d) you wanted !\n", buf_index);
	return NULL;
}

int buf_mgr_flush_AlltoAvali(buffer_mgr_t *this, mgr_lock_state_e lock)
{
	queue_node *node_pos, *node_next;
	buffer_state_e i;
	if(this == NULL) {
		vio_err("Mgr was null, failed.\n");
		return -1;
	}

	if(lock == MGR_LOCK)
	pthread_mutex_lock(&this->lock);

	for (i = BUFFER_PROCESS; i < BUFFER_INVALID; i++) {
		list_for_each_safe(node_pos, node_next, &this->buffer_queue[i])
		    trans_buffer(this, node_pos, BUFFER_AVAILABLE, MGR_NO_LOCK);
	}

	if (this->queued_count[BUFFER_AVAILABLE] != this->num_buffers) {
		vio_err("memory leak happen type(%d) orign alloced(%d) ==> now %s(%d)\n",
			this->buffer_type,
			this->num_buffers,
			buf_state_name[BUFFER_AVAILABLE],
			this->queued_count[BUFFER_AVAILABLE]);
	}
	if(lock == MGR_LOCK)
	pthread_mutex_unlock(&this->lock);
	return 0;
}

int buf_mgr_flush_Queueto(buffer_mgr_t * this, buffer_state_e src_queue,
                         buffer_state_e dst_queue, mgr_lock_state_e lock)
{
	queue_node *node_pos, *node_next;
	buffer_state_e i;
	buf_node_t *normal_node = NULL;
	pym_buf_node_t *pym_node = NULL;
	if(this == NULL) {
		vio_err("Mgr was null, failed.\n");
		return -1;
	}
	if((src_queue >= BUFFER_INVALID || src_queue < BUFFER_AVAILABLE)
		|| (dst_queue >= BUFFER_INVALID || dst_queue < BUFFER_AVAILABLE)) {
		vio_err("Mgr flush type was invalid. Type src(%d), dst(%d)\n",
		        src_queue, dst_queue);
		return -1;
	}

	if(lock == MGR_LOCK)
	pthread_mutex_lock(&this->lock);

	list_for_each_safe(node_pos, node_next, &this->buffer_queue[src_queue]) {
		trans_buffer(this, node_pos, dst_queue, MGR_NO_LOCK);
		if (this->buffer_type == HB_VIO_PYM_DATA) {
			int val1 = -1;
			sem_getvalue(&this->sem[src_queue], &val1);
			pym_node = (pym_buf_node_t *) node_pos;
			vio_dbg("sem(%d)trans type%d buf index %d"
					"%s(num %d)->%s(num %d) %s(%d) %s(%d)\n",
						val1,
						this->buffer_type,
						pym_node->pym_buf.pym_img_info.buf_index,
						buf_state_name[src_queue],
						this->queued_count[src_queue],
						buf_state_name[dst_queue],
						this->queued_count[dst_queue],
						buf_state_name[BUFFER_PROCESS],
						this->queued_count[BUFFER_PROCESS],
						buf_state_name[BUFFER_USER],
						this->queued_count[BUFFER_USER]);
			if(val1 >= 1 && BUFFER_DONE == src_queue) {
				//Here down the done sem count for done queue
				sem_wait(&this->sem[src_queue]);
				sem_getvalue(&this->sem[src_queue], &val1);
				vio_dbg("Down the sem count ok. now %d\n", val1);
			}
		} else {
			int val2 = -1;
			sem_getvalue(&this->sem[src_queue], &val2);
			normal_node = (buf_node_t *) node_pos;
			vio_dbg("sem(%d)trans type%d buf index %d"
					"%s(num %d)->%s(num %d) %s(%d) %s(%d)\n",
						val2,
						this->buffer_type,
						normal_node->vio_buf.img_info.buf_index,
						buf_state_name[src_queue],
						this->queued_count[src_queue],
						buf_state_name[dst_queue],
						this->queued_count[dst_queue],
						buf_state_name[BUFFER_PROCESS],
						this->queued_count[BUFFER_PROCESS],
						buf_state_name[BUFFER_USER],
						this->queued_count[BUFFER_USER]);
			if(val2 >= 1 && BUFFER_DONE == src_queue) {
				//Here down the done sem count for done queue
				sem_wait(&this->sem[src_queue]);
				sem_getvalue(&this->sem[src_queue], &val2);
				vio_dbg("Down the sem count ok. now %d\n", val2);
			}
		}
	}

	if (this->queued_count[BUFFER_DONE] != 0) {
		for (i = BUFFER_PROCESS; i < BUFFER_INVALID; i++) {
			vio_err("something wrong ==> now %s has buf(%d)\n",
			        buf_state_name[i],
			        this->queued_count[i]);
		}
	}
	if(lock == MGR_LOCK)
	pthread_mutex_unlock(&this->lock);
	return 0;
}

void print_buffer_queue(buffer_mgr_t * this, buffer_state_e state,
						mgr_lock_state_e lock)
{
	queue_node *node_pos, *node_next;
	buf_node_t *normal_node;
	pym_buf_node_t *pym_node;
	if(lock == MGR_LOCK)
	pthread_mutex_lock(&this->lock);

	list_for_each_safe(node_pos, node_next, &this->buffer_queue[state]) {

		if (this->buffer_type == HB_VIO_PYM_DATA) {
			pym_node = (pym_buf_node_t *) node_pos;
			vio_log
			    ("mgr type(%d) state(%s) buf_index %d,  size[%d]\n",
			     this->buffer_type, buf_state_name[state],
			     pym_node->pym_buf.pym_img_info.buf_index,
			     pym_node->pym_buf.pym_img_info.size[0]);
		} else {
			normal_node = (buf_node_t *) node_pos;
			vio_log
			    ("mgr type(%d)state(%s)index %d,width(%u),height(%u),stride(%u) paddr[0](%lu),paddr[1](%lu)\n",
			     this->buffer_type, buf_state_name[state],
			     normal_node->vio_buf.img_info.buf_index,
			     normal_node->vio_buf.img_addr.width,
			     normal_node->vio_buf.img_addr.height,
			     normal_node->vio_buf.img_addr.stride_size,
			     normal_node->vio_buf.img_addr.paddr[0],
			     normal_node->vio_buf.img_addr.paddr[1]);
		}
	}
	if(lock == MGR_LOCK)
	pthread_mutex_unlock(&this->lock);

}

void buf_mgr_print_queues(buffer_mgr_t * this, mgr_lock_state_e lock)
{
	if(lock == MGR_LOCK)
	pthread_mutex_lock(&this->lock);

	for (int i = 0; i < BUFFER_INVALID; i++)
		print_buffer_queue(this, (buffer_state_e) i, MGR_NO_LOCK);

	if(lock == MGR_LOCK)
	pthread_mutex_unlock(&this->lock);
}

void buf_mgr_print_qcount(buffer_mgr_t * this, mgr_lock_state_e lock)
{
	if(this != NULL) {
	if(lock == MGR_LOCK)
	pthread_mutex_lock(&this->lock);
	vio_dbg("Mgr(%d)state:Avail(%u)Process(%u)Done(%u)Repro(%u)User(%u).\n",
		this->buffer_type,
		this->queued_count[BUFFER_AVAILABLE],
		this->queued_count[BUFFER_PROCESS],
		this->queued_count[BUFFER_DONE],
		this->queued_count[BUFFER_REPROCESS],
		this->queued_count[BUFFER_USER]);
	if(lock == MGR_LOCK)
	pthread_mutex_unlock(&this->lock);
	}
}

