/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <sys/mman.h>
#include "dev_ioctl.h"
#include "hb_utils.h"

void print_frame_info(struct frame_info *info)
{
	vio_dbg("frameInfo bufferindex(%d)", info->bufferindex);
	vio_dbg("frameInfo format(%d)", info->format);
	vio_dbg("frameInfo width(%d)", info->width);
	vio_dbg("frameInfo height(%d)", info->height);
	vio_dbg("frameInfo frame_id(%d)", info->frame_id);
	vio_dbg("frameInfo planes(%d)", info->planes);
	vio_dbg("frameInfo addr0(%u)", info->addr[0]);
	vio_dbg("frameInfo addr1(%u)", info->addr[1]);
}

int dev_node_qbuf(int fd, uint64_t cmd, hb_vio_buffer_t *buf)
{
	int ret = -1;
	struct frame_info frameInfo = { 0 };

	if (fd < 0) {
		vio_err("invalid fd(%d) was set !\n", fd);
		return ret;
	}
	if (cmd < 0) {
		vio_err("invalid cmd(%lu) was set !\n", cmd);
		return ret;
	}
	if (buf != NULL) {
		vio_dbg("pipe(%d) Try to q buf(%d) type(%d) to node.\n",
			buf->img_info.pipeline_id,
			buf->img_info.buf_index, buf->img_info.data_type);

		//info for kernel
		frameInfo.bufferindex = buf->img_info.buf_index;
		frameInfo.format = buf->img_info.img_format;
		frameInfo.width = buf->img_addr.width;
		frameInfo.height = buf->img_addr.height;
		frameInfo.frame_id = buf->img_info.frame_id;
		frameInfo.tv = buf->img_info.tv;
		frameInfo.planes = buf->img_info.planeCount;
		frameInfo.addr[0] = buf->img_addr.paddr[0];	//y
		frameInfo.addr[1] = buf->img_addr.paddr[1];	//uv
		frameInfo.ion_share_fd[0] = buf->img_info.fd[0];
		frameInfo.ion_share_fd[1] = buf->img_info.fd[1];

		ret = ioctl(fd, cmd, &frameInfo);
		if (ret < 0) {
			vio_err("failed to ioctl: qbuf (%d - %s)\n", errno,
				strerror(errno));
			return ret;
		}
#ifdef DEBUG_BUF_TRACE
		print_frame_info(&frameInfo);
#endif
	} else {
		vio_err(" buf was null ! \n");
	}

	return ret;
}

int dev_node_dqbuf(int fd, uint64_t cmd, struct frame_info *info)
{
	int ret = 0;
	if (fd < 0) {
		vio_err("invalid fd(%d) was set !\n", fd);
		return ret;
	}
	if (cmd < 0) {
		vio_err("invalid cmd(%lu) was set !\n", cmd);
		return ret;
	}
	struct pollfd pfds = { 0 };

	pfds.fd = fd;
	pfds.events = POLLIN | POLLERR;
	pfds.revents = 0;

	ret = poll(&pfds, 1, DEV_POLL_TIMEOUT);
	if (ret < 0) {
		vio_dbg("dev poll err: %d, %s\n", errno, strerror(errno));
		return ret;
	} else if (ret == 0) {
		vio_dbg("dev poll Timeout(%d): %d, %s\n", DEV_POLL_TIMEOUT,
			errno, strerror(errno));
		return -1;
	}
	//ret > 0 : POLLIN(data ready) or POLLERR(frame drop)
    if (pfds.revents & POLLIN) {
		ret = ioctl(fd, cmd, info);
		if (ret < 0) {
			vio_err("failed to ioctl: dq (%d - %s)", errno, strerror(errno));
			return ret;
		}
		//vio_dbg("dev(%d)poll success : %d.\n", fd, pfds.revents);
	} else if (pfds.revents & POLLERR) {
		vio_err("dev fd(%d) frame drop!\n", fd);
			return -1;
	}
	return ret;
}

// sif, ipu norma buf operation can use same func
buf_node_t *entity_node_dqbuf(int fd, buffer_mgr_t * buf_mgr,
								uint64_t dev_cmd, buffer_state_e buf_queue)
{
	int ret = -1;
	int cur_index = -1;
	int dq_index = -1;
	int buf_type = -1;
	uint32_t pipe_id = -1;
	buf_node_t *buf_node = NULL;
	buffer_owner_e buf_owner;

	if (fd < 0) {
		vio_err("dev dq buf fd was null !\n");
		return NULL;
	}
	if (buf_mgr == NULL) {
		vio_err("dev dq buf mgr was null !\n");
		return NULL;
	}

	buffer_mgr_t *mgr = buf_mgr;
	buf_type = mgr->buffer_type;
	pipe_id = mgr->pipeline_id;
	struct frame_info frameInfo = { 0 };

	ret = dev_node_dqbuf(fd, dev_cmd, &frameInfo);
	if (ret < 0) {
		vio_err("dev type(%d) dq failed\n", buf_type);
		return NULL;
	}

	dq_index = frameInfo.bufferindex;
	bufmgr_en_lock(mgr);
	buf_owner = buffer_index_owner(mgr, dq_index);
	if (buf_owner == HB_VIO_BUFFER_THIS) {
		buf_node = (buf_node_t *)peek_buffer(mgr, buf_queue, MGR_NO_LOCK);
		if (buf_node != NULL) {
			cur_index = buf_node->vio_buf.img_info.buf_index;
			if (cur_index == frameInfo.bufferindex) {
				buf_node = (buf_node_t *)buf_dequeue(mgr, buf_queue, MGR_NO_LOCK);
				vio_dbg
				("pipe(%u) dq buf type(%d)done! kernel index(%d)last index in "
				"process(%d)!\n",
				pipe_id, buf_type, frameInfo.bufferindex, cur_index);
			} else {
				buf_node = (buf_node_t *)find_pop_buffer(mgr,
					buf_queue, frameInfo.bufferindex, MGR_NO_LOCK);
				vio_dbg
				("pipe(%u)Find buf type(%d),cur queue(%d)no match with proccess dq(%d)!\n",
					pipe_id, buf_type, cur_index, dq_index);
				if (buf_node)
					vio_dbg("find success.");
				else
					vio_dbg("find fail.");
			}
		} else {
			vio_err("pipe(%u)There no q before dq, should not be here!\n", pipe_id);
		}
#ifdef DEBUG_BUF_TRACE
		print_buffer_queue(mgr, buf_queue, MGR_NO_LOCK);
#endif
	} else if (buf_owner == HB_VIO_BUFFER_OTHER) {
		buffer_other_index_init(mgr, dq_index);
		buf_node = (buf_node_t*)frame_info_to_oth_idx(mgr, &frameInfo);
	} else {
		vio_err("pipe(%u)dq index %d err\n", pipe_id, dq_index);
	}
	bufmgr_un_lock(mgr);

	if (buf_node != NULL) {
		frame_info_to_buf(&buf_node->vio_buf.img_info, &frameInfo);
	}
	return buf_node;
}

int dev_get_buf_timeout(buffer_mgr_t * mgr,
								buffer_state_e buf_queue, int timeout)
{
	struct timeval time_now = { 0 };
	struct timespec ts = { 0 };
	buf_node_t *buf_node = NULL;
	hb_vio_buffer_t *buf = NULL;
	int ret = 0;
	if(mgr == NULL) {
		vio_err("mgr was null.\n");
		return -2;
	}
	int32_t pipe_id = mgr->pipeline_id;
	int buf_type = mgr->buffer_type;

	gettimeofday(&time_now, NULL);

	time_add_ms(&time_now, timeout);
	ts.tv_sec = time_now.tv_sec;
	ts.tv_nsec = time_now.tv_usec * 1000;
	int val1 = -1;
	sem_getvalue(&mgr->sem[buf_queue], &val1);
	//vio_dbg("pipe(%d) sem_value(%d) data type(%d) queue(%d).\n",
			//pipe_id, val1, buf_type, buf_queue);
	ret = sem_timedwait(&mgr->sem[buf_queue], &ts);
	if (ret == -1) {
		if (errno == ETIMEDOUT) {
			gettimeofday(&time_now, NULL);
			vio_err("pipe(%u)TIME OUT ,sec(%ld)usec(%ld)!\n", \
			pipe_id, time_now.tv_sec, time_now.tv_usec);
		}
		vio_err("pipe(%u)Get buf sem_timedwait failed %s!\n",
			pipe_id, strerror(errno));
		return ret;
	} else {
		//vio_dbg("pipe(%d)Type(%d)sem wait success.\n", pipe_id, buf_type);
	}
	return 0;
}

int entity_put_done_buf(buffer_mgr_t * mgr, hb_vio_buffer_t * buf,
				int fd, uint64_t dev_cmd)
{
	int ret = 0;
	int buf_type = -1;
	buf_node_t *buf_node = NULL;
	hb_vio_buffer_t *user_buf = NULL;
	buffer_owner_e buf_owner;

	if (mgr == NULL || buf == NULL) {
		vio_err("put done buf Mgr or buf was null!\n");
		return -1;
	}
	uint32_t pipe_id = mgr->pipeline_id;
	buf_type = buf->img_info.data_type;

	if (buf_type != mgr->buffer_type) {
		vio_err("Put done buf Mgr and buf type was mismatch!\n");
		return -1;
	}
	bufmgr_en_lock(mgr);
	buf_node = (buf_node_t *)peek_buffer(mgr, BUFFER_USER, MGR_NO_LOCK);
	if (buf_node != NULL) {
		user_buf = &buf_node->vio_buf;
		buf_owner = buffer_index_owner(mgr, buf->img_info.buf_index);
		if (buf_owner == HB_VIO_BUFFER_THIS) {
			if (user_buf->img_info.buf_index == buf->img_info.buf_index) {
				vio_dbg("pipe(%u)Put buf type(%d)sequence matched index(%d)!\n",
					pipe_id, buf_type, buf->img_info.buf_index);
				trans_buffer(mgr, &buf_node->node, BUFFER_AVAILABLE,
													MGR_NO_LOCK);
			} else {
				buf_node = (buf_node_t *) find_pop_buffer(mgr, BUFFER_USER,
										buf->img_info.buf_index, MGR_NO_LOCK);
				if (buf_node != NULL) {
					buf_enqueue(mgr, &buf_node->node, BUFFER_AVAILABLE,
														MGR_NO_LOCK);
					vio_dbg("pipe(%u)Put buf(%d) sequence unmatch but find "
						"(user %d)(now %d)!\n",
						pipe_id,
						buf_type,
						user_buf->img_info.buf_index,
						buf->img_info.buf_index);
				} else {
					vio_err("pipe(%u)buf type(%d)index(%d)user to release, "
						"no mark in user queue! may already free\n",
						pipe_id,
						buf_type,
						buf->img_info.buf_index);
					bufmgr_un_lock(mgr);
					return -1;
				}
			}
			if (buffer_index_owner(mgr, buf->img_info.buf_index)
				== HB_VIO_BUFFER_OTHER)
				buf_dequeue(mgr, BUFFER_USER, MGR_NO_LOCK);
		} else if (buf_owner == HB_VIO_BUFFER_OTHER && (fd > 0)) {
			/* the buffer belong to other process, should q early.*/
			dev_node_qbuf(fd, dev_cmd, buf);
			if (buffer_index_owner(mgr, buf->img_info.buf_index)
				== HB_VIO_BUFFER_OTHER)
				buf_dequeue(mgr, BUFFER_USER, MGR_NO_LOCK);
		} else {
			vio_err("pipe(%u)buf type(%d)index %d greater than 128, ",
				pipe_id,
				buf_type,
				buf->img_info.buf_index);
		}
	} else {
			vio_err("pipe(%u)buf type(%d)index(%d)user to release, "
				"no mark in user queue! may already free\n",
				pipe_id,
				buf_type,
				buf->img_info.buf_index);
			bufmgr_un_lock(mgr);
			return -1;
	}
	bufmgr_un_lock(mgr);
	vio_dbg("pipe(%u)User put buf type(%d)index(%d),frame_id(%d)time_stamp(%lu)",
		pipe_id,
		buf->img_info.data_type,
		buf->img_info.buf_index,
		buf->img_info.frame_id, buf->img_info.time_stamp);
	return 0;
}

int dev_buf_handle(buffer_mgr_t *mgr, buf_node_t * buf_node)
{
	buffer_owner_e index_owner;

	if (mgr == NULL || buf_node == NULL) {
		vio_err("mgr or buf was null.\n");
		return -1;
	}
	uint32_t pipe_id = mgr->pipeline_id;
	int buf_type = mgr->buffer_type;
	bufmgr_en_lock(mgr);
	index_owner = buffer_index_owner(mgr,
		buf_node->vio_buf.img_info.buf_index);
	if ((mgr->queued_count[BUFFER_AVAILABLE] < 2)
		&& (index_owner == HB_VIO_BUFFER_THIS)) {
		buf_enqueue(mgr, &(buf_node->node),
			BUFFER_AVAILABLE, MGR_NO_LOCK);
		vio_err("pipe(%u)buf type(%d) ava(%d)pro(%d)done(%d)rep(%d)user(%d)"
			"push buf(%d) frame(%u) back to avali \n",
			pipe_id,
			buf_type,
			mgr->queued_count[BUFFER_AVAILABLE],
			mgr->queued_count[BUFFER_PROCESS],
			mgr->queued_count[BUFFER_DONE],
			mgr->queued_count[BUFFER_REPROCESS],
			mgr->queued_count[BUFFER_USER],
			buf_node->vio_buf.img_info.buf_index,
			buf_node->vio_buf.img_info.frame_id);
	} else {
		buf_enqueue(mgr, &buf_node->node, BUFFER_DONE,
					MGR_NO_LOCK);
		sem_post(&mgr->sem[BUFFER_DONE]);
		vio_dbg("pipe(%u)buf type(%d) buf(%d) frame (%u) trans 2 Done.\n",
				pipe_id,
				buf_type,
				buf_node->vio_buf.img_info.buf_index,
				buf_node->vio_buf.img_info.frame_id);
	}
	bufmgr_un_lock(mgr);
	return 0;
}
void frame_info_to_buf(image_info_t * to_buf, struct frame_info *info)
{
	if (to_buf == NULL || info == NULL) {
		vio_err("buf or info null, pls check.\n");
		return;
	}
	to_buf->frame_id = info->frame_id;
	to_buf->time_stamp = info->timestamps;
	to_buf->tv.tv_sec = info->tv.tv_sec;
	to_buf->tv.tv_usec = info->tv.tv_usec;
#ifdef DEBUG_BUF_TRACE
	vio_dbg("to_buf->frame_id = %u time_stamp(%lu)tv_sec(%ld)tv_usec(%ld)\n",
	to_buf->frame_id,
	to_buf->time_stamp,
	to_buf->tv.tv_sec,
	to_buf->tv.tv_usec);
#endif
}

void conv_pym_address(pym_buffer_t *dst_pym_buf,	char *vaddr_base,
	struct frame_info *frameInfo)
{
	int i = 0;
	intptr_t base_vaddr_copy = 0;
	intptr_t base_vaddr_own = 0;

	/* get offset info from frameInfo, and config the pym paddr */
	base_vaddr_copy = (intptr_t)dst_pym_buf->pym[0].addr[0];
	base_vaddr_own = (intptr_t)vaddr_base;

	for (i = 0; i < 24; i++) {
		if (i % 4 == 0) {
			dst_pym_buf->pym[i / 4].paddr[0] =
				frameInfo->spec.ds_y_addr[0];
			dst_pym_buf->pym[i / 4].paddr[1] =
				frameInfo->spec.ds_uv_addr[0];
			dst_pym_buf->pym[i / 4].addr[0] -= base_vaddr_copy;
			dst_pym_buf->pym[i / 4].addr[0] += base_vaddr_own;
			dst_pym_buf->pym[i / 4].addr[1] -= base_vaddr_copy;
			dst_pym_buf->pym[i / 4].addr[1] += base_vaddr_own;
		} else if (dst_pym_buf->pym_roi[i / 4][i % 4 - 1].addr[0] != 0) {
			dst_pym_buf->pym_roi[i / 4][i % 4 - 1].paddr[0] =
				frameInfo->spec.ds_y_addr[i];
			dst_pym_buf->pym_roi[i / 4][i % 4 - 1].paddr[1] =
				frameInfo->spec.ds_uv_addr[i];
			dst_pym_buf->pym_roi[i / 4][i % 4 - 1].addr[0] -= base_vaddr_copy;
			dst_pym_buf->pym_roi[i / 4][i % 4 - 1].addr[0] += base_vaddr_own;
			dst_pym_buf->pym_roi[i / 4][i % 4 - 1].addr[1] -= base_vaddr_copy;
			dst_pym_buf->pym_roi[i / 4][i % 4 - 1].addr[1] += base_vaddr_own;
		}
	}

	for (i = 0; i < 6; i++) {
		if (dst_pym_buf->us[i].addr[0] != 0) {
			dst_pym_buf->us[i].paddr[0] =
				frameInfo->spec.us_y_addr[i];
			dst_pym_buf->us[i].paddr[1] =
				frameInfo->spec.us_uv_addr[i];
			dst_pym_buf->us[i].addr[0] -= base_vaddr_copy;
			dst_pym_buf->us[i].addr[0] += base_vaddr_own;
			dst_pym_buf->us[i].addr[1] -= base_vaddr_copy;
			dst_pym_buf->us[i].addr[1] += base_vaddr_own;
		}
	}
}

void * frame_info_to_oth_idx(buffer_mgr_t *mgr, struct frame_info *info)
{
	int self_index;
	buf_node_t *vio_buf_node;
	pym_buf_node_t *pym_buf_node;
	int buffer_index;
	int planeIndex, planeCount;
	char *ionAddr = NULL;
	int size;
	void *buf_node = NULL;
	buffer_state_e state;

	buffer_index = info->bufferindex;
	if (mgr->buffer_type == HB_VIO_PYM_DATA) {
		pym_buf_node = (pym_buf_node_t *)(mgr->buf_nodes) + buffer_index;
		self_index = pym_buf_node->pym_buf.pym_img_info.buf_index;
		if (self_index == -1) {	/* init */
			planeCount = pym_buf_node->pym_buf.pym_img_info.planeCount;
			for (planeIndex = 0; planeIndex < planeCount; planeIndex++) {
				size = pym_buf_node->pym_buf.pym_img_info.size[planeIndex];
				ionAddr = (char *)mmap(NULL, size,
					PROT_READ | PROT_WRITE, MAP_SHARED,
					mgr->dev_fd,
					(planeCount * buffer_index + planeIndex) << PAGE_SHITF);
				if (ionAddr == MAP_FAILED) {
					vio_err("mmap err,frameind%d planeindex%d",
						buffer_index, planeIndex);
					return NULL;
				}
				vio_dbg("mmap addr %lx,frameind%d planeindex%d",
						(uintptr_t)ionAddr, buffer_index, planeIndex);
				conv_pym_address(&pym_buf_node->pym_buf, ionAddr, info);
			}
			pym_buf_node->pym_buf.pym_img_info.buf_index = buffer_index;
		} else if (self_index == buffer_index) {
			state = pym_buf_node->pym_buf.pym_img_info.state;
			if (state != BUFFER_INVALID) {
				pym_buf_node = (pym_buf_node_t *)find_pop_buffer(mgr,
					state, buffer_index, MGR_NO_LOCK);
				if (pym_buf_node) {
					vio_dbg("pop buf index %d frameid %d from state %d",
						buffer_index,
						pym_buf_node->pym_buf.pym_img_info.frame_id, state);
				} else {
					vio_dbg("buf index %d frameid %d from state %d, but not in list",
						buffer_index,
						pym_buf_node->pym_buf.pym_img_info.frame_id, state);
					state = BUFFER_INVALID;
				}
			}
		} else {
			pym_buf_node = NULL;
			vio_err("findex %d and self_index %d err.",
				buffer_index, self_index);
		}
		buf_node = pym_buf_node;
	} else {
		vio_buf_node = (buf_node_t *)(mgr->buf_nodes) + buffer_index;
		self_index = vio_buf_node->vio_buf.img_info.buf_index;
		if (self_index == -1) {	/* init */
			planeCount = vio_buf_node->vio_buf.img_info.planeCount;
			for (planeIndex = 0; planeIndex < planeCount;
				planeIndex++) {
				vio_buf_node->vio_buf.img_addr.paddr[planeIndex]
					= info->addr[planeIndex];
				size = vio_buf_node->vio_buf.img_info.size[planeIndex];

				ionAddr = (char *)mmap(NULL, size,
					PROT_READ | PROT_WRITE, MAP_SHARED,
					mgr->dev_fd,
					(planeCount * buffer_index + planeIndex) << PAGE_SHITF);
				if (ionAddr == MAP_FAILED) {
					vio_err("mmap err,frameind%d planeindex%d",
						buffer_index, planeIndex);
					return NULL;
				}
				vio_buf_node->vio_buf.img_addr.addr[planeIndex]
						= ionAddr;
				vio_dbg("mmap addr %lx,frameind%d planeindex%d",
						(uintptr_t)ionAddr, buffer_index, planeIndex);
			}
			vio_buf_node->vio_buf.img_info.buf_index = buffer_index;
		} else if (self_index == buffer_index) {
			state = vio_buf_node->vio_buf.img_info.state;
			if (state != BUFFER_INVALID) {
				vio_buf_node = (buf_node_t *)find_pop_buffer(mgr,
					state, buffer_index, MGR_NO_LOCK);
				if (vio_buf_node) {
					vio_dbg("pop buf index %d frameid %d from state %d",
						buffer_index,
						vio_buf_node->vio_buf.img_info.frame_id, state);
				} else {
					vio_dbg("buf index %d frameid %d from state %d, but not in list",
						buffer_index,
						vio_buf_node->vio_buf.img_info.frame_id, state);
					state = BUFFER_INVALID;
				}
			}
		} else {
			vio_buf_node = NULL;
			vio_err("findex %d and self_index %d err.",
				buffer_index, self_index);
		}
		buf_node = vio_buf_node;
	}

	return buf_node;
}
