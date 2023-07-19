/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#define pr_fmt(fmt) "vio_framemgr: " fmt
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <asm/cacheflush.h>
#include <asm/pgtable.h>
#include <linux/firmware.h>

#include <linux/bug.h>

#include "vio_framemgr.h"
#include "vio_group_api.h"
// PRQA S 0497,3238,2810,2880,2992,2996 ++
int frame_fcount(struct vio_frame *frame, void *data)
{
	return frame->fcount - (u32)(ulong)data;
}
EXPORT_SYMBOL(frame_fcount);

int put_frame(struct vio_framemgr *this, struct vio_frame *frame,
			enum vio_frame_state state)
{
	if (state == FS_INVALID)
		return -EINVAL;
	if (!frame) {
		vio_err("invalid frame");
		return -EFAULT;
	}
	frame->state = state;
	list_add_tail(&frame->list, &this->queued_list[state]);
	this->queued_count[state]++;
#ifdef TRACE_FRAME
	print_frame_queue(this, state);
#endif
	return 0;
}
EXPORT_SYMBOL(put_frame);

struct vio_frame *get_frame(struct vio_framemgr *this,
			enum vio_frame_state state)
{
	struct vio_frame *frame;
	if (state == FS_INVALID)
		return NULL;
	if (!this->queued_count[state])
		return NULL;
	frame = list_first_entry(&this->queued_list[state], struct vio_frame, list);
	list_del(&frame->list);
	this->queued_count[state]--;
	frame->state = FS_INVALID;
	return frame;
}
EXPORT_SYMBOL(get_frame);

int trans_frame(struct vio_framemgr *this, struct vio_frame *frame,
			enum vio_frame_state state)
{
	if (!frame) {
		vio_err("invalid frame");
		return -EFAULT;
	}
	if ((frame->state == FS_INVALID) || (state == FS_INVALID))
		return -EINVAL;
	if (!this->queued_count[frame->state]) {
		vio_err("%s frame queue is empty (0x%08x)", frame_state_name[frame->state],
							this->id);
		return -EINVAL;
	}

	list_del(&frame->list);
	this->queued_count[frame->state]--;
	//if (state == FS_PROCESS && (!(this->id & FRAMEMGR_ID_HW)))
	//	frame->bak_flag = frame->out_flag;
	return put_frame(this, frame, state);
}
EXPORT_SYMBOL(trans_frame);

struct vio_frame *peek_frame(struct vio_framemgr *this,
			enum vio_frame_state state)
{
	if (state == FS_INVALID)
		return NULL;
	if (!this->queued_count[state])
		return NULL;
	return list_first_entry(&this->queued_list[state], struct vio_frame, list);
}
EXPORT_SYMBOL(peek_frame);

struct vio_frame *peek_frame_tail(struct vio_framemgr *this,
			enum vio_frame_state state)
{
	if (state == FS_INVALID)
		return NULL;
	if (!this->queued_count[state])
		return NULL;
	return list_last_entry(&this->queued_list[state], struct vio_frame, list);
}
EXPORT_SYMBOL(peek_frame_tail);

struct vio_frame *find_frame(struct vio_framemgr *this,
			     enum vio_frame_state state,
			     int (*fn)(struct vio_frame *, void *), void *data)
{
	struct vio_frame *frame;
	if (state == FS_INVALID)
		return NULL;
	if (!this->queued_count[state]) {
		vio_err("%s frame queue is empty (0x%08x)", frame_state_name[state],
							this->id);
		return NULL;
	}
	list_for_each_entry(frame, &this->queued_list[state], list) {
		if (!fn(frame, data))
			return frame;
	}
	return NULL;
}

void print_frame_queue(struct vio_framemgr *this,
			enum vio_frame_state state)
{
	struct vio_frame *frame, *temp;
	if (!(TRACE_ID & this->id))
			return;
	vio_info("[FRM] %s(0x%08x, %d) :", frame_state_name[state],
					this->id, this->queued_count[state]);
	list_for_each_entry_safe(frame, temp, &this->queued_list[state], list)
		vio_cont("%d[%d]->", frame->index, frame->fcount);

	vio_cont("X\n");
}

void print_frame_info_queue(struct vio_framemgr *this,
			enum vio_frame_state state)
{
	unsigned long long when[MAX_FRAME_INFO];
	unsigned long usec[MAX_FRAME_INFO];
	struct vio_frame *frame, *temp;
	if (!(TRACE_ID & this->id))
			return;
	vio_info("[FRM_INFO] %s(0x%08x, %d) :", hw_frame_state_name[state],
					this->id, this->queued_count[state]);
	list_for_each_entry_safe(frame, temp, &this->queued_list[state], list) {
		when[INFO_FRAME_START]    = frame->frame_info[INFO_FRAME_START].when;
		when[INFO_CONFIG_LOCK]    = frame->frame_info[INFO_CONFIG_LOCK].when;
		when[INFO_FRAME_END_PROC] = frame->frame_info[INFO_FRAME_END_PROC].when;
		usec[INFO_FRAME_START]    = do_div(when[INFO_FRAME_START], NSEC_PER_SEC);
		usec[INFO_CONFIG_LOCK]    = do_div(when[INFO_CONFIG_LOCK], NSEC_PER_SEC);
		usec[INFO_FRAME_END_PROC] = do_div(when[INFO_FRAME_END_PROC], NSEC_PER_SEC);
		vio_cont("%d[%d]([%5lu.%06lu],[%5lu.%06lu],[%5lu.%06lu])->",
			frame->index, frame->fcount,
			(unsigned long)when[INFO_FRAME_START],    usec[INFO_FRAME_START] / NSEC_PER_USEC,
			(unsigned long)when[INFO_CONFIG_LOCK],    usec[INFO_CONFIG_LOCK] / NSEC_PER_USEC,
			(unsigned long)when[INFO_FRAME_END_PROC], usec[INFO_FRAME_END_PROC] / NSEC_PER_USEC);
	}
	vio_cont("X\n");
}
EXPORT_SYMBOL(print_frame_info_queue);

void frame_work_function(struct kthread_work *work)
{
	struct vio_frame *frame;
	struct vio_group *group;
	struct vio_group *leader;
	struct vio_group_task *gtask;
	bool try_rdown = false;
	int ret = 0;

	frame = container_of(work, struct vio_frame, work);
	leader = (struct vio_group *) frame->data;
	gtask = leader->gtask;

	if (gtask == NULL) {
		vio_info("WARN: group leader(%d) task is NULL\n",
				leader->id);
		return;
	}

	set_bit(VIO_GTASK_SHOT, &gtask->state);
	atomic_dec(&leader->rcount);
	vio_dbg("[S%d][G%d]%s #0\n", leader->instance, leader->id, __func__); /*PRQA S ALL*/

	if (unlikely(test_bit(VIO_GTASK_REQUEST_STOP, &gtask->state))) {
		vio_err(" cancel by gstop0");
		goto p_err_ignore;
	}

	ret = down_interruptible(&gtask->hw_resource);
	if (ret) {
		vio_err(" down fail(%d)", ret);
		goto p_err_ignore;
	}

	try_rdown = true;

	if (unlikely(test_bit(VIO_GTASK_SHOT_STOP, &gtask->state))) {
		vio_err(" cancel by gstop1");
		goto p_err_ignore;
	}

	group = leader;
	while (group->next) {
		group = group->next;
		vio_dbg("[S%d][G%d]%s #1\n", leader->instance, leader->id, __func__); /*PRQA S ALL*/
		if(group->frame_work)
			group->frame_work(group);
	}
	vio_dbg("[S%d][G%d]%s #2\n", leader->instance, leader->id, __func__); /*PRQA S ALL*/

	leader->frame_work(leader);
	clear_bit(VIO_GTASK_SHOT, &gtask->state);

	return;

p_err_ignore:
	if (try_rdown)
		up(&gtask->hw_resource);

	clear_bit(VIO_GTASK_SHOT, &gtask->state);

	return;
}
EXPORT_SYMBOL(frame_work_function);

/**
 * @brief: frame work of leader
 * call the frame work of next multi modules,
 * then call the frame work of its own
 * @param work: frame work
 */
void frame_work_function_mp(struct kthread_work *work)
{
	struct vio_work *vwork;
	struct vio_group *group;
	struct vio_group *leader;
	struct vio_group_task *gtask;
	bool try_rdown = false;
	int ret = 0;

	vwork = container_of(work, struct vio_work, work);
	leader = vwork->group;
	gtask = leader->gtask;

	if (gtask == NULL) {
		vio_info("WARN: group leader(%d) task is NULL\n",
				leader->id);
		return;
	}

	set_bit(VIO_GTASK_SHOT, &gtask->state);
	atomic_dec(&leader->rcount);
	vio_dbg("[S%d][G%d]%s #0\n", leader->instance, leader->id, __func__); /*PRQA S ALL*/

	if (unlikely(test_bit(VIO_GTASK_REQUEST_STOP, &gtask->state))) {
		vio_err(" cancel by gstop0");
		goto p_err_ignore;
	}

	ret = down_interruptible(&gtask->hw_resource);
	if (ret) {
		vio_err(" down fail(%d)", ret);
		goto p_err_ignore;
	}

	try_rdown = true;

	if (unlikely(test_bit(VIO_GTASK_SHOT_STOP, &gtask->state))) {
		vio_err(" cancel by gstop1");
		goto p_err_ignore;
	}


	group = leader;
	while (group->next) {
		group = group->next;
		vio_dbg("[S%d][G%d]%s #1\n", leader->instance, leader->id, __func__); /*PRQA S ALL*/
		if(group->frame_work)
			group->frame_work(group);
	}
	vio_dbg("[S%d][G%d]%s #2\n", leader->instance, leader->id, __func__); /*PRQA S ALL*/
	leader->frame_work(leader);
	clear_bit(VIO_GTASK_SHOT, &gtask->state);

	return;

p_err_ignore:
	if (try_rdown)
		up(&gtask->hw_resource);

	clear_bit(VIO_GTASK_SHOT, &gtask->state);

	return;
}
EXPORT_SYMBOL(frame_work_function_mp);

int frame_manager_open(struct vio_framemgr *this, u32 buffers)
{
	u32 i;
	unsigned long flag;
	spin_lock_init(&this->slock);
	/*
	 * We already have frames allocated, so we should free them first.
	 * reqbufs(n) could be called multiple times from userspace after
	 * each video node was opened.
	 */
	if (this->frames)
		kfree(this->frames);
	this->frames = kzalloc(sizeof(struct vio_frame) * buffers, GFP_KERNEL);
	if (!this->frames) {
		vio_err("failed to allocate frames");
		return -ENOMEM;
	}
	spin_lock_irqsave(&this->slock, flag);
	this->num_frames = buffers;
	for (i = 0; i < NR_FRAME_STATE; i++) {
		this->queued_count[i] = 0;
		INIT_LIST_HEAD(&this->queued_list[i]);
	}
	for (i = 0; i < buffers; ++i) {
		this->frames[i].index = i;
		put_frame(this, &this->frames[i], FS_FREE);
		// kthread_init_work(&this->frames[i].work, frame_work_function);
	}

	spin_unlock_irqrestore(&this->slock, flag);
	return 0;
}
EXPORT_SYMBOL(frame_manager_open);

void frame_work_init(struct kthread_work *work)
{
	kthread_init_work(work, frame_work_function_mp);
}
EXPORT_SYMBOL(frame_work_init);

int frame_manager_close(struct vio_framemgr *this)
{
	u32 i;
	unsigned long flag;
	spin_lock_irqsave(&this->slock, flag);
	if (this->frames) {
		kfree(this->frames);
		this->frames = NULL;
	}
	this->num_frames = 0;
	for (i = 0; i < NR_FRAME_STATE; i++) {
		this->queued_count[i] = 0;
		INIT_LIST_HEAD(&this->queued_list[i]);
	}
	spin_unlock_irqrestore(&this->slock, flag);
	return 0;
}
EXPORT_SYMBOL(frame_manager_close);

int frame_manager_flush(struct vio_framemgr *this)
{
	unsigned long flag;
	struct vio_frame *frame, *temp;
	enum vio_frame_state i;
	spin_lock_irqsave(&this->slock, flag);
	for (i = FS_REQUEST; i < FS_INVALID; i++) {
		list_for_each_entry_safe(frame, temp, &this->queued_list[i], list)
			trans_frame(this, frame, FS_FREE);
	}
	spin_unlock_irqrestore(&this->slock, flag);
	BUG_ON(this->queued_count[FS_FREE] != this->num_frames);
	return 0;
}
EXPORT_SYMBOL(frame_manager_flush);

/*
 * init framemgr
 */
int frame_manager_init_mp(struct vio_framemgr *this)
{
	if (this == NULL) {
		vio_err("%s framemgr is null.\n", __func__);
		return -EFAULT;
	}
	spin_lock_init(&this->slock);

	return 0;
}
EXPORT_SYMBOL(frame_manager_init_mp);

/*
 * Open frame manager for multi-process scenario
 */
int frame_manager_open_mp(struct vio_framemgr *this, u32 buffers,
	u32 *index_start)
{
	u32 i, j;
	unsigned long flag;
	struct mp_vio_frame *frames = NULL;
	struct mp_vio_frame *frame;
	u32 ind_fst;

	frames = vmalloc(sizeof(struct mp_vio_frame) * buffers);
	if (!frames) {
		vio_err("failed to allocate frames");
		return -ENOMEM;
	}
	memset(frames, 0, sizeof(struct mp_vio_frame) * buffers);
	spin_lock_irqsave(&this->slock, flag);
	if ((this->num_frames + buffers) > VIO_MP_MAX_FRAMES) {
		spin_unlock_irqrestore(&this->slock, flag);
		vio_err("%s:apply for too much frames,max 128.", __func__);
		vfree(frames);
		return -ENOMEM;
	}

	// find the first continuous index for buffers
	ind_fst = 0;
	for (i = 0; i < VIO_MP_MAX_FRAMES; i++) {
		if (this->index_state[i] == FRAME_IND_FREE &&
				((i + buffers) <= VIO_MP_MAX_FRAMES)) {
			for (j = i; j < (i + buffers); j++) {
				/* coverity[overrun-local] */
				if (this->index_state[j] != FRAME_IND_FREE)
					break;
			}
			if (j == (i + buffers)) {
				ind_fst = i;
				break;
			}
		}
	}
	if (i == VIO_MP_MAX_FRAMES) {
		spin_unlock_irqrestore(&this->slock, flag);
		vio_err("%s no enough AVALABLE frames.", __func__);
		for (i = 0; i < VIO_MP_MAX_FRAMES; i++) {
			if ((i&0xF) == 0)
				vio_dbg("\nindex state %d-%d:", i, i+15); /*PRQA S ALL*/
			vio_dbg("%d ", this->index_state[i]); /*PRQA S ALL*/
		}
		vfree(frames);
		return -ENOMEM;
	}

	// note: this only give the mp_vio_frame->common_frame addr
	// to frames_mp
	for (i = 0; i < buffers; i++)
		this->frames_mp[i + ind_fst] = &frames[i].common_frame;

	// if framemgr not initialized
	// init all the FS states queues
	if (this->state != FRAMEMGR_INIT) {
		for (i = 0; i < NR_FRAME_STATE; i++) {
			this->queued_count[i] = 0;
			INIT_LIST_HEAD(&this->queued_list[i]);
		}
	}

	// all allocated frames push to FREE queue
	// work->func will be frame_work_function, it will be
	// changed to frame_work_function_mp in ipu_video_reqbufs again
	for (i = ind_fst; i < (buffers + ind_fst); ++i) {
		this->frames_mp[i]->index = i;
		put_frame(this, this->frames_mp[i], FS_FREE);
		kthread_init_work(&this->frames_mp[i]->work,
			frame_work_function);

		// frame index now is allocated and used
		this->index_state[i] = FRAME_IND_USING;

		// for multi process share
		// frame will record the first_index and index range
		// and ion addr now are not allocated, so set to 0
		frame = (struct mp_vio_frame *)this->frames_mp[i];
		frame->first_indx = ind_fst;
		frame->ion_bufffer_num = buffers;
		frame->addr[0] = 0; frame->addr[1] = 0; frame->addr[2] = 0;
		frame->ion_handle[0] = NULL;
		frame->ion_handle[1] = NULL;
		frame->ion_handle[2] = NULL;
	}

	// first index will be out to caller
	*index_start = ind_fst;
	this->num_frames += buffers;
	this->state = FRAMEMGR_INIT;
	if (this->max_index < (ind_fst + buffers))
		this->max_index = ind_fst + buffers;
	spin_unlock_irqrestore(&this->slock, flag);

	vio_dbg("%s first index %d, num %d, num_frames %d max index %d.vmalloc start:%p", /*PRQA S ALL*/
		__func__, ind_fst, buffers, this->num_frames, this->max_index, frames);
	/* coverity[leaked_storage] */
	return 0;
}
EXPORT_SYMBOL(frame_manager_open_mp);

#if 0
int frame_manager_close_mp(struct vio_framemgr *this,
	u32 index_start, u32 buffers, u32 ctx_index)
{
	u32 i;
	unsigned long flag;
	struct vio_frame *frame;
	struct vio_frame *free_addr;
	u32 tmp_num;

	if ((index_start + buffers) > VIO_MP_MAX_FRAMES) {
		vio_err("%s invalid index", __func__);
		return -EFAULT;
	}
	if (this->frames_mp[index_start] == NULL) {
		vio_err("%s start index is null", __func__);
		return -EFAULT;
	}
	if (buffers == 0) {
		vio_err("%s buffer number is ", __func__);
		return -EFAULT;
	}

	spin_lock_irqsave(&this->slock, flag);
	if ((this->index_state[index_start] == FRAME_IND_STREAMOFF)
		|| (this->index_state[index_start] == FRAME_IND_USING)) {
		free_addr = this->frames_mp[index_start];
	} else {
		vio_err("%s start index %d state %d err", __func__, index_start,
			this->index_state[index_start]);
		goto err_unlock;
	}
	for (i = index_start; i < (index_start + buffers); i++) {
		if ((this->index_state[i] != FRAME_IND_STREAMOFF)
			&& (this->index_state[i] != FRAME_IND_USING)) {
			vio_err("%s:index %d state %d error", __func__, i,
				this->index_state[i]);
			goto err_unlock;
		}
		frame = this->frames_mp[i];
		if (!frame) {
			vio_err("%s:frame%d null", __func__, i);
			continue;
		}
		if ((frame->state >= FS_INVALID)) {
			vio_err("%s:frame%d state%d err", __func__, i,
				frame->state);
			continue;
		}
		if (!this->queued_count[frame->state]) {
			vio_err("%s frame queue is empty (0x%08x)",
				frame_state_name[frame->state], this->id);
			continue;
		}
		list_del(&frame->list);
		this->queued_count[frame->state]--;
		this->index_state[i] = FRAME_IND_FREE;
		this->frames_mp[i] = NULL;
	}
	this->num_frames -= buffers;
	this->ctx_mask &= ~(1 << ctx_index);
	/* clear the frame mask bit of this ctx*/
	for (i = 0; i < VIO_MP_MAX_FRAMES; i++) {
		if ((this->index_state[i] != FRAME_IND_USING)
			&& (this->index_state[i] != FRAME_IND_STREAMOFF))
			continue;
		if ((this->dispatch_mask[i] >> ctx_index) & 0x01)
			this->dispatch_mask[i] &= ~(1 << ctx_index);
	}
	vfree(free_addr);
	if (this->max_index == (index_start + buffers)) {
		tmp_num = 0;
		for (i = 0; i < VIO_MP_MAX_FRAMES; i++) {
			if ((this->index_state[i] == FRAME_IND_USING)
				|| (this->index_state[i] == FRAME_IND_STREAMOFF))
				tmp_num++;
			if (tmp_num == this->num_frames)
				break;
		}
		this->max_index = i + 1;
	}
	vio_dbg("%s first index %d, num %d, this->num_frames %d max index %d.",
		__func__, index_start, buffers, this->num_frames, this->max_index);
	spin_unlock_irqrestore(&this->slock, flag);
	return 0;

err_unlock:
	spin_unlock_irqrestore(&this->slock, flag);
	return -EFAULT;
}
EXPORT_SYMBOL(frame_manager_close_mp);
#endif

/*
 * Wait for frame state to become UESD or FREE
 * Wait for frame to be qbuf by other process, which means other process free
 * this frame
 */
int frame_manager_flush_mp(struct vio_framemgr *this,
	u32 index_start, u32 buffers, u8 proc_id)
{
	unsigned long flag;
	struct vio_frame *frame;
	u32 i;
	u32 used_free_cnt = 0;

	if ((index_start + buffers) > VIO_MP_MAX_FRAMES) {
		vio_err("invalid index when flush frame manager.");
		return -EFAULT;
	}
	if (this->frames_mp[index_start] == NULL) {
		vio_err("%s start index is null", __func__);
		return -EFAULT;
	}
	if (buffers == 0) {
		vio_err("%s buffer number is ", __func__);
		return -EFAULT;
	}

	spin_lock_irqsave(&this->slock, flag);
	/* to USED or FREE*/
	for (i = index_start; i < (buffers + index_start); i++) {
		if ((this->index_state[i] != FRAME_IND_STREAMOFF)
			&& (this->index_state[i] != FRAME_IND_USING)) {
			vio_err("%s:index %d state %d error", __func__, i,
				this->index_state[i]);
			goto err_unlock;
		}
		frame = this->frames_mp[i];
		if (!frame) {
			vio_err("%s:frame%d null, this should never be", __func__, i);
			goto err_unlock;
		}
		if ((frame->state == FS_USED)
			|| (frame->state == FS_FREE))
			used_free_cnt++;
	}
	vio_dbg("%s:%d buffers in USED or FREE, %d not.", /*PRQA S ALL*/
		__func__, used_free_cnt, buffers - used_free_cnt);
	spin_unlock_irqrestore(&this->slock, flag);
	return 0;

err_unlock:
	spin_unlock_irqrestore(&this->slock, flag);
	return -EFAULT;;
}
EXPORT_SYMBOL(frame_manager_flush_mp);

/*
 * Set the streaming off mask of the frame, clear ctx_mask
 */
 #if 0
int frame_manager_flush_mp_prepare(struct vio_framemgr *this,
	u32 index_start, u32 buffers, u8 proc_id)
{
	unsigned long flag;
	int i;

	if ((index_start + buffers) > VIO_MP_MAX_FRAMES) {
		vio_err("invalid index when flush frame manager.");
		return -EFAULT;
	}
	if (buffers == 0) {
		vio_err("%s buffer number is ", __func__);
		return -EFAULT;
	}

	#if 0
	struct vio_frame *frame;
	for (i = index_start; i < (buffers + index_start); i++) {
		frame = this->frames_mp[i];
		vio_dbg("%s%d:index%d,state%d,mask%x.", __func__, __LINE__,
			i, this->frames_mp[i]->state,
			this->dispatch_mask[i]);
	}
	#endif
	spin_lock_irqsave(&this->slock, flag);
	for (i = index_start; i < (buffers + index_start); i++) {
		this->index_state[i] = FRAME_IND_STREAMOFF;
	}
	this->ctx_mask &= ~(1 << proc_id);
	/* clear the frame mask bit of this ctx*/
	for (i = 0; i < VIO_MP_MAX_FRAMES; i++) {
		if ((this->index_state[i] != FRAME_IND_USING)
			&& (this->index_state[i] != FRAME_IND_STREAMOFF))
			continue;
		this->dispatch_mask[i] &= ~(1 << proc_id);
	}
	spin_unlock_irqrestore(&this->slock, flag);

	vio_dbg("%s proc %d:", __func__, proc_id); /*PRQA S ALL*/
	for (i = 0; i < VIO_MP_MAX_FRAMES; i++) {
		if ((this->index_state[i] != FRAME_IND_USING)
			&& (this->index_state[i] != FRAME_IND_STREAMOFF))
			continue;
		vio_dbg("frm%d mask 0x%x", i, this->dispatch_mask[i]); /*PRQA S ALL*/
	}

	return 0;
}
EXPORT_SYMBOL(frame_manager_flush_mp_prepare); /*PRQA S ALL*/
#endif

void frame_manager_print_queues(struct vio_framemgr *this)
{
	int i;
	for (i = 0; i < NR_FRAME_STATE; i++)
		print_frame_queue(this, (enum vio_frame_state)i);
}
EXPORT_SYMBOL(frame_manager_print_queues);

void frame_manager_print_info_queues(struct vio_framemgr *this)
{
	int i;
	for (i = 0; i < NR_FRAME_STATE; i++)
		print_frame_info_queue(this, (enum vio_frame_state)i);
}
EXPORT_SYMBOL(frame_manager_print_info_queues);
// PRQA S --
