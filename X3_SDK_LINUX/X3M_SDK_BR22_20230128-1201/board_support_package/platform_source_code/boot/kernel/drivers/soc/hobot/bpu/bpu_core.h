/*
 * Copyright (C) 2019 Horizon Robotics
 *
 * Zhang Guoying <guoying.zhang@horizon.ai>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 *
 */
#ifndef __BPU_CORE_H__
#define __BPU_CORE_H__
#include <linux/interrupt.h>
#if defined(CONFIG_PM_DEVFREQ) && defined(CONFIG_DEVFREQ_THERMAL)
#include <linux/devfreq.h>
#endif
#ifdef CONFIG_HOBOT_XJ3
#include <linux/pm_qos.h>
#endif
#include "bpu.h"
#include "bpu_prio.h"
#include "hw_io.h"

enum bpu_core_status_cmd {
	/* get if bpu busy, 1:busy; 0:free*/
	BUSY_STATE = 0,
	/*
	 * get bpu work or not_work(dead or hung)
	 * return 1: working; 0: not work
	 */
	WORK_STATE,
	/* To update some val for state check */
	UPDATE_STATE,
	/* To get bpu core type(as pe type)*/
	TYPE_STATE,
	STATUS_CMD_MAX,
};

enum core_pe_type {
    CORE_TYPE_UNKNOWN,
    CORE_TYPE_4PE,
    CORE_TYPE_1PE,
    CORE_TYPE_2PE,
    CORE_TYPE_ANY,
    CORE_TYPE_INVALID,
};

struct bpu_core_hw_ops;

#if defined(CONFIG_PM_DEVFREQ) && defined(CONFIG_DEVFREQ_THERMAL)
struct bpu_core_dvfs {
	struct devfreq *devfreq;
	struct thermal_cooling_device *cooling;
	struct devfreq_dev_profile profile;
	uint64_t rate;
	uint64_t volt;
	/* store freq level num */
	uint32_t level_num;
};
#endif

struct bpu_hw_fc {
	uint8_t data[FC_SIZE];
};

struct bpu_core {
	struct list_head node;
	struct device *dev;
	struct bpu *host;
	struct miscdevice miscdev;
	atomic_t open_counter;

	atomic_t hw_id_counter[BPU_PRIO_NUM];

	/* use for bpu core pending */
	atomic_t pend_flag;

	struct bpu_prio *prio_sched;
	/*
	 * limit the fc number which write to
	 * hw fifo at the same time, which can
	 * influence prio sched max time.
	 */
	int32_t fc_buf_limit;

	uint32_t irq;
	void __iomem *base;
	/*
	 * some platform need other place to ctrl bpu,
	 * like pmu
	 */
	void __iomem *reserved_base;

	/*
	 * use to store bpu last done id which not
	 * report, if reported, the value set to 0
	 * HIGH 32bit store error status
	 */
	uint64_t done_hw_id;

	/*
	 * which store bpu read fc base
	 * alloc when core enable and
	 * free when core disable
	 * number for diff level fc fifo
	 */
	struct bpu_hw_fc *fc_base[BPU_PRIO_NUM];
	dma_addr_t fc_base_addr[BPU_PRIO_NUM];

	struct regulator *regulator;
	struct clk *aclk;
	struct clk *mclk;
	struct reset_control *rst;

	uint64_t buffered_time[BPU_PRIO_NUM];
	DECLARE_KFIFO_PTR(run_fc_fifo[BPU_PRIO_NUM], struct bpu_fc);
	DECLARE_KFIFO_PTR(done_fc_fifo, struct bpu_fc);

	struct mutex mutex_lock;
	spinlock_t spin_lock;

	/* list to store user */
	struct list_head user_list;

	struct bpu_core_hw_ops *hw_ops;
	int32_t index;

#if defined(CONFIG_PM_DEVFREQ) && defined(CONFIG_DEVFREQ_THERMAL)
	struct bpu_core_dvfs *dvfs;
#else
	void *dvfs;
#endif
#ifdef CONFIG_HOBOT_XJ3
	struct pm_qos_request pm_qos_req;
#endif
	/* bpu core ctrl */
	int32_t running_task_num;
	struct completion no_task_comp;
	/* > 0; auto change by governor; <=0: manual levle, 0 is highest */
	int32_t power_level;

	/*
	 * if hotplug = 1, powered off core's
	 * task will sched to other core
	 */
	uint16_t hotplug;
	uint16_t hw_enabled;

	struct tasklet_struct tasklet;
	/* the flowing for statistics */
	struct timeval last_done_point;
	/* run time in statistical period */
	struct timeval p_start_point;

	uint64_t p_run_time;
	/* running ratio */
	uint32_t ratio;

	uint64_t reserved[2];
	struct notifier_block bpu_pm_notifier;
};

struct bpu_core_hw_ops {
	int32_t (*enable)(struct bpu_core *);
	int32_t (*disable)(struct bpu_core *);
	int32_t (*reset)(struct bpu_core *);
	int32_t (*set_clk)(const struct bpu_core *, uint64_t);
	int32_t (*set_volt)(const struct bpu_core *, int32_t);
	/*
	 * write real fc to hw, return > 0: actual write fc num
	 * param include the offset pos in the bpu_fc raw slices
	 */
	int32_t (*write_fc)(const struct bpu_core *, struct bpu_fc *fc, uint32_t);
	/* get the fc process return */
	int32_t (*read_fc)(const struct bpu_core *, uint32_t *, uint32_t *);

	/* get bpu hw core running status */
	int32_t (*status)(struct bpu_core *, uint32_t);

	/* debug info for hw info */
	int32_t (*debug)(const struct bpu_core *, int32_t);
};

#endif
