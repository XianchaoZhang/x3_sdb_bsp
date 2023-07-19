/*
 * Horizon Robotics
 *
 *  Copyright (C) 2020 Horizon Robotics Inc.
 *  All rights reserved.
 *  Author: leye.wang<leye.wang@horizon.ai>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <asm-generic/errno-base.h>
#include <linux/init.h> 	/* PRQA S ALL */
#include <linux/module.h> 	/* PRQA S ALL */
#include <net/sock.h> 		/* PRQA S ALL */
#include <net/netlink.h> 	/* PRQA S ALL */
#include <linux/jiffies.h> 	/* PRQA S ALL */
#include <linux/printk.h> 	/* PRQA S ALL */

#ifdef SYSFS_DEBUG
#include <linux/sysfs.h> 	/* PRQA S ALL */
#include <linux/fs.h> 		/* PRQA S ALL */
#include <linux/device.h> 	/* PRQA S ALL */
#endif

#include "diag_drv.h"

// #define DIAG_DEBUG

#undef PDEBUG
#ifdef DIAG_DEBUG
#define PDEBUG(fmt, ...)		printk(KERN_ERR "[Diag]:"fmt, ##__VA_ARGS__)
#else
#define PDEBUG(fmt, ...)
#endif

#define EVENT_ELEMENT_ALL	100

#define USER_PORT	100

/* used to store all resources */
struct diag *g_diag_info = NULL;
static int32_t diag_had_init;
static struct diag_inject_ops {
	bool(*module_inject_registered) (uint16_t module_id, uint16_t event);
	int(*module_inject_val_get) (uint16_t module_id, uint16_t event,
					uint32_t *val);
} hb_diag_inject;

int32_t diag_inject_ops_register(
				bool (*module_inject_registered)(uint16_t, uint16_t),
				int (*module_inject_val_get)(uint16_t, uint16_t, uint32_t *))
{
	if (module_inject_registered == NULL || module_inject_val_get == NULL) {
		pr_err("%s: diag_inject_ops register failed!\n", __func__);
		return -EINVAL;
	}
	hb_diag_inject.module_inject_registered = module_inject_registered;
	hb_diag_inject.module_inject_val_get = module_inject_val_get;
	return 0;
}
EXPORT_SYMBOL(diag_inject_ops_register);

void diag_inject_ops_unregister()
{
	hb_diag_inject.module_inject_registered = NULL;
	hb_diag_inject.module_inject_val_get = NULL;
	return;
}
EXPORT_SYMBOL(diag_inject_ops_unregister);

int32_t diag_inject_val(uint16_t module_id, uint16_t event, uint32_t *reg_val)
{
	int32_t ret = 0;
	if (hb_diag_inject.module_inject_registered == NULL)
		return -EIO;

	if (!hb_diag_inject.module_inject_registered(module_id, event))
		return -EINVAL;

	ret = hb_diag_inject.module_inject_val_get(module_id, event, reg_val);
	return ret;
}
EXPORT_SYMBOL(diag_inject_val);

/* wether the module_id & event_id is registered or not,
 * if registerd, diag_module and event_id will be returned */
struct diag_module *is_module_event_registered(struct diag_event *event,
		struct diag_event_id **event_id)
{
	struct diag_module *pos;
	struct diag_module *target = NULL;
	struct diag_event_id *tmp_event = NULL;
	unsigned long flags;

	if (g_diag_info == NULL) {
		pr_err("diagnose driver unintialized\n");
		return target;
	} else {
		/* PRQA S 0497, 0602 */

		spin_lock_irqsave(&g_diag_info->module_lock, flags);
		list_for_each_entry(pos, &g_diag_info->module_list, list) {
			if (pos->module_id != event->module_id)
				continue;
			/* module_id has been registerd */
			list_for_each_entry(tmp_event, &pos->events, list) {
				if (tmp_event->id_handle.event_id == event->event_id) {
					/* event_id has been registerd */
					target = pos;
					*event_id = tmp_event;
					spin_unlock_irqrestore(&g_diag_info->module_lock, flags);
					return target;
				}
			}
		}
		spin_unlock_irqrestore(&g_diag_info->module_lock, flags);
	}

	/* module & event_id not registerd */
	*event_id = NULL;
	return target;
}
EXPORT_SYMBOL(is_module_event_registered);

static int32_t is_send_condition_ready(struct diag_event_id *event_id,
		struct diag_event *event)
{
	uint64_t diff;
	int32_t ret = 0;

	/* sanity check */
	if ((event_id == NULL) || (event_id->id_handle.event_id != event->event_id))
		return ret;

	diff = (uint64_t)jiffies_to_msecs(get_jiffies_64()) - event_id->last_snd_time;

	/* TODO need to add spinlock? */
	if ((event_id->last_sta != event->event_sta) ||
			((event_id->last_sta == event->event_sta) &&
			 (event->event_sta == DiagEventStaFail) &&
			 (diff >= event_id->id_handle.min_snd_ms))) {
		event_id->last_snd_time = jiffies_to_msecs(get_jiffies_64());
		event_id->last_sta = event->event_sta;
		ret = 1;
	}

	return ret;
}

static struct diag_module_event * try_to_free_event_element(void)
{
	struct diag_module_event *element = NULL;
	unsigned long flags;

	spin_lock_irqsave(&g_diag_info->prio_lock, flags);
	if (!list_empty(&g_diag_info->low_prio_list)) {
		element = list_first_entry(&g_diag_info->low_prio_list,
					struct diag_module_event, list);
	} else if (!list_empty(&g_diag_info->mid_prio_list)) {
		element = list_first_entry(&g_diag_info->mid_prio_list,
					struct diag_module_event, list);
	} else {
		element = list_first_entry(&g_diag_info->hig_prio_list,
					struct diag_module_event, list);
	}
	list_del(&element->list);
	spin_unlock_irqrestore(&g_diag_info->prio_lock, flags);

	return element;
}

static int32_t module_event_add_to_list(struct diag_event *event)
{
	struct diag_module_event *module_event = NULL;
	unsigned long flags;	/* PRQA S 5209 */

	spin_lock_irqsave(&g_diag_info->empty_spinlock, flags); /* PRQA S ALL */
	if (list_empty(&g_diag_info->empty_list)) { /* PRQA S ALL */
		spin_unlock_irqrestore(&g_diag_info->empty_spinlock, flags);
		if (diag_is_ready()) {
			pr_err("event element is used up,module:%hx event:%hx"
					"sta:%hhx discard\n", event->module_id,
					event->event_id, event->event_sta); /* PRQA S ALL */
			return -1;
		}
		module_event = try_to_free_event_element();
		pr_debug("event element used up,module:%hx event:%hx"
				"sta:%hhx was chosen to discard\n", module_event->module_id,
				module_event->event_id, module_event->event_sta);
	} else {
		/* get a node from empty_list */
		module_event = list_first_entry(&g_diag_info->empty_list, /* PRQA S ALL */
				struct diag_module_event, list);
		list_del(&module_event->list);
		spin_unlock_irqrestore(&g_diag_info->empty_spinlock, flags);
	}

	/* initialize module_event */
	module_event->module_id = event->module_id;
	module_event->event_id = event->event_id;
	module_event->event_sta = event->event_sta;

	if ((event->env_len > 0U) && (event->env_len < ENV_PAYLOAD_SIZE)) {
		memcpy(module_event->payload, event->payload, event->env_len); /* PRQA S ALL */
		module_event->when = event->when;
		module_event->env_len = event->env_len;
		module_event->ifenv = 1;
	} else {
		module_event->ifenv = 0;
	}

	/* insert to priority queue */
	spin_lock_irqsave(&g_diag_info->prio_lock, flags);
	switch (event->event_prio) {
		case (uint8_t)DiagMsgPrioLow:
			list_add_tail(&module_event->list,
					&g_diag_info->low_prio_list);
			spin_unlock_irqrestore(&g_diag_info->prio_lock, flags);
			break;
		case (uint8_t)DiagMsgPrioMid:
			list_add_tail(&module_event->list,
					&g_diag_info->mid_prio_list);
			spin_unlock_irqrestore(&g_diag_info->prio_lock, flags);
			break;
		case (uint8_t)DiagMsgPrioHigh:
			list_add_tail(&module_event->list,
					&g_diag_info->hig_prio_list);
			spin_unlock_irqrestore(&g_diag_info->prio_lock, flags);
			break;
		default: /* PRQA S 2024 */
			spin_unlock_irqrestore(&g_diag_info->prio_lock, flags);
			spin_lock_irqsave(&g_diag_info->empty_spinlock, flags); /* PRQA S ALL */
			list_add_tail(&module_event->list, &g_diag_info->empty_list);
			spin_unlock_irqrestore(&g_diag_info->empty_spinlock, flags);
			pr_err("It's a bug\n"); /* PRQA S ALL */
			return -1;
	}

	return 0;
}

static uint32_t diag_checksum(const struct diag_msg_data *data, uint8_t env)
{
	uint32_t sum = 0;
	uint8_t i;

	if (data != NULL) {
		sum = data->module_id;
		sum += data->event_id;
		sum += data->event_sta;
		if (env == 1) {
			sum += data->when;
			sum += data->env_len;
			for (i = 0; i < data->env_len; i++)
				sum += data->payload[i];
		}
	}

	return sum;
}

static void diag_msg_to_array(uint8_t *data, const struct diag_msg *msg)
{
	int8_t i;
	if ((data == NULL) || (msg == NULL))
		return;

	data[0] = msg->header.msg_type;
	data[1] = (uint8_t)((msg->header.msg_len & (uint16_t)0xFF00) >> 8);
	data[2] = (uint8_t)(msg->header.msg_len & (uint16_t)0xFF);
	data[3] = (uint8_t)((msg->header.checksum & (uint32_t)0xFF000000) >> 24);
	data[4] = (uint8_t)((msg->header.checksum & (uint32_t)0xFF0000) >> 16);
	data[5] = (uint8_t)((msg->header.checksum & (uint32_t)0xFF00) >> 8);
	data[6] = (uint8_t)(msg->header.checksum & (uint32_t)0xFF);
	data[7] = (uint8_t)msg->data.module_id;
	data[8] = (uint8_t)msg->data.event_id;
	data[9] = msg->data.event_sta;
	if (msg->header.msg_type == MSG_WITH_ENV) {
		for (i = 0; i < msg->data.env_len; i++)
			data[10 + i] = msg->data.payload[i];
		data[10 + ENV_PAYLOAD_SIZE] = msg->data.env_len;
		data[11 + ENV_PAYLOAD_SIZE] = msg->data.when;
	}
}

static int32_t netlink_snd_msg(struct diag_module_event *module_event)
{
	struct sk_buff *nl_skb;
	struct nlmsghdr *nlh;
	struct diag_msg msg;
	int32_t ret = 0;
	size_t len = 0;
	uint8_t data[12 + ENV_PAYLOAD_SIZE] = {0};
	/* initialize diag_msg */
	if (module_event->ifenv == 1) {
		msg.data.when = module_event->when;
		msg.header.msg_type = MSG_WITH_ENV;
		msg.data.env_len = module_event->env_len;
		memcpy(msg.data.payload, module_event->payload, module_event->env_len);
	} else {
		msg.header.msg_type = MSG_WITHOUT_ENV;
	}
	msg.header.msg_len = sizeof(struct diag_msg);
	msg.data.module_id = module_event->module_id;
	msg.data.event_id = module_event->event_id;
	msg.data.event_sta = module_event->event_sta;
	msg.header.checksum = diag_checksum(&msg.data, module_event->ifenv);

	len = 12 + ENV_PAYLOAD_SIZE;

	PDEBUG("message send, module: %d, event: %d, status: %d, data_len = %lu\n",
			msg.data.module_id, msg.data.event_id, msg.data.event_sta, len);
	PDEBUG("checksum = %d", msg.header.checksum);

	/* create netlink socket buffer */
	nl_skb = nlmsg_new(len, GFP_ATOMIC);
	if (nl_skb == NULL) {
		pr_err("netlink allocate fail\n");
		return -1;
	}

	/* set netlink header */
	nlh = nlmsg_put(nl_skb, 0, 0, (int32_t)NETLINK_DIAG, (int32_t)len, 0);
	if (nlh == NULL) {
		pr_err("netlink message put fail\n");
		nlmsg_free(nl_skb);
		return -1;
	}

	diag_msg_to_array(data, &msg);

	/* send data */
	memcpy(nlmsg_data(nlh), &data, len); /* PRQA S 1496 */
	//memcpy(nlmsg_data(nlh), &msg, sizeof(struct diag_msg)); /* PRQA S 1496 */
	//mutex_lock(&g_diag_info->netlink_mutex);
#ifdef NETLINK_BROADCAST
	if (!netlink_has_listeners(g_diag_info->netlink_sock, USER_GROUP))
		return -1;

	NETLINK_CB(nl_skb).portid = 0;
	NETLINK_CB(nl_skb).dst_group = USER_GROUP;
	ret = netlink_broadcast(g_diag_info->netlink_sock, nl_skb, 0,
							USER_GROUP, GFP_KERNEL);
	if (ret < 0) {
		if (ret == -3)
			pr_err("netlink broadcast send fail ret: %d\n", ret);
		return -1;
	}
#else
	ret = netlink_unicast(g_diag_info->netlink_sock, nl_skb,
			USER_PORT, MSG_DONTWAIT);
	if (ret < 0) {
		pr_err("netlink unicast send fail: %d\n", ret);
		return -1;
	}
#endif
	//mutex_unlock(&g_diag_info->netlink_mutex);

	return (int32_t)len;
}

static void netlink_recv_msg(struct sk_buff *skb) /* PRQA S ALL */
{
	/* not initialized, may do nothing */
	PDEBUG("not implemented\n");
}

struct netlink_kernel_cfg netlink_cfg = {
	.input = netlink_recv_msg,
};

static int32_t handle_and_move(struct list_head *prio_list)
{
	struct diag_module_event *module_event;
	struct diag_module_event *next;
	unsigned long flags;
	int32_t ret = 0;

	spin_lock_irqsave(&g_diag_info->prio_lock, flags); /* PRQA S 0515, 0602 */
	list_for_each_entry_safe(module_event, next, prio_list, list) { /* PRQA S 0497, 0602, 2471 */
		list_del(&module_event->list);
		spin_unlock_irqrestore(&g_diag_info->prio_lock, flags);

		ret = netlink_snd_msg(module_event);
		if (ret < 0) {
			pr_err("netlink send message fail\n");
		}

		/* insert back to empty_list */
		spin_lock_irqsave(&g_diag_info->empty_spinlock, flags); /* PRQA S 0515, 0602 */
		list_add_tail(&module_event->list, &g_diag_info->empty_list);
		spin_unlock_irqrestore(&g_diag_info->empty_spinlock, flags);

		/* note that new events may tailed to higher prio_list during message send */
		spin_lock_irqsave(&g_diag_info->prio_lock, flags); /* PRQA S 0515, 0602 */
		if (!list_empty(&g_diag_info->hig_prio_list) &&
				(prio_list != &g_diag_info->hig_prio_list)) {
			ret = 1U;
			break;
		}
		if (!list_empty(&g_diag_info->mid_prio_list) &&
				(prio_list == &g_diag_info->low_prio_list)) {
			ret = 2U;
			break;
		}
	}
	spin_unlock_irqrestore(&g_diag_info->prio_lock, flags);

	return ret;
}

static void diag_work_handler(struct work_struct *work) /* PRQA S ALL */
{
	int32_t ret;

	ret = diag_is_ready();
	if ((0 == ret) || (0 == diag_had_init)) {
		return;
	}

again1:
	/* process high priority list */
	handle_and_move(&g_diag_info->hig_prio_list);

again2:
	/* process middle priority list */
	ret = handle_and_move(&g_diag_info->mid_prio_list);
	if (ret == 1U)
		goto again1;

	/* process low priority list */
	ret = handle_and_move(&g_diag_info->low_prio_list);
	if (ret == 1U)
		goto again1;
	else if (ret == 2U)
		goto again2;
}

/* other drivers use this interface to send event */
int32_t diagnose_send_event(struct diag_event *event)
{
	struct diag_module *module;
	struct diag_event_id *event_id = NULL;
	int32_t ret;

	/* sanity check */
	if (((event->module_id == (uint8_t)0)
		|| (event->module_id >= (uint8_t)ModuleIdMax))
		|| ((event->event_id == (uint8_t)0)
		|| (event->event_id >= (uint8_t)EVENT_ID_MAX))
		|| ((event->event_prio == (uint8_t)0)
		|| (event->event_prio >= (uint8_t)DiagMsgPrioMax))
		|| ((event->event_sta == (uint8_t)0)
		|| (event->event_sta >= (uint8_t)DiagEventStaMax))) {
		pr_err("invalid input parameter\n");	/* PRQA S ALL */
		return -1;
	}

	module = is_module_event_registered(event, &event_id);
	if (module == NULL) {
		/* PRQA S 3200 ++ */
		pr_err("module:%hx event:%hx not registered,send fail\n",
				event->module_id, event->event_id);
		/* PRQA S 3200 -- */
		return -1;
	}

	/* check send time interval */
	ret = is_send_condition_ready(event_id, event);
	if (ret == 0) {
		pr_debug("module:%hx event:%hx send condition not meet\n",
					module->module_id, event_id->id_handle.event_id);	/* PRQA S ALL */
		return -1;
	}

	/* insert to priority queue */
	ret = module_event_add_to_list(event);
	if (ret < 0) {
		return -1;
	}

	pr_debug("module:%hx,event:%hx,sta:%hhx\n",
			event->module_id, event->event_id, event->event_sta);

	schedule_work(&g_diag_info->diag_work);	/* PRQA S 3200 */

	return 0;
}
EXPORT_SYMBOL(diagnose_send_event); /* PRQA S ALL */

int32_t diagnose_register(const struct diag_register_info *register_info)
{
	struct diag_module *module = NULL;
	struct diag_event_id *event = NULL;
	uint8_t j;
	uint8_t event_count;
	unsigned long flags;

	if (register_info == NULL) {
		pr_err("invalid input parameter\n");
		return -EINVAL;
	}

	/* whether the module has been resigsterd */
	spin_lock_irqsave(&g_diag_info->module_lock, flags);
	list_for_each_entry(module, &g_diag_info->module_list, list) { /* PRQA S 0497, 0602 */
		if (register_info->module_id == module->module_id) {
			/* module has been registered, and we need continue to check
			 * event_ids */
			event_count = module->event_cnt;
			for (j = 0; j < register_info->event_cnt; j++) {
				if (event_count >= (uint8_t)EVENT_ID_MAX) {
					spin_unlock_irqrestore(&g_diag_info->module_lock, flags);
					pr_err("event count out of range\n");
					return -EINVAL;
				}
				list_for_each_entry(event, &module->events, list) {
					if (register_info->event_handle[j].event_id ==
							event->id_handle.event_id) {
						spin_unlock_irqrestore(&g_diag_info->module_lock, flags);
						pr_err("module:%#x event:%#x already registered\n",
								module->module_id,
								register_info->event_handle[j].event_id);
						return -EEXIST;
					}
				}
				/* register new event */
				event = (struct diag_event_id *)kzalloc(sizeof(*event),
								GFP_ATOMIC);
				if (event == NULL) {
					spin_unlock_irqrestore(&g_diag_info->module_lock, flags);
					pr_err("module:%hx event:%hx register fail\n",
								module->module_id,
								register_info->event_handle[j].event_id);
					return -ENOMEM;
				}
				event->id_handle = register_info->event_handle[j];
				event->last_sta = (uint8_t)DiagEventStaUnknown;
				event->last_snd_time = 0;
				list_add_tail(&event->list, &module->events);
				event_count++;
				/* update the event count value */
				module->event_cnt = event_count;
			}
			spin_unlock_irqrestore(&g_diag_info->module_lock, flags);

			return 0;
		}
	}
	spin_unlock_irqrestore(&g_diag_info->module_lock, flags);

	module = (struct diag_module *)kzalloc(sizeof(struct diag_module), GFP_ATOMIC);
	if (module == NULL) {
		pr_err("module:%hx register fail\n", register_info->module_id);
		return -ENOMEM;
	}

	/* initialize diag_module */
	module->module_id = register_info->module_id;
	module->event_cnt = register_info->event_cnt;
	INIT_LIST_HEAD(&module->events);
	for (j = 0; (j < module->event_cnt) && (j < (uint8_t)EVENT_ID_MAX); j++) {
		event = (struct diag_event_id *)kzalloc(sizeof(*event), GFP_ATOMIC);
		if (event == NULL) {
			pr_err("module:%hx register fail\n", module->module_id);
			return -ENOMEM;
		}
		event->id_handle = register_info->event_handle[j];
		event->last_sta = (uint8_t)DiagEventStaUnknown;
		event->last_snd_time = 0;
		/* lockless here ,for the reason that event list is traversed only when
		 * related module has been inserted to module_list */
		list_add_tail(&event->list, &module->events);
		pr_debug("module:%hx event:%hx will be resgtered\n", module->module_id,
						register_info->event_handle[j].event_id);
	}

	/* add to module_list */
	spin_lock_irqsave(&g_diag_info->module_lock, flags);
	list_add_tail(&module->list, &g_diag_info->module_list);
	spin_unlock_irqrestore(&g_diag_info->module_lock, flags);

	return 0;
}
EXPORT_SYMBOL(diagnose_register); /* PRQA S ALL */

int32_t diagnose_unregister(uint16_t module_id)
{
	struct diag_module *module;
	struct diag_event_id *event, *next;
	unsigned long flags;

	spin_lock_irqsave(&g_diag_info->module_lock, flags);
	list_for_each_entry(module, &g_diag_info->module_list, list) { /* PRQA S 0497, 0602 */
		if (module->module_id != module_id)
			continue;
		list_for_each_entry_safe(event, next, &module->events, list) {
			list_del(&event->list);
			kfree(event);
		}
		list_del(&module->list);
		spin_unlock_irqrestore(&g_diag_info->module_lock, flags);
		kfree(module);

		return 0;
	}
	spin_unlock_irqrestore(&g_diag_info->module_lock, flags);

	pr_err("module:%hx unregister fail\n", module_id);

	return -EINVAL;
}
EXPORT_SYMBOL(diagnose_unregister); /* PRQA S ALL */

int32_t diag_event_unregister(uint16_t module, uint16_t event)
{
	struct diag_module *module_tmp = NULL;
	struct diag_event_id *event_tmp = NULL;
	unsigned long flags;

	spin_lock_irqsave(&g_diag_info->module_lock, flags);
	list_for_each_entry(module_tmp, &g_diag_info->module_list, list) {
		if (module_tmp->module_id != module)
			continue;
		list_for_each_entry(event_tmp, &module_tmp->events, list) {
			if (event_tmp->id_handle.event_id == event) {
				list_del(&event_tmp->list);
				kfree(event_tmp);
				module_tmp->event_cnt--;
				if (module_tmp->event_cnt == 0) {
					list_del(&module_tmp->list);
					kfree(module_tmp);
				}
				spin_unlock_irqrestore(&g_diag_info->module_lock, flags);
				pr_debug("module:%hx event:%hx unregistered,event count:%hhu\n",
								module, event, module_tmp->event_cnt);
				return 0;
			}
		}
	}
	spin_unlock_irqrestore(&g_diag_info->module_lock, flags);

	pr_err("module:%hx event:%hx unregister fail\n",
				module, event);
	return -EINVAL;
}
EXPORT_SYMBOL(diag_event_unregister); /* PRQA S ALL */

/* Compatible with previous interfaces
 * The interfaces below may be discarded in the near future.
 */
/* =============== obsolete interfaces start ==============*/
int32_t diag_send_event_stat_and_env_data(
			uint8_t msg_prio,
			uint16_t module_id,
			uint16_t event_id,
			uint8_t event_sta,
			uint8_t env_data_gen_timing, /* PRQA S ALL */
			uint8_t *env_data, /* PRQA S ALL */
			size_t env_len) /* PRQA S ALL */
{
	struct diag_event event = {0};
	int32_t ret;
	if (env_len > ENV_PAYLOAD_SIZE) {
		pr_err("env_len is unsafe\n");
		return -1;
	}

	event.module_id = (uint8_t)module_id;
	event.event_id = (uint8_t)event_id;
	event.event_prio = msg_prio;
	event.event_sta = event_sta;

	if (env_data != NULL && env_len != 0) {
		event.env_len = (uint8_t)env_len;
		event.when = env_data_gen_timing;
		memcpy(event.payload, env_data, env_len);
	}
	ret = diagnose_send_event(&event);
	if (ret < 0) {
		pr_debug("diagnose send event with env fail, module_id = %d, event_id = %d\n",
				event.module_id, event.event_id);
		return -1;
	}

	return 0;
}
EXPORT_SYMBOL(diag_send_event_stat_and_env_data); /* PRQA S ALL */

int32_t diag_send_event_stat(
		uint8_t msg_prio,
		uint16_t module_id,
		uint16_t event_id,
		uint8_t event_sta)
{
	struct diag_event event = {0};
	int32_t ret;

	event.module_id = (uint8_t)module_id;
	event.event_id = (uint8_t)event_id;
	event.event_prio = msg_prio;
	event.event_sta = event_sta;

	ret = diagnose_send_event(&event);
	if (ret < 0) {
		pr_debug("diagnose send event fail, module_id = %d, event_id = %d\n",
				event.module_id, event.event_id);
		return -1;
	}

	return 0;
}
EXPORT_SYMBOL(diag_send_event_stat); /* PRQA S ALL */

extern int32_t diag_register(
		uint16_t module_id,
		uint16_t event_id,
		size_t envdata_max_size, /* PRQA S ALL */
		uint32_t min_snd_ms,
		uint32_t max_time_out_snd,
		void (*rcvcallback)(void *p, size_t len)) /* PRQA S ALL */
{
	struct diag_register_info info;
	int32_t ret;

	info.module_id = (uint8_t)module_id;
	info.event_cnt = 1;
	info.event_handle[0].event_id = (uint8_t)event_id;
	info.event_handle[0].min_snd_ms = min_snd_ms;
	info.event_handle[0].max_snd_ms = max_time_out_snd;
	info.event_handle[0].cb = rcvcallback;

	ret = diagnose_register(&info);
	if (ret < 0) {
		pr_err("diagnose register fail\n");
		return -1;
	}

	PDEBUG("diag_register: module_id = %d, event_id = %d, registered!\n",
			module_id, event_id);

	return 0;
}
EXPORT_SYMBOL(diag_register); /* PRQA S ALL */
/* =============== obsolete interfaces end ==============*/

#ifdef SYSFS_DEBUG
static void diag_register_test(uint8_t module_id, uint8_t event_id)
{
	struct diag_register_info register_info;

	register_info.module_id = module_id;
	register_info.event_cnt = 1;
	register_info.event_handle[0].event_id = event_id;
	register_info.event_handle[0].min_snd_ms = 20;
	register_info.event_handle[0].max_snd_ms = 100;
	register_info.event_handle[0].cb = NULL;

	diagnose_register(&register_info);

	PDEBUG("diag_register_test\n");
}

static void diag_send_event_test(uint8_t module_id, uint8_t event_id,
		uint8_t event_prio, uint8_t event_sta)
{
	struct diag_event event;

	event.module_id = module_id;
	event.event_id = event_id;
	event.event_prio = event_prio;
	event.event_sta = event_sta;

	diagnose_send_event(&event);

	PDEBUG("diag_send_event_test\n");
}

static void diag_unregister_test(void)
{
	diagnose_unregister((uint8_t)ModuleDiagDriver);

	PDEBUG("diag_unregister_test");
}

static ssize_t diag_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf);
static ssize_t diag_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t n);

static struct kobj_attribute diag_attr =
	__ATTR(diagnose, 0644, diag_show, diag_store); /* PRQA S ALL */

static ssize_t diag_show(struct kobject *kobj, /* PRQA S ALL */
		struct kobj_attribute *attr, char *buf) /* PRQA S ALL */
{
	struct diag_module *module;
	uint8_t i;

	/* dump the information here */
	list_for_each_entry(module, &g_diag_info->module_list, list) { /* PRQA S 0497, 0602 */
		PDEBUG("Module ID--> %d\n", module->module_id);
		for (i = 0; i < module->event_cnt; i++) {
			PDEBUG("\tEvent_Id: %d, min_snd_ms: %d, max_snd_ms: %d\n",
					module->event_id[i].id_handle.event_id,
					module->event_id[i].id_handle.min_snd_ms,
					module->event_id[i].id_handle.max_snd_ms);
		}
	}

	return 0;
}

static ssize_t diag_store(struct kobject *kobj, /* PRQA S ALL */
		struct kobj_attribute *attr, const char *buf, size_t n) /* PRQA S ALL */
{
	int32_t command;
	int32_t ret;

	ret = sscanf(buf, "%u", &command);
	if (ret < 0) {
		pr_err("store input error\n");
		return 0;
	}

	PDEBUG("Input Command--> %d\n", command);

	switch (command) {
		case 1:		//diagnose register TEST-1
			diag_register_test((uint8_t)ModuleDiagDriver,
					(uint8_t)EventIdKernelToUserSelfTest);
			break;
		case 2:		//diagnose send event TEST-1
			diag_send_event_test((uint8_t)ModuleDiagDriver,
					(uint8_t)EventIdKernelToUserSelfTest,
					(uint8_t)DiagMsgPrioLow, (uint8_t)DiagEventStaSuccess);
			break;
		case 3:		//diagnose register TEST-2
			diag_register_test((uint8_t)ModuleDiagDriver,
					(uint8_t)EventIdKernelToUserSelfTest2);
			break;
		case 4:		//diagnose send event TEST-2
			diag_send_event_test((uint8_t)ModuleDiagDriver,
					(uint8_t)EventIdKernelToUserSelfTest2,
					(uint8_t)DiagMsgPrioMid, (uint8_t)DiagEventStaFail);
			break;
		case 5:		//diagnose unregister
			diag_unregister_test();
			break;
		default:
			pr_err("Command %d not supportted\n", command);
			break;
	}

	return (ssize_t)n;
}

#endif

static int32_t __init diag_init(void)
{
	if (g_diag_info == NULL) {
		g_diag_info = (struct diag *)kmalloc(sizeof(struct diag), GFP_ATOMIC);
		if (g_diag_info == NULL) {
			pr_err("struct diag allocate fail\n");
			return -1;
		}
	}

	INIT_LIST_HEAD(&g_diag_info->module_list);
	INIT_LIST_HEAD(&g_diag_info->low_prio_list);
	INIT_LIST_HEAD(&g_diag_info->mid_prio_list);
	INIT_LIST_HEAD(&g_diag_info->hig_prio_list);
	//INIT_LIST_HEAD(&g_diag_info->netlink_snd_list);
	INIT_LIST_HEAD(&g_diag_info->empty_list);

	spin_lock_init(&g_diag_info->module_lock);
	mutex_init(&g_diag_info->netlink_mutex); /* PRQA S ALL */
	spin_lock_init(&g_diag_info->prio_lock);
	spin_lock_init(&g_diag_info->empty_spinlock);

	/* initialize a work item */
	INIT_WORK(&g_diag_info->diag_work, diag_work_handler); /* PRQA S ALL */

	/* allocate module_event node and add to empty list */
	g_diag_info->module_event = (struct diag_module_event *)kmalloc(
			(size_t)sizeof(struct diag_module_event) *
			(size_t)EVENT_ELEMENT_ALL, GFP_ATOMIC);
	if (g_diag_info->module_event == NULL) {
		pr_err("event element allocate fail\n");

		kfree(g_diag_info);
		g_diag_info = NULL;

		return -1;
	} else {
		int32_t i;

		/* lockless here,for the reason that no diagnose work will be processed
		 * befor diagnose finish initialisation itsellf */
		for (i = 0; i < EVENT_ELEMENT_ALL; i++)
			list_add_tail(&g_diag_info->module_event[i].list,
					&g_diag_info->empty_list);
	}

	/* netlink socket initialize */
	g_diag_info->netlink_sock = (struct sock *)netlink_kernel_create(&init_net,
			NETLINK_DIAG, &netlink_cfg);
	if (g_diag_info->netlink_sock == NULL) {
		pr_err("netlink socket create fail\n");

		goto err_out;
	}

	diag_had_init = 1;
#ifdef SYSFS_DEBUG
	/* create class */
	g_diag_info->diag_class = class_create(THIS_MODULE, "x2_diag"); /* PRQA S 0602, 3237, 2754 */
	if (IS_ERR(g_diag_info->diag_class)) {
		pr_err("sysfs class create fail\n");

		netlink_kernel_release(g_diag_info->netlink_sock);
		g_diag_info->netlink_sock = NULL;
		goto err_out;
	}

	/* allocate device number */
	if (alloc_chrdev_region((dev_t *)&g_diag_info->diag_dev_num,
				0, 1, "x2_diag") < 0) {
		pr_err("device number allocate fail\n");

		class_destroy(g_diag_info->diag_class);
		netlink_kernel_release(g_diag_info->netlink_sock);
		g_diag_info->netlink_sock = NULL;
		goto err_out;
	}

	/* create device */
	g_diag_info->diag_device = device_create(g_diag_info->diag_class,
			NULL, (dev_t)g_diag_info->diag_dev_num, NULL, "x2_diag");
	if (IS_ERR(g_diag_info->diag_device)) {
		pr_err("sysfs device create fail\n");

		unregister_chrdev_region((dev_t)g_diag_info->diag_dev_num, 1);
		class_destroy(g_diag_info->diag_class);
		netlink_kernel_release(g_diag_info->netlink_sock);
		g_diag_info->netlink_sock = NULL;
		goto err_out;
	}

	if (sysfs_create_file(&g_diag_info->diag_device->kobj,
				&diag_attr.attr) == 0)
		return 0;

	pr_err("sysfs file create fail\n");
	unregister_chrdev_region((dev_t)g_diag_info->diag_dev_num, 1);
	class_destroy(g_diag_info->diag_class);
	netlink_kernel_release(g_diag_info->netlink_sock);
	g_diag_info->netlink_sock = NULL;
	goto err_out;
#endif

	PDEBUG("diag_init succeed\n");

	return 0;
err_out:
	kfree(g_diag_info->module_event);
	g_diag_info->module_event = NULL;
	kfree(g_diag_info);
	g_diag_info = NULL;
	return -1;
}

static void __exit diag_exit(void)
{
	struct diag_module *mod_pos, *mod_nxt;
	struct diag_event_id *evnt_pos, *evnt_nxt;

	if (g_diag_info) {
		if (g_diag_info->module_event) {
			kfree(g_diag_info->module_event);
			g_diag_info->module_event = NULL;
		}

		/* socket release */
		if (g_diag_info->netlink_sock) {
			netlink_kernel_release(g_diag_info->netlink_sock);
			g_diag_info->netlink_sock = NULL;
		}

		/* diag_module release */
		/* PRQA S 0497, 0602 ++ */
		/* lockless here, for the reason that no more diagnose message should be
		 * processed when diagnose is exiting itself */
		list_for_each_entry_safe(mod_pos, mod_nxt, &g_diag_info->module_list, list) {
		/* PRQA S 0497, 0602 -- */
			/* free events of this module first */
			list_for_each_entry_safe(evnt_pos, evnt_nxt, &mod_pos->events, list) {
				list_del(&evnt_pos->list);
				kfree(evnt_pos);
			}
			list_del(&mod_pos->list);
			kfree(mod_pos);
		}

#ifdef SYSFS_DEBUG
		sysfs_remove_file(&g_diag_info->diag_device->kobj,
				&diag_attr.attr);
		device_destroy(g_diag_info->diag_class, (dev_t)g_diag_info->diag_dev_num);
		unregister_chrdev_region((dev_t)g_diag_info->diag_dev_num, 1);
		class_destroy(g_diag_info->diag_class);
#endif

		kfree(g_diag_info);
	}

	g_diag_info = NULL;
}

//module_init(diag_init);
//module_exit(diag_exit);

/* PRQA S ALL ++ */
subsys_initcall(diag_init);
__exitcall(diag_exit);

MODULE_DESCRIPTION("Diagnose driver for Hobot X2/J2");
MODULE_AUTHOR("leye.wang@horizon.ai");
MODULE_LICENSE("GPL");
/* PRQA S ALL -- */
