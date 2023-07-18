/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef HB_X2A_VIO_LIST_H
#define HB_X2A_VIO_LIST_H

#include <stdlib.h>

struct list_head {
	struct list_head *next, *prev;
};

void vio_init_list_head(struct list_head *list);
void vio_list_add(struct list_head *new, struct list_head *head);
void vio_list_add_tail(struct list_head *new, struct list_head *head);
void vio_list_add_before(struct list_head *new, struct list_head *head);
void vio_list_del(struct list_head *entry);
int vio_list_is_last(const struct list_head *list,
	const struct list_head *head);
int vio_list_empty(const struct list_head *head);

#define list_first(head) ((head)->next)
#define list_last(head) ((head)->prev)

#define list_for_each(pos, head) \
	for (pos = (head)->next; pos != (head); pos = pos->next)

#define list_for_each_safe(pos, n, head) \
	for (pos = (head)->next, n = pos->next; pos != (head); \
		pos = n, n = pos->next)

#endif //HB_X2A_VIO_LIST_H
