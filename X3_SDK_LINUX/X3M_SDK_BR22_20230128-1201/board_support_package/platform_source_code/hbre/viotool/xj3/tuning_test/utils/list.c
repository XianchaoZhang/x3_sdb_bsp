/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#include <stdlib.h>
#include "list.h"

void vio_init_list_head(struct list_head *list)
{
	list->next = list;
	list->prev = list;

}

static inline void __list_add(struct list_head *new,
                              struct list_head *prev, struct list_head *next)
{
	next->prev = new;
	new->next = next;
	new->prev = prev;
	prev->next = new;
}

void vio_list_add(struct list_head *new, struct list_head *head)
{

	__list_add(new, head, head->next);
}

void vio_list_add_tail(struct list_head *new, struct list_head *head)
{

	__list_add(new, head->prev, head);
}

void vio_list_add_before(struct list_head *new, struct list_head *head)
{

	__list_add(new, head->prev->prev, head->prev);

}

static inline void __list_del(struct list_head *prev, struct list_head *next)
{
	if (prev && next) {
		next->prev = prev;
		prev->next = next;
	}
}

void vio_list_del(struct list_head *entry)
{

	__list_del(entry->prev, entry->next);
	entry->next = NULL;
	entry->prev = NULL;

}

int vio_list_is_last(const struct list_head *list, const struct list_head *head)
{
	return list->next == head;
}

int vio_list_empty(const struct list_head *head)
{
	return head->next == head;
}
