#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <pthread.h>
#include "list_func.h"
#include "vio_func.h"

int buf_list_empty(struct buf_list *list)
{
	int ret = 1;

	if (list != NULL) {
		vmon_dbg("--num %d--\n", list->num);
		if (list->num > 0) {
			ret = 0;
		}
	}

	return ret;
}

uint32_t get_list_length(struct buf_list *list)
{
	if (list == NULL)
		return 0;
	return list->num;
}

int buf_list_init(struct buf_list *list)
{
	struct buf_list_node *head = NULL;
	struct buf_list_node *rear = NULL;
	int ret = 0;

	if (list == NULL)
		return -1;

	pthread_mutex_init(&list->list_mutex, NULL);
	head = malloc(sizeof(struct buf_list_node));
	rear = malloc(sizeof(struct buf_list_node));
	if ((head != NULL) && (rear != NULL)) {
		memset(head, 0, sizeof(struct buf_list_node));
		memset(rear, 0, sizeof(struct buf_list_node));
		list->num = 0;
		list->head = head;
		list->rear = rear;
		list->head->back_node = (void *)rear;
		list->rear->prev_node = (void *)head;
	} else {
		if (head != NULL)
			free(head);
		if (rear != NULL)
			free(rear);
		ret = -1;
	}

	return ret;
}

int buf_list_add_node(struct buf_list *list, struct buf_list_node **node)
{
	int ret = 0;
	struct buf_list_node *temp = NULL;
	struct buf_list_node *temp_node = *node;
	void *temp_1 = NULL;
	void *temp_2 = NULL;

	if (list == NULL)
		return -1;

	pthread_mutex_lock(&list->list_mutex);
	temp = (struct buf_list_node *)list->rear->prev_node;
	temp->back_node = (void *)temp_node;
	temp_node->prev_node = (void *)temp;
	temp_node->back_node = (void *)list->rear;
	list->rear->prev_node = (void *)temp_node;
	list->num++;
	pthread_mutex_unlock(&list->list_mutex);
	vmon_dbg("--num is %d--\n", list->num);

	return ret;
}

int buf_list_remove_node(struct buf_list *list, struct buf_list_node **node)
{
	int ret = 0;
	struct buf_list_node *temp = NULL;
	struct buf_list_node *temp_node = NULL;

	if (list == NULL)
		return -1;

	pthread_mutex_lock(&list->list_mutex);

	if (buf_list_empty(list)) {
		vmon_err("--list is empty--\n");
		ret = -1;
	} else {
		temp_node = (struct buf_list_node *)list->head->back_node;
		temp = (struct buf_list_node *)temp_node->back_node;
		list->head->back_node = (void *)temp;
		temp->prev_node = (void *)list->head;
		temp_node->prev_node = NULL;
		temp_node->back_node = NULL;
		list->num--;
		*node = temp_node;
	}
	pthread_mutex_unlock(&list->list_mutex);
	vmon_dbg("--num is %d--\n", list->num);

	return ret;
}

void buf_free(struct buf_info *buf)
{
	uint32_t temp = 0;

	if (buf == NULL)
		return;

	while (temp < 4) {
		if (buf->ptr[temp] != NULL) {
			free(buf->ptr[temp]);
			buf->ptr[temp] = NULL;
		}
		temp++;
	}
}

void buf_list_free(struct buf_list *list)
{
	int ret = 1;
	uint32_t temp = 0;
	struct buf_list_node *node = NULL;

	if (list == NULL)
		return;

	while (list->num > 0) {
		ret = buf_list_remove_node(list, &node);
		if (ret < 0) {
		} else {
			free_yuv_buf(&node->buf);
			free(node);
			node = NULL;
		}
	}
}

void buf_list_deinit(struct buf_list *list)
{
	struct buf_list_node *node = NULL;

	if (list == NULL)
		return;

	buf_list_free(list);

	free(list->head);
	free(list->rear);
	list->head = NULL;
	list->rear = NULL;
}

