#ifndef __LIST_FUNC_H__
#define __LIST_FUNC_H__

#include "common.h"
#include "server_cmd.h"
#include "hb_vio_interface.h"

#define ENABELCPY 0

struct buf_info {
	struct cmd_header header_info;
// memcpy
	char *ptr[4];
// nomemcpy
	uint32_t pipe_line;
	uint32_t buf_valib;
	hb_vio_buffer_t buf;
};

struct buf_list_node {
        struct buf_info buf;
        void *prev_node;
        void *back_node;
};

struct buf_list {
        uint32_t num;
        pthread_mutex_t list_mutex;
        struct buf_list_node *head;
        struct buf_list_node *rear;
};

int buf_list_empty(struct buf_list *list);
uint32_t get_list_length(struct buf_list *list);
int buf_list_init(struct buf_list *list);
int buf_list_add_node(struct buf_list *list, struct buf_list_node **node);
int buf_list_remove_node(struct buf_list *list, struct buf_list_node **node);
void buf_free(struct buf_info *buf);
void buf_list_free(struct buf_list *list);
void buf_list_deinit(struct buf_list *list);

#endif
