#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "cJSON.h"
#include "server_config.h"

#define MIN(a, b) (a > b ? b : a)

struct dump_config {
	struct {
		char server_ip[16];
		short server_port;
	} server_config;
	uint32_t pipe_line;
	struct {
		uint32_t raw_enable;
	} raw_config;
	struct {
		uint32_t yuv_enable;
		uint32_t yuv_channel;
	} yuv_config; 
	struct {
		uint32_t video_enable;
		uint32_t video_channel;
	} video_config;
	struct {
		uint32_t jepg_enable;
		uint32_t jepg_channel;
	} jepg_config;
};

static struct dump_config g_dump_config;

static int dump_config_set_config(cJSON *root)
{
	cJSON *node, *sub;

	memset(&g_dump_config, 0, sizeof(struct dump_config));

	if (root == NULL) {
		printf("<%s: %d>root is NULL\n", __FUNCTION__, __LINE__);
		return -1;
	}

	sub = cJSON_GetObjectItem(root, "server_config");
	if (sub == NULL) {
		printf("<%s: %d>server_config is NULL\n", __FUNCTION__, __LINE__);
	}

	node = cJSON_GetObjectItem(sub, "ip_addr");
	if (node == NULL) {
		printf("<%s: %d>ip_addr is NULL\n", __FUNCTION__, __LINE__);
	} else {
		memcpy(g_dump_config.server_config.server_ip, node->valuestring,
			MIN(strlen(node->valuestring), 16));
	}

	node = cJSON_GetObjectItem(sub, "port");
	if (node == NULL) {
		printf("<%s: %d>port is NULL\n", __FUNCTION__, __LINE__);
	} else {
		g_dump_config.server_config.server_port = (node->valueint) & 0xffff;
	}

	sub = cJSON_GetObjectItem(root, "pipe_line");
	if (sub == NULL) {
		printf("<%s: %d>pipe_line is NULL\n", __FUNCTION__, __LINE__);
	} else {
		g_dump_config.pipe_line = sub->valueint;
	}

	sub = cJSON_GetObjectItem(root, "raw_config");
	if (sub == NULL) {
		printf("<%s: %d>raw_config is NULL\n", __FUNCTION__, __LINE__);
	}

	node = cJSON_GetObjectItem(sub, "raw_enable");
	if (node == NULL) {
		printf("<%s: %d>raw enable is NULL\n", __FUNCTION__, __LINE__);
	} else {
		g_dump_config.raw_config.raw_enable = node->valueint;
	}

	sub = cJSON_GetObjectItem(root, "yuv_config");
	if (sub == NULL) {
		printf("<%s: %d>yuv_config is NULL\n", __FUNCTION__, __LINE__);
	}

	node = cJSON_GetObjectItem(sub, "yuv_enable");
	if (node == NULL) {
		printf("<%s: %d>yuv enable is NULL\n", __FUNCTION__, __LINE__);
	} else {
		g_dump_config.yuv_config.yuv_enable = node->valueint;
	}

	node = cJSON_GetObjectItem(sub, "yuv_channel");
	if (node == NULL) {
		printf("<%s: %d>yuv channel is NULL\n", __FUNCTION__, __LINE__);
	} else {
		g_dump_config.yuv_config.yuv_channel = node->valueint;
	}
#if 0
	sub = cJSON_GetObjectItem(root, "video_config");
	if (sub == NULL) {
		printf("<%s: %d>video_config is NULL\n", __FUNCTION__, __LINE__);
	}

	node = cJSON_GetObjectItem(sub, "video_enable");
	if (node == NULL) {
		printf("<%s: %d>video enable is NULL\n", __FUNCTION__, __LINE__);
	} else {
		g_dump_config.video_config.video_enable = node->valueint;
	}

	node = cJSON_GetObjectItem(sub, "video_channel");
	if (node == NULL) {
		printf("<%s: %d>video channel is NULL\n", __FUNCTION__, __LINE__);
	} else {
		g_dump_config.video_config.video_channel = node->valueint;
	}
#endif
	sub = cJSON_GetObjectItem(root, "jepg_config");
	if (sub == NULL) {
		printf("<%s: %d>jepg_config is NULL\n", __FUNCTION__, __LINE__);
	}

	node = cJSON_GetObjectItem(sub, "jepg_enable");
	if (node == NULL) {
		printf("<%s: %d>jepg enable is NULL\n", __FUNCTION__, __LINE__);
	} else {
		g_dump_config.jepg_config.jepg_enable = node->valueint;
	}

	node = cJSON_GetObjectItem(sub, "jepg_channel");
	if (node == NULL) {
		printf("<%s: %d>jepg channel is NULL\n", __FUNCTION__, __LINE__);
	} else {
		g_dump_config.jepg_config.jepg_channel = node->valueint;
	}

	return 0;
}

int dump_config_init(const char *config_file_path)
{
	int size = 0, ret = 0, len = 0;
	char *file_buff = NULL;
	cJSON *root = NULL;

	if (config_file_path == NULL) {
		printf("<%s: %d>config_file_path is null\n", __FUNCTION__, __LINE__);
		ret = -1;
		goto err;
	}

	FILE *fp = fopen(config_file_path, "r");
	if (fp == NULL) {
		printf("<%s: %d>fopen failed\n", __FUNCTION__, __LINE__);
		ret = -1;
		goto err;
	}

	fseek(fp, 0, SEEK_END);
	size = ftell(fp);
	fseek(fp, 0, SEEK_SET);

	file_buff = malloc(size + 1);
	if (file_buff == NULL) {
		printf("<%s: %d>malloc failed\n", __FUNCTION__, __LINE__);
		ret = -1;
		goto err;
	}
	memset(file_buff, 0, size + 1);

	len = fread(file_buff, 1, size, fp);
	if (len != size) {
		printf("<%s: %d>fread failed len = %d\n", __FUNCTION__, __LINE__, len);
		ret = -1;
		goto err;
	}
	//printf("<%s: %d>file_buff = %s\n", __FUNCTION__, __LINE__, file_buff);

	root = cJSON_Parse(file_buff);
	if (root == NULL) {
		printf("<%s: %d>root failed\n", __FUNCTION__, __LINE__);
		ret = -1;
		goto err;
	}

	if (dump_config_set_config(root) < 0) {
		printf("<%s: %d>dump_config_set_config failed\n", __FUNCTION__, __LINE__);
		ret = -1;
		goto err;
	}

err:
	if (root != NULL)
		cJSON_Delete(root);
	if (file_buff != NULL)
		free(file_buff);
	if (fp != NULL)
		fclose(fp);

	return ret;
}

char *dump_config_get_server_ip(void)
{
	return g_dump_config.server_config.server_ip;
}

short dump_config_get_server_port(void)
{
	return g_dump_config.server_config.server_port;
}

void raw_config_info(uint32_t *raw_enable, uint32_t *pipe_line)
{
	*pipe_line = g_dump_config.pipe_line;
	*raw_enable = g_dump_config.raw_config.raw_enable;
}

void yuv_config_info(uint32_t *yuv_enable, uint32_t *pipe_line,
	uint32_t *yuv_channel)
{
	*pipe_line = g_dump_config.pipe_line;
	*yuv_enable = g_dump_config.yuv_config.yuv_enable;
	*yuv_channel = g_dump_config.yuv_config.yuv_channel;
}

void video_config_info(uint32_t *video_enable, uint32_t *pipe_line,
	uint32_t *video_channel)
{
	*pipe_line = g_dump_config.pipe_line;
	*video_enable = g_dump_config.video_config.video_enable;
	*video_channel = g_dump_config.video_config.video_channel;
}

void jepg_config_info(uint32_t *jepg_enable, uint32_t *pipe_line,
	uint32_t *jepg_channel)
{
	*pipe_line = g_dump_config.pipe_line;
	*jepg_enable = g_dump_config.jepg_config.jepg_enable;
	*jepg_channel = g_dump_config.jepg_config.jepg_channel;
}

