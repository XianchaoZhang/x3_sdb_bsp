#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>
#include <pthread.h>
#include <sys/stat.h>
#include <time.h>
#include <sys/time.h>
#include <signal.h>
#include <fcntl.h>
#include <errno.h>

#include "utils/cJSON.h"
#include "utils/cJSON_Direct.h"

#include "x3_config.h"

#define X3_CONFIG_PATH "../test_data/"
#define X3_CONFIG_FILE X3_CONFIG_PATH"x3_config.json"

int g_x3_cfg_is_load = 0;
x3_cfg_t g_x3_config;

static key_info_t cfg_pileline_key[] = {
	MAKE_KEY_INFO(x3_cfg_pileline_t, KEY_TYPE_STRING, sensor_name, NULL),
	MAKE_KEY_INFO(x3_cfg_pileline_t, KEY_TYPE_U32, venc_bitrate, NULL),
	MAKE_KEY_INFO(x3_cfg_pileline_t, KEY_TYPE_U32, alog_id, NULL),
	MAKE_END_INFO()
};

static key_info_t cfg_ipc_key[] = {
	MAKE_KEY_INFO(x3_cfg_ipc_t, KEY_TYPE_U32, pipeline_num, NULL),
	MAKE_ARRAY_INFO(x3_cfg_ipc_t, KEY_TYPE_ARRAY, pipelines, cfg_pileline_key, 2, KEY_TYPE_OBJECT),
	MAKE_END_INFO()
};

static key_info_t cfg_usb_cam_key[] = {
	MAKE_KEY_INFO(x3_cfg_usb_cam_t, KEY_TYPE_OBJECT, pipeline, 	cfg_pileline_key),
	MAKE_END_INFO()
};

static key_info_t cfg_box_key[] = {
	MAKE_KEY_INFO(x3_cfg_box_t, KEY_TYPE_U32, box_chns, 	NULL),
	MAKE_KEY_INFO(x3_cfg_box_t, KEY_TYPE_U32, venc_bitrate, 	NULL),
	MAKE_KEY_INFO(x3_cfg_box_t, KEY_TYPE_U32, alog_id, NULL),
	MAKE_END_INFO()
};

static key_info_t x3_cfg_key[] = {
	MAKE_KEY_INFO(x3_cfg_t, KEY_TYPE_U32, solution_id, 	NULL),
	MAKE_KEY_INFO(x3_cfg_t, KEY_TYPE_STRING, solution_id_comment, 	NULL),
	MAKE_KEY_INFO(x3_cfg_t, KEY_TYPE_STRING, alog_id_comment, 	NULL),
	MAKE_KEY_INFO(x3_cfg_t, KEY_TYPE_U32, disp_dev, 	NULL),
	MAKE_KEY_INFO(x3_cfg_t, KEY_TYPE_OBJECT, ipc_solution, 	cfg_ipc_key),
	MAKE_KEY_INFO(x3_cfg_t, KEY_TYPE_OBJECT, usb_cam_solution, 	cfg_usb_cam_key),
	MAKE_KEY_INFO(x3_cfg_t, KEY_TYPE_OBJECT, box_solution, 	cfg_box_key),
	MAKE_END_INFO()
};


static cJSON* open_json_file(char* filename)
{
    FILE* f;
    long len;
    char* data;
    cJSON* json;

    f = fopen(filename, "rb");
    if (NULL == f)
		return NULL;
    fseek(f, 0, SEEK_END);
    len = ftell(f);
    fseek(f, 0, SEEK_SET);
    data = (char*)malloc(len + 1);
    fread(data, 1, len, f);
    fclose(f);

    data[len] = '\0';
    json = cJSON_Parse(data);
    if (!json) {
		printf("Error before: [%s]\n", cJSON_GetErrorPtr());
		free(data);
		return NULL;
    }

    free(data);
    return json;
}

static int write_json_file(char* filename, char* out)
{
	FILE* fp = NULL;

	fp = fopen(filename, "a+");
	if (fp == NULL) {
		fprintf(stderr, "open file failed\n");
		return -1;
	}
	fprintf(fp, "%s", out);

	if (fp != NULL)
		fclose(fp);

	return 0;
}

int x3_cfg_load_default_config()
{
	memset(&g_x3_config, 0, sizeof(g_x3_config));
	g_x3_config.solution_id = 2; // vedio box
	strcpy(g_x3_config.solution_id_comment, "0:ipc 1:usb cam 2:video box");
	strcpy(g_x3_config.alog_id_comment, "0: off 1: mobilenet_v2, 2: yolov5, 3:horizon personMultitask, 4:fcos");
	g_x3_config.disp_dev = 0; // 0:hdmi, 1:mipi lcd

	// ip camera
	g_x3_config.ipc_solution.pipeline_num = 1;
	strcpy(g_x3_config.ipc_solution.pipelines[0].sensor_name, "F37");
	g_x3_config.ipc_solution.pipelines[0].venc_bitrate = 8192;
	g_x3_config.ipc_solution.pipelines[0].alog_id = 3;

	// usb camera
	strcpy(g_x3_config.usb_cam_solution.pipeline.sensor_name, "F37");
	g_x3_config.usb_cam_solution.pipeline.venc_bitrate = 8192;
	g_x3_config.usb_cam_solution.pipeline.alog_id = 0;

	// video box
	g_x3_config.box_solution.box_chns = 1;
	g_x3_config.box_solution.venc_bitrate = 8192;
	g_x3_config.box_solution.alog_id = 3;

	// 保存到配置文件中
	return x3_cfg_save();
}


int x3_cfg_load()
{
	cJSON* json_root = NULL;

	if(is_file_exist(X3_CONFIG_FILE) != 0) {
		printf("config file %s not exist\n", X3_CONFIG_FILE);
		// 使用默认配置
		x3_cfg_load_default_config();
	} else {
		char str_json[2048];
		FILE* fd = fopen(X3_CONFIG_FILE, "r");
		if(fd != NULL)
		{
			fread(str_json, 1, 2048, fd);
			fclose(fd);
		}
		printf("str_json: %s\n", str_json);
		if(cjson_string2object(x3_cfg_key, str_json, &g_x3_config) == NULL)
		{
			x3_cfg_load_default_config();
		}
	}

	return 0;
}

int x3_cfg_save()
{
	if(is_dir_exist(X3_CONFIG_PATH) != 0)
	{
		if(mkdir(X3_CONFIG_PATH, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) != 0)
		{
			printf("mkdir %s error", X3_CONFIG_PATH);
			return -1;
		}
	}

	char* out = cjson_object2string(x3_cfg_key, (void *)&g_x3_config);
	FILE* fd = fopen(X3_CONFIG_FILE, "w");
	if(fd != NULL)
	{
		fwrite(out, 1, strlen(out), fd);
		fclose(fd);
	}
	free(out);

	return 0;
}

char* x3_cfg_obj2string()
{
	return cjson_object2string(x3_cfg_key, (void *)&g_x3_config);
}

void x3_cfg_string2obj(char *in)
{
	cjson_string2object(x3_cfg_key, in, &g_x3_config);
}

