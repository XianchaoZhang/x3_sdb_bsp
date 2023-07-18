/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>
#include <pthread.h>
#include <time.h>
#include <sys/time.h>
#include <signal.h>
#include <fcntl.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "hb_vio_interface.h"
#include "hb_cam_interface.h"
#include "hb_isp.h"
#include "../utils/list.h"
#include "gdc_json_parser.h"
#include "iar_interface.h"
#include "hb_vp_api.h"
#include "sensor_info_common.h"
#include "cJSON.h"

#define VIO_MONTOR
#define SENSOR_MCLK 37125000
#ifdef VIO_MONTOR
#include "../../../dump_server/viomontor.h"
#endif

#define GAVG_Y      90
#define RATIO_RG    100
#define RATIO_BG    100

#define CFG_NODE_SIZE   (0x1c310 - 0xab6c + 4)
#define CTX_CP_SPEED_2	0x36c
#define CTX_NODE_TOTAL_SIZE (CFG_NODE_SIZE + CTX_CP_SPEED_2 + 4)

#define MAX_INTEGRATION_TIME    8000
#define MIN_INTEGRATION_TIME    10
#define MIN_GAIN                0

typedef enum HB_ISP_OP_TYPE_E {
    OP_TYPE_AUTO = 0,
    OP_TYPE_MANUAL,
} ISP_OP_TYPE_E;
typedef struct HB_ISP_AE_ATTR_S {
    uint32_t u32MaxExposureRatio;
    uint32_t u32MaxIntegrationTime;
    uint32_t u32MaxSensorAnalogGain;
    uint32_t u32MaxSensorDigitalGain;
    uint32_t u32MaxIspDigitalGain;
    uint32_t u32Exposure;
    uint32_t u32ExposureRatio;
    uint32_t u32IntegrationTime;
    uint32_t u32SensorAnalogGain;
    uint32_t u32SensorDigitalGain;
    uint32_t u32IspDigitalGain;
    ISP_OP_TYPE_E enOpType;
} ISP_AE_ATTR_S;

extern int HB_ISP_SetAeAttr(uint8_t pipeId, const ISP_AE_ATTR_S *pstAeAttr);
extern int HB_ISP_GetAeAttr(uint8_t pipeId, ISP_AE_ATTR_S *pstAeAttr);

typedef enum pipe_id {
	PIPE_0 = 0,
	PIPE_1,
	PIPE_2,
	PIPE_3,
	PIPE_4,
	PIPE_5,
	PIPE_6,
	PIPE_7,
	PIPE_MAX
} pipe_id_e;

typedef enum HB_ISP_FW_STATE_E {
	ISP_FW_STATE_RUN = 0,
	ISP_FW_STATE_FREEZE,
} ISP_FW_STATE_E;
extern int HB_ISP_SetFWState(uint8_t pipeId, const ISP_FW_STATE_E enState);

static const char *const buf_name[HB_VIO_DATA_TYPE_MAX] = {
	"ipu_ds0",
	"ipu_ds1",
	"ipu_ds2",
	"ipu_ds3",
	"ipu_ds4",
	"ipu_us",
	"pym_feed_src",
	"pym",
	"sif_feed_src",
	"sif raw",
	"isp",
	"gdc"
};

#ifdef BIT
#undef BIT
#endif
#define BIT(n)  (1UL << (n))

#define DS0_ENABLE BIT(HB_VIO_IPU_DS0_DATA)
#define DS1_ENABLE BIT(HB_VIO_IPU_DS1_DATA)
#define DS2_ENABLE BIT(HB_VIO_IPU_DS2_DATA)
#define DS3_ENABLE BIT(HB_VIO_IPU_DS3_DATA)
#define DS4_ENABLE BIT(HB_VIO_IPU_DS4_DATA)
#define US_ENABLE  BIT(HB_VIO_IPU_US_DATA)
#define PYM_ENABLE BIT(HB_VIO_PYM_DATA)

#define RAW_ENABLE BIT(HB_VIO_SIF_RAW_DATA) //for raw dump/fb
#define RAW_FEEDBACK_ENABLE BIT(HB_VIO_SIF_FEEDBACK_SRC_DATA)

typedef struct work_info_s {
	uint32_t pipe_id;
	VIO_DATA_TYPE_E data_type;
	pthread_t thid;
	int running;
} work_info_t;

typedef struct callback_list_s {
	struct list_head cb_process;
	pthread_t cb_pid;
	pthread_mutex_t cb_mutex;
	int run_enable;
} callback_list_t;

typedef struct callback_data_s {
	struct list_head node;
	int cb_type;
	int pipe_id;
	hb_vio_buffer_t ipu_buffer;
	pym_buffer_t pym_buffer;
} callback_data_t;

const char *vio_cfg_file;
const char *hobotplayer_cfg_file;
time_t start_time;
time_t run_time = 0;
time_t end_time = 0;
int pipe_num;
int data_type;
int pipe_mask = 0;
int need_clk;
int mipiIdx;
int loop;
int need_cam = 1;
int mode_switch = 0;
int cam_index;
int need_free;
int need_get;
int need_m_thread;
int need_gdc_fb = 0;
int need_dump;
int need_status;
int need_pym_feedback;
int need_gdc_process;
int need_pym_process;
int need_display = 0;
int condition_time = 0;
char *gdc_cfg = NULL;
const char *gdc_fb_pic = NULL;
char *raw_path;
char *yuv_dir;
int need_callback;
const char *pym_feedback_pic;
char *cam_cfg_file = NULL;
int need_share_ae = 0;
int image_check_on = 0;
int gavg_y_threshold = 0;
int grg_threshold = 0;
int gbg_threshold = 0;
int image_check_error_threshold = 30;
int ae_test = 0;
int need_dump_ctx = 0;
int32_t sen_devfd = -1;

// add sensor info
int set_sensor_info = 1;
int lines_per_second;
int exposure_time_long_max;
int exposure_time_max;
int analog_gain_max;
int digital_gain_max;
int exposure_time_min;
const char *sensor_info_cfg_file;

#define CAMERA_IOC_MAGIC   'x'
#define SENSOR_TURNING_PARAM   _IOW(CAMERA_IOC_MAGIC, 0, sensor_turning_data_t)


work_info_t work_info[PIPE_MAX][HB_VIO_DATA_TYPE_MAX];

callback_list_t cb_list;
isp_context_t ptx;

pthread_mutex_t g_pym_lock;
pthread_mutex_t g_gdc_lock[2];

void print_usage(const char *prog)
{
	printf("Usage: %s \n", prog);
	puts("  -v --vio_config       vio config file path\n"
		 "  -c --real_camera      real camera enable\n"
		 "  -C --cam_cfg_file     real camera cfg files\n"
	     "  -r --run_time         time measured in seconds the program runs\n"
	     "  -p --pipe_id          pipe type\n"
		 "  -M --pipe_mask        pipe mask\n"
	     "  -t --data_type        data type\n"
	     "  -l --loop             need while/loop\n"
	     "  -f --free             need free data\n"
	     "  -g --get              need get data\n"
	     "  -m --multi_thread     multi thread get\n"
	     "  -d --dump             dump img\n"
		 "  -k --mclk			  enable sensor mclk\n"
		 "  -i --mipiIdx			  mipiIdx\n"
	     "  -S --stat             print status info\n"
	     "  -F --feedback         pym feedback\n"
	     "  -z --pym_fb_pic       pym feedbakc img\n"
	     "  -G --gdc_process	gdc process\n"
	     "  -P --process          need pym process\n"
	     "  -T --condition_time   get condition time\n"
		"  -n --raw_feedback     get raw file path\n"
		"  -u --gdc_fb           gdc feedback process\n"
		"  -U --gdc_bin_cfg_path gdc cfg bin path\n"
		"  -w --gdc_fb_pic       gdc fb pic\n"
		"  -b --callback_enable  callback enable\n"
		"  -D --display          need display\n"
		"  -e --camera existence   need camera\n"
		"  -a --share_ae  need share ae\n"
		"  -E --sensor_info  need feedback mode\n"
        "  -A --ae_test need test ae access\n"
		"-R --dump or feedback ctx & raw\n");
	exit(1);
}

void parse_opts(int argc, char *argv[])
{
	while (1) {
		static const char short_options[] =
        "v:c:C:r:p:M:t:l:f:g:m:d:k:i:S:F:z:G:P:T:n:N:u:U:w:b:H:D:e:s:a:R:V:W:X:Y:Z:E:A:y:";
		static const struct option long_options[] = {
			{"vio_config", 1, 0, 'v'},
			{"cam_index", 1, 0, 'c'},
			{"run_time", 1, 0, 'r'},
			{"pipe_num", 1, 0, 'p'},
			{"pipe_mask", 1, 0, 'M'},
			{"data_type", 1, 0, 't'},
			{"loop", 1, 0, 'l'},
			{"free", 1, 0, 'f'},
			{"get", 1, 0, 'g'},
			{"multi_thread", 1, 0, 'm'},
			{"dump", 1, 0, 'd'},
			{"mclk", 1, 0, 'k'},
			{"mipiIdx", 1, 0, 'i'},
			{"stat", 1, 0, 'S'},
			{"feedback", 1, 0, 'F'},
			{"pym_fb_pic", 1, 0, 'z'},
			{"gdc_process", 1, 0, 'G'},
			{"process", 1, 0, 'P'},
			{"condition_time", 1, 0, 'T'},
			{"raw_feedback", 1, 0, 'n'},
			{"dis_feedback", 1, 0, 'N'},
			{"gdc_fb", 1, 0, 'u'},
			{"gdc_bin_cfg", 1, 0, 'U'},
			{"gdc_fb_pic", 1, 0, 'w'},
			{"callback_enable", 1, 0, 'b'},
			{"hobotplayer_cfg", 1, 0, 'H'},
			{"display", 1, 0, 'D'},
			{"need_cam", 1, 0, 'e'},
			{"mode_switch", 1, 0, 's'},
			{"cam_cfg_file", 1, 0, 'C'},
			{"share_ae", 1, 0, 'a'},
			{"dump ctx_raw", 1, 0, 'R'},
            {"image_check_error_threshold", 1, 0, 'V'},
            {"image_check_on", 1, 0, 'W'},
            {"global avg y", 1, 0, 'X'},
            {"global rg", 1, 0, 'Y'},
            {"global bg", 1, 0, 'Z'},
			{"sensor_info_cfg", 1, 0, 'E'},
            {"ae_test", 1, 0, 'A'},
	    {"dump_yuv_dir", 1, 0, 'y'},
			{NULL, 0, 0, 0},
		};

		int cmd_ret;

		cmd_ret =
		    getopt_long(argc, argv, short_options, long_options, NULL);

		if (cmd_ret == -1)
			break;

		switch (cmd_ret) {
		case 'v':
			vio_cfg_file = optarg;
			printf("vio_cfg_file = %s\n", vio_cfg_file);
			break;
		case 'H':
			hobotplayer_cfg_file = optarg;
			printf("hobotplayer_cfg_file = %s\n", hobotplayer_cfg_file);
			break;
		case 'c':
			cam_index = atoi(optarg);
			printf("cam_index = %d\n", cam_index);
			break;
		case 'r':
			run_time = atoi(optarg);
			printf("run_time = %ld\n", run_time);
			break;
		case 'p':
			pipe_num = atoi(optarg);
			printf("pipe_num = %d\n", pipe_num);
			break;
		case 'M':
			pipe_mask = atoi(optarg);
			printf("pipe_mask = %d\n", pipe_mask);
			break;
		case 't':
			data_type = atoi(optarg);
			printf("data_type = %d\n", data_type);
			break;
		case 'k':
			need_clk = atoi(optarg);
			printf("need_clk = %d\n", need_clk);
			break;
		case 'i':
			mipiIdx = atoi(optarg);
			printf("mipiIdx = %d\n", mipiIdx);
			break;
		case 'l':
			loop = atoi(optarg);
			printf("loop = %d\n", loop);
			break;
		case 'f':
			need_free = atoi(optarg);
			printf("need_free = %d\n", need_free);
			break;
		case 'g':
			need_get = atoi(optarg);
			printf("need_get = %d\n", need_get);
			break;
		case 'm':
			need_m_thread = atoi(optarg);
			printf("need_m_thread = %d\n", need_m_thread);
			break;
		case 'd':
			need_dump = atoi(optarg);
			printf("need_dump = %d\n", need_dump);
			break;
		case 'S':
			need_status = atoi(optarg);
			printf("need_status = %d\n", need_status);
			break;
		case 'F':
			need_pym_feedback = atoi(optarg);
			printf("need_pym_feedback = %d\n", need_pym_feedback);
			break;
		case 'z':
			pym_feedback_pic = optarg;
			printf("pym_feedback_pic = %s\n", pym_feedback_pic);
			break;
		case 'P':
			need_pym_process = atoi(optarg);
			printf("need_pym_process = %d\n", need_pym_process);
			break;
		case 'G':
			need_gdc_process = atoi(optarg);
			printf("need_gdc_process = %d\n", need_gdc_process);
			break;
		case 'T':
			condition_time = atoi(optarg);
			printf("condition_time = %d\n", condition_time);
			break;
		case 'n':
		 {
			raw_path = optarg;
			printf("raw_path = %s\n", raw_path);
			break;
		}
		case 'b':
			need_callback = atoi(optarg);
			printf("need_callback = %d\n", need_callback);
			break;
		case 'u':
			need_gdc_fb = atoi(optarg);
			printf("need_gdc_fb = %d\n", need_gdc_fb);
			break;
		case 'U':
			gdc_cfg = optarg;
			printf("gdc_cfg = %s\n", gdc_cfg);
			break;
		case 'w':
			gdc_fb_pic = optarg;
			printf("gdc_fb_pic = %s\n", gdc_fb_pic);
			break;
		case 'D':
		        need_display = atoi(optarg);
		        printf("need_display = %d\n", need_display);
		        break;
		case 'e':
				need_cam = atoi(optarg);
				printf("need_cam = %d\n", need_cam);
				break;
		case 's':
				mode_switch = atoi(optarg);
				printf("mode_switch = %d\n", mode_switch);
				break;
		case 'C':
				cam_cfg_file = optarg;
				printf("cam_cfg_file = %s\n", cam_cfg_file);
				break;
		case 'a':
				need_share_ae = atoi(optarg);
				printf("need_share_ae for pipe %d and %d\n",
					need_share_ae / 10, need_share_ae % 10);
				break;
		case 'R':
			need_dump_ctx = atoi(optarg);
			break;
        case 'V':
                image_check_error_threshold = atoi(optarg);
                printf("image_check_error_threshold = %d\n",
                        image_check_error_threshold);
                break;
        case 'W':
                image_check_on = atoi(optarg);
                printf("image_check_on = %d\n", image_check_on);
                break;
        case 'X':
                gavg_y_threshold = atoi(optarg);
                printf("gavg_y_threshold = %d\n", gavg_y_threshold);
                break;
        case 'Y':
                grg_threshold = atoi(optarg);
                printf("grg_threshold = %d\n", grg_threshold);
                break;
        case 'Z':
                gbg_threshold = atoi(optarg);
                printf("gbg_threshold = %d\n", gbg_threshold);
                break;
		case 'E':
			sensor_info_cfg_file = optarg;
			printf("sensor_info_cfg_file = %s\n", sensor_info_cfg_file);
			break;
        case 'y':
		yuv_dir = optarg;
		printf("yuv_dir = %s\n", yuv_dir);
 		break;
        case 'A':
                ae_test = atoi(optarg);
                printf("ae_test = %d\n", ae_test);
                break;
		default:
			print_usage(argv[0]);
			break;
		}
	}
}

static int sensor_info_config_set_config(cJSON *root)
{
        cJSON *sub;

        if (root == NULL) {
                printf("<%s: %d>root is NULL\n", __FUNCTION__, __LINE__);
                return -1;
        }

        sub = cJSON_GetObjectItem(root, "lines_per_second");
        if (sub == NULL) {
                lines_per_second = 10000;
        } else {
                lines_per_second = sub->valueint;
        }

	sub = cJSON_GetObjectItem(root, "exposure_time_long_max");
        if (sub == NULL) {
                exposure_time_long_max = 2000;
        } else {
                exposure_time_long_max = sub->valueint;
        }

	sub = cJSON_GetObjectItem(root, "exposure_time_max");
        if (sub == NULL) {
                exposure_time_max = 2000;
        } else {
                exposure_time_max = sub->valueint;
        }

	sub = cJSON_GetObjectItem(root, "analog_gain_max");
        if (sub == NULL) {
                analog_gain_max = 255;
        } else {
                analog_gain_max = sub->valueint;
        }

	sub = cJSON_GetObjectItem(root, "digital_gain_max");
        if (sub == NULL) {
                digital_gain_max = 0;
        } else {
                digital_gain_max = sub->valueint;
        }

	sub = cJSON_GetObjectItem(root, "exposure_time_min");
        if (sub == NULL) {
                exposure_time_min = 0;
        } else {
                exposure_time_min = sub->valueint;
        }

        return 0;
}

static int sensor_info_config_init(const char *config_file_path)
{
        int size = 0, ret = 0, len = 0;
        char *file_buff = NULL;
        cJSON *root = NULL;

        if (config_file_path == NULL) {
                ret = -1;
                goto err;
        }

        FILE *fp = fopen(config_file_path, "r");
        if (fp == NULL) {
                ret = -2;
                goto err;
        }

        fseek(fp, 0, SEEK_END);
        size = ftell(fp);
        fseek(fp, 0, SEEK_SET);

        file_buff = malloc(size + 1);
        if (file_buff == NULL) {
                ret = -3;
                goto err;
        }
		memset(file_buff, 0, size + 1);

        len = fread(file_buff, 1, size, fp);
        if (len != size) {
                ret = -4;
                goto err;
        }

        root = cJSON_Parse(file_buff);
        if (root == NULL) {
                ret = -5;
                goto err;
        }

        if (sensor_info_config_set_config(root) < 0) {
                ret = -6;
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

int dumpToFile(char *filename, char *srcBuf, unsigned int size)
{
	FILE *yuvFd = NULL;
	char *buffer = NULL;

	if (need_dump_ctx) {
		yuvFd = fopen(filename, "ab+");
	} else {
		yuvFd = fopen(filename, "w+");
	}

	if (yuvFd == NULL) {
		printf("ERRopen(%s) fail", filename);
		return -1;
	}

	buffer = (char *)malloc(size);

	if (buffer == NULL) {
		printf(":malloc file");
		fclose(yuvFd);
		return -1;
	}

	memcpy(buffer, srcBuf, size);

	fflush(stdout);

	fwrite(buffer, 1, size, yuvFd);

	fflush(yuvFd);

	if (yuvFd)
		fclose(yuvFd);
	if (buffer)
		free(buffer);

	printf("filedump(%s, size(%d) is successed!!", filename, size);

	return 0;
}

int dumpToFile2plane(char *filename, char *srcBuf, char *srcBuf1,
		     unsigned int size, unsigned int size1)
{
	FILE *yuvFd = NULL;
	char *buffer = NULL;

	yuvFd = fopen(filename, "w+");

	if (yuvFd == NULL) {
		printf("open(%s) fail", filename);
		return -1;
	}

	buffer = (char *)malloc(size + size1);

	if (buffer == NULL) {
		printf("ERR:malloc file");
		fclose(yuvFd);
		return -1;
	}

	memcpy(buffer, srcBuf, size);
	memcpy(buffer + size, srcBuf1, size1);

	fflush(stdout);

	fwrite(buffer, 1, size + size1, yuvFd);

	fflush(yuvFd);

	if (yuvFd)
		fclose(yuvFd);
	if (buffer)
		free(buffer);

	printf("DEBUG:filedump(%s, size(%d) is successed!!\n", filename, size);

	return 0;
}


static int check_end(void)
{
	time_t now = time(NULL);
	//printf("Time info :: now(%ld), end_time(%ld) run time(%ld)!\n",now, end_time, run_time);
	return !(now > end_time && run_time > 0);
}

static void normal_buf_info_print(hb_vio_buffer_t * buf)
{
	printf("normal pipe_id (%d)type(%d)frame_id(%d)buf_index(%d)w x h(%dx%d)\n",
		buf->img_info.pipeline_id,
		buf->img_info.data_type,
		buf->img_info.frame_id,
		buf->img_info.buf_index,
		buf->img_addr.width,
		buf->img_addr.height);
}

static void pym_buf_info_print(pym_buffer_t * buf)
{
	printf("pym pipe_id(%d)type(%d)frame_id(%d)buf_index(%d)\n",
		buf->pym_img_info.pipeline_id,
		buf->pym_img_info.data_type,
		buf->pym_img_info.frame_id,
		buf->pym_img_info.buf_index);
}

int time_cost_ms(struct timeval *start, struct timeval *end)
{
	int time_ms = -1;
	time_ms = ((end->tv_sec * 1000 + end->tv_usec /1000) -
		(start->tv_sec * 1000 + start->tv_usec /1000));
	printf("time cost %d ms \n", time_ms);
	return time_ms;
}

void *g_config_buffer = NULL;

int gdc_cfg_bin_gen(char *layout_file, char* config_file,
							uint64_t *config_size)
{
	int ret = 0;
	window_t* windows = NULL;
	uint32_t  wnd_num = 0;
	param_t gdc_param;

	// Open input configuration file
	FILE* f = fopen(layout_file,"r");
	if (!f) {
		printf("Can't open transfo para file %s\n", layout_file);
		return -1;
	}

	fseek(f, 0, SEEK_END);
	size_t json_sz = ftell(f);
	char* json = (char*)malloc(json_sz+1);
	rewind(f);
	json[fread(json,sizeof(char),json_sz,f)] = 0;
	fclose(f);

	// Parse input configuration and extract transformation
	// and input/output window information
	memset(&gdc_param, 0, sizeof(gdc_param));

	if (gdc_parse_json(json, &gdc_param, &windows, &wnd_num)) {
	    printf("ERROR: Can't process json\n");
	    gdc_parse_json_clean(&windows, wnd_num);
	    return -1;
	}

	free(json);
	json = NULL;

	if(gdc_param.format == FMT_UNKNOWN){
	    printf("Can't process json: unknown frame format.\n");
	    gdc_parse_json_clean(&windows, wnd_num);
	    return -1;
	}

	if(wnd_num == 0) {
	    printf("Warning: no windows are specified.\n");
	    gdc_parse_json_clean(&windows, wnd_num);
	    return -1;
	}

	ret = hb_vio_gen_gdc_cfg(&gdc_param,
							windows,
							wnd_num,
							&g_config_buffer,
							config_size);

	if (ret == 0 && config_file != NULL) {
		// Write binary sequence to output file
		FILE *f = fopen(config_file,"wb");
		if (f != NULL) {
			fwrite(g_config_buffer, sizeof(char), *config_size, f);
			fclose(f);
		}
	}
	printf("gdc gen cfg_buf %p, size %lu \n", g_config_buffer, *config_size);
	gdc_parse_json_clean(&windows, wnd_num);
	return ret;
}

static int gdc_cfg_bin_update(uint32_t pipe_id, char *layout_file)
{
	int ret = 0;
	uint64_t config_size = 0;
	ret = gdc_cfg_bin_gen(layout_file, "./gdc.bin", &config_size);

	if (ret == 0) {
	printf("pipe(%u) cfg_buf(%p) size(%lu)\n",
			pipe_id, g_config_buffer, config_size);
	ret = hb_vio_set_gdc_cfg(pipe_id, g_config_buffer, config_size);
		if (g_config_buffer) {
			hb_vio_free_gdc_cfg(g_config_buffer);
	printf("free config_buf %p size(%lu)\n", g_config_buffer, config_size);
			g_config_buffer = NULL;
		}
		if (ret < 0) {
			printf("gdc cfg bin set failed.\n");
			return -1;
		}
	} else {
		printf("gdc cfg bin gen failed.\n");
		return -1;
	}
	printf("pipe(%u)gdc bin update done.File %s size %lu\n",
			pipe_id, layout_file, config_size);
	return ret;
}

static int gdc_feedback_func(work_info_t * info)
{
	int ret = 0;
	char file_name[100];
	hb_vio_buffer_t buf = {0};
	hb_vio_buffer_t dst_buf = {0};
	//printf("pipe(%u)gdc fb process start!\n", info->pipe_id);
	ret = gdc_cfg_bin_update(info->pipe_id, gdc_cfg);
	if (ret < 0) {
		return -1;
	}

	ret = hb_vio_get_data(info->pipe_id, HB_VIO_GDC_FEEDBACK_SRC_DATA, &buf);
	if (ret < 0) {
		printf("hb_vio_get_data GDC_FEEDBACK error!\n");
		return -1;
	}
	printf("pipe(%u)gdc fb buf(%d)img_addr.w = %d, h = %d, stride = %d\n",
			info->pipe_id,
			buf.img_info.buf_index,
			buf.img_addr.width,
			buf.img_addr.height,
			buf.img_addr.stride_size);
	if (gdc_fb_pic == NULL) {
		hb_vio_free_gdcbuf(info->pipe_id, &buf);
		printf("pipe(%u)gdc_fb_pic null !\n", info->pipe_id);
		return -1;
	}

	int img_in_fd = open(gdc_fb_pic, O_RDWR | O_CREAT, 0644);
	int size_0 = buf.img_addr.stride_size * buf.img_addr.height;
	int size_1 = size_0 / 2;
	printf("pipe(%u)gdc fb size_0 = %d, size_1 = %d\n",
		info->pipe_id,
		size_0,
		size_1);
	if(img_in_fd < 0) {
		printf("pipe(%u)open image failed !\n", info->pipe_id);
		return -1;
	}
	read(img_in_fd, buf.img_addr.addr[0], size_0);
	usleep(10 * 1000);
	read(img_in_fd, buf.img_addr.addr[1], size_1);
	usleep(10 * 1000);
	close(img_in_fd);

	int i = info->data_type % 2;
	printf("hb_vio_gdc_process begin use %d, rotate %d\n", i, need_gdc_fb);
	pthread_mutex_lock(&g_gdc_lock[i]);
	ret = hb_vio_run_gdc(info->pipe_id, &buf, &dst_buf, need_gdc_fb);
	pthread_mutex_unlock(&g_gdc_lock[i]);

	if (ret < 0) {
		printf("hb_vio_gdc_process failed use %d\n", i);
		hb_vio_free_gdcbuf(info->pipe_id, &buf);
		return -1;
	}
	normal_buf_info_print(&dst_buf);

	snprintf(file_name, sizeof(file_name),
		"./gdc_%d_%d_%d.yuv", info->data_type,
		dst_buf.img_addr.width, dst_buf.img_addr.height);
	if(need_dump) {
	dumpToFile2plane(file_name, dst_buf.img_addr.addr[0],
					dst_buf.img_addr.addr[1],
					dst_buf.img_addr.width * dst_buf.img_addr.height,
					dst_buf.img_addr.width * dst_buf.img_addr.height / 2);
	}
	hb_vio_free_gdcbuf(info->pipe_id, &buf);
	hb_vio_free_gdcbuf(info->pipe_id, &dst_buf);
	printf("gdc_fb process done use %d\n", i);
	return 0;
}

int myclamp(int data, int min, int max)
{
    if (data < min) {
        data = min;
    }

    if (data > max) {
        data = max;
    }

    return data;
}

void yuv2rgb_normal(uint8_t yValue, uint8_t uValue, uint8_t vValue,
        uint8_t *r, uint8_t *g, uint8_t *b)
{
    int rTmp = (int)(yValue + (1.370705 * (vValue-128)));
    int gTmp = (int)(yValue - (0.698001 * (vValue-128)) -   \
               (0.337633 * (uValue-128)));
    int bTmp = (int)(yValue + (1.732446 * (uValue-128)));

    *r = (uint8_t)(myclamp(rTmp, 0, 255));
    *g = (uint8_t)(myclamp(gTmp, 0, 255));
    *b = (uint8_t)(myclamp(bTmp, 0, 255));
}

void yuv2rgb(char *yuv_data[2], uint8_t *rgb_data, int width, int height)
{
    int i, j;
    char y = 0, u = 0, v = 0;

    printf("width = %d, height = %d\n", width, height);
    for (i = 0; i < height; i++) {
        for (j = 0; j < width; j++) {
            y = yuv_data[0][i*width +j];
            u = yuv_data[1][(i/2)*width + j/2*2];
            v = yuv_data[1][(i/2)*width + j/2*2 + 1];
            yuv2rgb_normal(y, u, v,
                    rgb_data + ((i*width+j)*3),
                    rgb_data + ((i*width+j)*3+1),
                    rgb_data + ((i*width+j)*3+2));
        }
    }
}

void y_component_counting(char *yuv_data[2], int width,
        int height, double *avg_y)
{
        uint32_t i;
        uint32_t sum_y = 0;
        uint32_t ysize = width * height;

        for (i = 0; i < ysize; i++) {
            sum_y += yuv_data[0][i];
        }
        if (ysize != 0) {
            *avg_y = (double)sum_y / ysize;
        }
}

void rgb_component_counting(uint8_t *rgb_data, int size, double *rg, double *bg)
{
        uint32_t sum_r = 0, sum_g = 0, sum_b = 0;
        int i;

        for (i = 6; i < size; i+=3) {
            sum_r += rgb_data[i];
            sum_g += rgb_data[i+1];
            sum_b += rgb_data[i+2];
        }
        if (sum_g != 0) {
            *rg = (double)sum_r / sum_g;
            *bg = (double)sum_b / sum_g;
        }
}

int image_check_thread(hb_vio_buffer_t buf, int ipu_loop_count,
                        work_info_t * info)
{
        int dst_frame_size;
        uint8_t *rgb_data;
        double avg_y = 0;
        double rg = 0, bg = 0;
        static uint16_t frame_count = 0;
        char file_name[64];

        if (gavg_y_threshold > 15 || gavg_y_threshold <= 0 ||
            grg_threshold > 20 || grg_threshold <= 0 ||
            gbg_threshold > 20 || gbg_threshold <= 0 ||
            image_check_error_threshold < 0) {
            printf("invalid args. X:(0, 15], Y:(0, 20], Z:(0, 20], V > 0;\n");
            return -1;
        }
        frame_count++;
        // ignore the first frame.
        if (frame_count > image_check_error_threshold) {
            dst_frame_size = buf.img_addr.width * buf.img_addr.height * 3;
            rgb_data = malloc(dst_frame_size*sizeof(uint8_t));
            if (rgb_data == NULL) {
                    printf("rgb_data is NULL, malloc failed!\n");
                    return -1;
            }
            // YUV to RGB24
            yuv2rgb(buf.img_addr.addr, rgb_data, buf.img_addr.width,
                            buf.img_addr.height);

            // y component counting
            y_component_counting(buf.img_addr.addr, buf.img_addr.height,
                            buf.img_addr.width, &avg_y);
            // r g b component counting
            rgb_component_counting(rgb_data, dst_frame_size, &rg, &bg);
            printf("frame id %d, avg_y %f, rg %f, bg %f\n",
                            buf.img_info.frame_id, avg_y, rg, bg);
            // value check
            rg *= 100;
            bg *= 100;
            if (avg_y < GAVG_Y - gavg_y_threshold ||
                        avg_y > GAVG_Y + gavg_y_threshold ||
                        rg < RATIO_RG - grg_threshold ||
                        rg > RATIO_RG + grg_threshold ||
                        bg < RATIO_BG - gbg_threshold ||
                        bg > RATIO_BG + gbg_threshold) {
                   printf("image check failed.\n");
                    snprintf(file_name, sizeof(file_name),
                                    "./pipe%d_ipu_loop%08d_type%d_index%d.yuv",
                                    info->pipe_id,
                                    ipu_loop_count,
                                    info->data_type, buf.img_info.buf_index);
                    dumpToFile2plane(file_name, buf.img_addr.addr[0],
                                    buf.img_addr.addr[1],
                                    buf.img_addr.width * buf.img_addr.height,
                                    buf.img_addr.width * buf.img_addr.height/2);
            }
            // free
            if (rgb_data != NULL) {
                    free(rgb_data);
                    rgb_data = NULL;
            }
        }
        return 0;
}

/* @brief ae access test
 * @details set ae to manaul mode than config exposure, and check the image brightness
 */
int ae_test_func(work_info_t * info)
{
    int ret = 0;
    hb_vio_buffer_t buf;
    double hight_exp_y = 0;
    double low_exp_y = 0;
    ISP_AE_ATTR_S ae_attr = {0};
    printf("ae_test!\n");

    // get ae
    HB_ISP_GetAeAttr(info->pipe_id, &ae_attr);
    ae_attr.u32MaxIntegrationTime = MAX_INTEGRATION_TIME;
    // set ae to hight export
    ae_attr.u32IntegrationTime = ae_attr.u32MaxIntegrationTime;
    ae_attr.u32SensorAnalogGain = ae_attr.u32MaxSensorAnalogGain;
    ae_attr.u32SensorDigitalGain = ae_attr.u32MaxSensorDigitalGain;
    ae_attr.u32IspDigitalGain = ae_attr.u32MaxIspDigitalGain;
    ae_attr.enOpType = OP_TYPE_MANUAL;
    HB_ISP_SetAeAttr(info->pipe_id, &ae_attr);
    if (ret < 0) {
        printf("ae test fail\n");
        return ret;
    }
    usleep(500000);
    // get data and calculate brightness
    ret = hb_vio_get_data(info->pipe_id, info->data_type, &buf);
    if (ret < 0) {
        printf("pipe(%u)get data type(%d) failed!\n",
                info->pipe_id,
                info->data_type);
        return -1;
    }
    y_component_counting(buf.img_addr.addr, buf.img_addr.height,
                        buf.img_addr.width, &hight_exp_y);
    printf("ae test: hight_exp_y is %f\n", hight_exp_y);

    // set ae to low export
    ae_attr.u32IntegrationTime = MIN_INTEGRATION_TIME;
    ae_attr.u32SensorAnalogGain = MIN_GAIN;
    ae_attr.u32SensorDigitalGain = MIN_GAIN;
    ae_attr.u32IspDigitalGain = MIN_GAIN;
    ret = HB_ISP_SetAeAttr(info->pipe_id, &ae_attr);
    if (ret < 0) {
        printf("ae test fail\n");
        return ret;
    }
    // get data and calculate brightness
    usleep(500000);
    ret = hb_vio_get_data(info->pipe_id, info->data_type, &buf);
    if (ret < 0) {
        printf("pipe(%u)get data type(%d) failed!\n",
                info->pipe_id,
                info->data_type);
        return -1;
    }
    y_component_counting(buf.img_addr.addr, buf.img_addr.height,
                        buf.img_addr.width, &low_exp_y);
    printf("ae test: low_exp_y is %f\n", low_exp_y);

    if (hight_exp_y - low_exp_y > 40) {
        printf("ae test succes!\n");
        ae_attr.enOpType = OP_TYPE_AUTO;
        ret = HB_ISP_SetAeAttr(info->pipe_id, &ae_attr);
        if (ret < 0) {
            printf("set ae to auto mode fail!!! ret = %d\n", ret);
            return -1;
        }
        return 0;
    } else {
        printf("ae test fail!!!\n");
        return -1;
    }
}

int ipu_get_put_buf_func(work_info_t * info)
{
    int ret = 0;
    static int ipu_loop_count = 1;
    pym_buffer_t pym_buf = {0};
    hb_vio_buffer_t buf = {0};
    hb_vio_buffer_t dst_buf = {0};
    char file_name[64];
    if (info->data_type > HB_VIO_IPU_US_DATA) {
        printf("pipe(%u)Not ipu data type (%d) ! return 0!",
             info->pipe_id, info->data_type);
    }
    if (need_get) {
        printf("Try to get Pipe(%u) data type(%d) (%s),status(%d),time(%d)\n",
            info->pipe_id, info->data_type, buf_name[info->data_type],
            need_status, condition_time);
        if (need_status == 1) {
            ret = hb_vio_get_data_conditional(info->pipe_id,
                    info->data_type, &buf, condition_time);
        } else {
            ret = hb_vio_get_data(info->pipe_id, info->data_type, &buf);
        }
        if (ret < 0) {
             printf("pipe(%u)get data type(%d) failed!\n",
                info->pipe_id,
                info->data_type);
            return -1;
        } else {
            printf("Get pipe(%u) data type(%d) (%s) done !\n",
            info->pipe_id, info->data_type, buf_name[info->data_type]);
            ipu_loop_count++;
        }
    }

    if (image_check_on) {
        ret = image_check_thread(buf, ipu_loop_count, info);
        if (ret < 0)
        return ret;
    }
	normal_buf_info_print(&buf);
if (need_dump) {
	if (ret < 0) {
		printf("pipe(%u)ipu date %d get failed.skip file dump\n",
		info->pipe_id,
		info->data_type);
	} else {
	char file_name[100] = { 0 };
	struct timeval time_now = { 0 };
	struct timeval time_next = { 0 };
	int size = -1;

	size = buf.img_addr.stride_size * buf.img_addr.height;

	printf("yuv stride_size(%u) w x h%u x %u, size %d\n",
						buf.img_addr.stride_size,
						buf.img_addr.width, buf.img_addr.height, size);
			snprintf(file_name, sizeof(file_name),
					"./pipe%d_ipu_loop%08d_type%d_index%d.yuv",
					info->pipe_id,
					ipu_loop_count,
					info->data_type, buf.img_info.buf_index);
			gettimeofday(&time_now, NULL);
			dumpToFile2plane(file_name, buf.img_addr.addr[0],
							buf.img_addr.addr[1], size, size/2);
			gettimeofday(&time_next, NULL);
			int time_cost = time_cost_ms(&time_now, &time_next);
			printf("pipe(%u)dumpToFile yuv cost time %d ms\n",
					info->pipe_id,
					time_cost);
	}
}

	if (need_pym_process) {
		pthread_mutex_lock(&g_pym_lock);
		ret = hb_vio_run_pym(info->pipe_id, &buf);
		if (ret < 0) {
			hb_vio_free_ipubuf(info->pipe_id, &buf);
			printf("hb_vio_pym_process failed.\n");
			pthread_mutex_unlock(&g_pym_lock);
			return -1;
		}

		ret = hb_vio_get_data(info->pipe_id, HB_VIO_PYM_DATA, &pym_buf);
		if (ret < 0) {
			printf("hb_vio_get_data HB_VIO_PYM_DATA error\n");
			pthread_mutex_unlock(&g_pym_lock);
			return -1;
		}
		pthread_mutex_unlock(&g_pym_lock);

		pym_buf_info_print(&pym_buf);

		ret = hb_vio_free_pymbuf(info->pipe_id, HB_VIO_PYM_DATA, &pym_buf);
		if (ret < 0) {
			printf("hb_vio_pym_free error\n");
			hb_vio_free_ipubuf(info->pipe_id, &buf);
			return -1;
		}
	}

	if (need_gdc_process) {
		int i = info->data_type % 2;
		printf("hb_vio_gdc_process begin use %d\n", i);
		pthread_mutex_lock(&g_gdc_lock[i]);
		ret = hb_vio_run_gdc(info->pipe_id, &buf, &dst_buf, 90);
		pthread_mutex_unlock(&g_gdc_lock[i]);

		if (ret < 0) {
			printf("hb_vio_gdc_process failed use %d\n", i);
			hb_vio_free_ipubuf(info->pipe_id, &buf);
			return -1;
		}
		printf("___ hb_vio_gdc_process end use %d\n", i);
		normal_buf_info_print(&dst_buf);

		snprintf(file_name, sizeof(file_name),
			"./gdc_done_src%dloop%d_%d_%d.yuv", info->data_type, ipu_loop_count,
			dst_buf.img_addr.width, dst_buf.img_addr.height);
		if(need_dump) {
		dumpToFile2plane(file_name, dst_buf.img_addr.addr[0],
						dst_buf.img_addr.addr[1],
						dst_buf.img_addr.width * dst_buf.img_addr.height,
						dst_buf.img_addr.width * dst_buf.img_addr.height / 2);
		}
		usleep(20000);
		hb_vio_free_gdcbuf(info->pipe_id, &dst_buf);
	}

	if (need_free) {
		hb_vio_free_ipubuf(info->pipe_id, &buf);
		printf("Free Pipe(%u) data type(%d) (%s) done!\n",
					info->pipe_id, info->data_type, buf_name[info->data_type]);
	}
	return 0;
}

int pym_get_put_buf_func(work_info_t * info)
{
	int ret = 0;
	pym_buffer_t pym_buf = {0};
	char file_name[64];

	if (info->data_type != HB_VIO_PYM_DATA) {
		printf("Not pym data type (%d) ! ", info->data_type);
		return -1;
	}
	if (need_get) {
		printf("Try to get Pipe(%d) data type(%d) \n", info->pipe_id,
			info->data_type);
		if(need_status == 1) {
		ret = hb_vio_get_data_conditional(info->pipe_id,
					info->data_type, &pym_buf, condition_time);
		} else {
		ret = hb_vio_get_data(info->pipe_id, info->data_type, &pym_buf);
		}
		printf("Get Pipe(%d) data type(%d) done !\n", info->pipe_id,
			info->data_type);
	}

	pym_buf_info_print(&pym_buf);
	if(need_dump) {
		if (ret < 0) {
		printf("pym date %d get failed.skip file dump\n", info->data_type);
		} else {
		printf("___dump buf to file begin___\n");
	for (int i = 0; i < 6; i++) {
		snprintf(file_name, sizeof(file_name),
			"pym_out_basic_layer_%08d_DS%d_%d_%d.yuv",
			pym_buf.pym_img_info.frame_id,
			i * 4,
			pym_buf.pym[i].width, pym_buf.pym[i].height);
		dumpToFile2plane(file_name, pym_buf.pym[i].addr[0],
							pym_buf.pym[i].addr[1],
							pym_buf.pym[i].width * pym_buf.pym[i].height,
							pym_buf.pym[i].width * pym_buf.pym[i].height / 2);

		for (int j = 0; j < 3; j++) {
			snprintf(file_name, sizeof(file_name),
				"pym_out_roi_layer_%08d_DS%d_%d_%d.yuv",
				pym_buf.pym_img_info.frame_id,
				i * 4 + j + 1,
				pym_buf.pym_roi[i][j].width, pym_buf.pym_roi[i][j].height);
			dumpToFile2plane(file_name, pym_buf.pym_roi[i][j].addr[0],
				pym_buf.pym_roi[i][j].addr[1],
				pym_buf.pym_roi[i][j].width * pym_buf.pym_roi[i][j].height,
				pym_buf.pym_roi[i][j].width * pym_buf.pym_roi[i][j].height / 2);
		}

		snprintf(file_name, sizeof(file_name),
			"pym_out_us_layer_%08d_US%d_%d_%d.yuv",
			pym_buf.pym_img_info.frame_id,
			i, pym_buf.us[i].width, pym_buf.us[i].height);
		dumpToFile2plane(file_name, pym_buf.us[i].addr[0],
							pym_buf.us[i].addr[1],
							pym_buf.us[i].width * pym_buf.us[i].height,
							pym_buf.us[i].width * pym_buf.us[i].height / 2);
		}

		printf("___dump buf to file end___\n");
	}
	}
	if (need_free) {
		if(ret < 0) {
			printf("Skip free due to get data type(%d) failed.\n",
													info->data_type);
		} else {
			printf("Try to free Pipe0 data type(%d) \n", info->data_type);
			ret = hb_vio_free_pymbuf(info->pipe_id, HB_VIO_PYM_DATA, &pym_buf);
			printf("Free Pipe0 data type(%d) done!\n", info->data_type);
		}
	}
	return 0;
}

int pym_feedback_process_func(work_info_t * info)
{
	int ret = 0;
	hb_vio_buffer_t buf = {0};
	pym_buffer_t pym_buf = {0};
	char file_name[100];

	pthread_mutex_lock(&g_pym_lock);
	printf("pipe(%u)pym_feedback_process start!\n", info->pipe_id);
	ret = hb_vio_get_data(info->pipe_id, HB_VIO_PYM_FEEDBACK_SRC_DATA, &buf);
	if (ret < 0) {
		printf("hb_vio_get_data PYM_FEEDBACK_SRC_DATA error!\n");
		pthread_mutex_unlock(&g_pym_lock);
		return -1;
	}
	printf("pipe(%u)pym_feedback buf.img_addr.w = %d, h = %d, stride = %d\n",
			info->pipe_id,
			buf.img_addr.width,
			buf.img_addr.height,
			buf.img_addr.stride_size);

	int img_in_fd = open(pym_feedback_pic, O_RDWR | O_CREAT, 0644);
	int size_0 = buf.img_addr.stride_size * buf.img_addr.height;
	int size_1 = size_0 / 2;
	printf("pipe(%u)pym_feedback size_0 = %d, size_1 = %d\n",
		info->pipe_id,
		size_0,
		size_1);
	if(img_in_fd < 0) {
		printf("pipe(%u)open image failed !\n", info->pipe_id);
		pthread_mutex_unlock(&g_pym_lock);
		return -1;
	}
	read(img_in_fd, buf.img_addr.addr[0], size_0);
	usleep(10 * 1000);
	read(img_in_fd, buf.img_addr.addr[1], size_1);
	usleep(10 * 1000);
	close(img_in_fd);

	// verify src buf
	if(need_dump) {
	dumpToFile2plane("pym_in.yuv", buf.img_addr.addr[0],
									buf.img_addr.addr[1], size_0, size_1);
	}
	printf("pipe(%u)hb_vio_pym_process begin\n", info->pipe_id);
	ret = hb_vio_run_pym(info->pipe_id, &buf);
	if(ret < 0) {
		hb_vio_free_pymbuf(info->pipe_id, HB_VIO_PYM_FEEDBACK_SRC_DATA, &buf);
		printf("pipe(%u)hb_vio_pym_process error\n", info->pipe_id);
		pthread_mutex_unlock(&g_pym_lock);
		return -1;
	}
	printf("pipe(%u)hb_vio_pym_process end\n", info->pipe_id);

	ret = hb_vio_get_data(info->pipe_id, HB_VIO_PYM_DATA, &pym_buf);
	if (ret < 0) {
		printf("pipe(%u) hb_vio_get_data PYM_DATA error\n", info->pipe_id);
		pthread_mutex_unlock(&g_pym_lock);
		return -1;
	}

	pym_buf_info_print(&pym_buf);

	if(need_dump) {
		printf("pipe(%u) dump buf to file begin\n", info->pipe_id);
	for (int i = 0; i < 6; i++) {
		snprintf(file_name, sizeof(file_name),
			"pym_out_basic_layer_DS%d_%d_%d.yuv",
			i * 4, pym_buf.pym[i].width, pym_buf.pym[i].height);
		dumpToFile2plane(file_name, pym_buf.pym[i].addr[0],
							pym_buf.pym[i].addr[1],
							pym_buf.pym[i].width * pym_buf.pym[i].height,
							pym_buf.pym[i].width * pym_buf.pym[i].height / 2);

		for (int j = 0; j < 3; j++) {
			snprintf(file_name, sizeof(file_name),
				"pym_out_roi_layer_DS%d_%d_%d.yuv",
				i * 4 + j + 1,
				pym_buf.pym_roi[i][j].width, pym_buf.pym_roi[i][j].height);
			dumpToFile2plane(file_name, pym_buf.pym_roi[i][j].addr[0],
				pym_buf.pym_roi[i][j].addr[1],
				pym_buf.pym_roi[i][j].width * pym_buf.pym_roi[i][j].height,
				pym_buf.pym_roi[i][j].width * pym_buf.pym_roi[i][j].height / 2);
		}

		snprintf(file_name, sizeof(file_name),
			"pym_out_us_layer_US%d_%d_%d.yuv",
			i, pym_buf.us[i].width, pym_buf.us[i].height);
		dumpToFile2plane(file_name, pym_buf.us[i].addr[0],
							pym_buf.us[i].addr[1],
							pym_buf.us[i].width * pym_buf.us[i].height,
							pym_buf.us[i].width * pym_buf.us[i].height / 2);
		}

		printf("pipe(%u) dump buf to file end\n", info->pipe_id);
	}

	if (need_free) {
		ret = hb_vio_free_pymbuf(info->pipe_id, HB_VIO_PYM_DATA, &pym_buf);
		ret = hb_vio_free_pymbuf(info->pipe_id, HB_VIO_PYM_FEEDBACK_SRC_DATA, &buf);
		if(ret < 0) {
			pthread_mutex_unlock(&g_pym_lock);
			return -1;
		}
	}
	pthread_mutex_unlock(&g_pym_lock);
	printf("pipe(%u)pym_feedback_process done.\n", info->pipe_id);
	return 0;
}

int raw_dump_func(work_info_t * info)
{
	int ret = 0, get_ctx_rt = 0;
	hb_vio_buffer_t *sif_raw = NULL;
	hb_vio_buffer_t *isp_yuv = NULL;
	char file_name[256] = { 0 };
	char ctx_file_name[100] = {0};
	struct timeval time_now = { 0 };
	struct timeval time_next = { 0 };
	int size = -1;
	int time_ms = 0;

	if (info->data_type != HB_VIO_SIF_RAW_DATA) {
		printf("Not pym data type (%d) ! ", info->data_type);
		return -1;
	}
	sif_raw = (hb_vio_buffer_t *) malloc(sizeof(hb_vio_buffer_t));
	if (!sif_raw) {
		printf("alloc sif_raw failed\n");
                return -1;
	}
	isp_yuv = (hb_vio_buffer_t *) malloc(sizeof(hb_vio_buffer_t));
	if (!isp_yuv) {
		printf("alloc isp_yuv failed\n");
		free(sif_raw);
		sif_raw = NULL;
		return -1;
	}
	memset(sif_raw, 0, sizeof(hb_vio_buffer_t));
	memset(isp_yuv, 0, sizeof(hb_vio_buffer_t));
	ptx.ptr = calloc(CTX_NODE_TOTAL_SIZE, 1);
        if (!ptx.ptr) {
                printf("alloc ptx mem failed\n");
                free(sif_raw);
                sif_raw = NULL;
                free(isp_yuv);
                isp_yuv = NULL;
                return -1;
        }
	gettimeofday(&time_now, NULL);
	printf("pipe(%u)raw dump time sec(%ld)usec(%ld) !\n", \
				info->pipe_id, time_now.tv_sec, time_now.tv_usec);

		time_ms = time_now.tv_sec * 1000 + time_now.tv_usec /1000;
		static int loop_count = 0;

	if (need_get) {
		printf("Try to get raw data Pipe(%u) data type(%d) (%s) \n",
		info->pipe_id, info->data_type, buf_name[info->data_type]);
		//you can set yuv NULL to dump RAW only.
		ret = hb_vio_raw_dump(info->pipe_id, sif_raw, isp_yuv);

		//ctx dump
		if (need_dump_ctx) {
			printf("Try to get ctx data!\n");
			get_ctx_rt = hb_isp_get_context(0, &ptx);
			if (get_ctx_rt < 0)
				printf("Dump context failed!\n");
		}
	if(need_dump) {
		normal_buf_info_print(sif_raw);
		size = sif_raw->img_addr.stride_size * sif_raw->img_addr.height;
		printf("raw stride_size(%u) w x h%u x %u  size %d\n",
			sif_raw->img_addr.stride_size,
			sif_raw->img_addr.width, sif_raw->img_addr.height, size);

		if(ret < 0) {
				printf("raw dump failed.\n");
		} else {
			loop_count = sif_raw->img_info.frame_id;
			printf("ctx dump, frame_id: %d\n", loop_count);
			snprintf(ctx_file_name, sizeof(ctx_file_name), "context_%04d.ctx", loop_count);
			if (ptx.ptr && need_dump_ctx) {
				dumpToFile(ctx_file_name, (char *)&ptx.crc16, sizeof(ptx.crc16));
				dumpToFile(ctx_file_name, ptx.ptr, CTX_NODE_TOTAL_SIZE);
			} else {
				printf("ctx error");
			}
		gettimeofday(&time_now, NULL);
		if (sif_raw->img_info.planeCount == 1) {
			snprintf(file_name, sizeof(file_name),
				"%s/sif_raw%04d.raw", raw_path, loop_count);
			dumpToFile(file_name, sif_raw->img_addr.addr[0], size);
		} else if (sif_raw->img_info.planeCount == 2) {
			snprintf(file_name, sizeof(file_name),
				"sif_raw_dol0_%d.raw", loop_count);
			dumpToFile(file_name, sif_raw->img_addr.addr[0], size);
			snprintf(file_name, sizeof(file_name),
				"sif_raw_dol1_%d.raw", loop_count);
			dumpToFile(file_name, sif_raw->img_addr.addr[1], size);
		} else if (sif_raw->img_info.planeCount == 3) {
			snprintf(file_name, sizeof(file_name),
				"sif_raw_dol0_%d.raw", loop_count);
			dumpToFile(file_name, sif_raw->img_addr.addr[0], size);
			snprintf(file_name, sizeof(file_name),
				"sif_raw_dol1_%d.raw", loop_count);
			dumpToFile(file_name, sif_raw->img_addr.addr[1], size);
			snprintf(file_name, sizeof(file_name),
				"sif_raw_dol2_%d.raw", loop_count);
			dumpToFile(file_name, sif_raw->img_addr.addr[2], size);
		} else {
			printf("pipe(%d)raw buf planeCount wrong !!!\n", info->pipe_id);
		}

		gettimeofday(&time_next, NULL);
		int time_cost = time_cost_ms(&time_now, &time_next);
		printf("dumpToFile raw cost time %d ms", time_cost);
		}

		//pls check here if isp dma was closed
		if(ret < 0) {
		printf("raw isp dump skip.\n");
		} else if (isp_yuv->img_addr.stride_size != 0) {
		//
		normal_buf_info_print(isp_yuv);
				size = isp_yuv->img_addr.stride_size * isp_yuv->img_addr.height;
		printf("yuv stride_size(%u) w x h%u x %u, size %d\n",
					isp_yuv->img_addr.stride_size,
					isp_yuv->img_addr.width, sif_raw->img_addr.height, size);
		// snprintf(file_name, sizeof(file_name),
		// 			"./isp_yuv_%d_index%d.yuv", time_ms,
		// 										isp_yuv->img_info.buf_index);
		snprintf(file_name, sizeof(file_name), "%s/isp_yuv_%04d.yuv", yuv_dir, loop_count);
		gettimeofday(&time_now, NULL);
		dumpToFile2plane(file_name, isp_yuv->img_addr.addr[0],
						isp_yuv->img_addr.addr[1], size, size/2);
		gettimeofday(&time_next, NULL);
		int time_cost = time_cost_ms(&time_now, &time_next);
		printf("dumpToFile yuv cost time %d ms", time_cost);
		} else {
			printf("isp yuv was not done, skip. \n");
		}
	}
		printf(" Pipe(%u) raw dump get done type(%d) (%s) \n",
		info->pipe_id, info->data_type, buf_name[info->data_type]);
	}
	if (ptx.ptr) {
		free(ptx.ptr);
		ptx.ptr = NULL;
	}

	if (sif_raw) {
		free(sif_raw);
		sif_raw = NULL;
	}
	if (isp_yuv) {
		free(isp_yuv);
		isp_yuv = NULL;
	}

	return 0;
}

#define FILE_NAME_CACHE 1024
int img_num=0;
char img_path[FILE_NAME_CACHE][128];
char img_name[FILE_NAME_CACHE][128];
char ct_p[FILE_NAME_CACHE][128];
int file_name_collect(char *path)
{
	struct dirent **namelist;
	int n, i = 0;
	int img_count = 0;
	int ctx_count = 0;

	n = scandir(path, &namelist, NULL, alphasort);
	if (n == -1) {
		perror("scandir");
		exit(EXIT_FAILURE);
	}
        if (n > FILE_NAME_CACHE) {
                for (i = n - 1; i > FILE_NAME_CACHE - 1; i--) {
                    free(namelist[i]);
                }
            n = FILE_NAME_CACHE;
        }
	for (i = 0; i < n; i++) {
		if (!strstr(namelist[i]->d_name, ".raw") && !strstr(namelist[i]->d_name, ".ctx")) {
			free(namelist[i]);
			continue;
		}
		if (strstr(namelist[i]->d_name, ".raw")) {
			strcpy(img_path[img_count],path);
			img_path[img_count][strlen(img_path[img_count])] = '/';
			strcat(img_path[img_count], namelist[i]->d_name);
			memcpy(img_name[img_count++], namelist[i]->d_name, strlen(namelist[i]->d_name)-sizeof("raw"));
		}

		if (strstr(namelist[i]->d_name, ".ctx")) {
			strcpy(ct_p[ctx_count],path);
			ct_p[ctx_count][strlen(ct_p[ctx_count])] = '/';
			strcat(ct_p[ctx_count++], namelist[i]->d_name);
		}
		free(namelist[i]);
	}
	if (ctx_count != img_count && need_dump_ctx)
		printf("error: ctx number[%d] connt match img number[%d]\n", ctx_count, img_count);
	img_num = img_count;
	free(namelist);

	return 0;
}

int raw_feedback_func(work_info_t * info, int loop_count)
{
	int ret = 0;
	hb_vio_buffer_t sif_raw_src = {0};
	hb_vio_buffer_t isp_yuv_dst = {0};
	char file_name[256] = { 0 };
	struct timeval time_now = { 0 };
	struct timeval time_next = { 0 };
	int size = -1;
	static int round = 1;
	static int idx = 0;
	static int file_parsed = 0;
	static int dump_flag = 1;
	FILE *inputFp = NULL;
	int i, dol_num = 1;

	ptx.ptr = calloc(CTX_NODE_TOTAL_SIZE, 1);
        if (!ptx.ptr) {
                printf("alloc ptx mem failed\n");
		goto err;
        }

	if (!file_parsed) {
		file_parsed = 1;
		file_name_collect(raw_path);
	}

	if (info->data_type != HB_VIO_SIF_FEEDBACK_SRC_DATA) {
		printf("Not sif feedback data type (%d) ! ", info->data_type);
		goto err;
	}

	// set sensor info
	if ((!need_cam) && (sensor_info_cfg_file != NULL) && (set_sensor_info)) {
		set_sensor_info--;
		int port_index = 0;
		for (int i = 0; i < PIPE_MAX; i++) {
			if (((pipe_mask >> i) & 0x01) == 1) {
				// set sensor info
				{
				char str[12] = {0};
				uint32_t dev_port = 0;
				sensor_turning_data_t turning_data;
				int ret = 0;

				if (sen_devfd < 0) {
					snprintf(str, sizeof(str), "/dev/port_%d", port_index);
					if ((sen_devfd = open(str, O_RDWR | O_CLOEXEC)) < 0) {
						printf("sensor_%d open fail\n", dev_port);
					}
				}

				turning_data.sensor_data.lines_per_second = lines_per_second;
				turning_data.sensor_data.exposure_time_long_max = exposure_time_long_max;
				turning_data.sensor_data.exposure_time_max = exposure_time_max;
				turning_data.sensor_data.analog_gain_max = analog_gain_max * 8192;
				turning_data.sensor_data.digital_gain_max = digital_gain_max * 8192;
				turning_data.sensor_data.exposure_time_min = exposure_time_min;
				turning_data.sensor_data.fps = 15;

				ret = ioctl(sen_devfd, SENSOR_TURNING_PARAM, &turning_data);

				if (ret < 0) {
					printf("sensor_%d ioctl fail %d\n", dev_port, ret);
				}

				printf("port %d line_per_second %d, exposure_time_long_max %d, exposure_time_max %d\n", port_index,
						lines_per_second, exposure_time_long_max, exposure_time_max);
				}
				port_index++;
			}
		}

	}

	if (need_get) {
		printf("Try to raw feedback src buf Pipe(%u) data type(%d) (%s) \n",
		info->pipe_id, info->data_type, buf_name[info->data_type]);

		// 0 get a empty raw buf from vio
		ret = hb_vio_get_data(info->pipe_id, HB_VIO_SIF_FEEDBACK_SRC_DATA,
			&sif_raw_src);
		if(ret < 0) {
			printf("hb_vio_get_data  feedback src buf failed.\n");
			goto err;
		}
		normal_buf_info_print(&sif_raw_src);
		dol_num += sif_raw_src.img_addr.addr[1] == 0 ? 0:1;
		dol_num += sif_raw_src.img_addr.addr[2] == 0 ? 0:1;
		printf("dol_num is %d\n", dol_num);
		if (idx >= img_num) {
			idx = 0;
			dump_flag = 0;
		}
		// ctx data read
		if (need_dump_ctx && ptx.ptr) {
			strcpy(file_name, ct_p[idx]);
			printf("ctx file name %s\n", file_name);

			inputFp = fopen(file_name, "rb");
			if (inputFp == NULL) {
				printf("ctx file(%s) open failed\n", file_name);
				goto err;
			}

			int count = fread(&ptx.crc16, 1, sizeof(ptx.crc16), inputFp);
			count = fread(ptx.ptr, 1, CTX_NODE_TOTAL_SIZE, inputFp);

			printf("ctx data size %d, crc16 is 0x%x \n", count, ptx.crc16);

			fclose(inputFp);

			ret = hb_isp_set_context(0, &ptx);
			if (ret < 0)
				printf("ctx feedback failed!\n");
		}

		if (ptx.ptr) {
			free(ptx.ptr);
			ptx.ptr = NULL;
		}

		for (i = 0; i < dol_num; i++) {
			strcpy(file_name, img_path[idx++]);
			printf("feedback file name%s\n", file_name);
			inputFp = fopen(file_name, "rb");
			if (inputFp == NULL) {
				printf("raw file(%s) open failed\n", file_name);
				return -1;
			}
			printf("raw img_addr.addr[%d] = %p, size %d \n", i,
				sif_raw_src.img_addr.addr[i],
				sif_raw_src.img_info.size[i]);
			int count = fread(sif_raw_src.img_addr.addr[i], 1,
					sif_raw_src.img_info.size[i], inputFp);
			HB_SYS_CacheFlush(sif_raw_src.img_addr.paddr[i],
						sif_raw_src.img_addr.addr[i],
						sif_raw_src.img_info.size[i]);
			printf("read raw file size %d \n", count);
			fclose(inputFp);
		}
		// 2 feedback to isp ddr in
		sif_raw_src.img_info.frame_id = loop_count;
		ret =  hb_vio_raw_feedback(info->pipe_id, &sif_raw_src, &isp_yuv_dst);

		// 3 dump isp output here
		if(ret < 0) {
			printf("raw feed back failed.\n");
			goto err;
		} else {
		//
		normal_buf_info_print(&isp_yuv_dst);
		//just dump once
		if(need_dump && dump_flag) {
		size = isp_yuv_dst.img_addr.stride_size * isp_yuv_dst.img_addr.height;

			printf("yuv stride_size(%u) w x h%u x %u, size %d\n",
				isp_yuv_dst.img_addr.stride_size,
				isp_yuv_dst.img_addr.width, isp_yuv_dst.img_addr.height, size);

		if (access(yuv_dir, 0))
			mkdir(yuv_dir, 0644);

		// snprintf(file_name, sizeof(file_name), "./yuv_dump_dir/%dx%d_%d.nv12",
		// 		isp_yuv_dst.img_addr.width,
		// 		isp_yuv_dst.img_addr.height,
		// 		round);
		snprintf(file_name, sizeof(file_name), "%s/%s.yuv", yuv_dir, img_name[idx-1]);
		HB_SYS_CacheInvalidate(isp_yuv_dst.img_addr.paddr[0],
					isp_yuv_dst.img_addr.addr[0], size);
		HB_SYS_CacheInvalidate(isp_yuv_dst.img_addr.paddr[1],
					isp_yuv_dst.img_addr.addr[1], size / 2);
		gettimeofday(&time_now, NULL);
		dumpToFile2plane(file_name, isp_yuv_dst.img_addr.addr[0],
						isp_yuv_dst.img_addr.addr[1], size, size/2);
		gettimeofday(&time_next, NULL);
		int time_cost = time_cost_ms(&time_now, &time_next);
			printf("dumpToFile yuv cost time %d ms", time_cost);
		round++;
		}
		}

		printf(" Pipe(%u) raw feedback get done type(%d) (%s) \n",
		info->pipe_id, info->data_type, buf_name[info->data_type]);
	}
	return 0;
err:
	if (ptx.ptr) {
		free(ptx.ptr);
		ptx.ptr = NULL;
	}
	printf("vio feedback failed. out here\n");
	return -1;
}

void *ipu_data_worker_thread(void *arg)
{
	work_info_t *info = (work_info_t *) arg;
	printf("pipe(%u) (%s)ipu_data_worker_thread Ready !\n",
		info->pipe_id, buf_name[info->data_type]);

	while (check_end()) {
		ipu_get_put_buf_func(info);
		usleep(20000);
	}

	printf("pipe(%u) (%s)ipu_data_worker_thread quit !\n",
		info->pipe_id, buf_name[info->data_type]);

	return NULL;
}

void *pym_data_worker_thread(void *arg)
{
	work_info_t *info = (work_info_t *) arg;

	printf("--- data_worker_thread Ready !--- \n");

	while (check_end()) {
		pym_get_put_buf_func(info);
		usleep(20000);
	}

	printf("pym_data_worker_thread quit !\n");
	return NULL;
}

void *raw_data_worker_thread(void *arg)
{
	work_info_t *info = (work_info_t *) arg;
	printf("raw data_worker_thread Ready !\n");

	while (check_end()) {
		raw_dump_func(info);
		usleep(20000);
	}

	printf("raw_data_worker_thread quit !\n");
	return NULL;
}

void *raw_feedback_worker_thread(void *arg)
{
    int cnt = 0;
	work_info_t *info = (work_info_t *) arg;
	printf("raw data_worker_thread Ready !\n");

	while (check_end()) {
        raw_feedback_func(info, cnt);
        cnt++;
		usleep(20000);
	}
	if (sen_devfd >= 0) {
		close(sen_devfd);
		sen_devfd = -1;
	}

	printf("raw_feedback_worker_thread quit !\n");
	return NULL;
}

void *pym_feedback_worker_thread(void *arg)
{
	work_info_t *info = (work_info_t *) arg;
	printf("pym data_worker_thread Ready !\n");

	while (check_end()) {
		pym_feedback_process_func(info);
		usleep(20000);
	}

	printf("pym_feedback_worker_thread quit !\n");
	return NULL;
}

void work_info_init(void)
{
	for (int i = 0; i < PIPE_MAX; i++) {
		for (int j = 0; j < HB_VIO_DATA_TYPE_MAX; j++) {
			work_info[i][j].pipe_id = i;
			work_info[i][j].data_type = j;
			work_info[i][j].running = 0;
		}
	}
	pthread_mutex_init(&g_pym_lock, NULL);
	pthread_mutex_init(&g_gdc_lock[0], NULL);
	pthread_mutex_init(&g_gdc_lock[1], NULL);
}

void wait_time_loop()
{
	do {
		usleep(10000);
	} while (check_end());
}

void callback_data_dump(callback_data_t *cb_data, int num)
{
	char file_name[100] = { 0 };
	hb_vio_buffer_t *buf;
	pym_buffer_t *pym_buf;
	int size = -1;

	switch (cb_data->cb_type) {
	case HB_VIO_IPU_US_CALLBACK:
	case HB_VIO_IPU_DS0_CALLBACK:
	case HB_VIO_IPU_DS1_CALLBACK:
	case HB_VIO_IPU_DS2_CALLBACK:
	case HB_VIO_IPU_DS3_CALLBACK:
	case HB_VIO_IPU_DS4_CALLBACK:
		buf = &cb_data->ipu_buffer;
		size = buf->img_addr.stride_size * buf->img_addr.height;

		printf("yuv stride_size(%u) w x h%u x %u, size %d\n",
				buf->img_addr.stride_size,
				buf->img_addr.width, buf->img_addr.height, size);
		snprintf(file_name, sizeof(file_name),
				"./dump_pics/ipu_callback_loop%d_type%d_index%d.yuv", num,
				cb_data->cb_type, buf->img_info.buf_index);
		dumpToFile2plane(file_name, buf->img_addr.addr[0],
				buf->img_addr.addr[1], size, size/2);

		break;
	case HB_VIO_PYM_CALLBACK:
		pym_buf = &cb_data->pym_buffer;

		for (int i = 0; i < 6; i++) {
			snprintf(file_name, sizeof(file_name),
					"./dump_pics/pym_out_basic_layer_%d_DS%d_%d_%d.yuv",
					pym_buf->pym_img_info.frame_id,
					i * 4,
					pym_buf->pym[i].width, pym_buf->pym[i].height);
			dumpToFile2plane(file_name, pym_buf->pym[i].addr[0],
					pym_buf->pym[i].addr[1],
					pym_buf->pym[i].width * pym_buf->pym[i].height,
					pym_buf->pym[i].width * pym_buf->pym[i].height / 2);

			for (int j = 0; j < 3; j++) {
				snprintf(file_name, sizeof(file_name),
						"./dump_pics/pym_out_roi_layer_%d_DS%d_%d_%d.yuv",
						pym_buf->pym_img_info.frame_id,
						i * 4 + j + 1,
						pym_buf->pym_roi[i][j].width, pym_buf->pym_roi[i][j].height);
				dumpToFile2plane(file_name, pym_buf->pym_roi[i][j].addr[0],
						pym_buf->pym_roi[i][j].addr[1],
						pym_buf->pym_roi[i][j].width * pym_buf->pym_roi[i][j].height,
						pym_buf->pym_roi[i][j].width * pym_buf->pym_roi[i][j].height / 2);
			}

			snprintf(file_name, sizeof(file_name),
					"./dump_pics/pym_out_us_layer_%d_US%d_%d_%d.yuv",
					pym_buf->pym_img_info.frame_id,
					i, pym_buf->us[i].width, pym_buf->us[i].height);
			dumpToFile2plane(file_name, pym_buf->us[i].addr[0],
					pym_buf->us[i].addr[1],
					pym_buf->us[i].width * pym_buf->us[i].height,
					pym_buf->us[i].width * pym_buf->us[i].height / 2);
		}
		break;
	default:
		printf("invaild callback type.\n");
		break;
	}
}

void *callback_data_work_thread()
{
	struct list_head *this, *next;
	callback_data_t *cb_data;
	int cnt = 0;

	while (cb_list.run_enable) {
		pthread_mutex_lock(&cb_list.cb_mutex);

		list_for_each_safe(this, next, &cb_list.cb_process) {
			cb_data = (callback_data_t *)this;

			if (need_dump) {
				callback_data_dump(cb_data, cnt);
				printf("callback dump done, now count(%d)\n", cnt);
			}

			vio_list_del(&cb_data->node);
			if (need_free) {
				if (cb_data->cb_type >= HB_VIO_IPU_DS0_CALLBACK &&
						cb_data->cb_type <= HB_VIO_IPU_US_CALLBACK) {
					normal_buf_info_print(&cb_data->ipu_buffer);
					hb_vio_free_ipubuf(cb_data->pipe_id, &cb_data->ipu_buffer);
					printf("callback ipu buffer free done\n");
				} else if (cb_data->cb_type == HB_VIO_PYM_CALLBACK) {
					pym_buf_info_print(&cb_data->pym_buffer);
					hb_vio_free_pymbuf(cb_data->pipe_id, HB_VIO_PYM_DATA, &cb_data->pym_buffer);
					printf("callback pym buffer free done\n");
				} else {
					printf("error callback data type\n");
				}
			}
			free(cb_data);
			cnt++;
		}

		pthread_mutex_unlock(&cb_list.cb_mutex);

		usleep(20000);
	}

	printf("callback work thread exit\n");
	pthread_exit((void *)1);
}

static void vio_data_receive(uint32_t pipe_id, uint32_t info, void *data,
			     void *userdata)
{
	callback_data_t *cb_data;
	printf("pipe_id(%u), callback type(%u) data receiverd.\n", pipe_id, info);

	switch (info) {
	case HB_VIO_IPU_US_CALLBACK:
	case HB_VIO_IPU_DS0_CALLBACK:
	case HB_VIO_IPU_DS1_CALLBACK:
	case HB_VIO_IPU_DS2_CALLBACK:
	case HB_VIO_IPU_DS3_CALLBACK:
	case HB_VIO_IPU_DS4_CALLBACK:
		if(data != NULL) {
			cb_data = malloc(sizeof(callback_data_t));
			memcpy(&cb_data->ipu_buffer, (hb_vio_buffer_t *)data,
						sizeof(hb_vio_buffer_t));
			cb_data->cb_type = info;
			vio_list_add_tail(&cb_data->node, &cb_list.cb_process);
		}
		break;
	case HB_VIO_PYM_CALLBACK:
		if(data != NULL) {
			cb_data = malloc(sizeof(callback_data_t));
			memcpy(&cb_data->pym_buffer, (pym_buffer_t *)data,
						sizeof(pym_buffer_t));
			cb_data->cb_type = info;
			vio_list_add_tail(&cb_data->node, &cb_list.cb_process);
		}
		break;
	default:
		printf("invaild callback type.\n");
		break;
	}
}

void callback_list_init()
{
	int cb_en_status = -1;
	int cb_mesg = -1;
	int cb_info_type = -1;
	int ret;
	int callback_en = 0;

	printf("callback init in\n");

	for (int pipe_id = 0; pipe_id < pipe_num; pipe_id++) {
		if(data_type & DS0_ENABLE) {
			cb_mesg = HB_VIO_IPU_DS0_CB_MSG;
			cb_info_type = HB_VIO_IPU_DS0_CALLBACK;
			callback_en = 1;

			printf("callback set param data type(%d), info_type(%d)\n",
										cb_mesg, cb_info_type);
			hb_vio_set_param(pipe_id, HB_VIO_CALLBACK_ENABLE, (void *)&cb_mesg);
			hb_vio_set_callbacks(pipe_id, cb_info_type, vio_data_receive);
		}
		if (data_type & DS1_ENABLE) {
			cb_mesg = HB_VIO_IPU_DS1_CB_MSG;
			cb_info_type = HB_VIO_IPU_DS1_CALLBACK;
			callback_en = 1;

			printf("callback set param data type(%d), info_type(%d)\n",
										cb_mesg, cb_info_type);
			hb_vio_set_param(pipe_id, HB_VIO_CALLBACK_ENABLE, (void *)&cb_mesg);
			hb_vio_set_callbacks(pipe_id, cb_info_type, vio_data_receive);
		}
		if (data_type & DS2_ENABLE) {
			cb_mesg = HB_VIO_IPU_DS2_CB_MSG;
			cb_info_type = HB_VIO_IPU_DS2_CALLBACK;
			callback_en = 1;

			printf("callback set param data type(%d), info_type(%d)\n",
										cb_mesg, cb_info_type);
			hb_vio_set_param(pipe_id, HB_VIO_CALLBACK_ENABLE, (void *)&cb_mesg);
			hb_vio_set_callbacks(pipe_id, cb_info_type, vio_data_receive);
		}
		if (data_type & DS3_ENABLE) {
			cb_mesg = HB_VIO_IPU_DS3_CB_MSG;
			cb_info_type = HB_VIO_IPU_DS3_CALLBACK;
			callback_en = 1;

			printf("callback set param data type(%d), info_type(%d)\n",
										cb_mesg, cb_info_type);
			hb_vio_set_param(pipe_id, HB_VIO_CALLBACK_ENABLE, (void *)&cb_mesg);
			hb_vio_set_callbacks(pipe_id, cb_info_type, vio_data_receive);
		}
		if (data_type & DS4_ENABLE) {
			cb_mesg = HB_VIO_IPU_DS4_CB_MSG;
			cb_info_type = HB_VIO_IPU_DS4_CALLBACK;
			callback_en = 1;

			printf("callback set param data type(%d), info_type(%d)\n",
										cb_mesg, cb_info_type);
			hb_vio_set_param(pipe_id, HB_VIO_CALLBACK_ENABLE, (void *)&cb_mesg);
			hb_vio_set_callbacks(pipe_id, cb_info_type, vio_data_receive);
		}
		if (data_type & US_ENABLE) {
			cb_mesg = HB_VIO_IPU_US_CB_MSG;
			cb_info_type = HB_VIO_IPU_US_CALLBACK;
			callback_en = 1;

			printf("callback set param data type(%d), info_type(%d)\n",
										cb_mesg, cb_info_type);
			hb_vio_set_param(pipe_id, HB_VIO_CALLBACK_ENABLE, (void *)&cb_mesg);
			hb_vio_set_callbacks(pipe_id, cb_info_type, vio_data_receive);
		}
		if (data_type & PYM_ENABLE) {
			cb_mesg = HB_VIO_PYM_CB_MSG;
			cb_info_type = HB_VIO_PYM_CALLBACK;
			callback_en = 1;

			printf("callback set param data type(%d), info_type(%d)\n",
										cb_mesg, cb_info_type);
			hb_vio_set_param(pipe_id, HB_VIO_CALLBACK_ENABLE, (void *)&cb_mesg);
			hb_vio_set_callbacks(pipe_id, cb_info_type, vio_data_receive);
		}

		if (callback_en) {
			hb_vio_get_param(pipe_id, HB_VIO_CALLBACK_ENABLE, (void *)&cb_en_status);
			printf("callback set param done, cb_en_status = %d\n", cb_en_status);
		} else {
			printf("callback type error, skip callback\n");
			return;
		}
	}

	pthread_mutex_init(&cb_list.cb_mutex, NULL);

	vio_init_list_head(&cb_list.cb_process);

	cb_list.run_enable = 1;

	ret = pthread_create(&cb_list.cb_pid, NULL,
						(void *)callback_data_work_thread, NULL);
	if (ret < 0) {
		printf("callback thread start fail\n");
		return;
	}
}

void callback_list_deinit()
{
	void *pret;

	cb_list.run_enable = 0;

	pthread_join(cb_list.cb_pid, &pret);

	return;
}

void intHandler(int dummy)
{
#ifdef VIO_MONTOR
		vio_montor_stop();
#endif

	if (need_display) {
	    hb_disp_stop();
		hb_disp_close();
	}

	if(need_cam) {
		int port_index = 0;
		for (int i = 0; i < PIPE_MAX; i++) {
			if (((pipe_mask >> i) & 0x01) == 1) {
				hb_cam_stop(port_index);
				port_index++;
			}
		}
	}

	if (pipe_num > 0) {
		for (int i = 0; i < PIPE_MAX; i++) {
			if (((pipe_mask >> i) & 0x01) == 1) {
				printf("Test hb_vio_stop_pipeline(%d).\n", i);
				hb_vio_stop_pipeline(i);
			}
		}
	}

	if(need_cam) {
		hb_cam_deinit(need_cam);//camera config index now pass as need_cam
		if(need_clk) {
			hb_cam_disable_mclk(mipiIdx);
		}
	}

	hb_vio_deinit();
	printf("Test hb_vio_deinit done\n");
	_exit(0);
}

void *mode_switch_thread(void *arg)
{
	int ret = 0;
	int pipeid_a, pipeid_b;
	char str[4];
	char *linear_path = "/etc/cam/lib_ar0233_linear.so";
	char *pwl_path = "/etc/cam/lib_ar0233_pwl.so";

	camera_info_table_t linear;
	camera_info_table_t pwl;

	memset(&linear, 0, sizeof(linear));
	memset(&pwl, 0, sizeof(pwl));

	strcpy(linear.name, "linear");
	linear.mode = 1;
	linear.bit_width = 12;
	linear.cfa_pattern = 0;
	strcpy((char *)(linear.calib_path), linear_path);

	strcpy(pwl.name, "pwl");
	pwl.mode = 5;
	pwl.bit_width = 12;
	pwl.cfa_pattern = 1;
	strcpy((char *)(pwl.calib_path), pwl_path);

	while (check_end()) {

		printf("input switch cmd:>\n");

		scanf("%s", str);
        pipeid_a = str[0] - 48;
        pipeid_b = str[1] - 48;

		if (pipeid_a < 0 || pipeid_a > 8) {
			printf("pipeid_a %d is invalid.\n", pipeid_a);
			continue;
		}

		if (pipeid_b < 0 || pipeid_b > 8) {
			printf("pipeid_b %d is invalid.\n", pipeid_b);
			continue;
		}

		printf("pipe %d, pipe %d exchange mode\n", pipeid_a, pipeid_b);
		ret = hb_cam_dynamic_switch_mode(pipeid_a, &linear);
		if (ret < 0) {
			printf("pipe %d, switch to mode %d failed.\n", pipeid_a, linear.mode);
			continue;
		}
		ret = hb_isp_iridix_ctrl(pipeid_a, pipeid_b, 1);
		if (ret < 0) {
			printf("pipe %d, pipe %d iridix ctrl failed.\n", pipeid_a, pipeid_b);
			continue;
		}
		ret = hb_cam_dynamic_switch_mode(pipeid_b, &pwl);
		if (ret < 0) {
			printf("pipe %d, switch to mode %d failed.\n", pipeid_a, pwl.mode);
		}

		usleep(50 * 1000);
	}

	return NULL;
}

int main(int argc, char *argv[])
{
	printf("Hello, World! vio tool test start !\n");

	int ret = 0;
	int fb_count = 0;
	int ae_share_src = 0;
	int ae_share_dst = 0;

	parse_opts(argc, argv);

	if (cam_cfg_file == NULL)
		cam_cfg_file = "/etc/vio_tool/hb_x3player.json";

	if (argc < 2) {
		print_usage(argv[0]);
		printf("leave, World! \n");
		exit(1);
	}

	if(pipe_num > 8) {
		printf("Now only support total 8 pipe. 1~8 now(%d).\n", pipe_num);
		exit(1);
	}

	if (raw_path == NULL)
		raw_path = ".";
	
	if (yuv_dir == NULL)
		yuv_dir = ".";

	//test run time
	start_time = time(NULL);
	end_time = start_time + run_time;

	work_info_init();

	//all pipe config here
	if (vio_cfg_file != NULL) {
		ret = hb_vio_init(vio_cfg_file);
		if (ret < 0) {
			printf("vio init fail\n");
			return -1;
		}
	} else {
		printf("null config was set. Skip test.\n");
		exit(1);
	}

	if (need_callback) {
		callback_list_init();
	}

	if(need_cam) {
		if(need_clk) {
			ret = hb_cam_set_mclk(mipiIdx, SENSOR_MCLK);
			if (ret < 0) {
				printf("hb_cam_set_mclk  fail\n");
				return ret;
			}
			ret = hb_cam_enable_mclk(mipiIdx);
			if (ret < 0) {
				printf("hb_cam_enable_mclk fail\n");
				return ret;
			}
		}
		ret = hb_cam_init(cam_index, cam_cfg_file);
		if (ret < 0) {
			printf("cam init fail\n");
			hb_vio_deinit();
			return -1;
		}
	}

	if (pipe_mask == 0) {
		for (int i = 0; i < pipe_num; i++)
			pipe_mask |= (1 << i);
	}
	if (pipe_mask == 0) {
		printf("pipe_mask init err.\n");
		hb_vio_deinit();
		hb_cam_deinit(cam_index);
		return -1;
	}
	printf("Calculate or user config pipe_mask 0x%x.\n", pipe_mask);

	for (int i = 0; i < pipe_num; i++) {
		if (((pipe_mask >> i) & 0x01) == 0)
			continue;
		if (gdc_cfg != NULL) {
			ret = gdc_cfg_bin_update(i, gdc_cfg);
			if (ret < 0) {
				printf("gdc_cfg_bin_update failed\n");
				hb_vio_deinit();
				hb_cam_deinit(cam_index);
				return -1;
			}
		}
	}

	if (need_share_ae > 0) {
		ae_share_src = need_share_ae / 10;
		ae_share_dst = need_share_ae % 10;
	}

	printf("Test hb_vio_start_pipeline.\n");
	if (pipe_num > 0) {
		for (int i = 0; i < pipe_num; i++) {
			if ((i == ae_share_dst) && (ae_share_dst != ae_share_src)) {
			    hb_cam_share_ae(ae_share_src, ae_share_dst);
				continue;
			}
		}
	}

	for (int i = 0; i < PIPE_MAX; i++) {
		if (((pipe_mask >> i) & 0x01) == 0)
			continue;
		printf("Test hb_vio_start_pipeline(%d).\n", i);
		ret = hb_vio_start_pipeline(i);
		if (ret < 0) {
			printf("vio start fail, do cam&vio deinit.\n");
			hb_cam_deinit(cam_index);
			hb_vio_deinit();
			return -1;
		}
	}

	if (need_display) {
		ret = hb_disp_init_cfg("/etc/iar/iar_xj3_hdmi_1080p.json");
		if (ret) {
			printf("disp start fail, cfs is no existence.\n");
		} else {
			ret = hb_disp_start();
			if (ret) {
				printf("disp start fail.\n");
				hb_disp_close();
			}
			hb_disp_layer_off(2);
		}
	}

	if(need_cam) {
		int port_index = 0;
		for (int i = 0; i < PIPE_MAX; i++) {
			if (((pipe_mask >> i) & 0x01) == 1) {
				ret = hb_cam_start(port_index);
				if (ret < 0) {
					printf("cam start fail, do cam&vio deinit.\n");
					hb_cam_stop(port_index);
					hb_cam_deinit(cam_index);
					hb_vio_deinit();
					return -1;
				}
				port_index++;
			}
		}
	}

#ifdef VIO_MONTOR
	if (hobotplayer_cfg_file != NULL) {
		ret = vio_montor_start(hobotplayer_cfg_file, need_gdc_fb, 1, 1);
	} else {
		ret = vio_montor_start("/etc/vio_tool/dump_config.json", 0, 1, 1);
	}
#endif

	// get sensor info
	if ((!need_cam) && (sensor_info_cfg_file != NULL)) {
		set_sensor_info = 20;
		ret = sensor_info_config_init(sensor_info_cfg_file);
		if (ret < 0) {
			set_sensor_info = 0;
		}
	}

	pthread_t switch_thid;

	if (mode_switch) {
		ret = pthread_create(&switch_thid, NULL, (void *)mode_switch_thread, NULL);
		if (ret != 0) {
			printf("mode_switch_thread thread start error.\n");
		}
	}
    if (ae_test) {
        ret = hb_isp_dev_init();
        if (ret < 0) {
            printf("hb_isp_dev_init fail!!!\n");
            return ret;
        }
        ret = ae_test_func(&work_info[0][HB_VIO_IPU_DS2_DATA]);
        if (ret < 0) {
            printf("ae_test_func is error!!!\n");
        }
    }
	if (need_m_thread) {
		for (int id = 0; id < PIPE_MAX; id++) {
			if (((pipe_mask >> id) & 0x01) == 0)
				continue;
			if(data_type & DS0_ENABLE) {
			ret = pthread_create(&work_info[id][HB_VIO_IPU_DS0_DATA].thid,
						NULL,
					   (void *)ipu_data_worker_thread,
					   (void *)(&work_info[id][HB_VIO_IPU_DS0_DATA]));
			printf("pipe(%d)Test work DS0 thread---running.\n", id);
			}
			if(data_type & DS1_ENABLE) {
			ret = pthread_create(&work_info[id][HB_VIO_IPU_DS1_DATA].thid,
						NULL,
					   (void *)ipu_data_worker_thread,
					   (void *)(&work_info[id][HB_VIO_IPU_DS1_DATA]));
			printf("pipe(%d)Test work DS1 thread---running.\n", id);
			}
			if(data_type & DS2_ENABLE) {
			ret = pthread_create(&work_info[id][HB_VIO_IPU_DS2_DATA].thid,
						NULL,
					   (void *)ipu_data_worker_thread,
					   (void *)(&work_info[id][HB_VIO_IPU_DS2_DATA]));
			printf("pipe(%d)Test work DS2 thread---running.\n", id);
			}
			if(data_type & DS3_ENABLE) {
			ret = pthread_create(&work_info[id][HB_VIO_IPU_DS3_DATA].thid,
						NULL,
					   (void *)ipu_data_worker_thread,
					   (void *)(&work_info[id][HB_VIO_IPU_DS3_DATA]));
			printf("pipe(%d)Test work DS3 thread---running.\n", id);
			}
			if(data_type & DS4_ENABLE) {
			ret = pthread_create(&work_info[id][HB_VIO_IPU_DS4_DATA].thid,
						NULL,
					   (void *)ipu_data_worker_thread,
					   (void *)(&work_info[id][HB_VIO_IPU_DS4_DATA]));
			printf("pipe(%d)Test work DS4 thread---running.\n", id);
			}
			if(data_type & US_ENABLE) {
			ret = pthread_create(&work_info[id][HB_VIO_IPU_US_DATA].thid,
						NULL,
					   (void *)ipu_data_worker_thread,
					   (void *)(&work_info[id][HB_VIO_IPU_US_DATA]));
			printf("pipe(%d)Test work US thread---running.\n", id);
			}
			if(data_type & PYM_ENABLE) {
			ret = pthread_create(&work_info[id][HB_VIO_PYM_DATA].thid,
						NULL,
					   (void *)pym_data_worker_thread,
					   (void *)(&work_info[id][HB_VIO_PYM_DATA]));
			printf("pipe(%d)Test work PYM thread---running.\n", id);
			}
			//raw dump & feedback only for debug mode.
			//raw dump can not run with raw feedback together.
			if(data_type & RAW_ENABLE) {
			ret = pthread_create(&work_info[id][HB_VIO_SIF_RAW_DATA].thid,
						NULL,
					   (void *)raw_data_worker_thread,
					   (void *)(&work_info[id][HB_VIO_SIF_RAW_DATA]));
			printf("pipe(%d)Test work raw_data thread---running.\n", id);
			}
			if(data_type & RAW_FEEDBACK_ENABLE) {
			ret = pthread_create(
						&work_info[id][HB_VIO_SIF_FEEDBACK_SRC_DATA].thid,
						NULL,
					   (void *)raw_feedback_worker_thread,
					   (void *)(&work_info[id][HB_VIO_SIF_FEEDBACK_SRC_DATA]));
			printf("pipe(%d)Test work raw_feedback thread---running.\n", id);
			}
			if (need_pym_feedback) {
			ret = pthread_create(
						&work_info[id][HB_VIO_PYM_FEEDBACK_SRC_DATA].thid,
						NULL,
					   (void *)pym_feedback_worker_thread,
					   (void *)(&work_info[id][HB_VIO_PYM_FEEDBACK_SRC_DATA]));
			printf("pipe(%d)Test work pym_feedback thread---running.\n", id);
			}
		}
	} else if (loop > 0) {
		if(data_type & RAW_FEEDBACK_ENABLE && need_dump_ctx) {
			for (int id = 0; id < PIPE_MAX; id++) {
				if (((pipe_mask >> id) & 0x01) == 0)
					continue;
				//freeze isp_fw before feedback
				HB_ISP_SetFWState(work_info[id][HB_VIO_SIF_FEEDBACK_SRC_DATA].pipe_id, ISP_FW_STATE_FREEZE);
			}
		}
		// if (data_type & RAW_ENABLE) {
		// 	int dump_start = 0;
		// 	while (1) {
		// 		printf("press 1 to dump: ");
		// 		scanf("%d", &dump_start);
		// 		if (dump_start == 1) break;
		// 	}
		// }

		fb_count = loop;
		while (loop > 0) {
			for (int id = 0; id < PIPE_MAX; id++) {
				if (((pipe_mask >> id) & 0x01) == 0)
					continue;
				if(data_type & RAW_ENABLE) {
				printf("pipe(%d)data_type(%d)Get raw data in loop(%d)\n",
					id, data_type, loop);
				raw_dump_func(&work_info[id][HB_VIO_SIF_RAW_DATA]);
				}
				if(data_type & RAW_FEEDBACK_ENABLE) {
				printf("pipe(%d)data_type(%d)raw feedback data in loop(%d)\n",
					id, data_type, loop);
				raw_feedback_func(&work_info[id][HB_VIO_SIF_FEEDBACK_SRC_DATA],
									fb_count - loop);
				}
				if(data_type & DS0_ENABLE) {
				printf("pipe(%d)Get DS0 data in loop operation. loop(%d)\n",
					id, loop);
				ipu_get_put_buf_func(&work_info[id][HB_VIO_IPU_DS0_DATA]);
				}
				if(data_type & DS1_ENABLE) {
				printf("pipe(%d)Get DS1 data in loop operation. loop(%d)\n",
					id, loop);
				ipu_get_put_buf_func(&work_info[id][HB_VIO_IPU_DS1_DATA]);
				}
				if(data_type & DS2_ENABLE) {
				printf("pipe(%d)Get DS2 data in loop operation. loop(%d)\n",
					id, loop);
				ipu_get_put_buf_func(&work_info[id][HB_VIO_IPU_DS2_DATA]);
				}
				if(data_type & DS3_ENABLE) {
				printf("pipe(%d)Get DS3 data in loop operation. loop(%d)\n",
					id, loop);
				ipu_get_put_buf_func(&work_info[id][HB_VIO_IPU_DS3_DATA]);
				}
				if(data_type & DS4_ENABLE) {
				printf("pipe(%d)Get DS4 data in loop operation. loop(%d)\n",
					id, loop);
				ipu_get_put_buf_func(&work_info[id][HB_VIO_IPU_DS4_DATA]);
				}
				if(data_type & US_ENABLE) {
				printf("pipe(%d)Get US data in loop operation. loop(%d)\n",
					id, loop);
				ipu_get_put_buf_func(&work_info[id][HB_VIO_IPU_US_DATA]);
				}
				if(data_type & PYM_ENABLE) {
				printf("pipe(%d)data_type(%d)Get PYM data in loop(%d)\n",
					id, data_type, loop);
				pym_get_put_buf_func(&work_info[id][HB_VIO_PYM_DATA]);
				}
				if (need_pym_feedback) {
				pym_feedback_process_func(&work_info[id][HB_VIO_PYM_FEEDBACK_SRC_DATA]);
				}
				if (need_gdc_fb >= 0) {
				gdc_feedback_func(&work_info[id][HB_VIO_GDC_FEEDBACK_SRC_DATA]);
				}
			}
			loop--;
		}

		if(data_type & RAW_FEEDBACK_ENABLE) {
			for (int id = 0; id < PIPE_MAX; id++) {
				if (((pipe_mask >> id) & 0x01) == 0)
					continue;
				//reset isp_fw after feedback
				HB_ISP_SetFWState(work_info[id][HB_VIO_SIF_FEEDBACK_SRC_DATA].pipe_id, ISP_FW_STATE_RUN);
			}
		}
		printf("Loop Test done quit.\n");
	}

	wait_time_loop();
	printf("Test time is over, quit.\n");

	if (mode_switch) {
		pthread_join(switch_thid, NULL);
	}

	if (need_m_thread) {
		for (int id = 0; id < PIPE_MAX; id++) {
			if (((pipe_mask >> id) & 0x01) == 0)
				continue;
			if(data_type & DS0_ENABLE) {
			pthread_join(work_info[id][HB_VIO_IPU_DS0_DATA].thid, NULL);
			printf("pipe(%d) DS0 test thread quit done.\n", id);
			}
			if(data_type & DS1_ENABLE) {
			pthread_join(work_info[id][HB_VIO_IPU_DS1_DATA].thid, NULL);
			printf("pipe(%d) DS1 test thread quit done.\n", id);
			}
			if(data_type & DS2_ENABLE) {
			pthread_join(work_info[id][HB_VIO_IPU_DS2_DATA].thid, NULL);
			printf("pipe(%d) DS2 test thread quit done.\n", id);
			}
			if(data_type & DS3_ENABLE) {
			pthread_join(work_info[id][HB_VIO_IPU_DS3_DATA].thid, NULL);
			printf("pipe(%d) DS3 test thread quit done.\n", id);
			}
			if(data_type & DS4_ENABLE) {
			pthread_join(work_info[id][HB_VIO_IPU_DS4_DATA].thid, NULL);
			printf("pipe(%d) DS4 test thread quit done.\n", id);
			}
			if(data_type & US_ENABLE) {
			pthread_join(work_info[id][HB_VIO_IPU_US_DATA].thid, NULL);
			printf("pipe(%d) US test thread quit done.\n", id);
			}
			if(data_type & PYM_ENABLE) {
			pthread_join(work_info[id][HB_VIO_PYM_DATA].thid, NULL);
			printf("pipe(%d) PYM test thread quit done.\n", id);
			}
			if(data_type & RAW_ENABLE) {
			pthread_join(work_info[id][HB_VIO_SIF_RAW_DATA].thid, NULL);
			printf("pipe(%d)Test work raw_data thread quit done..\n", id);
			}
			if(data_type & RAW_FEEDBACK_ENABLE) {
			pthread_join(work_info[id][HB_VIO_SIF_FEEDBACK_SRC_DATA].thid,
						NULL);
			printf("pipe(%d)Test work raw_feedback thread quit done.\n", id);
			}
			if (need_pym_feedback) {
			pthread_join(work_info[id][HB_VIO_PYM_FEEDBACK_SRC_DATA].thid,
						NULL);
			printf("pipe(%d)Test work pym_feedback thread quit done.\n", id);
			}
		}
	}

#ifdef VIO_MONTOR
	vio_montor_stop();
#endif

	if (need_display) {
	        hb_disp_stop();
	        hb_disp_close();
	}

	if(need_cam) {
		int port_index = 0;
		for (int i = 0; i < PIPE_MAX; i++) {
			if (((pipe_mask >> i) & 0x01) == 1) {
				printf("Test hb_cam_stop(%d).\n", port_index);
				hb_cam_stop(port_index);
				port_index++;
			}
		}
	}
	sleep(1);
	for (int i = 0; i < PIPE_MAX; i++) {
		if (((pipe_mask >> i) & 0x01) == 0)
			continue;
		printf("Test hb_vio_stop_pipeline(%d).\n", i);
		hb_vio_stop_pipeline(i);
	}

	if(need_cam) {
		hb_cam_deinit(cam_index);
		if(need_clk) {
			hb_cam_disable_mclk(mipiIdx);
		}
	}
	printf("Test done, Do hb_vio_deinit.\n");
	hb_vio_deinit();
	printf("Test done success\n");

	return 0;
}
