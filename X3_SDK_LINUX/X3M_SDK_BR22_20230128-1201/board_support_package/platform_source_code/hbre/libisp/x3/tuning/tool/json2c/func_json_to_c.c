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
#include <stdarg.h>
#include <errno.h>
#include <stdint.h>
#include <stddef.h>
#include <ctype.h>
#include "cJSON.h"

char *dynamic_file;
char *static_file;
char *libname;

#define MAX_CHANGE_LIST 11
static char *change_lut_to_s16[MAX_CHANGE_LIST] = {
	"CALIBRATION_DEMOSAIC_NP_OFFSET",
	"CALIBRATION_SHARPEN_FR",
	"CALIBRATION_SHARP_ALT_D",
	"CALIBRATION_SHARP_ALT_DU",
	"CALIBRATION_SHARP_ALT_UD",
	"CALIBRATION_CNR_UV_DELTA12_SLOPE",
	"CALIBRATION_SINTER_STRENGTH",
	"CALIBRATION_SINTER_STRENGTH1",
	"CALIBRATION_SINTER_THRESH1",
	"CALIBRATION_SINTER_THRESH4",
	"CALIBRATION_TEMPER_STRENGTH",
};

int check_lut_inlist(char *name)
{
	int ret = 0;
	uint32_t i = 0;

	char *p0 = name;
	char *p1 = NULL;
	while(i < MAX_CHANGE_LIST) {
		p0 = name;
		p1 = change_lut_to_s16[i];
		while ((*p0 != '\0') && (*p1 != '\0')) {
			if (*p0 != *p1) {
				ret = 0;
				break;
			}
			p0++;
			p1++;
		}
		if ((*p0 == '\0') && (*p1 == '\0')) {
			return 1;
			break;
		}
		i++;
	};

	return ret;
}

static inline char *mstrlwr(char *str)
{
	if (str == NULL) {
		return NULL;
	}
	char *p = str;
	while(*p != '\0') {
		if ((*p) >= 'A' && (*p) <= 'Z') {
			*p = (*p) + 0x20;
		}
		p++;
	}

	return str;
}

int create_calib_file(cJSON *root, FILE *fp_c)
{
	cJSON *node, *sub_array;
	uint32_t array_size = 0;
	uint32_t array_index = 0;
	cJSON *task_array = NULL;
	uint32_t task_size = 0;
	cJSON *tasklist;
	uint32_t width;
	uint32_t rows;
	uint32_t cols;
	uint32_t count;
	char *name;
	char *tempdata = NULL;
	int change_flag = 0;
	uint32_t odd = 0;
	uint32_t even = 0;
	uint32_t temp = 0;

	if (root == NULL) {
                printf("<%s: %d>root is NULL\n", __func__, __LINE__);
                return -1;
        }

	node = cJSON_GetObjectItem(root, "target");
        if (node == NULL) {
                printf("<%s: %d> node is NULL\n", __func__, __LINE__);
		return -1;
        }

	task_array = cJSON_GetObjectItem(root,"luts");
	task_size = cJSON_GetArraySize(task_array);
	tasklist = task_array->child;
	while (tasklist != NULL) {
		node = cJSON_GetObjectItem(tasklist, "name");
		if (node != NULL) {
			name = node->valuestring;
			//printf("name  %s\n", name);
			// skip AWB_MIXED_LIGHT_PARAMETERS
			if (!strcmp(name, "AWB_MIXED_LIGHT_PARAMETERS"))
				goto next_list;
		} else {
			goto next_list;
		}

		node = cJSON_GetObjectItem(tasklist, "width");
		if (node != NULL) {
			width = node->valueint;
			tempdata = NULL;
			if (width == 0) {
				tempdata = node->valuestring;
				width = atoi(tempdata);
			}
			//printf("width is %d\n", width);
		} else {
			width = 0;
			goto next_list;
		}

		node = cJSON_GetObjectItem(tasklist, "rows");
		if (node != NULL) {
			rows = node->valueint;
			tempdata = NULL;
			if (rows == 0) {
				tempdata = node->valuestring;
				rows = atoi(tempdata);
			}
			//printf("rows is %d\n", rows);
		} else {
			rows = 0;
			goto next_list;
		}

		node = cJSON_GetObjectItem(tasklist, "cols");
		if (node != NULL) {
			cols = node->valueint;
			tempdata = NULL;
			if (cols == 0) {
				tempdata = node->valuestring;
				cols = atoi(tempdata);
			}
			//printf("cols is %d\n", cols);
		} else {
			cols = 0;
			goto next_list;
		}

		// modify CALIBRATION_CUSTOM_SETTINGS_CONTEXT register
		// target: static uint32_t _calibration_custom_settings_context[][4]
		if (!strcmp(name, "CALIBRATION_CUSTOM_SETTINGS_CONTEXT")) {
			rows = (rows * cols % 4) ? (rows * cols / 4 + 1) : (rows * cols / 4);
			cols = 4;
			if (rows == 1) {
				rows = 2;
			}
		}

		if (check_lut_inlist(name)) {
			change_flag = 1;
			if (cols > 2)
				fprintf(fp_c, "\n\n // %s (%dx%d %d bytes)\n", name, rows, 16, width);
			else
				fprintf(fp_c, "\n\n // %s (%dx%d %d bytes)\n", name, 16, cols, width);
		} else {
			change_flag = 0;
			fprintf(fp_c, "\n\n // %s (%dx%d %d bytes)\n", name, rows, cols, width);
		}

		//fprintf(fp_c, "\n\n // %s (%dx%d %d bytes)\n", name, rows, cols, width);
		name = mstrlwr(name);
		if (width == 1) {
			if (rows == 1) {
				fprintf(fp_c, "static uint8_t _%s[]\n", name);
			} else {
				fprintf(fp_c, "static uint8_t _%s[][%d]\n", name, cols);
			}
		} else if (width == 2) {
			if (rows == 1) {
				fprintf(fp_c, "static uint16_t _%s[]\n", name);
			} else {
				fprintf(fp_c, "static uint16_t _%s[][%d]\n", name, cols);
			}
		} else if (width == 4) {
			if (rows == 1) {
				fprintf(fp_c, "static uint32_t _%s[]\n", name);
			} else {
				fprintf(fp_c, "static uint32_t _%s[][%d]\n", name, cols);
			}
		}

		sub_array = cJSON_GetObjectItem(tasklist, "value");
		if (sub_array != NULL) {
			array_size = cJSON_GetArraySize(sub_array);
			if (rows * cols !=  array_size) {
				printf("length is err \n");
			}
			count = 0;
			fprintf(fp_c, "= { \n");
			for (array_index = 0; array_index < array_size; array_index++) {
				count++;
				node = cJSON_GetArrayItem(sub_array, array_index);
				if (node == NULL) {
					continue;
				}
				temp =  (uint32_t)node->valuedouble;
				fprintf(fp_c, " %u", temp);
				fprintf(fp_c, ", ");
				if (count % 2 == 0) {
					odd = temp;
				} else {
					even = temp;
				}
			}
			if (change_flag) {
				for (array_index = count; array_index < 32; array_index++) {
					count++;
					if (count % 2 == 0) {
						fprintf(fp_c, " %d", odd);
					} else {
						fprintf(fp_c, " %d", even);
					}
					fprintf(fp_c, ", ");
				}
			}
			fprintf(fp_c, "\n};\n");
		}
next_list:
		tasklist = tasklist->next;
	}

	return 0;
}

int create_calib_lut(cJSON *root, FILE *fp_c)
{
	cJSON *node, *sub_array;
	uint32_t array_size = 0;
	uint32_t array_index = 0;
	cJSON *task_array = NULL;
	uint32_t task_size = 0;
	cJSON *tasklist;
	char *tempdata = NULL;
	uint32_t width;
	uint32_t rows;
	uint32_t cols;
	uint32_t count;
	char *name;

	if (root == NULL) {
                printf("<%s: %d>root is NULL\n", __func__, __LINE__);
                return -1;
        }

	node = cJSON_GetObjectItem(root, "target");
        if (node == NULL) {
                printf("<%s: %d> node is NULL\n", __func__, __LINE__);
		return -1;
        }

	fprintf(fp_c, "\n\n");
	task_array = cJSON_GetObjectItem(root,"luts");
	task_size = cJSON_GetArraySize(task_array);
	tasklist = task_array->child;
	while (tasklist != NULL) {
		node = cJSON_GetObjectItem(tasklist, "name");
		if (node != NULL) {
			name = node->valuestring;
			// skip AWB_MIXED_LIGHT_PARAMETERS
			if (!strcmp(name, "AWB_MIXED_LIGHT_PARAMETERS"))
				goto next_list;
		} else {
			goto next_list;
		}

		node = cJSON_GetObjectItem(tasklist, "width");
		if (node != NULL) {
			width = node->valueint;
			tempdata = NULL;
			if (width == 0) {
				tempdata = node->valuestring;
				width = atoi(tempdata);
			}
			// printf("width is %d\n", width);
		} else {
			width = 0;
			goto next_list;
		}

		node = cJSON_GetObjectItem(tasklist, "rows");
		if (node != NULL) {
			rows = node->valueint;
			tempdata = NULL;
			if (rows == 0) {
				tempdata = node->valuestring;
				rows = atoi(tempdata);
			}
			// printf("rows is %d\n", rows);
		} else {
			rows = 0;
			goto next_list;
		}

		node = cJSON_GetObjectItem(tasklist, "cols");
		if (node != NULL) {
			cols = node->valueint;
			tempdata = NULL;
			if (cols == 0) {
				tempdata = node->valuestring;
				cols = atoi(tempdata);
			}
			// printf("cols is %d\n", cols);
		} else {
			cols = 0;
			goto next_list;
		}

		name = mstrlwr(name);
		if (rows > 1) {
			if (cols > 2) {
				fprintf(fp_c, "static LookupTable %s = {.ptr =_%s, .rows = sizeof(_%s)/sizeof(_%s[0]), .cols = %d, .width = sizeof(_%s[0][0])};\n", name, name, name, name, cols, name);
			} else {
				fprintf(fp_c, "static LookupTable %s = {.ptr =_%s, .rows = sizeof(_%s)/sizeof(_%s[0]), .cols = 2, .width = sizeof(_%s[0][0])};\n", name, name, name, name, name);
			}
		} else {
			fprintf(fp_c, "static LookupTable %s = {.ptr =_%s, .rows = 1, .cols = sizeof(_%s)/sizeof(_%s[0]), .width = sizeof(_%s[0])};\n", name, name, name, name, name);
		}
next_list:
		tasklist = tasklist->next;
	}

	return 0;
}

int create_calib_func(cJSON *root, FILE *fp_c)
{
	cJSON *node, *sub_array;
	uint32_t array_size = 0;
	uint32_t array_index = 0;
	cJSON *task_array = NULL;
	uint32_t task_size = 0;
	cJSON *tasklist;
	uint32_t width;
	uint32_t rows;
	uint32_t cols;
	uint32_t count;
	char *name;
	char *target;

	if (root == NULL) {
                printf("<%s: %d>root is NULL\n", __func__, __LINE__);
                return -1;
        }

	node = cJSON_GetObjectItem(root, "target");
        if (node != NULL) {
		target = node->valuestring;
                //printf("<%s: %d> node is %s\n", __func__, __LINE__, target);
        } else {
		return -1;
	}

	fprintf(fp_c, "\n\n");
	if (strstr(target, "static")) {
		fprintf(fp_c, "uint32_t get_static_calibrations( ACameraCalibrations * c ) {\n");
	} else {
		fprintf(fp_c, "uint32_t get_dynamic_calibrations(ACameraCalibrations * c ) {\n");
	}
	fprintf(fp_c, "        uint32_t result = 0;\n");
	fprintf(fp_c, "        if(c != 0) {\n");

	task_array = cJSON_GetObjectItem(root,"luts");
	task_size = cJSON_GetArraySize(task_array);
	tasklist = task_array->child;
	while (tasklist != NULL) {
		node = cJSON_GetObjectItem(tasklist, "name");
		if (node != NULL) {
			name = node->valuestring;
			// skip AWB_MIXED_LIGHT_PARAMETERS
			if (!strcmp(name, "AWB_MIXED_LIGHT_PARAMETERS"))
				goto next_list;
		} else {
			goto next_list;
		}

		node = cJSON_GetObjectItem(tasklist, "width");
		if (node != NULL) {
			width = node->valueint;
		} else {
			width = 0;
			goto next_list;
		}

		node = cJSON_GetObjectItem(tasklist, "rows");
		if (node != NULL) {
			rows = node->valueint;
		} else {
			rows = 0;
			goto next_list;
		}

		node = cJSON_GetObjectItem(tasklist, "cols");
		if (node != NULL) {
			cols = node->valueint;
		} else {
			cols = 0;
			goto next_list;
		}

		fprintf(fp_c, "               c->calibrations[%s] = ", name);
		name = mstrlwr(name);
		fprintf(fp_c, "&%s;\n", name);
	
next_list:
		tasklist = tasklist->next;
	}
	fprintf(fp_c, "        } else {\n");
	fprintf(fp_c, "               result = -1;\n");
	fprintf(fp_c, "        }\n");
	fprintf(fp_c, "        return result;\n");
	fprintf(fp_c, "}\n");

	return 0;
}

int open_calib_json(const char *file_path, FILE *fp_c, uint32_t mode)
{
        int size = 0, ret = 0, len = 0;
        char *file_buff = NULL;
        cJSON *root = NULL;

        if (file_path == NULL) {
                printf("<%s: %d>file_path is null\n", __func__, __LINE__);
                ret = -1;
                goto err;
        }

        FILE *fp = fopen(file_path, "r");
        if (fp == NULL) {
                printf("<%s: %d>fopen failed\n", __func__, __LINE__);
                ret = -1;
                goto err;
        }

        fseek(fp, 0, SEEK_END);
        size = ftell(fp);
        fseek(fp, 0, SEEK_SET);

        file_buff = malloc(size + 1);
        if (file_buff == NULL) {
                printf("<%s: %d>malloc failed\n", __func__, __LINE__);
                ret = -1;
                goto err;
        }
		memset(file_buff, 0, size + 1);

        len = fread(file_buff, 1, size, fp);
        if (len != size) {
                printf("<%s: %d>fread failed len = %d\n", __func__, __LINE__, len);
                ret = -1;
                goto err;
        }

        root = cJSON_Parse(file_buff);
        if (root == NULL) {
                printf("<%s: %d>root failed\n", __FUNCTION__, __LINE__);
                ret = -1;
                goto err;
        }

	if (mode == 1) { // param
		create_calib_file(root, fp_c);
	} else if (mode == 2) { // lut
		create_calib_lut(root, fp_c);
	} else if (mode == 3) { // calibration
		create_calib_func(root, fp_c);
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

void add_head(FILE *fp_c)
{
	fprintf(fp_c, "#include <unistd.h>\n");
	fprintf(fp_c, "#include <stdlib.h>\n");
	fprintf(fp_c, "#include <string.h>\n");
	fprintf(fp_c, "#include <stdarg.h>\n");
	fprintf(fp_c, "#include <errno.h>\n");
	fprintf(fp_c, "#include \"acamera_command_define.h\" \n\n");
}

void add_end(FILE *fp_c)
{
	fprintf(fp_c, "\n \n");
	fprintf(fp_c, "calib_module_t camera_calibration = {\n");
	fprintf(fp_c, "        .module = \"lib%s\",\n", libname);
	fprintf(fp_c, "        .get_calib_dynamic = get_dynamic_calibrations,\n");
	fprintf(fp_c, "        .get_calib_static = get_static_calibrations\n");
	fprintf(fp_c, "};\n");
}

void print_usage(const char *prog)
{
        printf("Usage: %s \n", prog);
        puts("  -d --dynamic       dynamic json\n"
             "  -s --static        static json\n"
             "  -n --name          name\n");
        exit(1);
}

void parse_opts(int argc, char *argv[])
{
        while (1) {
                static const char short_options[] =
                "d:s:n:";
                static const struct option long_options[] = {
                        {"dynamic_file", 1, 0, 'd'},
                        {"static_file", 1, 0, 's'},
                        {"libname", 1, 0, 'n'},
                        {NULL, 0, 0, 0},
                };

                int cmd_ret;

                cmd_ret =
                    getopt_long(argc, argv, short_options, long_options, NULL);

                if (cmd_ret == -1) {
                        break;
		}

                switch (cmd_ret) {
                case 'd':
                        dynamic_file = optarg;
                        printf("dynamic file = %s\n", dynamic_file);
                        break;
                case 's':
                        static_file = optarg;
                        printf("static file = %s\n", static_file);
                        break;
                case 'n':
                        libname = optarg;
                        printf("libname = %s\n", libname);
                        break;
                default:
                        print_usage(argv[0]);
                        break;
                }
        }
}

int check_file(const char *file_path1, const char *file_path2)
{
	int ret = 0;

	if (file_path1 == NULL || file_path2 == NULL) {
                printf("file is not null\n");
		return -1;
	}

        FILE *fp1 = fopen(file_path1, "r");
        if (fp1 == NULL) {
                printf("%s is not exist\n", file_path1);
                ret = -1;
                goto err;
	}
        fclose(fp1);

        FILE *fp2 = fopen(file_path2, "r");
        if (fp2 == NULL) {
                printf("%s is not exist\n", file_path2);
                ret = -1;
                goto err;
	}
        fclose(fp2);
err:
	return ret;
}


int main(int argc, char *argv[])
{
	FILE *fp;
	int err;
	char file[200];

	parse_opts(argc, argv);
	if (argc < 3) {
		return -1;
	}

	if (check_file(dynamic_file, static_file)) {
		return -1;
	}

	snprintf(file, 200, "%s_calibration.c", libname);
	fp = fopen(file, "w+");
	add_head(fp);

	// mode 1
	open_calib_json(dynamic_file, fp, 1);
	open_calib_json(static_file, fp, 1);
	// mode 2
	open_calib_json(dynamic_file, fp, 2);
	open_calib_json(static_file, fp, 2);
	// mode 3
	open_calib_json(dynamic_file, fp, 3);
	open_calib_json(static_file, fp, 3);

	add_end(fp);

	fclose(fp);
	return 0;
}
