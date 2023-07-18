#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <getopt.h>
#include <libxml2/libxml/parser.h>
#include <libxml2/libxml/tree.h>

#include "cJSON.h"

// #define MAX_REG_NUM 19
#define MAX_FIELD_LEN 64
#define MAX_ARRAY_LEN 400

#define LUT_MAX_LINE 1000

#define REG_JSON_PARSE 0
#define LUT_CALCULATE 1

#define ALIGN_DOWN(a, size)  (a & (~(size-1)) )

#define ALIGN_DOWN4(d)    ALIGN_DOWN(d, 4)

#define DECRYPT_PROC 0
#define ENCRYPT_PROC 1

#define ENCRYPT_KEY 0x35

#define REG_DEFAULT_NUM 10

struct isp_default_value_reg_attr {
	char *reg;
	char *name;
	uint32_t bit_start;
	uint32_t bit_num;
	uint32_t value;
};

// which register need set default value (have addr set in .c but use default value)
struct isp_default_value_reg_attr g_reg_default[REG_DEFAULT_NUM] = {
	{"top", "Bypass temper", 1, 1, 1}, // Bypass temper: 0x18eb8:1, set default value to avoid crash
	{"", "", 0, 0, 0}
};

// which register need be ignored (no addr set in .c)
uint32_t g_reg_ignore[REG_DEFAULT_NUM] = {
	0x18e88, // XJ3-4269:0x18e88 top width/height
	0x18e8c, // XJ3-4269:0x18e8c top pattern/data src
	0x18f98, // XJ3-4269:0x18f98 input formatter mod in/bit width
	0
};

char g_reg_str[MAX_ARRAY_LEN][MAX_FIELD_LEN] = {
	"top",
	"sensor offset wdr l",
	"sensor offset wdr m",
	"sensor offset wdr s",
	"sensor offset wdr vs",
	"gain wdr",
	"decompander0",
	"decompander1",
	"sqrt",
	"raw frontend",
	"defect pixel",
	"sinter",
	"sinter Noise Profile",
	"temper",
	"temper Noise Profile",
	"square be",
	"iridix",
	"demosaic rgb",
	"demosaic rgbir",
	"pf correction",
	"ccm",
	"cnr",
	"fr sharpen"
};

int g_reg_addr_num[MAX_ARRAY_LEN] = {0};

uint32_t g_reg_array[MAX_ARRAY_LEN][4];

char *g_reg_xml_file = NULL;
char *g_reg_xml_encrypt_file = NULL;
uint8_t g_reg_xml_encrypt_proc = 0u;
char *g_setting_file = NULL;
char *g_out_txt_file = NULL;
char *g_need_select_register_file = NULL;
char *g_out_c_file = NULL;
char *g_3d_lut_file = NULL;
int g_run_mode = REG_JSON_PARSE;

void print_usage(const char *prog)
{
	printf("Usage: %s \n", prog);
	puts("  -r --register_xml_file\n"
		 "  -s --setting_json_file\n"
	     "  -o --output_txt_file\n"
		 "  -n --need_select_register_file\n"
		 "  -c --out_c_file\n"
		 "  -l --3d_lut_file\n"
		 "  -m --run_mode\n"
		 "  -e --encrypt_proc\n");
	exit(1);
}

void parse_opts(int argc, char *argv[])
{
	while (1) {
		static const char short_options[] =
		    "r:s:o:n:c:l:m:e:";
		static const struct option long_options[] = {
			{"register_xml_file", 1, 0, 'r'},
			{"setting_json_file", 1, 0, 's'},
			{"output_array_file", 1, 0, 'o'},
			{"need_select_register_file", 1, 0, 'n'},
			{"out_c_file", 1, 0, 'n'},
			{"3d_lut_file", 1, 0, 'l'},
			{"run_mode", 1, 0, 'm'},
			{"encrypt_proc", 1, 0, 'e'},
			{NULL, 0, 0, 0},
		};

		int cmd_ret;

		cmd_ret =
		    getopt_long(argc, argv, short_options, long_options, NULL);

		if (cmd_ret == -1)
			break;

		switch (cmd_ret) {
		case 'r':
			g_reg_xml_file = optarg;
			printf("g_reg_xml_file = %s\n", g_reg_xml_file);
			break;
		case 's':
			g_setting_file = optarg;
			printf("g_setting_file = %s\n", g_setting_file);
			break;
		case 'o':
			g_out_txt_file = optarg;
			printf("g_out_txt_file = %s\n", g_out_txt_file);
			break;
		case 'n':
			g_need_select_register_file = optarg;
			printf("g_need_select_register_file = %s\n", g_need_select_register_file);
			break;
		case 'c':
			g_out_c_file = optarg;
			printf("g_out_c_file = %s\n", g_out_c_file);
			break;
		case 'l':
			g_3d_lut_file = optarg;
			printf("g_3d_lut_file = %s\n", g_3d_lut_file);
			break;
		case 'm':
			g_run_mode = atoi(optarg);
			printf("g_run_mode = %d\n", g_run_mode);
			break;
		case 'e':
			g_reg_xml_encrypt_proc = atoi(optarg);
			printf("g_reg_xml_encrypt_proc = %d\n", g_reg_xml_encrypt_proc);
			break;
		default:
			print_usage(argv[0]);
			break;
		}
	}
}

void set_bit(uint32_t *reg_value, uint32_t bit_start,
	uint32_t bit_num, uint32_t bit_value) {
	for (int i = 0; i < bit_num; i++) {
		if (((*reg_value >> bit_start) & 0x00000001) ^
			((bit_value & 0x1) & 0x00000001)) {
			*reg_value ^= (1 << bit_start);
		}
		bit_value = bit_value >> 1;
		bit_start ++;
	}
}

int axtoi(char *string, int base, int len) {
	char ch = *(string + len);
	int result = 0;
	int base_tmp = 1;
	for (int i = len - 1; i >= 0; i--) {
		ch = *(string + i);
		if (ch >= '0' && ch <= '9') {
			result += (ch - '0') * base_tmp;
		} else if (ch >= 'a' && ch <= 'f') {
			result += (ch - 'a' + 10) * base_tmp;
		} else if (ch >= 'A' && ch <= 'F') {
			result += (ch - 'A' + 10) * base_tmp;
		}
		base_tmp *= base;
	}
	return result;
}

int ctoi(char ch) {
	if (ch >= '0' && ch <= '9') {
		return (ch - '0');
	} else if (ch >= 'a' && ch <= 'f') {
		return (ch - 'a' + 10);
	} else if (ch >= 'A' && ch <= 'F') {
		return (ch - 'A' + 10);
	}
}

void delete_str_punc(char *string) {
	char *l_index = strchr(string, '"');
	char *r_index = strchr(l_index + 1, '"');
	char str[MAX_FIELD_LEN];
	memset(str, 0, MAX_FIELD_LEN);
	memcpy(str, l_index + 1, r_index - l_index - 1);
	memset(string, 0, MAX_FIELD_LEN);
	memcpy(string, str, MAX_FIELD_LEN);
	string[r_index - l_index - 1] = '\0';
}

int encrypt(char *buffer, int64_t size) {
	for (int i = 0; i < size; i++) {
		buffer[i] = ~(buffer[i] ^ ENCRYPT_KEY);
	}
	return 0;
}

int decrypt(char *buffer, int64_t size) {
	for (int i = 0; i < size; i++) {
		buffer[i] = (~buffer[i]) ^ ENCRYPT_KEY;
	}
	return 0;
}

int encrypt_decrypt_file(char *src_file, char *tar_file, uint8_t encrypt_proc) {
	FILE *src_fd = NULL, *tar_fd = NULL;
	char *filebuf = NULL;
    struct stat statbuf;
	int ret;

	src_fd = fopen(src_file, "r");
	if (src_fd == NULL) {
		printf("encrypt/decrypt file open %s fail!!", src_file);
		return -1;
	}
	ret = stat(src_file, &statbuf);
	if (0 == statbuf.st_size) {
		printf("encrypt/decrypt file size is zero !!\n");
		fclose(src_fd);
		return -1;
	}
	filebuf = (char *)malloc(statbuf.st_size);
	if (NULL == filebuf) {
		printf("encrypt/decrypt malloc buff fail !!");
		fclose(src_fd);
		return -1;
	}

	memset(filebuf, 0, statbuf.st_size);

	ret = fread(filebuf, statbuf.st_size, 1, src_fd);

	if (encrypt_proc == ENCRYPT_PROC) {
		ret = encrypt(filebuf, statbuf.st_size);
	} else if (encrypt_proc == DECRYPT_PROC) {
		ret = decrypt(filebuf, statbuf.st_size);
	}
    if (ret) {
		printf("encrypt/decrypt failed!\n");
		free(filebuf);
		fclose(src_fd);
		return -1;
	}
	fclose(src_fd);

	// write to target file
	tar_fd = fopen(tar_file, "w+");
	if (tar_fd == NULL) {
		printf("encrypt/decrypt open output file:%s fail\n", tar_file);
		free(filebuf);
		return -1;
	} else {
		fflush(stdout);
	}
	fwrite(filebuf, 1, statbuf.st_size, tar_fd);
	fflush(tar_fd);
	fclose(tar_fd);

	free(filebuf);

	return 0;
}

int reg_ignore_check(uint32_t reg_value)
{
	for (int i = 0; i < REG_DEFAULT_NUM; i++) {
		if (g_reg_ignore[i] <= 0) {
			break;
		}
		if (reg_value == g_reg_ignore[i]) {
			return 1;
		}
	}

	return 0;
}

int reg_use_setting_check(char *reg, char *name)
{
	for (int i = 0; i < REG_DEFAULT_NUM; i++) {
		if (strlen(g_reg_default[i].reg) <= 0) {
			break;
		}
		if ((!strcmp(reg, g_reg_default[i].reg)) && (!strcmp(name, g_reg_default[i].name))) {
			return 0;
		}
	}

	return 1;
}

void reg_set_default(uint32_t *reg_value, char *reg, char *name)
{
	for (int i = 0; i < REG_DEFAULT_NUM; i++) {
		if (strlen(g_reg_default[i].reg) <= 0) {
			break;
		}
		if ((!strcmp(reg, g_reg_default[i].reg)) && (!strcmp(name, g_reg_default[i].name))) {
			set_bit(reg_value, g_reg_default[i].bit_start, g_reg_default[i].bit_num, g_reg_default[i].value);
			return;
		}
	}
}

void read_need_select_reg(char *need_select_reg_file) {
	FILE *isp_fp = NULL;
	int ret;

    isp_fp = fopen(need_select_reg_file, "r");
	if (isp_fp == NULL) {
		printf("isp need select file open %s fail!!", need_select_reg_file);
		return;
	}

	memset(g_reg_str, 0, MAX_FIELD_LEN * MAX_ARRAY_LEN);

	for (int i = 0; i < MAX_ARRAY_LEN; i++) {
		if (fgets(g_reg_str[i], MAX_FIELD_LEN, isp_fp)) {
			if (strstr(g_reg_str[i], "\"") ||
				strstr(g_reg_str[i], ",")) {
				delete_str_punc(g_reg_str[i]);
			}
		} else {
			break;
		}
	}

	fclose(isp_fp);
}

void xml_addr_trans(char *addr, uint32_t *result, uint32_t *bit_start, uint32_t *bit_num) {
	uint32_t base_tmp = 16;

	char addr1 = addr[1];
	char addr2 = addr[2];
	char addr4 = addr[4];
	char addr5 = addr[5];
	char addr7 = addr[7];
	char addr8 = addr[8];
	if (!strchr(addr, ':')) {
		*bit_start = ctoi(addr[10]);
		*bit_num = 1;
	} else {
		*bit_start = ctoi(addr[12]);
		*bit_num = ctoi(addr[10]) - ctoi(addr[12]) + 1;
	}

	*result = (ctoi(addr1) * base_tmp) *
			(ctoi(addr2) * base_tmp * base_tmp + ctoi(addr4) * base_tmp + ctoi(addr5)) +
			(ctoi(addr7) * base_tmp + ctoi(addr8));

	*bit_start += (*result - ALIGN_DOWN4(*result)) * 8;
}

void xml_name_trans(char *name, char *result) {

	char *l_index = strchr(name, '(');
	char *y_index = strchr(name, ')');
	char *m_index = strchr(name, ':');
	memcpy(result, name, (l_index - name - 1));
	result[l_index - name - 1] = '\0';
}

//json file parse
cJSON *isp_json_parse(char *json_file) {
    FILE *isp_fp = NULL;
    char *filebuf = NULL;
    struct stat statbuf;
	cJSON *root;
	int ret;

    isp_fp = fopen(json_file, "r");
	if (isp_fp == NULL) {
		printf("isp json file open %s fail!!", json_file);
		return NULL;
	}
	ret = stat(json_file, &statbuf);
	if (0 == statbuf.st_size) {
		printf("json_file size is zero !!\n");
		fclose(isp_fp);
		return NULL;
	}
	filebuf = (char *)malloc(statbuf.st_size + 1);
	if (NULL == filebuf) {
		printf("isp malloc buff fail !!");
		fclose(isp_fp);
		return NULL;
	}

	memset(filebuf, 0, statbuf.st_size + 1);

	ret = fread(filebuf, statbuf.st_size, 1, isp_fp);

    root = cJSON_Parse(filebuf);
    if (NULL == root) {
		printf("Parse reg_rootinfo json failed!\n");
		return NULL;
	}
	free(filebuf);
	fclose(isp_fp);

	return root;
}

void isp_json_delete(cJSON *root) {
	cJSON_Delete(root);
}

int32_t isp_get_value_from_setting(cJSON *setting_root, char *setting_str) {
	for (cJSON *setting_item = setting_root->child;
			setting_item != NULL; setting_item = setting_item->next) {
		if (strstr(cJSON_GetObjectItem(setting_item, "name")->valuestring, setting_str) &&
			!strstr(cJSON_GetObjectItem(setting_item, "name")->valuestring, "pong")) {
			return cJSON_GetObjectItem(setting_item, "value")->valueint;
		}
	}
	return -1;
}

//array output
void isp_array_output(char *output_file)
{
	FILE *file = NULL;
	char array[100];
	int reg_addr_num = 0, array_index = 0;
	if (output_file) {
		file = fopen(output_file, "w+");
		if (file == NULL) {
			printf("open output file:%s fail\n", output_file);
		} else {
			fflush(stdout);
		}
	}
	for (int i = 1; i < MAX_ARRAY_LEN; i++) {
		if ((i == 1) && file) {
			sprintf(array, "// %s\n", g_reg_str[array_index]);
			fwrite(array, 1, strlen(array), file);
		}
		if (reg_ignore_check(g_reg_array[i][0])) {
			continue;
		}
		if ((reg_addr_num == g_reg_addr_num[array_index]) && file && (array_index < MAX_ARRAY_LEN)) {
			if (strlen(g_reg_str[array_index + 1])) {
				sprintf(array, "// %s\n", g_reg_str[array_index + 1]);
				fwrite(array, 1, strlen(array), file);
				array_index ++;
				reg_addr_num = 0;
				if (g_reg_addr_num[array_index] == 0) {
					i--;
					continue;
				}
			} else {
				break;
			}
		}
		sprintf(array, "{0x%x, 0x%x, 0x%x, %d},\n",
			g_reg_array[i][0],
			g_reg_array[i][1],
			g_reg_array[i][2],
			g_reg_array[i][3]);
		if (file)
			fwrite(array, 1, strlen(array), file);
		reg_addr_num ++;
	}
	if (file) {
		fflush(file);
		fclose(file);
	}
}

//array output --> c file
void isp_array_output_c(char *output_file)
{
	FILE *file = NULL;
	char array[100];
	int reg_addr_num = 0, array_index = 0;
	char *filebuf = NULL;
    struct stat statbuf;
	char *array_offset_addr = NULL;
	int array_offset = 0;
	int ret;

	char *array_name = "_calibration_custom_settings_context";
	if (output_file) {
		file = fopen(output_file, "r+");
		if (file == NULL) {
			printf("open output file:%s fail\n", output_file);
			return;
		} else {
			fflush(stdout);
		}
	}

	// read .c file
	stat(output_file, &statbuf);
	if (0 == statbuf.st_size) {
		printf("c_file size is zero !!\n");
		fclose(file);
		return;
	}
	filebuf = (char *)malloc(statbuf.st_size);
	if (NULL == filebuf) {
		printf("isp malloc buff fail !!");
		fclose(file);
		return;
	}

	memset(filebuf, 0, statbuf.st_size);
	ret = fread(filebuf, statbuf.st_size, 1, file);

	//search array position
	array_offset_addr = strstr(filebuf, "_calibration_custom_settings_context[]");
	if (array_offset_addr == NULL) {
		printf("can`t find array, retun\n");
		return;
	}
	for (;;) {
		if (*array_offset_addr == '{') {
			array_offset_addr++;
			break;
		}
		array_offset_addr++;
	}
	array_offset = array_offset_addr - filebuf;
	fseek(file, array_offset, SEEK_SET);

	// write array to c file
	for (int i = 1; i < MAX_ARRAY_LEN; i++) {
		if ((i == 1) && file) {
			sprintf(array, "// %s\n", g_reg_str[array_index]);
			fwrite(array, 1, strlen(array), file);
		}
		if (reg_ignore_check(g_reg_array[i][0])) {
			continue;
		}
		if ((reg_addr_num == g_reg_addr_num[array_index]) && file && (array_index < MAX_ARRAY_LEN)) {
			if (strlen(g_reg_str[array_index + 1])) {
				sprintf(array, "// %s\n", g_reg_str[array_index + 1]);
				fwrite(array, 1, strlen(array), file);
				array_index ++;
				reg_addr_num = 0;
				if (g_reg_addr_num[array_index] == 0) {
					i--;
					continue;
				}
			} else {
				break;
			}
		}
		sprintf(array, "{0x%x, 0x%x, 0x%x, %d},\n",
			g_reg_array[i][0],
			g_reg_array[i][1],
			g_reg_array[i][2],
			g_reg_array[i][3]);
		// printf("%s", array);
		if (file)
			fwrite(array, 1, strlen(array), file);
		reg_addr_num ++;
	}

	sprintf(array, "{0, 0, 0, 0},\n");
	fwrite(array, 1, strlen(array), file);
	for (;;) {
		if (*array_offset_addr == '}') {
			break;
		}
		array_offset_addr++;
	}
	array_offset = array_offset_addr - filebuf;
	fwrite(array_offset_addr, 1, statbuf.st_size - array_offset, file);

	if (filebuf) {
		free(filebuf);
	}
	if (file) {
		fflush(file);
		fclose(file);
		printf("array write to file:%s success\n", output_file);
	}
}

xmlDocPtr isp_xml_parse(char *reg_xml_file) {
	xmlDocPtr doc = NULL;
	// xmlKeepBlanksDefault(0);

    doc = xmlParseFile(reg_xml_file);
    if (NULL == doc) {
		printf("xml:%s parse failed!\n", reg_xml_file);
		return NULL;
	}

	return doc;
}

void isp_xml_delete(xmlDocPtr doc) {
	xmlFreeDoc(doc);
}

void xml_get_addr(xmlDocPtr reg_doc, cJSON *setting_root) {
	xmlNodePtr root_node = NULL;

	root_node = xmlDocGetRootElement(reg_doc);
	 if(root_node == NULL) {
		printf("ERROR: empty file\n");
		return;
	}
	xmlNodePtr list_node = root_node->xmlChildrenNode->next->next->next;
	for (xmlNodePtr node = list_node->xmlChildrenNode; node != NULL; node = node->next) {
		if ((!xmlStrcmp(node->name, (const xmlChar *)"device"))) {
			for (xmlNodePtr chl_node = node->xmlChildrenNode; chl_node != NULL;
				chl_node = chl_node->next) {
				if ((xmlStrcmp(chl_node->name, (const xmlChar *)"text"))) {
					xmlChar *szName = xmlGetProp(chl_node, BAD_CAST "name");
					xmlChar *szAddr = xmlGetProp(chl_node, BAD_CAST "addr");
					xmlFree(szName);
					xmlFree(szAddr);
				}
			}
		}
	}
}

void isp_generate_array_xml(xmlDocPtr reg_doc, cJSON *setting_root) {
	int array_index = 0;
	xmlNodePtr root_node = NULL;
	char cur_str[MAX_FIELD_LEN] = {0};
	char cur_reg_str[MAX_FIELD_LEN] = {0};
	int cur_bit_start = 0, cur_bit_num = 0;
	int reg_addr_num = 0;

	root_node = xmlDocGetRootElement(reg_doc);
	if(root_node == NULL) {
		printf("ERROR: empty file\n");
		return;
	}
	xmlNodePtr list_node = root_node->xmlChildrenNode->next->next->next;

	for (int i = 0; i < MAX_ARRAY_LEN; i++) {
		if (!strlen(g_reg_str[i])) {
			if (cur_bit_num != 0) {
				char setting_str[MAX_FIELD_LEN];
				int setting_value = 0;
				sprintf(setting_str, "%s/%s", cur_reg_str, cur_str);
				setting_value = isp_get_value_from_setting(setting_root, setting_str);
				set_bit(&g_reg_array[array_index][1], cur_bit_start, cur_bit_num, setting_value);
				memset(cur_str, 0, MAX_FIELD_LEN);
				memset(cur_reg_str, 0, MAX_FIELD_LEN);
				cur_bit_num = 0;
				cur_bit_start = 0;
			}
			continue;
		}
		for (xmlNodePtr node = list_node->xmlChildrenNode; node != NULL; node = node->next) {
			xmlChar *szRegName = xmlGetProp(node, BAD_CAST "name");
			if ((!xmlStrcmp(node->name, (const xmlChar *)"device")) &&
				(xmlStrstr(szRegName, (const xmlChar *)g_reg_str[i])) &&
				(!xmlStrcmp((xmlStrstr(szRegName, (const xmlChar *)g_reg_str[i])), (const xmlChar *)g_reg_str[i]))) {
				for (xmlNodePtr chl_node = node->xmlChildrenNode; chl_node != NULL;
					chl_node = chl_node->next) {
					if ((!xmlStrcmp(chl_node->name, (const xmlChar *)"bit")) ||
						(!xmlStrcmp(chl_node->name, (const xmlChar *)"byte"))) {
						xmlChar *szName = xmlGetProp(chl_node, BAD_CAST "name");
						xmlChar *szAddr = xmlGetProp(chl_node, BAD_CAST "addr");
						xmlChar *szDefault = xmlGetProp(chl_node, BAD_CAST "default");
						int default_val;
						if (xmlStrstr(szDefault, "x")) {
							default_val = axtoi(szDefault + 2, 16, (strlen(szDefault) - 2));
						} else {
							default_val = axtoi(szDefault, 16, strlen(szDefault));
						}
						char name[MAX_FIELD_LEN] = {0};
						int addr = 0, bit_start = 0, bit_num = 0;

						xml_addr_trans(szAddr, &addr, &bit_start, &bit_num);

						if (xmlStrstr(szName, "(")) {
							xml_name_trans(szName, name);
						} else {
							strcpy(name, szName);
						}
						if (strcmp(cur_str, name)) {
							if (cur_bit_num != 0) {
								char setting_str[MAX_FIELD_LEN];
								int setting_value = 0;
								if (reg_use_setting_check(cur_reg_str, cur_str)) {
									sprintf(setting_str, "%s/%s", cur_reg_str, cur_str);
									setting_value = isp_get_value_from_setting(setting_root, setting_str);
									if (setting_value >= 0) {
										set_bit(&g_reg_array[array_index][1], cur_bit_start, cur_bit_num, setting_value);
									}
								} else {
									reg_set_default(&g_reg_array[array_index][1], cur_reg_str, cur_str);
								}
								memset(cur_str, 0, MAX_FIELD_LEN);
								memset(cur_reg_str, 0, MAX_FIELD_LEN);
								cur_bit_num = 0;
								cur_bit_start = 0;
							}
							memcpy(cur_str, name, MAX_FIELD_LEN);
							memcpy(cur_reg_str, g_reg_str[i], MAX_FIELD_LEN);
							cur_bit_start = bit_start;
							cur_bit_num = bit_num;
						} else {
							bit_start = (bit_start < cur_bit_start) ? bit_start : cur_bit_start;
							bit_num = cur_bit_num + bit_num;
							cur_bit_start = bit_start;
							cur_bit_num = bit_num;
						}

						if (ALIGN_DOWN4(addr) != g_reg_array[array_index][0]) {
							array_index++;
							reg_addr_num++;
							g_reg_array[array_index][0] = ALIGN_DOWN4(addr);
							g_reg_array[array_index][1] = 0;
							g_reg_array[array_index][2] = 0xffffffff;
							g_reg_array[array_index][3] = 4;
						}
						set_bit(&g_reg_array[array_index][1], bit_start, bit_num, default_val);

						xmlFree(szName);
						xmlFree(szAddr);
						xmlFree(szDefault);
					}
				}
			}
			xmlFree(szRegName);
		}
		g_reg_addr_num[i] = reg_addr_num;
		reg_addr_num = 0;
	}
}

void gen_arrayttt(int *arrayttt) {
	int mflag = 0, location = 0, invalid = 0, linvalid = 0, mcount = 0;

	for (int i = 0; i < LUT_MAX_LINE; i++)
		arrayttt[i] = 1000;

	for (int tr = 0; tr < 10; tr += 2) {
		for (int tg = 0; tg < 10; tg += 2) {
			for (int tb = 0; tb < 10; tb += 2) {
				mflag = 0;
				arrayttt[location] = ((tr + 0) * 9 * 9 + (tg + 0) * 9 + (tb + 0));
				location++;
				invalid++;
				if (tb == 8) {
					mflag = 1;
					arrayttt[location] = 1000;
					linvalid++;
				} else {
					mflag = 0;
					invalid++;
					arrayttt[location] = ((tr + 0) * 9 * 9 + (tg + 0) * 9 + (tb + 1));
				}
				location++;
				if (tg == 8) {
					mflag = 1;
					arrayttt[location] = 1000;
					linvalid++;
				} else {
					mflag = 0;
					invalid++;
					arrayttt[location] = ((tr + 0) * 9 * 9 + (tg + 1) * 9 + (tb + 0));
				}
				location++;
				if ((tb == 8) || (tg == 8)) {
					mflag = 1;
					arrayttt[location] = 1000;
					linvalid++;
				} else {
					mflag = 0;
					invalid++;
					arrayttt[location] = ((tr + 0) * 9 * 9 + (tg + 1) * 9 + (tb + 1));
				}
				location++;
				if (tr == 8) {
					mflag = 1;
					arrayttt[location] = 1000;
					linvalid++;
				} else {
					mflag = 0;
					invalid++;
					arrayttt[location] = ((tr + 1) * 9 * 9 + (tg + 0) * 9 + (tb + 0));
				}
				location++;
				if ((tr == 8) || (tb == 8)) {
					mflag = 1;
					arrayttt[location] = 1000;
					linvalid++;
				} else {
					mflag = 0;
					invalid++;
					arrayttt[location] = ((tr + 1) * 9 * 9 + (tg + 0) * 9 + (tb + 1));
				}
				location++;
				if ((tr == 8) || (tg == 8)) {
					mflag = 1;
					arrayttt[location] = 1000;
					linvalid++;
				} else {
					mflag = 0;
					invalid++;
					arrayttt[location] = ((tr + 1) * 9 * 9 + (tg + 1) * 9 + (tb + 0));
				}
				location++;
				if ((tr == 8) || (tg == 8) || (tb == 8)) {
					mflag = 1;
					arrayttt[location] = 1000;
					linvalid++;
				} else {
					mflag = 0;
					invalid++;
					arrayttt[location] = ((tr + 1) * 9 * 9 + (tg + 1) * 9 + (tb + 1));
				}
				location++;
				// printf("tr:%d tg:%d tb:%d location%d\n", tr, tg, tb, location);
				if (mflag == 0) {
					mcount++;
				}
			}
		}
	}
}

void gen_arrayhw(int *arrayttt, int (*lut_create)[3], int *arrayhw) {
	int data = 0, location = 0, hcount = 0;
	for (int i = 0; i < LUT_MAX_LINE; i++) {
		// printf("i:%d ---%d %d %d\n", arrayttt[i],
		// 	(uint32_t)(((double)lut_create[arrayttt[i]][0] / 16) + 0.5),
		// 	((uint32_t)(((double)lut_create[arrayttt[i]][1] / 16) + 0.5)) << 10,
		// 	((uint32_t)(((double)lut_create[arrayttt[i]][2] / 16) + 0.5)) << 20);
		if (arrayttt[i] < 1000) {
			data = (uint32_t)(((double)lut_create[arrayttt[i]][0] / 16) + 0.5) +
					((uint32_t)(((double)lut_create[arrayttt[i]][1] / 16) + 0.5) << 10) +
					((uint32_t)(((double)lut_create[arrayttt[i]][2] / 16) + 0.5) << 20);
			arrayhw[location] = data;
		} else {
			arrayhw[location] = 0;
		}
		// printf("i:%d out:%u\n", i, arrayhw[location]);
		location++;
		hcount++;
	}
}

int read_lut_file(char *lut_file, int (*lut_create)[3]) {
	FILE *file;
	char line[100];
	char *number;
	char *sepa_str = " ,	";
	char *str;

	file = fopen(lut_file, "r");
	if (file == NULL) {
		printf("open lut file:%s fail\n", lut_file);
		return -1;
	}

	for (int i = 0; i < LUT_MAX_LINE + 1; i++) {
		if (feof(file)) {
			printf("file end--\n");
			break;
		}

		str = fgets(line, 100, file);

		number = strtok(line, sepa_str);
		for (int n = 0; n < 3; n++) {
			lut_create[i][n] = 0;
			if (number != NULL) {
				lut_create[i][n] = atoi(number);
				number = strtok(NULL, sepa_str);
			} else {
				break;
			}
		}
	}
	fclose(file);
}

// 3dlut --> .c todo  array name: _calibration_lut3d_mem
// 3dlut --> txt
int out_lut_data(int *arrayhw, char *output_file) {
	FILE *file = NULL;

	file = fopen(output_file, "w+");
	if (file == NULL) {
		printf("open output file:%s fail\n", output_file);
		return -1;
	} else {
		fflush(stdout);
	}

	for (int i = 0; i < LUT_MAX_LINE; i++) {
		if (i != 0) {
			fprintf(file, ", ");
		}
		fprintf(file, "%d", arrayhw[i]);
	}
	if (file) {
		fflush(file);
		fclose(file);
	}
}

int main(int argc, char *argv[]) {
    char default_reg_json_file[100] = "isp_reg.json";
	char default_reg_xml_file[100] = "IV009-SW-Control.xml";
	char default_setting_file[100] = "settings.json";
	char default_out_array_file[100] = "output_array.txt";
	char default_reg_xml_encrypt_file[100] = "encrypt.xml";
	char default_reg_xml_decrypt_file[100] = "decrypt.xml";
	char *reg_xml_file, *setting_file, *out_array_file, *reg_xml_encrypt_file;
    int ret;
    cJSON *reg_root, *setting_root;
	xmlDocPtr reg_doc;
	int lut_create[LUT_MAX_LINE + 1][3];
	int arrayttt[LUT_MAX_LINE];
	int arrayhw[LUT_MAX_LINE];
	// char *out_3d_lut_file = "test3dlut.txt";

	parse_opts(argc, argv);

	switch (g_run_mode)
	{
	case REG_JSON_PARSE:
		reg_xml_file = g_reg_xml_file ? g_reg_xml_file : default_reg_xml_file;
		if (g_reg_xml_encrypt_proc & (1 << ENCRYPT_PROC)) {
			ret = encrypt_decrypt_file(reg_xml_file,
				default_reg_xml_encrypt_file, ENCRYPT_PROC);
			if (ret) {
				printf("encrypt xml file:%s failed\n", reg_xml_file);
				return ret;
			}
			reg_xml_encrypt_file = default_reg_xml_encrypt_file;
		} else {
			reg_xml_encrypt_file = reg_xml_file;
		}
		if (g_reg_xml_encrypt_proc & (1 << DECRYPT_PROC)) {
			ret = encrypt_decrypt_file(reg_xml_encrypt_file,
				default_reg_xml_decrypt_file, DECRYPT_PROC);
			if (ret) {
				printf("decrypt xml file:%s failed\n", reg_xml_file);
				return ret;
			}
			reg_xml_file = default_reg_xml_decrypt_file;
		}

		setting_file = g_setting_file ? g_setting_file : default_setting_file;
		out_array_file = g_out_txt_file ? g_out_txt_file : default_out_array_file;
		if (g_need_select_register_file) {
			read_need_select_reg(g_need_select_register_file);
		}

		reg_doc = isp_xml_parse(reg_xml_file);
		if (reg_doc == NULL) {
			printf("file:%s parse fail, exit\n", reg_xml_file);
			return 1;
		}

		setting_root = isp_json_parse(setting_file);
		if (setting_root == NULL) {
			printf("file:%s parse fail, exit\n", setting_file);
			return 1;
		}

		isp_generate_array_xml(reg_doc, setting_root);

		if (g_out_c_file) {
			isp_array_output_c(g_out_c_file);
		} else {
			isp_array_output(out_array_file);
		}

		isp_xml_delete(reg_doc);
		isp_json_delete(setting_root);
		if (g_reg_xml_encrypt_proc & (1 << DECRYPT_PROC)) {
			remove(default_reg_xml_decrypt_file);
		}
		break;

	case LUT_CALCULATE:
		if (g_3d_lut_file) {
			ret = read_lut_file(g_3d_lut_file, lut_create);
			if (ret < 0) {
				printf("read 3d lut file:%s fail\n", g_3d_lut_file);
				return 1;
			}
			gen_arrayttt(arrayttt);
			gen_arrayhw(arrayttt, lut_create, arrayhw);
			if (g_out_txt_file) {
				out_lut_data(arrayhw, g_out_txt_file);
			} else {
				printf("please input out_txt file\n");
			}
		} else {
			printf("please input right lut file\n");
			return 1;
		}
		break;

	default:
		break;
	}

    return 0;
}
