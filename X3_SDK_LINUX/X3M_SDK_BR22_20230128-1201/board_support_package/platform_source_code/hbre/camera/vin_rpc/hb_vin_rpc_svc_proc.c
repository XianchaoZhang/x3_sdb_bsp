// hb_vin_rpc svc procs.
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include "hb_vin_rpc.h"
#include "inc/hb_vin.h"

int result;
#define LOG(fmt, ...) \
			fprintf(stdout, "[VINRPC][SVC] " fmt, ##__VA_ARGS__)
#define DBG(fmt, ...) \
			LOG(fmt, ##__VA_ARGS__)

int * hb_vin_init_1_svc(int *entry_num, struct svc_req *req)
{
	LOG("hb_vin_init %d\n", *entry_num);
	result = hb_vin_init(*entry_num);
	DBG("hb_vin_init ret=%d -- %s\n", result, (result) ?  "error" : "done");
	return &result;
}

int * hb_vin_start_1_svc(int *entry_num, struct svc_req *req)
{
	LOG("hb_vin_start %d\n", *entry_num);
	result = hb_vin_start(*entry_num);
	DBG("hb_vin_start ret=%d -- %s\n", result, (result) ?  "error" : "done");
	return &result;
}

int * hb_vin_stop_1_svc(int *entry_num, struct svc_req *req)
{
	LOG("hb_vin_stop %d\n", *entry_num);
	result = hb_vin_stop(*entry_num);
	DBG("hb_vin_stop ret=%d -- %s\n", result, (result) ?  "error" : "done");
	return &result;
}

int * hb_vin_deinit_1_svc(int *entry_num, struct svc_req *req)
{
	LOG("hb_vin_deinit %d\n", *entry_num);
	result = hb_vin_deinit(*entry_num);
	DBG("hb_vin_deinit ret=%d -- %s\n", result, (result) ?  "error" : "done");
	return &result;
}

int * hb_vin_reset_1_svc(int *entry_num, struct svc_req *req)
{
	LOG("hb_vin_reset %d\n", *entry_num);
	result = hb_vin_reset(*entry_num);
	DBG("hb_vin_reset ret=%d -- %s\n", result, (result) ?  "error" : "done");
	return &result;
}

int * hb_vin_set_bypass_1_svc(hb_vin_bypass_arg *bypass, struct svc_req *req)
{
	LOG("hb_vin_setbypass %d %d\n", bypass->port, bypass->enable);
	result = hb_vin_set_bypass(bypass->port, bypass->enable);
	DBG("hb_vin_setbypass ret=%d -- %s\n", result, (result) ?  "error" : "done");
	return &result;
}

int * hb_vin_snrclk_set_en_1_svc(hb_vin_snrclk_arg *snrclk, struct svc_req *req)
{
	LOG("hb_vin_snrclk_set_en %d %d\n", snrclk->entry, snrclk->value);
	result = hb_vin_snrclk_set_en(snrclk->entry, snrclk->value);
	DBG("hb_vin_snrclk_set_en ret=%d -- %s\n", result, (result) ?  "error" : "done");
	return &result;
}

int * hb_vin_snrclk_set_freq_1_svc(hb_vin_snrclk_arg *snrclk, struct svc_req *req)
{
	LOG("hb_vin_snrclk_set_freq %d %d\n", snrclk->entry, snrclk->value);
	result = hb_vin_snrclk_set_freq(snrclk->entry, snrclk->value);
	DBG("hb_vin_snrclk_set_freq ret=%d -- %s\n", result, (result) ?  "error" : "done");
	return &result;
}

int * hb_vin_pre_request_1_svc(hb_vin_pre_arg *pre, struct svc_req *req)
{
	uint32_t cnt;
	LOG("hb_vin_pre_request %d %d %d\n", pre->entry, pre->type, pre->value);
	result = hb_vin_pre_request(pre->entry, pre->type, pre->value);
	DBG("hb_vin_pre_request ret=%d -- %s\n", result, (result) ?  "error" : "done");
	return &result;
}

int * hb_vin_pre_result_1_svc(hb_vin_pre_arg *pre, struct svc_req *req)
{
	uint32_t cnt;
	LOG("hb_vin_pre_result %d %d %d\n", pre->entry, pre->type, pre->value);
	result = hb_vin_pre_result(pre->entry, pre->type, pre->value);
	DBG("hb_vin_pre_result ret=%d -- %s\n", result, (result) ?  "error" : "done");
	return &result;
}

FILE *fopen_new(char * file_buff)
{
	char dir_buff[256] = {0};
	char *s;

	s = strchr(file_buff, '/');
	while (s) {
		strncpy(dir_buff, file_buff, s - file_buff);
		if (0 != access(dir_buff, F_OK))
			mkdir(dir_buff, 0755);
		s++;
		s = strchr(s, '/');
	}
	return fopen(file_buff, "w+");
}

int * hb_cam_mipi_parse_cfg_1_svc(hb_vin_cfg_arg *cfg, struct svc_req *req)
{
	char file_buff[256] = {0};
	int len;
	FILE *fp = NULL;

	LOG("hb_cam_mipi_parse_cfg %s %d %d %d\n", cfg->filename,
			cfg->fps, cfg->resolution, cfg->entry_num);
	snprintf(file_buff, sizeof(file_buff), cfg->filename,
			cfg->fps, cfg->resolution);
	fp = fopen_new(file_buff);
	if (fp) {
		len = strlen(cfg->filecont);
		result = fwrite(cfg->filecont, 1, len, fp);
		fclose(fp);
		if (result < len) {
			result = (-1);
		} else {
			result = hb_cam_mipi_parse_cfg(cfg->filename,
				cfg->fps, cfg->resolution, cfg->entry_num);
		}
	} else {
		result = (-1);
	}
	DBG("hb_cam_mipi_parse_cfg ret=%d -- %s\n", result, (result) ?  "error" : "done");
	return &result;
}
