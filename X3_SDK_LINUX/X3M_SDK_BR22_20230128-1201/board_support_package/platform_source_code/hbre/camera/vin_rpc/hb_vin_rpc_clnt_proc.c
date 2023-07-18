// hb_vin_rpc svc procs.
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <sys/stat.h>
#include "hb_vin_rpc.h"

int result;
CLIENT *clnt;
#define LOG(fmt, ...) \
			fprintf(stdout, "[VINRPC][CLNT] " fmt, ##__VA_ARGS__)
#define DBG(fmt, ...) \
			LOG(fmt, ##__VA_ARGS__)

#ifdef VINRPC_HOST
#define VINRPC_IP VINRPC_HOST
#else
#define VINRPC_IP "127.0.0.1"
#endif

CLIENT * hb_rpc_clnt(void)
{
	const char *host;

	if (clnt)
		return clnt;

	host = getenv("VINRPC_HOST");
	host = (host) ? host : VINRPC_IP;
	clnt = clnt_create(host, VINRPC, VINRPC_V1, "tcp");
	if (clnt == NULL) {
		clnt_pcreateerror(host);
	}
	return clnt;
}

void hb_rpc_clnt_destory(void)
{
	if (clnt)
		clnt_destroy(clnt);
	clnt = NULL;
}

int hb_vin_init(uint32_t entry_num)
{
	int en, *ret;
	CLIENT *cl;

	LOG("hb_vin_init %d\n", entry_num);
	cl = hb_rpc_clnt();
	if (cl == NULL)
		return (-1);

	en = (int)entry_num;
	ret = hb_vin_init_1(&en, cl);
	DBG("hb_vin_init ret=%d -- %s\n", *ret, (*ret) ?  "error" : "done");
	return *ret;
}

int hb_vin_start(uint32_t entry_num)
{
	int en, *ret;
	CLIENT *cl;

	LOG("hb_vin_start %d\n", entry_num);
	cl = hb_rpc_clnt();
	if (cl == NULL)
		return (-1);

	en = (int)entry_num;
	ret = hb_vin_start_1(&en, cl);
	DBG("hb_vin_start ret=%d -- %s\n", *ret, (*ret) ?  "error" : "done");
	return *ret;
}

int hb_vin_stop(uint32_t entry_num)
{
	int en, *ret;
	CLIENT *cl;

	LOG("hb_vin_stop %d\n", entry_num);
	cl = hb_rpc_clnt();
	if (cl == NULL)
		return (-1);

	en = (int)entry_num;
	ret = hb_vin_stop_1(&en, cl);
	DBG("hb_vin_stop ret=%d -- %s\n", *ret, (*ret) ?  "error" : "done");
	return *ret;
}

int hb_vin_deinit(uint32_t entry_num)
{
	int en, *ret;
	CLIENT *cl;

	LOG("hb_vin_deinit %d\n", entry_num);
	cl = hb_rpc_clnt();
	if (cl == NULL)
		return (-1);

	en = (int)entry_num;
	ret = hb_vin_deinit_1(&en, cl);
	hb_rpc_clnt_destory();
	DBG("hb_vin_deinit ret=%d -- %s\n", *ret, (*ret) ?  "error" : "done");
	return *ret;
}

int hb_vin_reset(uint32_t entry_num)
{
	int en, *ret;
	CLIENT *cl;

	LOG("hb_vin_reset %d\n", entry_num);
	cl = hb_rpc_clnt();
	if (cl == NULL)
		return (-1);

	en = (int)entry_num;
	ret = hb_vin_reset_1(&en, cl);
	DBG("hb_vin_reset ret=%d -- %s\n", *ret, (*ret) ?  "error" : "done");
	return *ret;
}

int hb_vin_set_bypass(uint32_t port, uint32_t enable)
{
	int *ret;
	CLIENT *cl;
	hb_vin_bypass_arg bypass;

	LOG("hb_vin_set_bypass %d %d\n", port, enable);
	cl = hb_rpc_clnt();
	if (cl == NULL)
		return (-1);

	bypass.port = port;
	bypass.enable = enable;
	ret = hb_vin_set_bypass_1(&bypass, cl);
	DBG("hb_vin_set_bypass ret=%d -- %s\n", *ret, (*ret) ?  "error" : "done");
	return *ret;
}

int hb_cam_mipi_parse_cfg(char *filename, int fps, int resolution, int entry_num)
{
	int *ret;
	CLIENT *cl;
	FILE *fp;
	char file_buff[256] = {0}, *file_cont;
	struct stat statbuf;
	hb_vin_cfg_arg cfg;

	LOG("hb_cam_mipi_parse_cfg %s %d %d %d\n", filename,
		fps, resolution, entry_num);
	cl = hb_rpc_clnt();
	if (cl == NULL)
		return (-1);

	snprintf(file_buff, sizeof(file_buff), filename, fps, resolution);
	stat(file_buff, &statbuf);
	if(statbuf.st_size == 0) {
		LOG("mipi config file %s error\n", file_buff);
		return (-1);
	}
	fp = fopen(file_buff, "r");
	if (fp == NULL) {
		LOG("mipi config file %s open error\n", file_buff);
		return (-1);
	}
	file_cont = malloc(statbuf.st_size + 1);
	if (file_cont == NULL) {
		LOG("mipi parse cfg malloc error\n");
		fclose(fp);
		return (-1);
	}
	memset(file_cont, 0, statbuf.st_size + 1);
	fread(file_cont, 1, statbuf.st_size, fp);
	fclose(fp);
	cfg.filename = filename;
	cfg.fps = fps;
	cfg.resolution = resolution;
	cfg.entry_num = entry_num;
	cfg.filecont = file_cont;
	ret = hb_cam_mipi_parse_cfg_1(&cfg, cl);
	DBG("hb_cam_mipi_parse_cfg ret=%d -- %s\n", *ret, (*ret) ?  "error" : "done");
	free(file_cont);
	return *ret;
}

int hb_vin_snrclk_set_en(uint32_t entry_num, uint32_t enable)
{
	int *ret;
	CLIENT *cl;
	hb_vin_snrclk_arg snrclk;

	LOG("hb_vin_snrclk_set_en %d %d\n", entry_num, enable);
	cl = hb_rpc_clnt();
	if (cl == NULL)
		return (-1);

	snrclk.entry = entry_num;
	snrclk.value = enable;
	ret = hb_vin_snrclk_set_en_1(&snrclk, cl);
	DBG("hb_vin_snrclk_set_en ret=%d -- %s\n", *ret, (*ret) ?  "error" : "done");
	return *ret;
}

int hb_vin_snrclk_set_freq(uint32_t entry_num, uint32_t freq)
{
	int *ret;
	CLIENT *cl;
	hb_vin_snrclk_arg snrclk;

	LOG("hb_vin_snrclk_set_freq %d %d\n", entry_num, freq);
	cl = hb_rpc_clnt();
	if (cl == NULL)
		return (-1);

	snrclk.entry = entry_num;
	snrclk.value = freq;
	ret = hb_vin_snrclk_set_freq_1(&snrclk, cl);
	DBG("hb_vin_snrclk_set_freq ret=%d -- %s\n", *ret, (*ret) ?  "error" : "done");
	return *ret;
}

int hb_vin_pre_request(uint32_t entry_num, uint32_t type, uint32_t timeout)
{
	int *ret;
	CLIENT *cl;
	hb_vin_pre_arg pre;

	LOG("hb_vin_pre_request %d %d %d\n", entry_num, type, timeout);
	cl = hb_rpc_clnt();
	if (cl == NULL)
		return (-1);

	pre.entry = entry_num;
	pre.type = type;
	pre.value = timeout;
	ret = hb_vin_pre_request_1(&pre, cl);
	DBG("hb_vin_pre_request ret=%d -- %s\n", *ret, (*ret) ?  "error" : "done");
	return *ret;
}

int hb_vin_pre_result(uint32_t entry_num, uint32_t type, uint32_t result)
{
	int *ret;
	CLIENT *cl;
	hb_vin_pre_arg pre;

	LOG("hb_vin_pre_result %d %d %d\n", entry_num, type, result);
	cl = hb_rpc_clnt();
	if (cl == NULL)
		return (-1);

	pre.entry = entry_num;
	pre.type = type;
	pre.value = result;
	ret = hb_vin_pre_result_1(&pre, cl);
	DBG("hb_vin_pre_result ret=%d -- %s\n", *ret, (*ret) ?  "error" : "done");
	return *ret;
}

