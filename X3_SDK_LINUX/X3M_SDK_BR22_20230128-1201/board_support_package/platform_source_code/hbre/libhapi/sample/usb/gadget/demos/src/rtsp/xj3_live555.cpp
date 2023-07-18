#include <stdio.h>
#include <string.h>
#include "rtsp_server_handle.h"
#include "utils_log.h"

int hb_start_rtsp_server()
{
	int ret = 0;
	LOGI_print("hb_start_rtsp_server================\n");
	ret = rtsp_server_init();
	ret = rtsp_server_start();
	return 0;
}

int hb_close_rtsp_server()
{
	int ret;
	ret = rtsp_server_stop();
	ret = rtsp_server_uninit();
	return 0;
}

int hb_h264_data_push(int cmd, int index, unsigned char *data,
		      unsigned int length)
{
	rtsp_server_h264_data_put(cmd, index, data, length);
	return 0;
}

int hb_pcma_data_push(unsigned char *data, unsigned int length)
{
	rtsp_server_pcma_data_put(data, length);
	return 0;
}
