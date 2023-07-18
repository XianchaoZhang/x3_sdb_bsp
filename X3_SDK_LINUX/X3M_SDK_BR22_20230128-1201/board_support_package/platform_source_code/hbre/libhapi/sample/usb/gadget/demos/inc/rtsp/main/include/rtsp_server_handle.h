#ifndef RTSP_SERVER_HANDLE_H
#define RTSP_SERVER_HANDLE_H

#ifdef __cplusplus
extern "C"{
#endif
int rtsp_server_init();
int rtsp_server_uninit();
int rtsp_server_start();
int rtsp_server_stop();
int rtsp_server_param_get(void* param, unsigned int length);
int rtsp_server_param_set(void* param, unsigned int length);
int rtsp_server_h264_data_put(int cmd, int index, unsigned char* data, unsigned int length);
int rtsp_server_pcma_data_put(unsigned char* data, unsigned int length);
#ifdef __cplusplus
}
#endif

#endif