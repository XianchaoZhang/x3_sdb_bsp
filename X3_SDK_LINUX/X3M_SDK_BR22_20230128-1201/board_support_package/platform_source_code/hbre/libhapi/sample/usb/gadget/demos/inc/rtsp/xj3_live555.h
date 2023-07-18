#ifndef __XJ3_LIVE555_H__
#define __XJ3_LIVE555_H__

int hb_start_rtsp_server();
int hb_close_rtsp_server();
int hb_h264_data_push(int cmd, int index, unsigned char* data, unsigned int length);
int hb_pcma_data_push(unsigned char* data, unsigned int length);

#endif
