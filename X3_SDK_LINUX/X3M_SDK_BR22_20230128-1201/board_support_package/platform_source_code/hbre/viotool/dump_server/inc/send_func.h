#ifndef __SEND_FUNC_H__
#define __SEND_FUNC_H__

int start_send_raw_pic(void);
void stop_send_raw_pic(void);

int start_send_yuv_pic(void);
void stop_send_yuv_pic(void);

int start_send_stats_data(void);
void stop_send_stats_data(void);

#endif
