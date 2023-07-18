#ifndef RTSP_SVR_WRAP_HH
#define RTSP_SVR_WRAP_HH

void* rtspsvr_wrap_instance();
int rtspsvr_wrap_prepare(const char* stream_name, unsigned short port,
	int audioEnable, int audioType, int audioSampleRate, int audioBitPerSample, int audioChannles,
	int videoEnable, int videoType, int videoFrameRate);
int rtspsvr_wrap_reclaim();
int rtspsvr_wrap_start();
int rtspsvr_wrap_stop();
int rtspsvr_wrap_restart();

#endif