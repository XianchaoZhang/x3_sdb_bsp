#ifdef __cplusplus
extern "C"{
#endif
#include "RtspSvrWrap.hh"
#ifdef __cplusplus
}
#endif

#include "RtspSvr.hh"

static  CRtspServer* instance = NULL;
void* rtspsvr_wrap_instance()
{
	if(instance == NULL)
	{
		instance = CRtspServer::GetInstance();
	}
	return (void*)instance;
}

int rtspsvr_wrap_prepare(void* instance, unsigned short port)
{
	bool result = ((CRtspServer*)instance)->Create(port);
	if(result)
		return 0;
	else
		return -1;
}

int rtspsvr_wrap_reclaim(void* instance)
{
	bool result = ((CRtspServer*)instance)->Destory();
	if(result)
		return 0;
	else
		return -1;
}

int rtspsvr_wrap_start(void* instance)
{
	bool result = ((CRtspServer*)instance)->Start();
	if(result)
		return 0;
	else
		return -1;
}

int rtspsvr_wrap_stop(void* instance)
{
	bool result = ((CRtspServer*)instance)->Stop();
	if(result)
		return 0;
	else
		return -1;
}

int rtspsvr_wrap_restart(void* instance)
{
	bool result = ((CRtspServer*)instance)->Restart();
	if(result)
		return 0;
	else
		return -1;
}

int rtspsvr_wrap_add_sms(void* instance, const char* streamName,
	int audioEnable, int audioType, int audioSampleRate, int audioBitPerSample,
	int audioChannles, int videoEnable, int videoType, int videoFrameRate,
	char *shmId, char *shmName, int streamBufSize, int frameRate)
{
	bool result = ((CRtspServer*)instance)->DynamicAddSms(streamName,
		audioEnable, audioType, audioSampleRate, audioBitPerSample, audioChannles,
		videoEnable, videoType, videoFrameRate, shmId, shmName, streamBufSize, frameRate);
	if(result)
		return 0;
	else
		return -1;
}

int rtspsvr_wrap_del_sms(void* instance, const char* streamName)
{
	bool result = ((CRtspServer*)instance)->DynamicDelSms(streamName);
	if(result)
		return 0;
	else
		return -1;
}

