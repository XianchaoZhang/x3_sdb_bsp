#ifndef __RTSP_SVR_HH__
#define __RTSP_SVR_HH__

#include <pthread.h>
#include "BasicUsageEnvironment.hh"
#include "RTSPServer.hh"

class CRtspServer
{
public:
	static CRtspServer* GetInstance();

	bool Create(const char* streamName, portNumBits port,
		bool audioEnable, int audioType, int audioSampleRate, int audioBitPerSample, int audioChannles,
		bool videoEnable, int videoType, int videoFrameRate);
	bool Destory();
	bool Start();
	bool Stop();
	bool Restart();
	
protected:
	CRtspServer();
	~CRtspServer();
	
	static void* ThreadRtspServerProcImpl(void* arg);
	void ThreadRtspServer();
	
private:
	static  CRtspServer* instance;
	bool 	m_Stop;
	char 	m_watchVariable;
	TaskScheduler* 		m_scheduler;
    UsageEnvironment* 	m_env;
    RTSPServer* 		m_rtspServer;
    pthread_t 			m_pThread;
	portNumBits			m_port;
	char				m_streamName[128];
	bool 				m_audioEnable;
	int 				m_audioType;
	int 				m_audioSampleRate;
	int 				m_audioBitPerSample;
	int 				m_audioChannles;
	bool 				m_videoEnable;
	int 				m_videoType;
	int 				m_videoFrameRate;
};

#endif
