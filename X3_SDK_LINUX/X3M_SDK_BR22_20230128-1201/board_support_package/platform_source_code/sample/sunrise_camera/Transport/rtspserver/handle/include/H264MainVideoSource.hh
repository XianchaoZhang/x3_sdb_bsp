#ifndef _H264_MAIN_VIDEO_SOURCE_HH_
#define _H264_MAIN_VIDEO_SOURCE_HH_

#include "FramedSource.hh"
#include "utils/stream_manager.h"

/*extern shm_stream_t* 		fH264LiveShmSource;*/

class H264MainVideoSource : public FramedSource {
public:
	static H264MainVideoSource* createNew(UsageEnvironment& env,
		char *shmId, char *shmName, int streamBufSize, int frameRate,
		unsigned preferredFrameSize = 0,
		unsigned playTimePerFrame = 0);
	// "preferredFrameSize" == 0 means 'no preference'
	// "playTimePerFrame" is in microseconds

	void sync();
	void idr();

protected:
	H264MainVideoSource(UsageEnvironment& env,
		char *shmId, char *shmName, int streamBufSize, int frameRate,
		unsigned preferredFrameSize = 0,
		unsigned playTimePerFrame = 0);
	// called only by createNew()

	virtual ~H264MainVideoSource();

	static void incomingDataHandler(H264MainVideoSource* source);
	void incomingDataHandler1();

private:
	// redefined virtual functions:
	virtual void doGetNextFrame();
	
private:
	shm_stream_t* 		fShmSource;
	unsigned 	fPreferredFrameSize;
	unsigned 	fPlayTimePerFrame;
	unsigned 	fLastPlayTime;
	Boolean  	fLimitNumBytesToStream;
	u_int64_t 	fNumBytesToStream; // used iff "fLimitNumBytesToStream" is True
	unsigned long long	fPts;
	unsigned int		fNaluLen;
};


#endif
