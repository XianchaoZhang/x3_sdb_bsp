#include "GroupsockHelper.hh"
#include "H264MainVideoSource.hh"
#include "utils/utils_log.h"
#include "utils/stream_define.h"
#include "utils/stream_manager.h"
#include "utils/nalu_utils.h"

#include "communicate/sdk_communicate.h"
#include "communicate/sdk_common_struct.h"
#include "communicate/sdk_common_cmd.h"

H264MainVideoSource* H264MainVideoSource::createNew(UsageEnvironment& env,
	char *shmId, char *shmName, int streamBufSize, int frameRate,
	unsigned preferredFrameSize,
	unsigned playTimePerFrame)
{
	H264MainVideoSource* source = new H264MainVideoSource(env, shmId, shmName, streamBufSize, frameRate, preferredFrameSize, playTimePerFrame);
	return source;
}
H264MainVideoSource::H264MainVideoSource(UsageEnvironment& env,
	char *shmId, char *shmName, int streamBufSize, int frameRate,
	unsigned preferredFrameSize,
	unsigned playTimePerFrame)
	: FramedSource(env), fPreferredFrameSize(preferredFrameSize), fPlayTimePerFrame(playTimePerFrame), fLastPlayTime(0)
{
	fPresentationTime.tv_sec = 0;
	fPresentationTime.tv_usec = 0;

	/*char shn_name[64] = {0};*/
	/*sprintf(shn_name, "video%p", this);*/
	/*printf("shn_name: %s\n", shn_name);*/
	fShmSource = shm_stream_create(shmId, shmName, STREAM_MAX_USER,
		frameRate, streamBufSize,
		SHM_STREAM_READ, SHM_STREAM_MALLOC);

	LOGI_print("fShmSource: %p, shmId: %s, shmName: %s, streamBufSize: %d, frameRate:%d",
		fShmSource, shmId, shmName, streamBufSize, frameRate);
	fPts = 0;
	fNaluLen = 0;
}

H264MainVideoSource::~H264MainVideoSource()
{
	if(fShmSource != NULL)
	{
		shm_stream_destory(fShmSource);
		fShmSource = NULL;
	}
}

void H264MainVideoSource::sync()
{
	shm_stream_sync(fShmSource);
}

void H264MainVideoSource::idr()
{
	T_SDK_FORCE_I_FARME idr;
	idr.channel = 0;
	idr.num = 1;
	
	SDK_Cmd_Impl(SDK_CMD_CAMERA_VENC_FORCE_IDR, &idr);
}

void H264MainVideoSource::doGetNextFrame() {
	//do read from memory
	incomingDataHandler(this);
}

void H264MainVideoSource::incomingDataHandler(H264MainVideoSource* source) {
	if (!source->isCurrentlyAwaitingData())
	{
		source->doStopGettingFrames(); // we're not ready for the data yet
		return;
	}
	source->incomingDataHandler1();
}

/*extern int hdr_len;*/
/*extern unsigned char *hdr_data;*/
/*int isFirstFrame = 0;*/
void H264MainVideoSource::incomingDataHandler1()
{
	fFrameSize = 0;
	frame_info info;
	unsigned int length;
	unsigned char* data = NULL;

	if (shm_stream_front(fShmSource, &info, &data, &length) == 0)
	{
		NALU_t nalu;
		fFrameSize = length;
		if (fFrameSize > fMaxSize)
		{
			fNumTruncatedBytes = fFrameSize - fMaxSize;
			fFrameSize = fMaxSize;
		}
		else
		{
			fNumTruncatedBytes = 0;
		}
		int ret = get_annexb_nalu(data + fNaluLen, fFrameSize - fNaluLen, &nalu);
		if (ret > 0) fNaluLen += nalu.len + 4;    //记录nalu偏移总长

		//只发送sps pps i p nalu, 其他抛弃
		if (nalu.nal_unit_type == 7 || nalu.nal_unit_type == 8
			|| nalu.nal_unit_type == 1 || nalu.nal_unit_type == 5)
		{
				/*LOGI_print("nal_unit_type:%d data:%p buf:%p len:%u", nalu.nal_unit_type, data,*/
						   /*nalu.buf, nalu.len);*/
				/*LOGI_print("framer video pts:%llu remains:%d length:%d \n", info.pts,*/
						   /*shm_stream_remains(fShmSource), length);*/
			fFrameSize = nalu.len;
			memcpy(fTo, nalu.buf, nalu.len);

			/*printf("fMaxSize=%d, fFrameSize = %d, fNumTruncatedBytes=%d\n", fMaxSize, fFrameSize, fNumTruncatedBytes);*/

			if (fPresentationTime.tv_sec == 0 && fPresentationTime.tv_usec == 0)
			{
				// This is the first frame, so use the current time:
				gettimeofday(&fPresentationTime, NULL);
				fPts = info.pts;
				//LOGI_print("fPts: %llu\n", fPts);
			}
			else if (nalu.nal_unit_type == 1 || nalu.nal_unit_type == 5)
			{
				unsigned uSeconds = fPresentationTime.tv_usec + (info.pts  - fPts);
				fPresentationTime.tv_sec += uSeconds / 1000000;
				fPresentationTime.tv_usec = uSeconds % 1000000;
				fPts = info.pts;
				gettimeofday(&fPresentationTime, NULL);
			}

#if 0
			if (nalu.nal_unit_type == 5) {
				printf("I frame size:%d\n", nalu.len);
			}
#endif
		
			if (nalu.nal_unit_type == 1 || nalu.nal_unit_type == 5)
			{
				fNaluLen = 0;
				fDurationInMicroseconds = 1000 / 50;

				int remains = shm_stream_remains(fShmSource);
				if(remains > 3)
					LOGI_print("fShmSource:%p, framer video pts:%llu length:%d fFrameSize:%d remains:%d", fShmSource, info.pts, length, fFrameSize, remains);

				//该帧发送完毕，包括sps pps等nalu拆分完毕，可以释放
				shm_stream_post(fShmSource);
			}

			nextTask() = envir().taskScheduler().scheduleDelayedTask(fDurationInMicroseconds, (TaskFunc*)FramedSource::afterGetting, this);
		}
		else
		{
			printf("other nal_unit_type\n");
			fNaluLen = 0;
			shm_stream_post(fShmSource);
			fDurationInMicroseconds = 0;
			nextTask() = envir().taskScheduler().scheduleDelayedTask(fDurationInMicroseconds,
				(TaskFunc*)incomingDataHandler, this);
		}
	}
	else
	{
		nextTask() = envir().taskScheduler().scheduleDelayedTask(10,
			(TaskFunc*)incomingDataHandler, this);
	}

}
