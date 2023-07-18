/**********
This library is free software; you can redistribute it and/or modify it under
the terms of the GNU Lesser General Public License as published by the
Free Software Foundation; either version 2.1 of the License, or (at your
option) any later version. (See <http://www.gnu.org/copyleft/lesser.html>.)

This library is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for
more details.

You should have received a copy of the GNU Lesser General Public License
along with this library; if not, write to the Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
**********/
// "liveMedia"
// Copyright (c) 1996-2014 Live Networks, Inc.  All rights reserved.
// A 'ServerMediaSubsession' object that creates new, unicast, "RTPSink"s
// on demand, from a H264 Elementary Stream video file.
// C++ header

#ifndef _LPCM_AUDIO_LIVE_SERVER_MEDIA_SUBSESSION_HH
#define _LPCM_AUDIO_LIVE_SERVER_MEDIA_SUBSESSION_HH

#ifndef _ON_DEMAND_SERVER_MEDIA_SUBSESSION_HH
#include "OnDemandServerMediaSubsession.hh"
#endif

#include "LPCMAudioSource.hh"

class LPCMAudioLiveServerMediaSubsession : public OnDemandServerMediaSubsession {
public:
	static LPCMAudioLiveServerMediaSubsession*
		createNew(UsageEnvironment& env, Boolean reuseFirstSource,
		u_int8_t bitsPerSample, 
		u_int8_t samplingFrequencyIndex, 
		u_int8_t channelConfiguration);

protected:
	LPCMAudioLiveServerMediaSubsession(UsageEnvironment& env, Boolean reuseFirstSource,
		u_int8_t bitsPerSample, 
		u_int8_t samplingFrequencyIndex, 
		u_int8_t channelConfiguration);
	// called only by createNew();
	virtual ~LPCMAudioLiveServerMediaSubsession();

	void startStream(unsigned clientSessionId,
						void* streamToken,
						TaskFunc* rtcpRRHandler,
						void* rtcpRRHandlerClientData,
						unsigned short& rtpSeqNum,
						unsigned& rtpTimestamp,
						ServerRequestAlternativeByteHandler* serverRequestAlternativeByteHandler,
						void* serverRequestAlternativeByteHandlerClientData);

protected: // redefined virtual functions
	virtual FramedSource* createNewStreamSource(unsigned clientSessionId,
		unsigned& estBitrate);
	virtual RTPSink* createNewRTPSink(Groupsock* rtpGroupsock,
		unsigned char rtpPayloadTypeIfDynamic,
		FramedSource* inputSource);

private:
	unsigned fBitsPerSample;
	unsigned fSamplingFrequencyIndex;
	unsigned fNumChannels;
	LPCMAudioSource* fAudioSource;
};

#endif
