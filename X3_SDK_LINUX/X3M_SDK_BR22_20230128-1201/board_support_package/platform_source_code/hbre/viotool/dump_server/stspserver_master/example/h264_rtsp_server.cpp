#include <iostream>

#include "base/Logging.h"
#include "net/UsageEnvironment.h"
#include "base/ThreadPool.h"
#include "net/EventScheduler.h"
#include "net/Event.h"
#include "net/RtspServer.h"
#include "net/MediaSession.h"
#include "net/InetAddress.h"
#include "net/H264FileMediaSource.h"
#include "net/H264RtpSink.h"
#include "h264server.h"

void h264_rtp_start(void)
{
    usleep(3000000);
    //Logger::setLogFile("xxx.log");
    Logger::setLogLevel(Logger::LogWarning);
    std::cout<< __func__ << __LINE__<<std::endl;

    EventScheduler* scheduler = EventScheduler::createNew(EventScheduler::POLLER_SELECT);
    ThreadPool* threadPool = ThreadPool::createNew(2);
    UsageEnvironment* env = UsageEnvironment::createNew(scheduler, threadPool);

    std::cout<< __func__ << __LINE__<<std::endl;
    Ipv4Address ipAddr("0.0.0.0", 8554);
    RtspServer* server = RtspServer::createNew(env, ipAddr);
    MediaSession* session = MediaSession::createNew("live");
    MediaSource* mediaSource = H264FileMediaSource::createNew(env, "abc");
    RtpSink* rtpSink = H264RtpSink::createNew(env, mediaSource);

    std::cout<< __func__ << __LINE__<<std::endl;
    session->addRtpSink(MediaSession::TrackId0, rtpSink);
    //session->startMulticast(); //多播

    std::cout<< __func__ << __LINE__<<std::endl;
    server->addMeidaSession(session);
    server->start();

    std::cout<< __func__ << __LINE__<<std::endl;
    std::cout<<"Play the media using the URL \""<<server->getUrl(session)<<"\""<<std::endl;

    env->scheduler()->loop();

}
