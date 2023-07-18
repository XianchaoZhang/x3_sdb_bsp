/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2016 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/

#ifndef AUDIO_SERVER_INCLUDE_HB_COMMON_H_
#define AUDIO_SERVER_INCLUDE_HB_COMMON_H_

#include <stdint.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include <tinyalsa/pcm.h>
#include "./audio_log.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define AUDIO_SOCKET_PATH "/userdata/audio_socket"

enum hb_audio_cmd {
    HB_CMD_AUDIO_ERROR = 0,
    HB_CMD_AUDIO_OPEN = 1,
    HB_CMD_AUDIO_READ,
    HB_CMD_AUDIO_CLOSE,
};

enum hb_audio_status {
    HB_STATUS_ERR_AUDIO_INVALID = -6,
    HB_STATUS_ERR_AUDIO_NOMEM = -5,
    HB_STATUS_ERR_AUDIO_PRIMARY_OPEN = -4,
    HB_STATUS_ERR_AUDIO_PRIMARY_CLOSE = -3,
    HB_STATUS_ERR_AUDIO_PRIMARY_READ = -2,
    HB_STATUS_AUDIO_CLOSE = -1,
    HB_STATUS_AUDIO_READY = 0,
    HB_STATUS_AUDIO_OPEN,
    HB_STATUS_AUDIO_READ,
};

/* for audio msg info, client to server*/
struct hb_audio_msg_up {
    int32_t cmd;
    uint32_t card;
    uint32_t device;
    struct pcm_config config;
};

/* for audio msg info, server to client*/
struct hb_audio_msg_down {
    uint32_t group;
    uint32_t card;
    uint32_t device;
    enum hb_audio_status server_status;
};

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif  // AUDIO_SERVER_INCLUDE_HB_COMMON_H_
