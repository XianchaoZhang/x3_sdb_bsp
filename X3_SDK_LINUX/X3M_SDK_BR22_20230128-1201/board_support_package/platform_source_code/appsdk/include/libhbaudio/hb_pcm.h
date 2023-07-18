/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2016 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/

#ifndef AUDIO_SERVER_INCLUDE_HB_PCM_H_
#define AUDIO_SERVER_INCLUDE_HB_PCM_H_

#include "./hb_common.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define HB_AUDIO_SOCK_MAX 8
#define AUDIO_EMPTY (-1)

#define PCM_ERROR_MAX 128

/** A PCM handle.
 * @ingroup libtinyalsa-pcm
 */
struct pcm {
    /** The PCM's file descriptor */
    int fd;
    /** Flags that were passed to @ref pcm_open */
    unsigned int flags;
    /** Whether the PCM is running or not */
    int running:1;
    /** Whether or not the PCM has been prepared */
    int prepared:1;
    /** The number of underruns that have occured */
    int underruns;
    /** Size of the buffer */
    unsigned int buffer_size;
    /** The boundary for ring buffer pointers */
    unsigned int boundary;
    /** Description of the last error that occured */
    char error[PCM_ERROR_MAX];
    /** Configuration that was passed to @ref pcm_open */
    struct pcm_config config;
    /** Current hw pointer position and status */
    struct snd_pcm_mmap_status *mmap_status;
    /** Current sw pointer position and status */
    struct snd_pcm_mmap_control *mmap_control;
    /** pcm sync ptr */
    struct snd_pcm_sync_ptr *sync_ptr;
    /** mmap buffer */
    void *mmap_buffer;
    /** frame per msec of noirq */
    unsigned int noirq_frames_per_msec;
    /** The delay of the PCM, in terms of frames */
    long pcm_delay;
    /** The subdevice corresponding to the PCM */
    unsigned int subdevice;
};

struct audio_client_node {
    int32_t pcm_fd;
    int32_t sock_fd;
    uint32_t card;
    uint32_t device;
    struct pcm_config config;
};

extern struct pcm *hb_pcm_open(uint32_t card, uint32_t device,
                     uint32_t flags, const struct pcm_config *config);
extern int32_t hb_pcm_readi(struct pcm *pcm, void *data, uint32_t frame_count);
extern int hb_pcm_close(struct pcm *pcm);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif  // AUDIO_SERVER_INCLUDE_HB_PCM_H_
