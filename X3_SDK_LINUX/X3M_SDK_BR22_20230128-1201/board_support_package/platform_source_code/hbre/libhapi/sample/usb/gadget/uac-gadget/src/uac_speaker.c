#include "uac_speaker.h"
#include <pthread.h>
#include "utils.h"

static void *uac_speaker_loop(void *arg)
{
	uac_speaker_device_t *dev = (uac_speaker_device_t *)arg;
	snd_pcm_sframes_t frames;
	char *buffer;
	int r, size;

	trace_in();

	if (!dev || !dev->uac_device || !dev->speaker_device) {
		fprintf(stderr, "%s-%d error happen\n", __func__, __LINE__);
		return (void *)0;
	}

	alsa_device_t *uac_dev = dev->uac_device;
	alsa_device_t *speaker_dev = dev->speaker_device;

	// frames = uac_dev->period_size;
	frames = speaker_dev->period_size;
	size = snd_pcm_frames_to_bytes(speaker_dev->handle, frames);
	buffer = malloc(size);

	printf("%s, frames(%d), size(%d)\n", __func__, (int)frames, size);

	/* audio process loop */
	while (!dev->exit) {
		/* read data from uac recorder device */
		r = alsa_device_read(uac_dev, buffer, frames);

		/* write data to speaker device */
		if (r > 0)
			r = alsa_device_write(speaker_dev, buffer, frames);
	}

	free(buffer);

	trace_out();

	return (void *)0;
}

int uac_speaker_init(uac_speaker_device_t **uac_speaker)
{
	int r;

	trace_in();

	uac_speaker_device_t *dev = (uac_speaker_device_t *)calloc(1,
			sizeof(uac_speaker_device_t));
	if (!dev) {
		r = -ENOMEM;
		goto err1;
	}

	alsa_device_t *uac_device = alsa_device_allocate();
	alsa_device_t *speaker_device = alsa_device_allocate();

	if (!uac_device || !speaker_device) {
		r = -ENOMEM;
		goto err2;
	}

	/* init uac recorder device*/
	uac_device->name	= "uac_recorder";
	uac_device->format	= SND_PCM_FORMAT_S16;
	uac_device->direct	= SND_PCM_STREAM_CAPTURE;
	uac_device->rate	= 48000;
	uac_device->channels	= 2;
	uac_device->buffer_time = 0;		// use default buffer time
	uac_device->nperiods	= 4;
	uac_device->period_size = 1024;		// 1 period including 1024 frames

	r = alsa_device_init(uac_device);
	if (r < 0) {
		fprintf(stderr, "alsa_device_init %s failed. r(%d)\n",
				uac_device->name, r);
		goto err2;
	}

	/* init speaker device*/
	speaker_device->name		= "speaker";
	speaker_device->format		= SND_PCM_FORMAT_S16;
	speaker_device->direct		= SND_PCM_STREAM_PLAYBACK;
	speaker_device->rate		= 48000;
	speaker_device->channels	= 2;
	speaker_device->buffer_time	= 0;		// use default buffer time
	speaker_device->nperiods	= 4;
	// speaker_device->period_size	= 1024;		// 1 period including 1024 frames

	r = alsa_device_init(speaker_device);
	if (r < 0) {
		fprintf(stderr, "alsa_device_init speaker failed. r(%d)\n", r);
		goto err3;
	}

	dev->uac_device = uac_device;
	dev->speaker_device = speaker_device;
	dev->thread_id = 0;
	dev->exit = 0;
	*uac_speaker = dev;

	trace_out();

	return 0;

err3:
	alsa_device_deinit(uac_device);

err2:
	if (speaker_device)
		free(speaker_device);

	if (uac_device)
		free(uac_device);

	if (dev)
		free(dev);
err1:
	return r;
}

int uac_speaker_start(uac_speaker_device_t *uac_speaker)
{
	int r;

	trace_in();

	if (!uac_speaker)
		return -EINVAL;

	uac_speaker->exit = 0;

	r = pthread_create(&uac_speaker->thread_id, NULL, uac_speaker_loop,uac_speaker);
	if (r < 0)
		fprintf(stderr, "create thread uvc_speaker_loop failed\n");

	trace_out();

	return r;
}

int uac_speaker_stop(uac_speaker_device_t *uac_speaker)
{
	int r;

	trace_in();

	if (!uac_speaker)
		return -EINVAL;

	/* force uac speaker stop */
	uac_speaker->exit = 1;

	r = pthread_join(uac_speaker->thread_id, NULL);
	if (r < 0)
		fprintf(stderr, "uvc_speaker thread join failed\n");

	trace_out();

	return r;
}

void uac_speaker_deinit(uac_speaker_device_t *uac_speaker)
{
	trace_in();

	if (!uac_speaker)
		return;

	if (uac_speaker->uac_device) {
		alsa_device_deinit(uac_speaker->uac_device);
		alsa_device_free(uac_speaker->uac_device);
	}

	if (uac_speaker->speaker_device) {
		alsa_device_deinit(uac_speaker->speaker_device);
		alsa_device_free(uac_speaker->speaker_device);
	}

	free(uac_speaker);

	trace_out();
}
