#include "uac_micphone.h"
#include <pthread.h>
#include "utils.h"

static void *uac_micphone_loop(void *arg)
{
	uac_micphone_device_t *dev = (uac_micphone_device_t *)arg;
	snd_pcm_sframes_t frames;
	char *buffer;
	int r, size;

	trace_in();

	if (!dev || !dev->micphone_device || !dev->uac_device) {
		fprintf(stderr, "%s-%d error happen\n", __func__, __LINE__);
		return (void *)0;
	}

	alsa_device_t *micphone_dev = dev->micphone_device;
	alsa_device_t *uac_dev = dev->uac_device;

	// frames = uac_dev->period_size;
	frames = micphone_dev->period_size;
	size = snd_pcm_frames_to_bytes(micphone_dev->handle, frames);
	buffer = malloc(size);

	printf("%s, frames(%d), size(%d)\n", __func__, (int)frames, size);

	/* audio process loop */
	while (!dev->exit) {
		/* read data from micphone device */
		r = alsa_device_read(micphone_dev, buffer, frames);

		/* write data to uac playback device */
		if (r > 0)
			r = alsa_device_write(uac_dev, buffer, frames);
	}

	free(buffer);

	trace_out();

	return (void *)0;
}

int uac_micphone_init(uac_micphone_device_t **uac_micphone)
{
	int r;

	trace_in();

	uac_micphone_device_t *dev = (uac_micphone_device_t *)calloc(1,
			sizeof(uac_micphone_device_t));
	if (!dev) {
		r = -ENOMEM;
		goto err1;
	}

	alsa_device_t *micphone_device = alsa_device_allocate();
	alsa_device_t *uac_device = alsa_device_allocate();

	if (!micphone_device || !uac_device) {
		r = -ENOMEM;
		goto err2;
	}

	/* init micphone device*/
	micphone_device->name		= "micphone";
	micphone_device->format		= SND_PCM_FORMAT_S16;
	micphone_device->direct		= SND_PCM_STREAM_CAPTURE;
	micphone_device->rate		= 48000;
	micphone_device->channels	= 2;
	micphone_device->buffer_time	= 0;		// use default buffer time
	micphone_device->nperiods	= 4;
	micphone_device->period_size	= 1024;		// 1 period including 1024 frames

	r = alsa_device_init(micphone_device);
	if (r < 0) {
		fprintf(stderr, "alsa_device_init micphone failed. r(%d)\n", r);
		goto err3;
	}

	/* init uac recorder device*/
	uac_device->name	= "uac_playback";
	uac_device->format	= SND_PCM_FORMAT_S16;
	uac_device->direct	= SND_PCM_STREAM_PLAYBACK;
	uac_device->rate	= 48000;
	uac_device->channels	= 2;
	uac_device->buffer_time = 0;		// use default buffer time
	uac_device->nperiods	= 4;
	// uac_device->period_size = 1024;		// 1 period including 1024 frames

	r = alsa_device_init(uac_device);
	if (r < 0) {
		fprintf(stderr, "alsa_device_init %s failed. r(%d)\n",
				uac_device->name, r);
		goto err2;
	}

	dev->micphone_device = micphone_device;
	dev->uac_device = uac_device;
	dev->thread_id = 0;
	dev->exit = 0;
	*uac_micphone = dev;

	trace_out();

	return 0;

err3:
	alsa_device_deinit(uac_device);

err2:
	if (micphone_device)
		free(micphone_device);

	if (uac_device)
		free(uac_device);

	if (dev)
		free(dev);
err1:
	return r;
}

int uac_micphone_start(uac_micphone_device_t *uac_micphone)
{
	int r;

	trace_in();

	if (!uac_micphone)
		return -EINVAL;

	uac_micphone->exit = 0;

	r = pthread_create(&uac_micphone->thread_id, NULL, uac_micphone_loop,uac_micphone);
	if (r < 0)
		fprintf(stderr, "create thread uvc_micphone_loop failed\n");

	trace_out();

	return r;
}

int uac_micphone_stop(uac_micphone_device_t *uac_micphone)
{
	int r;

	trace_in();

	if (!uac_micphone)
		return -EINVAL;

	/* force uac micphone stop */
	uac_micphone->exit = 1;

	r = pthread_join(uac_micphone->thread_id, NULL);
	if (r < 0)
		fprintf(stderr, "uvc_micphone thread join failed\n");

	trace_out();

	return r;
}

void uac_micphone_deinit(uac_micphone_device_t *uac_micphone)
{
	trace_in();

	if (!uac_micphone)
		return;

	if (uac_micphone->micphone_device) {
		alsa_device_deinit(uac_micphone->micphone_device);
		alsa_device_free(uac_micphone->micphone_device);
	}

	if (uac_micphone->uac_device) {
		alsa_device_deinit(uac_micphone->uac_device);
		alsa_device_free(uac_micphone->uac_device);
	}

	free(uac_micphone);

	trace_out();
}
