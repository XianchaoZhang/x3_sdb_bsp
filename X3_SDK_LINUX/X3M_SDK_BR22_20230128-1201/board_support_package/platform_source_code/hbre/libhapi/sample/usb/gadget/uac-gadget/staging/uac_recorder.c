/*
 * This example reads from uac-recorder PCD device
 * and writes to dump file.
 */

#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
#include <alsa/asoundlib.h>

static int quit = 0;

static void signal_handler(int sig)
{
	if (sig == SIGINT || sig == SIGTERM || sig == SIGABRT)
		quit = 1;
}

void usage(const char *command)
{
	printf("usage: %s <audio_device> <dump_file>\n", command);

	exit(1);
}

int main(int argc, char *argv[])
{
	snd_pcm_t *handle;
	snd_pcm_hw_params_t *params;
	snd_pcm_uframes_t period_size_min;
	snd_pcm_uframes_t period_size_max;
	snd_pcm_uframes_t buffer_size_min;
	snd_pcm_uframes_t buffer_size_max;
	snd_pcm_uframes_t frames;
	unsigned int val;
	char *buffer;
	int dir, size, loops;
	int fd, r;

	if (argc != 3)
		usage(argv[0]);

	/* open file for audio recorder */
	fd = open(argv[2], O_CREAT|O_RDWR, 0666);
	if (fd < 0) {
		fprintf(stderr, "open %s failed\n", argv[2]);
		exit(1);
	}

	/* Open PCM device for recording(capture) */
	r = snd_pcm_open(&handle, "uac_recorder", SND_PCM_STREAM_CAPTURE, 0);
	if (r < 0) {
		fprintf(stderr, "unable to open pcm device: %s\n",
				snd_strerror(r));
		goto err1;
	}

	/* Allocate a hardware parameters object. */
	snd_pcm_hw_params_alloca(&params);

	/* Fill it in with default values. */
	snd_pcm_hw_params_any(handle, params);

	/* Set the desired hardware parameters */

	/* Interleaved mode */
	snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);

	/* Signed 16-bit little-endian format */
	snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S16_LE);

	/* Two Channels (stereo) */
	snd_pcm_hw_params_set_channels(handle, params, 2);

	/* 48000 bits/second sampling rate */
	val = 48000;
	snd_pcm_hw_params_set_rate_near(handle, params, &val, &dir);

	/* set the buffer time */
	r = snd_pcm_hw_params_get_buffer_size_min(params, &buffer_size_min);
	r = snd_pcm_hw_params_get_buffer_size_max(params, &buffer_size_max);
	r = snd_pcm_hw_params_get_period_size_min(params, &period_size_min, NULL);
	r = snd_pcm_hw_params_get_period_size_max(params, &period_size_max, NULL);
	printf("Buffer size range from %lu to %lu\n", buffer_size_min, buffer_size_max);
	printf("Period size range from %lu to %lu\n", period_size_min, period_size_max);

	/* Set period size to period_size_max frames, high latency */
	frames = period_size_max;
	snd_pcm_hw_params_set_period_size_near(handle, params, &frames, &dir);

	/* Write the parameters to the driver */
	r = snd_pcm_hw_params(handle, params);
	if (r < 0) {
		fprintf(stderr, "unable to set hw parameters: %s\n",
				snd_strerror(r));
		goto err2;
	}

	/* get period size(1 period including how many frames) as frames */
	snd_pcm_hw_params_get_period_size(params, &frames, &dir);

	size = frames * 4;	/* 2 bytes/sample, 2 channels */
	buffer = (char *)malloc(size);

	/* We want to loop for 60 seconds */
	snd_pcm_hw_params_get_period_time(params, &val, &dir);
	loops = 60 * 10000 * 1000 / val;

	/* hijack SIGINT, SIGTERM & SIGABRT */
	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);
	signal(SIGABRT, signal_handler);
	while (loops > 0 && !quit) {
		loops--;
		r = snd_pcm_readi(handle, buffer, frames);
		if (r == -EPIPE) {
			/* EPIPE means overrun */
			fprintf(stderr, "overrun occurred\n");
			snd_pcm_prepare(handle);
		} else if (r < 0) {
			fprintf(stderr, "error from read: %s\n",
					snd_strerror(r));
		} else if (r != (int)frames) {
			fprintf(stderr, "short read, read %d frames\n", r);
		}

		printf("snd_pcm_readi %d frames and write to file\n", (int)frames);

		r = write(fd, buffer, size);
		if (r != size)
			fprintf(stderr, "short write: wrote %d bytes\n", r);
	}

	snd_pcm_drain(handle);
	snd_pcm_close(handle);
	free(buffer);
	close(fd);

	return 0;
err2:
	snd_pcm_close(handle);

err1:
	close(fd);

	return 1;
}
