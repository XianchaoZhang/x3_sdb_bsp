#include <stdio.h>
#include <getopt.h>
#include <pthread.h>
#include <signal.h>
#include <poll.h>
#include <sys/eventfd.h>
#include "uac_speaker.h"
#include "uac_micphone.h"

enum {
	OPT_VERSION = 1,
	OPT_ONLY_UAC_SPEAKER,
	OPT_ONLY_UAC_MICPHONE,
	OPT_DUMP_HWPARAMS,
};

#define UAC_SPEAKER_MASK 0x1
#define UAC_MICPHONE_MASK 0x2
#define EVENT_SIGNAL_QUIT 1

static uac_speaker_device_t *uac_speaker = NULL;
static uac_micphone_device_t *uac_micphone = NULL;
static int event_fd = 0;

static void usage(char *command)
{
	printf(
		"Usage: %s [OPTION]... [FILE]...\n"
		"\n"
		"-h, --help              help\n"
		"    --version           print current version\n"
		"    --only-uac-speaker  only run uac speaker\n"
		"    --only-uac-micphone only run uac micphone\n"
		"    --dump-hw-params    dump hw_params of the device\n"
		, command);
}

static void version(char *command)
{
	printf("%s version. 1.0\n", command);
}

static void signal_handler(int signal)
{
	uint64_t event;

	if (signal == SIGINT || signal == SIGTERM
			|| signal == SIGABRT) {
		event = EVENT_SIGNAL_QUIT;

		printf("%s receive signal(%d)\n", __func__, signal);

		if (event_fd) {
			write(event_fd, &event, sizeof(uint64_t));
			printf("write singal out event to event fd\n");
		}
	}
}

void main_loop()
{
	struct pollfd pfd;
	uint64_t event;
	ssize_t sz;
	int quit = 0;

	printf("%s function in\n", __func__);

	event_fd = eventfd(0, 0);

	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);
	signal(SIGABRT, signal_handler);

	pfd.fd = event_fd;
	pfd.events = POLLIN;
	while (!quit) {
		if (poll(&pfd, 1, 0) == 1) {
			sz = read(event_fd, &event, sizeof(uint64_t));
			if (sz != sizeof(uint64_t)) {
				fprintf(stderr, "eventfd read abnormal. sz(%lu)\n",
						sz);
				continue;
			}

			printf("receive event %lu\n", event);

			switch (event) {
			case EVENT_SIGNAL_QUIT:
				printf("recv keyboard quit event\n");
				quit = 1;
				break;
			default:
				break;
			}
		}

		if (quit)
			break;

		usleep(300 * 1000);
	}

	printf("%s function out\n", __func__);
}

static int launch_uac_micphone()
{
	int r;

	r = uac_micphone_init(&uac_micphone);
	if (r < 0)
		goto err1;

	r = uac_micphone_start(uac_micphone);
	if (r < 0)
		goto err2;

	return 0;

err2:
	uac_micphone_deinit(uac_micphone);
	uac_micphone = NULL;

err1:
	return r;
}

static int launch_uac_speaker()
{
	int r;

	r = uac_speaker_init(&uac_speaker);
	if (r < 0)
		goto err1;

	r = uac_speaker_start(uac_speaker);
	if (r < 0)
		goto err2;

	return 0;

err2:
	uac_speaker_deinit(uac_speaker);
	uac_speaker = NULL;

err1:
	return r;
}

void stop_uac_micphone()
{
	if (!uac_micphone)
		return;

	uac_micphone_stop(uac_micphone);

	uac_micphone_deinit(uac_micphone);
	uac_micphone = NULL;
}

void stop_uac_speaker()
{
	if (!uac_speaker)
		return;

	uac_speaker_stop(uac_speaker);

	uac_speaker_deinit(uac_speaker);
	uac_speaker = NULL;
}

int launch_app(int mask)
{
	int r1 = -1;
	int r2 = -1;

	if (mask & UAC_MICPHONE_MASK) {
		printf("lanch uac micphone\n");
		r1 = launch_uac_micphone();
	}

	if (mask & UAC_SPEAKER_MASK) {
		printf("lanch uac speaker\n");
		r2 = launch_uac_speaker();
	}

	printf("%s, r1(%d), r2(%d)\n", __func__, r1, r2);

	return (r1 & r2);
}

void stop_app(int mask)
{
	if (mask & UAC_MICPHONE_MASK)
		stop_uac_micphone();


	if (mask & UAC_SPEAKER_MASK)
		stop_uac_speaker();
}

int main(int argc, char *argv[])
{
	char *command;
	int option_index, app_mask;
	int r, c;

	static const char short_options[] = "h";
	static const struct option long_options[] = {
		{"help", 0, 0, 'h'},
		{"version", 0, 0, OPT_VERSION},
		{"only-uac-speaker", 0, 0, OPT_ONLY_UAC_SPEAKER},
		{"only-uac-micphone", 0, 0, OPT_ONLY_UAC_MICPHONE},
		{"dump-hw-params", 0, 0, OPT_DUMP_HWPARAMS},
		{0, 0, 0, 0}
	};

	app_mask = UAC_SPEAKER_MASK | UAC_MICPHONE_MASK;
	command = argv[0];
	while ((c = getopt_long(argc, argv, short_options, long_options,
					&option_index)) != -1) {
		switch (c) {
		case 'h':
			usage(command);
			return 0;
		case OPT_VERSION:
			version(command);
			return 0;
		case OPT_ONLY_UAC_SPEAKER:
			app_mask = UAC_SPEAKER_MASK;
			break;
		case OPT_ONLY_UAC_MICPHONE:
			app_mask = UAC_MICPHONE_MASK;
			break;
		case OPT_DUMP_HWPARAMS:
			alsa_device_debug_enable(1);
			break;
		default:
			fprintf(stderr, "Try `%s --help' for more information.\n",
					command);
			return 1;
		}
	}

	printf("launch_app app_mask(%d)\n", app_mask);

	r = launch_app(app_mask);
	if (r)
		goto err;

	main_loop();

	stop_app(app_mask);

	printf("main function exit\n");

	return 0;

err:
	return r;
}
