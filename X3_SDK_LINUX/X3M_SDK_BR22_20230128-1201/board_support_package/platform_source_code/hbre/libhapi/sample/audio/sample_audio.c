/*************************************************************************
 *                     COPYRIGHT NOTICE
 *            Copyright 2019 Horizon Robotics, Inc.
 *                   All rights reserved.
 *************************************************************************/
#include <stdio.h>
#include <pthread.h>
#include "hb_audio_codec.h"
#include "hb_audio_io.h"

#define TAG "sample_audio"

typedef struct AudioRecorderTestContext {
	int error;
	int terminate;
	int time;
	int sample_rate;
	int sample_fmt;
	int channel;
	struct HB_AIO_DEVICE_ATTR_S *ainAttr;
	struct HB_AIO_DEVICE_ATTR_S *aotAttr;
	struct HB_AENC_CHN_ATTR_S *encAttr;
	struct HB_ADEC_CHN_ATTR_S *decAttr;
	pthread_t audio_capture_thread;
	pthread_t audio_encoder_thread;
	int encoder_thread_finished;
	int capture_thread_finished;
	pthread_t audio_decoder_thread;
	pthread_t audio_playback_thread;
	int decoder_thread_finished;
	int playback_thread_finished;
} AudioRecorderTestContext;

static int stopCapture(AudioRecorderTestContext *pstCtx) {
	int ret = 0;
	int devId = 0;
	int chn = 2;
	if (!pstCtx || !pstCtx->ainAttr) {
		printf("%s Invalid parameters\n", TAG);
		return -1;
	}

	ret = HB_AIN_Disable(devId);
	if (ret) {
		printf("%s Disable AIN pcm device error!\n", TAG);
		return -1;
	}

	printf("%s Success to stop audio capture.\n", TAG);
	return 0;
}

static int audio_capture_init(AudioRecorderTestContext *pstCtx) {
	int ret = 0;
	int devId = 0;
	int bytes_per_frame;
	int format;
	struct HB_AIO_DEVICE_ATTR_S *ainAttr;
	if (!pstCtx || !pstCtx->ainAttr) {
		printf("%s Invalid capture task!!!\n", TAG);
		return -1;
	}

	ainAttr = pstCtx->ainAttr;
	ainAttr->periodCount = 4;
	switch (ainAttr->sampleFmt) {
	case audio_sample_format_16:
		format = 16;
		break;
	case audio_sample_format_8:
		format = 8;
		break;
	default:
		format = 16;
		break;
	}
	bytes_per_frame = ainAttr->channels * (format/8);
	ainAttr->periodSize = pstCtx->encAttr->aeParams.frame_size/
		(bytes_per_frame * ainAttr->periodCount);
	ret = HB_AIN_SetPubAttr(devId, pstCtx->ainAttr);
	if (ret) {
		printf("%s Set AIN parameter error\n", TAG);
		return -1;
	}
	ret = HB_AIN_Enable(devId);
	if (ret) {
		printf("%s Enable pcm device error\n", TAG);
		return -1;
	}
	return 0;
}

int startCapture(AudioRecorderTestContext *pstCtx) {
	int ret = 0;
	uint8_t aeChn = 1;

	if (!pstCtx || !pstCtx->encAttr) {
                printf("%s Invalid parameters.\n", TAG);
                return -1;
        }

	// setup audio pcm parameter
	struct HB_AIO_DEVICE_ATTR_S *pstAinAttr;
	pstAinAttr = pstCtx->ainAttr;
	pstAinAttr->channels = pstCtx->channel;
	pstAinAttr->sampleRate = pstCtx->sample_rate;
	pstAinAttr->sampleFmt = pstCtx->sample_fmt;

	// setup audio encoder parameter
	struct HB_AENC_CHN_ATTR_S *pstAencAttr;
	struct HB_AENC_PARAMS *pstAencParam;
	pstAencAttr = pstCtx->encAttr;
	pstAencParam = &pstAencAttr->aeParams;
	struct AENC_ATTR_G711A g711a;
	pstAencAttr->type = PT_G711A;
	pstAencParam->bit_rate = 16000;
	pstAencParam->frame_buf_count = 5;
	pstAencParam->packet_count = 5;
	pstAencParam->sample_rate = pstCtx->sample_rate;
	pstAencParam->sample_fmt = pstCtx->sample_fmt;
	pstAencParam->channels = pstCtx->channel;
	if (pstAencParam->channels == 1) {
		pstAencParam->channel_layout = MC_AV_CHANNEL_LAYOUT_MONO;
	} else {
		pstAencParam->channel_layout = MC_AV_CHANNEL_LAYOUT_STEREO;
	}
	pstAencParam->stG711a = g711a;

	ret = HB_AENC_CreateChn(aeChn, pstAencAttr);
	if (ret) {
		printf("%s Create AENC chn error\n", TAG);
		return -1;
	}

	ret = audio_capture_init(pstCtx);
	if (ret < 0) {
		printf("%s Open pcm device error\n", TAG);
		return -1;
	}

	printf("%s Success to start capture\n", TAG);
	return 0;
}

int stopEncoder(AudioRecorderTestContext *pstCtx) {
	int ret = 0;
	uint8_t aeChn = 1;
	void *status;
	if (!pstCtx && !pstCtx->encAttr) {
		printf("%s Invalid recorder task!\n", TAG);
		return -1;
	}
	if (pstCtx->audio_encoder_thread) {
		if ((ret = pthread_join(pstCtx->audio_encoder_thread, &status)) == 0) {
			pstCtx->audio_encoder_thread = 0;
		} else {
			printf("%s Failed to pthread_join ret(%d)\n", TAG, ret);
		}
	}

	ret = HB_AENC_DestroyChn(aeChn);
	if (ret) {
		printf("%s Destory AENC chn error!\n", TAG);
		return -1;
	}

	printf("%s Success to stop audio encoder.\n", TAG);

	return ret;
}

static hb_s32 read_audio_frame(AudioRecorderTestContext *pstCtx) {
	int ret = 0;
	uint8_t devId = 0;
	uint8_t aeChn = 1;
	struct HB_AENC_CHN_ATTR_S *acodeAttr;
	if (!pstCtx || !pstCtx->encAttr) {
		printf("%s Failed to read audio frame\n", TAG);
		return -1;
	}

	acodeAttr = pstCtx->encAttr;
	struct HB_AIO_FRAME_S pcm_data;
	memset(&pcm_data, 0x00, sizeof(struct HB_AIO_FRAME_S));
	struct HB_AUDIO_FRAME_S input_frame;
	memset(&input_frame, 0x00, sizeof(struct HB_AUDIO_FRAME_S));

	ret = HB_AIN_GetFrame(devId, aeChn, &pcm_data, NULL, 10);
	if (ret) {
		printf("%s Get pcm data error\n", TAG);
		return -1;
	}

	input_frame.len = pcm_data.size;
	input_frame.virAddr = pcm_data.data;

	ret = HB_AENC_SendFrame(aeChn, &input_frame, NULL);
	if (ret) {
		printf("%s Send AENC frame error\n", TAG);
		return -1;
	}

	return 0;
}

static void *audio_encoder_thread(void *arg) {
	int ret = 0;
	int m = 0;
	AudioRecorderTestContext *ctx = (AudioRecorderTestContext *)arg;
	if (!ctx) {
		printf("%s Invalid parameter\n", TAG);
		return NULL;
	}

	while (!ret && !ctx->encoder_thread_finished) {
		ret = read_audio_frame(ctx);
		if (ret) {
			ctx->error = 1;
			break;
		}

		m++;
		if (m > ctx->time)
			break;
	}

	printf("%s Exit %s(ret = %d, stopping=%d, audio finished=%d)!\n",
		TAG, __func__, ret, ctx->terminate,
		ctx->encoder_thread_finished);

	return NULL;
}

static int startEncoder(AudioRecorderTestContext *pstCtx) {
	int ret;
	int thread_ret = 0;
	int aeChn = 1;
	struct HB_AENC_CHN_ATTR_S *acodeAttr;
	if (!pstCtx || !pstCtx->encAttr) {
		printf("%s Invalid parameter\n", TAG);
		return -1;
	}

	acodeAttr = pstCtx->encAttr;
	if ((thread_ret =
		pthread_create(&pstCtx->audio_encoder_thread, NULL,
			audio_encoder_thread, pstCtx) != 0)) {
		printf("%s Failed to pthread_create ret(%d)\n", TAG, thread_ret);
		ret = HB_AENC_DestroyChn(aeChn);
		if (ret < 0) {
			printf("%s destroy aenc chn error\n", TAG);
			return -1;
		}
		return -1;
	}

	printf("%s Success to start audio encoder\n", TAG);
	return 0;
}

static int stopDecoder(AudioRecorderTestContext *pstCtx) {
	int ret = 0;
	int adChn = 1;
	void *status;
	if (!pstCtx || !pstCtx->decAttr) {
		printf("%s Invalid decorder task!\n", TAG);
		return -1;
	}

	if (pstCtx->audio_decoder_thread) {
		if ((ret = pthread_join(pstCtx->audio_decoder_thread, &status)) == 0) {
			pstCtx->audio_decoder_thread = 0;
		} else {
			printf("%s Failed to pthread_join ret(%d)\n", TAG, ret);
		}
	}

	ret = HB_ADEC_DestroyChn(adChn);
	if (ret) {
		printf("%s Destroy ADEC chn error!\n", TAG);
		return -1;
	}

	printf("%s Success to stop audio decoder.\n", TAG);
	return 0;
}

static int read_audio_stream(AudioRecorderTestContext *pstCtx) {
	int ret = 0;
	int aeChn = 1;
	int adChn = 1;
	bool block = 0;
	bool bClear = 0;
	struct HB_ADEC_CHN_ATTR_S *pstAdecAttr;
	struct HB_AUDIO_STREAM_S input_buffer;
	struct HB_AUDIO_STREAM_S stream_buffer;
	if (!pstCtx || !pstCtx->decAttr) {
		printf("%s Invalid parameter\n", TAG);
		return -1;
	}

	memset(&stream_buffer, 0x00, sizeof(struct HB_AUDIO_STREAM_S));
	ret = HB_AENC_GetStream(aeChn, &stream_buffer, -1);
	if (ret) {
		printf("%s Get AENC stream buffer error\n", TAG);
		return -1;
	}
	pstAdecAttr = pstCtx->decAttr;
	memset(&input_buffer, 0x00, sizeof(struct HB_AUDIO_STREAM_S));
	input_buffer.len = stream_buffer.len;
	input_buffer.virAddr = stream_buffer.virAddr;
	ret = HB_ADEC_SendStream(adChn, &input_buffer, block);
	if (ret) {
		printf("%s Send ADEC stream error\n", TAG);
		return -1;
	}
	ret = HB_AENC_ReleaseStream(aeChn, &stream_buffer);
	if (ret) {
		printf("%s Release ADEC stream error\n", TAG);
		return -1;
	}

	ret = HB_ADEC_SendEndofStream(adChn, bClear);
	if (ret) {
		printf("%s Send EndofStream error\n", TAG);
		return -1;
	}
	return 0;
}

static void *audio_decoder_thread(void *arg) {
	int ret = 0;
	int m = 0;
	AudioRecorderTestContext *ctx = (AudioRecorderTestContext *)arg;
	if (!ctx) {
		printf("%s Invalid parameter\n", TAG);
		return NULL;
	}

	while (!ret && !ctx->decoder_thread_finished) {
		ret = read_audio_stream(ctx);
		if (ret) {
			ctx->error = 1;
			break;
		}

		m++;
		if (m >= ctx->time)
			break;
	}

	printf("%s Exit %s(ret=%d, stopping=%d, audio finished=%d)!\n",
		TAG, __func__, ret, ctx->terminate,
		ctx->decoder_thread_finished);
	return NULL;
}

static int startDecoder(AudioRecorderTestContext *pstCtx) {
	int ret = 0;
	int thread_ret = 0;
	int adChn = 1;
	struct HB_ADEC_CHN_ATTR_S *pstAdecAttr;
	struct HB_ADEC_PARAMS *pstAdecParam;
	if (!pstCtx || !pstCtx->decAttr) {
		printf("%s Invalid parameters\n", TAG);
		return -1;
	}

	pstAdecAttr = pstCtx->decAttr;
	pstAdecParam = &pstAdecAttr->adParams;
	struct ADEC_ATTR_G711A g711a;
	pstAdecAttr->type = PT_G711A;
	pstAdecParam->feed_mode = MC_FEEDING_MODE_FRAME_SIZE;
	pstAdecParam->packet_buf_size = 2048;
	pstAdecParam->frame_cache_size = 1;
	pstAdecParam->frame_buf_count = 5;
	pstAdecParam->packet_count = 5;
	g711a.channels = pstCtx->channel;
	g711a.sample_rate = pstCtx->sample_rate;
	pstAdecParam->stG711a = g711a;

	ret = HB_ADEC_CreateChn(adChn, pstAdecAttr);
	if (ret) {
		printf("%s Create ADEC chn error\n", TAG);
		ret = HB_ADEC_DestroyChn(adChn);
		if (ret) {
			printf("%s Destroy ADEC chn error\n", TAG);
			return -1;
		}
		return -1;
	}

	if ((thread_ret =
		pthread_create(&pstCtx->audio_decoder_thread, NULL,
			audio_decoder_thread, pstCtx)) != 0) {
		printf("%s Failed to pthread_create ret(%d)\n", TAG, thread_ret);
		ret = HB_ADEC_DestroyChn(adChn);
		if (ret) {
			printf("%s Destroy ADEC chn error\n", TAG);
			return -1;
		}
		return -1;
	}

	return 0;
}

static int stopPlayBack(AudioRecorderTestContext *pstCtx) {
	int ret = 0;
	void *status;
	int devId = 1;
	if (!pstCtx) {
		printf("%s Invalid parameters\n", TAG);
		return -1;
	}

	if (pstCtx->audio_playback_thread) {
		if ((ret = pthread_join(pstCtx->audio_playback_thread, &status)) == 0) {
			pstCtx->audio_playback_thread = 0;
		} else {
			printf("%s Failed to pthread_join ret(%d)\n", TAG, ret);
		}
	}

	ret = HB_AOT_Disable(devId);
	if (ret) {
		printf("%s Disable AOT pcm device error!\n", TAG);
		return -1;
	}

	return 0;
}

static int audio_decoder_init(AudioRecorderTestContext *pstCtx) {
	int ret = 0;
	int devId = 1;
	struct HB_AIO_DEVICE_ATTR_S *attr;
	if (!pstCtx || !pstCtx->aotAttr) {
		printf("%s Invalid AOT task\n", TAG);
		return -1;
	}
	attr = pstCtx->aotAttr;
	ret = HB_AOT_SetPubAttr(devId, attr);
	if (ret) {
		printf("%s Set AOT attr error\n", TAG);
		return -1;
	}

	ret = HB_AOT_Enable(devId);
	if (ret) {
		printf("%s Enable AOT pcm device error\n", TAG);
		return -1;
	}

	return 0;
}

static int read_pcm_data(AudioRecorderTestContext *pstCtx) {
	int ret = 0;
	int devId = 1;
	int adChn = 1;
	bool block = 0;
	if (!pstCtx || !pstCtx->aotAttr || !pstCtx->decAttr) {
		printf("%s Invalid playback task\n", TAG);
		return ret;
	}

	struct HB_AIO_FRAME_S frame;
	memset(&frame, 0x00, sizeof(struct HB_AIO_FRAME_S));
	struct HB_AUDIO_FRAME_S input_frame;
	memset(&input_frame, 0x00, sizeof(struct HB_AUDIO_FRAME_S));
	ret = HB_ADEC_GetFrame(adChn, &input_frame, block);
	if (ret) {
		printf("%s Get ADEC frame error\n", TAG);
		return -1;
	}

	frame.size = input_frame.len;
	frame.data = input_frame.virAddr;
	ret = HB_AOT_SendFrame(devId, 2, &frame, 10);
	if (ret) {
		printf("%s Send AOT frame error\n", TAG);
		return -1;
	}
	ret = HB_ADEC_ReleaseFrame(adChn, &input_frame);
	if (ret) {
		printf("%s Release AOT frame error\n", TAG);
		return -1;
	}
	return 0;
}

static void *audio_playback_thread(void *arg) {
	int ret = 0;
	int m = 0;
	AudioRecorderTestContext *ctx = (AudioRecorderTestContext *)arg;
	if (!ctx) {
		printf("%s Invalid parameter\n", TAG);
		return NULL;
	}

	while (!ret && !ctx->playback_thread_finished) {
		ret = read_pcm_data(ctx);
		if (ret) {
			ctx->error = 1;
			break;
		}

		m++;
                if (m >= ctx->time)
                        break;
	}
	printf("%s Exit %s(ret=%d, stopping=%d, audio finished=%d)\n",
		TAG, __func__, ret, ctx->terminate,
		ctx->playback_thread_finished);
	return NULL;
}

static int startPlayBack(AudioRecorderTestContext *pstCtx) {
	int ret = 0;
	int thread_ret = 0;

	if (!pstCtx) {
		printf("%s Invalid parameter\n", TAG);
		return -1;
	}

	// set aot pcm context
	struct HB_AIO_DEVICE_ATTR_S attr;
	memset(&attr, 0x00, sizeof(struct HB_AIO_DEVICE_ATTR_S));
	attr.sampleFmt = pstCtx->sample_fmt;
	attr.sampleRate = pstCtx->sample_rate;
	attr.channels = pstCtx->channel;
	attr.periodCount = 2;
	attr.periodSize = 1024;
	pstCtx->aotAttr = &attr;

	ret = audio_decoder_init(pstCtx);
	if (ret) {
		printf("%s Open pcm device error\n", TAG);
		return -1;
	}

	if ((thread_ret =
		pthread_create(&pstCtx->audio_playback_thread, NULL,
			audio_playback_thread, pstCtx)) != 0) {
		printf("%s Failed to pthread_create ret(%d)\n", TAG, thread_ret);
		return -1;
	}
	printf("%s Succes to start audio playback\n", TAG);
	return 0;
}

int main(int argc, char **argv) {
	int time = 10;
	int sample_rate = 16000;
	int sample_fmt = 1;  // 16bit
	int channel = 2;
	struct AudioRecorderTestContext ctx;
	struct HB_AIO_DEVICE_ATTR_S mAinAttr;
	struct HB_AIO_DEVICE_ATTR_S mAotAttr;
	struct HB_AENC_CHN_ATTR_S mAencAttr;
	struct HB_ADEC_CHN_ATTR_S mAdecAttr;
	memset(&ctx, 0x00, sizeof(struct AudioRecorderTestContext));
	memset(&mAinAttr, 0x00, sizeof(struct HB_AIO_DEVICE_ATTR_S));
	memset(&mAotAttr, 0x00, sizeof(struct HB_AIO_DEVICE_ATTR_S));
	memset(&mAencAttr, 0x00, sizeof(struct HB_AENC_CHN_ATTR_S));
	memset(&mAdecAttr, 0x00, sizeof(struct HB_ADEC_CHN_ATTR_S));

	if (argc < 1) {
		fprintf(stderr, "Usage: %s [-r rate] [-b format] [-c channel] [-t time]\n", argv[0]);
                return 1;
	}

	argv += 1;
	while (*argv) {
		if (strcmp(*argv, "-t") == 0) {
			argv++;
			if (*argv) {
				time = atoi(*argv);
			}
		}
		if (strcmp(*argv, "-r") == 0) {
			argv++;
			if (*argv) {
				sample_rate = atoi(*argv);
			}
		}
		if (strcmp(*argv, "-b") == 0) {
                        argv++;
                        if (*argv) {
                                sample_fmt = atoi(*argv);
                        }
                }
		if (strcmp(*argv, "-c") == 0) {
                        argv++;
                        if (*argv) {
                                channel = atoi(*argv);
                        }
                }
		if (*argv) {
			argv++;
		}
	}

	ctx.terminate = 0;
	ctx.error = 0;
	ctx.ainAttr = &mAinAttr;
	ctx.aotAttr = &mAotAttr;
	ctx.encAttr = &mAencAttr;
	ctx.decAttr = &mAdecAttr;
	ctx.time = time;
	ctx.sample_rate = sample_rate;
	ctx.sample_fmt = sample_fmt;
	ctx.channel = channel;

	if (startCapture(&ctx)) {
		printf("Failed to start audio capture\n");
		ctx.terminate = 1;
		ctx.error = 1;
	}
	if (startEncoder(&ctx)) {
		printf("Failed to start audio encoder\n");
                ctx.terminate = 1;
                ctx.error = 1;
	} else if (startDecoder(&ctx)) {
		printf("Failed to start audio decoder\n");
                ctx.terminate = 1;
                ctx.error = 1;
	} else if (startPlayBack(&ctx)) {
		printf("Failed to start audio playback\n");
                ctx.terminate = 1;
                ctx.error = 1;
	}

	while (ctx.terminate != 1) {
		sleep(5);
		ctx.terminate = 1;
	}

	if (stopPlayBack(&ctx)) {
		printf("Failed to stop audio playback\n");
		ctx.terminate = 1;
		ctx.error = 1;
	} else if (stopDecoder(&ctx)) {
		printf("Failed to stop audio decoder\n");
		ctx.terminate = 1;
		ctx.error = 1;
	} else if (stopEncoder(&ctx)) {
		printf("Failed to stop audio encoder\n");
		ctx.terminate = 1;
		ctx.error = 1;
	} else if (stopCapture(&ctx)) {
		printf("Failed to stop audio capture\n");
		ctx.terminate = 1;
		ctx.error = 1;
	}

	return 0;
}
