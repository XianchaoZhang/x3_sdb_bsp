#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/syscall.h>

#include "com_api.h"
#include "common.h"
#include "hb_media_error.h"
#include "hb_media_recorder.h"
#include "hb_vio_interface.h"
#include "vio_func.h"
#include "common.h"
#include "code_func.h"
#include "server_core.h"

#define VIDEO_BUF_NUM 5

typedef struct MRCameraBuffer {
	hb_vio_buffer_t buf;
	hb_bool valid;
	hb_s32 src_idx;
} MRCameraBuffer;

typedef enum ENC_CONFIG_MSG {
	ENC_CONFIG_H264_CBR = 0,
	ENC_CONFIG_H264_VBR = 1,
	ENC_CONFIG_H264_AVBR = 2,
	ENC_CONFIG_H264_FIX_QP = 3,
	ENC_CONFIG_H264_QP_MAP = 4,
	ENC_CONFIG_H265_CBR = 5,
	ENC_CONFIG_H265_VBR = 6,
	ENC_CONFIG_H265_AVBR = 7,
	ENC_CONFIG_H265_FIX_QP = 8,
	ENC_CONFIG_H265_QP_MAP = 9,
	ENC_CONFIG_MJPEG_FIX_QP = 10,
	ENC_CONFIG_H264_ENC_CONFIG = 11,
	ENC_CONFIG_H265_ENC_CONFIG = 12,
	ENC_CONFIG_VIDEO_GOP = 13,
	ENC_CONFIG_VIDEO_CODEC_ENC = 14,
	ENC_CONFIG_LONGTERM_REF = 15,
	ENC_CONFIG_INTRA_REFRESH = 16,
	ENC_CONFIG_H264_DEBLK_FILTER = 17,
	ENC_CONFIG_H265_DEBLK_FILTER = 18,
	ENC_CONFIG_DEBLK_FILTER = 19,
	ENC_CONFIG_H265_SAO = 20,
	ENC_CONFIG_H264_ENTROPY = 21,
	ENC_CONFIG_H264_TIMING = 22,
	ENC_CONFIG_H265_TIMING = 23,
	ENC_CONFIG_VUI_TIMING = 24,
	ENC_CONFIG_H264_SLICE = 25,
	ENC_CONFIG_H265_SLICE = 26,
	ENC_CONFIG_SLICE = 27,
	ENC_CONFIG_SMART_BG = 28,
	ENC_CONFIG_MONOCHROMA = 29,
	ENC_CONFIG_H264_INTRA_PRED = 30,
	ENC_CONFIG_H265_PRED_UINT = 31,
	ENC_CONFIG_PRED_UNIT = 32,
	ENC_CONFIG_H264_TRANSFORM = 33,
	ENC_CONFIG_H265_TRANSFORM = 34,
	ENC_CONFIG_TRANSFORM = 35,
	ENC_CONFIG_ROI = 36,
	ENC_CONFIG_MODE_DECISION = 37,
	ENC_CONFIG_TOTAL = 38,
} ENC_CONFIG_MSG;

typedef enum ENC_DYNAMIC_CONFIG_MSG {
	ENC_DYNAMIC_CONFIG_H264_CBR = (0 << 0),
	ENC_DYNAMIC_CONFIG_H264_VBR = (1 << 0),
	ENC_DYNAMIC_CONFIG_H264_AVBR = (2 << 0),
	ENC_DYNAMIC_CONFIG_H264_FIX_QP = (3 << 0),
	ENC_DYNAMIC_CONFIG_H264_QP_MAP = (4 << 0),
	ENC_DYNAMIC_CONFIG_H265_CBR = (5 << 0),
	ENC_DYNAMIC_CONFIG_H265_VBR = (6 << 0),
	ENC_DYNAMIC_CONFIG_H265_AVBR = (7 << 0),
	ENC_DYNAMIC_CONFIG_H265_FIX_QP = (8 << 0),
	ENC_DYNAMIC_CONFIG_H265_QP_MAP = (9 << 0),
	ENC_DYNAMIC_CONFIG_MJPEG_FIX_QP = (10 << 0),
	ENC_DYNAMIC_CONFIG_VIDEO_GOP = (13 << 0),
	ENC_DYNAMIC_CONFIG_LONGTERM_REF = (15 << 0),
	ENC_DYNAMIC_CONFIG_INTRA_REFRESH = (16 << 0),
	ENC_DYNAMIC_CONFIG_H264_DEBLK_FILTER = (17 << 0),
	ENC_DYNAMIC_CONFIG_H265_DEBLK_FILTER = (18 << 0),
	ENC_DYNAMIC_CONFIG_H265_SAO = (20 << 0),
	ENC_DYNAMIC_CONFIG_H264_ENTROPY = (21 << 0),
	ENC_DYNAMIC_CONFIG_H264_TIMING = (22 << 0),
	ENC_DYNAMIC_CONFIG_H265_TIMING = (23 << 0),
	ENC_DYNAMIC_CONFIG_H264_SLICE = (25 << 0),
	ENC_DYNAMIC_CONFIG_H265_SLICE = (26 << 0),
	ENC_DYNAMIC_CONFIG_SMART_BG = (28 << 0),
	ENC_DYNAMIC_CONFIG_H264_INTRA_PRED = (30 << 0),
	ENC_DYNAMIC_CONFIG_H265_PRED_UINT = (31 << 0),
	ENC_DYNAMIC_CONFIG_H264_TRANSFORM = (33 << 0),
	ENC_DYNAMIC_CONFIG_H265_TRANSFORM = (34 << 0),
	ENC_DYNAMIC_CONFIG_ROI = (36 << 0),
	ENC_DYNAMIC_CONFIG_MODE_DECISION = (37 << 0),
	ENC_DYNAMIC_CONFIG_TOTAL,
} ENC_DYNAMIC_CONFIG_MSG;

typedef struct MediaCodecContext {
	mc_h264_cbr_params_t h264_cbr;
	mc_h264_vbr_params_t h264_vbr;
	mc_h264_avbr_params_t h264_avbr;
	mc_h264_fix_qp_params_t h264_fixqp;
	mc_h264_qp_map_params_t h264_qpmap;

	mc_h265_cbr_params_t h265_cbr;
	mc_h265_vbr_params_t h265_vbr;
	mc_h265_avbr_params_t h265_avbr;
	mc_h265_fix_qp_params_t h265_fixqp;
	mc_h265_qp_map_params_t h265_qpmap;

	mc_h264_enc_config_t h264_encconfig;
	mc_h265_enc_config_t h265_encconfig;

	mc_video_gop_params_t gop_params;
	mc_video_codec_enc_params_t enc_params;
	mc_video_longterm_ref_mode_t ref_params;
	mc_video_intra_refresh_params_t refresh_params;
	mc_video_deblk_filter_params_t deblk;
	mc_h264_deblk_filter_params_t h264_deblk;
	mc_h265_deblk_filter_params_t h265_deblk;
	mc_h265_sao_params_t h265_sao;
	mc_h264_entropy_params_t h264_entropy;
	mc_video_vui_timing_params_t timing;
	mc_h264_timing_params_t h264_timing;
	mc_h265_timing_params_t h265_timing;
	mc_video_slice_params_t slice;
	mc_h264_slice_params_t h264_slice;
	mc_h265_slice_params_t h265_slice;
	mc_video_smart_bg_enc_params_t smart_params;
	mc_video_pred_unit_params_t pred;
	mc_h264_intra_pred_params_t h264_pred;
	mc_h265_pred_unit_params_t h265_pred;
	mc_video_transform_params_t transform;
	mc_h264_transform_params_t h264_transform;
	mc_h265_transform_params_t h265_transform;
	mc_video_roi_params_t roi;
	mc_video_mode_decision_params_t decision;
	pthread_mutex_t video_parameter_thread;
}MediaCodecContext;

typedef struct MediaRecorderTestContext_s {
	media_codec_context_t *encCtx;
	uint32_t encode_mode;
	uint32_t video_width;
	uint32_t video_height;
	uint32_t bit_rate;
	uint32_t intra_period;
	struct param_buf video_cfg;
	//global data, define by lf.s
	mc_video_longterm_ref_mode_t *ref_params;
	mc_video_intra_refresh_params_t *refresh_params;
	mc_video_smart_bg_enc_params_t *smart_params;
	mc_video_roi_params_t *roi_params;
	mc_video_codec_enc_params_t *enc_params;
	mc_video_deblk_filter_params_t *filter_params;
	mc_h264_entropy_params_t *entropy_params;
	mc_video_vui_timing_params_t *vui_params;
	mc_video_slice_params_t *slice_params;
	mc_video_pred_unit_params_t *pred_params;
	mc_video_transform_params_t *transform_params;
	mc_video_mode_decision_params_t *decision_params;
	//----------------------------------------------
	media_codec_context_t *jepgCtx;
	uint32_t jepg_skip;
	int video_terminate;
	int video_running;
	int jepg_terminate;
	uint32_t fps;
	int pipeline;
	int channel_id;
	MRCameraBuffer *camBuf;
	pthread_t video_encoder_putbuf_thread;
	pthread_t jepg_encoder_putbuf_thread;
	pthread_mutex_t cam_buf_lock;
	int video_put_thread_finished;
	int jepg_put_thread_finished;
	pthread_t video_encoder_getbuf_thread;
	pthread_t jepg_encoder_getbuf_thread;
	int video_get_thread_finished;
	int jepg_get_thread_finished;
} MediaRecorderTestContext;

media_codec_context_t mEncCtx;
media_codec_context_t mJpegCtx;
MediaRecorderTestContext mMRTestCtx;
MediaCodecContext mMCtx;

/*bufer - list*/
struct buf_list jepgdata_buf;
struct buf_list videodata_buf;

int set_dynamic_message(struct param_buf *video_cfg)
{
	int high, low;
	int i, temp;
	ENC_DYNAMIC_CONFIG_MSG message;

	if (!video_cfg) {
		printf("invalid parameter\n");
		return -1;
	}

	MediaRecorderTestContext *mr_ctx = &mMRTestCtx;

	mc_rate_control_params_t *rc_params = &mMRTestCtx.enc_params->rc_params;

	pthread_mutex_lock(&(mMCtx.video_parameter_thread));
	high = (video_cfg->param_id & 0xff00) >> 8;
	low = video_cfg->param_id & 0x00ff;
	switch (high) {
		case ENC_CONFIG_H264_CBR: {
			int *p1 = (int *)(&rc_params->h264_cbr_params);
			*(p1 + low) = video_cfg->param_data;
			rc_params->mode = MC_AV_RC_MODE_H264CBR;
			break;
		}
		case ENC_CONFIG_H264_VBR: {
			int *p2 = (int *)(&rc_params->h264_vbr_params);
			*(p2 + low) = video_cfg->param_data;
			rc_params->mode = MC_AV_RC_MODE_H264VBR;
			break;
		}
		case ENC_CONFIG_H264_AVBR: {
			int *p3 = (int *)(&rc_params->h264_avbr_params);
			*(p3 + low) = video_cfg->param_data;
			rc_params->mode = MC_AV_RC_MODE_H264AVBR;
			break;
		}
		case ENC_CONFIG_H264_FIX_QP: {
			int *p4 = (int *)(&rc_params->h264_fixqp_params);
			*(p4 + low) = video_cfg->param_data;
			rc_params->mode = MC_AV_RC_MODE_H264FIXQP;
			break;
		}
		case ENC_CONFIG_H264_QP_MAP: {
			int *p5 = (int *)(&rc_params->h264_qpmap_params);
			*(p5 + low) = video_cfg->param_data;
			rc_params->mode = MC_AV_RC_MODE_H264QPMAP;
			break;
		}
		case ENC_CONFIG_H265_CBR: {
			int *p6 = (int *)(&rc_params->h265_cbr_params);
			*(p6 + low) = video_cfg->param_data;
			rc_params->mode = MC_AV_RC_MODE_H265CBR;
			break;
		}
		case ENC_CONFIG_H265_VBR: {
			int *p7 = (int *)(&rc_params->h265_vbr_params);
			*(p7 + low) = video_cfg->param_data;
			rc_params->mode = MC_AV_RC_MODE_H265VBR;
			break;
		}
		case ENC_CONFIG_H265_AVBR: {
			int *p8 = (int *)(&rc_params->h265_avbr_params);
			*(p8 + low) = video_cfg->param_data;
			rc_params->mode = MC_AV_RC_MODE_H265AVBR;
			break;
		}
		case ENC_CONFIG_H265_FIX_QP: {
			int *p9 = (int *)(&rc_params->h265_fixqp_params);
			*(p9 + low) = video_cfg->param_data;
			rc_params->mode = MC_AV_RC_MODE_H265FIXQP;
			break;
		}
		case ENC_CONFIG_H265_QP_MAP: {
			int *p10 = (int *)(&rc_params->h265_qpmap_params);
			*(p10 + low) = video_cfg->param_data;
			rc_params->mode = MC_AV_RC_MODE_H265QPMAP;
			break;
		}
		case ENC_CONFIG_MJPEG_FIX_QP:
			break;
		case ENC_CONFIG_H264_ENC_CONFIG: {
			mc_h264_profile_t *p12 = (int *)(&mMCtx.h264_encconfig);
			*(p12 + low) = video_cfg->param_data;
			break;
		}
		case ENC_CONFIG_H265_ENC_CONFIG:
			break;
		case ENC_CONFIG_VIDEO_CODEC_ENC: {
			int *p15 = (int *)(&mMCtx.enc_params);
			*(p15 + low) = video_cfg->param_data;
			break;
		}
		case ENC_CONFIG_LONGTERM_REF: {
			int *p16 = (int *)(&mMCtx.ref_params);
			*(p16 + low) = video_cfg->param_data;
			break;
		}
		case ENC_CONFIG_INTRA_REFRESH: {
			//int *p17 = &mMCtx.refresh_params.intra_refresh_mode;
			//*(p17 + low) = video_cfg->param_data;
			break;
		}
		case ENC_CONFIG_H264_DEBLK_FILTER: {
			int *p18 = (int *)(&mMCtx.h264_deblk);
			*(p18 + low) = video_cfg->param_data;
			break;
		}
		case ENC_CONFIG_H265_DEBLK_FILTER: {
			int *p19 = (int *)(&mMCtx.h265_deblk);
			*(p19 + low) = video_cfg->param_data;
			break;
		}
		case ENC_CONFIG_DEBLK_FILTER: {
			mc_h264_deblk_filter_params_t *p20 = (int *)(&mMCtx.deblk);
			//*(p20 + low) = (mc_h264_deblk_filter_params_t *)video_cfg->param_data;
			break;
		}
		case ENC_CONFIG_H265_SAO: {
			mMCtx.h265_sao.sample_adaptive_offset_enabled_flag = video_cfg->param_data;
			break;
		}
		case ENC_CONFIG_H264_ENTROPY: {
			mMCtx.h264_entropy.entropy_coding_mode = video_cfg->param_data;
			break;
		}
		case ENC_CONFIG_H264_TIMING: {
			int *p23 = (int *)(&mMCtx.h264_timing);
			*(p23 + low) = video_cfg->param_data;
			break;
		}
		case ENC_CONFIG_H265_TIMING: {
			int *p24 = (int *)(&mMCtx.h265_timing);
			*(p24 + low) = video_cfg->param_data;
			break;
		}
		case ENC_CONFIG_VUI_TIMING: {
			mc_h264_timing_params_t *p25 = (int *)(&mMCtx.timing.h264_timing);
			//*(p25 + low) = (mc_h264_timing_params_t)video_cfg->param_data;
			break;
		}
		case ENC_CONFIG_H264_SLICE: {
			int *p26 = (int *)(&mMCtx.h264_slice.h264_slice_mode);
			*(p26 + low) = video_cfg->param_data;
			break;
		}
		case ENC_CONFIG_H265_SLICE: {
			int *p27 = (int *)(&mMCtx.h265_slice.h265_independent_slice_mode);
			*(p27 + low) = video_cfg->param_data;
			break;
		}
		case ENC_CONFIG_SLICE: {
			mc_h264_slice_params_t *p28 = (int *)(&mMCtx.slice.h264_slice);
			//*(p28 + low) = (mc_h264_slice_params_t)video_cfg->param_data;
			break;
		}
		case ENC_CONFIG_SMART_BG: {
			int *p29 = (int *)(&mMCtx.smart_params);
			*(p29 + low) = video_cfg->param_data;
			break;
		}
		case ENC_CONFIG_MONOCHROMA:
			break;
		case ENC_CONFIG_H264_INTRA_PRED: {
			mMCtx.h264_pred.constrained_intra_pred_flag = video_cfg->param_data;
			break;
		}
		case ENC_CONFIG_H265_PRED_UINT: {
			int *p32 = (int *)(&mMCtx.h265_pred);
			*(p32 + low) = video_cfg->param_data;
			break;
		}
		case ENC_CONFIG_PRED_UNIT: {
			mc_h264_intra_pred_params_t *p33 = (int *)(&mMCtx.pred);
			//*(p33 + low) = (mc_h264_intra_pred_params_t)video_cfg->param_data;
			break;
		}
		case ENC_CONFIG_H264_TRANSFORM: {
			int *p34 = (int *)(&mMCtx.h264_transform);
			*(p34 + low) = video_cfg->param_data;
			break;
		}
		case ENC_CONFIG_H265_TRANSFORM: {
			int *p35 = (int *)(&mMCtx.h265_transform);
			*(p35 + low) = video_cfg->param_data;
			break;
		}
		case ENC_CONFIG_TRANSFORM: {
			mc_h264_transform_params_t *p36 = (int *)(&mMCtx.transform);
			//*(p36 + low) = (mc_h264_transform_params_t)video_cfg->param_data;
			break;
		}
		case ENC_CONFIG_ROI: {
			int *p37 = (int *)(&mMCtx.roi);
			*(p37 + low) = video_cfg->param_data;
			break;
		}
		case ENC_CONFIG_MODE_DECISION: {
			int *p38 = (int *)(&mMCtx.decision);
			*(p38 + low) = video_cfg->param_data;
			break;
		}
		default:
			printf("invalid high(%d)\n", high);
			pthread_mutex_unlock(&(mMCtx.video_parameter_thread));
			return -1;
	}

	if (high >= ENC_CONFIG_H264_CBR || high <= ENC_CONFIG_H265_QP_MAP) {
		vmon_dbg("intra_period=%d\n", rc_params->h264_cbr_params.intra_period);
		vmon_dbg("intra_qp=%d\n", rc_params->h264_cbr_params.intra_qp);
		vmon_dbg("bit_rate=%d\n", rc_params->h264_cbr_params.bit_rate);
		vmon_dbg("frame_rate=%d\n", rc_params->h264_cbr_params.frame_rate);
		vmon_dbg("initial_rc_qp=%d\n", rc_params->h264_cbr_params.initial_rc_qp);
		vmon_dbg("vbv_buffer_size=%d\n", rc_params->h264_cbr_params.vbv_buffer_size);
		vmon_dbg("mb_level_rc_enable=%d\n", rc_params->h264_cbr_params.mb_level_rc_enalbe);
		vmon_dbg("min_qp_I=%d\n", rc_params->h264_cbr_params.min_qp_I);
		vmon_dbg("max_qp_I=%d\n", rc_params->h264_cbr_params.max_qp_I);
		vmon_dbg("min_qp_P=%d\n", rc_params->h264_cbr_params.min_qp_P);
		vmon_dbg("max_qp_P=%d\n", rc_params->h264_cbr_params.max_qp_P);
		vmon_dbg("min_qp_B=%d\n", rc_params->h264_cbr_params.min_qp_B);
                vmon_dbg("max_qp_B=%d\n", rc_params->h264_cbr_params.max_qp_B);
		vmon_dbg("hvs_qp_enable=%d\n", rc_params->h264_cbr_params.hvs_qp_enable);
		vmon_dbg("hvs_qp_scale=%d\n", rc_params->h264_cbr_params.hvs_qp_scale);
		vmon_dbg("max_delta_qp=%d\n", rc_params->h264_cbr_params.max_delta_qp);
		vmon_dbg("qp_map_enable=%d\n", rc_params->h264_cbr_params.qp_map_enable);

		hb_mm_mc_set_rate_control_config(mr_ctx->encCtx, rc_params);
	}

	switch(high) {
		case ENC_CONFIG_H264_DEBLK_FILTER:
			mMCtx.deblk.h264_deblk = mMCtx.h264_deblk;
			hb_mm_mc_set_deblk_filter_config(mr_ctx->encCtx, &mMCtx.deblk);
			break;
		case ENC_CONFIG_H265_DEBLK_FILTER:
			mMCtx.deblk.h265_deblk = mMCtx.h265_deblk;
			hb_mm_mc_set_deblk_filter_config(mr_ctx->encCtx, &mMCtx.deblk);
			break;
		case ENC_CONFIG_H264_ENTROPY:
			hb_mm_mc_set_entropy_config(mr_ctx->encCtx, &mMCtx.h264_entropy);
			break;
		case ENC_CONFIG_H264_SLICE:
			mMCtx.slice.h264_slice = mMCtx.h264_slice;
			hb_mm_mc_set_slice_config(mr_ctx->encCtx, &mMCtx.slice);
			break;
		case ENC_CONFIG_H265_SLICE:
			mMCtx.slice.h265_slice = mMCtx.h265_slice;
			hb_mm_mc_set_slice_config(mr_ctx->encCtx, &mMCtx.slice);
			break;
		case ENC_CONFIG_SMART_BG:
			hb_mm_mc_set_smart_bg_enc_config(mr_ctx->encCtx, &mMCtx.smart_params);
			break;
		case ENC_CONFIG_H264_INTRA_PRED:
			mMCtx.pred.h264_intra_pred = mMCtx.h264_pred;
			hb_mm_mc_set_pred_unit_config(mr_ctx->encCtx, &mMCtx.pred);
			break;
		case ENC_CONFIG_H265_PRED_UINT:
			mMCtx.pred.h265_pred_unit = mMCtx.h265_pred;
			hb_mm_mc_set_pred_unit_config(mr_ctx->encCtx, &mMCtx.pred);
			break;
		case ENC_CONFIG_H264_TRANSFORM:
			mMCtx.transform.h264_transform = mMCtx.h264_transform;
			hb_mm_mc_set_transform_config(mr_ctx->encCtx, &mMCtx.transform);
			break;
		case ENC_CONFIG_H265_TRANSFORM:
			mMCtx.transform.h265_transform = mMCtx.h265_transform;
			hb_mm_mc_set_transform_config(mr_ctx->encCtx, &mMCtx.transform);
			break;
		case ENC_CONFIG_ROI:
			hb_mm_mc_set_roi_config(mr_ctx->encCtx, &mMCtx.roi);
			break;
		case ENC_CONFIG_MODE_DECISION:
			hb_mm_mc_set_mode_decision_config(mr_ctx->encCtx, &mMCtx.decision);
			break;
		default:
			vmon_dbg("invalid high(%d) value\n", high);
			break;
	}
	pthread_mutex_unlock(&(mMCtx.video_parameter_thread));
	return 0;
}


int get_jepg_buf(struct buf_info *data)
{
	int ret = 0;
	struct buf_list_node *node = NULL;

	if (buf_list_empty(&jepgdata_buf)) {
		//printf("[%s--%d]-------buf is empty---------\n", __func__, __LINE__);
		return -1;
	} else {
		ret = buf_list_remove_node(&jepgdata_buf, &node);
		if (ret < 0) {
			//printf("[%s--%d]-------get buff fialed---------\n", __func__, __LINE__);
		} else {
			memcpy(data, &node->buf, sizeof(struct buf_info));
			free(node);
			node = NULL;
		}
	}

	return ret;
}


int get_video_buf(struct buf_info *data)
{
	int ret = 0;
	struct buf_list_node *node = NULL;

	if (buf_list_empty(&videodata_buf)) {
		//vmon_err("-------buf is empty---------\n");
		return -1;
	} else {
		ret = buf_list_remove_node(&videodata_buf, &node);
		if (ret < 0) {
			vmon_err("-------get buff fialed---------\n");
		} else {
			memcpy(data, &node->buf, sizeof(struct buf_info));
			free(node);
			node = NULL;
		}
	}

	return ret;
}

static int set_video_list(media_codec_buffer_t *input,
	struct buf_list *list, media_codec_output_buffer_info_t *info)
{
	struct buf_info data;
	struct buf_list_node *node = NULL;
	uint32_t size = 0;
	int ret = -1;

	if (get_list_length(list) >= 20) {
		return ret;
	}

	memset(&data, 0, sizeof(struct buf_info));
	data.header_info.type = VIDEO_DATA;//yuv

	//data.header_info.format = H264;//
	if (mMRTestCtx.encCtx->codec_id == MEDIA_CODEC_ID_H264) {
		data.header_info.format = H264;//
	} else {
		data.header_info.format = H265;//
	}

	data.header_info.width = mMRTestCtx.encCtx->video_enc_params.width;
	data.header_info.height = mMRTestCtx.encCtx->video_enc_params.height;
	data.header_info.stride = mMRTestCtx.encCtx->video_enc_params.width;
	data.header_info.length = input->vstream_buf.size;
	data.header_info.frame_id = info->video_stream_info.frame_index;
	data.header_info.frame_plane = 0;
	data.header_info.code_type = 1;

	data.ptr[0] = malloc(data.header_info.length);
	if(data.ptr[0] == NULL) {
		vmon_err("malloc size failed!\n");
		ret = -1;
	} else {
		memcpy(data.ptr[0], input->vstream_buf.vir_ptr, input->vstream_buf.size);
	}

	node = malloc(sizeof(struct buf_list_node));
	if (node == NULL) {
		vmon_err("malloc size failed!\n");
		buf_free(&data);
	} else {
		memcpy(&node->buf, &data, sizeof(struct buf_info));
		ret = buf_list_add_node(list, &node);
		if (ret < 0) {
			buf_free(&node->buf);
			free(node);
		}
	}
	node = NULL;

	return ret;
}

static int set_jepg_list(media_codec_buffer_t *input,
	struct buf_list *list, media_codec_output_buffer_info_t *info)
{
	struct buf_info data;
	struct buf_list_node *node = NULL;
	uint32_t size = 0;
	int ret = -1;

	if (get_list_length(list) >= 3) {
		return ret;
	}

	memset(&data, 0, sizeof(struct buf_info));
	data.header_info.type = JEPG_DATA;//yuv

	data.header_info.width = mMRTestCtx.jepgCtx->video_enc_params.width;
	data.header_info.height = mMRTestCtx.jepgCtx->video_enc_params.height;
	data.header_info.stride = mMRTestCtx.jepgCtx->video_enc_params.width;
	data.header_info.length = input->vstream_buf.size;
	data.header_info.frame_id = info->video_stream_info.frame_index;
	data.header_info.frame_plane = 0;
	data.header_info.code_type = 1;

	data.ptr[0] = malloc(data.header_info.length);
	if(data.ptr[0] == NULL) {
		vmon_err("malloc size failed!\n");
		ret = -1;
	} else {
		memcpy(data.ptr[0], input->vstream_buf.vir_ptr, input->vstream_buf.size);
	}

	node = malloc(sizeof(struct buf_list_node));
	if (node == NULL) {
		vmon_err("malloc size failed!\n");
		buf_free(&data);
	} else {
		memcpy(&node->buf, &data, sizeof(struct buf_info));
		ret = buf_list_add_node(list, &node);
		if (ret < 0) {
			buf_free(&node->buf);
			free(node);
		}
	}
	node = NULL;

	return ret;
}

static hb_s32 read_video_frame(MediaRecorderTestContext *ctx) {
	hb_s32 ret = 0;
	int vio_ret = 0;
	uint32_t j, k = 0;
	media_codec_context_t *vcodec_ctx;
	if (!ctx || !ctx->encCtx) {
		vmon_err("Invalid argument!\n");
		return -1;
	}

	struct buf_info info;
	hb_vio_buffer_t data;

	ret = get_yuv_buf(&info);
	if (ret < 0) {
		vmon_err(" Failed to get vio data.\n");
		usleep(15*1000);
		return -1;
	} else {
		if ((info.header_info.width == mMRTestCtx.video_width) &&
			(info.header_info.height == mMRTestCtx.video_height)) {
			if (info.buf_valib == 1) {
				memcpy(&data, &info.buf, sizeof(hb_vio_buffer_t));
			} else {
				vmon_err("buf is memcpy, free!\n");
				free_yuv_buf(&info);
				return -1;
			}
		} else {
			vmon_err("size is error.\n");
			free_yuv_buf(&info);
			return -1;
		}
	}
#if 0
	vio_ret = hb_vio_get_data(ctx->pipeline, (VIO_DATA_TYPE_E)ctx->channel_id, &data);
	if (vio_ret < 0) {
		vmon_err(" Failed to get vio data.\n");
		return -1;
	}
#endif

	media_codec_buffer_t inputBuffer;
	memset(&inputBuffer, 0x00, sizeof(media_codec_buffer_t));

	vcodec_ctx = ctx->encCtx;

	ret = hb_mm_mc_dequeue_input_buffer(vcodec_ctx, &inputBuffer, 30);
	if (!ret) {
		// Free the encoded camera source buffer
		pthread_mutex_lock(&ctx->cam_buf_lock);
		for (k=0; k < vcodec_ctx->video_enc_params.frame_buf_count; k++) {
			if (ctx->camBuf[k].valid &&
				ctx->camBuf[k].src_idx == inputBuffer.vframe_buf.src_idx) {
				hb_vio_free_ipubuf(ctx->pipeline, &ctx->camBuf[k].buf);
				ctx->camBuf[k].valid = 0;
				ctx->camBuf[k].src_idx = -1;
				break;
			}
		}
		for (j=0; j < 3; j++) {
			inputBuffer.vframe_buf.vir_ptr[j] = (hb_u8 *)data.img_addr.addr[j];
			inputBuffer.vframe_buf.phy_ptr[j] = (hb_u32)data.img_addr.paddr[j];
			inputBuffer.vframe_buf.fd[j] = data.img_info.fd[j];
		}
		inputBuffer.vframe_buf.size = 0;
		for (j=0; j < data.img_info.planeCount; j++) {
			inputBuffer.vframe_buf.size += data.img_info.size[j];
		}
		// store the input camera buffer info
		for (k=0; k < vcodec_ctx->video_enc_params.frame_buf_count; k++) {
			if (!ctx->camBuf[k].valid) {
				ctx->camBuf[k].buf = data;
				ctx->camBuf[k].src_idx = inputBuffer.vframe_buf.src_idx;
				ctx->camBuf[k].valid = 1;
				break;
			}
		}
		pthread_mutex_unlock(&ctx->cam_buf_lock);
		if (ctx->video_terminate) {
			vmon_err(" There is no more input data!\n");
			inputBuffer.vframe_buf.frame_end = 1;
			ctx->video_put_thread_finished = 1;
		}

		ret = hb_mm_mc_queue_input_buffer(vcodec_ctx, &inputBuffer, -1);
		if (ret) {
			hb_vio_free_ipubuf(ctx->pipeline, &data);
			vmon_err("Failed to queue video input frame buffer: 0x%x.\n", ret);
		} else if (ret == (hb_s32)HB_MEDIA_ERR_WAIT_TIMEOUT) {
			vmon_err("video time out data!\n");
			hb_vio_free_ipubuf(ctx->pipeline, &data);
			ret = 0;
		} else {
			hb_vio_free_ipubuf(ctx->pipeline, &data);
			vmon_err("Failed to dequeue video input frame buffer: 0x%x.\n", ret);
		}
	} else { //none
		hb_vio_free_ipubuf(ctx->pipeline, &data);
	}

	return ret;
}

static hb_s32 read_jepg_frame(MediaRecorderTestContext *ctx) {
	hb_s32 ret = 0;
	int vio_ret = 0;
	uint32_t j, k = 0;
	media_codec_context_t *jepg_ctx;
	if (!ctx || !ctx->jepgCtx) {
		vmon_err("Invalid argument!\n");
		return -1;
	}

	hb_vio_buffer_t data;
	vio_ret = hb_vio_get_data(ctx->pipeline, (VIO_DATA_TYPE_E)ctx->channel_id, &data);
	if (vio_ret < 0) {
		vmon_err(" Failed to get vio data.\n");
		return -1;
	}

	ctx->jepg_skip++;
	if (ctx->jepg_skip % 5 != 0) {
		hb_vio_free_ipubuf(ctx->pipeline, &data);
		return -1;
	} else {
		ctx->jepg_skip = 0;
	}

	media_codec_buffer_t inputBuffer;
	memset(&inputBuffer, 0x00, sizeof(media_codec_buffer_t));

	jepg_ctx = ctx->jepgCtx;

	ret = hb_mm_mc_dequeue_input_buffer(jepg_ctx, &inputBuffer, 100);
	if (!ret) {
		uint32_t size = data.img_addr.stride_size * data.img_addr.height;
		memcpy(inputBuffer.vframe_buf.vir_ptr[0], data.img_addr.addr[0], size);
		memcpy(inputBuffer.vframe_buf.vir_ptr[0] + size, data.img_addr.addr[1], size/2);
		hb_vio_free_ipubuf(ctx->pipeline, &data);

		if (ctx->jepg_terminate) {
			vmon_err(" There is no more input data!\n");
			inputBuffer.vframe_buf.frame_end = 1;
			ctx->jepg_put_thread_finished = 1;
		}
		ret = hb_mm_mc_queue_input_buffer(jepg_ctx, &inputBuffer, -1);
		if (ret) {
			hb_vio_free_ipubuf(ctx->pipeline, &data);
			vmon_err(" Failed to queue video input frame buffer: 0x%x.\n", ret);
		} else if (ret == (hb_s32)HB_MEDIA_ERR_WAIT_TIMEOUT) {
			vmon_err(" video time out data!\n");
			hb_vio_free_ipubuf(ctx->pipeline, &data);
			ret = 0;
		} else {
			hb_vio_free_ipubuf(ctx->pipeline, &data);
			vmon_err("Failed to dequeue video input frame buffer: 0x%x.\n", ret);
		}
	} else { //none
		hb_vio_free_ipubuf(ctx->pipeline, &data);
	}

	return ret;
}

static void *video_encoder_putbuf_thread(void *arg) {
	hb_s32 ret = 0;
	MediaRecorderTestContext *ctx = (MediaRecorderTestContext *)arg;
	if (!ctx) {
		vmon_err("Invalid argument!\n");
		return NULL;
	}

	//while (!ret && !ctx->encoder_thread_finished) {
	while (!ctx->video_put_thread_finished) {
		print_timestamp();
		ret = read_video_frame(ctx);
		if (ret) {
			//break;
		}
		usleep(1000);
	}

	vmon_dbg(" finsh thread!!!\n");
	return NULL;
}

static void *jepg_encoder_putbuf_thread(void *arg) {
	hb_s32 ret = 0;
	MediaRecorderTestContext *ctx = (MediaRecorderTestContext *)arg;
	if (!ctx) {
		vmon_err("%s Invalid argument!\n");
		return NULL;
	}

	while (!ctx->jepg_put_thread_finished) {
		print_timestamp();
		ret = read_jepg_frame(ctx);
		if (ret) {
			//break;
		}
		usleep(1000);
	}

	vmon_dbg(" finsh thread!!!\n");
	return NULL;
}

static hb_s32 read_video_stream(MediaRecorderTestContext *ctx) {
	hb_s32 ret = 0, ret2 = 0;
	media_codec_context_t *encCtx;
	media_codec_context_t *jepg_ctx;
	int readDone = 0;
	if (!ctx || !ctx->encCtx) {
		vmon_err(" Failed to read video stream.\n");
		return -1;
	}
	encCtx = ctx->encCtx;
	jepg_ctx = ctx->jepgCtx;

	media_codec_buffer_t streamBuffer;
	media_codec_output_buffer_info_t video_info;
	memset(&video_info, 0x00, sizeof(media_codec_output_buffer_info_t));
	memset(&streamBuffer, 0x00, sizeof(media_codec_buffer_t));

	ret = hb_mm_mc_dequeue_output_buffer(encCtx,
		&streamBuffer, &video_info, 30);
	if (!ret) {
		//get buf success
		set_video_list(&streamBuffer, &videodata_buf, &video_info);

		ret = hb_mm_mc_queue_output_buffer(encCtx, &streamBuffer, -1);
		if (ret) {
			vmon_err(" Failed to queue video output stream: 0x%x.\n", ret);
		}

		pthread_mutex_lock(&ctx->cam_buf_lock);
		for (uint32_t k=0; k < encCtx->video_enc_params.frame_buf_count; k++) {
			if (ctx->camBuf[k].valid &&
				ctx->camBuf[k].src_idx == streamBuffer.vstream_buf.src_idx) {
				hb_vio_free_ipubuf(ctx->pipeline, &(ctx->camBuf[k].buf));
				ctx->camBuf[k].valid = 0;
				ctx->camBuf[k].src_idx = -1;
				break;
			}
		}
		pthread_mutex_unlock(&ctx->cam_buf_lock);
		if (ctx->video_terminate || streamBuffer.vstream_buf.stream_end) {
			vmon_err(" There is no more input data!\n");
			ctx->video_get_thread_finished = 1;
		}
	} else if (ret == (hb_s32)HB_MEDIA_ERR_WAIT_TIMEOUT) {
		ret = 0;
	} else {
		vmon_err(" Failed to dequeue video output stream buffer: %d.\n", ret);
	}


	if (ctx->video_terminate) {
		ctx->video_get_thread_finished = 1;
	}

	return ret ? ret : ret2;
}

static hb_s32 read_jepg_stream(MediaRecorderTestContext *ctx) {
	hb_s32 ret = 0, ret2 = 0;
	media_codec_context_t *jepg_ctx;
	int readDone = 0;
	if (!ctx || !ctx->jepgCtx) {
		vmon_err(" Failed to read video stream.\n");
		return -1;
	}
	jepg_ctx = ctx->jepgCtx;

	media_codec_buffer_t streamBuffer;
	media_codec_output_buffer_info_t video_info;
	memset(&video_info, 0x00, sizeof(media_codec_output_buffer_info_t));
	memset(&streamBuffer, 0x00, sizeof(media_codec_buffer_t));

	ret = hb_mm_mc_dequeue_output_buffer(jepg_ctx,
		&streamBuffer, &video_info, 30);
	if (!ret) {
		//get buf success
		set_jepg_list(&streamBuffer, &jepgdata_buf, &video_info);

		ret = hb_mm_mc_queue_output_buffer(jepg_ctx, &streamBuffer, -1);
		if (ret) {
			vmon_err(" Failed to queue video output stream: 0x%x.\n", ret);
		}

		if (ctx->jepg_terminate || streamBuffer.vstream_buf.stream_end) {
			vmon_err(" There is no more input data!\n");
			ctx->jepg_get_thread_finished = 1;
		}
	} else if (ret == (hb_s32)HB_MEDIA_ERR_WAIT_TIMEOUT) {
		ret = 0;
	} else {
		vmon_err(" Failed to dequeue video output stream buffer: %d.\n", ret);
	}

	if (ctx->jepg_terminate) {
		ctx->jepg_get_thread_finished = 1;
	}

	return ret ? ret : ret2;
}

static void *video_encoder_getbuf_thread(void *arg) {
	hb_s32 ret = 0;
	MediaRecorderTestContext *ctx = (MediaRecorderTestContext *)arg;
	if (!ctx) {
		vmon_err(" Invalid argument!\n");
		return NULL;
	}

	while (!ctx->video_get_thread_finished) {
		print_timestamp();
		ret = set_dynamic_message(&ctx->video_cfg);
		if (ret < 0) {
			printf("set dynamic message failed!\n");
			return NULL;
		}
		ret = read_video_stream(ctx);
		if (ret) {
			//break;
		}
		usleep(1000);
	}
	vmon_dbg(" finsh thread!!!\n");

	return NULL;
}

static void *jepg_encoder_getbuf_thread(void *arg) {
	hb_s32 ret = 0;
	MediaRecorderTestContext *ctx = (MediaRecorderTestContext *)arg;
	if (!ctx) {
		vmon_err(" Invalid argument!\n");
		return NULL;
	}

	while (!ctx->jepg_get_thread_finished) {
		print_timestamp();
		ret = read_jepg_stream(ctx);
		if (ret) {
			//break;
		}
		usleep(1000);
	}

	vmon_dbg(" finsh thread!!!\n");
	return NULL;
}

static void send_videoinfo(struct cmd_header *info, struct send_buf *buf)
{
        char *out = NULL;
        uint32_t count = 0;
        int ret = 0;

        acquire_mutex();
        ret = send_data(info, sizeof(struct cmd_header));
        if (ret < 0) {
                goto err;
        }

        if (buf != NULL) {
                ret = send_data(buf, sizeof(buf->param_num));
                if (ret < 0) {
                        goto err;
                }
                ret = send_data(buf->param, sizeof(struct param_buf)*buf->param_num);
                if (ret < 0) {
                        goto err;
                }
        }

        release_mutex();
        return;
err:
        vmon_dbg("send_data error. \n");
        err_handler(0);
        release_mutex();
}

static int get_send_video_default_info(void)
{
	int ret = 0;
	uint32_t frameRate;
	media_codec_context_t *vcodec_ctx;

	MediaRecorderTestContext *mr_ctx = &mMRTestCtx;

	struct cmd_header header_info;
	struct send_buf send_buf;
	int i = 0;
	int temp = 0;
	int array[256] = {0};

	if (!mr_ctx || !mr_ctx->encCtx) {
		printf("%s Invalid parameters.\n", __func__);
		return -1;
	}

	memset(&header_info, 0, sizeof(header_info));
	memset(&send_buf, 0, sizeof(send_buf));
	send_buf.param_num = MAX_PARAM_BUF_SIZE;
	send_buf.param = (struct param_buf *)malloc(send_buf.param_num * sizeof(struct param_buf));
	if (!send_buf.param) {
		printf("%s Malloc send_buf.param failed\n", __func__);
		return -1;
	}

	pthread_mutex_lock(&(mMCtx.video_parameter_thread));
	memset(&mMCtx, 0x00, sizeof(MediaCodecContext));
	vcodec_ctx = mr_ctx->encCtx;
	mr_ctx->enc_params = &vcodec_ctx->video_enc_params;

	vcodec_ctx->encoder = 1;
	vcodec_ctx->codec_id = mr_ctx->encode_mode;
	ret = hb_mm_mc_get_default_context(vcodec_ctx->codec_id, vcodec_ctx->encoder, vcodec_ctx);
	if (ret) {
		printf("%s Get default context failed\n", __func__);
	}

	vcodec_ctx->video_enc_params.rc_params.mode = ENC_CONFIG_H264_CBR;
	ret = hb_mm_mc_get_rate_control_config(vcodec_ctx, &vcodec_ctx->video_enc_params.rc_params);
	vcodec_ctx->video_enc_params.rc_params.h264_cbr_params.bit_rate =
                mMRTestCtx.bit_rate;
	vcodec_ctx->video_enc_params.rc_params.h264_cbr_params.intra_period =
		mMRTestCtx.intra_period;
	memcpy(array, &vcodec_ctx->video_enc_params.rc_params.h264_cbr_params,
		sizeof(vcodec_ctx->video_enc_params.rc_params.h264_cbr_params));
	for (temp = 0; temp < sizeof(mMCtx.h264_cbr)/sizeof(uint32_t); temp++) {
		send_buf.param[i].param_id = ENC_CONFIG_H264_CBR;
		send_buf.param[i].param_id = (ENC_CONFIG_H264_CBR << 8) | temp;
		send_buf.param[i].param_data = array[temp];
		i++;
	}

	vcodec_ctx->video_enc_params.rc_params.mode = ENC_CONFIG_H264_VBR;
	ret = hb_mm_mc_get_rate_control_config(vcodec_ctx, &vcodec_ctx->video_enc_params.rc_params);
	memcpy(array, &vcodec_ctx->video_enc_params.rc_params.h264_vbr_params,
		sizeof(vcodec_ctx->video_enc_params.rc_params.h264_vbr_params));
	for (temp = 0; temp < sizeof(mMCtx.h264_vbr)/sizeof(uint32_t); temp++) {
		send_buf.param[i].param_id = ENC_CONFIG_H264_VBR;
		send_buf.param[i].param_id = (ENC_CONFIG_H264_VBR << 8) | temp;
		send_buf.param[i].param_data = array[temp];
		i++;
	}

	vcodec_ctx->video_enc_params.rc_params.mode = ENC_CONFIG_H264_AVBR;
	ret = hb_mm_mc_get_rate_control_config(vcodec_ctx, &vcodec_ctx->video_enc_params.rc_params);
	memcpy(array, &vcodec_ctx->video_enc_params.rc_params.h264_avbr_params,
		sizeof(vcodec_ctx->video_enc_params.rc_params.h264_avbr_params));
	for (temp = 0; temp < sizeof(mMCtx.h264_avbr)/sizeof(uint32_t); temp++) {
		send_buf.param[i].param_id = ENC_CONFIG_H264_AVBR;
		send_buf.param[i].param_id = (ENC_CONFIG_H264_AVBR << 8) | temp;
		send_buf.param[i].param_data = array[temp];
		i++;
	}

	vcodec_ctx->video_enc_params.rc_params.mode = ENC_CONFIG_H264_FIX_QP;
	ret = hb_mm_mc_get_rate_control_config(vcodec_ctx, &vcodec_ctx->video_enc_params.rc_params);
	memcpy(array, &vcodec_ctx->video_enc_params.rc_params.h264_fixqp_params,
		sizeof(vcodec_ctx->video_enc_params.rc_params.h264_fixqp_params));
	for (temp = 0; temp < sizeof(mMCtx.h264_fixqp)/sizeof(uint32_t); temp++) {
		send_buf.param[i].param_id = ENC_CONFIG_H264_FIX_QP;
		send_buf.param[i].param_id = (ENC_CONFIG_H264_FIX_QP << 8) | temp;
		send_buf.param[i].param_data = array[temp];
		i++;
	}

	vcodec_ctx->video_enc_params.rc_params.mode = ENC_CONFIG_H264_QP_MAP;
	ret = hb_mm_mc_get_rate_control_config(vcodec_ctx, &vcodec_ctx->video_enc_params.rc_params);
	temp = 0;
	send_buf.param[i].param_id = (ENC_CONFIG_H264_QP_MAP << 8) | temp;
	send_buf.param[i].param_data = vcodec_ctx->video_enc_params.rc_params.h264_qpmap_params.intra_period;
	i++;
	send_buf.param[i].param_id = (ENC_CONFIG_H264_QP_MAP << 8) | (temp + 1);
	send_buf.param[i].param_data = vcodec_ctx->video_enc_params.rc_params.h264_qpmap_params.frame_rate;
	i++;

	vcodec_ctx->video_enc_params.rc_params.mode = ENC_CONFIG_H265_CBR;
	ret = hb_mm_mc_get_rate_control_config(vcodec_ctx, &vcodec_ctx->video_enc_params.rc_params);
	vcodec_ctx->video_enc_params.rc_params.h265_cbr_params.bit_rate =
                mMRTestCtx.bit_rate;
        vcodec_ctx->video_enc_params.rc_params.h265_cbr_params.intra_period =
                mMRTestCtx.intra_period;
	memcpy(array, &vcodec_ctx->video_enc_params.rc_params.h265_cbr_params,
		sizeof(vcodec_ctx->video_enc_params.rc_params.h265_cbr_params));
	for (temp = 0; temp < sizeof(mMCtx.h265_cbr)/sizeof(uint32_t); temp++) {
		send_buf.param[i].param_id = ENC_CONFIG_H265_CBR;
		send_buf.param[i].param_id = (ENC_CONFIG_H265_CBR << 8) | temp;
		send_buf.param[i].param_data = array[temp];
		i++;
	}

	vcodec_ctx->video_enc_params.rc_params.mode = ENC_CONFIG_H265_VBR;
	ret = hb_mm_mc_get_rate_control_config(vcodec_ctx, &vcodec_ctx->video_enc_params.rc_params);
	memcpy(array, &vcodec_ctx->video_enc_params.rc_params.h265_vbr_params,
		sizeof(vcodec_ctx->video_enc_params.rc_params.h265_vbr_params));
	for (temp = 0; temp < sizeof(mMCtx.h265_vbr)/sizeof(uint32_t); temp++) {
		send_buf.param[i].param_id = ENC_CONFIG_H265_VBR;
		send_buf.param[i].param_id = (ENC_CONFIG_H265_VBR << 8) | temp;
		send_buf.param[i].param_data = array[temp];
		i++;
	}

	vcodec_ctx->video_enc_params.rc_params.mode = ENC_CONFIG_H265_AVBR;
	ret = hb_mm_mc_get_rate_control_config(vcodec_ctx, &vcodec_ctx->video_enc_params.rc_params);
	memcpy(array, &vcodec_ctx->video_enc_params.rc_params.h265_avbr_params,
		sizeof(vcodec_ctx->video_enc_params.rc_params.h265_avbr_params));
	for (temp = 0; temp < sizeof(mMCtx.h265_avbr)/sizeof(uint32_t); temp++) {
		send_buf.param[i].param_id = ENC_CONFIG_H265_AVBR;
		send_buf.param[i].param_id = (ENC_CONFIG_H265_AVBR << 8) | temp;
		send_buf.param[i].param_data = array[temp];
		i++;
	}

	vcodec_ctx->video_enc_params.rc_params.mode = ENC_CONFIG_H265_FIX_QP;
	ret = hb_mm_mc_get_rate_control_config(vcodec_ctx, &vcodec_ctx->video_enc_params.rc_params);
	memcpy(array, &vcodec_ctx->video_enc_params.rc_params.h265_fixqp_params,
		sizeof(vcodec_ctx->video_enc_params.rc_params.h265_fixqp_params));
	for (temp = 0; temp < sizeof(mMCtx.h265_fixqp)/sizeof(uint32_t); temp++) {
		send_buf.param[i].param_id = ENC_CONFIG_H265_FIX_QP;
		send_buf.param[i].param_id = (ENC_CONFIG_H265_FIX_QP << 8) | temp;
		send_buf.param[i].param_data = array[temp];
		i++;
	}

	vcodec_ctx->video_enc_params.rc_params.mode = ENC_CONFIG_H265_QP_MAP;
	ret = hb_mm_mc_get_rate_control_config(vcodec_ctx, &vcodec_ctx->video_enc_params.rc_params);
	temp = 0;
	send_buf.param[i].param_id = (ENC_CONFIG_H265_QP_MAP << 8) | temp;
	send_buf.param[i].param_data = vcodec_ctx->video_enc_params.rc_params.h265_qpmap_params.intra_period;
	i++;
	send_buf.param[i].param_id = (ENC_CONFIG_H265_QP_MAP << 8) | (temp + 1);
	send_buf.param[i].param_data = vcodec_ctx->video_enc_params.rc_params.h265_qpmap_params.frame_rate;
	i++;

	ret = hb_mm_mc_get_longterm_ref_mode(vcodec_ctx, &mMCtx.ref_params);
	if (!ret) {
		memcpy(array, &mMCtx.ref_params, sizeof(mMCtx.ref_params));
		for (temp = 0; temp < sizeof(mMCtx.ref_params)/sizeof(uint32_t); temp++) {
			send_buf.param[i].param_id = ENC_CONFIG_LONGTERM_REF;
			send_buf.param[i].param_id = (ENC_CONFIG_LONGTERM_REF << 8) | temp;
			send_buf.param[i].param_data = array[temp];
			i++;
		}
	}

	ret = hb_mm_mc_get_intra_refresh_config(vcodec_ctx, &mMCtx.refresh_params);
	if (!ret) {
		memcpy(array, &mMCtx.refresh_params, sizeof(mMCtx.refresh_params));
		for (temp = 0; temp < sizeof(mMCtx.refresh_params)/sizeof(int32_t); temp++) {
			send_buf.param[i].param_id = ENC_CONFIG_INTRA_REFRESH;
			send_buf.param[i].param_id = (ENC_CONFIG_INTRA_REFRESH << 8) | temp;
			send_buf.param[i].param_data = array[temp];
			i++;
		}
	}
	ret = hb_mm_mc_get_deblk_filter_config(vcodec_ctx,
		&mMCtx.deblk);
	if (!ret) {
		if (vcodec_ctx->codec_id == MEDIA_CODEC_ID_H264) {
			mMCtx.h264_deblk = mMCtx.deblk.h264_deblk;
			memcpy(array, &mMCtx.h264_deblk.disable_deblocking_filter_idc,
				sizeof(mMCtx.h264_deblk));
			for (temp = 0; temp < sizeof(mMCtx.h264_deblk)/sizeof(int32_t); temp++) {
				send_buf.param[i].param_id = ENC_CONFIG_H264_DEBLK_FILTER;
				send_buf.param[i].param_id = (ENC_CONFIG_H264_DEBLK_FILTER << 8) | temp;
				send_buf.param[i].param_data = array[temp];
				i++;
			}
		} else {
			mMCtx.h265_deblk = mMCtx.deblk.h265_deblk;
			memcpy(array, &mMCtx.h265_deblk,
				sizeof(mMCtx.h265_deblk));
			for (temp = 0; temp < sizeof(mMCtx.h265_deblk)/sizeof(int32_t); temp++) {
				send_buf.param[i].param_id = ENC_CONFIG_H265_DEBLK_FILTER;
				send_buf.param[i].param_id = (ENC_CONFIG_H265_DEBLK_FILTER << 8) | temp;
				send_buf.param[i].param_data = array[temp];
				i++;
			}
		}
	}

	if (vcodec_ctx->codec_id == MEDIA_CODEC_ID_H265) {
		ret = hb_mm_mc_get_sao_config(vcodec_ctx, &mMCtx.h265_sao);
		if (!ret) {
			send_buf.param[i].param_id = ENC_CONFIG_H265_SAO;
			send_buf.param[i].param_id = (ENC_CONFIG_H265_SAO << 8) | 0;
			send_buf.param[i].param_data = mMCtx.h265_sao.sample_adaptive_offset_enabled_flag;
			i++;
		}
	}

	if (vcodec_ctx->codec_id == MEDIA_CODEC_ID_H264) {
		ret = hb_mm_mc_get_entropy_config(vcodec_ctx, &mMCtx.h264_entropy);
		if (!ret) {
			send_buf.param[i].param_id = ENC_CONFIG_H264_ENTROPY;
			send_buf.param[i].param_id = (ENC_CONFIG_H264_ENTROPY << 8) | 0;
			send_buf.param[i].param_data = mMCtx.h264_entropy.entropy_coding_mode;
			i++;
		}
	}

	ret = hb_mm_mc_get_vui_timing_config(vcodec_ctx, &mMCtx.timing);
	if (!ret) {
		if (vcodec_ctx->codec_id == MEDIA_CODEC_ID_H264) {
			mMCtx.h264_timing = mMCtx.timing.h264_timing;
			memcpy(array, &mMCtx.h264_timing, sizeof(mMCtx.h264_timing));
			for (temp = 0; temp < sizeof(mMCtx.h264_timing)/sizeof(uint32_t); temp++) {
				send_buf.param[i].param_id = ENC_CONFIG_H264_TIMING;
				send_buf.param[i].param_id = (ENC_CONFIG_H264_TIMING << 8) | temp;
				send_buf.param[i].param_data = array[temp];
				i++;
			}
		} else {
			mMCtx.h265_timing = mMCtx.timing.h265_timing;
			memcpy(array, &mMCtx.h265_timing, sizeof(mMCtx.h265_timing));
			for (temp = 0; temp < sizeof(mMCtx.h265_timing)/sizeof(uint32_t); temp++) {
				send_buf.param[i].param_id = ENC_CONFIG_H265_TIMING;
				send_buf.param[i].param_id = (ENC_CONFIG_H265_TIMING << 8) | temp;
				send_buf.param[i].param_data = array[temp];
				i++;
			}
		}
	}

	ret = hb_mm_mc_get_slice_config(vcodec_ctx, &mMCtx.slice);
	if (!ret) {
		if (vcodec_ctx->codec_id == MEDIA_CODEC_ID_H264) {
			mMCtx.h264_slice = mMCtx.slice.h264_slice;
			memcpy(array, &mMCtx.h264_slice, sizeof(mMCtx.h264_slice));
			for (temp = 0; temp < sizeof(mMCtx.h264_slice)/sizeof(uint32_t); temp++) {
				send_buf.param[i].param_id = ENC_CONFIG_H264_SLICE;
				send_buf.param[i].param_id = (ENC_CONFIG_H264_SLICE << 8) | temp;
				send_buf.param[i].param_data = array[temp];
				i++;
			}
		} else {
			mMCtx.h265_slice = mMCtx.slice.h265_slice;
			memcpy(array, &mMCtx.h265_slice, sizeof(mMCtx.h265_slice));
			for (temp = 0; temp < sizeof(mMCtx.h265_slice)/sizeof(uint32_t); temp++) {
				send_buf.param[i].param_id = ENC_CONFIG_H265_SLICE;
				send_buf.param[i].param_id = (ENC_CONFIG_H265_SLICE << 8) | temp;
				send_buf.param[i].param_data = array[temp];
				i++;
			}
		}
	}

	ret = hb_mm_mc_get_smart_bg_enc_config(vcodec_ctx, &mMCtx.smart_params);
	if (!ret) {
		memcpy(array, &mMCtx.smart_params.bg_detect_enable,
			sizeof(mMCtx.smart_params));
		for (temp = 0; temp < sizeof(mMCtx.smart_params)/sizeof(int32_t); temp++) {
			send_buf.param[i].param_id = ENC_CONFIG_SMART_BG;
			send_buf.param[i].param_id = (ENC_CONFIG_SMART_BG << 8) | temp;
			send_buf.param[i].param_data = array[temp];
			i++;
		}
	}

	ret = hb_mm_mc_get_pred_unit_config(vcodec_ctx, &mMCtx.pred);
	if (!ret) {
		if (vcodec_ctx->codec_id == MEDIA_CODEC_ID_H264) {
			mMCtx.h264_pred = mMCtx.pred.h264_intra_pred;
			memcpy(array, &mMCtx.h264_pred, sizeof(mMCtx.h264_pred));
			for (temp = 0; temp < sizeof(mMCtx.h264_pred)/sizeof(uint32_t); temp++) {
				send_buf.param[i].param_id = ENC_CONFIG_H264_INTRA_PRED;
				send_buf.param[i].param_id = (ENC_CONFIG_H264_INTRA_PRED << 8) | temp;
				send_buf.param[i].param_data = array[temp];
				i++;
			}
		} else {
			mMCtx.h265_pred = mMCtx.pred.h265_pred_unit;
			memcpy(array, &mMCtx.h265_pred, sizeof(mMCtx.h265_pred));
			for (temp = 0; temp < sizeof(mMCtx.h265_pred)/sizeof(uint32_t); temp++) {
				send_buf.param[i].param_id = ENC_CONFIG_H265_PRED_UINT;
				send_buf.param[i].param_id = (ENC_CONFIG_H265_PRED_UINT << 8) | temp;
				send_buf.param[i].param_data = array[temp];
				i++;
			}
		}
	}

	ret = hb_mm_mc_get_transform_config(vcodec_ctx, &mMCtx.transform);
	if (!ret) {
		if (vcodec_ctx->codec_id == MEDIA_CODEC_ID_H264) {
			mMCtx.h264_transform = mMCtx.transform.h264_transform;
			temp = 0;
			send_buf.param[i].param_id = (ENC_CONFIG_H264_TRANSFORM << 8) | (++temp);
			send_buf.param[i].param_data = mMCtx.h264_transform.transform_8x8_enable;
			i++;
			send_buf.param[i].param_id = (ENC_CONFIG_H264_TRANSFORM << 8) | (++temp);
			send_buf.param[i].param_data = mMCtx.h264_transform.transform_8x8_enable;
			i++;
			send_buf.param[i].param_id = (ENC_CONFIG_H264_TRANSFORM << 8) | (++temp);
			send_buf.param[i].param_data = mMCtx.h264_transform.transform_8x8_enable;
			i++;
			send_buf.param[i].param_id = (ENC_CONFIG_H264_TRANSFORM << 8) | (++temp);
			send_buf.param[i].param_data = mMCtx.h264_transform.transform_8x8_enable;
			i++;
		} else {
			mMCtx.h265_transform = mMCtx.transform.h265_transform;
			temp = 0;
			send_buf.param[i].param_id = (ENC_CONFIG_H265_TRANSFORM << 8) | (++temp);
			send_buf.param[i].param_data = mMCtx.h265_transform.chroma_cb_qp_offset;
			i++;
			send_buf.param[i].param_id = (ENC_CONFIG_H265_TRANSFORM << 8) | (++temp);
			send_buf.param[i].param_data = mMCtx.h265_transform.chroma_cr_qp_offset;
			i++;
			send_buf.param[i].param_id = (ENC_CONFIG_H265_TRANSFORM << 8) | (++temp);
			send_buf.param[i].param_data = mMCtx.h265_transform.user_scaling_list_enable;
			i++;
		}
	}
	ret = hb_mm_mc_get_roi_config(vcodec_ctx, &mMCtx.roi);
	if (!ret) {
		send_buf.param[i].param_id = ENC_CONFIG_ROI;
		send_buf.param[i].param_id = (ENC_CONFIG_ROI << 8) | 0;
		send_buf.param[i].param_data = mMCtx.roi.roi_enable;
		i++;

		if (mMCtx.roi.roi_enable == 1) {
			if (mMCtx.roi.roi_map_array) {
				for (temp = 0; temp < mMCtx.roi.roi_map_array_count; temp++) {
					send_buf.param[i].param_id = ENC_CONFIG_ROI;
					send_buf.param[i].param_id = (ENC_CONFIG_ROI << 8) | 1;
					send_buf.param[i].param_data = mMCtx.roi.roi_map_array[temp];
				}
				i++;
			}
			send_buf.param[i].param_id = ENC_CONFIG_ROI;
			send_buf.param[i].param_id = (ENC_CONFIG_ROI << 8) | 2;
			send_buf.param[i].param_data = mMCtx.roi.roi_map_array_count;
			i++;
		}
	}

	if (vcodec_ctx->codec_id == MEDIA_CODEC_ID_H265) {
		ret = hb_mm_mc_get_mode_decision_config(vcodec_ctx, &mMCtx.decision);
		if (!ret) {
			memcpy(array, &mMCtx.decision, sizeof(mMCtx.decision));
			for (temp = 0; temp < sizeof(mMCtx.decision)/sizeof(uint32_t); temp++) {
				send_buf.param[i].param_id = ENC_CONFIG_MODE_DECISION;
				send_buf.param[i].param_id = (ENC_CONFIG_MODE_DECISION << 8) | temp;
				send_buf.param[i].param_data = array[temp];
				i++;
			}
		}
	}
	pthread_mutex_unlock(&(mMCtx.video_parameter_thread));

	send_buf.param_num = i;
	header_info.type = VIDEO_CFG;//yuv
	header_info.width = 0;
	header_info.height = 0;
	header_info.stride = 0;
	header_info.length = send_buf.param_num * sizeof(struct param_buf) + sizeof(send_buf.param_num);
	header_info.frame_id = 0;
	header_info.frame_plane = 0;
	header_info.code_type = 1;
	send_videoinfo(&header_info, &send_buf);

	if (send_buf.param != NULL) {
		free(send_buf.param);
		send_buf.param = NULL;
	}

	return ret;
}

static int startEncoder(MediaRecorderTestContext *mr_ctx)
{
	int ret = 0;
	hb_s32 mc_ret = 0, thread_ret = 0;
	media_codec_context_t *vcodec_ctx;
	mc_video_codec_enc_params_t *enc_params;

	if (!mr_ctx || !mr_ctx->encCtx) {
		vmon_err(" Invalid parameters.\n");
		return -1;
	}

	mr_ctx->video_terminate = 0;
	mr_ctx->video_get_thread_finished = 0;
	mr_ctx->video_running = 1;
	mr_ctx->video_put_thread_finished = 0;

	// get inital video info
        get_send_video_default_info();

	// setup video encoder context
	vcodec_ctx = mr_ctx->encCtx;
	enc_params = &vcodec_ctx->video_enc_params;

	if (mr_ctx->encode_mode == 0) {
		vcodec_ctx->codec_id = MEDIA_CODEC_ID_H264;
	} else {
		vcodec_ctx->codec_id = MEDIA_CODEC_ID_H265;
	}

	vcodec_ctx->encoder = 1;
	enc_params->width = mr_ctx->video_width;
	enc_params->height = mr_ctx->video_height;
	enc_params->pix_fmt = MC_PIXEL_FORMAT_NV12;

	enc_params->frame_buf_count = VIDEO_BUF_NUM;

	enc_params->external_frame_buf = 1;
	enc_params->bitstream_buf_count = 5;
	if (vcodec_ctx->codec_id == MEDIA_CODEC_ID_H264) {
		enc_params->rc_params.mode = MC_AV_RC_MODE_H264CBR;
		mc_ret = hb_mm_mc_get_rate_control_config(vcodec_ctx, &enc_params->rc_params);
		if (!mc_ret) {
			printf("%s SET H264 video encoder context.\n", __func__);
			enc_params->rc_params.h264_cbr_params.bit_rate = mr_ctx->bit_rate;
			enc_params->rc_params.h264_cbr_params.frame_rate = mr_ctx->fps;
			enc_params->rc_params.h264_cbr_params.intra_period = mr_ctx->intra_period;
			enc_params->gop_params.decoding_refresh_type = 2;
			enc_params->gop_params.gop_preset_idx = 2;
			enc_params->rot_degree = MC_CCW_0;
			enc_params->mir_direction = MC_DIRECTION_NONE;
			enc_params->frame_cropping_flag = 0;
		}
	} else { // MEDIA_CODEC_ID_H265
		enc_params->rc_params.mode = MC_AV_RC_MODE_H265CBR;
		mc_ret = hb_mm_mc_get_rate_control_config(vcodec_ctx, &enc_params->rc_params);
		if (!mc_ret) {
			printf("%s SET H265 video encoder context.\n", __func__);
			enc_params->rc_params.h265_cbr_params.bit_rate = mr_ctx->bit_rate;
			enc_params->rc_params.h265_cbr_params.frame_rate = mr_ctx->fps;
			enc_params->rc_params.h265_cbr_params.intra_period = mr_ctx->intra_period;
			enc_params->gop_params.decoding_refresh_type = 2;
			enc_params->gop_params.gop_preset_idx = 2;
			enc_params->rot_degree = MC_CCW_0;
			enc_params->mir_direction = MC_DIRECTION_NONE;
			enc_params->frame_cropping_flag = 0;
		}
	}

	if (mc_ret) {
		printf("%s Failed to setup video encoder context.\n", __func__);
		return -1;
	}

	// initialize media codec
	mc_ret = hb_mm_mc_initialize(vcodec_ctx);
	if (mc_ret) {
		printf("%s-%d, hb_mm_initialize fail.\n", __func__, __LINE__);
		return -1;
	}

	mc_ret = hb_mm_mc_request_idr_header(vcodec_ctx, 2);
	if (mc_ret) {
		printf("%s-%d, hb_mm_mc_configure fail.\n", __func__, __LINE__);
		hb_mm_mc_release(vcodec_ctx);
		return -1;
	}

	mc_ret = hb_mm_mc_configure(vcodec_ctx);
	if (mc_ret) {
		printf("%s-%d, hb_mm_mc_configure fail.\n", __func__, __LINE__);
		hb_mm_mc_release(vcodec_ctx);
		return -1;
	}

	mc_av_codec_startup_params_t startup_params;
	startup_params.video_enc_startup_params.receive_frame_number = 0;
	mc_ret = hb_mm_mc_start(vcodec_ctx, &startup_params);
	if (mc_ret) {
		printf("%s-%d, hb_mm_mc_start fail.\n", __func__, __LINE__);
		hb_mm_mc_release(vcodec_ctx);
		return -1;
	}

	if ((thread_ret =
		pthread_create(&mr_ctx->video_encoder_putbuf_thread, NULL,
		    video_encoder_putbuf_thread, mr_ctx)) != 0) {
		printf("%s Failed to pthread_create ret(%d)\n", __func__, thread_ret);
		hb_mm_mc_stop(vcodec_ctx);
		hb_mm_mc_release(vcodec_ctx);
		return -1;
	}

	if ((thread_ret =
		pthread_create(&mr_ctx->video_encoder_getbuf_thread, NULL,
		    video_encoder_getbuf_thread, mr_ctx)) != 0) {
		printf("%s Failed to pthread_create ret(%d)\n", __func__, thread_ret);
		hb_mm_mc_stop(vcodec_ctx);
		hb_mm_mc_release(vcodec_ctx);

		mr_ctx->video_terminate = 1;
		if ((ret = pthread_join(mr_ctx->video_encoder_putbuf_thread, NULL)) == 0) {
                	mr_ctx->video_encoder_putbuf_thread = 0;
        	} else {
                	printf("%s Failed to pthread_join ret(%d)\n", __func__, ret);
		}

		return -1;
	}

	return 0;
}

static int stopEncoder(MediaRecorderTestContext *mr_ctx) {
	hb_s32 ret = 0;
	void *status;
	if (!mr_ctx && !mr_ctx->encCtx) {
		printf("%s Invalid recorder task!\n", __func__);
		return -1;
	}

	mr_ctx->video_terminate = 1;

	if (mr_ctx->video_encoder_putbuf_thread) {
		if ((ret = pthread_join(mr_ctx->video_encoder_putbuf_thread, &status)) == 0) {
			mr_ctx->video_encoder_putbuf_thread = 0;
		} else {
			printf("%s Failed to pthread_join ret(%d)\n", __func__, ret);
		}
	} else {
		printf("video pthread is not exist\n");
		return -1;
	}

	if (mr_ctx->video_encoder_getbuf_thread) {
		if ((ret = pthread_join(mr_ctx->video_encoder_getbuf_thread, &status)) == 0) {
			mr_ctx->video_encoder_getbuf_thread = 0;
		} else {
			printf("%s Failed to pthread_join ret(%d)\n", __func__, ret);
		}
	} else {
		printf("video pthread is not exist\n");
		return -1;
	}

	hb_mm_mc_stop(mr_ctx->encCtx);

	ret = hb_mm_mc_release(mr_ctx->encCtx);
	mr_ctx->video_running = 0;

	return ret;
}

static int startEncoderJpeg(MediaRecorderTestContext *mr_ctx) {
	hb_s32 mc_ret = 0, thread_ret = 0;
	media_codec_context_t *jepg_ctx;
	mc_video_codec_enc_params_t *jepg_params;
	chn_img_info_t info;

	if (!mr_ctx || !mr_ctx->jepgCtx) {
		vmon_err(" Invalid parameters.\n");
		return -1;
	}

	// setup video encoder context
	jepg_ctx = mr_ctx->jepgCtx;
	jepg_params = &jepg_ctx->video_enc_params;
	// get vio info
	memset(&info, 0x00, sizeof(chn_img_info_t));
	VIO_INFO_TYPE_E info_type = HB_VIO_INFO_MAX;
	switch (mr_ctx->channel_id) {
	case HB_VIO_IPU_DS0_DATA:
		info_type = HB_VIO_IPU_DS0_IMG_INFO;
		break;
	case HB_VIO_IPU_DS1_DATA:
		info_type = HB_VIO_IPU_DS1_IMG_INFO;
		break;
	case HB_VIO_IPU_DS2_DATA:
		info_type = HB_VIO_IPU_DS2_IMG_INFO;
		break;
	case HB_VIO_IPU_DS3_DATA:
		info_type = HB_VIO_IPU_DS3_IMG_INFO;
		break;
	case HB_VIO_IPU_DS4_DATA:
		info_type = HB_VIO_IPU_DS4_IMG_INFO;
		break;
	case HB_VIO_IPU_US_DATA:
		info_type = HB_VIO_IPU_US_IMG_INFO;
		break;
	default:
		break;
	}
	if (info_type == HB_VIO_INFO_MAX) {
		printf("%s Invalid camera information.\n", __func__);
		return -1;
	}
	if (hb_vio_get_param(mr_ctx->pipeline, info_type, &info)) {
		printf("%s Failed to get camera information.\n", __func__);
		return -1;
	}
	if (info.buf_count <= 0 || info.format != HB_YUV420SP) {
		printf("%s Invalid IPU buffer count or format.\n", __func__);
		return -1;
	}

	jepg_ctx->codec_id = MEDIA_CODEC_ID_JPEG;
	jepg_ctx->encoder = 1;
	jepg_params->width = info.width;
	jepg_params->height = info.height;
	jepg_params->pix_fmt = (info.format == HB_YUV420SP)
		? MC_PIXEL_FORMAT_NV12 : MC_PIXEL_FORMAT_YUV420P;
	//jepg_params->frame_buf_count = info.buf_count;
	jepg_params->frame_buf_count = 2;
	jepg_params->external_frame_buf = 1;
	jepg_params->bitstream_buf_count = 1;

	jepg_params->rot_degree = MC_CCW_0;
	jepg_params->mir_direction = MC_DIRECTION_NONE;
	jepg_params->frame_cropping_flag = 0;
	mr_ctx->jepg_terminate = 0;

	// initialize media codec
	mc_ret = hb_mm_mc_initialize(jepg_ctx);
	if (mc_ret) {
		printf("%s-%d, hb_mm_initialize fail.\n", __func__, __LINE__);
		return -1;
	}

	mc_ret = hb_mm_mc_configure(jepg_ctx);
	if (mc_ret) {
		printf("%s-%d, hb_mm_mc_configure fail.\n", __func__, __LINE__);
		hb_mm_mc_release(jepg_ctx);
		return -1;
	}

	mc_av_codec_startup_params_t startup_params;
	memset(&startup_params, 0x00, sizeof(mc_av_codec_startup_params_t));
	startup_params.video_enc_startup_params.receive_frame_number = 0;
	mc_ret = hb_mm_mc_start(jepg_ctx, &startup_params);
	if (mc_ret) {
		printf("%s-%d, hb_mm_mc_start fail.\n", __func__, __LINE__);
		hb_mm_mc_release(jepg_ctx);
		return -1;
	}

	if ((thread_ret =
		pthread_create(&mr_ctx->jepg_encoder_putbuf_thread, NULL,
		    jepg_encoder_putbuf_thread, mr_ctx)) != 0) {
		printf("%s Failed to pthread_create ret(%d)\n", __func__, thread_ret);
		hb_mm_mc_stop(jepg_ctx);
		hb_mm_mc_release(jepg_ctx);
		return -1;
	}

	if ((thread_ret =
		pthread_create(&mr_ctx->jepg_encoder_getbuf_thread, NULL,
		    jepg_encoder_getbuf_thread, mr_ctx)) != 0) {
		printf("%s Failed to pthread_create ret(%d)\n", __func__, thread_ret);
		return -1;
	}

	return 0;
}

static int stopEncoderJpeg(MediaRecorderTestContext *mr_ctx) {
	hb_s32 ret = 0;
	void *status;
	if (!mr_ctx && !mr_ctx->jepgCtx) {
		printf("%s Invalid recorder task!\n", __func__);
		return -1;
	}

	mr_ctx->jepg_terminate = 1;

	if (mr_ctx->jepg_encoder_putbuf_thread) {
		if ((ret = pthread_join(mr_ctx->jepg_encoder_putbuf_thread, &status)) == 0) {
			mr_ctx->jepg_encoder_putbuf_thread = 0;
		} else {
			printf("Failed to pthread_join ret(%d)\n", ret);
		}
	} else {
		return -1;
	}

	if (mr_ctx->jepg_encoder_getbuf_thread) {
		if ((ret = pthread_join(mr_ctx->jepg_encoder_getbuf_thread, &status)) == 0) {
			mr_ctx->jepg_encoder_getbuf_thread = 0;
		} else {
			printf("Failed to pthread_join ret(%d)\n", ret);
		}
	} else {
		return -1;
	}

	hb_mm_mc_stop(mr_ctx->jepgCtx);
	ret = hb_mm_mc_release(mr_ctx->jepgCtx);

	if (mr_ctx->camBuf) {
		free(mr_ctx->camBuf);
		mr_ctx->camBuf = NULL;
	}
	pthread_mutex_destroy(&mr_ctx->cam_buf_lock);
	return ret;
}

int video_func_init(uint32_t pipeline, uint32_t channel)
{
	int ret = 0;

	memset(&mEncCtx, 0x00, sizeof(media_codec_context_t));
	memset(&mJpegCtx, 0x00, sizeof(media_codec_context_t));
	memset(&mMRTestCtx, 0x00, sizeof(MediaRecorderTestContext));
	mMRTestCtx.fps = 25;
	mMRTestCtx.pipeline = pipeline;
	mMRTestCtx.channel_id = channel;
	mMRTestCtx.encode_mode = 0;
	mMRTestCtx.video_width = 1920;
	mMRTestCtx.video_height = 1080;
	mMRTestCtx.bit_rate = 5000;
	mMRTestCtx.intra_period = 25;

	mMRTestCtx.encCtx = &mEncCtx;
	mMRTestCtx.jepgCtx = &mJpegCtx;

	ret = buf_list_init(&videodata_buf);
	ret = buf_list_init(&jepgdata_buf);

	// Allocate camera information buffer
	mMRTestCtx.camBuf = (MRCameraBuffer *)malloc(
		VIDEO_BUF_NUM * sizeof(MRCameraBuffer));
	if (!mMRTestCtx.camBuf) {
		printf("%s Failed to allocate camera buffer!\n", __func__);
		return -1;
	}
	memset(mMRTestCtx.camBuf, 0x00,
		VIDEO_BUF_NUM * sizeof(MRCameraBuffer));
	pthread_mutex_init(&mMRTestCtx.cam_buf_lock, NULL);

	return ret;
}

void set_dynamic_param(struct param_buf video_cfg) {
	mMRTestCtx.video_cfg = video_cfg;
}

void set_video_param(uint32_t width, uint32_t height,
	uint32_t encode_mode, uint32_t bit_rate, uint32_t intra_period)
{
	mMRTestCtx.encode_mode = encode_mode;
	mMRTestCtx.video_width = width;
	mMRTestCtx.video_height = height;
	mMRTestCtx.bit_rate = bit_rate * 1000;
	mMRTestCtx.intra_period = intra_period;
}

int video_start(void)
{
	int ret = 0;

	if(mMRTestCtx.video_running == 0) {
		printf("start encoder.\n", __func__);
		ret = startEncoder(&mMRTestCtx);
		if (ret < 0) {
			printf("%s Failed to start encoder.\n", __func__);
			mMRTestCtx.video_terminate = 1;
		}
	} else {
		ret = -1;
	}
	return ret;
}

int jepg_start(void)
{
	int ret = 0;
	ret = startEncoderJpeg(&mMRTestCtx);
	if (ret < 0) {
		printf("%s Failed to start encoder.\n", __func__);
		mMRTestCtx.jepg_terminate = 1;
	}
	return ret;
}

void video_stop(void)
{
	if (mMRTestCtx.encCtx && stopEncoder(&mMRTestCtx) < 0) {
		//printf("%s Failed to stop encoder.\n", __func__);
	}
}

void jepg_stop(void)
{
	if (mMRTestCtx.jepgCtx && stopEncoderJpeg(&mMRTestCtx) < 0) {
		//printf("%s Failed to stop jepg.\n", __func__);
	}
}

void video_func_deinit(void)
{
	/*jepg*/
	if (mMRTestCtx.camBuf) {
		free(mMRTestCtx.camBuf);
		mMRTestCtx.camBuf = NULL;
	}
	pthread_mutex_destroy(&mMRTestCtx.cam_buf_lock);

	buf_list_deinit(&videodata_buf);
	buf_list_deinit(&jepgdata_buf);
}

