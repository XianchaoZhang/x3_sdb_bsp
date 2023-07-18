/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#include <fstream>
#include "vio_vin.h"
#include "vio_vps.h"
#include "vio/vio_log.h"

int hb_vps_init(uint32_t pipeId, vio_cfg_t cfg, IOT_VIN_ATTR_S * vin_attr)
{
	VPS_GRP_ATTR_S grp_attr;
	VPS_CHN_ATTR_S chn_attr;
	VPS_PYM_CHN_ATTR_S pym_chn_attr;
	int ipu_ds2_en = 0;
	int ipu_ds2_crop_en = 0;
	int ret = 0;

	if (vin_attr->pipeinfo == nullptr) {
		pr_err("VPS Init, pipeinfo is NULL\n");
		goto chn_err;
	}
	memset(&grp_attr, 0, sizeof(VPS_GRP_ATTR_S));
	grp_attr.maxW = vin_attr->pipeinfo->stSize.width;
	grp_attr.maxH = vin_attr->pipeinfo->stSize.height;
	grp_attr.frameDepth = cfg.grp_frame_depth[pipeId];
	ret = HB_VPS_CreateGrp(pipeId, &grp_attr);
	if (ret) {
		pr_err("HB_VPS_CreateGrp error!!!\n");
		goto chn_err;
	} else {
		pr_info("created a group ok:GrpId = %d\n", pipeId);
	}

	/* set group gdc */
	if (cfg.need_gdc[0]) {
		if ((cfg.sensor_id[0] == OS8A10_30FPS_3840P_RAW10_LINEAR) ||
		    (cfg.sensor_id[0] == OS8A10_30FPS_3840P_RAW10_DOL2)) {
			pr_info("start to set GDC!!!\n");
			std::ifstream ifs("/app/bin/hapi_xj3/os8a10.bin");
			if (!ifs.is_open()) {
				pr_err("GDC file open failed!\n");
				goto chn_err;
			}
			ifs.seekg(0, std::ios::end);
			auto len = ifs.tellg();
			ifs.seekg(0, std::ios::beg);
			auto buf = new char[len];
			ifs.read(buf, len);

			ret = HB_VPS_SetGrpGdc(pipeId, buf, len, ROTATION_0);
			if (ret) {
				pr_err("HB_VPS_SetGrpGdc error!!!\n");
				goto chn_err;
			} else {
				pr_info("HB_VPS_SetGrpGdc ok: pipeId = %d\n",
					pipeId);
			}
			free(buf);
		}
	}

	for (int i = 0; i < cfg.chn_num[pipeId]; i++) {
		/* 1. set ipu chn */
		if (cfg.ipu_chn_en[pipeId][i] == 1) {
			memset(&chn_attr, 0, sizeof(VPS_CHN_ATTR_S));
			chn_attr.enScale = cfg.scale_en[pipeId][i];
			chn_attr.width = cfg.width[pipeId][i];
			chn_attr.height = cfg.height[pipeId][i];
			chn_attr.frameDepth = cfg.frame_depth[pipeId][i];
			ret = HB_VPS_SetChnAttr(pipeId, i, &chn_attr);
			if (ret) {
				pr_err("HB_VPS_SetChnAttr error!!!\n");
				goto chn_err;
			} else {
				pr_info
				    ("set ipu chn Attr ok: GrpId = %d, chn_id = %d\n",
				     pipeId, i);
			}

			if (cfg.crop_en[pipeId][i]) {
				int crop_chn = i;
				if (i == 6 && ipu_ds2_en == 0) {
					/* set ipu chn2 attr */
					crop_chn = 2;
					memset(&chn_attr, 0,
					       sizeof(VPS_CHN_ATTR_S));
					chn_attr.enScale =
					    cfg.scale_en[pipeId][i];
					chn_attr.width = cfg.width[pipeId][i];
					chn_attr.height = cfg.height[pipeId][i];
					chn_attr.frameDepth = 0;
					ret =
					    HB_VPS_SetChnAttr(pipeId, crop_chn,
							      &chn_attr);
					if (ret) {
						pr_err
						    ("HB_VPS_SetChnAttr error!!!\n");
						goto chn_err;
					} else {
						pr_info
						    ("set ipu chn Attr ok: GrpId = %d, chn_id = %d\n",
						     pipeId, crop_chn);
					}
				}
				if (i == 6 && ipu_ds2_crop_en == 1) {
					pr_info
					    ("ipu chn6 is not need set crop, because chn2 has been"
					     "crop(chn6 and chn2 is shared crop chn settings)");
				} else {
					VPS_CROP_INFO_S crop_info;
					memset(&crop_info, 0,
					       sizeof(VPS_CROP_INFO_S));
					crop_info.en = 1;
					crop_info.cropRect.width =
					    cfg.crop_width[pipeId][i];
					crop_info.cropRect.height =
					    cfg.crop_height[pipeId][i];
					crop_info.cropRect.x =
					    cfg.crop_x[pipeId][i];
					crop_info.cropRect.y =
					    cfg.crop_y[pipeId][i];
					ret =
					    HB_VPS_SetChnCrop(pipeId, crop_chn,
							      &crop_info);
					if (ret) {
						pr_err
						    ("HB_VPS_SetChnCrop error!!!\n");
						goto chn_err;
					} else {
						pr_info
						    ("set ipu chn Crop ok: GrpId = %d, chn_id = %d\n",
						     pipeId, i);
					}
					if (i == 2) {
						ipu_ds2_crop_en = 1;
					}
				}
			}
			if (i == 2) {
				ipu_ds2_en = 1;
			}
			HB_VPS_EnableChn(pipeId, i);
		}
		/* 2. set pym chn */
		if (cfg.pym_chn_en[pipeId][i] == 1) {
			memset(&pym_chn_attr, 0, sizeof(VPS_PYM_CHN_ATTR_S));
			memcpy(&pym_chn_attr,
			       &cfg.pym_cfg[pipeId][i],
			       sizeof(VPS_PYM_CHN_ATTR_S));
			ret = HB_VPS_SetPymChnAttr(pipeId, i, &pym_chn_attr);
			if (ret) {
				pr_err("HB_VPS_SetPymChnAttr error!!!\n");
				goto chn_err;
			} else {
				pr_info
				    ("HB_VPS_SetPymChnAttr ok: grp_id = %d g_pym_chn = %d\n",
				     pipeId, i);
			}
			HB_VPS_EnableChn(pipeId, i);
		}
	}
	return ret;

chn_err:
	// HB_VIN_DestroyPipe(pipeId);  // isp && dwe deinit
	return ret;
}

int hb_vps_start(uint32_t pipe_id)
{
	int ret = 0;
	ret = HB_VPS_StartGrp(pipe_id);
	if (ret) {
		printf("HB_VPS_StartGrp error!!!\n");
	} else {
		printf("start grp ok: grp_id = %d\n", pipe_id);
	}
	return ret;
}

void hb_vps_stop(int pipeId)
{
	HB_VPS_StopGrp(pipeId);
}

void hb_vps_deinit(int pipeId)
{
	HB_VPS_DestroyGrp(pipeId);
}

int hb_vps_input(uint32_t pipe_id, hb_vio_buffer_t * buffer)
{
	int ret = 0;
	ret = HB_VPS_SendFrame(pipe_id, buffer, 1000);
	if (ret != 0) {
		printf("HB_VPS_SendFrame Failed. ret=%d\n", ret);
	}
	return ret;
}

int hb_vps_get_output(uint32_t pipe_id, int channel, hb_vio_buffer_t * buffer)
{
	int ret = 0;
	ret = HB_VPS_GetChnFrame(pipe_id, channel, buffer, 1000);
	if (ret != 0) {
		printf("HB_VPS_GetChnFrame Failed. ret = %d\n", ret);
	}

	return ret;
}

int hb_vps_output_release(uint32_t pipe_id, int channel,
			  hb_vio_buffer_t * buffer)
{
	int ret = 0;
	ret = HB_VPS_ReleaseChnFrame(pipe_id, channel, buffer);
	if (ret != 0) {
		printf("HB_VPS_ReleaseChnFrame Failed. ret = %d\n", ret);
	}
	return ret;
}

#if 1
int hb_vps_change_pym_us(uint32_t pipe_id, int layer, int enable)
{
	int ret = 0;
	ret = HB_VPS_ChangePymUs(pipe_id, layer, enable);
	if (ret != 0) {
		printf("HB_VPS_ReleaseChnFrame Failed. ret = %d\n", ret);
	}
	return ret;
}
#else
int hb_vps_change_pym_us(uint32_t pipe_id, int layer, int enable)
{
	return 0;
}
#endif
