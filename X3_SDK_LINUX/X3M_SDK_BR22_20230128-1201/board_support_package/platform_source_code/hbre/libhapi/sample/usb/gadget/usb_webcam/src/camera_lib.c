/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * camera_lib.c
 *	camera operation function list. (camera_comp_t *handle, eg. fps, brightness, contrast,
 *	saturation, sharpness and so on...)
 *
 * Copyright (camera_comp_t *handle, C) 2019 Horizon Robotics, Inc.
 *
 * Contact: jianghe xu<jianghe.xu@horizon.ai>
 */
#include <stdlib.h>
#include <errno.h>
#include <linux/string.h>
#include <stdio.h>
#include "camera_lib.h"
#include "hb_isp_api.h"

/* Here we set the pipeId to 0.
PipeId is the software path, which is set by the user */
#define CUR_PIPEID    0

int camera_open(camera_comp_t **handle)
{
	int ret = 0;

	if (!handle) {
		ret = -EINVAL;
		goto err;
	}

	camera_comp_t *new_handle = calloc(1, sizeof(camera_comp_t));
	if (!new_handle) {
		ret = -ENOMEM;
		goto err;
	}

	new_handle->name = "camera front";

	/* init ISP before get or set param data */
	HB_ISP_GetSetInit();

	*handle = new_handle;

err:
	return ret;
}

void camera_close(camera_comp_t *handle)
{
	if (!handle)
		return;

	free(handle);

	/* Exit ISP when closing camera */
	HB_ISP_GetSetExit();

	return;
}

int camera_get_fps(camera_comp_t *handle, int *out_fps)
{
	return 0;
}

int camera_set_fps(camera_comp_t *handle, int fps)
{
	return 0;
}

/**
 * camera_get_brightness - get the brightness value of the camera.
 * @handle: pointer to the handle of the camera
 * @out_bri:pointer to the value of the brightness
**/
int camera_get_brightness(camera_comp_t *handle, int *out_bri)
{
	int ret = 0;

	if (!handle) {
		ret = -EINVAL;
		goto err;
	}

	ISP_SCENE_MODES_ATTR_S scenemodesattr;
	memset(&scenemodesattr, 0, sizeof(scenemodesattr));
	ret = HB_ISP_GetSceneModesAttr(CUR_PIPEID, &scenemodesattr);
	*out_bri = scenemodesattr.u32BrightnessStrength;

err:
	return ret;
}
/**
 * camera_set_brightness - set the brightness value of the camera.
 * @handle: pointer to the handle of the camera
 * @bri:the value of the brightness
**/
int camera_set_brightness(camera_comp_t *handle, int bri)
{
	int ret = 0;

	if (!handle) {
		ret = -EINVAL;
		goto err;
	}

	ISP_SCENE_MODES_ATTR_S scenemodesattr;
	memset(&scenemodesattr, 0, sizeof(scenemodesattr));
	ret = HB_ISP_GetSceneModesAttr(CUR_PIPEID, &scenemodesattr);
	if(!ret) {
		scenemodesattr.u32BrightnessStrength = bri;
		ret = HB_ISP_SetSceneModesAttr(CUR_PIPEID, &scenemodesattr);
	}

err:
	return ret;
}

/**
 * camera_get_contrast - get the brightness value of the camera.
 * @handle: pointer to the handle of the camera
 * @out_con:pointer to the value of the contrast
**/
int camera_get_contrast(camera_comp_t *handle, int *out_con)
{
	int ret = 0;

	if (!handle) {
		ret = -EINVAL;
		goto err;
	}

	ISP_SCENE_MODES_ATTR_S scenemodesattr;
	memset(&scenemodesattr, 0, sizeof(scenemodesattr));
	ret = HB_ISP_GetSceneModesAttr(CUR_PIPEID, &scenemodesattr);
	*out_con = scenemodesattr.u32ContrastStrength;

err:
	return ret;
}
/**
 * camera_set_contrast - set the contrast value of the camera.
 * @handle: pointer to the handle of the camera
 * @con:the value of the contrast
**/
int camera_set_contrast(camera_comp_t *handle, int con)
{
	int ret = 0;

	if (!handle) {
		ret = -EINVAL;
		goto err;
	}

	ISP_SCENE_MODES_ATTR_S scenemodesattr;
	memset(&scenemodesattr, 0, sizeof(scenemodesattr));
	ret = HB_ISP_GetSceneModesAttr(CUR_PIPEID, &scenemodesattr);
	if(!ret) {
		scenemodesattr.u32ContrastStrength = con;
		ret = HB_ISP_SetSceneModesAttr(CUR_PIPEID, &scenemodesattr);
	}

err:
	return ret;
}

/**
 * camera_get_saturation - get the saturation value of the camera.
 * @handle: pointer to the handle of the camera
 * @out_sat:pointer to the value of the contrast
**/
int camera_get_saturation(camera_comp_t *handle, int *out_sat)
{
	int ret = 0;

	if (!handle) {
		ret = -EINVAL;
		goto err;
	}

	ISP_SCENE_MODES_ATTR_S scenemodesattr;
	memset(&scenemodesattr, 0, sizeof(scenemodesattr));
	ret = HB_ISP_GetSceneModesAttr(CUR_PIPEID, &scenemodesattr);
	*out_sat = scenemodesattr.u32SaturationStrength;

err:
	return ret;
}
/**
 * camera_set_saturation - set the saturation value of the camera.
 * @handle: pointer to the handle of the camera
 * @sat: the value of the contrast
**/
int camera_set_saturation(camera_comp_t *handle, int sat)
{
	int ret = 0;

	if (!handle) {
		ret = -EINVAL;
		goto err;
	}

	ISP_SCENE_MODES_ATTR_S scenemodesattr;
	memset(&scenemodesattr, 0, sizeof(scenemodesattr));
	ret = HB_ISP_GetSceneModesAttr(0, &scenemodesattr);
	if(!ret) {
		scenemodesattr.u32SaturationStrength = sat;
		ret = HB_ISP_SetSceneModesAttr(CUR_PIPEID, &scenemodesattr);
	}

err:
	return ret;
}
/**
 * camera_get_sharpness - get the sharpness value of the camera.
 * @handle: pointer to the handle of the camera
 * @out_sp:pointer to the value of the sharpness
**/
int camera_get_sharpness(camera_comp_t *handle, int *out_sp)
{
	int ret = 0;

	if (!handle) {
		ret = -EINVAL;
		goto err;
	}

	ISP_SHARPEN_ATTR_S sharpenattrs;
	memset(&sharpenattrs, 0, sizeof(sharpenattrs));
	ret = HB_ISP_GetSharpenAttr(CUR_PIPEID, &sharpenattrs);
	*out_sp = sharpenattrs.stManual.u32Strength;

err:
	return ret;
}
/**
 * camera_set_sharpness - set the sharpness value of the camera.
 * @handle: pointer to the handle of the camera
 * @sp: the value of the sharpness
**/
int camera_set_sharpness(camera_comp_t *handle, int sp)
{
	int ret = 0;

	if (!handle) {
		ret = -EINVAL;
		goto err;
	}
	ISP_SHARPEN_ATTR_S sharpenattrs;
	memset(&sharpenattrs, 0, sizeof(sharpenattrs));
	ret = HB_ISP_GetSharpenAttr(CUR_PIPEID, &sharpenattrs);
	if(!ret) {
		sharpenattrs.stManual.u32Strength = sp;
		ret = HB_ISP_SetSharpenAttr(CUR_PIPEID, &sharpenattrs);
	}

err:
	return ret;
}

int camera_get_gain(camera_comp_t *handle, int *out_iso)
{
	return 0;
}

int camera_set_gain(camera_comp_t *handle, int iso)
{
	return 0;
}

int camera_set_wb(camera_comp_t *handle, enum cam_wb_mode mode)
{
	return 0;
}

int camera_get_wb(camera_comp_t *handle, enum cam_wb_mode *out_mode)
{
	return 0;
}
/**
 * camera_get_hue - get the hue value of the camera.
 * @handle: pointer to the handle of the camera
 * @out_hue:pointer to the value of the hue
**/
int camera_get_hue(camera_comp_t *handle, int *out_hue)
{
	int ret = 0;

	if (!handle) {
		ret = -EINVAL;
		goto err;
	}

	ISP_SCENE_MODES_ATTR_S scenemodesattr;
	memset(&scenemodesattr, 0, sizeof(scenemodesattr));
	ret = HB_ISP_GetSceneModesAttr(CUR_PIPEID, &scenemodesattr);
	*out_hue = scenemodesattr.u32HueTheta;

err:
	return ret;
}
/**
 * camera_set_hue - set the hue value of the camera.
 * @handle: pointer to the handle of the camera
 * @hue: the value of the hue
**/
int camera_set_hue(camera_comp_t *handle, int hue)
{
	int ret = 0;

	if (!handle) {
		ret = -EINVAL;
		goto err;
	}
	ISP_SCENE_MODES_ATTR_S scenemodesattr;
	memset(&scenemodesattr, 0, sizeof(scenemodesattr));
	ret = HB_ISP_GetSceneModesAttr(CUR_PIPEID, &scenemodesattr);
	if(!ret) {
		scenemodesattr.u32HueTheta = hue;
		ret = HB_ISP_SetSceneModesAttr(CUR_PIPEID, &scenemodesattr);
	}

err:
	return ret;
}
/**
 * camera_get_exposure - get the exposure value of the camera.
 * @handle: pointer to the handle of the camera
 * @out_level:pointer to the value of the exposure
**/
int camera_get_exposure(camera_comp_t *handle, int *out_level)
{
	int ret = 0;

	if (!handle) {
		ret = -EINVAL;
		goto err;
	}

	ISP_AE_ATTR_S ispaeattr;
	memset(&ispaeattr, 0, sizeof(ispaeattr));
	ret = HB_ISP_GetAeAttr(CUR_PIPEID, &ispaeattr);
	*out_level = ispaeattr.u32IntegrationTime;

err:
	return ret;
}
/**
 * camera_set_exposure - set the exposure value of the camera.
 * @handle: pointer to the handle of the camera
 * @level: the value of the exposure
**/
int camera_set_exposure(camera_comp_t *handle, int level)
{
	int ret = 0;

	if (!handle) {
		ret = -EINVAL;
		goto err;
	}
	ISP_AE_ATTR_S ispaeattr;
	memset(&ispaeattr, 0, sizeof(ispaeattr));
	ret = HB_ISP_GetAeAttr(CUR_PIPEID, &ispaeattr);
	if(!ret) {
		ispaeattr.u32IntegrationTime = level;
		/* Setting this value requires setting it to manual mode first*/
		ispaeattr.enOpType = OP_TYPE_MANUAL;
		ret = HB_ISP_SetAeAttr(CUR_PIPEID, &ispaeattr);
	}

err:
	return ret;
}

/**
 * camera_get_zoompos - get the zoompos value of the camera.
 * @handle: pointer to the handle of the camera
 * @out_level:pointer to the value of the zoompos
**/
int camera_get_zoompos(camera_comp_t *handle, int *out_level)
{
	int ret = 0;

	if (!handle) {
		ret = -EINVAL;
		goto err;
	}

	ISP_AF_ATTR_S ispafattr;
	memset(&ispafattr, 0, sizeof(ispafattr));
	ret = HB_ISP_GetAfAttr(CUR_PIPEID, &ispafattr);
	*out_level = ispafattr.u32ZoomPos;

err:
	return ret;
}

/**
 * camera_set_zoompos - set the zoompos value of the camera.
 * @handle: pointer to the handle of the camera
 * @level: the value of the zoompos
**/
int camera_set_zoompos(camera_comp_t *handle, int level)
{
	int ret = 0;

	if (!handle) {
		ret = -EINVAL;
		goto err;
	}

	ISP_AF_ATTR_S ispafattr;
	memset(&ispafattr, 0, sizeof(ispafattr));
	ret = HB_ISP_GetAfAttr(CUR_PIPEID, &ispafattr);
	if(!ret) {
		ispafattr.u32ZoomPos = level;
		ret = HB_ISP_SetAfAttr(CUR_PIPEID, &ispafattr);
	}

err:
	return ret;
}

int camera_monochrome_is_enabled(camera_comp_t *handle)
{
	return 0;
}

int camera_monochrome_enable(camera_comp_t *handle, int en)
{
	return 0;
}

int camera_ae_is_enabled(camera_comp_t *handle)
{
	return 0;
}
/**
 * camera_ae_enable - set the ae status of the camera.
 * @handle: pointer to the handle of the camera
 * @en: the status value: 0：disable 1:enable
**/
int camera_ae_enable(camera_comp_t *handle, int en)
{
	int ret = 0;

	if (!handle) {
		ret = -EINVAL;
		goto err;
	}

	ISP_AE_ATTR_S ispaeattr;
	memset(&ispaeattr, 0, sizeof(ispaeattr));
	ret = HB_ISP_GetAeAttr(CUR_PIPEID, &ispaeattr);
	if(!ret) {
		ispaeattr.enOpType = en;
		ret = HB_ISP_SetAeAttr(CUR_PIPEID, &ispaeattr);
	}

err:
	return ret;
}

/**
 * camera_get_aemode - get the ae status of the camera.
 * @handle: pointer to the handle of the camera
 * @aemode: pointer to the status of aemode
**/
int camera_get_aemode(camera_comp_t *handle, int *aemode)
{
	int ret = 0;

	if (!handle) {
		ret = -EINVAL;
		goto err;
	}

	ISP_AE_ATTR_S ispaeattr;
	memset(&ispaeattr, 0, sizeof(ispaeattr));
	ret = HB_ISP_GetAeAttr(CUR_PIPEID, &ispaeattr);
	*aemode = ispaeattr.enOpType;

err:
	return ret;
}

int camera_wdr_is_enabled(camera_comp_t *handle)
{
	return 0;
}

int camera_wdr_enable(camera_comp_t *handle, int en)
{
	return 0;
}
