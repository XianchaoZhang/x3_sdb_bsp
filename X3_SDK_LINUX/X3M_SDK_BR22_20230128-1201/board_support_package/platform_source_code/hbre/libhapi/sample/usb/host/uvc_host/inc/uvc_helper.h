/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * uvc_helper.h
 *	some uvc helper functions
 *
 * Copyright (C) 2019 Horizon Robotics, Inc.
 *
 * Contact: jianghe xu<jianghe.xu@horizon.ai>
 */


/**
 * convert_yuy2_to_nv12 - image convert from yuy2 to nv12
 * @in_frame: input frame pointer
 * @out_frame: output frame pointer
 * @width: width of the frame
 * @height: height of the frame
 *
 * Pay attention, please prepare the in_frame & out_frame by yourself.
 * And, make sure the image size is right:
 * yuy2, width * height * 2
 * nv12, width * height * 1.5
 */
int convert_yuy2_to_nv12(void *in_frame, void *out_frame,
		int width, int height);
