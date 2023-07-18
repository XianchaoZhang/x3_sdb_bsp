/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * uvc_helper.c
 *	some uvc helper functions
 *
 * Copyright (C) 2019 Horizon Robotics, Inc.
 *
 * Contact: jianghe xu<jianghe.xu@horizon.ai>
 */
#include <stdio.h>

int convert_yuy2_to_nv12(void *in_frame, void *out_frame,
		unsigned int width, unsigned int height)
{
	int i, j, k, tmp;
	unsigned char *src, *dest;

	if (!in_frame || !out_frame || !width || !height) {
		printf("some error happen... in_frame(%p), out_frame(%p),"
				"width(%u), height(%u)",
				in_frame, out_frame, width, height);
		return -1;
	}

	src = in_frame;
	dest = out_frame;

	/* convert y */
	for (i = 0, k = 0; i < width * height * 2 && k < width * height;
			i += 2, k++) {
		dest[k] = src[i];
	}

	/* convert u, v */
	for (j = 0, k = width * height; j < height && k < width * height * 3 / 2;
			j += 2) {        /* 4:2:0, 1/2 u&v */
		for (i = 1; i < width * 2; i += 2) {
			tmp = i + j * width * 2;
			dest[k++] = src[tmp];
		}
	}

	return 0;
}
