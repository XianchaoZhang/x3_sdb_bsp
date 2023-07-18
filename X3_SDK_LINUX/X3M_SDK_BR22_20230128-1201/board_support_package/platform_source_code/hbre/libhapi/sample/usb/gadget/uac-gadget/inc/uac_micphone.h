/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * uac_micphone.h
 *	uac micphone api.
 *
 * Copyright (C) 2019 Horizon Robotics, Inc.
 *
 * Contact: jianghe xu<jianghe.xu@horizon.ai>
 */

#ifndef _UAC_micphone_H_
#define _UAC_micphone_H_

#include "alsa_device.h"

typedef struct uac_micphone_device {
	alsa_device_t *micphone_device;
	alsa_device_t *uac_device;
	pthread_t thread_id;
	int exit;
} uac_micphone_device_t;

uac_micphone_device_t *uac_micphone_allocate(void);

int uac_micphone_init(uac_micphone_device_t **uac_micphone);
int uac_micphone_start(uac_micphone_device_t *uac_micphone);
int uac_micphone_stop(uac_micphone_device_t *uac_micphone);
void uac_micphone_deinit(uac_micphone_device_t *uac_micphone);

#endif	/* _UAC_micphone_H_ */
