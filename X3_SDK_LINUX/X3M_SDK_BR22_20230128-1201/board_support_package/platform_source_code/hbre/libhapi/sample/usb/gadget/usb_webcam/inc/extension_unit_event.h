/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * extension_unit_event.h
 *		extension unit event, structure definition
 *
 * Copyright (C) 2019 Horizon Robotics, Inc.
 *
 * Contact: jianghe xu<jianghe.xu@horizon.ai>
 */

#ifndef __EXTENSION_UNIT_EVENT_H__
#define __EXTENSION_UNIT_EVENT_H__

/* extension unit enumeration - customer defined */
#define UVC_XU_CONTROL_UNDEFINED			0x00
#define UVC_XU_FIRMWARE_VERSION				0x01

union uvc_extension_unit_control_u {
	int32_t dwFirmwareVersion;			// UVC_XU_FIRMWARE_VERSION
};


#endif	/* __EXTENSION_UNIT_EVENT_H__ */

