/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * config.h
 *	option config with file
 *
 * Copyright (C) 2019 Horizon Robotics, Inc.
 *
 * Contact: jianghe xu<jianghe.xu@horizon.ai>
 */

#ifndef _USB_WEBCAM_COFNIG_H_
#define _USB_WEBCAM_CONFIG_H_

struct config_key {
	struct config_key *next;
	char *name;
	char *val;
};

struct config_group {
	struct config_group *next;
	char *name;
	struct config_key *keylist;
};

#define config_key_for_each(pos, head) \
	for (pos = head;  pos != NULL; pos = pos->next)

int load_config_file(const char* config_file);
struct config_group *get_config_tree();
struct config_group *find_config_group(const char *group);
struct config_key *find_config_key_in_group(struct config_group *group,
				const char *key);

#endif  /* _USB_WEBCAM_CONFIG_H_ */
