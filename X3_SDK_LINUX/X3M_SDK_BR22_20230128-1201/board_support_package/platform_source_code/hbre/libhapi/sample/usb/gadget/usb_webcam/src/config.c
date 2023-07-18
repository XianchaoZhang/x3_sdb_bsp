/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * config.c
 *	option config with file
 *
 * Copyright (C) 2019 Horizon Robotics, Inc.
 *
 * Contact: jianghe xu<jianghe.xu@horizon.ai>
 */

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include "config.h"

static struct config_group *config_tree = NULL;
struct config_group *get_config_tree()
{
	return config_tree;
}

static char *trim_string(char *str)
{
	char *s = str;
	char *end;

	/* remove leading spaces/tabs */
	while (isspace(*s))
		s++;

	/* check for null string */
	if (*s == '\0')
		return s;	/* already zero length */

	/* remove trailing spaces and newlines */
	end = s + strlen(s) - 1;
	while (end !=s && isspace(*end))
		end--;

	*(end + 1) = '\0';

	return s;
}

struct config_group *find_config_group(const char *group)
{
	struct config_group *grp;

	for (grp = config_tree; grp != NULL; grp = grp->next) {
		if (strcasecmp(grp->name, group) == 0)
			return grp;
	}

	return NULL;
}

struct config_key *find_config_key_in_group(struct config_group *group,
				const char *key)
{
	struct config_key *k;

	for (k = group->keylist; k != NULL; k = k->next) {
		if (strcasecmp(k->name, key) == 0)
			return k;
	}

	return NULL;
}

static int add_config_key(const char *group, const char *key, const char *val)
{
	struct config_group *grp;
	struct config_key *k;

	printf("add_config_key: %s.%s = '%s'\n", group, key, val);

	/* fine or create key */
	grp = find_config_group(group);
	if (grp == NULL) {
		/* create a new one */
		grp = malloc(sizeof(struct config_group));
		grp->name = strdup(group);
		grp->keylist = NULL;

		/* add it to the config list */
		grp->next = config_tree;
		config_tree = grp;
	}

	/* find or create key */
	k = find_config_key_in_group(grp, key);
	if (k == NULL) {
		/* create a new one */
		k = malloc(sizeof(struct config_key));
		k->name = strdup(key);
		k->val = strdup(val);

		/* add it to the group */
		k->next = grp->keylist;
		grp->keylist = k;
	} else {
		/* the key already exists, override it */
		if (k->val != NULL)
			free(k->val);
		k->val = strdup(val);
	}

	return 0;
}

int load_config_file(const char* filename)
{
	FILE *fp = NULL;
	char line_buf[1024];
	char group_buf[1024];
	char *curr_group = NULL;
	int line_num = -1;

	if (!filename)
		return -1;

	fp = fopen(filename, "r");
	if (!fp)
		return -1;

	/* initialize the config tree */
	config_tree = NULL;
	while (!feof(fp)) {
		char *s;
		char *g;

		/* read in a line */
		line_num++;
		s = fgets(line_buf, sizeof(line_buf), fp);
		if (!s)
			break;

		/* remove leading and trailing spaces */
		s = trim_string(s);

		/* see if it's zero length */
		if (*s == '\0')
			continue;

		if (*s == '[') {
			s++;
			g = group_buf;
			while (*s != '\0' && *s != ']')
				*g++ = *s++;
			*g = 0;

			/* trim the group buffer */
			curr_group = trim_string(group_buf);
		} else if (*s == '#') {
			/* it's comment, ignore */
			continue;
		} else {
			/* assume it's a name value pair, in 'name = val' format */
			char *name, *val;

			/* pick out the name */
			name = s;
			while (*s != '\0' && *s != '=')
				s++;

			if (*s == '\0') {
				printf("load_config_file: name with no value, "
						"line %d\n", line_num);
				continue;
			}
			*s++ = 0;

			/* value should be the rest of the line */
			val = s;

			/* trim the name and value pairs */
			name = trim_string(name);
			val = trim_string(val);

			/* see if we're outside of a group */
			if (curr_group[0] == '\0') {
				printf("load_config_file: name/value pair "
					"outside of group, line %d\n", line_num);
				continue;
			}

			/* add the name/value to the config tree */
			add_config_key(curr_group, name, val);
		}
	}

	fclose(fp);

	return 0;
}
