// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (C) 2016 The Android Open Source Project
 */

#include <common.h>
#include <fastboot.h>
#include <fastboot-internal.h>
#include <fb_mmc.h>
#include <fb_nand.h>
#include <fb_spinand.h>
#include <fs.h>
#include <version.h>

static void getvar_version(char *var_parameter, char *response);
static void getvar_bootloader_version(char *var_parameter, char *response);
static void getvar_downloadsize(char *var_parameter, char *response);
static void getvar_serialno(char *var_parameter, char *response);
static void getvar_version_baseband(char *var_parameter, char *response);
static void getvar_product(char *var_parameter, char *response);
static void getvar_current_slot(char *var_parameter, char *response);
static void getvar_slot_suffixes(char *var_parameter, char *response);
static void getvar_has_slot(char *var_parameter, char *response);
#if CONFIG_IS_ENABLED(FASTBOOT_FLASH_MMC)
static void getvar_partition_type(char *part_name, char *response);
#endif
#if CONFIG_IS_ENABLED(FASTBOOT_FLASH)
static void getvar_partition_size(char *part_name, char *response);
static void getvar_block_size(char *part_name, char *response);
#endif

static const struct {
	const char *variable;
	void (*dispatch)(char *var_parameter, char *response);
} getvar_dispatch[] = {
	{
		.variable = "version",
		.dispatch = getvar_version
	}, {
		.variable = "bootloader-version",
		.dispatch = getvar_bootloader_version
	}, {
		.variable = "version-bootloader",
		.dispatch = getvar_bootloader_version
	}, {
		.variable = "downloadsize",
		.dispatch = getvar_downloadsize
	}, {
		.variable = "max-download-size",
		.dispatch = getvar_downloadsize
	}, {
		.variable = "serialno",
		.dispatch = getvar_serialno
	}, {
		.variable = "version-baseband",
		.dispatch = getvar_version_baseband
	}, {
		.variable = "product",
		.dispatch = getvar_product
	}, {
		.variable = "current-slot",
		.dispatch = getvar_current_slot
	}, {
		.variable = "slot-suffixes",
		.dispatch = getvar_slot_suffixes
	}, {
		.variable = "has_slot",
		.dispatch = getvar_has_slot
#if CONFIG_IS_ENABLED(FASTBOOT_FLASH_MMC)
	}, {
		.variable = "partition-type",
		.dispatch = getvar_partition_type
#endif
#if CONFIG_IS_ENABLED(FASTBOOT_FLASH)
	}, {
		.variable = "partition-size",
		.dispatch = getvar_partition_size
	}, {
		.variable = "block-size",
		.dispatch = getvar_block_size
#endif
	}
};

static void getvar_version(char *var_parameter, char *response)
{
	fastboot_okay(FASTBOOT_VERSION, response);
}

static void getvar_bootloader_version(char *var_parameter, char *response)
{
	fastboot_okay(U_BOOT_VERSION, response);
}

static void getvar_downloadsize(char *var_parameter, char *response)
{
	fastboot_response("OKAY", response, "0x%08x", fastboot_buf_size);
}

static void getvar_serialno(char *var_parameter, char *response)
{
	const char *tmp = env_get("serial#");

	if (tmp)
		fastboot_okay(tmp, response);
	else
		fastboot_fail("Value not set", response);
}

static void getvar_version_baseband(char *var_parameter, char *response)
{
	fastboot_okay("N/A", response);
}

static void getvar_product(char *var_parameter, char *response)
{
	const char *board = env_get("board");

	if (board)
		fastboot_okay(board, response);
	else
		fastboot_fail("Board not set", response);
}

static void getvar_current_slot(char *var_parameter, char *response)
{
	/* A/B not implemented, for now always return _a */
	fastboot_okay("_a", response);
}

static void getvar_slot_suffixes(char *var_parameter, char *response)
{
	fastboot_okay("_a,_b", response);
}

static void getvar_has_slot(char *part_name, char *response)
{
	if (part_name && (!strcmp(part_name, "boot") ||
			  !strcmp(part_name, "system")))
		fastboot_okay("yes", response);
	else
		fastboot_okay("no", response);
}

#if CONFIG_IS_ENABLED(FASTBOOT_FLASH_MMC)
static void getvar_partition_type(char *part_name, char *response)
{
	int r;
	struct blk_desc *dev_desc;
	disk_partition_t part_info;

	if (fastboot_get_flash_type() != FLASH_TYPE_UNKNOWN &&
			fastboot_get_flash_type() != FLASH_TYPE_EMMC) {
		fastboot_fail("not emmc flash, couldn't get partition type",
				response);
		return;
	}

	r = fastboot_mmc_get_part_info(part_name, &dev_desc, &part_info,
				       response);
	if (r >= 0) {
		r = fs_set_blk_dev_with_part(dev_desc, r);
		if (r < 0)
			fastboot_fail("failed to set partition", response);
		else
			fastboot_okay(fs_get_type_name(), response);
	}
}
#endif

#if CONFIG_IS_ENABLED(FASTBOOT_FLASH)
static void getvar_partition_size(char *part_name, char *response)
{
	char *cur, *next;
	unsigned long start, length;
	int r = -1;
	size_t size = 0;

#if CONFIG_IS_ENABLED(FASTBOOT_FLASH_MMC)
	if (fastboot_get_flash_type() == FLASH_TYPE_UNKNOWN ||
			fastboot_get_flash_type() == FLASH_TYPE_EMMC) {
		struct blk_desc *dev_desc;
		disk_partition_t part_info;

		next = part_name;
		cur = strsep(&next, "@");

		/* addr@length or addr@end_part case */
		if (cur && !strict_strtoul(cur, 16, &start)) {
			if (!next)
				goto out;

			/* addr@length */
			if (!strict_strtoul(next, 16, &length)) {
				size = length;

				r = 0;
				goto out;
			}

			/* addr@end_part_name */
			r = fastboot_mmc_get_part_info(next, &dev_desc, &part_info,
						       response);
			if (r >= 0) {
				size = part_info.start + part_info.size;

				goto out;
			}
		}

		r = fastboot_mmc_get_part_info(part_name, &dev_desc, &part_info,
					       response);
		if (r >= 0)
			size = part_info.size;
	}
#endif
#if CONFIG_IS_ENABLED(FASTBOOT_FLASH_NAND)
	if (fastboot_get_flash_type() == FLASH_TYPE_NAND) {
		struct part_info *part_info;

		r = fastboot_nand_get_part_info(part_name, &part_info, response);
		if (r >= 0)
			size = part_info->size;
	}
#endif
#if CONFIG_IS_ENABLED(FASTBOOT_FLASH_SPINAND)
	if (fastboot_get_flash_type() == FLASH_TYPE_SPINAND) {
		struct part_info *part_info;

		r = fastboot_spinand_get_part_info(part_name, &part_info, response);
		if (r >= 0)
			size = part_info->size;
	}
#endif

out:
	if (r >= 0)
		fastboot_response("OKAY", response, "0x%016zx", size);
}
#endif

#if CONFIG_IS_ENABLED(FASTBOOT_FLASH)
static void getvar_block_size(char *part_name, char *response)
{
	int r = -1;
	size_t size = 0;

#if CONFIG_IS_ENABLED(FASTBOOT_FLASH_MMC)
	if (fastboot_get_flash_type() == FLASH_TYPE_UNKNOWN ||
			fastboot_get_flash_type() == FLASH_TYPE_EMMC) {
		struct blk_desc *dev_desc;

		dev_desc = blk_get_dev("mmc", CONFIG_FASTBOOT_FLASH_MMC_DEV);
		if (!dev_desc) {
			fastboot_fail("block device not found", response);
		} else {
			size = dev_desc->blksz;
			r = 0;
		}
	}
#endif
#if CONFIG_IS_ENABLED(FASTBOOT_FLASH_NAND)
	if (fastboot_get_flash_type() == FLASH_TYPE_NAND) {
		struct part_info *part_info;

		r = fastboot_nand_get_part_info(part_name, &part_info, response);
		if (r >= 0)
			size = part_info->sector_size;
	}
#endif
#if CONFIG_IS_ENABLED(FASTBOOT_FLASH_SPINAND)
	if (fastboot_get_flash_type() == FLASH_TYPE_SPINAND) {
		struct part_info *part_info;

		r = fastboot_spinand_get_part_info(part_name, &part_info, response);
		if (r >= 0)
			size = part_info->sector_size;
	}
#endif
	if (r >= 0)
		fastboot_response("OKAY", response, "0x%016zx", size);
}
#endif

/**
 * fastboot_getvar() - Writes variable indicated by cmd_parameter to response.
 *
 * @cmd_parameter: Pointer to command parameter
 * @response: Pointer to fastboot response buffer
 *
 * Look up cmd_parameter first as an environment variable of the form
 * fastboot.<cmd_parameter>, if that exists return use its value to set
 * response.
 *
 * Otherwise lookup the name of variable and execute the appropriate
 * function to return the requested value.
 */
void fastboot_getvar(char *cmd_parameter, char *response)
{
	if (!cmd_parameter) {
		fastboot_fail("missing var", response);
	} else {
#define FASTBOOT_ENV_PREFIX	"fastboot."
		int i;
		char *var_parameter = cmd_parameter;
		char envstr[FASTBOOT_RESPONSE_LEN];
		const char *s;

		snprintf(envstr, sizeof(envstr) - 1,
			 FASTBOOT_ENV_PREFIX "%s", cmd_parameter);
		s = env_get(envstr);
		if (s) {
			fastboot_response("OKAY", response, "%s", s);
			return;
		}

		strsep(&var_parameter, ":");
		for (i = 0; i < ARRAY_SIZE(getvar_dispatch); ++i) {
			if (!strcmp(getvar_dispatch[i].variable,
				    cmd_parameter)) {
				getvar_dispatch[i].dispatch(var_parameter,
							    response);
				return;
			}
		}
		pr_warn("WARNING: unknown variable: %s\n", cmd_parameter);
		fastboot_fail("Variable not implemented", response);
	}
}
