// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2011 Sebastian Andrzej Siewior <bigeasy@linutronix.de>
 */

#include <common.h>
#include <image.h>
#include <android_image.h>
#include <malloc.h>
#include <errno.h>
#include <hb_info.h>
#include <asm/unaligned.h>

#define ANDROID_IMAGE_DEFAULT_KERNEL_ADDR	0x10008000

static char andr_tmp_str[ANDR_BOOT_ARGS_SIZE + 1];

static ulong android_image_get_kernel_addr(const struct andr_img_hdr *hdr)
{
	/*
	 * All the Android tools that generate a boot.img use this
	 * address as the default.
	 *
	 * Even though it doesn't really make a lot of sense, and it
	 * might be valid on some platforms, we treat that adress as
	 * the default value for this field, and try to execute the
	 * kernel in place in such a case.
	 *
	 * Otherwise, we will return the actual value set by the user.
	 */
	if (hdr->kernel_addr == ANDROID_IMAGE_DEFAULT_KERNEL_ADDR)
		return (ulong)hdr + hdr->page_size;

	return hdr->kernel_addr;
}

/**
 * android_image_get_kernel() - processes kernel part of Android boot images
 * @hdr:	Pointer to image header, which is at the start
 *			of the image.
 * @verify:	Checksum verification flag. Currently unimplemented.
 * @os_data:	Pointer to a ulong variable, will hold os data start
 *			address.
 * @os_len:	Pointer to a ulong variable, will hold os data length.
 *
 * This function returns the os image's start address and length. Also,
 * it appends the kernel command line to the bootargs env variable.
 *
 * Return: Zero, os start address and length on success,
 *		otherwise on failure.
 */
int android_image_get_kernel(const struct andr_img_hdr *hdr, int verify,
			     ulong *os_data, ulong *os_len)
{
	u32 kernel_addr = android_image_get_kernel_addr(hdr);

	/*
	 * Not all Android tools use the id field for signing the image with
	 * sha1 (or anything) so we don't check it. It is not obvious that the
	 * string is null terminated so we take care of this.
	 */
	strncpy(andr_tmp_str, hdr->name, ANDR_BOOT_NAME_SIZE);
	andr_tmp_str[ANDR_BOOT_NAME_SIZE] = '\0';
	if (strlen(andr_tmp_str))
		DEBUG_LOG("Android's image name: %s\n", andr_tmp_str);

	DEBUG_LOG("Kernel load addr 0x%08x size %u KiB\n",
	       kernel_addr, DIV_ROUND_UP(hdr->kernel_size, 1024));

	int len = 0;
	if (*hdr->cmdline) {
		printf("Kernel command line: %s\n", hdr->cmdline);
		len += strlen(hdr->cmdline);
	}

	char *bootargs = env_get("bootargs");
	if (bootargs)
		len += strlen(bootargs);

	char *newbootargs = malloc(len + 2);
	if (!newbootargs) {
		puts("Error: malloc in android_image_get_kernel failed!\n");
		return -ENOMEM;
	}
	*newbootargs = '\0';

	if (bootargs) {
		strcpy(newbootargs, bootargs);
		strcat(newbootargs, " ");
	}
	if (*hdr->cmdline)
		strcat(newbootargs, hdr->cmdline);

	env_set("bootargs", newbootargs);

	if (os_data) {
		*os_data = (ulong)hdr;
		*os_data += hdr->page_size;
	}
	if (os_len)
		*os_len = hdr->kernel_size;
	return 0;
}

int android_image_check_header(const struct andr_img_hdr *hdr)
{
	return memcmp(ANDR_BOOT_MAGIC, hdr->magic, ANDR_BOOT_MAGIC_SIZE);
}

/* Check Linux kernel compress algorithm (Image.lz4 or Image.gz) */
ulong android_image_get_kcomp(const struct andr_img_hdr *hdr)
{
	const void *p = (void *)((uintptr_t)hdr + hdr->page_size);

	DEBUG_LOG("Kernel compress with: ");
	if (get_unaligned_le32(p) == LZ4F_MAGIC) {
		DEBUG_LOG("lz4\n");
		return IH_COMP_LZ4;
	} else if (get_unaligned_le16(p) == GZIPF_MAGIC) {
		DEBUG_LOG("gzip\n");
		return IH_COMP_GZIP;
	} else {
		DEBUG_LOG("unknow\n");
		return IH_COMP_NONE;
	}
}

ulong android_image_get_end(const struct andr_img_hdr *hdr)
{
	ulong end;
	/*
	 * The header takes a full page, the remaining components are aligned
	 * on page boundary
	 */
	end = (ulong)hdr;
	end += hdr->page_size;
	end += ALIGN(hdr->kernel_size, hdr->page_size);
	end += ALIGN(hdr->ramdisk_size, hdr->page_size);
	end += ALIGN(hdr->second_size, hdr->page_size);

	return end;
}

ulong android_image_get_kload(const struct andr_img_hdr *hdr)
{
	return android_image_get_kernel_addr(hdr);
}

int android_image_get_ramdisk(const struct andr_img_hdr *hdr,
			      ulong *rd_data, ulong *rd_len)
{
	if (!hdr->ramdisk_size) {
		*rd_data = *rd_len = 0;
		return -1;
	}

	*rd_data = (unsigned long)hdr;
	*rd_data += hdr->page_size;
	*rd_data += ALIGN(hdr->kernel_size, hdr->page_size);

	*rd_len = hdr->ramdisk_size;
	return 0;
}

static char *x3_image_get_dtb(unsigned int board_type,
		struct hb_kernel_hdr *config, ulong *second_data,
		ulong *second_len)
{
	char *s = NULL;
	int i, count;
	ulong base_addr;

	count = config->dtb_number;

	if (count > DTB_MAX_NUM) {
		printf("error: count %02x not support\n", count);
		return NULL;
	}

	for (i = 0; i < count; i++) {
		if (board_type == config->dtb[i].board_id) {
			s = (char *)config->dtb[i].dtb_name;
			base_addr = ((ulong)config) + sizeof(struct hb_kernel_hdr);

			/* base_addr + offset */
			*second_data = base_addr + config->dtb[i].dtb_addr;
			*second_len = config->dtb[i].dtb_size;

			break;
		}
	}

	if (i == count) {
		printf("error: board_type %02x not support\n", board_type);
		return NULL;
	}

	return s;
}

int android_image_get_second(const struct andr_img_hdr *hdr,
	      ulong *second_data, ulong *second_len)
{
	uint64_t second_addr = 0;
	char *name = NULL;
	uint32_t board_type = 0;

	if (!hdr->second_size) {
		*second_data = *second_len = 0;
		return -1;
	}

	/* get second stage addr */
	second_addr = (uint64_t)hdr;
	second_addr += hdr->page_size;
	second_addr += ALIGN(hdr->kernel_size, hdr->page_size);
	second_addr += ALIGN(hdr->ramdisk_size, hdr->page_size);

	/* search file dtb depend on board_id */
    /* if BOARD_TYPE is defined, read board_type by pin state
     * Vaild define is:
     *     BOARD_TYPE_PIN_0   BOARD_TYPE_PIN_1   BOARD_TYPE_PIN_2
     *           x                   0                 0
     *           x                   x                 0
     *           x                   x                 x
     * x > 0 && x < HB_PIN_MAX_NUMS
     */
    if (BOARD_TYPE_PIN_0 > 0 && BOARD_TYPE_PIN_0 < HB_PIN_MAX_NUMS
        && BOARD_TYPE_PIN_1 == 0 && BOARD_TYPE_PIN_2 == 0) {
        board_type = hb_board_type_get_by_pin(1);
    } else if (BOARD_TYPE_PIN_0 > 0 && BOARD_TYPE_PIN_0 < HB_PIN_MAX_NUMS
        && BOARD_TYPE_PIN_1 > 0 && BOARD_TYPE_PIN_1 < HB_PIN_MAX_NUMS
        && BOARD_TYPE_PIN_2 == 0) {
        board_type = hb_board_type_get_by_pin(2);
    } else if (BOARD_TYPE_PIN_0 > 0 && BOARD_TYPE_PIN_0 < HB_PIN_MAX_NUMS
        && BOARD_TYPE_PIN_1 > 0 && BOARD_TYPE_PIN_1 < HB_PIN_MAX_NUMS
        && BOARD_TYPE_PIN_2 > 0 && BOARD_TYPE_PIN_2 < HB_PIN_MAX_NUMS) {
        board_type = hb_board_type_get_by_pin(3);
    } else {
        board_type = hb_board_type_get();
    }
	name = x3_image_get_dtb(board_type,
		(struct hb_kernel_hdr *)second_addr, second_data, second_len);
	DEBUG_LOG("dtb_name = %s\n", name);

	return 0;
}

#if !defined(CONFIG_SPL_BUILD)
/**
 * android_print_contents - prints out the contents of the Android format image
 * @hdr: pointer to the Android format image header
 *
 * android_print_contents() formats a multi line Android image contents
 * description.
 * The routine prints out Android image properties
 *
 * returns:
 *     no returned results
 */
void android_print_contents(const struct andr_img_hdr *hdr)
{
	const char * const p = IMAGE_INDENT_STRING;
	/* os_version = ver << 11 | lvl */
	u32 os_ver = hdr->os_version >> 11;
	u32 os_lvl = hdr->os_version & ((1U << 11) - 1);

	printf("%skernel size:      %x\n", p, hdr->kernel_size);
	printf("%skernel address:   %x\n", p, hdr->kernel_addr);
	printf("%sramdisk size:     %x\n", p, hdr->ramdisk_size);
	printf("%sramdisk addrress: %x\n", p, hdr->ramdisk_addr);
	printf("%ssecond size:      %x\n", p, hdr->second_size);
	printf("%ssecond address:   %x\n", p, hdr->second_addr);
	printf("%stags address:     %x\n", p, hdr->tags_addr);
	printf("%spage size:        %x\n", p, hdr->page_size);
	/* ver = A << 14 | B << 7 | C         (7 bits for each of A, B, C)
	 * lvl = ((Y - 2000) & 127) << 4 | M  (7 bits for Y, 4 bits for M) */
	printf("%sos_version:       %x (ver: %u.%u.%u, level: %u.%u)\n",
	       p, hdr->os_version,
	       (os_ver >> 7) & 0x7F, (os_ver >> 14) & 0x7F, os_ver & 0x7F,
	       (os_lvl >> 4) + 2000, os_lvl & 0x0F);
	printf("%sname:             %s\n", p, hdr->name);
	printf("%scmdline:          %s\n", p, hdr->cmdline);
}
#endif
