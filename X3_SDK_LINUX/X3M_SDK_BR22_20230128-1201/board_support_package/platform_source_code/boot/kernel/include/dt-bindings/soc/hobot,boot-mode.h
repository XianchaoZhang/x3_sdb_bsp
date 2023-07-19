/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __HBOOT_BOOT_MODE_H
#define __HBOOT_BOOT_MODE_H

/* high 24 bits is tag, low 8 bits is type */
#define REBOOT_FLAG		0x5242C300
/* normal boot */
#define BOOT_NORMAL		(REBOOT_FLAG + 0)
/* enter bootloader dfu mode */
#define BOOT_BL_DOWNLOAD	(REBOOT_FLAG + 1)
/* enter recovery */
#define BOOT_RECOVERY		(REBOOT_FLAG + 3)
/* enter fastboot mode */
#define BOOT_FASTBOOT		(REBOOT_FLAG + 9)
/* enter usb mass storage mode */
#define BOOT_UMS		(REBOOT_FLAG + 11)
/* enter device firmware upgrade mode */
#define BOOT_DFU		(REBOOT_FLAG + 12)
/* enter usb fimrware upgrade mode */
#define BOOT_UFU		(REBOOT_FLAG + 13)
/* enter eye diagram debug mdoe */
#define BOOT_EYE_DIAG		(REBOOT_FLAG + 14)

#endif /* __HBOOT_BOOT_MODE_H */
