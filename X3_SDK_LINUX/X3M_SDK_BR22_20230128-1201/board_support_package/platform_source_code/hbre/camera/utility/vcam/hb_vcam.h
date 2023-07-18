/*************************************************************************
 *                     COPYRIGHT NOTICE
 *            Copyright 2016-2020 Horizon Robotics, Inc.
 *                   All rights reserved.
 *************************************************************************/
#ifndef UTILITY_HB_VCAM_H_
#define UTILITY_HB_VCAM_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "hb_cam_interface.h"

/* for msg type */
enum vcam_cmd {
	CMD_VCAM_INIT = 1,
	CMD_VCAM_START,
	CMD_VCAM_NEXT_REQUST,
	CMD_VCAM_STOP,
	CMD_VCAM_DEINIT,
	CMD_VCAM_MAX
};

/* for image format */
#define VCAM_YUV_420_8		0x18
#define VCAM_YUV_420_10		0x19
#define VCAM_YUV_422_8		0x1E
#define VCAM_YUV_422_10		0x1F
#define VCAM_RAW_8			0x2A
#define VCAM_RAW_10			0x2B
#define VCAM_RAW_12			0x2C
#define VCAM_RAW_14			0x2D
/* for init info, may be TBD */
typedef struct vcam_init_info_s {
	int ipu_enable;
	int width;
	int height;
	int stride;
	int format;
} vcam_init_info_t;

/* for vcam image info */
typedef struct vcam_img_info_s {
	int width;
	int height;
	int stride;
	int format;
} vcam_img_info_t;

/* for vcam slot info */
typedef struct vcam_slot_info_s {
	int						slot_id;
	int						cam_id;
	int						frame_id;
	int64_t					timestamp;
	vcam_img_info_t			img_info;
} vcam_slot_info_t;

/*
 * for vcam group info
 * group_size = slot_size * slot_num;
 * every_group_addr = base + g_id * group_size;
*/
typedef struct vcam_group_info_s {
	int			g_id;		// group id
	uint64_t	base;		// first group paddr
	int			slot_size;  // a slot size
	int			slot_num;   // a group have slot_num slot
	int			flag;		// 0 free 1 busy
} vcam_group_info_t;

/* for vcam msg info */
typedef struct hb_vcam_msg_s {
	int					info_type;
	vcam_group_info_t	group_info;
	vcam_slot_info_t	slot_info;
} hb_vcam_msg_t;

/* vcam memory info */
struct mem_info_t {
	int size;
	uint64_t base;
};

int hb_vcam_init();
int hb_vcam_start();
int hb_vcam_stop();
int hb_vcam_deinit();
int hb_vcam_next_group();
int hb_vcam_get_img(cam_img_info_t *cam_img_info);
int hb_vcam_free_img(cam_img_info_t *cam_img_info);
int hb_vcam_clean(cam_img_info_t *cam_img_info);

#ifdef __cplusplus
}
#endif

#endif  // UTILITY_HB_VCAM_H_
