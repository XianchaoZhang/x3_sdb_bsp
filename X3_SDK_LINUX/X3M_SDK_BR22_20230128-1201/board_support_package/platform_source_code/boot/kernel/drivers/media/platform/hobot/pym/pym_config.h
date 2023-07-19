/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef __HOBOT_PYM_CONFIG_H__
#define __HOBOT_PYM_CONFIG_H__

#define MAX_PYM_DS_COUNT        24
#define MAX_PYM_US_COUNT        6

typedef struct pym_scale_box_s {
	u8 factor;
	u16 roi_x;
	u16 roi_y;
	u16 roi_width;
	u16 roi_height;
	u16 tgt_width;
	u16 tgt_height;
} pym_scale_box_t;

enum ch_type {
	PYM_CH_DS,
	PYM_CH_US,
};

typedef struct pym_scale_ch_s {
	enum ch_type type;
	uint8_t ch;	/* ds:0~23, us,0~5 */
	pym_scale_box_t ch_scale;
} pym_scale_ch_t;

typedef struct pym_cfg_s {
	u8 img_scr;		//1: ipu mode 0: ddr mode
	u16 img_width;
	u16 img_height;
	u32 frame_id;
	u32 ds_uv_bypass;
	u16 ds_layer_en;
	u8 us_layer_en;
	u8 us_uv_bypass;
	u16 ddr_in_buf_num;
	u16 output_buf_num;
	int timeout;
	u32 cfg_index;
	uint32_t bind_to_ipu;
	int binding_chn_id;
	pym_scale_box_t stds_box[MAX_PYM_DS_COUNT];
	pym_scale_box_t stus_box[MAX_PYM_US_COUNT];
} pym_cfg_t;

#endif
