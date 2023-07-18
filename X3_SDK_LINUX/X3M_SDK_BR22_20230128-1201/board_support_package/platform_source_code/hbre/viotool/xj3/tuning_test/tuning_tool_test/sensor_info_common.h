/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef UTILITY_SENSOR_INC_SENSOR_INFO_COMMON_H_
#define UTILITY_SENSOR_INC_SENSOR_INFO_COMMON_H_

#ifdef __cplusplus
extern "C" {
#endif

#define CAMERA_SENSOR_NAME  20


typedef struct sensor_data {
	uint32_t  turning_type;  //  1:imx290 2: ar0233
	uint32_t  step_gain;
	uint32_t  again_prec;
	uint32_t  dgain_prec;
	uint32_t  conversion;
	uint32_t  VMAX;
	uint32_t  HMAX;
	uint32_t  FSC_DOL2;
	uint32_t  FSC_DOL3;
	uint32_t  RHS1;
	uint32_t  RHS2;
	uint32_t  lane;
	uint32_t  clk;
	uint32_t  fps;
	uint32_t  gain_max;
	uint32_t  lines_per_second;
	uint32_t  analog_gain_max;
	uint32_t  digital_gain_max;
	uint32_t  exposure_time_max;
	uint32_t  exposure_time_min;
	uint32_t  exposure_time_long_max;
	uint32_t  active_width;
	uint32_t  active_height;
}sensor_data_t;

/* line use y = ratio * x + offset;
 * input param:
 * ratio(0,1) : 0: -1, 1: 1
 * offset:
 * max:
 */

typedef struct ctrlp_s {
	int32_t ratio;
	uint32_t offset;
	uint32_t max;
} ctrlp_t;

/*
 * distinguish between dgain and again
 * note1: a sensor could only have again or dgain
 * note2: some sensor again/dgain in the same register, could only use again
 *        eg. imx327
 * note3: some sensor the again is stepped, could noly use again.
 * note4: again/dgain could have multiple registers,
 * note5: again [0,255], actual = 2^(again/32)
 * note6: dgain [0,255], actual = 2^(dgain/32)
 * note7: dol2/dol3/dol4 used the same again/dgain
 */

typedef struct dol3_s {
	uint32_t param_hold;
	uint32_t param_hold_length;
	ctrlp_t  line_p[3];
	uint32_t s_line;
	uint32_t s_line_length;
	uint32_t m_line;
	uint32_t m_line_length;
	uint32_t l_line;
	uint32_t l_line_length;
	uint32_t again_control_num;
	uint32_t again_control[4];
	uint32_t again_control_length[4];
	uint32_t dgain_control_num;
	uint32_t dgain_control[4];
	uint32_t dgain_control_length[4];
	uint32_t *again_lut;
	uint32_t *dgain_lut;
} dol3_t;

typedef struct dol2_s {
	uint32_t param_hold;
	uint32_t param_hold_length;
	ctrlp_t  line_p[2];
	uint32_t s_line;
	uint32_t s_line_length;
	uint32_t m_line;
	uint32_t m_line_length;
	uint32_t again_control_num;
	uint32_t again_control[4];
	uint32_t again_control_length[4];
	uint32_t dgain_control_num;
	uint32_t dgain_control[4];
	uint32_t dgain_control_length[4];
	uint32_t *again_lut;
	uint32_t *dgain_lut;
}dol2_t;
typedef struct normal_s {
	uint32_t param_hold;
	uint32_t param_hold_length;
	ctrlp_t  line_p;
	uint32_t s_line;
	uint32_t s_line_length;
	uint32_t again_control_num;
	uint32_t again_control[4];
	uint32_t again_control_length[4];
	uint32_t dgain_control_num;
	uint32_t dgain_control[4];
	uint32_t dgain_control_length[4];
	uint32_t *again_lut;
	uint32_t *dgain_lut;
}normal_t;
typedef struct pwl_s {
	uint32_t param_hold;
	uint32_t param_hold_length;
	ctrlp_t  line_p;
	uint32_t line;
	uint32_t line_length;
	uint32_t again_control_num;
	uint32_t again_control[4];
	uint32_t again_control_length[4];
	uint32_t dgain_control_num;
	uint32_t dgain_control[4];
	uint32_t dgain_control_length[4];
	uint32_t *again_lut;
	uint32_t *dgain_lut;
}pwl_t;

typedef struct stream_ctrl_s {
	uint32_t stream_on[10];
	uint32_t stream_off[10];
	uint32_t data_length;
}stream_ctrl_t;

typedef struct sensor_awb_ctrl_s {
	uint32_t rgain_addr[4];
	uint32_t rgain_length[4];
	uint32_t bgain_addr[4];
	uint32_t bgain_length[4];
	uint32_t grgain_addr[4];
	uint32_t grgain_length[4];
	uint32_t gbgain_addr[4];
	uint32_t gbgain_length[4];
	uint32_t rb_prec;
} sensor_awb_ctrl_t;

typedef struct sensor_turning_data {
	uint32_t  port;
	char      sensor_name[CAMERA_SENSOR_NAME];
	uint32_t  sensor_addr;
    uint32_t  bus_num;
	uint32_t  bus_type;
	uint32_t  reg_width;
	uint32_t  chip_id;
	uint32_t  mode;
	uint32_t  cs;
	uint32_t  spi_mode;
	uint32_t  spi_speed;
	normal_t normal;
	dol2_t   dol2;
	dol3_t   dol3;
	pwl_t    pwl;  // ar0233
	sensor_awb_ctrl_t sensor_awb;
	stream_ctrl_t stream_ctrl;
	sensor_data_t sensor_data;
}sensor_turning_data_t;

#ifdef __cplusplus
}
#endif

#endif // UTILITY_SENSOR_INC_SENSOR_EFFECT_COMMON_H_

