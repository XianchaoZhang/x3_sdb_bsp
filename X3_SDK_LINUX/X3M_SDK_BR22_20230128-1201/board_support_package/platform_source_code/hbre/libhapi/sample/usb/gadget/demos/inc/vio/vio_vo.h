/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef INCLUDE_VIO_VO_H_
#define INCLUDE_VIO_VO_H_
#include <stdlib.h>
#include "hb_vio_interface.h"
#include "hb_common_vot.h"

int hb_vo_init();
int hb_vo_send_frame(address_info_t addr);
int hb_vo_deinit();
#endif // INCLUDE_VIO_VO_H_