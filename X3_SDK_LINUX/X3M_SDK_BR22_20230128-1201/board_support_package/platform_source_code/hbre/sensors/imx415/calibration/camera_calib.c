#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <errno.h>

#include "acamera_command_define.h"

extern uint32_t get_dynamic_calibrations( ACameraCalibrations *c );
extern uint32_t get_static_calibrations( ACameraCalibrations *c );

calib_module_t camera_calibration = {
	.module = "imx415",
	.get_calib_dynamic = get_dynamic_calibrations,
	.get_calib_static = get_static_calibrations
};


