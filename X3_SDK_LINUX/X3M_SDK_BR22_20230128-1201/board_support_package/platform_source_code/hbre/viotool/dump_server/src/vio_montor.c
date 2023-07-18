#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <fcntl.h>
#include <signal.h>

#include "server_config.h"
#include "server_core.h"
#include "com_api.h"
#include "code_func.h"
#include "vio_func.h"
#include "send_func.h"
#include "common.h"
#include "hb_isp_api.h"

int vio_montor_start(const char *pathname, uint32_t gdc_rotation, uint32_t video, uint32_t stats)
{
	int ret = 0;
	uint32_t enable = 0;
	uint32_t pipe_line = 0;
	uint32_t channel = 0;

	if (dump_config_init(pathname) < 0) {
		vmon_err("dump_config_init failed! \n");
		return -1;
	}

	if (dump_server_core_start_services() < 0) {
		vmon_err("dump_server_core_start_services failed! \n");
		return -1;
	}
	if (video) {
		/*init vio*/
		ret = init_process(gdc_rotation);
		if (ret < 0) {
			vmon_err("dump_server_core_start_services failed! \n");
			goto err_vio;
		}

		/*init jepg*/
		jepg_config_info(&enable, &pipe_line, &channel);
		video_func_init(pipe_line, channel);
		if (enable == 1) {
			ret = jepg_start();
			if (ret < 0) {
				goto err_video;
			}
		}

		/*init send process*/
		ret = start_send_yuv_pic();
		if (ret < 0) {
			goto err_sendyuv;
		}
		/*init raw process*/
		ret = start_send_raw_pic();
		if (ret < 0) {
			goto err_sendraw;
		}
	}
	ret = HB_ISP_GetSetInit();

	if (stats) {
		/*init stats process */
		ret = start_send_stats_data();
		if (ret < 0) {
			goto err_sendstats;
		}
	}

	return ret;
err_sendstats:
	stop_send_raw_pic();
err_sendraw:
	stop_send_yuv_pic();
err_sendyuv:
	//jepg_func_deinit();
err_video:
	deinit_process();
err_vio:
	return ret;
}

//stop
void vio_montor_stop(void)
{
	stop_send_yuv_pic();
	stop_send_raw_pic();
	video_stop();
	jepg_stop();
	video_func_deinit();
	deinit_process();
	HB_ISP_GetSetExit();
	//dump_server_core_stop_services();
}
