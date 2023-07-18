/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#include <cstdint>
#include <fstream>
#include <iostream>
#include <sstream>
#include <memory>
#include <string>
#include "vio/vio_log.h"
#include "vio/vio_cfg.h"
int running = 0;

bool VioConfig::LoadConfig()
{
	std::ifstream ifs(path_);
	if (!ifs.is_open()) {
		std::cout << "Open config file " << path_ << " failed" << std::
		    endl;
		return false;
	}
	std::cout << "Open config file " << path_ << " success" << std::endl;
	std::stringstream ss;
	ss << ifs.rdbuf();
	ifs.close();
	std::string content = ss.str();
	Json::Value value;
	Json::CharReaderBuilder builder;
	builder["collectComments"] = false;
	JSONCPP_STRING error;
	std::shared_ptr < Json::CharReader > reader(builder.newCharReader());
	try {
		bool ret = reader->parse(content.c_str(),
					 content.c_str() + content.size(),
					 &json_, &error);
		if (ret) {
			std::cout << "Open config file1 " << path_ << std::endl;
			if (json_)
				return true;
		} else {
			std::cout << "Open config file2 " << path_ << std::endl;
			return false;
		}
	}
	catch(std::exception & e) {
		std::cout << "Open config file3 " << path_ << std::endl;
		return false;
	}
}

int VioConfig::GetIntValue(const std::string & key) const
{
	std::lock_guard < std::mutex > lk(mutex_);
	if (json_[key].empty()) {
		std::cout << "Can not find key: " << key << std::endl;
		return -1;
	}

	return json_[key].asInt();
}

std::string VioConfig::GetStringValue(const std::string & key) const
{
	std::lock_guard < std::mutex > lk(mutex_);
	if (json_[key].empty()) {
		std::cout << "Can not find key: " << key << std::endl;
		return "";
	}

	return json_[key].asString();
}

Json::Value VioConfig::GetJson() const
{
	return this->json_;
}

bool VioConfig::ParserConfig()
{
	int pym_num;
	int grp_start_index, grp_max_num;
	int i, j, k, m, n;

	vio_cfg_.need_cam = GetIntValue("need_cam");
	vio_cfg_.cam_num = GetIntValue("cam_num");
	if (vio_cfg_.cam_num > MAX_CAM_NUM) {
		pr_err("cam num exceed max\n");
		return -1;
	}
	vio_cfg_.need_vo = GetIntValue("need_vo");
	vio_cfg_.vps_dump = GetIntValue("vps_dump");
	vio_cfg_.need_rtsp = GetIntValue("need_rtsp");
	for (n = 0; n < vio_cfg_.cam_num; n++) {
		/* 1. sensor config */
		std::string cam_name = "cam" + std::to_string(n);
		vio_cfg_.sensor_id[n] =
		    json_[cam_name]["sensor"]["sensor_id"].asInt();
		vio_cfg_.sensor_port[n] =
		    json_[cam_name]["sensor"]["sensor_port"].asInt();
		if (n < MAX_MIPIID_NUM) {
			vio_cfg_.mipi_idx[n] =
			    json_[cam_name]["sensor"]["mipi_idx"].asInt();
		}
		vio_cfg_.i2c_bus[n] =
		    json_[cam_name]["sensor"]["i2c_bus"].asInt();
		vio_cfg_.serdes_index[n] =
		    json_[cam_name]["sensor"]["serdes_index"].asInt();
		vio_cfg_.serdes_port[n] =
		    json_[cam_name]["sensor"]["serdes_port"].asInt();
		vio_cfg_.temper_mode[n] =
		    json_[cam_name]["isp"]["temper_mode"].asInt();
		vio_cfg_.isp_out_buf_num[n] =
		    json_[cam_name]["isp"]["isp_out_buf_num"].asInt();
		vio_cfg_.gpio_num[n] =
		    json_[cam_name]["sensor"]["gpio_num"].asInt();
		vio_cfg_.gpio_pin[n] =
		    json_[cam_name]["sensor"]["gpio_pin"].asInt();
		vio_cfg_.gpio_level[n] =
		    json_[cam_name]["sensor"]["gpio_level"].asInt();
		vio_cfg_.grp_num[n] = json_[cam_name]["grp_num"].asInt();
		if (vio_cfg_.grp_num[n] > MAX_GRP_NUM) {
			pr_err("exceed max grp num\n");
		}
		if (vio_cfg_.need_cam == 1) {
			grp_max_num = n + 1;
		} else {
			grp_start_index = 0;
			grp_max_num = vio_cfg_.grp_num[n];
			// HOBOT_CHECK(vio_cfg_.cam_num <= 1);
		}
		/* 2. group config, a sensor use a group if in a process */
		for (i = n; i < grp_max_num; i++) {
			pym_num = 0;
			std::string grp_name = "grp0";
			vio_cfg_.fb_width[i] =
			    json_[cam_name][grp_name]["fb_width"].asInt();
			vio_cfg_.fb_height[i] =
			    json_[cam_name][grp_name]["fb_height"].asInt();
			vio_cfg_.fb_buf_num[i] =
			    json_[cam_name][grp_name]["fb_buf_num"].asInt();
			vio_cfg_.vin_fd[i] =
			    json_[cam_name][grp_name]["vin_fd"].asInt();
			vio_cfg_.vin_vps_mode[i] =
			    json_[cam_name][grp_name]["vin_vps_mode"].asInt();
			vio_cfg_.need_clk[i] =
			    json_[cam_name][grp_name]["need_clk"].asInt();
			vio_cfg_.need_md[i] =
			    json_[cam_name][grp_name]["need_md"].asInt();
			vio_cfg_.need_chnfd[i] =
			    json_[cam_name][grp_name]["need_chnfd"].asInt();
			vio_cfg_.need_dis[i] =
			    json_[cam_name][grp_name]["need_dis"].asInt();
			vio_cfg_.need_gdc[i] =
			    json_[cam_name][grp_name]["need_gdc"].asInt();
			vio_cfg_.grp_rotate[i] =
			    json_[cam_name][grp_name]["grp_rotate"].asInt();
			vio_cfg_.dol2_vc_num[i] =
			    json_[cam_name][grp_name]["dol2_vc_num"].asInt();
			vio_cfg_.grp_frame_depth[i] =
			    json_[cam_name][grp_name]["grp_frame_depth"].
			    asInt();
			/* 3. channel config */
			vio_cfg_.chn_num[i] = json_[cam_name][grp_name]
			    ["chn_num"].asInt();
			if (vio_cfg_.chn_num[i] > MAX_CHN_NUM) {
				pr_err("chn num exceed max\n");
			}
			for (j = 0; j < vio_cfg_.chn_num[i]; j++) {
				std::string chn_name =
				    "chn" + std::to_string(j);
				vio_cfg_.ipu_chn_en[i][j] =
				    json_[cam_name][grp_name][chn_name]
				    ["ipu_chn_en"].asInt();
				vio_cfg_.pym_chn_en[i][j] =
				    json_[cam_name][grp_name][chn_name]
				    ["pym_chn_en"].asInt();
				vio_cfg_.scale_en[i][j] =
				    json_[cam_name][grp_name][chn_name]
				    ["scale_en"].asInt();
				vio_cfg_.width[i][j] =
				    json_[cam_name][grp_name][chn_name]
				    ["width"].asInt();
				vio_cfg_.height[i][j] =
				    json_[cam_name][grp_name][chn_name]
				    ["height"].asInt();
				vio_cfg_.frame_depth[i][j] =
				    json_[cam_name][grp_name][chn_name]
				    ["frame_depth"].asInt();
				auto item = json_[cam_name][grp_name][chn_name];
				if (item["crop_chn_en"].isInt()) {
					vio_cfg_.crop_en[i][j] =
					    item["crop_chn_en"].asInt();
					vio_cfg_.crop_width[i][j] =
					    item["crop_width"].asInt();
					vio_cfg_.crop_height[i][j] =
					    item["crop_height"].asInt();
					vio_cfg_.crop_x[i][j] =
					    item["crop_x"].asInt();
					vio_cfg_.crop_y[i][j] =
					    item["crop_y"].asInt();
				} else {
					vio_cfg_.crop_en[i][j] = 0;
				}
				if (vio_cfg_.pym_chn_en[i][j] == 1) {
					/* 4. pym config */
					/* 4.1 pym ctrl config */
					std::string chn_name =
					    "chn" + std::to_string(j);
					vio_cfg_.pym_cfg[i][j].frame_id =
					    json_[cam_name][grp_name][chn_name]
					    ["pym"]
					    ["pym_ctrl_config"]["frame_id"].
					    asUInt();
					vio_cfg_.pym_cfg[i][j].ds_uv_bypass =
					    json_[cam_name][grp_name][chn_name]
					    ["pym"]
					    ["pym_ctrl_config"]["ds_uv_bypass"].
					    asUInt();
					vio_cfg_.pym_cfg[i][j].ds_layer_en =
					    (uint16_t)
					    json_[cam_name][grp_name][chn_name]
					    ["pym"]
					    ["pym_ctrl_config"]["ds_layer_en"].
					    asUInt();
					vio_cfg_.pym_cfg[i][j].us_layer_en =
					    (uint8_t)
					    json_[cam_name][grp_name][chn_name]
					    ["pym"]
					    ["pym_ctrl_config"]["us_layer_en"].
					    asUInt();
					vio_cfg_.pym_cfg[i][j].us_uv_bypass =
					    (uint8_t)
					    json_[cam_name][grp_name][chn_name]
					    ["pym"]
					    ["pym_ctrl_config"]["us_uv_bypass"].
					    asUInt();
					vio_cfg_.pym_cfg[i][j].frameDepth =
					    json_[cam_name][grp_name][chn_name]
					    ["pym"]
					    ["pym_ctrl_config"]["frame_depth"].
					    asUInt();
					vio_cfg_.pym_cfg[i][j].timeout =
					    json_[cam_name][grp_name][chn_name]
					    ["pym"]
					    ["pym_ctrl_config"]["timeout"].
					    asInt();
					/* 4.2 pym downscale config */
					for (k = 0; k < MAX_PYM_DS_NUM; k++) {
						if (k % 4 == 0)
							continue;
						std::string factor_name =
						    "factor_" +
						    std::to_string(k);
						std::string roi_x_name =
						    "roi_x_" +
						    std::to_string(k);
						std::string roi_y_name =
						    "roi_y_" +
						    std::to_string(k);
						std::string roi_w_name =
						    "roi_w_" +
						    std::to_string(k);
						std::string roi_h_name =
						    "roi_h_" +
						    std::to_string(k);
						vio_cfg_.pym_cfg[i][j].
						    ds_info[k].factor =
						    (uint8_t)
						    json_[cam_name][grp_name]
						    [chn_name]["pym"]
						    ["pym_ds_config"]
						    [factor_name].asUInt();
						vio_cfg_.pym_cfg[i][j].
						    ds_info[k].roi_x =
						    (uint16_t)
						    json_[cam_name][grp_name]
						    [chn_name]["pym"]
						    ["pym_ds_config"]
						    [roi_x_name].asUInt();
						vio_cfg_.pym_cfg[i][j].
						    ds_info[k].roi_y =
						    (uint16_t)
						    json_[cam_name][grp_name]
						    [chn_name]["pym"]
						    ["pym_ds_config"]
						    [roi_y_name].asUInt();
						vio_cfg_.pym_cfg[i][j].
						    ds_info[k].roi_width =
						    (uint16_t)
						    json_[cam_name][grp_name]
						    [chn_name]["pym"]
						    ["pym_ds_config"]
						    [roi_w_name].asUInt();
						vio_cfg_.pym_cfg[i][j].
						    ds_info[k].roi_height =
						    (uint16_t)
						    json_[cam_name][grp_name]
						    [chn_name]["pym"]
						    ["pym_ds_config"]
						    [roi_h_name].asUInt();
					}
					/* 4.3 pym upscale config */
					for (m = 0; m < MAX_PYM_US_NUM; m++) {
						std::string factor_name =
						    "factor_" +
						    std::to_string(m);
						std::string roi_x_name =
						    "roi_x_" +
						    std::to_string(m);
						std::string roi_y_name =
						    "roi_y_" +
						    std::to_string(m);
						std::string roi_w_name =
						    "roi_w_" +
						    std::to_string(m);
						std::string roi_h_name =
						    "roi_h_" +
						    std::to_string(m);
						vio_cfg_.pym_cfg[i][j].
						    us_info[m].factor =
						    (uint8_t)
						    json_[cam_name][grp_name]
						    [chn_name]["pym"]
						    ["pym_us_config"]
						    [factor_name].asUInt();
						vio_cfg_.pym_cfg[i][j].
						    us_info[m].roi_x =
						    (uint16_t)
						    json_[cam_name][grp_name]
						    [chn_name]["pym"]
						    ["pym_us_config"]
						    [roi_x_name].asUInt();
						vio_cfg_.pym_cfg[i][j].
						    us_info[m].roi_y =
						    (uint16_t)
						    json_[cam_name][grp_name]
						    [chn_name]["pym"]
						    ["pym_us_config"]
						    [roi_y_name].asUInt();
						vio_cfg_.pym_cfg[i][j].
						    us_info[m].roi_width =
						    (uint16_t)
						    json_[cam_name][grp_name]
						    [chn_name]["pym"]
						    ["pym_us_config"]
						    [roi_w_name].asUInt();
						vio_cfg_.pym_cfg[i][j].
						    us_info[m].roi_height =
						    (uint16_t)
						    json_[cam_name][grp_name]
						    [chn_name]["pym"]
						    ["pym_us_config"]
						    [roi_h_name].asUInt();
					}
					/* 4.4 calculate pym num */
					pym_num++;
				}	// end pym channel config
			}	// end all channel config
		}		// end group config
	}			// end cam config

	// PrintConfig();
	return true;
}

bool VioConfig::PrintConfig()
{
	int i, j, k, m, n;
	int grp_start_index, grp_max_num = 0;
	/* 1. print sensor config */
	std::cout << "*********** iot vio config start ***********" << std::
	    endl;
	std::cout << "cam_num: " << vio_cfg_.cam_num << std::endl;
	std::cout << "vps_dump: " << vio_cfg_.vps_dump << std::endl;
	std::cout << "need_vo: " << vio_cfg_.need_vo << std::endl;
	for (n = 0; n < vio_cfg_.cam_num; n++) {
		std::
		    cout << "#########cam:" << n << " cam config start#########"
		    << std::endl;
		std::
		    cout << "cam_index: " << n << " " << "sensor_id: " <<
		    vio_cfg_.sensor_id[n] << std::endl;
		std::
		    cout << "cam_index: " << n << " " << "sensor_port: " <<
		    vio_cfg_.sensor_port[n] << std::endl;
		if (n < MAX_MIPIID_NUM) {
			std::cout << "cam_index: " << n << " " << "mipi_idx: "
			    << vio_cfg_.mipi_idx[n] << std::endl;
		}
		std::cout << "cam_index: " << n << " " << "i2c_bus: "
		    << vio_cfg_.i2c_bus[n] << std::endl;
		std::cout << "cam_index: " << n << " " << "serdes_index: "
		    << vio_cfg_.serdes_index[n] << std::endl;
		std::cout << "cam_index: " << n << " " << "serdes_port: "
		    << vio_cfg_.serdes_port[n] << std::endl;
		std::cout << "cam_index: " << n << " " << "temper_mode: "
		    << vio_cfg_.temper_mode[n] << std::endl;
		std::cout << "cam_index: " << n << " " << "isp_out_buf_num: "
		    << vio_cfg_.isp_out_buf_num[n] << std::endl;
		std::cout << "cam_index: " << n << " " << "grp_num: "
		    << vio_cfg_.grp_num[n] << std::endl;
		std::cout << "cam_index: " << n << " " << "gpio_num: "
		    << vio_cfg_.gpio_num[n] << std::endl;
		std::cout << "cam_index: " << n << " " << "gpio_pin: "
		    << vio_cfg_.gpio_pin[n] << std::endl;
		/* 2. print group config */
		if (vio_cfg_.need_cam == 1) {
			grp_start_index = n;
			grp_max_num = grp_start_index + 1;
		} else {
			grp_start_index = 0;
			grp_max_num = vio_cfg_.grp_num[n];
		}
		for (i = grp_start_index; i < grp_max_num; i++) {
			std::
			    cout << "=========grp:" << i <<
			    " group config start==========" << std::endl;
			std::
			    cout << "grp_index: " << i << " " << "fb_width: " <<
			    vio_cfg_.fb_width[i] << std::endl;
			std::
			    cout << "grp_index: " << i << " " << "fb_height: "
			    << vio_cfg_.fb_height[i] << std::endl;
			std::
			    cout << "grp_index: " << i << " " << "fb_buf_num: "
			    << vio_cfg_.fb_buf_num[i] << std::endl;
			std::
			    cout << "grp_index: " << i << " " << "vin_fd: " <<
			    vio_cfg_.vin_fd[i] << std::endl;
			std::
			    cout << "grp_index: " << i << " " <<
			    "vin_vps_mode: " << vio_cfg_.
			    vin_vps_mode[i] << std::endl;
			std::
			    cout << "grp_index: " << i << " " << "need_clk: " <<
			    vio_cfg_.need_clk[i] << std::endl;
			std::
			    cout << "grp_index: " << i << " " << "need_md: " <<
			    vio_cfg_.need_md[i] << std::endl;
			std::
			    cout << "grp_index: " << i << " " << "need_chnfd: "
			    << vio_cfg_.need_chnfd[i] << std::endl;
			std::
			    cout << "grp_index: " << i << " " << "need_dis: " <<
			    vio_cfg_.need_dis[i] << std::endl;
			std::
			    cout << "grp_index: " << i << " " << "need_gdc: " <<
			    vio_cfg_.need_gdc[i] << std::endl;
			std::
			    cout << "grp_index: " << i << " " << "grp_rotate: "
			    << vio_cfg_.grp_rotate[i] << std::endl;
			std::
			    cout << "grp_index: " << i << " " << "dol2_vc_num: "
			    << vio_cfg_.dol2_vc_num[i] << std::endl;
			std::
			    cout << "grp_index: " << i << " " <<
			    "grp_frame_depth: " << vio_cfg_.
			    grp_frame_depth[i] << std::endl;
			std::
			    cout << "grp_index: " << i << " " << "chn_num: " <<
			    vio_cfg_.chn_num[i] << std::endl;
			/* 3. print channel config */
			for (j = 0; j < vio_cfg_.chn_num[i]; j++) {
				std::
				    cout << "chn_index: " << j << " " <<
				    "ipu_chn_en: " << vio_cfg_.
				    ipu_chn_en[i][j] << std::endl;
				std::
				    cout << "chn_index: " << j << " " <<
				    "pym_chn_en: " << vio_cfg_.
				    pym_chn_en[i][j] << std::endl;
				std::
				    cout << "chn_index: " << j << " " <<
				    "scale_en: " << vio_cfg_.
				    scale_en[i][j] << std::endl;
				std::
				    cout << "chn_index: " << j << " " <<
				    "width: " << vio_cfg_.
				    width[i][j] << std::endl;
				std::
				    cout << "chn_index: " << j << " " <<
				    "height: " << vio_cfg_.
				    height[i][j] << std::endl;
				std::
				    cout << "chn_index: " << j << " " <<
				    "frame_depth: " << vio_cfg_.
				    frame_depth[i][j] << std::endl;
				std::
				    cout << "chn_index: " << j << " " <<
				    "crop_en: " << vio_cfg_.
				    crop_en[i][j] << std::endl;
				std::
				    cout << "chn_index: " << j << " " <<
				    "crop_width: " << vio_cfg_.
				    crop_width[i][j] << std::endl;
				std::
				    cout << "chn_index: " << j << " " <<
				    "crop_height: " << vio_cfg_.
				    crop_height[i][j] << std::endl;
				std::
				    cout << "chn_index: " << j << " " <<
				    "crop_x: " << vio_cfg_.
				    crop_x[i][j] << std::endl;
				std::
				    cout << "chn_index: " << j << " " <<
				    "crop_y: " << vio_cfg_.
				    crop_y[i][j] << std::endl;
				if (vio_cfg_.pym_chn_en[i][j] == 1) {
					std::
					    cout << "--------chn:" << j <<
					    " pym config start---------" <<
					    std::endl;
					std::
					    cout << "chn_index: " << j << " " <<
					    "frame_id: " << vio_cfg_.
					    pym_cfg[i][j].frame_id << std::endl;
					std::
					    cout << "chn_index: " << j << " " <<
					    "ds_layer_en: " << vio_cfg_.
					    pym_cfg[i][j].
					    ds_layer_en << std::endl;
					std::
					    cout << "chn_index: " << j << " " <<
					    "ds_uv_bypass: " << vio_cfg_.
					    pym_cfg[i][j].
					    ds_uv_bypass << std::endl;
					std::
					    cout << "chn_index: " << j << " " <<
					    "us_layer_en: " << static_cast <
					    int >(vio_cfg_.
						  pym_cfg[i][j]. us_layer_en) <<
					    std::endl;
					std::
					    cout << "chn_index: " << j << " " <<
					    "us_uv_bypass: " << static_cast <
					    int >(vio_cfg_.
						  pym_cfg[i][j]. us_uv_bypass)
					    << std::endl;
					std::
					    cout << "chn_index: " << j << " " <<
					    "frameDepth: " << vio_cfg_.
					    pym_cfg[i][j].
					    frameDepth << std::endl;
					std::
					    cout << "chn_index: " << j << " " <<
					    "timeout: " << vio_cfg_.
					    pym_cfg[i][j].timeout << std::endl;

					for (k = 0; k < MAX_PYM_DS_NUM; k++) {
						if (k % 4 == 0)
							continue;
						std::
						    cout << "ds_pym_layer: " <<
						    k << " " << "ds roi_x: " <<
						    vio_cfg_.pym_cfg[i][j].
						    ds_info[k].
						    roi_x << std::endl;
						std::
						    cout << "ds_pym_layer: " <<
						    k << " " << "ds roi_y: " <<
						    vio_cfg_.pym_cfg[i][j].
						    ds_info[k].
						    roi_y << std::endl;
						std::
						    cout << "ds_pym_layer: " <<
						    k << " " << "ds roi_width: "
						    << vio_cfg_.pym_cfg[i][j].
						    ds_info[k].
						    roi_width << std::endl;
						std::
						    cout << "ds_pym_layer: " <<
						    k << " " <<
						    "ds roi_height: " <<
						    vio_cfg_.pym_cfg[i][j].
						    ds_info[k].
						    roi_height << std::endl;
						std::
						    cout << "ds_pym_layer: " <<
						    k << " " << "ds factor: " <<
						    static_cast <
						    int >(vio_cfg_.
							  pym_cfg[i][j].
							  ds_info[k].
							  factor) << std::endl;
					}
					/* 4.3 pym upscale config */
					for (m = 0; m < MAX_PYM_US_NUM; m++) {
						std::
						    cout << "us_pym_layer: " <<
						    m << " " << "us roi_x: " <<
						    vio_cfg_.pym_cfg[i][j].
						    us_info[m].
						    roi_x << std::endl;
						std::
						    cout << "us_pym_layer: " <<
						    m << " " << "us roi_y: " <<
						    vio_cfg_.pym_cfg[i][j].
						    us_info[m].
						    roi_y << std::endl;
						std::
						    cout << "us_pym_layer: " <<
						    m << " " << "us roi_width: "
						    << vio_cfg_.pym_cfg[i][j].
						    us_info[m].
						    roi_width << std::endl;
						std::
						    cout << "us_pym_layer: " <<
						    m << " " <<
						    "us roi_height: " <<
						    vio_cfg_.pym_cfg[i][j].
						    us_info[m].
						    roi_height << std::endl;
						std::
						    cout << "us_pym_layer: " <<
						    m << " " << "us factor: " <<
						    static_cast <
						    int >(vio_cfg_.
							  pym_cfg[i][j].
							  us_info[m].
							  factor) << std::endl;
					}
					std::
					    cout << "---------chn:" << j <<
					    " pym config end----------" << std::
					    endl;
				}
			}
			std::
			    cout << "=========grp:" << i <<
			    " group config end==========" << std::endl;
		}
		std::
		    cout << "#########cam:" << n << " cam config end#########"
		    << std::endl;
	}
	std::cout << "*********** iot vio config end ***********" << std::endl;

	return true;
}
