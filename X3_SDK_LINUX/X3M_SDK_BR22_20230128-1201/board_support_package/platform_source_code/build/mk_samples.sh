#!/bin/bash

set -e

######## setting utils_funcs ###################
source ./utils_funcs.sh

export HR_TOP_DIR=$(realpath $(cd $(dirname $0); pwd)/../)
export HR_LOCAL_DIR=$(realpath $(cd $(dirname $0); pwd))

# 选择板级配置
if [ "$1" = "lunch" ];then
    lunch_board_combo ${@:2}
    exit 0
fi

# check board config
check_board_config ${@:1}

export LD_LIBRARY_PATH=${TOOLCHAIN_LD_LIBRARY_PATH}

# 编译出来的镜像保存位置
export IMAGE_DEPLOY_DIR=${HR_IMAGE_DEPLOY_DIR}
[ ! -z $IMAGE_DEPLOY_DIR ] && [ ! -d $IMAGE_DEPLOY_DIR ] && mkdir $IMAGE_DEPLOY_DIR

samples_list=(get_isp_data get_sif_data sample_isp sample_lcd \
	sample_osd sample_video_codec sample_vot sample_vps \
	sample_vps_zoom sunrise_camera sample_usb_cam_4k60)

for sample_name in ${samples_list[@]}
do
	echo "************* Build ${sample_name} begin ****************"
	cd ${HR_TOP_DIR}/sample/${sample_name}
	make clean && make && make install
	mkdir -p ${IMAGE_DEPLOY_DIR}/sample/${sample_name}
	mv out/* ${IMAGE_DEPLOY_DIR}/sample/${sample_name}
	make clean
	echo "************* Build ${sample_name} end ******************"
done
