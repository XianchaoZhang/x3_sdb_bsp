#!/bin/bash

set -e
######## setting utils_funcs ###################
source ./utils_funcs.sh

export HR_TOP_DIR=$(realpath $(cd $(dirname $0); pwd)/../)
export HR_LOCAL_DIR=$(realpath $(cd $(dirname $0); pwd))

if [ ! -d ${HR_TOP_DIR}/app ];then
	echo "App directory does not exist"
	exit -1
fi

# check board config
check_board_config ${@:1}

# 编译出来的镜像保存位置
export IMAGE_DEPLOY_DIR=${HR_IMAGE_DEPLOY_DIR}
[ ! -z ${IMAGE_DEPLOY_DIR} ] && [ ! -d $IMAGE_DEPLOY_DIR ] && mkdir $IMAGE_DEPLOY_DIR

rm -f ${IMAGE_OUT_DIR}/app.img

# 指定app分区的源文件目录
APP_DIR=${HR_TOP_DIR}/app

# 对于emmc的存储介质，支持 "ext4" 和 "squashfs" 两种文件系统格式
# 生成 "ext4" 格式的分区镜像
# 参数说明： -l 指定分区大小， -L 指定分区的label， 生成的镜像名， 源文件目录
make_ext4fs -l 268435456 -L app ${IMAGE_DEPLOY_DIR}/app.img ${APP_DIR}

# 生成 "squashfs" 格式的分区镜像，并且可以设置不通的压缩算法：lx4, xz等
# mksquashfs ${APP_DIR} ${IMAGE_DEPLOY_DIR}/app.img -b 64K -comp lz4 -noappend -all-root
# mksquashfs ${APP_DIR} ${IMAGE_DEPLOY_DIR}/app.img -b 256K -comp xz -noappend -all-root