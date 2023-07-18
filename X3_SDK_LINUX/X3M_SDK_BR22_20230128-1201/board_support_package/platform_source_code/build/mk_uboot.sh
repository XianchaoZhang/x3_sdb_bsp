#!/bin/bash
# @Author: yaqiang.li
# @Date:   2021-07-04 09:37:06
# @Last Modified by:   yaqiang.li
# @Last Modified time: 2022-03-19 17:19:17

set -e

######## setting utils_funcs ###################
source ./utils_funcs.sh

export HR_TOP_DIR=$(realpath $(cd $(dirname $0); pwd)/../)
export HR_LOCAL_DIR=$(realpath $(cd $(dirname $0); pwd))

# check board config
check_board_config ${@:1}

# 编译出来的镜像保存位置
export IMAGE_DEPLOY_DIR=${HR_IMAGE_DEPLOY_DIR}
[ ! -z $IMAGE_DEPLOY_DIR ] && [ ! -d $IMAGE_DEPLOY_DIR ] && mkdir $IMAGE_DEPLOY_DIR

bootmode=$HR_BOOT_MODE

config_file=$HR_UBOOT_CONFIG_FILE

N=$(( ($(cat /proc/cpuinfo |grep 'processor'|wc -l)  + 1 ) / 2 ))

function build_all()
{
    # 配置uboot配置
    make ${config_file} || {
        echo "make $config failed"
        exit 1
    }

    # 编译生成uboot.bin
    if [ $# -eq 1 ]; then
        # 编译生成刷机工具使用的使用的uboot.bin，固定设置ddr_dqmap，指定som_type用于做必要的电压域配置和phy复位
        # som type: x3som | j3som | customer som
        case ${HR_SOM_TYPE} in
            x3som)
                som_type=1
                ;;
            j3som)
                som_type=2
                ;;
            x3sdb)
                som_type=3
                ;;
            *)
                if [[ ${HR_SOM_TYPE} =~ ^[0-9]+$ ]];then
                    som_type=${HR_SOM_TYPE}
                else
                    som_type=0
                fi
                ;;
        esac
        make KCFLAGS="-DROOTFS_TYPE=${HR_ROOTFS_FS_TYPE} -DHR_BASE_BOARD_TYPE=${1} -DHR_SOM_TYPE=${som_type}" -j${N} || {
            echo "make failed"
            exit 1
        }
    else
        # 编程生成正常使用的uboot.bin
        make KCFLAGS="-DROOTFS_TYPE=${HR_ROOTFS_FS_TYPE}" -j${N} || {
            echo "make failed"
            exit 1
        }
    fi


    cd ${HR_LOCAL_DIR}
    # 添加uboot header
    bash pack_uboot_tool.sh || {
        echo "pack_uboot_tool.sh failed"
        exit 1
    }
    cd ${HR_LOCAL_DIR}
}

function build_clean()
{
    make clean
}

function build_distclean()
{
    make distclean
}

# 进入源码目录
cd ${HR_TOP_DIR}/uboot

# 根据命令参数编译
if [ $# -eq 0 ] || [ $# -eq 2 ] || [ "$1" = "all" ]; then
    if [ $# -eq 2 ]; then
        build_all $2
    else
        build_all
    fi
elif [ "$1" = "clean" ]; then
    build_clean
elif [ "$1" = "distclean" ]; then
    build_distclean
fi
