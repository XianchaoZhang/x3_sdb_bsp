#!/bin/bash
# @Author: yaqiang.li
# @Date:   2021-07-04 09:51:07
# @Last Modified by:   yaqiang.li
# @Last Modified time: 2022-03-19 17:20:08

set -e

######## setting utils_funcs ###################
source ./utils_funcs.sh

export HR_TOP_DIR=$(realpath $(cd $(dirname $0); pwd)/../)
export HR_LOCAL_DIR=$(realpath $(cd $(dirname $0); pwd))

# check board config
check_board_config ${@:1}

# 编译出来的镜像保存位置
export IMAGE_DEPLOY_DIR=${HR_IMAGE_DEPLOY_DIR}
[ ! -z ${IMAGE_DEPLOY_DIR} ] && [ ! -d $IMAGE_DEPLOY_DIR ] && mkdir $IMAGE_DEPLOY_DIR

bootmode=$HR_BOOT_MODE

KERNEL_WITH_RECOVERY=${HR_KERNEL_WITH_RECOVERY}
KERNEL_BUILD_DIR=${IMAGE_DEPLOY_DIR}/kernel
[ ! -z ${IMAGE_DEPLOY_DIR} ] && [ ! -d ${KERNEL_BUILD_DIR} ] && mkdir $KERNEL_BUILD_DIR

N=$(( ($(cat /proc/cpuinfo |grep 'processor'|wc -l)  + 1 ) / 2 ))

# 默认使用emmc配置，对于nor、nand需要使用另外的配置文件
kernel_config_file=$HR_KERNEL_CONFIG_FILE
kernel_image_name="Image.lz4"

BOOT_SRC_DIR=${HR_TOP_DIR}/boot
KERNEL_SRC_DIR=${BOOT_SRC_DIR}/kernel

kernel_version=$(awk '/^VERSION\ =/{print $3}' ${KERNEL_SRC_DIR}/Makefile)
kernel_patch_lvl=$(awk '/^PATCHLEVEL\ =/{print $3}' ${KERNEL_SRC_DIR}/Makefile)
kernel_sublevel=$(awk '/^SUBLEVEL\ =/{print $3}' ${KERNEL_SRC_DIR}/Makefile)
export KERNEL_VER="${kernel_version}.${kernel_patch_lvl}.${kernel_sublevel}"


function make_recovery_img()
{
    echo "Making recovery image: recovery.lz4"
    cd ${KERNEL_SRC_DIR}
    recovery_config=${kernel_config_file}

    # 解压 rootfs_recovery.tgz
    tar -xf rootfs_recovery.tgz

    # real build
    make ${recovery_config} || {
        echo "make ${recovery_config} failed"
        exit 1
    }

    make ${kernel_image_name} dtbs -j${N} CONFIG_INITRAMFS_SOURCE="./rootfs_recovery" CONFIG_INITRAMFS_ROOT_UID=0 CONFIG_INITRAMFS_ROOT_GID=0 CONFIG_INITRAMFS_COMPRESSION=".gz" || {
        echo "make failed"
        exit 1
    }

    # put binaries to dest directory
    cp -f "${KERNEL_SRC_DIR}/arch/${ARCH}/boot/${kernel_image_name}"  ${KERNEL_BUILD_DIR}/recovery.lz4
    rm -rf ./rootfs_recovery
    echo "Making recovery image end"
}

function pre_pkg_preinst() {
    # Get the signature algorithm used by the kernel.
    local module_sig_hash="$(grep -Po '(?<=CONFIG_MODULE_SIG_HASH=").*(?=")' "${KERNEL_SRC_DIR}/.config")"
    # Get the key file used by the kernel.
    local module_sig_key="$(grep -Po '(?<=CONFIG_MODULE_SIG_KEY=").*(?=")' "${KERNEL_SRC_DIR}/.config")"
    module_sig_key="${module_sig_key:-certs/hobot_fixed_signing_key.pem}"
    # Path to the key file or PKCS11 URI
    if [[ "${module_sig_key#pkcs11:}" == "${module_sig_key}" && "${module_sig_key#/}" == "${module_sig_key}" ]]; then
        local key_path="${KERNEL_SRC_DIR}/${module_sig_key}"
    else
        local key_path="${module_sig_key}"
    fi
    # Certificate path
    local cert_path="${KERNEL_SRC_DIR}/certs/signing_key.x509"
    # Sign all installed modules before merging.
    find ${KO_INSTALL_DIR}/lib/modules/${KERNEL_VER}/ -name "*.ko" -exec "${KERNEL_SRC_DIR}/scripts/sign-file" "${module_sig_hash}" "${key_path}" "${cert_path}" '{}' \;
}

function build_all()
{
    # 生成内核配置.config
    make $kernel_config_file || {
        echo "make $config failed"
        exit 1
    }

    # 编译生成 zImage.lz4 和 dtb.img
    make ${kernel_image_name} dtbs -j${N} || {
        echo "make ${kernel_image_name} failed"
        exit 1
    }

    # 编译内核模块
    make modules -j${N} || {
        echo "make modules failed"
        exit 1
    }

    # 安装内核模块
    KO_INSTALL_DIR=${IMAGE_DEPLOY_DIR}/ko_install
    [ ! -d $KO_INSTALL_DIR ] && mkdir $KO_INSTALL_DIR

    [ ! -d $KO_INSTALL_DIR ] && mkdir $KO_INSTALL_DIR
    rm -rf $KO_INSTALL_DIR/*

    [ -d ${BOOT_SRC_DIR}/ko/ ] && cp -arf ${BOOT_SRC_DIR}/ko/* $KO_INSTALL_DIR

    make INSTALL_MOD_PATH=$KO_INSTALL_DIR INSTALL_MOD_STRIP=1 INSTALL_NO_SUBDIR=1 modules_install || {
        echo "make modules_install to INSTALL_MOD_PATH for release ko failed"
        exit 1
    }

    # strip 内核模块, 去掉debug info
    ${CROSS_COMPILE}strip -v -g ${KO_INSTALL_DIR}/lib/modules/${KERNEL_VER}/*.ko

    # ko 签名
    pre_pkg_preinst

    # 拷贝 内核 zImage.lz4
    cp -f "arch/arm64/boot/${kernel_image_name}" ${KERNEL_BUILD_DIR}/

    # 生成 dtb 镜像
    cp -arf arch/arm64/boot/dts/hobot/*.dtb ${KERNEL_BUILD_DIR}

    path=./tools/dtbmapping

    cd $path

    export TARGET_KERNEL_DIR=${KERNEL_BUILD_DIR}
    # build dtb
    python2 makeimg.py || {
        echo "make failed"
        exit 1
    }

    # 生成 recovery 分区镜像
    if [ "x$HR_KERNEL_WITH_RECOVERY" = "xtrue" ];then
        # get recovery.gz
        make_recovery_img
    fi

    # 生成boot.img
    cd ${KERNEL_BUILD_DIR}

    if [ ! -f ${kernel_image_name} ];then
        echo "file ${KERNEL_BUILD_DIR}/boot/${kernel_image_name} not exist! "
        echo "please first compile x3j3 project"
        exit 1
    fi

    echo "******************python2 mkbootimg.py Image.lz4***************"
    cd ${KERNEL_BUILD_DIR}
    python2 ${HR_LOCAL_DIR}/mkbootimg.py \
        --base 0x0 \
        --kernel ${kernel_image_name} \
        --kernel_offset 0x200000 \
        --second hobot-xj3-dtb.img \
        --second_offset 0x6000000 \
        --board x3j3 \
        -o ${IMAGE_DEPLOY_DIR}/boot.img

    if [ x"$HR_KERNEL_WITH_RECOVERY" = x"true" ];then
        echo "******************python2 mkbootimg.py recovery.gz***************"
        cd ${KERNEL_BUILD_DIR}
        python2 ${HR_LOCAL_DIR}/mkbootimg.py \
            --base 0x0 \
            --kernel recovery.lz4 \
            --kernel_offset 0x200000 \
            --second hobot-xj3-dtb.img \
            --second_offset 0x6000000 \
            --board x3j3 \
            -o ${IMAGE_DEPLOY_DIR}/recovery.img
    fi
}

function build_clean()
{
    make clean
}

function build_distclean()
{
    make distclean
}

# 进入内核目录
cd ${KERNEL_SRC_DIR}
# 根据命令参数编译
if [ $# -eq 0 ] || [ "$1" = "all" ]; then
    build_all
elif [ "$1" = "clean" ]; then
    build_clean
elif [ "$1" = "distclean" ]; then
    build_distclean
fi
