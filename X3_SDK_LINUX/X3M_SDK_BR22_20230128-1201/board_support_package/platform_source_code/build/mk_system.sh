#!/bin/bash
# @Author: yaqiang.li
# @Date:   2021-07-04 14:00:07
# @Last Modified by:   yaqiang.li
# @Last Modified time: 2022-07-15 21:02:06

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

rm -f ${IMAGE_DEPLOY_DIR}/${HR_ROOTFS_PART_NAME}.img

ROOTFS_ORIG_DIR=${HR_ROOTFS_DIR}
ROOTFS_BUILD_DIR=${IMAGE_DEPLOY_DIR}/rootfs
rm -rf ${ROOTFS_BUILD_DIR}
[ ! -d $ROOTFS_BUILD_DIR ] && mkdir ${ROOTFS_BUILD_DIR}

gpt_config=${HR_GPT_CONF_DIR}/${HR_GPT_CONF_FILENAME}

function get_rootfs_param()
{
    line=`sed -n "/$1/p; /$1/q" ${gpt_config}`
    if [ "${line}" = "" ]; then
        unset line
        return
    fi
    arr=(${line//:/ })
    local needparted=${arr[0]}
    local _name=${arr[1]}
    local dir=${_name%/*}
    local starts=${arr[3]}
    local stops=${arr[4]}
    local stop=${stops%?}
    local start=${starts%?}

    local fs_size=$((512 * $((${stop} - ${start} + 1))))
    fs_type=${arr[2]}
    partition_size_allocation=$fs_size
    partition_align_size_allocation=$(($partition_size_allocation - 512*1024))

    unset line
}

function get_partition_size()
{
    partition_size=$(du -sk ${ROOTFS_BUILD_DIR} | awk '{print $1}')
    partition_size=$((${partition_size} * 1024))
    if [ ! -z ${partition_size} ];then
        # 扩大两倍，因为制作的
        partition_size=$((${partition_size} * 2))
        partition_align_size=$(($partition_size - 512*1024))
    else
        echo "rootfs size error: ${partition_size}"
        exit -1
    fi
}

partition_name=${HR_ROOTFS_PART_NAME}
fs_type=""
partition_size_allocation=0
partition_align_size_allocation=0
partition_size=0
partition_align_size=0
# get rootfs part size
get_rootfs_param ${partition_name}

if [[ "$fs_type" = "none" ]] || [[ "$fs_type" = "" ]] ;then
    fs_type=${HR_ROOTFS_FS_TYPE}
fi

function get_system_partition_number()
{
    for line in $(cat $gpt_config)
    do
        arr=(${line//:/ })

        local needparted=${arr[0]}
        local _name=${arr[1]}
        local dir=${_name%/*}

        if [ x"$needparted" = x"1" ];then
            system_id=$(($system_id + 1))
        fi

        if [ x"$dir" = x"${partition_name}" ];then
            return
        fi
    done
}

function create_config_file()
{
    local file=$1

    echo "mount_point=$partition_name" > $file
    echo "fs_type=$fs_type" >> $file
    echo "partition_size=$partition_align_size_allocation" >> $file
    echo "partition_name=$partition_name" >> $file
    echo "uuid=da594c53-9beb-f85c-85c5-cedf76546f7a" >> $file
    echo "verity=true" >> $file
    if [[ "$fs_type" = *"ext"* ]];then
        echo "ext_mkuserimg=make_ext4fs" >> $file
    elif [[ "$fs_type" = *"squash"* ]];then
        # avialable options: block_size, noI, noD, noF, noX, core_num - processors
        echo "squash_mkuserimg=mksquashfs" >> $file
        echo "comp_type=gzip" >> $file
        echo "all_root=y" >> $file
    fi

    local avb_tool_path=${HR_AVB_TOOLS_PATH}
    echo "avb_avbtool=$avb_tool_path/avbtool" >> $file
    echo "verity_key=$avb_tool_path/keys/shared.priv.pem" >> $file
    echo "verity_block_device=/dev/mmcblk0p$system_id" >> $file
    echo "skip_fsck=true" >> $file
    echo "verity_signer_cmd=verity/verity_signer" >> $file
}

# 制作 yocto 根文件系统镜像
function make_yocto_image()
{
    # copy rootfs to build dir
    cp -ar ${ROOTFS_ORIG_DIR}/root/* ${ROOTFS_BUILD_DIR}

    # install ko
    if [ x"true" = x"${HR_ROOTFS_WITH_KO}" ]; then
        echo "Install kernel modules"
        [ -d ${IMAGE_DEPLOY_DIR}/ko_install ] && cp -arf ${IMAGE_DEPLOY_DIR}/ko_install/* ${ROOTFS_BUILD_DIR}/
    fi

    # install media librarys
    if [ x"true" = x"${HR_ROOTFS_WITH_MEDIA_LIBS}" ]; then
        echo "Install media librarys"
        [ -d ${IMAGE_DEPLOY_DIR}/appsdk/lib ] && cp -arf ${IMAGE_DEPLOY_DIR}/appsdk/lib/ ${ROOTFS_BUILD_DIR}/
        [ -d ${IMAGE_DEPLOY_DIR}/appsdk/usr ] && cp -arf ${IMAGE_DEPLOY_DIR}/appsdk/usr/ ${ROOTFS_BUILD_DIR}/
        [ -d ${IMAGE_DEPLOY_DIR}/appsdk/etc ] && cp -arf ${IMAGE_DEPLOY_DIR}/appsdk/etc/ ${ROOTFS_BUILD_DIR}/
        [ -d ${IMAGE_DEPLOY_DIR}/appsdk/bin ] && cp -arf ${IMAGE_DEPLOY_DIR}/appsdk/bin/ ${ROOTFS_BUILD_DIR}/
    fi

    # install Sensor driver librarys
    echo "Install Sensor driver librarys"
    mkdir -p ${ROOTFS_BUILD_DIR}/lib/sensorlib
    [ -d ${IMAGE_DEPLOY_DIR}/camera ] && cp -arf ${IMAGE_DEPLOY_DIR}/camera/utility/sensor/*.so ${ROOTFS_BUILD_DIR}/lib/sensorlib

    # delete static librarys
    rm -f ${ROOTFS_BUILD_DIR}/lib/*.a ${ROOTFS_BUILD_DIR}/lib/hbmedia/*.a ${ROOTFS_BUILD_DIR}/lib/hbbpu/*.a

    # 创建根文件系统中的空目录，git不管理空目录，也不管理dev目录下的设备类型文件
    mkdir -p ${ROOTFS_BUILD_DIR}/{home,home/root,mnt,root,usr/lib,var,media,tftpboot,var/lib,var/volatile,dev,proc,tmp,run,sys,userdata,app}

    echo "Horizon Robotic System Ver:${HR_BSP_VERSION} yocto release \n \l" >${ROOTFS_BUILD_DIR}/etc/issue
    echo "Horizon Robotic System Ver:${HR_BSP_VERSION} yocto release \n \l" >${ROOTFS_BUILD_DIR}/etc/issue.net
    echo "${HR_BSP_VERSION}" >${ROOTFS_BUILD_DIR}/etc/version

    # Do Hijack
    SRC_HIJACK_ROOTFS_DIR=${ROOTFS_ORIG_DIR}/hijack
    if [ -x ${SRC_HIJACK_ROOTFS_DIR}/do_hijack.sh ]; then
        ${SRC_HIJACK_ROOTFS_DIR}/do_hijack.sh ${SRC_HIJACK_ROOTFS_DIR} ${ROOTFS_BUILD_DIR} "false"
    fi

    if [ x"false" = x"${HR_ROOTFS_AVB_VERITY}" ]; then
        if [[ "$fs_type" = *"ext"* ]];then
            make_ext4fs -l ${partition_size_allocation} -L ${HR_ROOTFS_PART_NAME} ${IMAGE_DEPLOY_DIR}/${HR_ROOTFS_PART_NAME}.img ${ROOTFS_BUILD_DIR}
            resize2fs -M ${IMAGE_DEPLOY_DIR}/${HR_ROOTFS_PART_NAME}.img
        elif [[ "$fs_type" = *"squash"* ]];then
            mksquashfs ${${ROOTFS_BUILD_DIR}} ${IMAGE_OUT_DIR}/${HR_ROOTFS_PART_NAME}.img -comp gzip -all-root
        fi
        exit 0
    fi

    # 根文件系统添加avb校验，根文件系统的挂载方式也会更改为挂载 /dev/dm-0
    if [ "$BOOT_MODE" != "emmc" -a "$HR_ROOTFS_AVB_VERITY" != "true" ];then
        echo "build rootfs image failed, configuration parameters not allowed"
        exit 0
    fi

    outimage=${IMAGE_DEPLOY_DIR}/${HR_ROOTFS_PART_NAME}.img
    indir=${ROOTFS_BUILD_DIR}

    system_id=0
    # get system_id
    get_system_partition_number

    # 设置avb tools bin的路径
    export PATH=$PATH:${HR_AVB_TOOLS_PATH}/bin

    cd ${IMAGE_DEPLOY_DIR}

    # create config file
    system_image_info="system_image_info.txt"
    rm -rf $system_image_info
    touch $system_image_info
    create_config_file $system_image_info
    mkdir -p out
    echo "python2 ${HR_AVB_TOOLS_PATH}/build_image.py $indir $system_image_info $outimage $partition_align_size_allocation"
    python2 ${HR_AVB_TOOLS_PATH}/build_image.py $indir $system_image_info $outimage $partition_align_size_allocation || {
        echo "${HR_AVB_TOOLS_PATH}/build_image.py $indir failed!"
        exit 1
    }
    rm $system_image_info
    rm -rf ${IMAGE_DEPLOY_DIR}/${HR_ROOTFS_PART_NAME}_output
    mv out ${IMAGE_DEPLOY_DIR}/${HR_ROOTFS_PART_NAME}_output
}

function install_deb_chroot()
{
    local name=$1
    local dst_dir=$2

    cd "${dst_dir}/app/hobot_debs"
    echo "Installing" "${name}"
    if [ -f ${name} ];then
        chroot "${dst_dir}" /bin/bash -c "dpkg -i /app/hobot_debs/$name"
    fi
}


function install_packages()
{
    local dst_dir=$1
    if [ ! -d ${dst_dir} ]; then
        echo "dst_dir is not exist!" "${dst_dir}"
        exit -1
    fi

    echo "Start install hobot packages"

    cd "${dst_dir}/app/hobot_debs"
    deb_list=$(ls)
    # deb_list=(hobot-arm64-boot hobot-arm64-bins hobot-arm64-configs \
    #     hobot-arm64-desktop hobot-arm64-dnn-python hobot-arm64-gpiopy \
    #     hobot-arm64-hdmi-sdb hobot-arm64-includes hobot-arm64-libs \
    #     hobot-arm64-modules hobot-arm64-sdb-ap6212 hobot-arm64-srcampy)

    for deb_name in ${deb_list[@]}
    do
        install_deb_chroot "${deb_name}" "${dst_dir}"
    done

    chroot ${dst_dir} /bin/bash -c "apt clean"
    echo "Install hobot packages is finished"
}

# 制作 ubuntu 根文件系统镜像
function make_ubuntu_image()
{
    # ubuntu 系统直接解压制作image
    echo "cat ${ROOTFS_ORIG_DIR}/${HR_ROOTFS_PKG}[0-9] | tar -Jxf - -C ${ROOTFS_BUILD_DIR}"
    cat ${ROOTFS_ORIG_DIR}/${HR_ROOTFS_PKG}[0-9] | tar -Jxf - -C ${ROOTFS_BUILD_DIR}
    mkdir -p ${ROOTFS_BUILD_DIR}/{home,home/root,mnt,root,usr/lib,var,media,tftpboot,var/lib,var/volatile,dev,proc,tmp,run,sys,userdata,app,boot/hobot}
    echo "${HR_BSP_VERSION}" >${ROOTFS_BUILD_DIR}/etc/version

    # install debs
    if [ x"true" = x"${HR_ROOTFS_WITH_HR_DEBS}" ]; then
        echo "Install hobot debs in /app/hobot_debs"
        mkdir -p ${ROOTFS_BUILD_DIR}/app/hobot_debs
        [ -d ${IMAGE_DEPLOY_DIR}/debs ] && cp -arf ${IMAGE_DEPLOY_DIR}/debs/*.deb ${ROOTFS_BUILD_DIR}/app/hobot_debs
        [ -d ${IMAGE_DEPLOY_DIR}/../third_party_debs ] && cp -arf ${IMAGE_DEPLOY_DIR}/../third_party_debs/*.deb ${ROOTFS_BUILD_DIR}/app/hobot_debs
        install_packages ${ROOTFS_BUILD_DIR}
        rm ${ROOTFS_BUILD_DIR}/app/hobot_debs/ -rf
    fi

    rm -rf ${ROOTFS_BUILD_DIR}/lib/aarch64-linux-gnu/dri/

    # Do Hijack
    SRC_HIJACK_ROOTFS_DIR=${ROOTFS_ORIG_DIR}/hijack
    if [ -x ${SRC_HIJACK_ROOTFS_DIR}/do_hijack.sh ]; then
        ${SRC_HIJACK_ROOTFS_DIR}/do_hijack.sh ${SRC_HIJACK_ROOTFS_DIR} ${ROOTFS_BUILD_DIR} "false"
    fi

    # 把内核和设备树都放到sdcard boot目录下
    if [ x"sdcard" = x"${HR_LUNCH_MODE}" ];then
        if [ ! -d ${IMAGE_DEPLOY_DIR}/kernel ];then
            echo "Please compile the kernel first"
            exit -1
        fi
        cp -arf ${IMAGE_DEPLOY_DIR}/kernel/*.dtb "${ROOTFS_BUILD_DIR}/boot/hobot/"
        kernel_src_dir=${HR_TOP_DIR}/boot/kernel
        kernel_version="4.14.87"
        boot_dest_dir=${ROOTFS_BUILD_DIR}/boot
        cp -arf ${kernel_src_dir}/.config ${boot_dest_dir}/config-${kernel_version}
        cp -arf ${kernel_src_dir}/.config ${boot_dest_dir}/config-${kernel_version}.old
        cp -arf ${kernel_src_dir}/System.map ${boot_dest_dir}/System.map-${kernel_version}
        cp -arf ${kernel_src_dir}/System.map ${boot_dest_dir}/System.map-${kernel_version}.old
        cp -arf ${kernel_src_dir}/arch/arm64/boot/Image ${boot_dest_dir}/vmlinuz-${kernel_version}
    fi

    # 从实际的根文件系统大小里面直接计算得到根文件系统大小
    get_partition_size

    if [[ "$fs_type" = *"ext"* ]];then
        make_ext4fs -l ${partition_size} -L ${HR_ROOTFS_PART_NAME} ${IMAGE_DEPLOY_DIR}/${HR_ROOTFS_PART_NAME}.img ${ROOTFS_BUILD_DIR}
        # 压缩根文件系统镜像到最小尺寸
        resize2fs -M ${IMAGE_DEPLOY_DIR}/${HR_ROOTFS_PART_NAME}.img
        # 再最小尺寸上增加50MB空间，用于在系统第一次启动时存放各种服务的启动文件，方式服务启动失败
        image_size=`ls -l --block-size=M ${IMAGE_DEPLOY_DIR}/${HR_ROOTFS_PART_NAME}.img | awk '{print $5}'`
        resize2fs ${IMAGE_DEPLOY_DIR}/${HR_ROOTFS_PART_NAME}.img `expr ${image_size%?} + 50`M
    else
        echo "Rootfs filesystem type \'${fs_type}\' not supported"
        exit -1
    fi

    # 如果是sdcard启动方式，需要在生成的根文件系统前头加上分区表头用于烧录到sdcard上
    if [ x"sdcard" = x"${HR_LUNCH_MODE}" ];then
        # 在原来的文件之前添加1MB数据用于存放分区表
        dd if=${IMAGE_DEPLOY_DIR}/${HR_ROOTFS_PART_NAME}.img of=${IMAGE_DEPLOY_DIR}/${HR_ROOTFS_PART_NAME}_sdcard.img bs=512 seek=2048

        # 添加分区信息
        parted -s ${IMAGE_DEPLOY_DIR}/${HR_ROOTFS_PART_NAME}_sdcard.img -- mklabel msdos
        parted -s ${IMAGE_DEPLOY_DIR}/${HR_ROOTFS_PART_NAME}_sdcard.img -- mkpart primary ${fs_type} 2048s 100%
    fi
    exit 0
}

if [ x"yocto" = x"${HR_ROOTFS_TYPE}" ]; then
    make_yocto_image
elif [ x"ubuntu" = x"${HR_ROOTFS_TYPE}" ]; then
    make_ubuntu_image
else
    echo "Rootfs type \'${HR_ROOTFS_TYPE}\' not supported"
    exit -1
fi
