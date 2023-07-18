#!/bin/bash
# @Author: yaqiang.li
# @Date:   2021-07-04 15:42:32
# @Last Modified by:   yaqiang.li
# @Last Modified time: 2022-06-29 15:45:21

#set -x
set -e

######## setting utils_funcs ###################
source ./utils_funcs.sh

export HR_TOP_DIR=$(realpath $(cd $(dirname $0); pwd)/../)
export HR_LOCAL_DIR=$(realpath $(cd $(dirname $0); pwd))

# 选择板级配置
# 如果无参数则打印出板级配置列表，输入数字选择配置
# 如果输入 数字 参数，则使用该数字代表的板级配置
# 如果直接输入 板级配置名， 则直接使用该板级配置
if [ "$1" = "lunch" ];then
    lunch_board_combo ${@:2}
    exit 0
fi

# check board config
check_board_config ${@:1}

# print all configs
echo "declare -x ARCH=${ARCH}"
echo "declare -x CROSS_COMPILE=${CROSS_COMPILE}"
echo "$(export | grep " HR_")"
export LD_LIBRARY_PATH=${TOOLCHAIN_LD_LIBRARY_PATH}

export IMAGE_DEPLOY_DIR=${HR_IMAGE_DEPLOY_DIR}
[ ! -z ${IMAGE_DEPLOY_DIR} ] && [ ! -d $IMAGE_DEPLOY_DIR ] && mkdir $IMAGE_DEPLOY_DIR

# release board config
if [ -L ${HR_TOP_DIR}/device/.board_config.mk ] && [ ! -z ${IMAGE_DEPLOY_DIR} ]; then
	cp ${HR_TOP_DIR}/device/.board_config.mk ${IMAGE_DEPLOY_DIR}/real_board_config.mk
fi

function help_msg
{
    echo "./xbuild.sh [all | function] [all | clean]"
    echo "Functions: miniboot, uboot, boot, system, pack, debs, factory"
}

function build_miniboot
{
    if [ "$1" = "all" ]; then
        echo "******************************"
        echo "begin to build miniboot"
        bash mk_miniboot.sh || exit 1
        echo "end build miniboot"
        echo "******************************"
    fi
}

function build_uboot
{
    echo "******************************"
    echo "begin to build uboot"
    bash mk_uboot.sh $1 || exit 1
    echo "end build uboot"
    echo "******************************"
}

function build_boot
{
    echo "******************************"
    echo "begin to build boot(kernel)"
    bash mk_boot.sh $1 || exit 1
    echo "end build boot(kernel)"
    echo "******************************"
}

function build_hbre
{
    if [ "$1" = "all" ]; then
        echo "******************************"
        echo "begin to build hbre"
        bash mk_hbre.sh || exit 1
        echo "end build hbre"
        echo "******************************"
    fi
}

function build_debs
{
    if [ "$1" = "all" ]; then
        echo "******************************"
        echo "begin to build ubuntu debs"
        bash mk_debs.sh || exit 1
        echo "end build ubuntu debs"
        echo "******************************"
    fi
}

function build_system
{
    if [ "$1" = "all" ]; then
        echo "******************************"
        echo "begin to build system(rootfs)"
        bash mk_system.sh || exit 1
        echo "end build system(rootfs)"
        echo "******************************"
    fi
}

function build_app
{
    if [ "$1" = "all" ]; then
        echo "******************************"
        echo "begin to build app"
        bash mk_app.sh || exit 1
        echo "end build app"
        echo "******************************"
    fi
}

function build_pack
{
    if [ "$1" = "all" ]; then
        gpt_config_file=${HR_GPT_CONF_DIR}/${HR_GPT_CONF_FILENAME}

        if [ x"sdcard" != x"${HR_LUNCH_MODE}" ];then
            echo "******************************"
            echo "begin to signed boot.img/system.img and build vbmeta.img"
            cd ${HR_AVB_TOOLS_PATH}
            bash mk_vbmeta.sh $gpt_config_file ${HR_AVB_TOOLS_PATH} ${HR_BOOT_MODE} ${IMAGE_DEPLOY_DIR} || exit 1
            echo "end to signed boot.img/system.img and build vbmeta.img"
            echo "******************************"
        fi

        local image_name=disk.img

	if [ "$HR_BOOT_MODE" = "nand" ]; then
		image_name=disk_nand_minimum_boot.img
	fi

        echo "******************************"
        echo "begin to pack all image to ${image_name}"
        cd ${HR_LOCAL_DIR}

        rm -f ${IMAGE_DEPLOY_DIR}/${image_name}

        cur_size=0
        is_fill_miniboot=0
        for line in $(cat ${gpt_config_file})
        do
            arr=(${line//:/ })
            needparted=${arr[0]}
            name="$(eval echo ${arr[1]})"
            dir=${name%%/*}
            blkstart=$(( $(trans_unit ${arr[3]}) ))
            blkend=$(( $(trans_unit ${arr[4]}) ))
            size=$(($((${blkend} - ${blkstart} + 1))))
            needpack=${arr[5]}

            #echo ${dir} $blkend $part_img $name

            if [ "$is_fill_miniboot" = "0" ]; then
                part_img=${IMAGE_DEPLOY_DIR}/miniboot.img
            else
                part_img=${IMAGE_DEPLOY_DIR}/${name##*/}
            fi

            if [ "$needparted" = "1" ] && [ "$needpack" = "1" ];then
                if [ ! -f "${part_img}" ];then
                    # 分区镜像不存在直接退出
                    echo "[ERROR] ${part_img} does not exist!"
                    #runcmd "dd if=/dev/zero of=$part_img bs=512 count=${size} status=none"
                    exit -1
                fi
                if [ $is_fill_miniboot = 0 ]; then
                    cur_size=0
                else
                    cur_size=$blkstart
                    image_size=`du --block-size=512 ${part_img} | awk '{print $1}'`
                    if [ ${image_size} -gt ${size} ]; then
                        echo "[ERROR] The ${part_img} size(${image_size}) is larger than the partition size(${size}). "
                        echo "[ERROR] Please adjust the partition table or reduce the image content!"
                        exit -1
                    fi
                fi
                runcmd "dd if=${part_img} of=${IMAGE_DEPLOY_DIR}/${image_name} bs=512 seek=${cur_size} conv=sync,notrunc status=none"

                if [ ${dir} = "miniboot" ]; then
                    is_fill_miniboot=1
                fi
            fi
        done
        echo "end pack all image to ${image_name}"
        echo "******************************"
    elif [ "$1" = "clean" ] || [ "$1" = "distclean" ];then
        echo "delete deploy"
        [ ! -z ${IMAGE_DEPLOY_DIR} ] && [ -d ${IMAGE_DEPLOY_DIR} ] && rm -rf ${IMAGE_DEPLOY_DIR}
    fi
    return 0
}

function build_factory
{
    if [ "$1" = "all" ]; then
        echo "******************************"
        echo "begin to build boot_uart_secure.pkg"
        # setting BASE_BOARD_TYPE index
        case ${HR_BASE_BOARD_TYPE} in
            x3dvb)
                index=1
                ;;
            j3dvb)
                index=2
                ;;
            cvb)
                index=3
                ;;
            x3sdb)
                index=4
                ;;
            customer)
                index=5
                ;;
            *)
                # AUTO_DETECTION by gpio 12 and 14
                index=0
                ;;
        esac
        bash mk_uboot.sh $1 $index || exit 1

        ${HR_MK_UART_BOOT_PACKAGE_TOOLS_PATH}/x3-img-pkg.sh -t uart  -i ${IMAGE_DEPLOY_DIR}/factory_images/spl_storage_secure.bin \
        -b ${IMAGE_DEPLOY_DIR}/factory_images/bl31.img  -u ${IMAGE_DEPLOY_DIR}/uboot.img -o ${IMAGE_DEPLOY_DIR}/factory_images/boot_uart_secure.pkg || exit 1
        echo "end build boot_uart_secure.pkg"
        echo "******************************"
    fi
}

function build_ota
{
    if [ "$1" = "all" ]; then
        echo "******************************"
        echo "begin to create ota upgrade package"
        cd ${HR_LOCAL_DIR}/ota_tools
        local ota_tool_build="./make_update_package.sh"
        echo "${ota_tool_build} -b ${HR_BOOT_MODE} "
        ${ota_tool_build} -b ${HR_BOOT_MODE} || {
            echo "$ota_tool_build -b ${HR_BOOT_MODE} failed!"
            exit 1
        }
        echo "end create ota upgrade package"
        echo "******************************"
    fi
}

function build_all
{
    opt=$1

    build_miniboot $opt

    build_uboot $opt

    build_boot $opt

    build_hbre $opt

    if [ x"ubuntu" = x"${HR_ROOTFS_TYPE}" ]; then
        build_debs $opt
    fi

    build_system $opt

    if [ -d ${HR_TOP_DIR}/app ];then
        build_app
    fi

    build_pack $opt
}

avail_func=("miniboot" "uboot" "boot" "hbre" "debs" "system" "pack" "factory" "lunch" "app" "ota")

if [ $# -eq 0 ];then
    build_all all
elif [ $# -eq 1 ];then
    if [ "$1" = "clean" ];then
        build_all clean
    elif [ "$1" = "distclean" ];then
        build_all distclean
	rm -f ${HR_TOP_DIR}/device/.board_config.mk
    elif [ "$1" = "all" ];then
        build_all all
    elif inList "$1" "${avail_func[*]}";then
        build_$1 all
    else
        help_msg
        exit 1
    fi
elif [ $# -eq 2 ];then
    function=$1
    opt=$2
    build_$function $opt
else
    help_msg
    exit 1
fi
echo "*********************************"
echo "Congratulations, build SUCCEEDED."
echo "*********************************"
