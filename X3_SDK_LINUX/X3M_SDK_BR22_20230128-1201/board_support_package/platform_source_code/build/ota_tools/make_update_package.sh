#!/bin/bash

set -e

######## setting utils_funcs ###################
source ../utils_funcs.sh

export HR_TOP_DIR=$(realpath $(cd $(dirname $0); pwd)/../../)
export HR_LOCAL_DIR=$(realpath $(cd $(dirname $0); pwd))

# check board config
check_board_config ${@:1}

export IMAGE_DEPLOY_DIR=${HR_IMAGE_DEPLOY_DIR}

function usage()
{
    echo "Usage: $0  [-b emmc|nor|nand]"
    echo "Options:"
    echo "    -b      choose board type"
    echo "              'emmc' emmc board type"
    echo "              'nor'  nor board type"
    echo "              'nand'  nand board type"
    echo "    -h      this help info"
}

function gen_gpt_config()
{
    local gpt=${IMAGE_DEPLOY_DIR}/${HR_GPT_CONF_FILENAME}
    local file="${ota_gpt_file}"

    rm -rf $file
    touch $file
    local i=0
    local prev_end=-1
    local part_added=0
    local orig_part_end=0
    local blk_sz=512
    for line in $(cat $gpt);do
        arr=(${line//:/ })

        local needparted=${arr[0]}
        local _name=${arr[1]}
        local part=${_name%/*}
        local starts=${arr[3]}
        local stops=${arr[4]}
        local part_sz=$(( $(trans_unit $blk_sz ${arr[3]}) ))

        if [ x"$needparted" = x"1" ];then
            orig_part_end=${stops}
            if [ -z "${arr[5]}" ];then
                if [ "$part_added" = "1" ];then
                    part_added=0
                    tmp="$(awk -v OFS=":" -v prev_end="$prev_end" -F ":" 'END{$3=prev_end;print;}' $file)"
                    sed -i "$ s/.*/$tmp/" $file
                fi
                if [ ${prev_end%%s} -lt 0 ];then
                    if [ x"$boot_mode" = x"emmc" ];then
                        starts="34s"
                    else
                        starts="0s"
                    fi
                else
                    starts="$(( ${prev_end%%s} + 1 ))s"
                fi
                prev_end=${starts}
                stops="$(( ${prev_end%%s} + ${part_sz%%s} - 1 ))s"
                prev_end=${stops}
            fi

            echo "$part:$starts:$stops" >> $file
        else
            part_added=1
            prev_end="$(( ${prev_end%%s} + ${part_sz%%s} ))s"
        fi
    done

    # compatible AB mode
    # only remove the last partition for part extend compatibility
    sed -i '$d' $file
}

function gen_sys_version()
{
    local version=${IMAGE_DEPLOY_DIR}/rootfs/etc/version
    local file="${ota_version_file}"

    cp $version $file
}

function calculate_upgrade_need_time()
{
    local package_size=$(du -b ${IMAGE_DEPLOY_DIR}/disk.img)
    local format_time=20
    local reboot_time=12

    package_size=${package_size%%/*}

    # package_size: MByte
    package_size=$(($package_size/1024/1024))

    # need_time: second
    need_time=$(($package_size/4 + $reboot_time + $format_time))

    echo "wait_before_query: $need_time" > ${all_warning_txt}
    echo "estimate: $need_time" >> ${all_warning_txt}
    echo "msg: The update will take about $need_time seconds!" >> ${all_warning_txt}
}

function build_diskimg()
{
    echo "make disk.img upgrade patch"
    rm -f ${deploy_ota_dir}/all_disk*.zip

    # warning file: wait_before_query, estimate and msg
    touch ${all_warning_txt}

    if [ x"$boot_mode" = x"emmc" ];then
        # emmc mode: calculate the time of upgrade disk.img
        calculate_upgrade_need_time
    fi

    # cpfile to deploy
    mkdir -p ${deploy_ota_dir}
    zip -j -y ${deploy_ota_dir}/all_disk.zip ${all_warning_txt} ${HR_LOCAL_DIR}/all/update.sh
    zip -j ${deploy_ota_dir}/all_disk.zip ${IMAGE_DEPLOY_DIR}/disk.img
    mv ${all_warning_txt} ${deploy_ota_dir}/info/all_disk_warning.txt
}

function build_all(){
    if [ x"$boot_mode" = x"emmc" ];then
        # get gpt config
        gen_gpt_config
    fi
    rm -f ${deploy_ota_dir}/all_in_one*.zip

    # get sys version
    gen_sys_version

    # warning file: wait_before_query, estimate and msg
    touch ${all_warning_txt}
    if [ x"$boot_mode" = x"emmc" ];then
        echo "wait_before_query: $mmc_all_uptime" > ${all_warning_txt}
        echo "estimate: $mmc_all_uptime" >> ${all_warning_txt}
        echo "msg: the update will take about $mmc_all_uptime seconds!" >> ${all_warning_txt}
    elif [ x"$boot_mode" = x"nor" ];then
        echo "wait_before_query: $nor_all_uptime" > ${all_warning_txt}
        echo "estimate: $nor_all_uptime" >> ${all_warning_txt}
        echo "msg: the update will take about $nor_all_uptime seconds!" >> ${all_warning_txt}
    elif [ x"$boot_mode" = x"nand" ];then
        echo "wait_before_query: $nand_all_uptime" > ${all_warning_txt}
        echo "estimate: $nand_all_uptime" >> ${all_warning_txt}
        echo "msg: the update will take about $nand_all_uptime seconds!" >> ${all_warning_txt}
    fi

    echo "make uboot update package"
    bash ${src_ota_tools_dir}/uboot_package_maker/build_uboot_update_package.sh $boot_mode

    echo "make kernel update package"
    bash ${src_ota_tools_dir}/kernel_package_maker/build_kernel_update_package.sh $boot_mode

    echo "make rootfs update package"
    bash ${src_ota_tools_dir}/system_package_maker/build_system_update_package.sh $boot_mode

    echo "make all in one update package"

    zip -j -y ${deploy_ota_dir}/all_in_one.zip ${deploy_ota_dir}/*[!signed].zip ${src_ota_tools_dir}/all/update.sh ${ota_version_file} ${all_warning_txt} ${ota_gpt_file} -x "*all*.zip"

    # cpfile to deploy
    mkdir -p ${deploy_ota_dir}
    mkdir -p ${deploy_ota_dir}/script
    mkdir -p ${deploy_ota_dir}/info

    cp -rf ${src_ota_tools_dir}/all/update.sh ${deploy_ota_dir}/script/all_update.sh
    mv ${all_warning_txt} ${deploy_ota_dir}/info/all_warning.txt
}

function ota_package_signed(){
    local sign_script_path=${src_ota_tools_dir}/ota_sign_tools
    local rsa_priv_key_path=$sign_script_path/private.pem
    local deploy_ota_pack_path=${IMAGE_DEPLOY_DIR}/ota
    local package_array=(uboot boot system all_in_one all_disk)
    if [ x"${HR_OTA_SIGNED}" == x"true" ]; then
        for name in ${package_array[@]}; do
            python3 $sign_script_path/sign_package.py $deploy_ota_pack_path/${name}.zip $deploy_ota_pack_path/${name}_signed.zip $rsa_priv_key_path || {
                echo "${src_ota_tools_dir}/ota_sign_tools/sign_package.py failed"
                exit 1
            }
        done
    fi
}

boot_mode="emmc"
img_type="all"
nor_all_uptime="150"
nand_all_uptime="150"
mmc_disk_uptime="60"
mmc_all_uptime="60"

src_ota_tools_dir="${HR_LOCAL_DIR}"
deploy_ota_dir="${IMAGE_DEPLOY_DIR}/ota"
all_warning_txt="${deploy_ota_dir}/warning.txt"
ota_gpt_file="${deploy_ota_dir}/gpt.conf"
ota_version_file="${deploy_ota_dir}/version"

if [ -z $1 ];then
    usage
    exit 1
fi

if [ -z ${IMAGE_DEPLOY_DIR} ];then
     echo "Usage:  compile project before make ota update package"
     exit 0
fi

while getopts "b:e:h:" opt
do
    case $opt in
        b)
            arg="$OPTARG"
            if [ x"$arg" = x"emmc" ];then
                boot_mode=$arg
            elif [ x"$arg" = x"nor" ];then
                boot_mode=$arg
            elif [ x"$arg" = x"nand" ];then
                boot_mode=$arg
            else
                usage
                exit 1
            fi
            ;;
        h)
            usage
            exit 1
            ;;
        \?)
            usage
            exit 1
            ;;
    esac
done

echo "Creating OTA packages for $boot_mode boot mode:"

shift $[ $OPTIND - 1 ]

if [ "$1" = "clean" ];then
    echo -n "Cleaning all OTA temperory files...."
    rm -rf ${deploy_ota_dir}
    echo "Done!"
    exit 0
fi

mkdir -p ${deploy_ota_dir}
# build OTA package: uboot.zip, boot.zip, system.zip
build_all

# build OTA package: disk.img
if [ "${boot_mode}" = "emmc" ];then
    build_diskimg
fi

# make OTA signature package:uboot/boot/system/all_in_one/all_disk_signed.zip
ota_package_signed
