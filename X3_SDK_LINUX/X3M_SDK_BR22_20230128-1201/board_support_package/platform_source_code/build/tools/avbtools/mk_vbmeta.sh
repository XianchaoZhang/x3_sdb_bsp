#!/bin/sh
set -e

function get_system_partition_number()
{
    local config=$GPT_CONFIG

    for line in $(cat $config)
    do
        arr=(${line//:/ })

        local needparted=${arr[0]}
        local _name=${arr[1]}
        local dir=${_name%/*}

	if [ x"$needparted" = x"1" ];then
		system_id=$(($system_id + 1))
	fi

	if [ x"$dir" = x"${HR_ROOTFS_PART_NAME}" ];then
		return
	fi
    done
}

function get_rootfs_param()
{
    line=`sed -n "/$1/p; /$1/q" ${GPT_CONFIG}`
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
    partition_size=$fs_size
    partition_align_size=$(($partition_size - 512*1024))

    unset line
}


function build_vbmeta()
{
	# get system id
    get_system_partition_number

    partition_name=${HR_ROOTFS_PART_NAME}
    fs_type=""
    partition_size=0
    partition_align_size=0
    # get rootfs part size
    get_rootfs_param ${partition_name}

    fs_type=${HR_ROOTFS_FS_TYPE}

    # build vbmeta.img
    echo "begin to compile vbmeta.img"
    cd $path_avbtool

    echo "**************************"
    echo "bash build_boot_vbmeta.sh boot $system_id"
    bash build_boot_vbmeta.sh boot $system_id

    if [ x"$HR_ROOTFS_AVB_VERITY" = x"true" ]; then
        echo "*************************"
        echo "bash build_system_vbmeta.sh"
        bash build_system_vbmeta.sh $partition_size $fs_type || {
            echo "build_system_vbmeta.sh $partition_size failed!"
            exit 1
        }
    fi

    echo "*************************"
    echo "bash build_vbmeta.sh"
    bash build_vbmeta.sh
    cd -
}

export GPT_CONFIG=$1
path_avbtool=$2
export BOOT_MODE=$3
export TARGET_DEPLOY_DIR=$4

build_vbmeta

rm -rf out images
