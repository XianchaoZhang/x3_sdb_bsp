#!/bin/sh
# update kernel partition

part_id=$1
ota_bootmode=$2
upgrade_file_path=$3

if [ -z $upgrade_file_path ];then
    upgrade_file_path="/userdata/cache"
fi

ota_logpath="/ota"
if [ ! -d "$ota_logpath" ];then
    ota_logpath="/userdata/cache"
    if [ ! -d "$ota_logpath" ];then
        ota_logpath="/tmp/cache"
    fi
fi

tmp="${upgrade_file_path}/tmpdir"
ota_info="${ota_logpath}/info"
update_result="${ota_logpath}/result"

mtd_device=''
ubi_device=''
ota_upmode="golden"
ab_partition_status=0

function printinfo()
{
    local log=$1

    echo "[OTA_INFO] ${log}"
    echo "[OTA_INFO]${log}[OFNI_ATO]" >> $ota_info
}

function get_ab_partition_status()
{
    ab_partition_status=$(/usr/bin/hrut_otastatus g partstatus)
}

function mmc_part_number()
{
    line=$(busybox fdisk -l | grep -w "$1")
    if [ ! -z "$line" ];then
        echo $(echo "$line" | awk -F ' ' '{print $1}')
    else
        printinfo "Error：mmc partition $1 not found"
        echo "2" >> $update_result
        exit 1
    fi
}

function hb_check_mmc_partid()
{
    partid=$(mmc_part_number $2)

    if [ x"$partid" != x"$1" ];then
        printinfo "Error: partition check failed, part_name: $2, part_id: $1 !"
        echo "2" >> $update_result
        exit 1
	fi
}
function get_upmode()
{
    ota_upmode=$(/usr/bin/hrut_otastatus g upmode)
}

function kill_process()
{
    local process=$1

    tmpinfo=$(ps | grep $process | grep -v grep)
    if [ ! -z "$tmpinfo" ];then
        killall -9 $process
    fi
}

function prepare_for_upgrade_nor_flash()
{
    # find mounted ubi
    local ubi_dev_num=""
    local mounted=$(df | grep /dev/ubi)
    local process_need_kill="dropbear telnetd udevd syslogd klogd cp_time_sync \
        diag hbipc-utilsd"
    for item in $mounted; do
        ubinfo $item >/dev/null 2>&1
        if [ $? -eq 0 ];then
            ubi_dev_num=${item##/dev/ubi}
            ubi_dev_num=${ubi_dev_num%_*}
        fi

        if [ -d "$item" ] && [[ "$ubi_dev_num" != "" ]];then
            local appstop="$item/appstop.sh"
            # stop app and  umount partition
            if [ -f "$appstop" ];then
                chmod +x $appstop
                $appstop
            fi
            umount $item
            ubidetach -d $ubi_dev_num
            ubi_dev_num=""
        fi
    done

    # kill system progress
    for process in ${process_need_kill};do
        kill_process $process
    done
}

function mtd_number_get()
{
    for d in /sys/class/mtd/*/; do
        if [ -f "$d/name" ] && [ "$(cat $d/name)" = "$1" ];then
            mtd_device=${d##/sys/class/mtd/}
            mtd_device="/dev/"${mtd_device%%/}
            return 0
        fi
    done
    printinfo "Error: $1 mtd device not exist !"
    echo "2" >> $update_result
    exit 1
}

function ubi_number_get()
{
    for d in /sys/class/ubi/*/; do
        if [ -f "$d/name" ] && [ "$(cat $d/name)" = "$1" ];then
            ubi_device=${d##/sys/class/ubi/}
            ubi_device="/dev/"${ubi_device%%/}
            return 0
        fi
    done
    printinfo "Error: $1 ubi volume not exist !"
    echo "2" >> $update_result
    exit 1
}

if [ -z $part_id ] || [ -z $ota_bootmode ];then
    printinfo "Usage: update.sh <part_id> <ota_bootmode>"
    echo "2" >> $update_result
    exit 1
fi

# get OTA upgrade mode
get_upmode

if [ ! -f ${tmp}/$VBMETA_PARTITION_NAME.img ] || [ ! -f ${tmp}/$KERNEL_PARTITION_NAME.img ];then
    printinfo "Error: file ${tmp}/$VBMETA_PARTITION_NAME.img or ${tmp}/$KERNEL_PARTITION_NAME.img not exist !"
    echo "2" >> $update_result
    exit 1
fi

if [ x"$ota_upmode" = x"golden" -a x"ota_bootmode" = x"emmc" ];then
    # check part_id
    hb_check_mmc_partid $part_id $KERNEL_PARTITION_NAME
fi

if [ x"$ota_bootmode" = x"nor_flash" ];then
    # get kernel mtd device
    mtd_number_get "vbmeta"

    printinfo "flashcp -v -A ${tmp}/vbmeta.img $mtd_device"
    flashcp -v -A ${tmp}/vbmeta.img $mtd_device
    ret1=$?

    # get kernel mtd device
    mtd_number_get "boot"

    printinfo "flashcp -v -A ${tmp}/boot.img $mtd_device"
    flashcp -v -A ${tmp}/boot.img $mtd_device
    ret2=$?
elif [ x"$ota_bootmode" = x"nand_flash" ];then
    # get kernel ubi volume
    ubi_number_get "vbmeta"
    printinfo "ubiupdatevol ${ubi_device} ${tmp}/vbmeta.img"
    ubiupdatevol ${ubi_device} ${tmp}/vbmeta.img
    ret1=$?

    ubi_number_get "boot"
    printinfo "ubiupdatevol ${ubi_device} ${tmp}/boot.img"
    ubiupdatevol ${ubi_device} ${tmp}/boot.img
    ret2=$?
else
    if [ x"$ota_upmode" != x"AB" ] && [ x"$ota_upmode" != x"golden" ];then
        printinfo "Error: upmode $ota_upmode not supprot!"
        echo "2" >> $update_result
        exit 1
    fi

    vbmeta_id=$(mmc_part_number ${VBMETA_PARTITION_NAME})

    if [ x"$ota_upmode" = x"AB" ];then
        get_ab_partition_status
        boot_flag=$((($ab_partition_status >> 2) & 0x1))
        ota_upflag=$(/usr/bin/hrut_otastatus g upflag)
        # Compatible with old version upflag = 10
        if [ x"$ota_upflag" = x"10" ];then
            if [ $boot_flag = "1" ];then
                vbmeta_id=$(mmc_part_number ${VBMETA_PARTITION_BAK_NAME})
            fi
        else
            if [ $boot_flag = "0" ];then
                vbmeta_id=$(mmc_part_number ${VBMETA_PARTITION_BAK_NAME})
            fi
        fi
    fi
    printinfo "dd if=${tmp}/$VBMETA_PARTITION_NAME.img of=/dev/mmcblk0p${vbmeta_id} bs=512 2> /dev/null"
    dd if=${tmp}/$VBMETA_PARTITION_NAME.img of=/dev/mmcblk0p${vbmeta_id} bs=512 2> /dev/null
    ret1=$?

    printinfo "dd if=${tmp}/$KERNEL_PARTITION_NAME.img of=/dev/mmcblk0p${part_id} bs=512 2> /dev/null"
    dd if=${tmp}/$KERNEL_PARTITION_NAME.img of=/dev/mmcblk0p${part_id} bs=512 2> /dev/null
    ret2=$?
fi

if [ $ret1 != "0" ] || [ $ret2 != "0" ];then
    printinfo "Error: vbmeta/boot partition update failed"
    echo "2" >> $update_result
    exit 1
fi

exit 0
