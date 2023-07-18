#!/bin/sh
# update uboot partition

part_id=$1
ota_bootmode=$2
upgrade_file_path=$3
if [ -z $upgrade_file_path ];then
    upgrade_file_path="/userdata/cache"
fi

tmp="${upgrade_file_path}/tmpdir"
ab_partition_status=0
ota_upmode="AB"
mtd_device=''

ota_logpath="/ota"
if [ ! -d "$ota_logpath" ];then
    ota_logpath="/userdata/cache"
    if [ ! -d "$ota_logpath" ];then
        ota_logpath="/tmp/cache"
    fi
fi

ota_info="${ota_logpath}/info"
update_result="${ota_logpath}/result"

function printinfo()
{
    local log=$1

    echo "[OTA_INFO] ${log}"
    echo "[OTA_INFO]${log}[OFNI_ATO]" >> $ota_info
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
    local part_num=$(mmc_part_number $2)

    if [ x"$part_num" != x"$1" ];then
        printinfo "Error: partition check failed, part_name: $2, part_id: $1 !"
        echo "2" >> $update_result
        exit 1
	fi
}

function get_ab_partition_status()
{
    ab_partition_status=$($/usr/bin/hrut_otastatus g partstatus)
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
    echo "Error: $1 mtd device not exist !"
    echo "2" >> $update_result
    exit 1
}

if [ -z $part_id ] || [ -z $ota_bootmode ];then
    echo "[OTA_INFO] Usage: update.sh <part_id> <ota_bootmode>"
    echo "[OTA_INFO]Usage: update.sh <part_id> <ota_bootmode>[OFNI_ATO]" >> $ota_info
    echo "2" >> $update_result
    exit 1
fi

if [ ! -f ${tmp}/${UBOOT_PARTITION_NAME}.img -a ! -f ${tmp}/bootloader.img ];then
    printinfo "Error: upgrade image for uboot not exist !"
    echo "2" >> $update_result
    exit 1
fi

# get OTA upgrade mode
get_upmode

if [ x"$ota_upmode" = x"golden" -a x"ota_bootmode" = x"emmc" ];then
    # check part_id
    hb_check_mmc_partid $part_id $UBOOT_PARTITION_NAME
fi

ret=0
if [ x"$ota_bootmode" = x"nor_flash" ];then
    # get mtd device
    mtd_number_get "uboot"

    # erase and write uboot
    dd if=$mtd_device of=${tmp}/uboot.img bs=512 count=2048 seek=2048 2> /dev/null
    sync
    printinfo "flashcp -v -A ${tmp}/uboot.img $mtd_device"
    flashcp -v -A ${tmp}/uboot.img $mtd_device
    ret=$?
elif [ x"$ota_bootmode" = x"nand_flash" ];then
    # get mtd device
    mtd_number_get "bootloader"

    # erase and write uboot
    image_uboot_offset="0x""$(hexdump -n 4 -s 132 -e '1/4 "%02x"' ${tmp}/bootloader.img)"
    image_uboot_offset=$((${image_uboot_offset}))
    image_uboot_size=$((2*1024*1024))
    nand_uboot_offset="0x""$(hexdump -n 4 -s 132 -e '1/4 "%02x"' $mtd_device)"
    erase_blk_cnt=$(( ${image_uboot_size} / $(cat /sys/class/mtd/${mtd_device##*/}/erasesize) ))
    # Check if content at ${image_uboot_offset} in nand is actually uboot
    for i in $(seq 0 5); do
        magic_str=$(hexdump -n 4 -s $((${image_uboot_offset} + $i * $(cat /sys/class/mtd/${mtd_device##*/}/erasesize) + 0x200 )) -e '1/1 "%c"' $mtd_device)
        if [ "${magic_str}" == "HBOT" ];then
            nand_uboot_offset=$((${image_uboot_offset} + $i * $(cat /sys/class/mtd/${mtd_device##*/}/erasesize)))
            break
        fi
    done
    # Copy current uboot to bootloader.img
    image_ubootbak_offset="0x""$(hexdump -n 4 -s 144 -e '1/4 "%02x"' ${tmp}/bootloader.img)"
    image_ubootbak_offset=$((${image_ubootbak_offset}))
    dd if=${mtd_device} of=${tmp}/bootloader.img bs=512 count=2048 skip=$((${nand_uboot_offset} / 512)) seek=$((${image_ubootbak_offset} / 512)) 2> /dev/null
    # start erase and write
    printinfo "flash_erase -q ${mtd_device} ${nand_uboot_offset} ${erase_blk_cnt}"
    flash_erase -q ${mtd_device} ${nand_uboot_offset} ${erase_blk_cnt}

    printinfo "nandwrite -a -k -m -p -s ${nand_uboot_offset} --input-skip=${image_uboot_offset} --input-size=${image_uboot_size} ${mtd_device} ${tmp}/bootloader.img"
    nandwrite -a -k -m -p -q -s ${nand_uboot_offset} --input-skip=${image_uboot_offset}\
              --input-size=${image_uboot_size} ${mtd_device} ${tmp}/bootloader.img
    ret=$?
else
    if [ x"$ota_upmode" = x"golden" ]; then
        dd if=/dev/mmcblk0p${part_id} of=${tmp}/$UBOOT_PARTITION_NAME.img bs=512 count=2048 seek=2048 2> /dev/null
        sync
    fi
    printinfo "dd if=${tmp}/$UBOOT_PARTITION_NAME.img of=/dev/mmcblk0p${part_id} bs=512 count=2048"
    dd if=${tmp}/$UBOOT_PARTITION_NAME.img of=/dev/mmcblk0p${part_id} bs=512 2> /dev/null
    sync
    ret=$?
fi

if [ $ret != "0" ];then
    echo "[OTA_INFO] Error: write uboot partition error"
    echo "[OTA_INFO]Error: write uboot partition error[OFNI_ATO]" >> $ota_info
    echo "2" >> $update_result
    exit 1
fi
sync
exit 0
