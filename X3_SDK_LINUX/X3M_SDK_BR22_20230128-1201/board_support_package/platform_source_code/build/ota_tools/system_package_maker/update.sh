#!/bin/sh
# update uboot partition

part_id=$1
ota_bootmode=$2
upgrade_file_path=$3
if [ -z $upgrade_file_path ];then
    upgrade_file_path="/userdata/cache"
fi

tmp="${upgrade_file_path}/tmpdir"
ota_upmode="AB"
ota_resetreason="normal"
board_status="normal"

ota_logpath="/ota"
if [ ! -d "$ota_logpath" ];then
    ota_logpath="/userdata/cache"
    if [ ! -d "$ota_logpath" ];then
        ota_logpath="/tmp/cache"
    fi
fi

ota_info="${ota_logpath}/info"
update_result="${ota_logpath}/result"
upgrade_file="bin etc lib sbin usr"
contents="bin dev lib normal_rootfs proc sys tmp userdata"
minirootfs="/tmp/minirootfs"
mtd_device=''

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

function get_upmode()
{
    ota_upmode=$(/usr/bin/hrut_otastatus g upmode)
}

function upgrade_rootfs()
{
    echo "prepare temp rootfs"
    local file_img=$1
    rm -rf ${minirootfs}
    mkdir -p ${minirootfs}
    cd ${minirootfs}
    mkdir ${contents}
    mkdir -p ${minirootfs}/usr/bin
    mkdir -p ${minirootfs}/sys/class/socinfo
    mkdir -p ${minirootfs}/usr/sbin
    mkdir -p ${minirootfs}/sbin

    # cp file to minirootfs
    cp -rf /bin/busybox  ${minirootfs}/bin/busybox
    cp -rf /bin/dd  ${minirootfs}/bin/dd
    cp -rf /bin/sh ${minirootfs}/bin/sh
    cp -rf /bin/rm ${minirootfs}/bin/rm
    cp -rf /lib/libpthread-2.31.so  ${minirootfs}/lib/libpthread.so.0
    cp -rf /lib/ld-2.31.so  ${minirootfs}/lib/ld-linux-aarch64.so.1
    cp -rf /lib/libc-2.31.so ${minirootfs}/lib/libc.so.6
    cp -rf /lib/libm-2.31.so ${minirootfs}/lib/libm.so.6
    cp -rf /lib/libblkid.so.1.1.0  ${minirootfs}/lib/libblkid.so.1
    cp -rf /lib/libuuid.so.1.3.0  ${minirootfs}/lib/libuuid.so.1
    cp -rf /lib/libresolv-2.31.so  ${minirootfs}/lib/libresolv.so.2
    cp -rf /sbin/reboot ${minirootfs}/sbin/reboot
    cp -rf /sys/class/socinfo/*  ${minirootfs}/sys/class/socinfo/
    cp -rf /usr/bin/hrut* ${minirootfs}/usr/bin/
    cp -rf /usr/sbin/flashcp  ${minirootfs}/usr/sbin/flashcp
    cp -rf /usr/sbin/flash_erase  ${minirootfs}/usr/sbin/flash_erase
    cp -rf /usr/sbin/nandwrite.mtd-utils  ${minirootfs}/usr/sbin/nandwrite
    cp -rf /bin/sync  ${minirootfs}/bin/sync
    sync

    # mount need contents
    mount --bind /dev ${minirootfs}/dev
    mount --bind /proc ${minirootfs}/proc
    mount --bind /userdata ${minirootfs}/userdata
    mount --bind /tmp  ${minirootfs}/tmp
    mount --bind /sys ${minirootfs}/sys

    # generate update script in temp rootfs
    echo "#!/bin/sh" > ${minirootfs}/update.sh
    echo "echo \"upgrade system.img \"" >> ${minirootfs}/update.sh
    if [ x"$ota_bootmode" = x"nor_flash" ];then
        echo "echo \"flashcp -v -A ${file_img} $mtd_device\"" >> ${minirootfs}/update.sh
        echo "flashcp -v -A ${file_img} $mtd_device" >> ${minirootfs}/update.sh
    elif [ x"$ota_bootmode" = x"nand_flash" ];then
        echo "echo \"flash_erase -q $mtd_device 0x0 0\"" >> ${minirootfs}/update.sh
        echo "flash_erase -q $mtd_device 0x0 0" >> ${minirootfs}/update.sh
        echo "echo \"nandwrite -a -k -m -q -p $mtd_device ${file_img}\"" >> ${minirootfs}/update.sh
        echo "nandwrite -a -k -m -q -p $mtd_device ${file_img}" >> ${minirootfs}/update.sh
    else
        echo "echo \"dd if=${file_img} of=/dev/mmcblk0p${part_id} bs=512\"" >> ${minirootfs}/update.sh
        echo "busybox dd if=${file_img} of=/dev/mmcblk0p${part_id} bs=512 2> /dev/null" >> ${minirootfs}/update.sh
    fi
    echo "echo \"[OTA_INFO] update_img success !\"" >> ${minirootfs}/update.sh
    echo "echo \"[OTA_INFO] 90%\"" >> ${minirootfs}/update.sh
    echo "tmp=\$(hrut_otastatus s upflag 14)" >> ${minirootfs}/update.sh
    echo "rm -rf ${upgrade_file_path}/tmpdir" >> ${minirootfs}/update.sh
    echo "rm -rf ${upgrade_file_path}/tmpdir_all" >> ${minirootfs}/update.sh
    echo "echo \"[OTA_INFO] 100%\"" >> ${minirootfs}/update.sh
    echo "echo \"[OTA_INFO] reboot ...\"" >> ${minirootfs}/update.sh
    echo "busybox sync" >> ${minirootfs}/update.sh
    echo "busybox reboot -f" >> ${minirootfs}/update.sh

    chmod +x ${minirootfs}/update.sh
    echo "start update in temp rootfs"
    chroot ${minirootfs} /update.sh

    umount ${minirootfs}/dev
    umount ${minirootfs}/proc
    umount ${minirootfs}/userdata
    umount ${minirootfs}/tmp
    umount ${minirootfs}/normal_rootfs
}

function kill_process()
{
    local process=$1

    tmpinfo=$(ps | grep $process | grep -v grep)
    if [ ! -z "$tmpinfo" ];then
        killall -9 $process
    fi
}

function prepare_for_upgrade_flash()
{
    # find mounted ubi
    local ubi_dev_num=""
    local mounted=$(df -k /app | grep /dev/ubi)
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

function query_board_status()
{
    local devroot="/dev/root"
    tmpinfo=$(df -h | grep "$devroot")
    if [ -z "$tmpinfo" ];then
        board_status="recovery"
        echo "check board status: recovery mode"
    fi
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

if [ -z $part_id ] || [ -z $ota_bootmode ];then
    printinfo "Usage: update.sh <part_id> <ota_bootmode>"
    echo "2" >> $update_result
    exit 1
fi

# get OTA upgrade mode
get_upmode

if [ x"$ota_upmode" = x"golden" -a x"ota_bootmode" = x"emmc" ];then
    # check part_id
    hb_check_mmc_partid $part_id $SYSTEM_PARTITON_NAME
fi

file="${upgrade_file_path}/tmpdir/rootfs.zip"
ota_resetreason=$(hrut_resetreason)

# emmc mode: get board status normal status or recovery mode
if [ x"$ota_bootmode" = x"emmc" ];then
    query_board_status
fi

file="${upgrade_file_path}/tmpdir/$SYSTEM_PARTITON_NAME.img"
if [ ! -f $file ];then
    printinfo "Error: file $file not exist"
    echo "2" >> $update_result
    exit 1
fi

if [ x"$ota_bootmode" = x"nor_flash" ] || [ x"$ota_bootmode" = x"nand_flash" ];then
    # get mtd device
    mtd_number_get "system"

    # remount rootfs, set read only
    mount -o remount,ro /

    # prepare for upgrade flash
    tmpinfo=$(df | grep "/mnt")
    if [ ! -z "$tmpinfo" ];then
        prepare_for_upgrade_flash
    fi

    upgrade_rootfs $file
elif [ x"$ota_upmode" = x"AB" ] || [ x"$board_status" = x"recovery" ];then
    printinfo "dd if=$file of=/dev/mmcblk0p${part_id} bs=512"
    dd if=$file of=/dev/mmcblk0p${part_id} bs=512 2> /dev/null
    ret=$?
    if [ $ret != "0" ];then
        printinfo "Error: system image write failed!"
        echo "2" >> $update_result
        exit 1
    fi

    cd /home/root
    sync
elif [ x"$ota_upmode" = x"golden" ];then
    upgrade_rootfs $file
    sync
else
    printinfo "Error: ota boot_mode $ota_bootmode not support"
    echo "2" >> $update_result
    exit 1
fi

printinfo "rootfs update finish"
exit 0
