#!/bin/sh

function printinfo()
{
    local log=$1

    echo "[OTA_INFO] ${log}"
    echo "[OTA_INFO]${log}[OFNI_ATO]" >> $ota_info
}

function printprogress()
{
    local log=$1

    echo "[OTA_INFO] ${log}"
    if [ x"$ota_bootmode" != x"nor_flash" ];then
        echo "[OTA_PROGRESS]${log}[SSERGORP_ATO]" >> $progress
    fi
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

function get_upmode()
{
    ota_upmode=$(/usr/bin/hrut_otastatus g upmode)
}

function get_upflag()
{
    ota_upflag=$(/usr/bin/hrut_otastatus g upflag)
}

function get_resetreson()
{
    ota_resetreason=$(/usr/bin/hrut_resetreason)
}

function update_partition()
{
    local partition=$1
    local flag_offset=$2

    if [ "$ota_bootmode" = "nand_flash" -a "$partition" = "uboot" ];then
        local signed_package_name="${path}/bootloader_signed.zip"
        local package_name="${path}/bootloader.zip"
    else
        local signed_package_name="${path}/${partition}_signed.zip"
        local package_name="${path}/${partition}.zip"
    fi

    if [ -f $signed_package_name ];then
        unzip -o -q $signed_package_name -d ${tmp}
    elif [ -f $package_name ];then
        unzip -o -q $package_name -d ${tmp}
    else
        printinfo "Error: $package_name not exist"
        echo "2" >> $update_result
        exit 1
    fi

    if [ $ota_upmode = "AB" ];then
        local flag=$((($part_status >> ${flag_offset}) & 0x1))
        # Compatible with old version upgrade
        if [ x"$ota_upflag" = x"10" ] && [ x"$ota_resetreason" = x"all" ] ;then
            if [ $flag = "1" ];then
                partition="${partition%%$PARTITION_NAME_SUFFIX}$BAK_PARTITION_NAME_SUFFIX"
            fi
        else
            if [ $flag = "0" ];then
                partition="${partition%%$PARTITION_NAME_SUFFIX}$BAK_PARTITION_NAME_SUFFIX"
            fi
        fi
    fi

    if [ "$ota_bootmode" = "emmc" ];then
        part_id=$(mmc_part_number ${partition})
    else
        part_id=0
    fi

    # when find update.sh, progress update.sh
    if [ -f $file_update ];then
        chmod +x $file_update
        echo "$file_update $part_id $ota_bootmode $upgrade_file_path"
        $file_update $part_id $ota_bootmode $upgrade_file_path
        ret=$?
        if [ "$ret" == "1" ];then
            exit 1
        fi
    else
        printinfo "No file update.sh, skip update ${partition}"
    fi
    rm -rf ${tmp}/*
}

function parameter_check()
{
    if [ -f $all_package ] || [ -f $disk_package ];then
        return 0
    fi

    for part in "$UBOOT_PARTITION_NAME" "$KERNEL_PARTITION_NAME" "$SYSTEM_PARTITON_NAME"; do
        if [ "$ota_bootmode" = "nand_flash" -a "$part" = "uboot" ];then
            local signed_package_name="${path}/bootloader_signed.zip"
            local package_name="${path}/bootloader.zip"
        else
            local signed_package_name="${path}/${part}_signed.zip"
            local package_name="${path}/${part}.zip"
        fi

        if [ ! -f $package_name ] && [ ! -f $signed_package_name ];then
            printinfo "Error: $package_name not exist"
            echo "2" >> $update_result
            exit 1
        fi
    done
}

function upgrade_disk_in_minirootfs()
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
    echo "echo \"upgrade disk.img \"" >> ${minirootfs}/update.sh

    if [ x"$ota_bootmode" = x"nor_flash" ];then
        echo "echo \"flashcp -v -A ${file_img} $mtd_device\"" >> ${minirootfs}/update.sh
        echo "flashcp -v -A ${file_img} $mtd_device" >> ${minirootfs}/update.sh
    elif [ x"$ota_bootmode" = x"nand_flash" ];then
        echo "echo \"flash_erase $mtd_device 0x0 0\"" >> ${minirootfs}/update.sh
        echo "flash_erase $mtd_device 0x0 0" >> ${minirootfs}/update.sh
        echo "echo \"nandwrite -a -k -m $mtd_device ${file_img}\"" >> ${minirootfs}/update.sh
        echo "nandwrite -a -k -m $mtd_device ${file_img}" >> ${minirootfs}/update.sh
    else
        echo "echo \"dd if=${file_img} of=/dev/mmcblk0 bs=512\"" >> ${minirootfs}/update.sh
        echo "busybox dd if=${file_img} of=/dev/mmcblk0 bs=512 2> /dev/null" >> ${minirootfs}/update.sh
     fi

    echo "echo \"[OTA_INFO] update_img success !\"" >> ${minirootfs}/update.sh
    echo "echo \"[OTA_INFO] 90%\"" >> ${minirootfs}/update.sh
    echo "tmp=\$(hrut_otastatus s upflag 14)" >> ${minirootfs}/update.sh
    echo "echo \"[OTA_INFO] set_count 23\"" >> ${minirootfs}/update.sh
    echo "tmp=\$(hrut_count 23)" >> ${minirootfs}/update.sh
    echo "echo \"[OTA_INFO] set resetreason all\"" >> ${minirootfs}/update.sh
    echo "tmp=\$(hrut_resetreason all)" >> ${minirootfs}/update.sh
    echo "rm -rf ${upgrade_file_path}/tmpdir" >> ${minirootfs}/update.sh
    echo "rm -rf ${upgrade_file_path}/tmpdir_all" >> ${minirootfs}/update.sh
    echo "echo \"[OTA_INFO] 100%\"" >> ${minirootfs}/update.sh
    echo "echo \"[OTA_INFO] reboot ...\"" >> ${minirootfs}/update.sh
    echo "busybox sync" >> ${minirootfs}/update.sh
    echo "busybox reboot -f" >> ${minirootfs}/update.sh
    chmod +x ${minirootfs}/update.sh
    echo "start update in temp rootfs"
    chroot ${minirootfs} /update.sh
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

function upgrade_disk()
{
    local file="${disk_package}"

    # remount rootfs, set read only
    mount -o remount,ro /

    # prepare for upgrade nor_flash
    if [ x"$ota_bootmode" = x"nor_flash" ];then
        echo "Error: nor does NOT support all_disk.zip !"
        echo "2" >> $update_result
        exit 1
    fi

    upgrade_disk_in_minirootfs $file $ota_bootmode
}

function mtd_number_get()
{
    for d in /sys/class/mtd/*/; do
        if [ -f "$d/name" ] && [ "$(cat $d/name)" = "$1" ];then
            mtd_device=${d##/sys/class/mtd/}
            mtd_device="/dev/"${mtd_device%%/}
            break
        fi
    done
}

function update_all()
{
    local ret="0"
    if [ -z $part_status ];then
        echo "Usage: update.sh <part_status>"
        echo "2" >> $update_result
        exit 1
    fi

    get_upmode
    parameter_check
    ret=0

    get_upflag
    get_resetreson
    # support AB and golden mode
    # create tmp file
    rm -rf $tmp
    mkdir $tmp

    # when find update.sh, progress update.sh
    if [ -f $disk_package ];then
        upgrade_disk
    else
        printinfo "update uboot partition"
        printprogress "45%"
        update_partition $UBOOT_PARTITION_NAME 1

        printinfo "update boot partition"
        printprogress "60%"
        update_partition $KERNEL_PARTITION_NAME 2

        if [ -f $app_package ];then
            printinfo "update app partition"
            update_partition "app" 4
        fi

        printinfo "update system partition"
        printprogress "75%"
        update_partition $SYSTEM_PARTITON_NAME 3

        sync
    fi
}

# update all_in_one.zip
part_status=$1
ota_bootmode=$2
upgrade_file_path=$3
if [ -z $upgrade_file_path ];then
    upgrade_file_path="/userdata/cache"
fi
minirootfs="/tmp/minirootfs"
upgrade_file="bin etc lib sbin usr"
contents="bin dev etc lib normal_rootfs sbin sys proc tmp userdata"
eeprom_value="00"

ota_upmode="AB"
path="${upgrade_file_path}/tmpdir_all"

# update package path and name
disk_package="${path}/disk.img"
all_package="${path}/all_in_one.img"
app_package="${path}/app.zip"
mtd_device=''

tmp="${upgrade_file_path}/tmpdir"
file_update="${tmp}/update.sh"

ota_logpath="/ota"
if [ ! -d "$ota_logpath" ];then
    ota_logpath="/userdata/cache"
    if [ ! -d $ota_logpath ];then
        ota_logpath="/tmp/cache"
    fi
fi

progress="${ota_logpath}/progress"
ota_info="${ota_logpath}/info"
update_result="${ota_logpath}/result"

update_all
exit 0
