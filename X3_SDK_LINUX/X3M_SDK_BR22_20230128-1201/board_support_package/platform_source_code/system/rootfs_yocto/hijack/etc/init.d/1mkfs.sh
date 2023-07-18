#!/bin/sh

#$1 partition id
#$2 volume label
function mk_part_fs()
{
    if [ -f /etc/crypttab ] && [ ! -z $(awk -v pat="$2" '$1~pat {print $1}' /etc/crypttab) ];then
        echo "$2 Found in crypttab, skipping fs check!"
        return
    fi
    file_sys=$(parted /dev/mmcblk0p$1 print |tail -n 2 | head -n 1|awk '{print $5}')
    if [ x"$file_sys" != x"ext4" ];then
        echo "/dev/mmcblk0p$1 is empty"
        mkfs.ext4 -O 64bit -L $2 -F /dev/mmcblk0p$1
    else
        echo "/dev/mmcblk0p$1 file system is $file_sys"
    fi
}

function is_emmc_inited()
{
    local count=0
    while [ ! -n "$(find /dev -name mmcblk0*)" ]
    do
        if [ $count -eq 4 ];then
            echo "Error:Wait emmc init done timeout" > /dev/ttyS0
            exit 0
        else
            sleep 0.5
            count=$(expr $count + 1)
            echo "wait emmc count :$count"
        fi
    done
}

file="/sys/class/socinfo/boot_mode"

if [ x"$(cat $file)" = x"0" ];then
    is_emmc_inited
fi
reset_reason=$(hrut_resetreason)

if [ "${reset_reason}" = "recovery" ];then
    exit 0
fi

if [ -f $file ];then
    boot_mode=$(cat $file)

    # nor boot
    if [ x"$boot_mode" = x"5" ];then
        device="/dev/$(mtdinfo -a | sed -n -e '/app/{g;1!p;};h')"
        if [ -c $device ];then
            mkdir -p /app
            ubiattach  -p $device -d 1
            mount -t ubifs  /dev/ubi1_0 /app -o sync
        fi
    elif [ x"$boot_mode" = x"1" ];then
        device="/dev/$(mtdinfo -a | sed -n -e '/userdata/{g;1!p;};h')"
        if [ -c $device ];then
            mkdir -p /userdata
            ubiattach  -p $device -d 1
            mount -t ubifs  /dev/ubi1_0 /userdata -o sync
        fi
    elif [ x"$boot_mode" = x"0" ];then
        abstatus=`hrut_otastatus g partstatus`
        tmp=$(((abstatus >> 4) & 1))
        if [ $tmp -eq 1 ]; then
                app_name="app_b"
                appbk_name="app"
        else
                app_name="app"
                appbk_name="app_b"
        fi
        part_id=$(fdisk -l | grep -w $app_name | awk '{print $1}')
        # if "app_b" not exist, switch to "app" partition
        if [ -z $part_id ] && [ x"$app_name" == x"app_b" ];then
            echo "1mkfs.sh: $app_name partition not exist, switch to app partition!"
            app_name="app"
            part_id=$(fdisk -l | grep -w $app_name | awk '{print $1}')
        fi

        if [ -z $part_id ];then
            echo "$app_name partition not found! skip filesystem creation for app!" > /dev/ttyS0
        else
            mk_part_fs $part_id "app"
        fi

        # Handle app_bak
        part_id=$(fdisk -l | grep -w ${appbk_name} | awk '{print $1}')
        if [ ! -z ${part_id} ]; then
            mk_part_fs ${part_id} "app"
        fi

        part_id=$(fdisk -l | grep userdata | awk '{print $1}')
        mk_part_fs $part_id "userdata"

        IFS_old=$IFS
        IFS=$'\n'
        ff=$(blkid)
        for blk in $ff
        do
            IFS=":\""
            disk=$(echo $blk | awk '{print $1}')
            label=$(echo $blk | awk '{print $3}')
            type=$(echo $blk | awk '{print $7}')
            # skip partitions in crypttab
            if [ -f /etc/crypttab ] && [ ! -z $(awk -v pat="$label" '$1~pat {print $1}' /etc/crypttab) ];then
                continue
            fi
            if [ "$label" != "system" -a "$type" = "ext4" ];then
                e2fsck -y $disk >/dev/null 2>&1
                ck_res=$?
                if [ $ck_res -lt 4 ];then
                    resize2fs -f $disk > /dev/null 2>&1
                else
                    echo "#### e2fsck $disk FAILED with ret:$ck_res ! ####" > /dev/ttyS0
                fi
            fi
        done
        IFS=$IFS_old
    fi
fi

#[ ! -f "/etc/mkfs_mmcblk0p5.done" ] && mkfs.ext4 -O 64bit -F /dev/mmcblk0p5 && touch /etc/mkfs_mmcblk0p5.done
#[ ! -f "/etc/mkfs_mmcblk0p6.done" ] && mkfs.ext4 -O 64bit -F /dev/mmcblk0p6 && touch /etc/mkfs_mmcblk0p6.done
