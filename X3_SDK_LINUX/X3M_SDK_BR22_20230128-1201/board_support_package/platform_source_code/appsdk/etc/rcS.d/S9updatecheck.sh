#!/bin/sh

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

function printinfo()
{
    local log=$1

    echo "[OTA_INFO] ${log}" > /dev/ttyS0
    echo "[OTA_INFO]${log}[OFNI_ATO]" >> $ota_info
}

function check_hrut_bin()
{
    otastatus_bin="/usr/bin/hrut_otastatus"
    resetreason_bin="/usr/bin/hrut_resetreason"
    count_bin="/usr/bin/hrut_count"
    for bin in $otastatus_bin $resetreason_bin $count_bin; do
        if [ ! -f $bin ];then
            printinfo "Error: file $bin not found!"
            exit 1
        fi
    done
}

function get_updateflag()
{
    update_flag=$($otastatus_bin g upflag)
}

function get_continue_up_flag()
{
    continue_flag=$($otastatus_bin g continue_up)
}

function set_updateflag()
{
    if [ -z $1 ];then
        printinfo "Error: upflag parameter not set"
        return 1
    fi

    tmp=$($otastatus_bin s upflag $1)
}

function get_upmode()
{
    ota_upmode=$($otastatus_bin g upmode)
}

function set_resetreason()
{
    if [ -z $1 ];then
        printinfo "Error: upflag parameter not set"
        return 1
    fi

    tmp=$($resetreason_bin $1)
}

function get_resetreason()
{
    resetreason=$($resetreason_bin)
}

function set_count()
{
    tmp=$($count_bin $1)
}

function get_ab_partition_status()
{
    ab_partition_status=$($otastatus_bin g partstatus)
}

function set_ab_partition_status()
{
    $otastatus_bin s partstatus $1
    local flag=$($otastatus_bin g partstatus)

    if [ x"$flag" = x"$1" ];then
        printinfo "set upmode $flag success ! "
        ab_partition_status=$1
    else
        printinfo "set upmode $flag failed !"
        exit 1
    fi
}

function abstatus_to_number()
{
    local partname=$1
    local i=0

    for element in spl uboot boot system app userdata;do
        if [ x"$partname" = x"$element" ];then
            return $i
        fi
        i=$((i+1))
    done
}

function recovery_partition()
{
    partition=$1
    part_id=$(mmc_part_number "uboot")
    part_id1=$(mmc_part_number "${partition}")
    part_id2=$(mmc_part_number "${partition}bak")

    printinfo "${partition} recovery: Using ${partition}bak partition recovery!"
    if [ "${partition}" = "uboot" ];then
        dd if=/dev/mmcblk0${part_id} of=/dev/mmcblk0${part_id} bs=512 count=1024 skip=1024 seek=0 2> /dev/null
    else
        dd if=/dev/mmcblk0${part_id2} of=/dev/mmcblk0${part_id1} bs=512 2> /dev/null
    fi
    if [ "$?" != "0" ];then
        printinfo "Recover $partition failed!"
        exit 1
    fi
}

function recovery_system()
{
    if [ x"$1" = x"all" ];then
        recovery_partition "uboot"
        recovery_partition "boot"
        recovery_partition "rootfs"
    else
        recovery_partition "$1"
    fi
}

function update_partstatus()
{
    # update OTA progress
    echo "[OTA_PROGRESS]100%[SSERGORP_ATO]" >> $progress

    if [ x"$ota_upmode" = x"AB" ];then
        local flag=$ab_partition_status
        get_ab_partition_status

        update_success=$(($update_flag>>3 & 0x1))
        if [ x"$update_success" = x"0" ];then
            if [ x"$resetreason" = x"all" ] || [ x"$resetreason" = x"normal" ];then
                # all status has been set by uboot, no not change it again
                flag=$(($ab_partition_status))
            else
                abstatus_to_number $resetreason
                local suffix=$?

                flag=$(($ab_partition_status ^ (0x1 << $suffix)))
            fi

            set_ab_partition_status $flag

            # get_updateflag
            update_flag=$((($update_flag)|0x8))
            set_updateflag $update_flag

            echo "2" >> $update_result
            printinfo "Error: Update $ota_upmode failed, return to old verson, please upgrade again!"
        else
            echo "1" >> $update_result
        fi
    elif [ x"$ota_upmode" = x"golden" ];then
        update_success=$(($update_flag>>3 & 0x1))
        if [ x"$update_success" = x"0" ];then
            echo "2" >> $update_result
            printinfo "Error: Update $resetreason reboot verify failed ! "
            printinfo "Error: Using backup system recovery!"
            printinfo "Enter recovery mode, reboot......"

            # update failed, enter recovery mode
            set_resetreason recovery
            sync
            sleep 2
            reboot
        else
            echo "1" >> $update_result
        fi
    fi
}

function ota_get_boot_mode()
{
    local file="/sys/class/socinfo/boot_mode"

    if [ ! -f $file ];then
        echo "warning：not enable socinfo boot_mode support"
        mode="0"
    else
        mode=$(cat $file)
    fi

    if [ x"$mode" = x"0" ];then
        ota_bootmode="emmc"
    elif [ x"$mode" = x"1" ];then
        ota_bootmode="nand_flash"
    elif [ x"$mode" = x"5" ];then
        ota_bootmode="nor_flash"
    elif [ x"$mode" = x"2" ];then
        ota_bootmode="ap"
    elif [ x"$mode" = x"3" ];then
        ota_bootmode="uart"
    else
        ota_bootmode="emmc"
    fi
}

function clear_update_file()
{
    # clear tmp file: tmpdir, tmpdir_all and minirootfs
    rm -rf ${upgrade_file_path}/tmpdir
    rm -rf ${upgrade_file_path}/tmpdir_all
    rm -rf ${upgrade_file_path}/minirootfs
    sync
}

function entry_recovery_mode()
{
    printinfo ""
    printinfo "******************************************"
    printinfo "        Welcome to recovery mode          "
    printinfo "******************************************"

    get_updateflag

    if [ x"$update_flag" = x"13" ] && [ $(stat -c %s "$command") -eq 0 ];then
        # normal force into recovery
        set_resetreason normal
    elif [ x"$update_flag" = x"4" ];then
        # boot faild after golden updata , next boot into recovery
        set_resetreason recovery
    else
        # possible power down when updata in recovery
        set_resetreason recovery
    fi

    if [ "$?" -eq "1" ];then
        printinfo "Error: read update flag"
        echo "2" >> $update_result
        echo "[OTA_PROGRESS]100%[SSERGORP_ATO]" >> $progress
        exit 1
    fi

    local file="${ota_logpath}/commend"
    if [ ! -f $file ];then
        printinfo "Error：$file not exist"
        echo "2" >> $update_result
        echo "[OTA_PROGRESS]100%[SSERGORP_ATO]" >> $progress
        exit 1
    fi

    # golden mode: update or recovery system
    if [ x"$ota_upmode" = x"golden" ];then
        local name=$(cat $file)
        local up_partition=${name%%/*}
        local upgrade_image="/${name#*/}"

        # check update flag
        if [ x"$update_flag" = x"13" ] || [ x"$update_flag" = x"4" ];then
            # compatible with previous schemes
            local image=${name##*/}
            local file_path="/userdata/cache/${image}"
            if [ -f $file_path ];then
                file="/usr/bin/up_partition.sh"

                printinfo "ota_img_write $file $up_partition $image"
                $file $up_partition $image &
            else
                printinfo "[OTA_INFO] forced into recovery mode"
                exit 0
            fi
        fi
    else
        if [ x"$ota_bootmode" != x"emmc" ];then
            printinfo "Flash OTA upgrade, can't automatic recovery $up_partition"
            printinfo "Warning: you can upgrade $up_partition again"
            printinfo "Recovery: Error upgrade $up_partition failed ! "
            set_updateflag 13
            clear_update_file
            exit 0
        else
            line=$(busybox fdisk -l | grep "boot_b")
            if [ -z "$line" ];then
                printinfo "Single partition, can't automatic recovery $up_partition"
                printinfo "Warning: you can upgrade $up_partition again"
                printinfo "Recovery: Error upgrade $up_partition failed ! "
                set_updateflag 13
                clear_update_file
                exit 0
            fi
            recovery_system $up_partition
            set_updateflag 13
            printinfo "Recovery $up_partition success !"
            clear_update_file
            reboot
        fi
    fi
}

function mkdir_log_file()
{
    if [ ! -d $ota_logpath ] && [ x"$ota_bootmode" != x"nor_flash" ];then
        local userdata_free_space tmp_free_space
        local log_need_space="150"
        userdata_free_space=$(df -k /userdata/ | grep "userdata" | awk '{print $4}')
        tmp_free_space=$(df -k /tmp/ | grep "tmp" | awk '{print $4}')

        # clear log file: /ota
        rm -rf /ota

        if [ "$userdata_free_space" -gt "$log_need_space" ];then
            ota_logpath="/userdata/cache"
        elif [ "$tmp_free_space" -gt "$log_need_space" ];then
            rm -rf /userdata/cache
            ota_logpath="/tmp/cache"
        else
            printinfo "Error: stop upgrade, user space is full ! "
            exit 1
        fi
    fi

    if [ x"$ota_bootmode" = x"nor_flash" ];then
        ota_logpath="/tmp/cache"
    fi
    progress="${ota_logpath}/progress"
    ota_info="${ota_logpath}/info"
    update_result="${ota_logpath}/result"
    app_log="${ota_logpath}/applog"
    #command="${cache}/command"

    local file="${ota_logpath}"
    if [ ! -d $file ];then
        mkdir $file
    fi

    file=$progress
    if [ ! -f $file ];then
        touch $file
    fi

    file=$ota_info
    if [ ! -f $file ];then
        touch $file
    fi

    file=$app_log
    if [ ! -f $file ];then
        touch $file
    fi
    echo " " > $file

    file=$update_result
    if [ ! -f $file ];then
        touch $file
    fi
    ## add updata command ##
    file=$command
    if [ ! -f $file ];then
        touch $file
    fi
}

function check_otaflag()
{
    get_updateflag
    if [ "$?" -eq "1" ];then
        printinfo "Error: read update flag"
        exit 1
    fi

    # normal boot
    if [ x"$update_flag" = x"13" ];then
        set_resetreason normal
        exit 0
    fi

    if [ x"$resetreason" = x"app" ];then
        flash_flag=$(($update_flag>>2 & 0x1))
        if [ x"$flash_flag" = x"1" ];then
            # set first_try=0
            update_flag=$((($update_flag)&0xd))
            echo "1" >> $update_result
            echo "[OTA_PROGRESS]100%[SSERGORP_ATO]" >> $progress
        else
            echo "2" >> $update_result
            printinfo "Error: $resetreason image write failed ! "
            exit 1
        fi
    elif [ x"$resetreason" = x"userdata" ];then
        # set first_try=0
        update_flag=$((($update_flag)&0xd))
        echo "1" >> $update_result
        echo "[OTA_PROGRESS]100%[SSERGORP_ATO]" >> $progress
    else
        # rootfs boot success, set app_success=1
        update_flag=$((($update_flag)|0x1))
        if [ x"$ota_upmode" = x"golden" ];then
            # synchronization uboot to uboot_bak
            part_id=$(mmc_part_number "uboot")
            #echo "[OTA_INFO] dd if=/dev/mmcblk0${part_id} of=/dev/mmcblk0${part_id} bs=512 count=2048  seek=2048 2> /dev/null"
            dd if=/dev/mmcblk0p${part_id} of=/dev/mmcblk0p${part_id} bs=512 count=2048  seek=2048 2> /dev/null
        fi
        update_partstatus

    fi

    set_updateflag $update_flag
    set_resetreason normal

    clear_update_file
}

function power_down_handle()
{
    local verify_file="${upgrade_file_path}/verify_tmp"
    get_continue_up_flag
    # flash may be not finished , not clear updata file
    if [ x"$resetreason" = x"normal" ];then
        if [ -d "${upgrade_file_path}/tmpdir_all" ];then
            if [ x"$continue_flag" = x"0" ];then
                clear_update_file
                echo "2" >> $update_result
                printinfo "Error: Power failure may occur during upgrade ! "
            elif [ x"$continue_flag" = x"1" ];then
                local file="${ota_logpath}/commend"
                local name=$(cat $file)
                local up_partition=${name%%/*}
                local image=${name##*/}
                if [ ! -f $file ];then
                    printinfo "Error：$file not exist"
                    echo "2" >> $update_result
                    echo "[OTA_PROGRESS]100%[SSERGORP_ATO]" >> $progress
                    exit 1
                fi

                file="/usr/bin/up_partition.sh"
                # mv name verify_tmp
                mv ${upgrade_file_path}/tmpdir_all $verify_file
                # continue updata
                printinfo "Power down continue $file $up_partition $image"
                $file $up_partition $image
            fi
        fi
    elif [ x"$resetreason" = x"all" ];then
        if [ x"$update_flag" = x"13" ];then
            power_down_version_verify
        elif [ x"$update_flag" = x"12" ];then
            return 0
        fi

    fi

}

function power_down_version_verify()
{
    local sys_version="/etc/version"
    local pkg_version="${upgrade_file_path}/tmpdir_all/version"
    local tmp_partstatus

    if [ ! -f $pkg_version ];then
        printinfo "warning: pkg version file not exist"
        return 0
    fi

    if [ ! -f $sys_version ];then
        printinfo "warning: sys version file not exist"
        return 0
    fi
    pkg_version=$(cat $pkg_version)
    sys_version=$(cat $sys_version)

    pkg_version=${pkg_version#*20}
    pkg_version=${pkg_version%% *}
    sys_version=${sys_version#*20}
    sys_version=${sys_version%% *}

    if [ x"$sys_version" \< x"$pkg_version" ];then
        printinfo "REBOOT: reversal AB partstatus"
        tmp_partstatus=$(($ab_partition_status ^ 0x1E))
        # try boot updata partition
        set_ab_partition_status $tmp_partstatus
        set_updateflag 14
        reboot
    elif [ x"$sys_version" -eq x"$pkg_version" ];then
        # updata success clear updata file
        printinfo "sys_version pkg_version equal , rm unzip package"
        echo "1" >> $update_result
        clear_update_file
        exit 1
    fi
}

function veeprom_init()
{
    # boot success set count 10, if count <=0 ,using bak partition
    set_count 0

    local file="/usr/bin/hrut_boardid"
    if [ ! -f $file  ];then
        printinfo "file $file not exist"
        exit 1
    fi

    file="/usr/bin/hrut_otastatus"
    ota_upmode=$($file g upmode)

    if [ -z "$ota_upmode" ] || [ x"$ota_upmode" != x"AB" -a x"$ota_upmode" != x"golden" ];then
        printinfo "veeprom init "

        board_id=$(cat /sys/class/socinfo/board_id)
        /usr/bin/hrut_boardid s $board_id
        tmp=$(/usr/bin/hrut_count 0)
        tmp=$(/usr/bin/hrut_otastatus s upflag 13)
        tmp=$(/usr/bin/hrut_otastatus s partstatus 0)
        # hrut_resetreason normal
        tmp=$(/usr/bin/hrut_eeprom w 0x9 $(echo -n "normal" | hexdump -e '/1 "0x%X "'))
        # hrut_otastatus s upmode golden
        tmp=$(/usr/bin/hrut_eeprom w 0x1d $(echo -n "golden" | hexdump -e '/1 "0x%X "'))
        # hrut_countflag s on
        tmp=$(/usr/bin/hrut_eeprom w 0x93 $(echo -n "on" | hexdump -e '/1 "0x%X "'))
    fi
}

function all()
{
    # get bootmode
    ota_get_boot_mode

    set_count 0

    # Init veeprom boardid
    board_id=$(cat /sys/class/socinfo/board_id)
    tmp=$(/usr/bin/hrut_boardid s $board_id)

    get_resetreason

    # make sure log file exist
    mkdir_log_file

    # ota pipe init
    if [ x"$resetreason" = x"recovery" ];then
        hbota_utility
    else
        rm -rf $command
        hbota_utility &
    fi
    # get ota upmode from veeprom
    get_upmode

    # get system boot reason
    get_resetreason

    # AB power down handle
    if [ x"$ota_upmode" = x"AB" ];then
        power_down_handle
    fi
    # synchronize data to device
    sync

    if [ x"$resetreason" = x"recovery" ] && [ x"$ota_upmode" = x"golden" ];then
        # recovery mode: update partitions
        entry_recovery_mode
    else
        # check ota update status
        check_otaflag
    fi
}

# gpt partition name
export UBOOT_PARTITION_NAME="uboot"
export UBOOT_PARTITION_BAK_NAME="uboot_b"
export KERNEL_PARTITION_NAME="boot"
export KERNEL_PARTITION_BAK_NAME="boot_b"
export VBMETA_PARTITION_NAME="vbmeta"
export VBMETA_PARTITION_BAK_NAME="vbmeta_b"
export SYSTEM_PARTITON_NAME="system"
export SYSTEM_PARTITON_BAK_NAME="system_b"
export PARTITION_NAME_SUFFIX=""
export BAK_PARTITION_NAME_SUFFIX="_b"

ota_logpath="/userdata/cache"
progress="${ota_logpath}/progress"
ota_info="${ota_logpath}/info"
update_result="${ota_logpath}/result"
upgrade_file_path="/userdata/cache"

update_flag=0
ota_upmode="AB"
ab_partition_status=0
reboot_flag=0
resetreason="normal"
ota_bootmode="emmc"

# update_flag=$(($update_flag & 0x1))
cache="/userdata/cache"
# ota command
command="${cache}/command"
#check utility
check_hrut_bin

all

sync
