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

function usage()
{
    echo "Using: "
    echo "    ./up_partition.sh partition_name file_name"
    echo "    partition_name: all | uboot | boot | system"
    echo "    file_name: update img's name"
}

function printinfo()
{
    local log=$1

    echo "[OTA_INFO] ${log}" > /dev/ttyS0

    # upgrade nor, system is read only, not write log file
    if [ x"$nor_upimg_finish" = x"false" ];then
        echo "[OTA_INFO]${log}[OFNI_ATO]" >> $ota_info
    fi
}

function printprogress()
{
    local log=$1

    printinfo "${log}"

    # upgrade nor, system is read only, not write log file
    if [ x"$nor_upimg_finish" = x"false" ];then
        echo "[OTA_PROGRESS]${log}[SSERGORP_ATO]" >> $progress
    fi
}

function check_hrut_bin()
{
    otastatus_bin="/usr/bin/hrut_otastatus"
    resetreason_bin="/usr/bin/hrut_resetreason"
    count_bin="/usr/bin/hrut_count"
    for bin in $otastatus_bin $resetreason_bin $count_bin; do
        if [ ! -f $bin ];then
            printinfo "Error: file $bin not found!"
            echo "2" >> $update_result
            exit 1
        fi
    done
}

function get_ab_partition_status()
{
    ab_partition_status=$($otastatus_bin g partstatus)
    printinfo "get ab_partition_status $ab_partition_status"
}

function get_upmode()
{
    ota_upmode=$($otastatus_bin g upmode)
    printinfo "get ota_upmode $ota_upmode"
}

function update_flag()
{
    $otastatus_bin s upflag $1
    local flag=$($otastatus_bin g upflag )

    if [ x"$flag" = x"$1" ];then
        printinfo "update flag $flag success ! "
    else
        printinfo "Error：update flag $flag failed !"
        echo "2" >> $update_result
    fi
}

function set_resetreason()
{
    tmp=$(${resetreason_bin} $1)
    printinfo "update resetreason $1 success"
}

function set_count()
{
    local count=$($count_bin $1)
    printinfo "set_count $count"
}

function ota_update_flag()
{
    # update flag
    #update_flag 10

    # set resetreason
    local partition_tmp=${partition%_b*}
    printinfo "set resetreason $partition_tmp"
    set_resetreason $partition_tmp

    # set AB mode partstatus
    if [ x"$ota_upmode" = x"AB" ];then
        if [ x"$partition_tmp" = x"all" ];then
            ab_partition_status=$(($ab_partition_status ^ 0x1E))
        elif [ x"$partition_tmp" != x"app" -a x"$partition_tmp" != x"userdata" ];then
            abstatus_to_number $partition_tmp
            local suffix=$?

            ab_partition_status=$(($ab_partition_status ^ (0x1 << $suffix) ))
        fi
        tmp=$($otastatus_bin s partstatus $ab_partition_status)
        printinfo "update status $ab_partition_status finish "
    fi
}

function ota_update_flag_rear()
{
    partition=${partition%_b*}

    if [ x"$partition" = x"all" ];then
        set_count 23
        printinfo "${ota_upmode} mode"
    fi

    clear_update_file
}

function ota_get_boot_mode()
{
    local file="/sys/class/socinfo/boot_mode"

    if [ ! -f $file ];then
        printinfo "warning：not enable socinfo boot_mode support"
        mode="0"
    else
        mode=$(cat $file)
    fi

    if [ x"$mode" = x"0" ];then
        ota_bootmode="emmc"
    elif [ x"$mode" = x"5" ];then
        ota_bootmode="nor_flash"
    elif [ x"$mode" = x"1" ];then
        ota_bootmode="nand_flash"
    elif [ x"$mode" = x"2" ];then
        ota_bootmode="ap"
    elif [ x"$mode" = x"3" ];then
        ota_bootmode="uart"
    else
        ota_bootmode="emmc"
    fi

    printinfo "boot mode: $ota_bootmode"
}

function parameter_init()
{
    # get partition and file name
    local file="${ota_logpath}/commend"
    local verify_file="${upgrade_file_path}/verify_tmp"
    if [ ! -f $file ];then
        printinfo "Error：$file not exist"
        return 1
    fi

    # get update img
    local suffix=${image##*.}
    if [ x"$suffix" = x"zip" ];then
        if [ x"$partition" = x"all" ];then
            file="${upgrade_file_path}/tmpdir_all"
        else
            file="${upgrade_file_path}/tmpdir"
        fi

        # get update flag
        up_flag=$($otastatus_bin g upflag)

        # when power down up_flag 2
        if [ x"$up_flag" != x"2" ];then
            rm -rf $file
            mv $verify_file $file
            sync
        fi
    fi

    # get system partition status
    if [ x"$ota_upmode" = x"AB" ];then
        get_ab_partition_status
    fi

    # get ota update mode
    get_upmode

    printinfo "parameter and file_path check pass"
    printprogress "30%"

    # get ota boot mode
    ota_get_boot_mode
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

function update_nor_partition_img()
{
    local file=$1
    local partition=$2

    mtd_number_get $partition

    if [ x"$ota_upmode" = x"golden" ];then
        printinfo "flashcp -A -v $file ${mtd_device}"
        flashcp -A -v $file ${mtd_device}
        return $?
    else
        printinfo "Error: nor flash not support $ota_upmode mode"
        return 1
    fi
}

function update_partition_img()
{
    local file=$1
    local partition=$2
    local eeprom_value

    if [ x"$ota_bootmode" = x"nor_flash" ];then
        update_nor_partition_img $file $partition
        ret=$?
        return $ret
    else
        if [ x"$ota_upmode" = x"golden" ];then
            part_id=$(mmc_part_number $partition)

            if [ x"$partition" = x"all" ];then
                umount /app
                printinfo "read eeprom value"
                eeprom_value=$(hrut_eeprom d)
                eeprom_value=${eeprom_value#*:}

                printinfo "dd if=${file} of=/dev/mmcblk0 bs=512"
                dd if=${file} of=/dev/mmcblk0 bs=512 2> /dev/null
                ret=$?
                printinfo "write eeprom value"
                hrut_eeprom w 0 ${eeprom_value}
            fi

            if [ x"$partition" = x"uboot" ];then
                printinfo "dd if=${file} of=/dev/mmcblk0p${part_id} bs=512 count=1024"
                dd if=${file} of=/dev/mmcblk0p${part_id} bs=512 count=1024 2> /dev/null
                ret=$?
            else
                printinfo "dd if=${file} of=/dev/mmcblk0p${part_id} bs=512"
                dd if=${file} of=/dev/mmcblk0p${part_id} bs=512 2> /dev/null
                ret=$?
            fi
        elif [ x"$ota_upmode" = x"AB" ];then
            abstatus_to_number $partition
            local suffix=$?
            part_status=$((($ab_partition_status >> $suffix) ^ 0x1))

            if [ x"$part_status" = x"0" ] && [ x"$partition" != x"app" ];then
                partition=${partition}_b
            fi

            if [ x"$partition" = x"uboot_b" ];then
                part_id=$(mmc_part_number "uboot")
                printinfo "dd if=${file} of=/dev/mmcblk0p${part_id} bs=512 count=1024 seek=1024"
                dd if=${file} of=/dev/mmcblk0p${part_id} bs=512 count=1024 seek=1024 2> /dev/null
                ret=$?
            elif [ x"$partition" = x"uboot" ];then
                part_id=$(mmc_part_number $partition)
                printinfo "dd if=${file} of=/dev/mmcblk0p${part_id} bs=512 count=1024"
                dd if=${file} of=/dev/mmcblk0p${part_id} bs=512 count=1024 2> /dev/null
                ret=$?
            else
                part_id=$(mmc_part_number $partition)
                printinfo "dd if=${file} of=/dev/mmcblk0p${part_id} bs=512"
                dd if=${file} of=/dev/mmcblk0p${part_id} bs=512 2> /dev/null
                ret=$?
            fi
        fi
    fi

    return $ret
}

function all_in_one_img_write()
{
    local ret="0"
    local file_exist="0"
    local file

    for part in uboot vbmeta boot system app all;do
        file="${upgrade_file_path}/tmpdir_all/${part}.img"
        sync
        if [ -f $file ];then
            update_partition_img $file $part
            ret=$?

            if [ x"$ret" = x"0" ];then
                printinfo "update $part img success !"
            else
                upgrade_failed_reason="Error：write $part image failed !"
                return $ret
            fi

            file_exist="1"
        fi
    done

    if [ x"$file_exist" = x"0" ];then
        upgrade_failed_reason="Error: file update.sh or partition image not exist! "
        ret="1"
    fi

    return $ret
}

function update_img()
{
    local file="${upgrade_file_path}/tmpdir/update.sh"
    local ret

    if [ x"$partition" = x"all" ];then
        # update all_in_one.zip
        file="${upgrade_file_path}/tmpdir_all/update.sh"

        printinfo "$file $ab_partition_status"
        sync
        if [ -f $file ];then
            chmod +x $file
            printinfo "$file $ab_partition_status $ota_bootmode $upgrade_file_path"
            $file $ab_partition_status $ota_bootmode $upgrade_file_path
        else
            # write .img file to partition
            all_in_one_img_write
        fi
        ret=$?
    else
        if [ x"$ota_bootmode" = x"nor_flash" ];then
            # nor flash ota update
            # when find update.sh, progress update.sh
            part_id="NULL"
            sync
            if [ -f $file ];then
                chmod +x $file
                $file $part_id $ota_bootmode $upgrade_file_path
                ret=$?
            else
                local file_path="${upgrade_file_path}/tmpdir"
                local upgrade_file=${file_path}/${partition%_b*}.img
                if [ ! -f $upgrade_file ];then
                    upgrade_failed_reason="Error: partition file ${partition%_b*}.img not exist !"
                    ret="1"
                else
                    update_nor_partition_img $upgrade_file $partition
                    ret=$?
                fi
            fi
            nor_upimg_finish="true"
        else
            # emmc ota update
            part_id=$(mmc_part_number $partition)
            sync
            # when find update.sh, progress update.sh
            if [ -f $file ];then
                chmod +x $file
                printinfo "$file $part_id $ota_bootmode $upgrade_file_path"
                $file $part_id $ota_bootmode $upgrade_file_path
                ret=$?
            else
                local file_path="${upgrade_file_path}/tmpdir"
                local upgrade_file=${file_path}/${partition%_b*}.img

                if [ ! -f $upgrade_file ];then
                    upgrade_failed_reason="Error: file update.sh or ${partition%_b*}.img not exist !"
                    ret="1"
                else
                    if [ x"$partition" = x"uboot_b" ];then
                        printinfo "dd if=${upgrade_file} of=/dev/mmcblk0p${part_id} bs=512 count=1024 seek=1024"
                        dd if=${upgrade_file} of=/dev/mmcblk0p${part_id} bs=512 count=1024 seek=1024 2> /dev/null
                    elif [ x"$partition" = x"uboot" ];then
                        printinfo "dd if=${upgrade_file} of=/dev/mmcblk0p${part_id} bs=512 count=1024"
                        dd if=${upgrade_file} of=/dev/mmcblk0p${part_id} bs=512 count=1024 2> /dev/null
                    else
                        printinfo "dd if=${upgrade_file} of=/dev/mmcblk0p${part_id} bs=512"
                        dd if=${upgrade_file} of=/dev/mmcblk0p${part_id} bs=512 2> /dev/null
                    fi
                    ret=$?
                fi
            fi
        fi
    fi

    if [ x"$ret" = x"0" ];then
        printinfo "update_img success!"
        printprogress "90%"
    else
        update_flag 13
        ret="1"
    fi
    sync

    return $ret
}


function all_incremental_update_img()
{
    local patch_tool=/usr/bin/bspatch
    local upgrade_file
    local file_path="${upgrade_file_path}/tmpdir_all"
    local curr_ab_part
    local ret="0"
    if [ x"$ota_bootmode" = x"nor_flash" ];then
        printinfo "NOT suported nor_flash incremental updata"
    else
        # emmc ota update
        if [ x"$ota_upmode" = x"golden" ];then
            for part in uboot boot system ;do
                part_id=$(mmc_part_number $part)
                upgrade_file=${file_path}/$part/inc_${part}.img
                if [ -f $upgrade_file ];then
                    if [ x"$part" = x"uboot" ];then
                        printinfo "update uboot partition"

                        printinfo "dd if=/dev/mmcblk0p${part_id} of=$file_path/$part/uboot_bak.img bs=512 count=2048"
                        dd if=/dev/mmcblk0p${part_id} of=$file_path/$part/uboot_bak.img bs=512 count=2048

                        printinfo "$patch_tool /dev/mmcblk0p${part_id} /dev/mmcblk0p${part_id} $upgrade_file"
                        $patch_tool /dev/mmcblk0p${part_id} /dev/mmcblk0p${part_id} $upgrade_file
                        md5sum_updata_check uboot $part_id
                        if [ x"$check_ret" = x"false" ];then
                            return  "1"
                        fi

                        printinfo "dd if=$file_path/$part/uboot_bak.img of=/dev/mmcblk0p${part_id} bs=512 count=2048 seek=2048 2> /dev/null"
                        dd if=$file_path/$part/uboot_bak.img of=/dev/mmcblk0p${part_id} bs=512 count=2048 seek=2048 2> /dev/null
                        sync

                        printprogress "45%"
                    else
                        printinfo "update $part partition"
                        #part_id=$(mmc_part_number $part)
                        if [ x"$part" = x"boot" ];then
                            vbmeta_id=$(($part_id - 1))
                            printinfo "$patch_tool /dev/mmcblk0p${vbmeta_id} /dev/mmcblk0p${vbmeta_id} ${file_path}/$part/inc_vbmeta.img"
                            $patch_tool /dev/mmcblk0p${vbmeta_id} /dev/mmcblk0p${vbmeta_id} ${file_path}/$part/inc_vbmeta.img
                            md5sum_updata_check vbmeta $vbmeta_id
                            if [ x"$check_ret" = x"false" ];then
                                return  "1"
                            fi
                            printinfo "$patch_tool /dev/mmcblk0p${part_id} /dev/mmcblk0p${part_id} $upgrade_file"
                            $patch_tool /dev/mmcblk0p${part_id} /dev/mmcblk0p${part_id} $upgrade_file
                            md5sum_updata_check boot $part_id
                            if [ x"$check_ret" = x"false" ];then
                                return  "1"
                            fi
                            printprogress "60%"
                        else
                            printinfo "$patch_tool /dev/mmcblk0p${part_id} /dev/mmcblk0p${part_id} $upgrade_file"
                            $patch_tool /dev/mmcblk0p${part_id} /dev/mmcblk0p${part_id} $upgrade_file
                            md5sum_updata_check system $part_id
                            if [ x"$check_ret" = x"false" ];then
                                return  "1"
                            fi
                            printprogress "75%"
                        fi

                    fi
                else
                    printinfo "Error: $upgrade_file not exist"
                    echo "2" >> $update_result
                    exit 1
                fi
            done

        elif [ x"$ota_upmode" = x"AB" ];then

            for part in uboot boot system ;do
                abstatus_to_number $part
                local abstatus_number=$?
                upgrade_file=${file_path}/$part/inc_${part}.img
                part_status=$((($ab_partition_status >> $abstatus_number) & 0x1))

                if [ x"$part_status" = x"1" ];then
                    ab_part=${part}
                    curr_ab_part=${part}_b
                else
                    ab_part=${part}_b
                    curr_ab_part=${part}
                fi
                part_id=$(mmc_part_number $ab_part)
                curr_part_id=$(mmc_part_number $curr_ab_part)

                if [ -f $upgrade_file ];then
                    if [ x"$ab_part" = x"boot" ] || [ x"$ab_part" = x"boot_b" ];then
                        vbmeta_id=$(($part_id - 2))
                        curr_vbmeta_id=$(($curr_part_id - 2))
                        printinfo "$patch_tool /dev/mmcblk0p${curr_vbmeta_id} /dev/mmcblk0p${vbmeta_id} ${file_path}/$part/inc_vbmeta.img"
                        $patch_tool /dev/mmcblk0p${curr_vbmeta_id} /dev/mmcblk0p${vbmeta_id} ${file_path}/$part/inc_vbmeta.img
                        md5sum_updata_check vbmeta $vbmeta_id
                        if [ x"$check_ret" = x"false" ];then
                            return  "1"
                        fi
                        sync

                        printinfo "$patch_tool /dev/mmcblk0p${curr_part_id} /dev/mmcblk0p${part_id} $upgrade_file"
                        $patch_tool /dev/mmcblk0p${curr_part_id} /dev/mmcblk0p${part_id} $upgrade_file
                        md5sum_updata_check $ab_part $part_id
                        if [ x"$check_ret" = x"false" ];then
                            return  "1"
                        fi
                        sync
                        printprogress "60%"

                    elif [ x"$ab_part" = x"uboot" ] || [ x"$ab_part" = x"uboot_b" ];then
                        printinfo "$patch_tool /dev/mmcblk0p${curr_part_id} /dev/mmcblk0p${part_id} $upgrade_file"
                        $patch_tool /dev/mmcblk0p${curr_part_id} /dev/mmcblk0p${part_id} $upgrade_file
                        md5sum_updata_check $ab_part $part_id
                        if [ x"$check_ret" = x"false" ];then
                            return  "1"
                        fi
                        sync
                        printprogress "45%"
                    else
                        printinfo "$patch_tool /dev/mmcblk0p${curr_part_id} /dev/mmcblk0p${part_id} $upgrade_file"
                        $patch_tool /dev/mmcblk0p${curr_part_id} /dev/mmcblk0p${part_id} $upgrade_file
                        md5sum_updata_check $ab_part $part_id
                        if [ x"$check_ret" = x"false" ];then
                            return  "1"
                        fi
                        sync
                        printprogress "75%"
                    fi
                else
                    printinfo "Error: $upgrade_file not exist"
                    echo "2" >> $update_result
                    exit 1
                fi

            done

        fi

    fi

    printinfo "update_img success!"
    printprogress "90%"
    sync
    return $ret


}

function all_incremental_update()
{
    local patch_tool=/usr/bin/bspatch
    local file_path="${upgrade_file_path}/tmpdir_all"
    #local unzip_file_path="${upgrade_file_path}/tmpdir_all/tmpdir"
    local unzip_dir
    local file_exist="0"
    local file

    for part in uboot  boot system  ;do
        unzip_dir=$file_path/${part}
        mkdir $unzip_dir
        file="${upgrade_file_path}/tmpdir_all/${part}_inc.zip"
        sync
        if [ -f $file ];then
            unzip -o -q $file -d $unzip_dir
        else
            printinfo "Error: $file not exist"
            echo "2" >> $update_result
            exit 1
        fi
    done
    # update all_in_one_inc
    all_incremental_update_img

    return $?

}

function app_incremental_updata()
{
    local file="${upgrade_file_path}/tmpdir/update.sh"
    local ret
    if [ x"$ota_bootmode" = x"emmc" ];then
        # when find update.sh, progress update.sh
        if [ -f $file ];then
            chmod +x $file
            printinfo "$file $ota_bootmode $upgrade_file_path"
            $file $ota_bootmode $upgrade_file_path
            ret=$?
        else
            printinfo "Error: app update file not exist"
            ret=1
        fi
    fi
    return $ret
}

function incremental_update_img()
{
    local patch_tool=/usr/bin/bspatch
    local file_path="${upgrade_file_path}/tmpdir"
    local ret="0"
    if [ x"$ota_bootmode" = x"nor_flash" ];then
        printinfo "NOT suported nor_flash incremental updata"
    else
        # emmc ota incremental update
        if [ x"$ota_upmode" = x"golden" ];then
            part_id=$(mmc_part_number $partition)
            local upgrade_file=${file_path}/inc_${partition}.img

            if [ ! -f $upgrade_file ];then
                upgrade_failed_reason="Error:  ${upgrade_file}.img not exist !"
                ret="1"
            elif [ x"$partition" = x"uboot" ];then
                printinfo "dd if=/dev/mmcblk0p${part_id} of=$file_path/uboot_bak.img bs=512 count=2048"
                dd if=/dev/mmcblk0p${part_id} of=$file_path/uboot_bak.img bs=512 count=2048

                printinfo "$patch_tool /dev/mmcblk0p${part_id} /dev/mmcblk0p${part_id} $upgrade_file"
                $patch_tool /dev/mmcblk0p${part_id} /dev/mmcblk0p${part_id} $upgrade_file
                md5sum_updata_check $partition $part_id
                if [ x"$check_ret" = x"false" ];then
                    return  "1"
                fi

                printinfo "dd if=$file_path/uboot_bak.img of=/dev/mmcblk0p${part_id} bs=512 count=2048 seek=2048 2> /dev/null"
                dd if=$file_path/uboot_bak.img of=/dev/mmcblk0p${part_id} bs=512 count=2048 seek=2048 2> /dev/null
            elif [ x"$partition" = x"system" ];then
                printinfo "$patch_tool /dev/mmcblk0p${part_id} /dev/mmcblk0p${part_id} $upgrade_file"
                $patch_tool /dev/mmcblk0p${part_id} /dev/mmcblk0p${part_id} $upgrade_file
                md5sum_updata_check $partition $part_id
                if [ x"$check_ret" = x"false" ];then
                    return  "1"
                fi
            elif [ x"$partition" = x"boot" ];then
                vbmeta_id=$(($part_id - 1))
                printinfo "$patch_tool /dev/mmcblk0p${vbmeta_id} /dev/mmcblk0p${vbmeta_id} ${file_path}/inc_vbmeta.img"
                $patch_tool /dev/mmcblk0p${vbmeta_id} /dev/mmcblk0p${vbmeta_id} ${file_path}/inc_vbmeta.img
                md5sum_updata_check vbmeta $vbmeta_id
                if [ x"$check_ret" = x"false" ];then
                    return  "1"
                fi

                printinfo "$patch_tool /dev/mmcblk0p${part_id} /dev/mmcblk0p${part_id} $upgrade_file"
                $patch_tool /dev/mmcblk0p${part_id} /dev/mmcblk0p${part_id} $upgrade_file
                md5sum_updata_check $partition $part_id
                if [ x"$check_ret" = x"false" ];then
                    return  "1"
                fi

            fi
        elif [ x"$ota_upmode" = x"AB" ];then
            local ab_partition_tmp=${partition%_b*}
            local ab_upgrade_file=${file_path}/inc_${ab_partition_tmp}.img
            local curr_part
            if [ ! -f $ab_upgrade_file ];then
                upgrade_failed_reason="Error:  ${ab_upgrade_file}.img not exist !"
                printinfo "###upgrade_failed_reason is $upgrade_failed_reason "
                ret="1"
            else
                curr_part=${partition#*_}
                part_id=$(mmc_part_number $partition)
                if [ x"$curr_part" = x"b" ];then
                    curr_part_id=$(mmc_part_number ${partition%_b*})
                else
                    curr_part=${partition}_b
                    curr_part_id=$(mmc_part_number $curr_part)
                fi

                if [ x"$partition" = x"boot_b" ] || [ x"$partition" = x"boot" ] ;then
                    vbmeta_id=$(($part_id - 2))
                    curr_vbmeta_id=$(($curr_part_id - 2))
                    printinfo "$patch_tool /dev/mmcblk0p${curr_vbmeta_id} /dev/mmcblk0p${vbmeta_id} ${file_path}/inc_vbmeta.img"
                    $patch_tool /dev/mmcblk0p${curr_vbmeta_id} /dev/mmcblk0p${vbmeta_id} ${file_path}/inc_vbmeta.img
                    md5sum_updata_check vbmeta $vbmeta_id
                    if [ x"$check_ret" = x"false" ];then
                        return  "1"
                    fi

                    printinfo "$patch_tool /dev/mmcblk0p${curr_part_id} /dev/mmcblk0p${part_id} $ab_upgrade_file"
                    $patch_tool /dev/mmcblk0p${curr_part_id} /dev/mmcblk0p${part_id} $ab_upgrade_file
                    md5sum_updata_check boot $part_id
                    if [ x"$check_ret" = x"false" ];then
                        return  "1"
                    fi
                else
                    printinfo "$patch_tool /dev/mmcblk0p${curr_part_id} /dev/mmcblk0p${part_id} $ab_upgrade_file"
                    $patch_tool /dev/mmcblk0p${curr_part_id} /dev/mmcblk0p${part_id} $ab_upgrade_file
                    md5sum_updata_check $partition $part_id
                    if [ x"$check_ret" = x"false" ];then
                        return  "1"
                    fi

                fi
            fi
        fi

    fi

    printinfo "update_img success!"
    printprogress "90%"
    sync
    return $ret
}

function md5sum_updata_check()
{
    local file=${upgrade_file_path}/tmpdir_all/incremental.txt
    local check_partition
    local md5sum_value
    local md5sum_value_real
    local part=$1
    local check_part_id=$2

    if [ ! -f $file ];then
        file=${upgrade_file_path}/tmpdir/incremental.txt
    fi

    for line in $(cat $file);do
        check_partition=$(echo "$line" |  awk -F ":" '{print $1}')
        if [ x"$check_partition" = x"${part%_b*}" ];then
            md5sum_value=$(echo "$line" |  awk -F ":" '{print $2}')
            md5sum_value_real=$(dd if=/dev/mmcblk0p${check_part_id} 2> /dev/null |md5sum |awk '{print $1}')
            if [ x"$md5sum_value" = x"$md5sum_value_real" ];then
                check_ret="ok"
                printinfo "$part md5sum :$md5sum_value_real check Success !"
            else
                check_ret="false"
                upgrade_failed_reason="Error: $part md5sum :$md5sum_value_real check failed !"
                echo "2" >> $update_result
            fi
            break
        fi

    done

}

function clear_update_file()
{
    rm -rf ${upgrade_file_path}/tmpdir
    rm -rf ${upgrade_file_path}/tmpdir_all
    sync
}

function set_app_upgrade_flag()
{
    # update flag
    update_flag 13

    # update reasetreason
    set_resetreason normal

    printinfo "write $partition img to $ota_bootmode success "
    printprogress "100%"

    echo "1" >> $update_result
}

function ota_deinit_app()
{
    # /mnt/deinit.sh
    local deinit_file="/mnt/deinit.sh"
    if [ -f "$deinit_file" ];then
        chmod +x $deinit_file
        $deinit_file
    fi

    # /app/deinit.sh
    deinit_file="/app/deinit.sh"
    if [ -f "$deinit_file" ];then
        chmod +x $deinit_file
        $deinit_file
    fi

    # /userdata/deinit.sh
    deinit_file="/userdata/deinit.sh"
    if [ -f "$deinit_file" ];then
        chmod +x $deinit_file
        $deinit_file
    fi
}

function app_log_process()
{
    if [ x"$partition" = x"app" ];then
        if [ -f $app_log ];then
            local app_error_log=$(tail -n 1 $app_log)

            if [ x"$app_error_log" != x" " ];then
                printinfo "${app_error_log}"
            fi
        fi
    fi
}

function all()
{
    # init parameter: partition, file, name
    parameter_init
    local ret=$?
    if [ x"$ret" != x"0" ];then
        # parameter check failed, return normal status
        set_resetreason normal
        printinfo "Error：ota update parameter check failed !"
        clear_update_file
        echo "2" >> $update_result
        return
    fi

    # deinit app
    if [ x"$partition" != x"app" ];then
        ota_deinit_app
    fi

    # write image
    if [ x"$partition" = x"all" ];then
        if [ ! -f ${upgrade_file_path}/tmpdir_all/incremental.txt ];then
            update_img
        else
            # write all_in_one incremental image
            all_incremental_update
        fi
    else
        if [ ! -f ${upgrade_file_path}/tmpdir/incremental.txt ];then
            update_img
        else
            if [ x"$partition" = x"app" ];then
                # start app incremental update
                app_incremental_updata
            else
                # write partition incremental image
                incremental_update_img
            fi
        fi
    fi

    ret=$?
    if [ x"$ret" = x"0" ];then

        # after write image: update partstatus and resetreson
        ota_update_flag

        if [ x"$partition" = x"app" ];then
            # update upflag, resetreason and log file
            set_app_upgrade_flag
        else
            # after write image: update partstatus resetreson
            #ota_update_flag
            # after write image: update flag
            update_flag 14

            printprogress "95%"

            printinfo "write $partition img to $ota_bootmode success "
            # update status set count 23
            ota_update_flag_rear
            printprogress "100%"

            printinfo "reboot ... "
            sync

            # delay 2s
            sleep 2
            reboot
        fi
    else
        # update failed, return normal status
        set_resetreason normal

        if [ ! -z "$upgrade_failed_reason" ];then
            printinfo "$upgrade_failed_reason"
        else
            printinfo "Error：ota update $partition failed !"
        fi

        # handle app log info
        app_log_process

        clear_update_file
        echo "2" >> $update_result
        return
    fi
}

partition=$1
image=$2
upgrade_file_path=$3

ota_upmode="AB"
ab_partition_status=0
update_flag=10
ota_bootmode="emmc"
nor_upimg_finish="false"
upgrade_failed_reason=""
mtd_device="/dev/mtd0"

ota_logpath="/userdata/cache"
if [ "$(cat /sys/class/socinfo/boot_mode)" != "0" ] || [ ! -d "$ota_logpath" ];then
    ota_logpath="/tmp/cache"
    mkdir -p ${ota_logpath}
fi

progress="${ota_logpath}/progress"
ota_info="${ota_logpath}/info"
app_log="${ota_logpath}/applog"
update_result="${ota_logpath}/result"

# check ota utility bin
check_hrut_bin

ota_get_boot_mode

if [ -z $upgrade_file_path ];then
    upgrade_file_path="${ota_logpath}"
    image="${ota_logpath}/${image}"
fi

all
sync
