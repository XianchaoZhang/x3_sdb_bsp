#!/bin/bash

set -e
dir=$1
system_id=$2
config=$GPT_CONFIG
# nand_config=${SRC_DEVICE_DIR}/$TARGET_VENDOR/$TARGET_PROJECT/nand_ubi_sys.cfg
dir_partition_size=""
if [ -z $dir ] || [ -z $system_id ];then
    echo "Usage: $0 dir system_id"
    exit 1
fi

echo "*******************make dir: $dir system_id: $system_id vbmeta image******************"

function get_partition_size()
{
    local partition_name=$1
    local part_found="false"
    for line in $(cat $config)
    do
        if [ "$BOOT_MODE" = "emmc" -o "$BOOT_MODE" = "nor" ];then
            arr=(${line//:/ })

            local needparted=${arr[0]}
            local _name=${arr[1]}
            local dir_tmp=${_name%/*}
            local starts=${arr[3]}
            local start=${starts%?}
            local stops=${arr[4]}
            local stop=${stops%?}

            if [ x"$dir_tmp" = x"$partition_name" ];then
                echo "dir:$dir_tmp  start: $start stop: $stop"
                dir_partition_size=$((512 * $((${stop} - ${start} + 1))))
                break
            fi
        elif [ "$BOOT_MODE" = "nand" ];then
            if [ "$line" = "[$partition_name]" ];then
                part_found="true"
            fi
            if [ "$part_found" = "true" ] && [[ "$line" = *"vol_size"* ]];then
                local size_num=(${line//[!0-9]/})
                if [[ "$line" = *"MiB"* ]];then
                    dir_partition_size=$(( size_num * 1024 *1024 ))
                elif [[ "$line" = *"KiB"* ]];then
                    dir_partition_size=$(( size_num * 1024 ))
                else
                    dir_partition_size=$size_num
                fi
                break
            fi
        fi
    done
}

rm -rf $dir.img
cp -rf ${TARGET_DEPLOY_DIR}/$dir.img ./

mkdir -p out
mkdir -p images

if [ "$BOOT_MODE" = "nand" ];then
    config=$nand_config
fi

# git dir parttiton size
get_partition_size ${dir}

python2 avbtool add_hash_footer \
    --hash_algorithm sha256 \
    --partition_name $dir \
    --partition_size $dir_partition_size \
    --image $dir.img \
    --key keys/shared.priv.pem \
    --algorithm SHA256_RSA2048 || {
    echo "make boot vbmeta add footer $dir failed!"
    exit 1
}

rm -rf $TARGET_DEPLOY_DIR/$dir.img
cp $dir.img $TARGET_DEPLOY_DIR/$dir.img

mv $dir.img images
