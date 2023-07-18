#!/bin/bash

set -e

system_partition_size=$1
system_fs_type=$2
if [ -z "$system_partition_size" ];then
    echo "usage: $0 partiton_size"
    exit 1
fi

rm -rf out/vbmeta_system.img
cp -rf $TARGET_DEPLOY_DIR/${HR_ROOTFS_PART_NAME}.img ./

file="$TARGET_DEPLOY_DIR/${HR_ROOTFS_PART_NAME}_output/system_max_data_size"
partition_max_data_size=$(cat $file)
partition_max_data_size=$(($partition_max_data_size/512))

system_partition_size=$1
echo "system_partition_size: $system_partition_size"
echo $partition_max_data_size

verity_table="$TARGET_DEPLOY_DIR/${HR_ROOTFS_PART_NAME}_output/verity_info"
# bootargs="earlycon loglevel=8 console=ttyS0 clk_ignore_unused"
bootargs="rootfstype=$system_fs_type ro root=/dev/dm-0 rootwait skip_initramfs init=/init"
dm_para="1 vroot none ro 1,0 $partition_max_data_size verity"
verity_info=$(cat $verity_table)

dm_para="$dm_para $verity_info"
dm_para="$dm_para 2 restart_on_corruption ignore_zero_blocks"

echo "$bootargs $dm_para"

echo "python2 avbtool add_hashtree_footer"
python2 avbtool add_hashtree_footer \
    --hash_algorithm sha256 \
    --partition_name ${HR_ROOTFS_PART_NAME} \
    --partition_size $system_partition_size \
    --image ${HR_ROOTFS_PART_NAME}.img \
    --key keys/shared.priv.pem \
    --algorithm SHA256_RSA2048 \
    --no_hashtree \
    --do_not_generate_fec \
    --output_vbmeta_image out/vbmeta_system.img \
    --kernel_cmdline "$bootargs dm=\"$dm_para\"" || {
    echo "make boot vbmeta add footer $dir failed!"
    exit 1
}

mv ${HR_ROOTFS_PART_NAME}.img images/${HR_ROOTFS_PART_NAME}.img
