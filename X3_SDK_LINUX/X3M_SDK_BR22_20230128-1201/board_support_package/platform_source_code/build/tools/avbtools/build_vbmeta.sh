#!/bin/bash
rm -rf out/vbmeta.img

include_boot="--include_descriptors_from_image images/boot.img"

if [ "$HR_ROOTFS_AVB_VERITY" = "true" -a "$HR_BOOT_MODE" = "emmc" ];then
    include_system="--include_descriptors_from_image images/system.img"
else
    include_system=""
fi

echo "********************** system dm *****************"
python2 avbtool make_vbmeta_image \
    --output out/vbmeta.img \
    --algorithm SHA256_RSA2048 \
    --key keys/shared.priv.pem \
    ${include_boot} \
    ${include_system} || {
    echo "make vbmeta_dm.img with recovery failed!"
    exit 1
}

rm -rf $TARGET_DEPLOY_DIR/vbmeta.img
cp out/vbmeta.img $TARGET_DEPLOY_DIR/
