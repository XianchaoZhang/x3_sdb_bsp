#!/bin/sh
set -e
# this is the first update script which will be executed
# add something you want

target=$1

deploy_ota_dir="${IMAGE_DEPLOY_DIR}/ota/"

update_sh="${HR_LOCAL_DIR}/kernel_package_maker/update.sh"
warning_txt="${deploy_ota_dir}/boot/warning.txt"
gpt="${deploy_ota_dir}/gpt.conf"
version="${deploy_ota_dir}/version"

if [ -z "$1" ];then
    echo "Usage: $0 <emmc | nor | nand>"
    exit 1
fi

boot_uptime="20"

rm -rf ${deploy_ota_dir}/boot.zip

if [ x"$target" != x"emmc" -a x"$target" != x"nor" -a x"$target" != x"nand" ];then
    echo "Usage: $0 <emmc | nor | nand>"
    exit 1
fi

# warning file: wait_before_query, estimate and msg
mkdir -p ${deploy_ota_dir}/boot
touch ${warning_txt}

mkdir -p ${deploy_ota_dir}/script
mkdir -p ${deploy_ota_dir}/info

if [ x"$target" = x"nor" ];then
    boot_uptime="70"
elif [ x"$target" = x"nand" ];then
    boot_uptime="35"
else
    # creating emmc ota package
    zip -j -y ${deploy_ota_dir}/boot.zip $gpt
fi
echo "wait_before_query: $boot_uptime" > ${warning_txt}
echo "estimate: $boot_uptime" >> ${warning_txt}
echo "msg: the update will take about $boot_uptime seconds" >> ${warning_txt}

zip -j -y ${deploy_ota_dir}/boot.zip ${update_sh} ${warning_txt} ${version} ${IMAGE_DEPLOY_DIR}/vbmeta.img ${IMAGE_DEPLOY_DIR}/boot.img

mkdir -p ${deploy_ota_dir}/script
mkdir -p ${deploy_ota_dir}/info

cp -rf ${update_sh} ${deploy_ota_dir}/script/boot_update.sh
mv ${warning_txt} ${deploy_ota_dir}/info/boot_warning.txt

exit 0
