#!/bin/sh
set -e
# this is the first update script which will be executed
# add something you want

deploy_ota_dir="${IMAGE_DEPLOY_DIR}/ota"

update_sh="${HR_LOCAL_DIR}/uboot_package_maker/update.sh"
warning_txt="${deploy_ota_dir}/uboot/warning.txt"
gpt="${deploy_ota_dir}/gpt.conf"
version="${deploy_ota_dir}/version"
uboot_uptime="30"

if [ "$1" = "nand" ];then
    target="uboot"
else
    target="uboot"
fi

image_file=${IMAGE_DEPLOY_DIR}/uboot.img

if [ -z ${image_file} ];then
    "Usage: build uboot, before make update package"
    exit 1
fi

rm -rf ${deploy_ota_dir}/${target}.zip

mkdir -p ${deploy_ota_dir}/script
mkdir -p ${deploy_ota_dir}/info
mkdir -p ${deploy_ota_dir}/uboot

if [ x"$1" = x"emmc" ];then
    zip -j ${deploy_ota_dir}/${target}.zip $gpt
fi

# warning file: wait_before_query, estimate and msg
touch ${warning_txt}
echo "wait_before_query: $uboot_uptime" > ${warning_txt}
echo "estimate: $uboot_uptime" >> ${warning_txt}
echo "msg: the update will take about $uboot_uptime seconds" >> ${warning_txt}

zip -j -y ${deploy_ota_dir}/${target}.zip ${update_sh} ${warning_txt} $version ${image_file}

mkdir -p ${deploy_ota_dir}/script
mkdir -p ${deploy_ota_dir}/info

cp -rf ${update_sh} ${deploy_ota_dir}/script/${target}_update.sh
mv ${warning_txt} ${deploy_ota_dir}/info/${target}_warning.txt
