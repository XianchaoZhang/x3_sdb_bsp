#!/bin/sh
set -e
# this is the first update script which will be executed
# add something you want
target=$1

deploy_ota_dir="${IMAGE_DEPLOY_DIR}/ota/"

update_sh="${HR_LOCAL_DIR}/system_package_maker/update.sh"
warning_txt="${deploy_ota_dir}/system/warning.txt"
gpt="${deploy_ota_dir}/gpt.conf"
version="${deploy_ota_dir}/version"

system_uptime="40"
if [ -z "$1" ];then
    echo "Usage: $0 <emmc | nor | nand>"
    exit 1
fi

rm -rf ${deploy_ota_dir}/system.zip

if [ x"$target" != x"emmc" -a x"$target" != x"nor" -a x"$target" != x"nand" ];then
    echo "Usage: $0 <emmc | nor | nand>"
    exit 1
fi

# warning file: wait_before_query, estimate and msg
mkdir -p ${deploy_ota_dir}/system
touch ${warning_txt}

mkdir -p ${deploy_ota_dir}/script
mkdir -p ${deploy_ota_dir}/info

if [ x"$target" = x"nor" ];then
    system_uptime="140"
elif [ x"$target" = x"nand" ];then
    system_uptime="80"
else
    # creating emmc ota package
    zip -j -y ${deploy_ota_dir}/system.zip $gpt
fi
echo "wait_before_query: $system_uptime" > ${warning_txt}
echo "estimate: $system_uptime" >> ${warning_txt}
echo "msg: the update will take about $system_uptime seconds" >> ${warning_txt}

zip -j -y ${deploy_ota_dir}/system.zip $update_sh ${warning_txt} $version ${IMAGE_DEPLOY_DIR}/system.img

mkdir -p ${deploy_ota_dir}/script
mkdir -p ${deploy_ota_dir}/info

cp -rf $update_sh ${deploy_ota_dir}/script/system_update.sh
mv ${warning_txt} ${deploy_ota_dir}/info/system_warning.txt
