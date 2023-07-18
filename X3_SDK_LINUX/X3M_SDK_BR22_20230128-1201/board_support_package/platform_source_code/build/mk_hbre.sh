#!/bin/bash
# @Author: yaqiang.li
# @Date:   2022-02-08 22:53:32
# @Last Modified by:   yaqiang.li
# @Last Modified time: 2022-06-22 23:00:54

# hbre编译过程
# 1、 拷贝或者解压预编译的根文件系统到 prebuilt_rootfs 作为基准文件系统，为什么需要这个基准文件系统是因为编译hbre时有库依赖它
# 2、 拷贝 prebuilt_rootfs 为 appsdk
# 3、 生成 hbre.sh， 并且执行编译hbre， hbre生成的内容都存放在进 appsdk，部分测试程序存放在unittest目录
# 4、 对比 prebuilt_rootfs 和 appsdk， 把相同部分从appsdk里面删除，得到一个干净的 hbre 头文件和库文件包

set -e

######## setting utils_funcs ###################
source ./utils_funcs.sh

export HR_TOP_DIR=$(realpath $(cd $(dirname $0); pwd)/../)
export HR_LOCAL_DIR=$(realpath $(cd $(dirname $0); pwd))

# check board config
check_board_config ${@:1}

export HBRE_DIR=${HR_TOP_DIR}/hbre
export BUILD_OUT_DIR=${HR_IMAGE_DEPLOY_DIR}/hbre
export TARGET_TMPROOTFS_DIR=${HR_IMAGE_DEPLOY_DIR}/appsdk
export PREBUILT_ROOTFS_DIR=${HR_IMAGE_DEPLOY_DIR}/prebuilt_rootfs
export INCLUDE_FUNCS=${HR_LOCAL_DIR}/hbre_sub_scripts/includefuncs.sh

export TARGET_MODE="release"
export SRC_HOST_DIR=${HBRE_DIR} # for diagnose
export SRC_BUILD_DIR=${HR_LOCAL_DIR}/hbre_sub_scripts
export SRC_HBRE_DIR=${HBRE_DIR} # for libisp
export TARGET_TMPUNITTEST_DIR=${HR_IMAGE_DEPLOY_DIR}/appsdk/usr # for tuning_tool and hapi_test
export TOPDIR=${HR_TOP_DIR} # for libhapi
export TARGET_PROJECT=xj3 # for libdisp

export TEMP_HBRE_SCRIPT=${BUILD_OUT_DIR}/hbre.sh

[ ! -z ${BUILD_OUT_DIR} ] && [ ! -d ${BUILD_OUT_DIR} ] && mkdir -p ${BUILD_OUT_DIR}

# setting build action
if [ $# -eq 0 ] || [ "$1" = "all" ]; then
    action="all"
elif [ "$1" = "clean" ]; then
    action="clean"
    rm -rf ${TARGET_TMPROOTFS_DIR}
    rm -rf ${PREBUILT_ROOTFS_DIR}
else
    echo "Build action mismatch"
    exit 1
fi

rm -rf ${TARGET_TMPROOTFS_DIR}

# copy rootfs
mkdir -p ${TARGET_TMPROOTFS_DIR}
if [ x"yocto" = x"${HR_ROOTFS_TYPE}" ]; then
    mkdir -p ${PREBUILT_ROOTFS_DIR}
    if [ -d ${HR_ROOTFS_DIR}/root ]; then
        cp -arf ${HR_ROOTFS_DIR}/root/* ${PREBUILT_ROOTFS_DIR}
    fi
elif [ x"ubuntu" = x"${HR_ROOTFS_TYPE}" ]; then
    if [ ! -d ${PREBUILT_ROOTFS_DIR} ]; then
        mkdir -p ${PREBUILT_ROOTFS_DIR}
        cat ${HR_ROOTFS_DIR}/${HR_ROOTFS_PKG}[0-9] | tar -Jxf - -C ${PREBUILT_ROOTFS_DIR}
    fi

    # 修复无效软链接
    cd ${PREBUILT_ROOTFS_DIR}/usr/lib/aarch64-linux-gnu
    for linkfile in $(find -type l)
    do
        if [ ! -e ${linkfile} ]
        then
            orig_target=`ls -l ${linkfile} | awk -F' -> ' '{print $2}'`
            echo "ln -sf ../..${orig_target} ${linkfile}"
            ln -sf ../..${orig_target} ${linkfile}
        fi
    done
    # Do Hijack
    SRC_HIJACK_ROOTFS_DIR=${HR_ROOTFS_DIR}/hijack
    if [ -x ${SRC_HIJACK_ROOTFS_DIR}/do_hijack.sh ]; then
        ${SRC_HIJACK_ROOTFS_DIR}/do_hijack.sh ${SRC_HIJACK_ROOTFS_DIR} ${PREBUILT_ROOTFS_DIR} "false"
    fi
else
    echo "Rootfs type \'${HR_ROOTFS_TYPE}\' not supported"
    exit -1
fi

# 把预编译的根文件系统拷贝为appsdk
cp -arf ${PREBUILT_ROOTFS_DIR}/. ${TARGET_TMPROOTFS_DIR}
if [ -d ${HR_TOP_DIR}/appsdk ]; then
	cp -arf ${HR_TOP_DIR}/appsdk/. ${TARGET_TMPROOTFS_DIR}
fi

${SRC_BUILD_DIR}/xgenbuild.sh ${SRC_BUILD_DIR}/modules.manifest all $action || {
    exit 1
}

echo "start to build hbre"
${TEMP_HBRE_SCRIPT} ${SRC_BUILD_DIR}/modules.manifest $action || {
    echo "$TEMP_HBRE_SCRIPT ${SRC_BUILD_DIR}/modules.manifest $action failed"
exit 1
}

# Extract appsdk
echo "Extract appsdk"
export LC_ALL=C
diff -rs --no-dereference ${PREBUILT_ROOTFS_DIR} ${TARGET_TMPROOTFS_DIR} | grep "identical$" | awk -F ' and ' '{print $2}' | awk -F ' are identical' '{print "\""$1"\""}' | xargs rm -rf
unset LC_ALL

function delete_empty_dir()
{
    find ${1:-.} -mindepth 1 -maxdepth 1 -type d | while read -r dir
do
    if [[ -z "$(find "$dir" -mindepth 1 -type f)" ]] >/dev/null
    then
        rm -rf ${dir} 2>&- || echo "Delete error"
    fi
    if [ -d ${dir} ]
    then
        delete_empty_dir "$dir"
    fi
done
}

cd ${TARGET_TMPROOTFS_DIR}
echo "Delete empty dirs"
delete_empty_dir
cd -

if [ x"ubuntu" = x"${HR_ROOTFS_TYPE}" ]; then
    # 给ubuntu系统创建 /lib 和 /bin 软链接
    [ -L ${TARGET_TMPROOTFS_DIR}/bin ] || ln -sf usr/bin ${TARGET_TMPROOTFS_DIR}/bin
    [ -L ${TARGET_TMPROOTFS_DIR}/lib ] || ln -sf usr/lib ${TARGET_TMPROOTFS_DIR}/lib
fi

echo "Install third party components"
# Install third party components
mkdir -p ${TARGET_TMPROOTFS_DIR}/include
cp -arf ${HBRE_DIR}/third_party/usr/include/* ${TARGET_TMPROOTFS_DIR}/include
mkdir -p ${TARGET_TMPROOTFS_DIR}/usr/lib
cp -arf ${HBRE_DIR}/third_party/usr/lib/* ${TARGET_TMPROOTFS_DIR}/usr/lib
mkdir -p ${TARGET_TMPROOTFS_DIR}/usr/share
cp -arf ${HBRE_DIR}/third_party/usr/share/* ${TARGET_TMPROOTFS_DIR}/usr/share

if [ x"ubuntu" = x"${HR_ROOTFS_TYPE}" ]; then
    echo "create ${TARGET_TMPROOTFS_DIR}_without_third_party"
    # 复制一份不合并开源第三方库的appsdk出来，避免与安装在ubuntu系统里面的开源组件发生冲突
    rm -rf ${TARGET_TMPROOTFS_DIR}_without_third_party
    cp -arf ${TARGET_TMPROOTFS_DIR} ${TARGET_TMPROOTFS_DIR}_without_third_party
    export LC_ALL=C
    diff -rs --no-dereference ${HBRE_DIR}/third_party/usr/include ${TARGET_TMPROOTFS_DIR}_without_third_party/include | grep "identical$" | awk -F ' and ' '{print $2}' | awk -F ' are identical' '{print "\""$1"\""}' | xargs rm -rf
    diff -rs --no-dereference ${HBRE_DIR}/third_party/usr/lib ${TARGET_TMPROOTFS_DIR}_without_third_party/usr/lib | grep "identical$" | awk -F ' and ' '{print $2}' | awk -F ' are identical' '{print "\""$1"\""}' | xargs rm -rf
    diff -rs --no-dereference ${HBRE_DIR}/third_party/usr/share ${TARGET_TMPROOTFS_DIR}_without_third_party/usr/share | grep "identical$" | awk -F ' and ' '{print $2}' | awk -F ' are identical' '{print "\""$1"\""}' | xargs rm -rf
    unset LC_ALL
    cd ${TARGET_TMPROOTFS_DIR}_without_third_party
    echo "Delete empty dirs of ${TARGET_TMPROOTFS_DIR}_without_third_party"
    delete_empty_dir
    cd -
fi

function strip_elf() {
    for f in $(find $TARGET_TMPROOTFS_DIR/ -path $TARGET_TMPROOTFS_DIR/etc/fusa -a -prune -o -type f -print); do
        fm=$(file $f)
        slim=${fm##*, }
        if [ "${slim}" = "not stripped" ]; then
            ${CROSS_COMPILE}strip $f
        fi
    done
}

strip_elf
