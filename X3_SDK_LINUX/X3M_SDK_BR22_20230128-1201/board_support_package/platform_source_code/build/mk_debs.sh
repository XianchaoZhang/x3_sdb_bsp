#!/bin/bash

# set -x
set -e

######## setting utils_funcs ###################
source ./utils_funcs.sh

export HR_TOP_DIR=$(realpath $(cd $(dirname $0); pwd)/../)
export HR_LOCAL_DIR=$(realpath $(cd $(dirname $0); pwd))

# check board config
check_board_config ${@:1}

# 编译出来的镜像保存位置
export IMAGE_DEPLOY_DIR=${HR_IMAGE_DEPLOY_DIR}
[ ! -z $IMAGE_DEPLOY_DIR ] && [ ! -d $IMAGE_DEPLOY_DIR ] && mkdir $IMAGE_DEPLOY_DIR

Version="${HR_BUILD_VERSION}"
#不推荐使用gen_contrl_file生成control,推荐使用gen_contrl_file_standard
function gen_contrl_file () {
    control_path=$1
    Package=hobot-$ARCH-$2
    Description=$3
    Architecture=$ARCH
    Maintainer="technical_support@horizon.ai"
    if [ ! -f ${control_path}/control ];then
        touch ${control_path}/control
    fi
    cat <<-EOF > ${control_path}/control
	Package:${Package}
	Version:${Version}
	Architecture:${ARCH}
	Maintainer:${Maintainer}
	Description:${Description}
	EOF
}
#推荐使用
function gen_contrl_file_standard () {
    control_path=$1
    Package=hobot-$2
    Description=$3
    Architecture=$ARCH
    Maintainer="technical_support@horizon.ai"
    if [ ! -f ${control_path}/control ];then
        touch ${control_path}/control
    fi
    cat <<-EOF > ${control_path}/control
	Package:${Package}
	Version:${Version}
	Architecture:${ARCH}
	Maintainer:${Maintainer}
	Description:${Description}
	EOF
}


debs_src_dir="${HR_ROOTFS_DIR}/deb_packages"
debs_dst_dir="${IMAGE_DEPLOY_DIR}/debs"

rm -rf $debs_dst_dir
mkdir -p $debs_dst_dir

# 需要预先编译好的组件
APPSDK_DIR=${IMAGE_DEPLOY_DIR}/appsdk_without_third_party
KO_INSTALL_DIR=${IMAGE_DEPLOY_DIR}/ko_install
PREROOTFS_DIR=${IMAGE_DEPLOY_DIR}/prebuilt_rootfs

if [ ! -d ${APPSDK_DIR} ];then
    echo "${APPSDK_DIR} does not exist"
    exit -1
fi

if [ ! -d ${KO_INSTALL_DIR} ];then
    echo "${KO_INSTALL_DIR} does not exist"
    exit -1
fi

if [ ! -d ${PREROOTFS_DIR} ];then
    echo "${PREROOTFS_DIR} does not exist"
    exit -1
fi
#推荐使用
function make_ubuntu_debs(){
    select=$1
    desc=$2
    #命名规范：hobot-包名_版本_架构
    deb_name=hobot-${select}_${Version}_$ARCH
    deb_dst_dir=$debs_dst_dir/${deb_name}
    deb_src_dir=$debs_src_dir/hobot-${select}
    is_allowed=0
    rm -rf $deb_dst_dir
    echo deb_dst_dir = $deb_dst_dir
    mkdir -p $deb_dst_dir
    cp -a ${deb_src_dir}/* ${deb_dst_dir}/
    gen_contrl_file_standard "${deb_dst_dir}/DEBIAN" ${select} "$desc"
    cat ${deb_dst_dir}/DEBIAN/control
    echo start $FUNCNAME : ${deb_dst_dir}/${deb_name}.deb
    case $select in
    sp-cdev)
            mkdir -p $deb_dst_dir/usr/lib
            mkdir -p $deb_dst_dir/usr/include
            cp -arf "${APPSDK_DIR}/sp_cdev/lib/libhb_dnn.so" "$deb_dst_dir/usr/lib/"
            cp -arf "${APPSDK_DIR}/hb_ubuntu_py/hobot-srcampy/libsrcampy.so" "$deb_dst_dir/usr/lib/libspcdev.so"
            cp -arf ${APPSDK_DIR}/sp_cdev/include/*.h  $deb_dst_dir/usr/include/
            is_allowed=1
            ;;
        *)
            echo "err select : (sp-cdev)"
            is_allowed=0
            ;;
    esac
    if [ $is_allowed == 1 ];then
        echo "Installed-Size: `du -ks ${deb_dst_dir}|cut -f 1`" >> "${deb_dst_dir}/DEBIAN/control"
        dpkg-deb --root-owner-group -b $deb_dst_dir $deb_dst_dir.deb
    fi
}
#不推荐使用make_a_xj3_deb制作deb，推荐使用make_ubuntu_debs
function make_a_xj3_deb (){
    select=$1
    desc=$2
    deb_name=hobot-$ARCH-${select}_${Version}
    deb_dst_dir=$debs_dst_dir/${deb_name}
    deb_src_dir=$debs_src_dir/hobot-${select}
    is_allowed=0
    rm -rf $deb_dst_dir
    echo deb_dst_dir = $deb_dst_dir
    mkdir -p $deb_dst_dir
    cp -a ${deb_src_dir}/* ${deb_dst_dir}/
    gen_contrl_file "${deb_dst_dir}/DEBIAN" ${select} "$desc"
    cat ${deb_dst_dir}/DEBIAN/control
    echo start $FUNCNAME : ${deb_dst_dir}/${deb_name}.deb
    case $select in
        all)
            # make_ubuntu_debs
            echo "make_ubuntu_debs do no thing!"
            ;;
        configs)
            mkdir -p "$deb_dst_dir/etc/"
            mkdir -p "$deb_dst_dir/usr/bin/"
            # for sensor and isp tuning
            cp -a "${APPSDK_DIR}/etc/vio" "$deb_dst_dir/etc/"
            cp -a "${APPSDK_DIR}/etc/tuning_tool" "$deb_dst_dir/etc/"
            cp -a "${APPSDK_DIR}/etc/control-tool" "$deb_dst_dir/etc/"
            cp -a "${APPSDK_DIR}/usr/bin/tuning_tool" "$deb_dst_dir/usr/bin/"
            cp -a "${APPSDK_DIR}/usr/bin/act-server" "$deb_dst_dir/usr/bin/"
            echo "${HR_BSP_VERSION}" > $deb_dst_dir/etc/version
            is_allowed=1
            ;;
        modules)
            deb_dst_ko_dir=${deb_dst_dir}/lib/modules/4.14.87
            mkdir -p $deb_dst_dir/lib/modules $deb_dst_dir/lib/modules-load.d/ ${deb_dst_ko_dir}
            rm -rf $deb_dst_dir/lib/modules-load.d/hobot_mod.conf
            ls ${KO_INSTALL_DIR}/lib/modules/*/*.ko > $deb_dst_dir/lib/modules/modules.list.tmp
            for mod in `cat $deb_dst_dir/lib/modules/modules.list.tmp`
            do
                modname=`basename $mod`
                modname=${modname%.*}
                echo "# ${modname}" >> $deb_dst_dir/lib/modules-load.d/hobot_mod.conf
            done
            rm -rf $deb_dst_dir/lib/modules/modules.list.tmp
            cp -a ${KO_INSTALL_DIR}/lib/modules $deb_dst_dir/lib/
            echo rm -rf ${deb_dst_ko_dir}/{build,source}
            rm -rf ${deb_dst_ko_dir}/{build,source}

            # del Ap6212 wifi and bt ko
            # del hobot_hdmi_lt8618.ko
            ko_list=(cfg80211.ko rfkill.ko \
                brcmfmac.ko brcmutil.ko bcmdhd.ko \
                ecdh_generic.ko bluetooth.ko btbcm.ko hci_uart.ko \
                hobot_hdmi_lt8618.ko)

            for ko in ${ko_list[@]}
            do
                if [ -f ${deb_dst_ko_dir}/${ko} ];then
                    rm -rf  ${deb_dst_ko_dir}/${ko}
                fi
            done

            if [[ ! -f $deb_dst_dir/lib/modules-load.d/hobot_mod.conf ]];then
                echo "failed to create $deb_dst_dir/lib/modules-load.d/hobot_mod.conf"
            fi

            # set Depends
            echo "Depends:hobot-arm64-boot" >> "${deb_dst_dir}/DEBIAN/control"
            is_allowed=1
            ;;
        libs)
            set -x
            mkdir -p $deb_dst_dir/usr/lib/hobot $deb_dst_dir/etc/ld.so.conf.d/ $deb_dst_dir/lib/
            cp -a ${APPSDK_DIR}/usr/lib/* $deb_dst_dir/usr/lib/
            sed -n 's/\/lib/\/usr\/lib/gp' ${PREROOTFS_DIR}/etc/ld.so.conf > $deb_dst_dir/etc/ld.so.conf.d/hobot.conf
            echo "/usr/lib/hbmedia" >> $deb_dst_dir/etc/ld.so.conf.d/hobot.conf
            echo "/usr/lib/sensorlib" >> $deb_dst_dir/etc/ld.so.conf.d/hobot.conf
            echo "/usr/lib/hbbpu" >> $deb_dst_dir/etc/ld.so.conf.d/hobot.conf
            is_allowed=1
            unset x
            ;;
        includes)
            mkdir -p $deb_dst_dir/usr/include
            cp -arf ${APPSDK_DIR}/include/* $deb_dst_dir/usr/include/

            mkdir -p $deb_dst_dir/usr/share
            cp -arf ${APPSDK_DIR}/usr/share/OpenCV $deb_dst_dir/usr/share
            is_allowed=1
            ;;
        tools)
            is_allowed=0
            ;;
        bins)
            mkdir -p $deb_dst_dir/usr/bin
            ls ${APPSDK_DIR}/usr/bin/hrut* > /dev/null
            if [ $? -eq 0 ];then
                echo "cp -a ${APPSDK_DIR}/usr/bin/hrut* $deb_dst_dir/usr/bin"
                cp -a ${APPSDK_DIR}/usr/bin/hrut* $deb_dst_dir/usr/bin
            fi
            cp -a "${APPSDK_DIR}/usr/bin/procrank" "$deb_dst_dir/usr/bin/"
            is_allowed=1
            ;;
        gpiopy)
            mkdir -p $deb_dst_dir/usr/bin
            hb_gpiopy_dir=${APPSDK_DIR}/hb_ubuntu_py/hb_gpio_py
            echo "cp -af ${hb_gpiopy_dir}/*pi-config $deb_dst_dir/usr/bin"
            cp -af ${hb_gpiopy_dir}/*pi-config $deb_dst_dir/usr/bin

            if [ -f ${hb_gpiopy_dir}/hb_dtb_tool ];then
                echo "cp -a ${hb_gpiopy_dir}/hb_dtb_tool $deb_dst_dir/usr/bin"
                cp -a ${hb_gpiopy_dir}/hb_dtb_tool $deb_dst_dir/usr/bin
            fi

            if [ -f ${hb_gpiopy_dir}/hobot-gpio.tar.gz ];then
                echo "cp -a ${hb_gpiopy_dir}/hobot-gpio.tar.gz $deb_dst_dir/usr/ib/hobot-gpio"
                mkdir -p $deb_dst_dir/usr/lib/hobot-gpio
                cp -a ${hb_gpiopy_dir}/hobot-gpio.tar.gz $deb_dst_dir/usr/lib/hobot-gpio
            fi

            mkdir -p $deb_dst_dir/app/
            cp -a ${APPSDK_DIR}/hb_ubuntu_py/40pin_samples $deb_dst_dir/app/

            is_allowed=1
            ;;
        srcampy)
            mkdir -p $deb_dst_dir/usr/lib/hobot-srcampy
            cp -arf ${APPSDK_DIR}/hb_ubuntu_py/hobot-srcampy/*.whl $deb_dst_dir/usr/lib/hobot-srcampy

            is_allowed=1
            ;;
        hdmi-dvb)
            is_allowed=0
            ;;
        hdmi-sdb)
            mkdir -p $deb_dst_dir/usr/bin

            deb_dst_ko_dir=${deb_dst_dir}/lib/modules/4.14.87
            mkdir -p ${deb_dst_ko_dir}
            cp -af ${KO_INSTALL_DIR}/lib/modules/4.14.87/hobot_hdmi_lt8618.ko ${deb_dst_ko_dir}
            is_allowed=1
            ;;
        sdb-ap6212)
            ko_src_dir=${KO_INSTALL_DIR}/lib/modules/4.14.87
            deb_dst_ko_dir=${deb_dst_dir}/lib/modules/4.14.87
            mkdir -p ${deb_dst_ko_dir}

            ko_list=(cfg80211.ko rfkill.ko \
                brcmfmac.ko brcmutil.ko bcmdhd.ko \
                ecdh_generic.ko bluetooth.ko btbcm.ko hci_uart.ko)

            for ko in ${ko_list[@]}
            do
                if [ -f ${ko_src_dir}/${ko} ];then
                    cp -af ${ko_src_dir}/${ko} ${deb_dst_ko_dir}
                fi
            done
            is_allowed=1
            ;;
        dnn-python)
            is_allowed=1
            ;;
        desktop)
           is_allowed=1
           ;;
        boot)
            boot_dest_dir=${deb_dst_dir}/boot
            kernel_src_dir=${HR_TOP_DIR}/boot/kernel
            kernel_version="4.14.87"
            mkdir -p ${boot_dest_dir}
            cp -arf ${kernel_src_dir}/.config ${boot_dest_dir}/config-${kernel_version}
            cp -arf ${kernel_src_dir}/.config ${boot_dest_dir}/config-${kernel_version}.old
            cp -arf ${kernel_src_dir}/System.map ${boot_dest_dir}/System.map-${kernel_version}
            cp -arf ${kernel_src_dir}/System.map ${boot_dest_dir}/System.map-${kernel_version}.old
            cp -arf ${kernel_src_dir}/arch/arm64/boot/Image ${boot_dest_dir}/vmlinuz-${kernel_version}
            mkdir -p ${boot_dest_dir}/hobot
            cp -arf ${kernel_src_dir}/arch/arm64/boot/dts/hobot/*.dtb ${boot_dest_dir}/hobot
            is_allowed=1
           ;;
        *)
            echo "err select : (all|configs|modules|includes|libs|bins)"
            is_allowed=0
            ;;
    esac
    if [ $is_allowed == 1 ];then
        echo "Installed-Size: `du -ks ${deb_dst_dir}|cut -f 1`" >> "${deb_dst_dir}/DEBIAN/control"
        dpkg-deb --root-owner-group -b $deb_dst_dir $deb_dst_dir.deb
    fi
}

function gen_contrl_file2 () {
    control_path=$1
    Package=hobot-linux-headers
    Description=$3
    Architecture=$ARCH
    Maintainer="technical_support@horizon.ai"
    if [ ! -f ${control_path}/control ];then
        touch ${control_path}/control
    fi
    cat <<-EOF > ${control_path}/control
	Package:${Package}
	Version:${Version}
	Architecture:${ARCH}
	Maintainer:${Maintainer}
	Description:${Description}
	EOF
}

function make_kernel_headers()
{
    deb_name=hobot-kernel-headers_${Version}
    deb_dst_dir=${debs_dst_dir}/${deb_name}
    deb_src_dir=${debs_src_dir}/hobot-kernel-headers
    rm -rf ${deb_dst_dir}
    echo deb_dst_dir = ${deb_dst_dir}
    mkdir -p ${deb_dst_dir}
    cp -a ${deb_src_dir}/* ${deb_dst_dir}/
    gen_contrl_file2 "${deb_dst_dir}/DEBIAN" kernel-headers "Linux kernel headers for 4.14.87 on arm64
 This package provides kernel header files for 4.14.87 on arm64.
 This is useful for people who need to build external modules.
 Generally used for building out-of-tree kernel modules."
    cat ${deb_dst_dir}/DEBIAN/control
    echo start $FUNCNAME : ${deb_dst_dir}/${deb_name}.deb

    SRCDIR=${HR_TOP_DIR}/boot/kernel
    HDRDIR=${deb_dst_dir}/usr/src/linux-headers-4.14.87
    mkdir -p ${HDRDIR}

    cd ${SRCDIR}

    mkdir -p ${HDRDIR}/arch
    cp -Rf ${SRCDIR}/arch/arm64        ${HDRDIR}/arch/
    cp -Rf ${SRCDIR}/include           ${HDRDIR}
    cp -Rf ${SRCDIR}/scripts           ${HDRDIR}
    cp -Rf ${SRCDIR}/Module.symvers    ${HDRDIR}
    cp -Rf ${SRCDIR}/Makefile          ${HDRDIR}
    cp -Rf ${SRCDIR}/System.map        ${HDRDIR}
    cp -Rf ${SRCDIR}/.config           ${HDRDIR}
    cp -Rf ${SRCDIR}/security          ${HDRDIR}
    cp -Rf ${SRCDIR}/tools             ${HDRDIR}
    cp -Rf ${SRCDIR}/certs             ${HDRDIR}

    rm -rf ${HDRDIR}/arch/arm64/boot

    cd ${SRCDIR}
    cp --parents -Rf `find -iname "KConfig*"`   ${HDRDIR}
    cp --parents -Rf `find -iname "Makefile*"`  ${HDRDIR}
    cp --parents -Rf `find -iname "*.pl"`       ${HDRDIR}
    cd ${HR_LOCAL_DIR}

    find ${HDRDIR} -depth -name '.svn' -type d  -exec rm -rf {} \;

    find ${HDRDIR} -depth -name '*.c' -type f -exec rm -rf {} \;

    exclude=("*.c" \
            "*.o" \
            "*.S" \
            "*.s" \
            "*.ko" \
            "*.cmd" \
            "*.a" \
            "modules.builtin" \
            "modules.order")
    for element in ${exclude[@]}
    do
    find ${HDRDIR} -depth -name ${element} -type f -exec rm -rf {} \;
    done

    cd ${SRCDIR}
    cp --parents -Rf `find scripts -iname '*.c'` ${HDRDIR}
    make M=${HDRDIR}/scripts clean

    cd ${HR_LOCAL_DIR}
    rm -rf ${HDRDIR}/arch/arm64/mach*
    rm -rf ${HDRDIR}/arch/arm64/plat*

    mv ${HDRDIR}/include/asm-generic/ ${HDRDIR}/
    rm -rf ${HDRDIR}/inclde/asm-*
    mv ${HDRDIR}/asm-generic ${HDRDIR}/include/

    rm -rf ${HDRDIR}/arch/arm64/configs

    rm -rf ${HDRDIR}/debian

    # set Depends
    echo "Depends:hobot-arm64-bins" >> "${deb_dst_dir}/DEBIAN/control"
    echo "Installed-Size: `du -ks ${HDRDIR}|cut -f 1`" >> "${deb_dst_dir}/DEBIAN/control"

    dpkg-deb --root-owner-group -b ${deb_dst_dir} ${deb_dst_dir}.deb
}

make_a_xj3_deb "dnn-python" "DNN inference library and sample program for python interface"
make_a_xj3_deb "configs"    "The installation package includes the files in /etc"
make_a_xj3_deb "bins"       "The installation package includes the files in /usr/bin"
make_a_xj3_deb "libs"       "The installation package includes the files in /usr/lib/hobot"
make_a_xj3_deb "includes"   "The installation package includes the files in /usr/include"
make_a_xj3_deb "modules"    "The installation package includes the files in /lib/modules"
make_a_xj3_deb "tools"      "The installation package includes the hobotpi-config"
make_a_xj3_deb "gpiopy"     "The installation package includes the gpio python libs"
make_a_xj3_deb "srcampy"    "The installation package includes the vio camera python libs"
make_a_xj3_deb "hdmi-sdb"   "The installation package includes the hdmi driver, configs and start scripts"
make_a_xj3_deb "sdb-ap6212" "The installation package includes the wifi and bt driver, configs and start scripts"
make_a_xj3_deb "desktop"    "The package include desktop configures"
make_a_xj3_deb "boot"       "The package include linux kerel image and devicetree configs"
make_ubuntu_debs "sp-cdev"    "The package include bpu interface and vio interface"

make_kernel_headers
