#!/bin/bash

function all()
{
    prefix=$TARGET_TMPROOTFS_DIR

    # real build
    make -j${N} || {
        echo "make failed"
        exit 1
    }
    make -j${N} -C bpu_utility
    make -j${N} -C hbm_utility
    [ -z "${BUILD_OUTPUT_PATH}" ] && export BUILD_OUTPUT_PATH="$(pwd)"
    # put binaries to dest directory
    mkdir -p $prefix/usr/bin
    cpfiles "${BUILD_OUTPUT_PATH}/_install/*" "$prefix/usr/bin"
    cpfiles "utility/prebuilds/*" "$prefix/usr/bin"
    mkdir -p $prefix/lib

    cpfiles "${BUILD_OUTPUT_PATH}/hrut_bpu" "$prefix/usr/bin"
    cpfiles "${BUILD_OUTPUT_PATH}/hrut_hbm" "$prefix/usr/bin"

    mkdir -p $prefix/etc/rcS.d
    mkdir -p $prefix/etc/rc5.d
    # put ota file to dest directory
    cpfiles "ota_utility/hrut_checkinfo" "$prefix/usr/bin"
    cpfiles "ota_utility/hrut_checkprogress" "$prefix/usr/bin"
    cpfiles "ota_utility/hrut_checksuccess" "$prefix/usr/bin"
    cpfiles "ota_utility/hrut_otaextrainfo" "$prefix/usr/bin"
    cpfiles "ota_utility/hrut_syswhitelist" "$prefix/usr/bin"
    cpfiles "ota_utility/otaupdate" "$prefix/usr/bin"
    cpfiles "ota_utility/up_partition.sh" "$prefix/usr/bin"
    cpfiles "ota_utility/hbota_utility" "$prefix/usr/bin"
    cpfiles "ota_utility/S9updatecheck.sh" "$prefix/etc/rcS.d"
    cpfiles "ota_utility/S46_otaserver.sh" "$prefix/etc/rc5.d"

    # put userdata_whitelist to /usr/bin
    cpfiles "ota_utility/userdata_whitelist" "$prefix/usr/bin"

    # put pubkey.pem to dest directory
    if [ x"$OTA_SIGNED" == x"true" ]; then
        cpfiles "$SRC_BUILD_DIR/ota_tools/ota_sign_tools/pubkey.pem" "$prefix/etc"
    else
        cpfiles "ota_utility/pubkey.pem" "$prefix/etc"
    fi
}

function all_32()
{
    all
}

function clean()
{
    make clean
    make -C hrut_cleanuserdata clean
    make -C bpu_utility clean
    make -C hbm_utility clean
}

# include
. $INCLUDE_FUNCS
# include end

cd $(dirname $0)

buildopt $1
