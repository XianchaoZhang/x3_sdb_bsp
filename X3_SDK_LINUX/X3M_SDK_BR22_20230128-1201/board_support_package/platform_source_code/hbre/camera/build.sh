#!/bin/bash

function all()
{
    prefix=$TARGET_TMPROOTFS_DIR
	make -j${N} -C vcam_daemon
    # real build
	make -j${N} || {
		echo "make failed"
		exit 1
	}
    [ -z "${BUILD_OUTPUT_PATH}" ] && export BUILD_OUTPUT_PATH="$(pwd)"
    # put binaries to dest directory
    mkdir -p $prefix/lib/sensorlib
    mkdir -p $prefix/include/cam
    mkdir -p $prefix/etc/cam/
    mkdir -p $prefix/bin
    cpfiles "${BUILD_OUTPUT_PATH}/libcam.so" "$prefix/lib/"
    cpfiles "${BUILD_OUTPUT_PATH}/libcam.a" "$prefix/lib/"
    cpfiles "${BUILD_OUTPUT_PATH}/utility/vcam/*.so" "$prefix/lib/sensorlib/"
    cpfiles "${BUILD_OUTPUT_PATH}/utility/sensor/*.so" "$prefix/lib/sensorlib/"
    cpfiles "${BUILD_OUTPUT_PATH}/utility/deserial/*.so" "$prefix/lib/sensorlib/"
    cpfiles "./utility/sensor_calbration/*/*/*.so" "$prefix/etc/cam/"
    cpfiles "./utility/sensor_calbration/*/*/*.json" "$prefix/etc/cam/"
    cpfiles "cfg/*.json" "$prefix/etc/cam/"
    cpfiles "inc/*.h" "$prefix/include/cam/"
	cpfiles "${BUILD_OUTPUT_PATH}/vcam_daemon/vcam_daemon" "$prefix/bin/"
}

function all_32()
{
    prefix=$TARGET_TMPROOTFS_DIR
    make -C vcam_daemon
    # real build
    make all_32 -j${N} || {
        echo "make failed"
        exit 1
    }

    [ -z "${BUILD_OUTPUT_PATH}" ] && export BUILD_OUTPUT_PATH="$(pwd)"
    # put binaries to dest directory
    mkdir -p $prefix/lib/sensorlib
    mkdir -p $prefix/include/cam
    mkdir -p $prefix/etc/cam/
    mkdir -p $prefix/bin
    cpfiles "${BUILD_OUTPUT_PATH}/libcam.so" "$prefix/lib/"
    cpfiles "${BUILD_OUTPUT_PATH}/libcam.a" "$prefix/lib/"
    cpfiles "${BUILD_OUTPUT_PATH}/utility/vcam/*.so" "$prefix/lib/sensorlib/"
    cpfiles "${BUILD_OUTPUT_PATH}/utility/sensor/*.so" "$prefix/lib/sensorlib/"
    cpfiles "${BUILD_OUTPUT_PATH}/utility/deserial/*.so" "$prefix/lib/sensorlib/"
    cpfiles "cfg/*.json" "$prefix/etc/cam/"
    cpfiles "inc/*.h" "$prefix/include/cam/"
	cpfiles "${BUILD_OUTPUT_PATH}/vcam_daemon/vcam_daemon" "$prefix/bin/"
}
function clean()
{
	make -C vcam_daemon clean
    make clean
}

# include
. $INCLUDE_FUNCS
# include end
#dependon libisp
#dependon diagnose

cd $(dirname $0)

buildopt $1
