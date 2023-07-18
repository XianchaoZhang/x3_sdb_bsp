#!/bin/bash

function all()
{
    prefix=$TARGET_TMPUNITTEST_DIR

    # real build
    make -j${N} || {
        echo "make failed"
        exit 1
    }

    # put binaries to dest directory
    cpfiles "${BUILD_OUTPUT_PATH}/hapi_bpu" "${prefix}/bin/hapi_xj3/ipc/"
    cpfiles "startdh_8a10.sh" "${prefix}/bin/hapi_xj3/ipc/"
    cpfiles "startdh_imx327.sh" "${prefix}/bin/hapi_xj3/ipc/"
    cpfiles "sample.yuv" "${prefix}/bin/hapi_xj3/ipc/"
    cpfiles "detection.hbm" "${prefix}/bin/hapi_xj3/ipc/"
    cpfiles "os8a10.bin" "${prefix}/bin/hapi_xj3/ipc/"
    cpfiles "lib/libhbrt_bernoulli_aarch64.so" "${prefix}/bin/hapi_xj3/ipc/"
}

function all_32()
{
    prefix=$TARGET_TMPUNITTEST_DIR

    # real build
    make all_32 -j${N} || {
        echo "make failed"
        exit 1
    }

    # put binaries to dest directory
    cpfiles "${BUILD_OUTPUT_PATH}/hapi_bpu" "${prefix}/bin/hapi_xj3/ipc/"
    cpfiles "startdh_8a10.sh" "${prefix}/bin/hapi_xj3/ipc/"
    cpfiles "startdh_imx327.sh" "${prefix}/bin/hapi_xj3/ipc/"
    cpfiles "sample.yuv" "${prefix}/bin/hapi_xj3/ipc/"
    cpfiles "detection.hbm" "${prefix}/bin/hapi_xj3/ipc/"
    cpfiles "os8a10.bin" "${prefix}/bin/hapi_xj3/ipc/"
    cpfiles "lib/libhbrt_bernoulli_aarch64.so" "${prefix}/bin/hapi_xj3/ipc/"
}

function clean()
{
    make clean
}

# include
. $INCLUDE_FUNCS
# include end
#dependon camera
#dependon libisp
#dependon libdisp
#dependon libmm
#dependon libvio
#dependon libhapi
#dependon libion
cd $(dirname $0)

buildopt $1
