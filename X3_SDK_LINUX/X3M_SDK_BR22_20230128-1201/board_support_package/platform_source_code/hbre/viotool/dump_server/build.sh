#!/bin/bash

function all()
{
    prefix=$TARGET_TMPROOTFS_DIR

    # real build
    make -j${N} || {
         echo "make failed"
         exit 1
       }

    # put binaries to dest directory

    cpfiles "${BUILD_OUTPUT_PATH}/*.so" "$prefix/lib/hbmedia"
    cpfiles "${BUILD_OUTPUT_PATH}/*.a" "$prefix/lib/hbmedia"
    mkdir -p "$prefix/etc/tuning_tool/"
    cpfiles "./cfg/*.json" "$prefix/etc/tuning_tool/"
}

function all_32()
{
    prefix=$TARGET_TMPROOTFS_DIR

    # real build
    make all_32 -j${N} || {
        echo "make failed"
        exit 1
    }

    # put binaries to dest directory

    cpfiles "./*.so" "$prefix/lib/"
    mkdir -p "$prefix/include/vio_tool/"
    cpfiles "./*.h" "$prefix/include/vio_tool/"
}

function clean()
{
    make clean
}

# include
. $INCLUDE_FUNCS
# include end
#dependon libvio
#dependon libisp
#dependon libhapi

cd $(dirname $0)

buildopt $1
