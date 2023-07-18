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
    cpfiles "${BUILD_OUTPUT_PATH}/tuning_tool" "${prefix}/bin/"

}

function all_32()
{
    prefix=$TARGET_TMPUNITTEST_DIR

    # real build
    make all_32 -j${N} || {
        echo "make failed"
        exit 1
    }

    cpfiles "${BUILD_OUTPUT_PATH}/tuning_tool" "${prefix}/bin/"
}

function clean()
{
    make clean
}

# include
. $INCLUDE_FUNCS
# include end
#dependon camera
#dependon libvio
#dependon libisp
#dependon libdisp
#dependon libhapi

cd $(dirname $0)

buildopt $1
