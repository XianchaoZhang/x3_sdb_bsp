#!/bin/bash

function all()
{
    prefix=$TARGET_TMPUNITTEST_DIR

    # real build
    make -j${N} all || {
        echo "make failed"
        exit 1
    }

    # put binaries to dest directory
    install -p -D -m 755 ${BUILD_OUTPUT_PATH}/vin_vps_test $prefix/bin/vin_vps_test
}

function all_32()
{
	all
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

cd $(dirname $0)

buildopt $1
