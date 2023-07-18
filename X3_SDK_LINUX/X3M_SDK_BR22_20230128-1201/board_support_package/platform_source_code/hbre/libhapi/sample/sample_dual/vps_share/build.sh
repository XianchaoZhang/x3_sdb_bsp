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
	install -p -D -m 755 ${BUILD_OUTPUT_PATH}/vps_share $prefix/bin/vps_share
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
#dependon libion

cd $(dirname $0)

buildopt $1
