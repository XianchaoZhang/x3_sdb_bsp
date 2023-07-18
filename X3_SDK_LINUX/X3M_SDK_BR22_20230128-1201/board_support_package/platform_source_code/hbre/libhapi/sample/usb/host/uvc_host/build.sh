#!/bin/bash

function all()
{
	prefix=$TARGET_TMPROOTFS_DIR

	# real build
	make -j${N} all || {
	echo "make failed"
	exit 1
	}

	# put binaries to dest directory
	install -p -D -m 755 ${BUILD_OUTPUT_PATH}/uvc-host $prefix/usr/bin/uvc-host
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
#dependon libuvc

cd $(dirname $0)

buildopt $1
