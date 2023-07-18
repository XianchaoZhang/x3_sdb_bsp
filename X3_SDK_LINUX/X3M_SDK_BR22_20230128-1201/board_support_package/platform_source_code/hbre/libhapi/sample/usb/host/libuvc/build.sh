#!/bin/bash

function all()
{
	prefix=$TARGET_TMPROOTFS_DIR

	# real build
	make -j${N} all || {
	echo "make failed"
	exit 1
	}

	# put include, lib& binaries to dest directory
	install -D -m 0644 inc/camera.h $prefix/include/uvc/camera.h
	install -p -D -m 755 ${BUILD_OUTPUT_PATH}/uvc-demo $prefix/usr/bin/uvc-demo
	install -p -D -m 755 ${BUILD_OUTPUT_PATH}/uvc-demo-static $prefix/usr/bin/uvc-demo-static
	install -p -D -m 755 ${BUILD_OUTPUT_PATH}/libuvc.so $prefix/lib/libuvc.so
	install -p -D -m 755 ${BUILD_OUTPUT_PATH}/libuvc.a $prefix/lib/libuvc.a
}

function all_32()
{
	all
}

function clean()
{
	make clean
	make dist-clean
}

# include
. $INCLUDE_FUNCS
# include end

cd $(dirname $0)

buildopt $1
