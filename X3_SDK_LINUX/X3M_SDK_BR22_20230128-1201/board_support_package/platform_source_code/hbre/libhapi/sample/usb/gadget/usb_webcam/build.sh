#!/bin/bash

include_deps=false

function all()
{
	prefix=$TARGET_TMPROOTFS_DIR

	# real build
	make -j${N} all || {
	echo "make failed"
	exit 1
	}

	# put binaries to dest directory
	install -p -D -m 755 ${BUILD_OUTPUT_PATH}/usb_webcam $prefix/usr/bin/usb_webcam
	install -D -m 0644 config/usb_webcam.conf $prefix/etc/usb_webcam.conf

	if $include_deps; then
		# dependence
		install -p -D -m 755 deps/imx307/libimx307.so $prefix/lib/sensorlib/libimx307.so
		install -p -D -m 755 deps/imx307/libimx307_linear.so $prefix/etc/cam/libimx307_linear.so
		install -p -D -m 755 deps/imx307/fiture_imx307_enable.sh $prefix/usr/bin/fiture_imx307_enable.sh
	fi
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
#dependon libguvc

cd $(dirname $0)

buildopt $1
