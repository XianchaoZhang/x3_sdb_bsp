#!/bin/bash

set -x

function all()
{
	prefix=$TARGET_TMPUNITTEST_DIR

	echo "build sunrise camera"
	# real build
	make || {
		echo "make failed"
		exit 1
	}

	# put binaries to dest directory
	cpfiles "sunrise_camera/*" "${prefix}/sunrise_camera"
	cpfiles "WebServer" "${prefix}"
	cpfiles "start_app.sh" "${prefix}"

	# modify init.rc for auto start sunrise_camera
	cpfiles "Platform/x3/x3_auto_start/init.rc" "${TARGET_TMPROOTFS_DIR}"

	if [ x"$release_pkg" = x"true" ];then
		tar -czvf sunrise_camera_v2.0.0.tar.gz sunrise_camera WebServer start_app.sh
	fi
}

function clean()
{
	make clean
}

function usage()
{
	echo "Usage: build.sh [-p]"
	echo "Options:"
	echo "  -p  Generate release package"
	echo "  -h  help info"
	echo "Command:"
	echo "  clean clean all the object files along with the executable"
}

release_pkg="false"
while getopts "ph:" opt
do
	case $opt in
		p)
			release_pkg="true"
			;;
		h)
			usage
			exit 0
			;;
		\?)
			usage
			exit 1
			;;
	esac
done

shift $[ $OPTIND - 1 ]

. $INCLUDE_FUNCS
cd $(dirname $0)

cmd=$1
buildopt $cmd
