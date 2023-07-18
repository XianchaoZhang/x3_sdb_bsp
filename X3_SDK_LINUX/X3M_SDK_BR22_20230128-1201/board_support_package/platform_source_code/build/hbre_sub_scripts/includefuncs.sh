#!/bin/bash

function dependon()
{
    if [ "$ENABLE_BUILD_DEPENDENCY" = "false" ];then
        return
    fi
    for m in $@
    do
        local path=$(grep -r "/$m/build.sh$" $ALL_MODULES_LIST)

        echo "+++++++++++++++++"
        echo "begin to build $m"
        $path || exit 1
        echo "end build $m"
        echo "+++++++++++++++++"
    done
    unset m
}

function cpfiles()
{
    if [ $# -ne 2 ];then
        echo "Usage: cpfiles \"sourcefiles\" \"destdir\""
        exit 1
    fi

    mkdir -p $2 || {
        echo "mkdir -p $2 failed"
        exit 1
    }

    for f in $1
    do
        if [ -a $f ];then
            cp -af $f $2 || {
                echo "cp -af $f $2 failed"
                exit 1
            }
        fi
    done
    echo "cpfiles $1 $2"
}

function prepare_sources()
{
	current_path=`pwd`
	export BUILD_OUTPUT_PATH=${BUILD_OUT_DIR}${current_path#*${HBRE_DIR}}
	echo "${BUILD_OUTPUT_PATH} out of source build"
	mkdir -p ${BUILD_OUTPUT_PATH}
}

function buildopt()
{
	# Export this variable so that all projects can link to the HOST library
	export LD_LIBRARY_PATH=${TOOLCHAIN_LD_LIBRARY_PATH}

	prepare_sources

	if [ -z "$1" ];then
		echo "build all"
		all
	else
        case $1 in
        all)
            echo "build all"
            all
            ;;
        clean)
            echo "build clean"
            clean
            ;;
        *)
            echo "Usage: $0 [all|clean]"
            exit 1
            ;;
        esac
    fi
}

