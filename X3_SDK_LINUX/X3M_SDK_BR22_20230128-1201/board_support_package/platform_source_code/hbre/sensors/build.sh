#!/bin/bash

set -ex

function all()
{
    prefix=$TARGET_TMPROOTFS_DIR

    sensen_list="f37 gc4663 gc4c33 imx307 imx327 imx415 imx586 s5kgm1 imx219 imx477 ov5647"

    for sensor in ${sensen_list};
    do
        [ ! -d "$prefix/lib/sensorlib/" ] && mkdir -p "$prefix/lib/sensorlib/"
        if [ -f ${sensor}/lib*${sensor}_linear.so ]; then
            cpfiles "${sensor}/lib*${sensor}_linear.so" "$prefix/lib/sensorlib/"
        fi
        if [ -f ${sensor}/lib${sensor}_dol2.so ]; then
            cpfiles "${sensor}/lib*${sensor}_dol2.so" "$prefix/lib/sensorlib/"
        fi
    done

    cpfiles "common/initweb.sh" "$prefix/usr/bin/"
}

function clean()
{
    echo "clean sensor librarys"
}

# include
. $INCLUDE_FUNCS
# include end

cd $(dirname $0)

buildopt $1
