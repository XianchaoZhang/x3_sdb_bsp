#!/bin/bash

function getmodules()
{
    if [ ! -f "$1" ];then
        echo "Error: $1 doesnot a FILE"
        exit 1
    fi
    local mfile="$1"
    local MANIFEST="${BUILD_OUT_DIR}/.${mfile##*/}" 
    mkdir -p ${BUILD_OUT_DIR}

    sort -u $mfile > $MANIFEST

    ## format
    sed -i 's/\\/\//g' $MANIFEST > /dev/null
    sed -i '/^[[:space:]]*$/d' $MANIFEST > /dev/null
    sed -i '/^[[:space:]]*[#]/d' $MANIFEST > /dev/null
    #sed -i 's/^\//+\/&/g' $MANIFEST > /dev/null
    #sed -i 's/\*$//g' $MANIFEST > /dev/null

    while read line;
    do
        local str=$line
        local firstc=${str:0:1}
        local rpath=${str:1}

        if [ "$firstc" = "-" ];then
            excludes+=" -path $(cd ${HR_TOP_DIR}/$rpath;pwd) -a -prune -o "
        else
            srcs+=" $(cd ${HR_TOP_DIR}/$rpath && pwd) "
            #srcs+=" $(test -d ${HR_TOP_DIR}/$rpath && cd ${HR_TOP_DIR}/$rpath && pwd) "
        fi
    done < $MANIFEST
    find $srcs $excludes -name "build.sh" -print >> $modules || {
        echo "Error: Cmd find result:$?, failed"
        exit 1
    }
    rm -rf $MANIFEST
}

function getdependname() {
    local modules=$1
    for f in $(cat $modules|sort)
    do
        local dirname=${f%/*}
        dirname=${dirname##*/}
        for line in $(cat $f | grep "^dependon ")
        do
            if [ "$line" != "dependon" ];then
                echo "$dirname:$line" >> $xdependx
            fi
        done
        echo "$dirname" >> $xdepend
    done
    unset line
    unset f
}

function check_depends() {
    if [ ! -f $xdependx ];then
        return
    fi
    for line in $(cat $xdependx)
    do
        src=${line%:*}
        dep=${line#*:}
        result=$(grep "^$dep$" $xdepend)
        if [ $? -ne 0 ];then
            echo "Error: Cannot find $src dependency $dep"
            exit 1
        fi
    done
    unset line
}

function isleaf() {
    local line
    local leaf=$1
    if [ ! -f $xdependx ];then
        return 0
    fi
    local srcs=$(cat $xdependx)
    if [ -z "$srcs" ];then
        return 0
    fi
    for line in $srcs
    do 
        src=${line%:*}
        if [ "$src" = "$leaf" ];then
            return 1
        fi
    done
    for line in $srcs
    do 
        dep=${line#*:}
        if [ "$dep" = "$leaf" ];then
            sed -i "/^$line$/d" $xdependx
        fi
    done
    return 0
}

function generate_build() {
    local line
    local unknown=true

    if [ ! -f $xdepend ];then
        echo "Error: Cannot find $xdepend"
        exit 1
    fi
    echo "write to ${temp_hbre_script}"
    while ((1==1))
    do
        local srcs=$(cat $xdepend)
        if [ -z "$srcs" ];then
            break
        fi
        for line in $srcs
        do
            isleaf $line
            if [ $? -eq 0 ];then
                local dir=$(grep -r "/$line/build.sh$" $modules)
                dir=${dir%/*}
                if [ "$modulename" = "all" ];then
                    echo -e "echo \"******************************\"" >> $temp_hbre_script
                    echo -e "echo \"begin to build $line\"" >> $temp_hbre_script
                    echo -e "cd $dir" >> $temp_hbre_script
                    echo -e "./build.sh $action || exit 1" >> $temp_hbre_script
                    echo -e "echo \"end build $line\"" >> $temp_hbre_script
                    echo -e "echo \"******************************\"" >> $temp_hbre_script
                    unknown=false
                else
                    local dirname=${dir##*/}
                    if [ "$dirname" = "$1" ];then
                        echo "if [ \"$dirname\" = \"\$1\" ];then" >> $temp_hbre_script
                        echo -e "\techo \"******************************\"" >> $temp_hbre_script
                        echo -e "\techo \"begin to build $line\"" >> $temp_hbre_script
                        echo -e "\tcd $dir" >> $temp_hbre_script
                        echo -e "\t./build.sh $action || exit 1" >> $temp_hbre_script
                        echo -e "\techo \"end build $line\"" >> $temp_hbre_script
                        echo -e "\techo \"******************************\"" >> $temp_hbre_script
                        echo "fi" >> $temp_hbre_script
                        unknown=false
                    fi
                fi
                sed -i "/^$line$/d" $xdepend
            fi
        done
    done
    if $unknown ;then
        echo "Unknown module: $1"
        exit 1
    fi
}

if [ $# -ne 3 ];then
    echo "Usage: $0 modules.manifest modulename [all|all_32|clean]"
    exit 1
fi
if [ "$3" != "all" -a "$3" != "all_32" -a "$3" != "clean" ];then
    echo "Error: 3th param $3 is neither 'all', "all_32", 'clean'"
    exit 1
fi

manifest="$1"
modulename="$2"
action="$3"

temp_hbre_script=$TEMP_HBRE_SCRIPT
xdepend=${BUILD_OUT_DIR}/.xdepend
xdependx=${BUILD_OUT_DIR}/.xdependx
modules=${BUILD_OUT_DIR}/.modules
rm -rf $temp_hbre_script $xdepend $xdependx $modules

echo "#!/bin/sh" >> $temp_hbre_script
echo "" >> $temp_hbre_script

echo "getmodules $manifest"
getmodules $manifest
getdependname $modules

check_depends
generate_build $modulename $action
chmod +x $temp_hbre_script
