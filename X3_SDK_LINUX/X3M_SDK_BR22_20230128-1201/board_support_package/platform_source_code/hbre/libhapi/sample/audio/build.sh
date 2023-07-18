#!/bin/bash

function all()
{
	prefix=$TARGET_TMPUNITTEST_DIR

	# real build
	make -j${N} all || {
	echo "make failed"
	exit 1
	}
	
	mkdir -p $prefix/bin

        cpfiles "sample_audio" "$prefix/bin"
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
