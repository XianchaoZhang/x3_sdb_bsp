#!/bin/bash
disk=$1
blk=$2
blk=$((blk * 1024 * 2))
size=`ls -l $disk | awk '{print $5}'`
prt=0
prt_Bytes=0
count=0
while [ $prt_Bytes -lt $size ]
do
    target="${count}_${disk}"
    dd if=/dev/zero of=$target bs=512 count=$blk
    ls -l $target
    dd if=${disk} of=$target bs=512 count=$blk skip=$prt conv=notrunc
    prt=$((prt + blk))
    prt_Bytes=$((prt_Bytes + blk * 512))
    count=$((count + 1))
    echo $prt
done
