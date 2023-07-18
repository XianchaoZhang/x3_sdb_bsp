#!/bin/sh

IFS_old=$IFS
IFS=$'\n'
ff=$(blkid)
for blk in $ff
do
    IFS=":\""
    disk=$(echo $blk | awk '{print $1}')
    type=$(echo $blk | awk '{print $7}')
    if [ "$type" = "ext4" ];then
        resize2fs -f $disk > /dev/null 2>&1
    fi
done
IFS=$IFS_old
setprop resize.ready 1
