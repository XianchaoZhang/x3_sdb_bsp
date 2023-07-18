#!/bin/sh

##APP deinit
boardid=$(cat /sys/class/socinfo/board_id)

if [ $boardid == '204' ] || [ $boardid == '205' ]; then
    echo "I am sk,run sk deinit"
    if [ -x /mnt/j2ready ]; then
        /mnt/j2ready 0
    fi
fi
