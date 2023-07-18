#!/bin/sh

function insert_ko()
{
    if [ $# -eq 0 ];then
        echo "$0: error usage, no input found!" > /dev/ttyS0
    fi

    if [ -f $1 ];then
        res="$(insmod $@ 2>&1)"
        # if insmod has failed, print result to console
        [ $? -gt 0 ] && { echo "$res" > /dev/ttyS0; }
    else
        echo "$1 not found, continue!"
    fi
}

bpu_version=`ls -l $(find /lib -name "libcnn_intf.so") | sed 's/.*so.\([0-9]*\).*/\1/g'`
knl_version=`uname -r`
ko_root="/lib/modules/${knl_version%%-*}"
board_type=$((0x`cat /sys/class/socinfo/board_id` & 0xf))

if [ "$bpu_version" = 1 ]; then
    insert_ko $ko_root/bpu_framework.ko
    insert_ko $ko_root/bpu_cores.ko
else
    insert_ko $ko_root/hobot_cnn_host_total.ko
fi

insert_ko $ko_root/hobot_efuse.ko
insert_ko $ko_root/iar.ko logo=0
insert_ko $ko_root/hobot_vpu.ko
insert_ko $ko_root/hobot_jpu.ko
insert_ko $ko_root/hobot_sif.ko
# videobuf-core.ko
insert_ko $ko_root/videobuf-core.ko
# v4l2 related kernel modules
insert_ko $ko_root/videobuf2-core.ko
insert_ko $ko_root/videobuf2-v4l2.ko
insert_ko $ko_root/videobuf2-memops.ko
insert_ko $ko_root/videobuf2-dma-contig.ko
insert_ko $ko_root/videobuf2-vmalloc.ko
insert_ko $ko_root/hobot_isp_log.ko
insert_ko $ko_root/hobot_iq.ko
insert_ko $ko_root/hobot_sns_bridge.ko
insert_ko $ko_root/hobot_sensor.ko
insert_ko $ko_root/hobot_lens.ko
insert_ko $ko_root/hobot_dwe.ko
insert_ko $ko_root/hobot_isp.ko
insert_ko $ko_root/hobot-fb.ko
insert_ko $ko_root/iar_cdev.ko

if [ $board_type -eq 4 ];then
    insert_ko $ko_root/rtc-hobot.ko
else
    insert_ko $ko_root/hobot_hdmi.ko
fi

insert_ko $ko_root/iar_mmap.ko
insert_ko $ko_root/hobot_ipu.ko
insert_ko $ko_root/hobot_pym.ko
insert_ko $ko_root/hobot_gdc.ko
insert_ko $ko_root/hobot_spidev.ko
insert_ko $ko_root/hobot_osd.ko

# Network drivers maybe not build in kernel, provided as driver module
insert_ko $ko_root/8021q.ko
insert_ko $ko_root/mii.ko
insert_ko $ko_root/hobot_xj3_ethX.ko

#udevadm trigger buildin drivers
udevadm trigger --action=add --subsystem-nomatch=i2c --subsystem-nomatch=platform
setprop loadko.ready 1
