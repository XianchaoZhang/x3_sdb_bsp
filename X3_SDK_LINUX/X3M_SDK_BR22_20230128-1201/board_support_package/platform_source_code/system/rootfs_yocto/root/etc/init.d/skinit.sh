#!/bin/sh

app_dev=`cat /proc/mounts |grep "/app" |awk '{print $1}'`
umount $app_dev
mount $app_dev /mnt -o sync

printf "%d" 0x1000000 >/sys/module/x2_mipi_host/parameters/adv_value

ifconfig eth0 192.168.1.10 netmask 255.255.255.0
ifconfig eth0 up
route add default gw 192.168.1.1

echo "start chronyd"
/usr/sbin/chronyd &

export BMEM_CACHEABLE=true

echo "start vio service"
/bin/vio_service &

if [ -x /mnt/init.sh ]; then
    echo "run /mnt/init.sh"
    chmod +x /mnt/init.sh
    source /mnt/init.sh
fi

# echo "start vio service"
# /bin/vio_service
