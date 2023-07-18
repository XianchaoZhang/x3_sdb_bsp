#!/bin/sh

ifconfig eth0 192.168.1.10 netmask 255.255.255.0                                                                                                                                             
ifconfig eth0 up
route add default gw 192.168.1.1
ifconfig eth0 down

if [ -x /mnt/init.sh ]; then
    echo "run /mnt/init.sh"                    
    source /mnt/init.sh                              
fi
