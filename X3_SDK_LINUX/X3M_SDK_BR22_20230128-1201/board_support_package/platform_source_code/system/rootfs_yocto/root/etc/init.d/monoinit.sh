#!/bin/sh


bootmode=$(cat /sys/class/socinfo/boot_mode)

if [ $bootmode == '0' ]; then
	echo "boot from emmc"
	umount /dev/mmcblk0p7
	mount /dev/mmcblk0p7 /mnt -o sync
else
	echo "boot from nor-flash"
fi
echo "info mcu bif ddr ok ..."               
echo 22 > /sys/class/gpio/export             
echo "out" > /sys/class/gpio/gpio22/direction
echo "0" > /sys/class/gpio/gpio22/value      
sleep 0.05
echo "1" > /sys/class/gpio/gpio22/value                                    
                                                                           
                                                                           
echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor
echo 105000 >  /sys/devices/virtual/thermal/thermal_zone0/trip_point_1_temp                                              
                                                                           
#echo "reset eth phy..."
#echo 69 > /sys/class/gpio/export
#echo "out" > /sys/class/gpio/gpio69/direction
#echo "0" > /sys/class/gpio/gpio69/value
#sleep 1
#echo "1" > /sys/class/gpio/gpio69/value

#ifconfig eth0 192.168.1.10 netmask 255.255.255.0
#ifconfig eth0 up
#route add default gw 192.168.1.1

echo "start cp_time_sync"
/bin/cp_time_sync &                                                          

echo "start diag"
/bin/diag/diag &

export BMEM_CACHEABLE=true

if [ -x /mnt/init.sh ]; then
    echo "run /mnt/init.sh"                    
    source /mnt/init.sh                              
fi


echo "start vio service"
/bin/vio_service
