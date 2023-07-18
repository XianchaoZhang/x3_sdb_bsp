#!/bin/sh

umount /dev/mmcblk0p7         
mount /dev/mmcblk0p7 /mnt -o sync

#J2A_1V8_GPIO4
echo "info mcu bif ddr ok ..."
echo 21 > /sys/class/gpio/export
echo "out" > /sys/class/gpio/gpio21/direction
echo "0" > /sys/class/gpio/gpio21/value
sleep 1
echo "1" > /sys/class/gpio/gpio21/value

#for network monitor
echo 22  > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio22/direction
/etc/init.d/quad_mon_eth.sh &

#ti954 address is 0x3d on quad without gw5200
TI954_ADDR=0x3d
DEV_ID_REG=0
i2cget -f -y 0 $TI954_ADDR $DEV_ID_REG >/dev/null 2>&1

if [ $? -eq 0 ];then
        echo "quad"
else
        echo "quad gw5200"
        mcu_util --cas 3 0
        echo 65 > /sys/class/gpio/export
        echo out > /sys/class/gpio/gpio65/direction
        echo 71 > /sys/class/gpio/export
        echo out > /sys/class/gpio/gpio71/direction
        echo 91 > /sys/class/gpio/export
        echo out > /sys/class/gpio/gpio91/direction

        echo 0 > /sys/class/gpio/gpio65/value
        echo 0 > /sys/class/gpio/gpio71/value
        echo 0 > /sys/class/gpio/gpio91/value
        sleep 0.01

        echo 1 > /sys/class/gpio/gpio65/value
        echo 1 > /sys/class/gpio/gpio71/value

        sleep 0.01
        echo 1 > /sys/class/gpio/gpio91/value
        sleep 1
        echo 65 > /sys/class/gpio/unexport
        echo 71 > /sys/class/gpio/unexport
        echo 91 > /sys/class/gpio/unexport

        echo "start camera tirger via MCU"
        mcu_util --cas 0 33.333 50 1
fi

echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor
echo 105000 >  /sys/devices/virtual/thermal/thermal_zone0/trip_point_1_temp                                              
echo 0 > /sys/module/x2_mipi_dphy/parameters/txout_freq_autolarge_enbale

#already done in S45defaultip
#defaultip.sh

echo "start time sync service"
/bin/cp_time_sync &                                                          

#veriy & enable giag since V2.0.0
echo "start diag"
/bin/diag/diag &

export BMEM_CACHEABLE=true

if [ -x /mnt/init.sh ]; then
    echo "run /mnt/init.sh"                    
    source /mnt/init.sh                              
fi


#echo "start vio service"
/bin/vio_service
