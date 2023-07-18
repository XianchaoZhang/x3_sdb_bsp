#!/bin/sh

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/lib/sensorlib/:/lib/hbmedia/:/etc/vio/
##for toggle mode between PCBA and APP
if [ -f /userdata/testmode ];then
    echo "factory PCBA test mode..."
    rm /userdata/testmode
    touch /tmp/factorytest
    sync
    exit 1
else
    echo "normal APP runing mode..."
fi
echo ttyS0 > /sys/module/kgdboc/parameters/kgdboc
##normal APP running.
boardid=$(cat /sys/class/socinfo/board_id)
if [ -z $boardid ];then
    exit 0
fi

if [ $boardid == '202' ]; then
    echo "I am mono,run mono init"
    /etc/init.d/monoinit.sh &
elif [ $boardid == '204' ] || [ $boardid == '205' ]; then
    echo "I am sk,run sk init"
    /etc/init.d/skinit.sh &
elif [ $boardid == '300' ]; then
    echo "it's j2quad board, run quadinit.sh"
    /etc/init.d/quadinit.sh &
elif [ $boardid == '400' ]; then
    echo "it's mm tianpai board, run mminit.sh"
    /etc/init.d/mminit.sh &
elif [ $boardid == '401' ]; then
    echo "it's mm s202 board, run mminit.sh"
    /etc/init.d/mminit.sh &
elif [ $boardid == '101' ]; then
    echo "I am x2dev, run x2init.sh"
    /etc/init.d/x2devinit.sh &
elif [ $boardid == '102' ]; then
    echo "I am x2ipc, run x2ipcinit.sh"
    /etc/init.d/x2ipcinit.sh &
elif [ $boardid == '203' ]; then
    echo "I am j2dev, run j2devinit.sh"
    /etc/init.d/j2devinit.sh &
else
    if [ -x /app/init.sh ]; then
        /app/init.sh &
    fi
fi
chmod 666 /sys/devices/platform/soc/a8000000.vpu/fps
chmod 666 /sys/devices/platform/soc/b3000000.isp/v3a
