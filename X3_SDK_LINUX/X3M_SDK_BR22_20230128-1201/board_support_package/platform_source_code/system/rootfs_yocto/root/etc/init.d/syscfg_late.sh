#!/bin/sh
# Do some system config after all insmod finished

# CPU freq governer is performance when boot,
# switch to ondemand in runtime for x3.
board_name=`cat /sys/class/socinfo/board_name`

if [ ${board_name:5:2} == "x3" ]; then
        cur_gov=`cat /sys/bus/cpu/devices/cpu0/cpufreq/scaling_governor`
        echo ondemand > /sys/bus/cpu/devices/cpu0/cpufreq/scaling_governor
        echo "switch cpu scaling_governor: $cur_gov -> ondemand" > /dev/kmsg
fi
