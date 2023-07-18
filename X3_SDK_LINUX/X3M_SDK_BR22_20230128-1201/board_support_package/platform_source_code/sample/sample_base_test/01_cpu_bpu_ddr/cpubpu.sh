#!/bin/sh

local_path=$(cd `dirname $0`; pwd)

mount -o remount,rw /

echo 148360000 > /sys/module/hobot_dev_ips/parameters/sif_mclk_freq
echo 204000000 > /sys/module/hobot_vpu/parameters/vpu_clk_freq

echo 105000 > /sys/devices/virtual/thermal/thermal_zone0/trip_point_1_temp
echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor

# 固定bpu频率
echo 1000000000 > /sys/class/devfreq/devfreq1/userspace/set_freq
cat /sys/class/devfreq/devfreq1/userspace/set_freq
echo 1000000000 > /sys/class/devfreq/devfreq2/userspace/set_freq
cat /sys/class/devfreq/devfreq2/userspace/set_freq

cd ${local_path}
chmod +x *.sh
cd ${local_path}/bpu_cpu
./bpu-group-test1.sh -b 2 -p 99 &
cd ${local_path}
../bin/stressapptest -M 50 -s 172800 &
