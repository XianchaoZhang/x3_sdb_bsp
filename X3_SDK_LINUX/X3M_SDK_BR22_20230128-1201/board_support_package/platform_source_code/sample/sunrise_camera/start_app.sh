#!/bin/sh

local_path=$(cd `dirname $0`; pwd)

devmem 0xa4000038 32 0x10100000 # axibus_ctrl
devmem 0xA2D10000 32 0x04032211 # read_qos_ctrl
devmem 0xA2D10004 32 0x04032211 # write_qos_ctrl

#echo 0 >  /sys/class/misc/ion/cma_carveout_size

# 配置cpu降频的结温温度
echo 105000 > /sys/devices/virtual/thermal/thermal_zone0/trip_point_1_temp

# 设置cpu运行在高性能模式
echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor

# 设置从VPS通道中拿到的y 和 uv分量的内存地址是连续的
export IPU_YUV_CONSECTIVE=1

# 启动 usb Gadget
# 强制usb工作在为device模式
echo device > /sys/devices/platform/soc/b2000000.usb/b2000000.dwc3/role

echo soc:usb-id > /sys/bus/platform/drivers/extcon-usb-gpio/unbind
service adbd stop
/etc/init.d/usb-gadget.sh start uvc-rndis isoc

# Start web server
echo "============= Start Web Server ==============="
cd ${local_path}/WebServer
./start_lighttpd.sh

cd ${local_path}/sunrise_camera/bin
echo "============= Start Sunrise Camera ==============="
export LD_LIBRARY_PATH=../bin:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=../lib:$LD_LIBRARY_PATH
echo 1 > /sys/class/vps/mipi_host1/param/stop_check_instart
./sunrise_camera
