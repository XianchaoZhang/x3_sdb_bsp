#!/bin/sh

export LD_LIBRARY_PATH=.:$LD_LIBRARY_PATH

# reset sensor
echo 118 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio118/direction
echo 0 > /sys/class/gpio/gpio118/value
sleep 0.2
echo 1 > /sys/class/gpio/gpio118/value
echo 118 > /sys/class/gpio/unexport

# enable sensor mclk
echo 1 > /sys/class/vps/mipi_host0/param/snrclk_en
echo 24000000 > /sys/class/vps/mipi_host0/param/snrclk_freq
echo 1 > /sys/class/vps/mipi_host1/param/snrclk_en
echo 24000000 > /sys/class/vps/mipi_host1/param/snrclk_freq

# echo 1 > /sys/class/vps/mipi_host0/param/stop_check_instart
# echo 1 > /sys/class/vps/mipi_host1/param/stop_check_instart

# i2cdetect -r -y 0
# i2cdetect -r -y 1
# i2cdetect -r -y 2

# Start ISP Control Tool
# Access via browser: http://192.168.1.10:8214/
act-server --http-port=8214 --http-root=/etc/control-tool/content --fw-acc=stream --fw-channel=chardev --fw-dev=/dev/ac_isp --hw-acc=stream --hw-buf-size=1024 --hw-channel=devmem --hw-dev=/dev/mem --hw-mem-offset=0xb3000000 --hw-mem-size=0x400000 &

sync
sleep 0.5

sensor_index=$1
if [ "${sensor_index}" == "" ]; then
	echo "Please select sensor:"
	echo "  0: imx415 linear mode"
	echo "  1: imx415 dol2 mode"
	echo -n "please input: "
	read sensor_index
fi
echo "get sensor_index is ${sensor_index}"

sleep 0.5
run_mode=$2
if [ "${run_mode}" == "" ]; then
	echo "please select online/offline:"
	echo "  o: online"
	echo "  f: offline"
	echo -n "please input: "
	read run_mode
fi
echo "get run_mode is ${run_mode}"

set -ex
if [ "${sensor_index}" == "0" ]; then
	if [ "${run_mode}" == "f" ]; then
		tuning_tool -C "./hb_x3player.json" -v "./sdb_imx415_raw_10bit_3840x2160_offline.json" -H "/etc/tuning_tool/dump_raw.json"  -r 100000 -c 0  -p 1 -t 1 -l 4 -m 0 -f 0 -g 0 -D 0
	elif [ "${run_mode}" == "o" ];then
		tuning_tool -C "./hb_x3player.json" -v "./sdb_imx415_raw_10bit_3840x2160_online.json" -H "/etc/tuning_tool/dump_yuv.json"  -r 100000 -c 0  -p 1 -t 1 -l 4 -m 0 -f 0 -g 0 -D 0
	fi

fi

if [ "${sensor_index}" == "1" ]; then
	if [ "${run_mode}" == "f" ]; then
		tuning_tool -C "./hb_x3player.json" -v "./sdb_imx415_raw_10bit_3840x2160_dol2_offline.json" -H "/etc/tuning_tool/dump_raw.json"  -r 100000 -c 1  -p 1 -t 1 -l 4 -m 0 -f 0 -g 0 -D 0
	elif [ "${run_mode}" == "o" ];then
		tuning_tool -C "./hb_x3player.json" -v "./sdb_imx415_raw_10bit_3840x2160_dol2_online.json" -H "/etc/tuning_tool/dump_yuv.json"  -r 100000 -c 1  -p 1 -t 1 -l 4 -m 0 -f 0 -g 0 -D 0
	fi
	
fi