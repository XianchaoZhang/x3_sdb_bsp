#!/bin/sh

som_name=$(cat /sys/class/socinfo/som_name)

# reset bt
if [ ${som_name} == '5' ];then
	# X3 PI
	echo 57 > /sys/class/gpio/export
	echo out > /sys/class/gpio/gpio57/direction
	echo 0 > /sys/class/gpio/gpio57/value
	sleep 0.5
	echo 1 > /sys/class/gpio/gpio57/value
	echo 57 > /sys/class/gpio/unexport
elif [ ${som_name} == '3' ]  || [ ${som_name} == '4' ]; then
	# X3 SDB
	echo 23 > /sys/class/gpio/export
	echo out > /sys/class/gpio/gpio23/direction
	echo 0 > /sys/class/gpio/gpio23/value
	sleep 0.5
	echo 1 > /sys/class/gpio/gpio23/value
	echo 23 > /sys/class/gpio/unexport
fi

id messagebus >& /dev/null
if [ $? -ne 0 ]; then
	mount -o rw,remount /
	root_dev=`cat /proc/cmdline | sed -e 's/^.*root=//' -e 's/ .*$//'`
	resize2fs ${root_dev}
	groupadd  messagebus
	useradd -g messagebus messagebus
	mount -o ro,remount /
fi

brcm_patchram_plus --enable_hci --no2bytes --tosleep 200000 --baudrate 460800 --patchram /lib/firmware/brcm/BCM4343A1.hcd /dev/ttyS1 &

wait_hci0=0
while true
do
	[ -d /sys/class/bluetooth/hci0 ] && break
	sleep 1
	let wait_hci0++
	[ $wait_hci0 -eq 30 ] && {
		echo "bring up hci0 failed"
		exit 1
	}
done

sleep 1
hciconfig hci0 up
hciconfig

hciconfig hci0 piscan

sleep 0.2
mkdir -p /var/run/dbus
dbus-daemon --system --syslog &
bluetoothd &
