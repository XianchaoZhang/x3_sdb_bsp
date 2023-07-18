#!/bin/sh

run_usb3_otg()
{
	# stop adb deamon
	service adbd stop

	# change usb controller to host mode
	echo host > /sys/devices/platform/soc/b2000000.usb/b2000000.dwc3/role

	# unbind extcon-usb-gpio driver to release gpio pin
	echo "soc:usb-id" > /sys/bus/platform/drivers/extcon-usb-gpio/unbind

	# export gpio 65 to sysfs
	echo 65 > /sys/class/gpio/export

	# set gpio 65 as output direction
	echo "out" > /sys/class/gpio/gpio65/direction

	# pulldown gpio 65 in order to supply power for usb vbus
	echo 0 > /sys/class/gpio/gpio65/value

	# then usb3.0 device can be enumerated successfully
	exit 0
}

stop_usb3_otg()
{
	# change usb controller to device mode
	echo device > /sys/devices/platform/soc/b2000000.usb/b2000000.dwc3/role

	# unexport gpio 65
	echo 65 > /sys/class/gpio/unexport

	# bind extcon-usb-gpio driver
	echo "soc:usb-id" > /sys/bus/platform/drivers/extcon-usb-gpio/bind

	# start adb deamon
	service adbd start
}

helper()
{
    echo ---------------------------------------------------------------------
    echo "Usage:  "
    echo "  usb3_otg.sh [option]"
    echo "    option:"
    echo "    -r: run usb3 otg for special otg cable(no usb_pin)"
    echo "    -s: stop usb3 otg and restore to default mode"
    echo "    -h: helper prompt"
    echo
}


####################################
# main logic from here
####################################

has_opts=0
#"uh" > "u:h:" if need args
while getopts "rsh" opt; do
  case $opt in
	r)
		run_usb3_otg
		exit
		;;
	s)
		stop_usb3_otg
		exit
		;;
	h)
		helper
		exit
		;;
	?)
		helper
		;;
	*)
		helper
		;;
	esac

  has_opts=1
done

if [ $has_opts -eq "0" ]; then
	helper
fi
