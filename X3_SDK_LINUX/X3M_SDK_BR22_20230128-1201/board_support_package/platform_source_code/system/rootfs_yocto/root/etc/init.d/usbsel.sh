#!/bin/sh

### BEGIN INIT INFO
# Provides:
# Required-Start:
# Required-Stop:
# Default-Start:     S, 5
# Default-Stop:
# Short-Description: usb try & port select for x3 dvb
# Description:
### END INIT INFO

. /etc/default/rcS

check_usb_port () {
    # switch to host mode
    echo host > /sys/devices/platform/soc/b2000000.usb/b2000000.dwc3/role

    sleep 1

    idVendor=$(cat /sys/devices/platform/soc/b2000000.usb/b2000000.dwc3/xhci-hcd.0.auto/usb1/1-1/idVendor 2>/dev/null)
    idProduct=$(cat /sys/devices/platform/soc/b2000000.usb/b2000000.dwc3/xhci-hcd.0.auto/usb1/1-1/idProduct 2>/dev/null)

    if [ -z $idVendor ] || [ -z $idProduct ]; then
        echo device > /sys/devices/platform/soc/b2000000.usb/b2000000.dwc3/role
        sleep 0.3
        setprop adbd.start 1
        exit 0
    fi

    if [ $idVendor == '0451' ] && [ $idProduct == '8027' ]; then
        echo "usb switch to standard-a port? usb work as host in default"
    else
        echo device > /sys/devices/platform/soc/b2000000.usb/b2000000.dwc3/role
        sleep 0.3
    fi
}

#
# There is a USB_SW_SEL pin in DIP switch. But it doesn't connect to
# usb_id pin or other gpio pins, which cause cpu couldn't know the
# usb port situations.
# Below is for usb port check script. Just force to host mode and check if
# ti usb hub (0451, 8025/8027) is connected.
#

# check board_id
board_name=$(cat /sys/class/socinfo/board_name)
if [ -z $board_name ]; then
    setprop adbd.start 1
    exit 0;
fi

# currently only x3 dvb has usb switch chip
if [ ${board_name:0:5} != "x3dvb" ]; then
    setprop adbd.start 1
    exit 0;
fi

# check usb mode
usb_mode=$(cat /sys/devices/platform/soc/b2000000.usb/b2000000.dwc3/role)
if [ -z $usb_mode ]; then
    setprop adbd.start 1
    exit 0;
fi

# check & set usb port
if [ $usb_mode == 'device' ]; then
    check_usb_port
elif [ $usb_mode == 'host' ]; then
    echo "likely otg cable inserted?"
else
    echo "invalid usb mode: $usb_mode"
fi
setprop adbd.start 1
