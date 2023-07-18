#!/bin/sh

export PATH=$PWD:$PATH

# Reg Offset & bit-field
USBCMD=0x0
PORTSC_HS=0x420
PORTPMSC_HS=0x424
PORTSC_SS=0x430
PORTPMSC_SS=0x434
GUSB3PIPECTL=0xC2c0

HCRST=[1:1]
PLS=[8:5]
PP=[9:9]
HstPrtCmpl=[30:30]
TestMode=[31:28]

# switch to host mode in default

echo host > /sys/devices/platform/soc/b2000000.usb/b2000000.dwc3/role
#
# Start to Reset & Enter Compliance mode
#
hbx3dbg.arm -w USB+$USBCMD  $HCRST  1
usleep 1000
hbx3dbg.arm -r USB+$USBCMD  $HCRST
if [ $? -eq 0 ]; then
    hbx3dbg.arm -w USB+$PORTPMSC_HS  $TestMode = 0
    var=$(hbx3dbg.arm -r USB+$PORTPMSC_HS  $TestMode)
    echo "reset port test mode: ${var:0-3}"

    hbx3dbg.arm -w USB+$PORTPMSC_HS  $TestMode = 4

    var=$(hbx3dbg.arm -r USB+$PORTPMSC_HS  $TestMode)
    echo "set port test mode to: ${var:0-3}"
else
    echo "Host Controller Reset Faill!!"
fi
