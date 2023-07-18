# Reg Offset & bit-field
USBCMD=0x0
PORTSC_HS=0x420
PORTSC_SS=0x430
GUSB3PIPECTL=0xC2c0

HCRST=[1:1]
PLS=[8:5]
PP=[9:9]
HstPrtCmpl=[30:30]


#
# Next Compliance mode
#
hbx3dbg.arm -r USB+$GUSB3PIPECTL  $HstPrtCmpl
if [ $? -eq 1 ]; then
    hbx3dbg.arm -w USB+$GUSB3PIPECTL  $HstPrtCmpl = 0
    usleep 1000
    hbx3dbg.arm -w USB+$GUSB3PIPECTL  $HstPrtCmpl = 1
else 
    echo "USB is not in the Compliance mode, run usb_enter_CP0.sh first"
fi
