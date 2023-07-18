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
# Start to Reset & Enter Compliance mode
#
hbx3dbg.arm -w USB+$USBCMD  $HCRST  1
usleep 1000
hbx3dbg.arm -r USB+$USBCMD  $HCRST
if [ $? -eq 0 ]; then
    hbx3dbg.arm -w USB+$PORTSC_SS  $PLS = 0xA
    hbx3dbg.arm -w USB+$PORTSC_SS  $PP  = 0x0
    hbx3dbg.arm -w USB+$GUSB3PIPECTL  $HstPrtCmpl = 1
else 
    echo "Host Controller Reset Faill!!"
fi
