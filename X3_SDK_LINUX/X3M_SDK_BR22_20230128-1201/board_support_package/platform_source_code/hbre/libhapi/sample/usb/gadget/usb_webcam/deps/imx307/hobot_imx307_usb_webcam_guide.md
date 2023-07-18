### hobot usb_webcam manaul

#### command as below:

fiture_imx307_enable.sh
service adbd stop
/etc/init.d/usb-gadget.sh start uvc-rndis isoc
ifconfig usb0 192.168.100.100

usb_webcam -e 12 -m 2 -t 10
