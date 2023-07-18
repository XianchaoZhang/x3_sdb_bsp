usb_webcam sample
=================
This is a sample/demo code for usb webcam,
which will be made as an usb composite gadget.
- uvc: video data path, will support yuy2, nv12, mjpeg, h264...
- hid: ai structure data path
- uac: audio(microphone/speaker) data path

Data flow:
==========
Typical data flow as below:

UVC: 
sensor -> mipi -> sif -> isp -> ipu(pym) -> mjpeg/h264 encoder -> usb ---uvc protocol--- usb host

Modules
========
- usb_webcam: uvc userspace application based on v4l2 api, to do control handler and feed video stream.

other code:
- hid-gadget: a hid/interrupt path demo code.
- uac : on going. (aplayer, arecoder might be ok.)

how to build
=============
disk.img already include usb_webcam, hid_demo, usb-gadget.sh and related files.

If manually build, please do as below:
- source build/envsetup.sh
- lunch 6
- make
or
- build.sh

how to run
==========
uvc only:
- service adbd stop
- modprobe g_webcam streaming_bulk=1		# isoc has hung issue, use bulk mode instead
- usb_webcam

uvc + hid composite:
- service adbd stop
- /etc/init.d/usb-gadget start uvc-hid
- usb_webcam
- hid_demo

uvc + hid + uac2 composite:
- service adbd stop
- /etc/init.d/usb-gadget start uvc-hid-uac2
- usb_webcam
- hid_demo
- aplay/arecord

--------------------------------------------------------------------------
host PC:
win 10, 
uvc: enumerate ok, just use capture/amcap app to preview
hid: enumerate ok, needs specific hid app.
uac2: currently use some 3rd part driver, like xmos... and with pc's audio recorder...

ubuntu:
- uvc: cheese, luvcview, guvcview, capture.c, or ... self develop app... (some has isses...)
- hid_demo to test hid path.
- aplay/arecord

android:
- uvc: just install some usb webcam app in app store...
- hid: todo...
- uac2: todo...

notes
=====

to be continued
===============
