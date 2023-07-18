uvc host app
=================
This is a sample/demo code for uvc host app,
Which will get usb camera's data and feed to vps, bpu modules.

Data flow:
==========
Typical data flow as below:

UVC: 
usb camera -> uvc protocol -> v4l2 -> vps -> vdec(optional) -> bpu

Modules
========
- uvc_host : uvc host application to feed uvc video data to vps, vdec, bpu device for ai process.

how to build
=============
disk.img already include uvc_host and related files.

If manually build, please do as below:
- source build/envsetup.sh
- lunch 6
- make
or
- build.sh

how to run
==========

notes
=====

to be continued
===============
