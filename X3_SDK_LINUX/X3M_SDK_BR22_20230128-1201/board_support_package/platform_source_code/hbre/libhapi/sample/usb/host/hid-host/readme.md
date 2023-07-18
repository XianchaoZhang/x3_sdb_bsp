hid-gadget samples
==================
This is hid-gadget samples with configfs.
Directories tree as below:
.
├── gadget              # hid-gadget sample
│   ├── build.sh
│   ├── hid_demo.c
│   └── Makefile
├── host                # hid-host sample
│   ├── hid_demo.c
│   └── Makefile
└── readme.md

build
=====
gadget:
- source envsetup.sh
- lunch 6
- build.sh
- or make

host:
make

how to run
==========
xj3-board:
- service adbd stop
- uvc-gadget.sh start
- camera_test 3 1 1 1
- usb_camera -s 6 -i 1 -p 1 -c 1 -b 5 -o 0 -O 0 -u 0 -w 1920 -h 1080 -r 500 -I 64 -P 64 -M 0 -V 3 -U 1
- ./hid_demo

ubuntu-host:
- ./hid_demo


