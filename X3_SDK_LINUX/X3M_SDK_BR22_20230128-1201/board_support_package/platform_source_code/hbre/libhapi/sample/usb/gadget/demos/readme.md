usb camera demos
================
This is a sample/demo code for usb camera, which is porting
from aiot's project, based on c++ and cmake.

It just link to our rootfs dir(TARGET_TMPROOTFS_DIR), then
no need to copy our sdk libraries and header files to other
project.

This is a sample application's format, which also need to be
optimized step by step.
(eg. like yocto's recipes is good solution for project development.)

samples
=====
- usb_camera_eptz	# a sample for usb camera with eptz feature.
			# eptz means - electronic Pan, Tilt and Zoom.

how to build
============
### prepare >>
1. source build/envsetup.sh	# source xj3 repo's build environment.
				# mainly for TARGET_TMPROOTFS_DIR env.
2. make sure toolchain is ready. (please refer to CMakeLists.txt)

### building >>
- mkdir build
- cd build
- cmake ..
- make
- make install

how to run
==========
Just copy output folder to target board, and run script as below:
./run_usb_cam.sh

notes (for bulk and isoc)
============
Default using usb isoc case.
If user want to test bulk case

please modify run_usb_cam.sh, 
/etc/init.d/usb-gadget.sh start uvc isoc
./usb_camera_eptz
>>>
/etc/init.d/usb-gadget.sh start uvc
./usb_camera_eptz -b


to be continued
===============
