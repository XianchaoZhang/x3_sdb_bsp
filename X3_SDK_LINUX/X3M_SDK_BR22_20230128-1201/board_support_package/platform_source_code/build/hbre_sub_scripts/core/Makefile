# Note: This is a core Makefile, which is included in the makefiles of all projects. It is a general configuration. If there is a unified modification, please modify it here.

CC = ${CROSS_COMPILE}gcc
CXX = ${CROSS_COMPILE}g++
AR = ${CROSS_COMPILE}ar
AS = ${CROSS_COMPILE}as
LD = ${CROSS_COMPILE}ld
STRIP = ${CROSS_COMPILE}strip
RANLIB = ${CROSS_COMPILE}ranlib
OBJCOPY = ${CROSS_COMPILE}objcopy
OBJDUMP = ${CROSS_COMPILE}objdump
READELF = ${CROSS_COMPILE}readelf
NM = ${CROSS_COMPILE}nm

CFLAGS = -Wall
CXXFLAGS = -Wall
CFLAGS_DYNAMIC = -fPIC -shared
CFLAGS_STATIC = -rcs

ifeq (${TARGET_MODE},debug)
        CFLAGS += -g -O0
        CXXFLAGS += -g -O0
endif
ifeq (${TARGET_MODE},release)
        CFLAGS += -O2
        CXXFLAGS += -O2
endif

LDFLAGS = -L${TARGET_TMPROOTFS_DIR}/lib
LDFLAGS += -L${TARGET_TMPROOTFS_DIR}/lib/hbbpu
LDFLAGS += -L${TARGET_TMPROOTFS_DIR}/lib/hbmedia
LDFLAGS += -L${TARGET_TMPROOTFS_DIR}/lib/sensorlib
LDFLAGS += -L${TARGET_TMPROOTFS_DIR}/usr/lib
LDFLAGS += -L${TARGET_TMPROOTFS_DIR}/libdisp/
LDFLAGS += -L${TARGET_TMPROOTFS_DIR}/lib
LDFLAGS += -L${TARGET_TMPROOTFS_DIR}/usr/lib
LDFLAGS += -L${SRC_HOST_DIR}/third_party/usr/lib

LDFLAGS += -Wl,-rpath-link=${TARGET_TMPROOTFS_DIR}/lib
LDFLAGS += -Wl,-rpath-link=${TARGET_TMPROOTFS_DIR}/usr/lib
LDFLAGS += -Wl,-rpath-link=${TARGET_TMPROOTFS_DIR}/lib
LDFLAGS += -Wl,-rpath-link=${TARGET_TMPROOTFS_DIR}/usr/lib
LDFLAGS += -Wl,-rpath-link=${TARGET_TMPROOTFS_DIR}/lib/bpu
LDFLAGS += -Wl,-rpath-link=${TARGET_TMPROOTFS_DIR}/lib/hbmedia
LDFLAGS += -Wl,-rpath-link=${TARGET_TMPUNITTEST_DIR}/libbpu_predict
LDFLAGS += -Wl,-rpath-link=${SRC_HOST_DIR}/third_party/usr/lib

INCS = -I${TARGET_TMPROOTFS_DIR}/include
INCS += -I${TARGET_TMPROOTFS_DIR}/include/log
INCS += -I${TARGET_TMPROOTFS_DIR}/include/vio
INCS += -I${TARGET_TMPROOTFS_DIR}/include/hbmedia
INCS += -I${TARGET_TMPROOTFS_DIR}/include/libmm
INCS += -I${TARGET_TMPROOTFS_DIR}/include/cam
INCS += -I${TARGET_TMPROOTFS_DIR}/include/iar
INCS += -I${TARGET_TMPROOTFS_DIR}/include/linux
INCS += -I${TARGET_TMPROOTFS_DIR}/include/isp
INCS += -I${TARGET_TMPROOTFS_DIR}/include/hbusb
INCS += -I${SRC_HOST_DIR}/third_party/usr/include
INCS += -I${SRC_HOST_DIR}/third_party/usr/include
INCS += -I${SRC_HOST_DIR}/third_party/usr/include/cjson

LIBS =

ifeq (${HR_ROOTFS_TYPE},ubuntu)
	CFLAGS += --sysroot=${TARGET_TMPROOTFS_DIR}
	CXXFLAGS += --sysroot=${TARGET_TMPROOTFS_DIR}
	CFLAGS_DYNAMIC += --sysroot=${TARGET_TMPROOTFS_DIR}
	LDFLAGS += -L${TARGET_TMPROOTFS_DIR}/usr/lib/aarch64-linux-gnu --sysroot=${TARGET_TMPROOTFS_DIR}
	# If you use the toolchain compiled by yocto later, you should comment this out
	export LD_LIBRARY_PATH=${TOOLCHAIN_LD_LIBRARY_PATH}
endif
