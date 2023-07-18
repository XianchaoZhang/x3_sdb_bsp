include  ${SRC_BUILD_DIR}/core/Makefile

ifeq (${BUILD_OUTPUT_PATH},)
export BUILD_OUTPUT_PATH=$(shell pwd)
endif
MK_FILE_DIR = $(shell pwd)
libhbipc = ${SRC_HBRE_DIR}/libhbipc

CFLAGS += -D_GNU_SOURCE
INCS   += -I$(libhbipc)/include
LIBS += -lpthread

all: ${BUILD_OUTPUT_PATH}/vcam_daemon

${BUILD_OUTPUT_PATH}/vcam_daemon: $(MK_FILE_DIR)/vcam_daemon.c $(libhbipc)/lib/hbipc_cp.c
	@mkdir -p $(abspath $(dir $@))
	$(CC) $(CFLAGS) -o $@ $(MK_FILE_DIR)/vcam_daemon.c $(libhbipc)/lib/hbipc_cp.c $(LIBS) $(INCS)

clean:
	rm -rf ${BUILD_OUTPUT_PATH}/*.o ${BUILD_OUTPUT_PATH}/vcam_daemon
