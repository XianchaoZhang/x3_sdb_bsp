include ${SRC_BUILD_DIR}/core/Makefile

ifeq (${BUILD_OUTPUT_PATH},)
export BUILD_OUTPUT_PATH=$(shell pwd)
endif

.PHONY: all all_32 clean

SRC := $(wildcard *.c)

OBJECT = ${BUILD_OUTPUT_PATH}/hbre/libhapi/sample/usb/host/hid-host/hid_demo
OBJS = $(addprefix ${BUILD_OUTPUT_PATH}/hbre/libhapi/sample/usb/host/hid-host/, $(SRC:.c=.o))

all all_32: $(OBJECT)

$(OBJECT): $(SRC)
	@mkdir -p $(abspath $(dir $@))
	$(CC) $(CFLAGS) $^ $(LDFLAGS) -o $@

clean:
	rm -fr $(OBJS) $(OBJECT)
