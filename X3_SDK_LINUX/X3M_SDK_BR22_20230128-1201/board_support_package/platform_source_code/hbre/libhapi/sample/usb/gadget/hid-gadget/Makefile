include ${SRC_BUILD_DIR}/core/Makefile

ifeq (${BUILD_OUTPUT_PATH},)
export BUILD_OUTPUT_PATH=$(shell pwd)
endif

.PHONY: all all_32 clean

SRCS = hid_demo.c

OBJECT := ${BUILD_OUTPUT_PATH}/hid-demo

all all_32: $(OBJECT)

$(OBJECT): $(SRCS)
	@mkdir -p $(abspath $(dir $@))
	$(CC) $(CFLAGS) $^ $(INCS) $(LDFLAGS) -o $@

clean:
	rm -f $(OBJECT)
