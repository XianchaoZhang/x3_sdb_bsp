include ${SRC_BUILD_DIR}/core/Makefile

ifeq (${BUILD_OUTPUT_PATH},)
export BUILD_OUTPUT_PATH=$(shell pwd)
endif

.PHONY: all all_32 clean

SRCS = $(wildcard ./*.c)

OBJECT := ${BUILD_OUTPUT_PATH}/sample_audio
OBJS := $(SRCS:.c=.o)

INCS += -I./inc -I$(TARGET_TMPROOTFS_DIR)/include/vio -I$(TARGET_TMPROOTFS_DIR)/include/libmm
LIBS += -lcam -lalog -lvio -lisp -liar -lisp_algo -ltinyalsa -lmultimedia -lz -lhbmedia -ldl -lpthread -lion -lcjson -lhbmem -lgdcbin

all all_32: clean $(OBJECT)

$(OBJECT): $(SRCS)
	@mkdir -p $(abspath $(dir $@))
	$(CC) $(CFLAGS) $^ $(INCS) -Wl,--start-group $(LIBS) -Wl,--end-group $(LDFLAGS) -o $@

clean:
	rm -rf $(OBJECT) $(OBJS)
