include ${SRC_BUILD_DIR}/core/Makefile

ifeq (${BUILD_OUTPUT_PATH},)
export BUILD_OUTPUT_PATH=$(shell pwd)
endif

.PHONY: all all_32 clean

SRC = $(wildcard ./src/*.c)

OBJECT = ${BUILD_OUTPUT_PATH}/usb_webcam
OBJS = $(patsubst ./%,${BUILD_OUTPUT_PATH}/%,$(SRC:.c=.o))

CFLAGS += -D_GNU_SOURCE
INCS += -I$(TARGET_TMPROOTFS_DIR)/include/guvc -I./inc
LIBS += -lcam 	\
	-lgdcbin 	\
	-lvio		\
	-lisp		\
	-liar 		\
	-lisp_algo	\
	-ltinyalsa	\
	-lmultimedia	\
	-lz -lavformat	\
	-lavcodec	\
	-lavutil	\
	-lswresample	\
	-lhbmedia	\
	-ldl	\
	-lpthread 	\
	-lion 		\
	-lalog -lcjson -lguvc

all all_32: $(OBJECT)

$(OBJECT): $(OBJS)
	@mkdir -p $(abspath $(dir $@))
	$(CC) $(CFLAGS) $(OBJS) $(LDFLAGS) -Wl,--start-group $(LIBS) -Wl,--end-group -o $@

${BUILD_OUTPUT_PATH}/%.o: %.c
	@mkdir -p $(abspath $(dir $@))
	$(CC) $(CFLAGS) -c $< $(INCS) -o $@

clean:
	rm -rf $(OBJECT) $(OBJS)
