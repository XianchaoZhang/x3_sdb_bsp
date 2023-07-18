include ${SRC_BUILD_DIR}/core/Makefile

ifeq (${BUILD_OUTPUT_PATH},)
export BUILD_OUTPUT_PATH=$(shell pwd)
endif

.PHONY: all all_32 clean

SRCS = main.c

OBJECT := ${BUILD_OUTPUT_PATH}/isp_xt
OBJS := ${BUILD_OUTPUT_PATH}/$(SRCS:.c=.o)

INCS += -I$(TARGET_TMPROOTFS_DIR)/include/hbmedia
INCS += -I$(TARGET_TMPROOTFS_DIR)/include/vio
LIBS += -lcam	\
	-lvio		\
	-lgdcbin	\
	-lalog		\
	-lisp_algo	\
	-lmultimedia	\
	-lz -lavformat	\
	-lavcodec	\
	-lavutil	\
	-lswresample	\
	-lhbmedia	\
	-ldl		\
	-lpthread	\
	-lion -ltinyalsa -liar -lisp -lcjson -lgdcbin

all all_32: $(OBJECT)

$(OBJECT): $(OBJS)
	@mkdir -p $(abspath $(dir $@))
	$(CC) $(CFLAGS) $^ $(LDFLAGS) $(LIBS) -o $@

${BUILD_OUTPUT_PATH}/%.o: %.c
	@mkdir -p $(abspath $(dir $@))
	$(CC) $(CFLAGS) -c $< $(INCS) -o $@

clean:
	rm -rf $(OBJECT) $(OBJS)
