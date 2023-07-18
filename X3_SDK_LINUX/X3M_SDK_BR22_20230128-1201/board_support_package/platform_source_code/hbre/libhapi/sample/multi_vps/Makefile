include ${SRC_BUILD_DIR}/core/Makefile

ifeq (${BUILD_OUTPUT_PATH},)
export BUILD_OUTPUT_PATH=$(shell pwd)
endif

.PHONY: all all_32 clean

SRC = multi_vps.c

OBJECT = ${BUILD_OUTPUT_PATH}/multi_vps
OBJS = ${BUILD_OUTPUT_PATH}/$(SRC:.c=.o)

LIBS += -lcam	\
	-lvio	\
	-lgdcbin \
	-lisp		\
	-lisp_algo	\
	-lmultimedia	\
	-lz -lavformat	\
	-lavcodec	\
	-lavutil	\
	-lswresample	\
	-lhbmedia	\
	-ldl		\
	-lpthread       \
	-lion		\
	-lalog -lcjson -liar -ltinyalsa

all all_32: $(OBJECT)

$(OBJECT): $(OBJS)
	@mkdir -p $(abspath $(dir $@))
	$(CC) $(CFLAGS) $^ $(INCS) $(LDFLAGS) $(LIBS) -o $@

${BUILD_OUTPUT_PATH}/%.o: %.c
	@mkdir -p $(abspath $(dir $@))
	$(CC) $(CFLAGS) -c $< $(INCS) -o $@

clean:
	rm -rf $(OBJECT) $(OBJS)
