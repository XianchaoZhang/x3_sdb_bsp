include ${SRC_BUILD_DIR}/core/Makefile

ifeq (${BUILD_OUTPUT_PATH},)
export BUILD_OUTPUT_PATH=$(shell pwd)
endif

.PHONY: all all_32 clean


SRC = $(wildcard  ./*.c)

OBJECT := ${BUILD_OUTPUT_PATH}/sample_splice
OBJS := $(patsubst ./%,${BUILD_OUTPUT_PATH}/%,$(SRC:.c=.o))

LIBS += -lcam	\
	-lvio	\
	-lgdcbin \
	-lisp		\
	-liar		\
	-lisp_algo	\
	-ltinyalsa	\
	-lmultimedia	\
	-lz -lavformat	\
	-lavcodec	\
	-lavutil	\
	-lswresample	\
	-lhbmedia	\
	-ldl		\
	-lpthread	\
	-lion              \
	-lalog -lcjson

all all_32: $(OBJECT)

$(OBJECT): $(OBJS)
	@mkdir -p $(abspath $(dir $@))
	$(CC) $(CFLAGS) $^ $(LDFLAGS) $(LIBS) -Wl,--start-group $(LIBS) -Wl,--end-group -o $@

${BUILD_OUTPUT_PATH}/%.o: %.c
	@mkdir -p $(abspath $(dir $@))
	$(CC) $(CFLAGS) -c $< $(INCS) -o $@

clean:
	rm -rf $(OBJECT) $(OBJS)
