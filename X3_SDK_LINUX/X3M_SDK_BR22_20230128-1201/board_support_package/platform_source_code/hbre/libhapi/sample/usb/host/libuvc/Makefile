include ${SRC_BUILD_DIR}/core/Makefile

ifeq (${BUILD_OUTPUT_PATH},)
export BUILD_OUTPUT_PATH=$(shell pwd)
endif

.PHONY: all all_32 clean dist-clean install

SRC = demo/uvc_demo.c
LIB_SRC = src/v4l2.c 	\
	src/camera.c 	\
	src/video-buffers.c

OBJS := $(addprefix ${BUILD_OUTPUT_PATH}/,$(SRC:.c=.o))
LIB_OBJS := $(addprefix ${BUILD_OUTPUT_PATH}/,$(LIB_SRC:.c=.o))
BINARY := ${BUILD_OUTPUT_PATH}/uvc-demo
BINARY2 := ${BUILD_OUTPUT_PATH}/uvc-demo-static
LIB_SO := ${BUILD_OUTPUT_PATH}/libuvc.so
LIB_A := ${BUILD_OUTPUT_PATH}/libuvc.a

CFLAGS += -D_GNU_SOURCE
INCS += -I./inc
LIBS += -lpthread

all all_32: $(LIB_SO) $(LIB_A) $(BINARY) $(BINARY2)

${BUILD_OUTPUT_PATH}/%.o: %.c
	@mkdir -p $(abspath $(dir $@))
	$(CC) $(CFLAGS) -c $< $(INCS) -o $@

$(LIB_SO): $(LIB_OBJS)
	@mkdir -p $(abspath $(dir $@))
	$(CC) $(CFLAGS) $(CFLAGS_DYNAMIC) $^ $(LDFLAGS) -o $@

$(LIB_A): $(LIB_OBJS)
	@mkdir -p $(abspath $(dir $@))
	$(AR) $(CFLAGS_STATIC) $@ $^

$(BINARY): $(OBJS) $(LIB_SO)
	@mkdir -p $(abspath $(dir $@))
	$(CC) $(CFLAGS) $^ $(LDFLAGS) $(LIBS) -o $@

$(BINARY2):$(OBJS) $(LIB_A)
	@mkdir -p $(abspath $(dir $@))
	$(CC) $(CFLAGS) $^ $(LDFLAGS) $(LIBS) -o $@

install:
	install -p -D -m 755 $(LIB_SO) ${BUILD_OUTPUT_PATH}/_install/lib/$(LIB_SO)
	install -p -D -m 755 $(BINARY) ${BUILD_OUTPUT_PATH}/_install/bin/$(BINARY)
	@install -d ${BUILD_OUTPUT_PATH}/_install/inc
	install inc/* ${BUILD_OUTPUT_PATH}/_install/inc

dist-clean:
	rm -rf $(BINARY) $(BINARY2) $(OBJS) $(LIB_SO) $(LIB_A) $(LIB_OBJS) ${BUILD_OUTPUT_PATH}/_install

clean:
	rm -rf $(BINARY) $(BINARY2) $(OBJS) $(LIB_SO) $(LIB_A) $(LIB_OBJS)
