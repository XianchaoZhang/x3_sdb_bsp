include  ${SRC_BUILD_DIR}/core/Makefile

ifeq (${BUILD_OUTPUT_PATH},)
export BUILD_OUTPUT_PATH=$(shell pwd)
endif
LIB_OBJECT = ${BUILD_OUTPUT_PATH}/libcam.so
LIB_OBJECT_STC = ${BUILD_OUTPUT_PATH}/libcam.a
SUBSYS_NAME = camera
LIB_VIN = ${BUILD_OUTPUT_PATH}/src/libvin.a
VIN_SRCS = $(wildcard ./src/hb_vin*.c)
SRCS = $(filter-out $(VIN_SRCS),$(wildcard ./src/*.c))
SRCS += $(wildcard ./utility/*.c)

INCS += -I. -I./inc -I./utility

ifeq ($(NODIAG), y)
CFLAGS  := -O2 -DNODIAG
LDFLAGS = -lvio -lpthread -lstdc++
else
CFLAGS  := -O2
LIBS += -lalog -ldiag -lpthread -lrt -lstdc++ -ljsoncpp
endif
LDFLAGS += -L${BUILD_OUTPUT_PATH}/src/

ifeq ($(NOSIF), y)
CFLAGS  += -DNOSIF
endif

CFLAGS += -DSUBSYS_NAME=${SUBSYS_NAME}

VIN_OBJS = $(patsubst ./%,${BUILD_OUTPUT_PATH}/%,$(VIN_SRCS:.c=.o))
OBJS = $(patsubst ./%,${BUILD_OUTPUT_PATH}/%,$(SRCS:.c=.o))



.PHONY: all all_32 clean sensor desrial vcam
all all_32:$(LIB_OBJECT) $(LIB_OBJECT_STC) sensor vcam

$(LIB_VIN): $(VIN_OBJS)
	$(AR) $(CFLAGS_STATIC) $@ $(VIN_OBJS)

ifneq ($(VINRPC),)
ifneq ($(VINRPC), y)
CFLAGS += -DVINRPC_HOST='"$(VINRPC)"'
endif
VINSVC_SRCS = $(wildcard ./vin_rpc/hb_vin_rpc_svc*.c) ./vin_rpc/hb_vin_rpc_xdr.c
VINSVC_OBJS = $(patsubst ./%,${BUILD_OUTPUT_PATH}/%,$(VINSVC_SRCS:.c=.o))
BINSVC_VINRPC = ./vin_rpc/vinrpc_svc

$(BINSVC_VINRPC): $(VINSVC_OBJS) $(LIB_VIN)
	@mkdir -p $(abspath $(dir $@))
	$(CC) -o $@ $(VINSVC_OBJS) -fpic $(LDFLAGS) $(LIBS)

VINRPC_SRCS = $(wildcard ./vin_rpc/hb_vin_rpc_clnt*.c) ./vin_rpc/hb_vin_rpc_xdr.c
VINRPC_OBJS = $(patsubst ./%,${BUILD_OUTPUT_PATH}/%,$(VINRPC_SRCS:.c=.o))
LIB_VINRPC = ${BUILD_OUTPUT_PATH}/vin_rpc/libvinrpc.a

$(LIB_VINRPC): $(VINRPC_OBJS) $(BINSVC_VINRPC)
	@mkdir -p $(abspath $(dir $@))
	$(AR) $(CFLAGS_STATIC) $@ $(VINRPC_OBJS)

LIB_VIN_USE = $(LIB_VINRPC)
LIB_VIN_OBJS = $(VINRPC_OBJS)
LDFLAGS += -L${BUILD_OUTPUT_PATH}/vin_rpc
LIBS += -lvinrpc
else
LIB_VIN_USE = $(LIB_VIN)
LIB_VIN_OBJS = $(VIN_OBJS)
LDFLAGS += -Lsrc
LIBS += -lvin
endif

$(LIB_OBJECT): $(OBJS) $(LIB_VIN_USE)
	@mkdir -p $(abspath $(dir $@))
	$(CC) $(CFLAGS_DYNAMIC) -o $@ $(OBJS) $(LDFLAGS) $(LIBS)

$(LIB_OBJECT_STC): $(OBJS) $(LIB_VIN_OBJS)
	@mkdir -p $(abspath $(dir $@))
	$(AR) $(CFLAGS_STATIC) $@ $(OBJS) $(LIB_VIN_OBJS)

${BUILD_OUTPUT_PATH}/%.o:./%.c
	@mkdir -p $(abspath $(dir $@))
	$(CC) $(CFLAGS) $(INCS) -g -o $@ -fpic -c $<

sensor: $(LIB_OBJECT)
	make BUILD_OUTPUT_PATH=${BUILD_OUTPUT_PATH}/utility/sensor/ -C ./utility/sensor

vcam:
	make BUILD_OUTPUT_PATH=${BUILD_OUTPUT_PATH}/utility/vcam/ -C ./utility/vcam

clean:
	rm -rf $(LIB_OBJECT) $(LIB_OBJECT_STC) $(OBJS) ${VIN_OBJS} ${VINRPC_OBJS} ${LIB_VIN} ${LIB_VINRPC}
	make BUILD_OUTPUT_PATH=${BUILD_OUTPUT_PATH}/utility/sensor/ -C ./utility/sensor clean
	make BUILD_OUTPUT_PATH=${BUILD_OUTPUT_PATH}/utility/vcam/ -C ./utility/vcam clean
