include  ${SRC_BUILD_DIR}/core/Makefile

ifeq (${BUILD_OUTPUT_PATH},)
export BUILD_OUTPUT_PATH=$(shell pwd)
endif
TARGET = ${BUILD_OUTPUT_PATH}/libhbplayer.so
TARGET_STC = ${BUILD_OUTPUT_PATH}/libhbplayer.a

SRCS = $(wildcard ./src/*.c)

INCS += -I./inc
LIBS += -lpthread -fPIC -ldl -shared -lcjson -lvio -lmultimedia -lhbmedia

OBJS = $(SRCS:.c=.o)

%.o:%.c
	@mkdir -p ${BUILD_OUTPUT_PATH}/src/
	$(CC) $(LDFLAGS) $(CFLAGS) $(INCS) $(LIBS) -o ${BUILD_OUTPUT_PATH}/$@ -c $<

.PHONY: all clean
all:$(TARGET) $(TARGET_STC)
$(TARGET): $(OBJS)
	@mkdir -p ${BUILD_OUTPUT_PATH}/
	$(CC) $(LDFLAGS) $(INCS) $(LIBS) -o $@ $(addprefix ${BUILD_OUTPUT_PATH}/,$(OBJS))

$(TARGET_STC): $(OBJS)
	@mkdir -p ${BUILD_OUTPUT_PATH}/
	$(AR) -crv $@ $(addprefix ${BUILD_OUTPUT_PATH}/,$(OBJS))

clean:
	rm -rf ${BUILD_OUTPUT_PATH}/$(TARGET) ${BUILD_OUTPUT_PATH}/$(TARGET_STC) ${BUILD_OUTPUT_PATH}/$(OBJS)
