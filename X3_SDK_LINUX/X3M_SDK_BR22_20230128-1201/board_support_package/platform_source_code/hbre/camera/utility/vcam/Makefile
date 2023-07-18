include  ${SRC_BUILD_DIR}/core/Makefile

ifeq (${BUILD_OUTPUT_PATH},)
export BUILD_OUTPUT_PATH=$(shell pwd)
endif

INCS += -I. -I../../inc
LIBS += -ldl -lpthread

LIB_OBJECT = ${BUILD_OUTPUT_PATH}/libvcam.so
SRCS = $(wildcard ./*.c)
OBJS = $(patsubst ./%,${BUILD_OUTPUT_PATH}/%,$(SRCS:.c=.o))

${BUILD_OUTPUT_PATH}/%.o:./%.c
	@mkdir -p $(abspath $(dir $@))
	$(CC) $(CFLAGS) $(INCS) -g -o $@ -fpic -c $<

.PHONY: all clean
all:$(LIB_OBJECT)
$(LIB_OBJECT): $(OBJS)
	@mkdir -p $(abspath $(dir $@))
	$(CC) $(CFLAGS_DYNAMIC) $(LIBS) -o $@ $(OBJS)
	rm -rf $(OBJS)

clean:
	rm -rf $(LIB_OBJECT)
