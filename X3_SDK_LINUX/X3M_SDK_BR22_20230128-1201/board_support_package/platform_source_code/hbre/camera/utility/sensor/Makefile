include  ${SRC_BUILD_DIR}/core/Makefile

ifeq (${BUILD_OUTPUT_PATH},)
export BUILD_OUTPUT_PATH=$(shell pwd)
endif
SUBSYS_NAME = sensor

CFLAGS += -DSUBSYS_NAME=${SUBSYS_NAME}
INCS += -I../../  -I./ -I../ -I${TOPDIR}/hbre/liblog/include
LIBS += -lcam -lalog -lpthread -lm
LDFLAGS += -L${BUILD_OUTPUT_PATH}../../ -Wl,-rpath=/usr/lib/hobot/

.PHONY: all all_32 clean

SRCS = $(wildcard ./*_utility.c)
LIB_OBJECT = $(patsubst ./%,${BUILD_OUTPUT_PATH}/%,$(patsubst ./%,./lib%,$(subst _utility.c,.so,$(SRCS))))
OBJS = $(patsubst ./%,${BUILD_OUTPUT_PATH}/%,$(SRCS:.c=.o))

all all_32:$(LIB_OBJECT)

${BUILD_OUTPUT_PATH}/%.o:./%.c
	@mkdir -p $(abspath $(dir $@))
	$(CC) $(CFLAGS) $(INCS) -g -o $@ -fpic -c $<

$(LIB_OBJECT):$(OBJS)
	@mkdir -p $(abspath $(dir $@))
	$(CC) $(CFLAGS_DYNAMIC) -o $@ $(patsubst ./%,${BUILD_OUTPUT_PATH}/%,$(subst lib,,$(subst .so,_utility.o,$@))) $(LDFLAGS) $(LIBS)

clean:
	rm -rf $(LIB_OBJECT) $(OBJS)
