include ${SRC_BUILD_DIR}/core/Makefile

ifeq (${BUILD_OUTPUT_PATH},)
export BUILD_OUTPUT_PATH=$(shell pwd)
endif

.PHONY: all all_32 clean

SRCS = $(wildcard *.c)

OBJECT := ${BUILD_OUTPUT_PATH}/hrut_bpu

INCS += -I../
LIBS = -lpthread -lrt

all all_32: $(OBJECT)

$(OBJECT): $(SRCS)
	@mkdir -p $(abspath $(dir $@))
	${CC} ${CFLAGS} $^ ${INCS} ${LDFLAGS} $(LIBS) -o $@

clean:
	rm -rf $(OBJECT)
