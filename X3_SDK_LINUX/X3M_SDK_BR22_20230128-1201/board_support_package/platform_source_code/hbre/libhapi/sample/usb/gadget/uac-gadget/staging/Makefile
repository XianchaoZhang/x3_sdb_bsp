include ${SRC_BUILD_DIR}/core/Makefile

ifeq (${BUILD_OUTPUT_PATH},)
export BUILD_OUTPUT_PATH=$(shell pwd)
endif

.PHONY: all all_32 clean

SRC = $(wildcard  ./*.c)

OBJECT = ${BUILD_OUTPUT_PATH}/uac-record
OBJS = $(patsubst ./%,${BUILD_OUTPUT_PATH}/%,$(SRC:.c=.o))

CFLAGS += -D_GNU_SOURCE
INCS += -I./inc
LIBS += -lasound -lpthread

all all_32: $(OBJECT)

$(OBJECT): $(OBJS)
	@mkdir -p $(abspath $(dir $@))
	$(CC) $(CFLAGS) $^ $(LDFLAGS) $(LIBS) -o $@

${BUILD_OUTPUT_PATH}/%.o: %.c
	@mkdir -p $(abspath $(dir $@))
	$(CC) $(CFLAGS) -c $<  $(INCS) -o $@

clean:
	rm -rf $(OBJECT) $(OBJS)
