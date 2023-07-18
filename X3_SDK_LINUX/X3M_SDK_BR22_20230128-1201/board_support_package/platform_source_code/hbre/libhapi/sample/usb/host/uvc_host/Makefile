include ${SRC_BUILD_DIR}/core/Makefile

ifeq (${BUILD_OUTPUT_PATH},)
export BUILD_OUTPUT_PATH=$(shell pwd)
endif

.PHONY: all all_32 clean

SRC = $(wildcard  ./src/*.c)

OBJECT = ${BUILD_OUTPUT_PATH}/uvc-host
OBJS = $(patsubst ./%,${BUILD_OUTPUT_PATH}/%,$(SRC:.c=.o))

CFLAGS += -D_GNU_SOURCE
INCS += -I$(TARGET_TMPROOTFS_DIR)/include/uvc/ -I./inc
LIBS += -luvc -lrt

all all_32: $(OBJECT)

$(OBJECT): $(OBJS)
	@mkdir -p $(abspath $(dir $@))
	$(CC) $(CFLAGS) $(OBJS) $(LDFLAGS) -Wl,--start-group $(LIBS) -Wl,--end-group -o $@

${BUILD_OUTPUT_PATH}/%.o: %.c
	@mkdir -p $(abspath $(dir $@))
	$(CC) $(CFLAGS) -c $< $(INCS) $(LDFLAGS) $(LIBS) -o $@

clean:
	rm -rf $(OBJECT) $(OBJS)
