include ${SRC_BUILD_DIR}/core/Makefile

ifeq (${BUILD_OUTPUT_PATH},)
export BUILD_OUTPUT_PATH=$(shell pwd)
endif

.PHONY: all all_32 clean install uninstall

SRCS_C = $(wildcard ./*.c)
OBJS_ORI = $(SRCS_C:.c=)
OBJS = $(patsubst ./%,${BUILD_OUTPUT_PATH}/%,$(OBJS_ORI))

INCS += -I../
LIBS = -lpthread

all all_32: $(OBJS)

${BUILD_OUTPUT_PATH}/veeprom.o: ../veeprom.c
	@echo CC $@
	@mkdir -p $(abspath $(dir $@))
	@${CC} ${CFLAGS} -c $< ${INCS} ${LDFLAGS} $(LIBS) -o $@

${BUILD_OUTPUT_PATH}/%.o: ./%.c
	@mkdir -p $(abspath $(dir $@))
	@${CC} ${CFLAGS} -c $< ${INCS} ${LDFLAGS} $(LIBS) -o $@

${BUILD_OUTPUT_PATH}/%: ${BUILD_OUTPUT_PATH}/%.o ${BUILD_OUTPUT_PATH}/veeprom.o
	@echo CC $@
	@mkdir -p $(abspath $(dir $@))
	@${CC} ${CFLAGS} $^ ${INCS} ${LDFLAGS} $(LIBS) -o $@

clean:
	rm -rf $(OBJS) ${BUILD_OUTPUT_PATH}/*.o

install:
	@install -d ${BUILD_OUTPUT_PATH}/../_install
	install $(OBJS) ${BUILD_OUTPUT_PATH}/../_install

uninstall:
	rm -fr ${BUILD_OUTPUT_PATH}/../_install
