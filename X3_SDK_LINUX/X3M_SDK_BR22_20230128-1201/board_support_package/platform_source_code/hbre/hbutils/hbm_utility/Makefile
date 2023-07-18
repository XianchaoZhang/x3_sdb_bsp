include ${SRC_BUILD_DIR}/core/Makefile

ifeq (${BUILD_OUTPUT_PATH},)
export BUILD_OUTPUT_PATH=$(shell pwd)
endif

.PHONY: all all_32 clean install

SRCS = $(wildcard *.c)

OBJECT := ${BUILD_OUTPUT_PATH}/hrut_hbm

LIBS= -lcjson

all all_32: $(OBJECT)

$(OBJECT): $(SRCS)
	@mkdir -p $(abspath $(dir $@))
	${CC} ${CFLAGS} $^ ${INCS} ${LDFLAGS} $(LIBS) -o $@

clean:
	rm -rf $(OBJECT)

install:
	@install -d ${TARGET_UNITTEST_DIR}/bin
	install $(OBJECT) ${TARGET_UNITTEST_DIR}/bin/
	@install -d ${TARGET_UNITTEST_DIR}/testcase/libbpu/MODEL-000
	install MODEL-000/* ${TARGET_UNITTEST_DIR}/testcase/libbpu/MODEL-000/
	@install -d ${TARGET_UNITTEST_DIR}/testcase/libbpu/MODEL-957
	install MODEL-957/* ${TARGET_UNITTEST_DIR}/testcase/libbpu/MODEL-957/
