include ${SRC_BUILD_DIR}/core/Makefile

ifeq (${BUILD_OUTPUT_PATH},)
export BUILD_OUTPUT_PATH=$(shell pwd)
endif

.PHONY: veeprom clean

veeprom:
	${MAKE} BUILD_OUTPUT_PATH=${BUILD_OUTPUT_PATH}/hrut_cleanuserdata -C hrut_cleanuserdata
	${MAKE} BUILD_OUTPUT_PATH=${BUILD_OUTPUT_PATH}/hrut_cleanuserdata -C hrut_cleanuserdata install
	${MAKE} BUILD_OUTPUT_PATH=${BUILD_OUTPUT_PATH}/utility -C utility
	${MAKE} BUILD_OUTPUT_PATH=${BUILD_OUTPUT_PATH}/utility -C utility install

clean:
	${MAKE} BUILD_OUTPUT_PATH=${BUILD_OUTPUT_PATH}/utility -C utility clean
