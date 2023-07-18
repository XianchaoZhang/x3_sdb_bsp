include  ${SRC_BUILD_DIR}/core/Makefile

ifeq (${BUILD_OUTPUT_PATH},)
export BUILD_OUTPUT_PATH=$(shell pwd)
endif

all all_32: tuning_tool

LIBS += -lvio -lcam -lisp -lhbplayer -liar -lhbmedia -lpthread -ldl -lcjson
LIBS += -lalog  \
        -lisp_algo      \
        -ltinyalsa      \
        -lmultimedia    \
        -lavformat  \
        -lavcodec       \
        -lavutil        \
        -lswresample    \
        -ldl            \
        -lion           \
        -lcnn_intf -lgdcbin

tuning_tool: vio_tool.c gdc_json_parser.c
	@mkdir -p ${BUILD_OUTPUT_PATH}/
	$(CC) $(CFLAGS) $^ $(LDFLAGS) $(LIBS) $(INCS) -o ${BUILD_OUTPUT_PATH}/$@
%.o:%.c
	@mkdir -p ${BUILD_OUTPUT_PATH}/
	$(CC) $(CFLAGS)  -c $^ -o ${BUILD_OUTPUT_PATH}/$@

clean:
	-rm -f ${BUILD_OUTPUT_PATH}/tuning_tool
