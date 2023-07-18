include ${SRC_BUILD_DIR}/core/Makefile

ifeq (${BUILD_OUTPUT_PATH},)
export BUILD_OUTPUT_PATH=$(shell pwd)
endif

.PHONY: all all_32 clean

SRC := $(wildcard  ./src/*.c)
SRCPP := $(wildcard  ./src/*.cpp)

OBJECT = ${BUILD_OUTPUT_PATH}/dual_group
OBJS = $(patsubst ./%,${BUILD_OUTPUT_PATH}/%,$(SRC:.c=.o)) $(patsubst ./%,${BUILD_OUTPUT_PATH}/%,$(SRCPP:.cpp=.o))

LDFLAGS +=  -L./lib
INCS += -I./inc -I./inc/hbdk/
LIBS += -lcam	\
	-lalog	\
	-lvio	\
	-lisp		\
	-liar		\
	-lisp_algo	\
	-ltinyalsa	\
	-lmultimedia	\
	-lz -lavformat	\
	-lavcodec	\
	-lavutil	\
	-lswresample	\
	-lhbmedia	\
	-ldl		\
	-lion		\
	-lpthread -lhbrt_bernoulli_aarch64 -lcnn_intf -lgdcbin -lcjson

all all_32: $(OBJECT)

$(OBJECT): $(OBJS)
	@mkdir -p $(abspath $(dir $@))
	$(CXX) $(CXXFLAGS) $^ $(LDFLAGS) -Wl,--start-group $(LIBS) -Wl,--end-group -o $@

${BUILD_OUTPUT_PATH}/%.o: %.c
	@mkdir -p $(abspath $(dir $@))
	$(CC) $(CFLAGS) -c $< $(INCS) -o $@

${BUILD_OUTPUT_PATH}/%.o: %.cpp
	@mkdir -p $(abspath $(dir $@))
	$(CXX) $(CFLAGS) -c $< $(INCS) -o $@

clean:
	rm -rf $(OBJECT) $(OBJS)
