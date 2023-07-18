include ${SRC_BUILD_DIR}/core/Makefile

ifeq (${BUILD_OUTPUT_PATH},)
export BUILD_OUTPUT_PATH=$(shell pwd)
endif

.PHONY: all all_32 clean

SRC = $(wildcard  ./ipu/*.c)
SRC += $(wildcard  ./common/*.c)
SRCPP = $(wildcard  ./ipu/*.cpp)
SRCPP += $(wildcard  ./common/*.cpp)

OBJECT = ${BUILD_OUTPUT_PATH}/hapi_bpu
OBJS = $(patsubst ./%,${BUILD_OUTPUT_PATH}/%,$(SRC:.c=.o)) $(patsubst ./%,${BUILD_OUTPUT_PATH}/%,$(SRCPP:.cpp=.o))

LDFLAGS += -L./lib
INCS += -I$(TARGET_TMPROOTFS_DIR)/include/vio
INCS += -I$(TARGET_TMPROOTFS_DIR)/include/hbmedia
INCS += -I$(TARGET_TMPROOTFS_DIR)/include/libmm
INCS += -I./inc -I./inc/hbdk -I./common
LIBS += -lcam	\
	-lalog	\
	-lvio	\
	-lisp		\
	-lisp_algo	\
	-lz -lavformat  \
    -lavcodec       \
    -lavutil        \
    -lswresample    \
	-lmultimedia	\
	-lhbmedia	\
	-ldl		\
	-lion		\
	-lpthread   \
	-lhbrt_bernoulli_aarch64 \
	-lcnn_intf               \
	-lgdcbin                 \
	-lcjson                  \
	-liar                    \
	-ltinyalsa

all all_32: $(OBJECT)

$(OBJECT): $(OBJS)
	@mkdir -p $(abspath $(dir $@))
	$(CXX) $(CFLAGS) $^ $(LDFLAGS) $(LIBS) -o $@

${BUILD_OUTPUT_PATH}/ipu/%.o: ./ipu/%.c
	@mkdir -p $(abspath $(dir $@))
	$(CC) $(CFLAGS) -c $< $(INCS) -o $@

${BUILD_OUTPUT_PATH}/common/%.o: ./common/%.c
	@mkdir -p $(abspath $(dir $@))
	$(CC) $(CFLAGS) -c $< $(INCS) -o $@

${BUILD_OUTPUT_PATH}/ipu/%.o: ./ipu/%.cpp
	@mkdir -p $(abspath $(dir $@))
	$(CXX) $(CFLAGS) -c $< $(INCS) -o $@

${BUILD_OUTPUT_PATH}/common/%.o: ./common/%.cpp
	@mkdir -p $(abspath $(dir $@))
	$(CXX) $(CFLAGS) -c $< $(INCS) -o $@

clean:
	rm -rf $(OBJECT) $(OBJS)
