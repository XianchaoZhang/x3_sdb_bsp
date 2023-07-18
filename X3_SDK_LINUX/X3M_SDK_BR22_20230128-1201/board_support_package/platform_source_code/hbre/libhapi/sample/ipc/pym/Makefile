include ${SRC_BUILD_DIR}/core/Makefile

ifeq (${BUILD_OUTPUT_PATH},)
export BUILD_OUTPUT_PATH=$(shell pwd)
endif

.PHONY: all all_32 clean

SRC = $(wildcard  ./*.c)
SRC += $(wildcard  ../common/*.c)
SRCPP = $(wildcard  ./*.cpp)
SRCPP := $(wildcard  ../common/*.cpp)

OBJECT = hapi_bpu
OBJS = $(SRC:.c=.o) $(SRCPP:.cpp=.o)

LDFLAGS += -L$(TARGET_PREROOTFS_DIR)/lib -L./lib

INCS += -I$(TARGET_TMPROOTFS_DIR)/include/vio
INCS += -I$(TARGET_TMPROOTFS_DIR)/include/hbmedia
INCS += -I$(TARGET_TMPROOTFS_DIR)/include/libmm
INCS += -I../inc -I../inc/hbdk -I../common

LIBS += -lcam	\
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
	-lpthread -lhbrt_bernoulli_aarch64 -lcnn_intf -lgdcbin

all all_32: $(OBJECT)

$(OBJECT):$(OBJS)
	@mkdir -p $(abspath $(dir $@))
	$(CXX) $(CFLAGS) $^ $(INCS) $(LDFLAGS) $(LIBS) -o $@

%.o:%.c
	@mkdir -p $(abspath $(dir $@))
	$(CC) $(CFLAGS) -c $< $(INCS) -o $@

%.o:%.cpp
	@mkdir -p $(abspath $(dir $@))
	$(CXX) $(CXXFLAGS) -c $< $(INCS) -o $@

clean:
	rm -rf $(OBJECT) $(OBJS)
