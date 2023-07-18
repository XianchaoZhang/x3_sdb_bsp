include ${SRC_BUILD_DIR}/core/Makefile

ifeq (${BUILD_OUTPUT_PATH},)
export BUILD_OUTPUT_PATH=$(shell pwd)
endif

.PHONY: all all_32 clean

SRC := $(wildcard  ./*.c) $(wildcard  ../common/*.c)
SRCPP := $(wildcard  ./*.cpp) $(wildcard  ../common/*.cpp)

OBJECT = hapi_test
OBJS = $(SRC:.c=.o) $(SRCPP:.cpp=.o)

LDFLAGS += -L../lib
INCS += -I../inc -I../inc/hbdk -I../common
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
	-lpthread -lcnn_intf -lgdcbin

all all_32: $(OBJECT)

$(OBJECT): $(OBJS)
	$(CXX) $(CFLAGS) $^ $(LDFLAGS) -Wl,--start-group $(LIBS) -Wl,--end-group -o $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< $(INCS) -o $@
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< $(INCS) -o $@

clean:
	rm -rf $(OBJECT) $(OBJS)
