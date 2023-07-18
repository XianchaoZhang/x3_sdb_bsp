include  ${SRC_BUILD_DIR}/core/Makefile

TOR_DIR 	= $(shell pwd)
SRC_DIR 	= $(TOR_DIR)/src
OBJS_PATH 	= $(TOR_DIR)/objs

INCS += -I$(TOR_DIR)/src

CXXFLAGS += -fPIC -ldl -shared
LIBS += -lhbplayer -lpthread

SRC_BASE		= $(shell find $(SRC_DIR)/base -name *.cpp)
SRC_NET			= $(shell find $(SRC_DIR)/net -name *.cpp)

SRC_BASE_NODIR			= $(notdir $(wildcard $(SRC_BASE)))
SRC_NET_NODIR			= $(notdir $(wildcard $(SRC_NET)))

OBJS_BASE			= $(patsubst %.cpp,$(OBJS_PATH)/%.o,$(SRC_BASE_NODIR))
OBJS_NET			= $(patsubst %.cpp,$(OBJS_PATH)/%.o,$(SRC_NET_NODIR))

OBJ_EXAMPLE_H264_RTSP_SERVER = $(OBJS_PATH)/h264_rtsp_server.o

TARGET_TEST_EXAMPLE_H264_RTSP_SERVER		= $(TOR_DIR)/example/libh264server.so

all: MKAE_DIR TARGET_EXAMPLE

MKAE_DIR:
	@-mkdir -p $(OBJS_PATH)

TARGET_EXAMPLE : $(TARGET_TEST_EXAMPLE_H264_RTSP_SERVER)

$(TARGET_TEST_EXAMPLE_H264_RTSP_SERVER) : $(OBJ_EXAMPLE_H264_RTSP_SERVER) $(OBJS_BASE) $(OBJS_NET)
	$(CXX) $^ -o $@ $(SRCS) $(LIBS) $(CXXFLAGS) $(LDFLAGS) $(INCS)


$(OBJS_PATH)/%.o : $(TOR_DIR)/example/%.cpp
	$(CXX) -c -o $@ $< $(CXXFLAGS) $(LIBS) $(LDFLAGS) $(INCS)
$(OBJS_PATH)/%.o : $(SRC_DIR)/base/%.cpp
	$(CXX) -c -o $@ $< $(CXXFLAGS) $(LIBS) $(LDFLAGS) $(INCS)
$(OBJS_PATH)/%.o : $(SRC_DIR)/net/%.cpp
	$(CXX) -c -o $@ $< $(CXXFLAGS) $(LIBS) $(LDFLAGS) $(INCS)
$(OBJS_PATH)/%.o : $(SRC_DIR)/net/poller/%.cpp
	$(CXX) -c -o $@ $< $(CXXFLAGS) $(LIBS) $(LDFLAGS) $(INCS)

.PHONY : clean
clean:
	rm -rf $(OBJS_PATH)
	rm -rf $(TARGET_TEST_EXAMPLE_H264_RTSP_SERVER)
