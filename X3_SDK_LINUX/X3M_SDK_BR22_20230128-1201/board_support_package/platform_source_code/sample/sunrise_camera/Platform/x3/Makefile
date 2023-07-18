include ../makefile.param
include ./makefile.param

INC_DIR := main api x3_camera x3_sdk_swap x3_bpu x3_gusb $(APPSDK_DIR)
INC_DIR += $(GLOBAL_EXTERN_INC_DIR) $(GLOBAL_INSTALL_DIR)
INC := $(patsubst %,-I%/include,$(INC_DIR))
LIB := $(patsubst %,-L%/lib,$(INC_DIR))

# X3 sdk 的自带头文件里面对自身头文件的引用依赖问题
# 在sdk自带的头文件中存在互相include的情况，所以需要把被包含头文件的路径include进来
INC += -I$(APPSDK_DIR)/include/vio

# ffmpeg库、头文件目录
INC += -I$(APPSDK_DIR)/include/libmm

# libdnn 库、头文件目录
#INC += -I$(APPSDK_DIR)/..//dnn/include

SRC := $(wildcard *.cpp *.c $(patsubst %,%/src/*.cpp,$(INC_DIR)) $(patsubst %,%/src/*.c,$(INC_DIR)))
OBJ := $(patsubst %.cpp,%.obj,$(patsubst %.c,%.o,$(SRC)))
DEP := $(patsubst %.obj,%.dep,$(patsubst %.o,%.d,$(OBJ)))

CC := $(COMPILE_PREFIX)gcc
CFLAGS := $(INC) -O0 -DBSD=1 -fPIC $(CFLAGS_EX)
#CFLAGS := $(INC) -O0 -Wall -DBSD=1 -fPIC
CXX := $(COMPILE_PREFIX)c++
CXXFLAGS := $(CFLAGS)
LINK := $(COMPILE_PREFIX)c++ -o
LIBRARY_LINK := $(COMPILE_PREFIX)c++ -shared -o
LIBRARY_LINK_STATIC := $(COMPILE_PREFIX)ar cr
LIBRARY_LINK_OPTS =
STRIP := $(COMPILE_PREFIX)strip

TARGET := libcamera.a

.PHONY : all clean

all : $(TARGET) install

%.o : %.c
	$(CC) $(CFLAGS) -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c $< -o $@

%.obj : %.cpp
	$(CXX) $(CXXFLAGS) -MMD -MP -MF"$(@:%.obj=%.dep)" -MT"$(@:%.obj=%.dep)" -c $< -o $@

$(TARGET) : $(OBJ)
	$(LIBRARY_LINK_STATIC) $@ $^

install:
	-mkdir -p $(GLOBAL_INSTALL_DIR)/lib
	-mkdir -p $(GLOBAL_INSTALL_DIR)/test_data
	-mkdir -p $(GLOBAL_INSTALL_DIR)/model_zoom
	mv $(TARGET) $(GLOBAL_INSTALL_DIR)/lib
	cp -rf x3_tools/* $(GLOBAL_INSTALL_DIR)/bin
	cp -rf test_data/* $(GLOBAL_INSTALL_DIR)/test_data
	cp -rf model_zoom/* $(GLOBAL_INSTALL_DIR)/model_zoom

clean:
	@rm -rf $(OBJ) $(DEP) $(TARGET)

sinclude $(DEP)

