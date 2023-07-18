cmake_minimum_required(VERSION 2.8)

set(SOURCE_FILES usb_camera_eptz.cpp)

add_executable(usb_camera_eptz ${SOURCE_FILES})
target_link_libraries(usb_camera_eptz ${PROJECT_NAME} guvc ${BASE_LIBRARIES})
