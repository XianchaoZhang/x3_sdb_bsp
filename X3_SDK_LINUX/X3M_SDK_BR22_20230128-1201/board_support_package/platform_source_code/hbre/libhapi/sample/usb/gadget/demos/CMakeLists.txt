cmake_minimum_required(VERSION 2.8)
project(demos)
set(CMAKE_C_COMPILER $ENV{CROSS_COMPILE}gcc)
set(CMAKE_CXX_COMPILER $ENV{CROSS_COMPILE}g++)
set(OUTPUT_ROOT ${CMAKE_SOURCE_DIR}/output/)
set(ROOTFS_DIR $ENV{TARGET_TMPROOTFS_DIR})

MESSAGE(STATUS "TARGET_TMPROOTFS_DIR = $ENV{TARGET_TMPROOTFS_DIR}")
MESSAGE(STATUS "ROOTFS = ${ROOTFS_DIR}")

list(APPEND CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -std=c++11")
string(REGEX REPLACE ";" " " CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})

include_directories(
    inc
    inc/vio
    inc/bind
    inc/rtsp/main/include
    inc/rtsp/utils/include/utils/
    ${ROOTFS_DIR}/include
    ${ROOTFS_DIR}/include/guvc
    ${ROOTFS_DIR}/include/hbmedia
    ${ROOTFS_DIR}/include/vio
    ${ROOTFS_DIR}/include/libmm
)

link_directories(
    ${CMAKE_SOURCE_DIR}/lib/
    ${CMAKE_SOURCE_DIR}/lib/jsoncpp
    ${CMAKE_SOURCE_DIR}/lib/rtsp
    ${ROOTFS_DIR}/lib
    ${ROOTFS_DIR}/lib/hbmedia
    ${ROOTFS_DIR}/lib/hbbpu
    ${ROOTFS_DIR}/usr/lib
)

set(BASE_LIBRARIES hbmedia cam vio gdcbin ion multimedia alog avformat
    avcodec avutil swresample tinyalsa iar isp isp_algo jsoncpp live555 pthread
)

file(GLOB PROJECT_SRC
    "src/vio/*.cpp"
    "src/camera/camera_os8a10.cpp"
    "src/camera/camera_f37.cpp"
    "src/camera/camera_s5kgm1.cpp"
    "src/camera/camera_sc8238.cpp"
    "src/camera/camera_imx415.cpp"
    "src/utils/utils.cpp"
    "src/rtsp/xj3_live555.cpp"
)

set(SOURCE_FILES ${PROJECT_SRC})
add_library(${PROJECT_NAME} STATIC ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${BASE_LIBRARIES})

add_subdirectory(usb_camera_eptz)

install(DIRECTORY
        ${CMAKE_SOURCE_DIR}/config/
        DESTINATION ${OUTPUT_ROOT}/config/)

install(FILES
    ${CMAKE_SOURCE_DIR}/run_usb_cam.sh
    DESTINATION ${OUTPUT_ROOT}/)

install(FILES
    ${CMAKE_SOURCE_DIR}/build/usb_camera_eptz/usb_camera_eptz
    DESTINATION ${OUTPUT_ROOT}/)
