cmake_minimum_required(VERSION 3.18)
project(openscanjni C)

set(OPENSCAN_SRC "${CMAKE_CURRENT_LIST_DIR}/../../../../..")
set(CMAKE_C_STANDARD 11)

add_library(openscanjni SHARED
    openscan_jni.c
    "${OPENSCAN_SRC}/src/calib/openscan_calib.c"
    "${OPENSCAN_SRC}/src/live/os_logic.c"
    "${OPENSCAN_SRC}/src/live/os_live.c"
    "${OPENSCAN_SRC}/src/device/openscan_cam_name.c"
    "${OPENSCAN_SRC}/src/device/openscan_v4l2.c"
)

target_include_directories(openscanjni PRIVATE "${OPENSCAN_SRC}/include")
target_compile_definitions(openscanjni PRIVATE OPENSCAN_VERSION="0.6.14" OPENSCAN_HOST="android")
target_link_options(openscanjni PRIVATE "-Wl,-z,max-page-size=16384" "-Wl,-z,common-page-size=16384")
target_link_libraries(openscanjni log android)
