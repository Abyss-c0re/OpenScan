cmake_minimum_required(VERSION 3.18)
project(openscanjni)

set(OPENSCAN_SRC "${CMAKE_CURRENT_LIST_DIR}/../../../../..")

if(NOT ANDROID_NDK_ABI_NAME AND ANDROID_ABI)
    set(ANDROID_NDK_ABI_NAME "${ANDROID_ABI}")
endif()

if(NOT OpenCV_DIR)
    if(DEFINED ENV{OPENCV_ANDROID_SDK})
        set(OpenCV_DIR "$ENV{OPENCV_ANDROID_SDK}/sdk/native/jni")
    elseif(EXISTS "/home/voldemar/Android/OpenCV-android-sdk/sdk/native/jni/OpenCVConfig.cmake")
        set(OpenCV_DIR "/home/voldemar/Android/OpenCV-android-sdk/sdk/native/jni")
    endif()
endif()

if(NOT OpenCV_DIR OR NOT EXISTS "${OpenCV_DIR}/OpenCVConfig.cmake")
    message(FATAL_ERROR
        "OpenCV Android SDK not found. Unpack opencv-4.10.0-android-sdk.zip and set OPENCV_ANDROID_SDK "
        "or place it at /home/voldemar/Android/OpenCV-android-sdk")
endif()

find_package(OpenCV REQUIRED core imgproc imgcodecs calib3d)

add_library(openscanjni SHARED
    openscan_jni.cpp
    "${OPENSCAN_SRC}/src/calib/openscan_calib.c"
    "${OPENSCAN_SRC}/src/live/openscan_live.cpp"
    "${OPENSCAN_SRC}/src/live/openscan_oneshot.cpp"
    "${OPENSCAN_SRC}/src/scan/openscan_scan.cpp"
    "${OPENSCAN_SRC}/src/device/openscan_v4l2.c"
)

target_include_directories(openscanjni PRIVATE
    "${OPENSCAN_SRC}/include"
    "${OPENSCAN_SRC}/src/live"
    ${OpenCV_INCLUDE_DIRS}
)

target_compile_definitions(openscanjni PRIVATE OPENSCAN_VERSION="0.6.13")
target_link_options(openscanjni PRIVATE "-Wl,-z,max-page-size=16384" "-Wl,-z,common-page-size=16384")

target_link_libraries(openscanjni
    ${OpenCV_LIBS}
    log
    android
    z
)
