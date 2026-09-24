cmake_minimum_required(VERSION 3.16)
project(openscan VERSION 0.6.13 LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs calib highgui rgbd stereo geometry)
find_package(Qt6 REQUIRED COMPONENTS Widgets)
find_package(CURL REQUIRED)
find_package(OpenSSL REQUIRED)

add_executable(openscan
    src/main.cpp
    src/device/openscan_v4l2.c
    src/calib/openscan_calib.c
    src/calib/openscan_calib_fetch.c
    src/scan/openscan_scan.cpp
    src/live/openscan_live.cpp
    src/live/openscan_oneshot.cpp
    src/app/openscan_app.cpp
)

target_include_directories(openscan PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/live
    ${OpenCV_INCLUDE_DIRS}
)

target_compile_definitions(openscan PRIVATE
    OPENSCAN_CALIB_DIR="${CMAKE_CURRENT_SOURCE_DIR}/calib"
    OPENSCAN_VERSION="${PROJECT_VERSION}"
)

target_compile_options(openscan PRIVATE
    -Wall -Wextra
    $<$<COMPILE_LANGUAGE:CXX>:-mno-direct-extern-access>
)

# Link the OpenCV shared libraries directly. The imported opencv_calib
# target also pulls opencv_viz, whose VTK libraries are not installed.
set(_openscan_opencv
    opencv_highgui opencv_rgbd opencv_stereo opencv_calib
    opencv_imgcodecs opencv_imgproc opencv_geometry opencv_core
)
set(_openscan_opencv_paths "")
foreach(_lib IN LISTS _openscan_opencv)
    find_library(_openscan_${_lib} NAMES ${_lib} REQUIRED)
    list(APPEND _openscan_opencv_paths ${_openscan_${_lib}})
endforeach()

target_link_libraries(openscan PRIVATE
    Qt6::Widgets
    ${_openscan_opencv_paths}
    CURL::libcurl
    OpenSSL::Crypto
)

set_target_properties(openscan PROPERTIES
    OUTPUT_NAME openscan
    BUILD_RPATH_USE_ORIGIN ON
)
