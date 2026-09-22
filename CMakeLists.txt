cmake_minimum_required(VERSION 3.16)
project(fox3d VERSION 0.6.0 LANGUAGES C CXX)

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

add_executable(fox3d
    src/main.cpp
    src/device/fox_v4l2.c
    src/calib/fox_calib.c
    src/calib/fox_calib_fetch.c
    src/scan/fox_scan.cpp
    src/live/fox_live.cpp
    src/live/fox_oneshot.cpp
    src/app/fox_app.cpp
)

target_include_directories(fox3d PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/live
    ${OpenCV_INCLUDE_DIRS}
)

target_compile_definitions(fox3d PRIVATE
    FOX3D_CALIB_DIR="${CMAKE_CURRENT_SOURCE_DIR}/calib"
    FOX3D_VERSION="${PROJECT_VERSION}"
)

target_compile_options(fox3d PRIVATE
    -Wall -Wextra
    $<$<COMPILE_LANGUAGE:CXX>:-mno-direct-extern-access>
)

# Link the OpenCV shared libraries directly. The imported opencv_calib
# target also pulls opencv_viz, whose VTK libraries are not installed.
set(_fox_opencv
    opencv_highgui opencv_rgbd opencv_stereo opencv_calib
    opencv_imgcodecs opencv_imgproc opencv_geometry opencv_core
)
set(_fox_opencv_paths "")
foreach(_lib IN LISTS _fox_opencv)
    find_library(_fox_${_lib} NAMES ${_lib} REQUIRED)
    list(APPEND _fox_opencv_paths ${_fox_${_lib}})
endforeach()

target_link_libraries(fox3d PRIVATE
    Qt6::Widgets
    ${_fox_opencv_paths}
    CURL::libcurl
    OpenSSL::Crypto
)

set_target_properties(fox3d PROPERTIES
    OUTPUT_NAME fox3d
    BUILD_RPATH_USE_ORIGIN ON
)
