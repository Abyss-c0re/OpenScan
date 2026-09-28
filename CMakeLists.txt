cmake_minimum_required(VERSION 3.16)
project(openscan VERSION 0.6.14 LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(WIN32)
    set(OPENSCAN_HOST "windows")
elseif(APPLE)
    set(OPENSCAN_HOST "mac")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(OPENSCAN_HOST "linux")
else()
    set(OPENSCAN_HOST "${CMAKE_SYSTEM_NAME}")
endif()

set(OPENSCAN_SRC
    src/main.c
    src/ui/os_ui.c
    src/host/os_host.c
    src/image/os_jpeg.c
    src/live/os_logic.c
    src/live/os_live.c
    src/device/openscan_cam_name.c
    src/calib/openscan_calib.c
    src/calib/openscan_calib_fetch.c
)

if(WIN32)
    list(APPEND OPENSCAN_SRC src/ui/os_surf_win32.c src/device/openscan_cam_win.c)
elseif(APPLE)
    enable_language(OBJC)
    list(APPEND OPENSCAN_SRC src/ui/os_surf_mac.m src/device/openscan_cam_mac.m)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(X11 REQUIRED x11)
    find_package(JPEG REQUIRED)
    list(APPEND OPENSCAN_SRC src/ui/os_surf_x11.c src/device/openscan_v4l2.c)
else()
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(X11 REQUIRED x11)
    find_package(JPEG REQUIRED)
    list(APPEND OPENSCAN_SRC src/ui/os_surf_x11.c src/device/openscan_cam_none.c)
endif()

add_executable(openscan ${OPENSCAN_SRC})

target_include_directories(openscan PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
if(X11_INCLUDE_DIRS)
    target_include_directories(openscan PRIVATE ${X11_INCLUDE_DIRS})
endif()

target_compile_definitions(openscan PRIVATE
    OPENSCAN_CALIB_DIR="${CMAKE_CURRENT_SOURCE_DIR}/calib"
    OPENSCAN_VERSION="${PROJECT_VERSION}"
    OPENSCAN_HOST="${OPENSCAN_HOST}"
)

if(MSVC)
    target_compile_options(openscan PRIVATE /W3)
else()
    target_compile_options(openscan PRIVATE -Wall -Wextra)
endif()

if(WIN32)
    target_link_libraries(openscan PRIVATE
        user32 gdi32 ole32 uuid mfplat mfreadwrite mfuuid windowscodecs)
elseif(APPLE)
    target_link_libraries(openscan PRIVATE
        "-framework Cocoa" "-framework AVFoundation" "-framework CoreMedia"
        "-framework CoreVideo" "-framework CoreGraphics" "-framework ImageIO")
else()
    target_link_libraries(openscan PRIVATE ${X11_LIBRARIES} JPEG::JPEG)
endif()

if(CMAKE_C_COMPILER_ID STREQUAL "GNU" OR CMAKE_C_COMPILER_ID STREQUAL "Clang")
    if(NOT APPLE AND NOT MSVC)
        target_link_options(openscan PRIVATE -Wl,--as-needed)
    endif()
endif()

set_target_properties(openscan PROPERTIES OUTPUT_NAME openscan)
