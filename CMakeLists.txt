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

# The engine. Link this from another program and include openscan/openscan.h.
set(OPENSCAN_LIB_SRC
    src/live/os_logic.c
    src/live/os_live.c
    src/calib/openscan_calib.c
    src/calib/openscan_calib_fetch.c
    src/device/openscan_cam_name.c
    src/image/os_jpeg.c
    src/image/os_frame.c
)
set(OPENSCAN_APP_SRC
    src/main.c
    src/ui/os_ui.c
    src/host/os_host.c
)

if(WIN32)
    list(APPEND OPENSCAN_LIB_SRC src/device/openscan_cam_win.c)
    list(APPEND OPENSCAN_APP_SRC src/ui/os_surf_win32.c)
elseif(APPLE)
    enable_language(OBJC)
    list(APPEND OPENSCAN_LIB_SRC src/device/openscan_cam_mac.m)
    list(APPEND OPENSCAN_APP_SRC src/ui/os_surf_mac.m)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(X11 REQUIRED x11)
    find_package(JPEG REQUIRED)
    list(APPEND OPENSCAN_LIB_SRC src/device/openscan_v4l2.c)
    list(APPEND OPENSCAN_APP_SRC src/ui/os_surf_x11.c)
else()
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(X11 REQUIRED x11)
    find_package(JPEG REQUIRED)
    list(APPEND OPENSCAN_LIB_SRC src/device/openscan_cam_none.c)
    list(APPEND OPENSCAN_APP_SRC src/ui/os_surf_x11.c)
endif()

add_library(openscan_core STATIC ${OPENSCAN_LIB_SRC})
add_executable(openscan ${OPENSCAN_APP_SRC})

target_include_directories(openscan_core
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_include_directories(openscan PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
if(X11_INCLUDE_DIRS)
    target_include_directories(openscan PRIVATE ${X11_INCLUDE_DIRS})
endif()

target_compile_definitions(openscan_core
    PUBLIC OPENSCAN_VERSION="${PROJECT_VERSION}" OPENSCAN_HOST="${OPENSCAN_HOST}"
    PRIVATE OPENSCAN_CALIB_DIR="${CMAKE_CURRENT_SOURCE_DIR}/calib"
)

if(MSVC)
    target_compile_options(openscan_core PRIVATE /W3)
    target_compile_options(openscan PRIVATE /W3)
else()
    target_compile_options(openscan_core PRIVATE -Wall -Wextra)
    target_compile_options(openscan PRIVATE -Wall -Wextra)
endif()

if(WIN32)
    target_link_libraries(openscan_core PUBLIC ole32 uuid mfplat mf mfreadwrite mfuuid windowscodecs)
    target_link_libraries(openscan PRIVATE openscan_core user32 gdi32 comdlg32)
elseif(APPLE)
    set_source_files_properties(src/device/openscan_cam_mac.m src/ui/os_surf_mac.m
        PROPERTIES COMPILE_FLAGS "-fno-objc-arc")
    target_link_libraries(openscan_core PUBLIC
        "-framework AVFoundation" "-framework CoreMedia" "-framework CoreVideo"
        "-framework CoreGraphics" "-framework ImageIO" "-framework Foundation")
    target_link_libraries(openscan PRIVATE openscan_core "-framework Cocoa")
else()
    target_link_libraries(openscan_core PUBLIC JPEG::JPEG)
    target_link_libraries(openscan PRIVATE openscan_core ${X11_LIBRARIES})
endif()

if(CMAKE_C_COMPILER_ID STREQUAL "GNU" OR CMAKE_C_COMPILER_ID STREQUAL "Clang")
    if(NOT APPLE AND NOT MSVC)
        target_link_options(openscan PRIVATE -Wl,--as-needed)
        target_link_options(openscan_core PRIVATE -Wl,--as-needed)
    endif()
endif()

set_target_properties(openscan_core PROPERTIES OUTPUT_NAME openscan)
set_target_properties(openscan PROPERTIES OUTPUT_NAME openscan)
