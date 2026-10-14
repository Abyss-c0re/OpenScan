cmake_minimum_required(VERSION 3.18)
project(lizard_oxr C)
set(CMAKE_C_STANDARD 11)

set(OPENSCAN "${CMAKE_CURRENT_SOURCE_DIR}/../..")
set(LIZARD_OPENXR_LOADER "" CACHE FILEPATH "Android libopenxr_loader.so")

add_library(android_native_app_glue STATIC
  ${ANDROID_NDK}/sources/android/native_app_glue/android_native_app_glue.c)
target_include_directories(android_native_app_glue PUBLIC
  ${ANDROID_NDK}/sources/android/native_app_glue)

add_library(lizard_oxr SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/lizard_oxr.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/lizard_cam.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/lizard_uvc.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/lizard_pose.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/lizard_color.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/lizard_mold.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/lizard_cube.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/lizard_menu.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/lizard_settings.c
  ${OPENSCAN}/src/live/os_stripe.c
  ${OPENSCAN}/src/live/os_logic.c
  ${OPENSCAN}/src/calib/openscan_calib.c
  ${OPENSCAN}/src/image/os_pattern.c)

# System OpenXR 1.1.60, isolated so include_next does not pull host libc.
set(OXR_WRAP "${CMAKE_CURRENT_BINARY_DIR}/oxr-inc")
file(MAKE_DIRECTORY "${OXR_WRAP}")
if(NOT EXISTS "${OXR_WRAP}/openxr/openxr.h")
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
    /usr/include/openxr "${OXR_WRAP}/openxr")
endif()
target_include_directories(lizard_oxr PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../src
  ${OPENSCAN}/include
  ${OPENSCAN}/src/live
  ${OXR_WRAP})
target_compile_definitions(lizard_oxr PRIVATE OPENSCAN_HOST="android")
target_link_options(lizard_oxr PRIVATE
  "-Wl,-z,max-page-size=16384"
  "-Wl,-z,common-page-size=16384"
  "-u" "ANativeActivity_onCreate")
target_link_libraries(lizard_oxr PRIVATE
  android_native_app_glue
  android
  log
  EGL
  GLESv3
  m)

if(LIZARD_OPENXR_LOADER)
  add_library(openxr_loader SHARED IMPORTED)
  set_target_properties(openxr_loader PROPERTIES IMPORTED_LOCATION "${LIZARD_OPENXR_LOADER}")
  target_link_libraries(lizard_oxr PRIVATE openxr_loader)
endif()
