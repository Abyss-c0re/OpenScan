cmake_minimum_required(VERSION 3.16)
project(lizard_host C)
set(CMAKE_C_STANDARD 11)
add_executable(lizard_selftest
  src/lizard_selftest.c
  src/lizard_pose.c
  src/lizard_color.c
  src/lizard_mold.c
  src/lizard_cube.c
  src/lizard_menu.c
  src/lizard_settings.c)
target_include_directories(lizard_selftest PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/../include)
target_link_libraries(lizard_selftest m)
