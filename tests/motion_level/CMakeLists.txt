cmake_minimum_required(VERSION 3.16)
project(motion_level_test LANGUAGES C CXX)

set(LV_BUILD_CONF_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lv_conf.h" CACHE PATH "" FORCE)
set(CONFIG_LV_BUILD_DEMOS OFF CACHE BOOL "" FORCE)
set(CONFIG_LV_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(CONFIG_LV_USE_THORVG_INTERNAL OFF CACHE BOOL "" FORCE)
add_subdirectory(../../managed_components/lvgl__lvgl lvgl)

add_executable(test_motion_level test_motion_level.c ../../main/motion_level.c)
target_include_directories(test_motion_level PRIVATE ../../main)
target_compile_options(test_motion_level PRIVATE -Wall -Wextra -Werror -UNDEBUG)
target_link_libraries(test_motion_level PRIVATE lvgl::lvgl m)

enable_testing()
add_test(NAME motion_level COMMAND test_motion_level)
