cmake_minimum_required(VERSION 3.5)

project(listener_baro)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(std_msgs REQUIRED)

function(custom_executable subfolder target)
  add_executable(${target}${target_suffix}
    src/${subfolder}/${target}.cpp)
  ament_target_dependencies(${target}${target_suffix}
    "rclcpp${target_suffix}"
    "std_msgs"
    "example_interfaces")
  install(TARGETS ${target}${target_suffix}
    DESTINATION bin)
endfunction()

macro(targets)
  if(NOT target_suffix STREQUAL "")
    get_rclcpp_information("${rmw_implementation}" "rclcpp${target_suffix}")
  endif()

  # Examples of Publish/Subscribe with Topics
  #custom_executable(. talker_baro)
  #custom_executable(. listener_baro)
endmacro()

call_for_each_rmw_implementation(targets GENERATE_DEFAULT)

#
# Library
#

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_interfaces_INCLUDE_DIRS}
)
add_library(ros__example src/listener_baro.cpp)
target_link_libraries(ros__example ${rclcpp_LIBRARIES})

add_library(listener_baro src/listener_baro.cpp)
target_link_libraries(listener_baro ${rclcpp_LIBRARIES})

ament_package()

px4_add_module(
    MODULE systemcmds__listener_baro
    MAIN listener_baro
    STACK_MAIN 1200
    COMPILE_FLAGS
        -Wno-write-strings
    SRCS
        src/listener_baro.cpp
    DEPENDS
        platforms__common
)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
