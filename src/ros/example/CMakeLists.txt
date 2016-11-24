# cmake_minimum_required(VERSION 3.5)
#
# project(listener_baro)
#
# if(NOT WIN32)
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
# endif()
#
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(std_msgs REQUIRED)
#
# function(custom_executable subfolder target)
#   add_executable(${target}${target_suffix}
#     src/${subfolder}/${target}.cpp)
#   ament_target_dependencies(${target}${target_suffix}
#     "rclcpp${target_suffix}"
#     "std_msgs"
#     "example_interfaces")
#   install(TARGETS ${target}${target_suffix}
#     DESTINATION bin)
# endfunction()
#
# macro(targets)
#   if(NOT target_suffix STREQUAL "")
#     get_rclcpp_information("${rmw_implementation}" "rclcpp${target_suffix}")
#   endif()
#
#   # Examples of Publish/Subscribe with Topics
#   #custom_executable(. talker_baro)
#   #custom_executable(. listener_baro)
# endmacro()
#
# call_for_each_rmw_implementation(targets GENERATE_DEFAULT)
#
# #
# # Library
# #
#
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_interfaces_INCLUDE_DIRS}
)
# add_library(ros__example src/listener_baro.cpp)
# target_link_libraries(ros__example ${rclcpp_LIBRARIES})
#
# add_library(listener_baro src/listener_baro.cpp)
# target_link_libraries(listener_baro ${rclcpp_LIBRARIES})
#
# ament_package()

##################
##################

# px4_add_module(
#     MODULE ros__example
#     MAIN listener_baro
#     STACK_MAIN 1200
#     COMPILE_FLAGS
#         -Wno-write-strings
#     SRCS
#         src/listener_baro.cpp
#     DEPENDS
#         platforms__common
# )

##################
##################

# Define a new function meant to support ROS 2.0
#   ament and linking mechanisms

function(px4_parse_function_args)
	cmake_parse_arguments(IN "" "NAME" "OPTIONS;ONE_VALUE;MULTI_VALUE;REQUIRED;ARGN" "${ARGN}")
	cmake_parse_arguments(OUT "${IN_OPTIONS}" "${IN_ONE_VALUE}" "${IN_MULTI_VALUE}" "${IN_ARGN}")
	if (OUT_UNPARSED_ARGUMENTS)
		message(FATAL_ERROR "${IN_NAME}: unparsed ${OUT_UNPARSED_ARGUMENTS}")
	endif()
	foreach(arg ${IN_REQUIRED})
		if (NOT OUT_${arg})
			message(FATAL_ERROR "${IN_NAME} requires argument ${arg}\nARGN: ${IN_ARGN}")
		endif()
	endforeach()
	foreach(arg ${IN_OPTIONS} ${IN_ONE_VALUE} ${IN_MULTI_VALUE})
		set(${arg} ${OUT_${arg}} PARENT_SCOPE)
	endforeach()
endfunction()

function(px4_add_ros_module)

	px4_parse_function_args(
		NAME px4_add_module
		ONE_VALUE MODULE MAIN STACK STACK_MAIN STACK_MAX PRIORITY
		MULTI_VALUE COMPILE_FLAGS LINK_FLAGS SRCS INCLUDES DEPENDS
		REQUIRED MODULE
		ARGN ${ARGN})

	add_library(${MODULE} STATIC EXCLUDE_FROM_ALL ${SRCS})

	# set defaults if not set
	set(MAIN_DEFAULT MAIN-NOTFOUND)
	set(STACK_MAIN_DEFAULT 1024)
	set(PRIORITY_DEFAULT SCHED_PRIORITY_DEFAULT)

	# default stack max to stack main
	if(NOT STACK_MAIN AND STACK)
		set(STACK_MAIN ${STACK})
		message(AUTHOR_WARNING "STACK deprecated, USE STACK_MAIN instead!!!!!!!!!!!!")
	endif()

	foreach(property MAIN STACK_MAIN PRIORITY)
		if(NOT ${property})
			set(${property} ${${property}_DEFAULT})
		endif()
		set_target_properties(${MODULE} PROPERTIES ${property}
			${${property}})
	endforeach()

	# default stack max to stack main
	if(NOT STACK_MAX)
		set(STACK_MAX ${STACK_MAIN})
	endif()
	set_target_properties(${MODULE} PROPERTIES STACK_MAX
		${STACK_MAX})

	if(${OS} STREQUAL "qurt" )
		set_property(TARGET ${MODULE} PROPERTY POSITION_INDEPENDENT_CODE TRUE)
	elseif(${OS} STREQUAL "nuttx" )
		list(APPEND COMPILE_FLAGS -Wframe-larger-than=${STACK_MAX})
	endif()

	if(MAIN)
		set_target_properties(${MODULE} PROPERTIES
			COMPILE_DEFINITIONS PX4_MAIN=${MAIN}_app_main)
		add_definitions(-DMODULE_NAME="${MAIN}")
	else()
		add_definitions(-DMODULE_NAME="${MODULE}")
	endif()

	if(INCLUDES)
		target_include_directories(${MODULE} ${INCLUDES})
	endif()

	if(DEPENDS)
		add_dependencies(${MODULE} ${DEPENDS})
	endif()

	# join list variables to get ready to send to compiler
	foreach(prop LINK_FLAGS COMPILE_FLAGS)
		if(${prop})
			px4_join(OUT ${prop} LIST ${${prop}} GLUE " ")
		endif()
	endforeach()

	# store module properties in target
	# COMPILE_FLAGS and LINK_FLAGS are passed to compiler/linker by cmake
	# STACK_MAIN, MAIN, PRIORITY are PX4 specific
	foreach (prop COMPILE_FLAGS LINK_FLAGS STACK_MAIN MAIN PRIORITY)
		if (${prop})
			set_target_properties(${MODULE} PROPERTIES ${prop} ${${prop}})
		endif()
	endforeach()

endfunction()

px4_add_ros_module(
    MODULE ros__example
    MAIN listener_baro
    STACK_MAIN 1200
    COMPILE_FLAGS
        -Wno-write-strings
    SRCS
        src/listener_baro.cpp
    # DEPENDS
    #     platforms__common
)


# vim: set noet ft=cmake fenc=utf-8 ff=unix :
