
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

px4_add_module(
	MODULE ros__examples__baro_pubsub
	MAIN baro_pubsub
  STACK_MAIN 5700
	STACK_MAX 13000
	COMPILE_FLAGS
		-std=c++11 -Wall -Wextra
	SRCS
		src/baro_pubsub.cpp
	DEPENDS
		platforms__common
)
ament_target_dependencies(ros__examples__baro_pubsub "rclcpp" "std_msgs")

# vim: set noet ft=cmake fenc=utf-8 ff=unix :
