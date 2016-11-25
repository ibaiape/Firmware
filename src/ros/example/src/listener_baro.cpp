#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

void chatterCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  std::cout << "I heard: [" << msg->data << "]" << std::endl;
}

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   auto node = rclcpp::Node::make_shared("listener_baro");
//   auto sub = node->create_subscription<std_msgs::msg::Float32>(
//     "sensor_baro", chatterCallback, rmw_qos_profile_default);
//
//   rclcpp::spin(node);
//
//   return 0;
// }

__EXPORT int listener_baro_main(int argc, char *argv[]);
int listener_baro_main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("listener_baro");
  auto sub = node->create_subscription<std_msgs::msg::Float32>(
    "sensor_baro", chatterCallback, rmw_qos_profile_default);

	printf("listener_baro main\n");
	return 0;
}
