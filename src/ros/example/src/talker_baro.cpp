#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

extern "C" __EXPORT int talker_baro_main(int argc, char *argv[]);

int talker_baro_main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::node::Node::make_shared("talker_baro");

  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 7;
  auto chatter_pub = node->create_publisher<std_msgs::msg::Float32>("sensor_baro", custom_qos_profile);

  rclcpp::WallRate loop_rate(2);
  auto msg = std::make_shared<std_msgs::msg::Float32>();
  auto i = 1;

  while (rclcpp::ok()) {
    msg->data = i++;
    std::cout << "Publishing: '" << msg->data << "'" << std::endl;
    chatter_pub->publish(msg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  return 0;
}
