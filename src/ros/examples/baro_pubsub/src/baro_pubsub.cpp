#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include <px4_posix.h>

extern "C" __EXPORT int baro_pubsub_main(int argc, char *argv[]);

//std::shared_ptr<ROS2Baro> ros2baro_node_ptr;

class ROS2Baro;

namespace ros_2_baro
{
ROS2Baro *instance;
}

class ROS2Baro
{
public:
	/**
	 * Constructor
	 */
	 ROS2Baro();

	 /**
 	 * Destructor, also kills task.
 	 */
	 ~ROS2Baro();

	 int start_talker();
	 static void	task_main_talker(int argc, char *argv[]);

	 int start_listener();
	 static void	task_main_listener(int argc, char *argv[]);

	 int		_control_task = -1;			/**< task handle for task */
	 int		_control_task_listener = -1;			/**< task handle for task */

	 void chatterCallback(const std_msgs::msg::Float32::SharedPtr msg);

};


ROS2Baro::ROS2Baro()
{
	//ros2baro_node_ptr.reset(new ROS2Baro());
}

ROS2Baro::~ROS2Baro()
{

}


int ROS2Baro::start_talker()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("talker_baro",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   2000,
					   (px4_main_t)&ROS2Baro::task_main_talker,
					   nullptr);

	if (_control_task < 0) {
		warn("task talker start failed");
		return -errno;
	}

	return OK;
}

int ROS2Baro::start_listener()
{
	ASSERT(_control_task_listener == -1);

	/* start the task */
	_control_task_listener = px4_task_spawn_cmd("listener_baro",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   2000,
					   (px4_main_t)&ROS2Baro::task_main_listener,
					   nullptr);

	if (_control_task_listener < 0) {
		warn("task listener start failed");
		return -errno;
	}

	return OK;
}

void ROS2Baro::task_main_listener(int argc, char *argv[])
{
	(void)argc;
	(void)argv;

	auto node = rclcpp::Node::make_shared("listener_baro");

  auto cb_holder = std::make_shared<const ROS2Baro>();

	auto cb_float_function
			= std::bind(
        &ROS2Baro::chatterCallback,
				 *cb_holder,
				 std::placeholders::_1);

	auto sub = node->create_subscription<std_msgs::msg::Float32>(
		"sensor_baro", cb_float_function, rmw_qos_profile_default);

	rclcpp::spin(node);
}

void ROS2Baro::task_main_talker(int argc, char *argv[])
{
	(void)argc;
	(void)argv;

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
}

void ROS2Baro::chatterCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  std::cout << "I heard: [" << msg->data << "]" << std::endl;
}

int baro_pubsub_main(int argc, char *argv[])
{

  if (argc < 2) {
		printf("usage: baro_pubsub {talker|listener}");
		return 1;
	}

  if (!strcmp(argv[1], "talker")) {

		if (ros_2_baro::instance != nullptr) {
			warnx("already running");
		}else{
			ros_2_baro::instance = new ROS2Baro;
	  	rclcpp::init(argc, argv);
		}


		if (OK != ros_2_baro::instance->start_talker()) {
			delete ros_2_baro::instance;
			ros_2_baro::instance = nullptr;
			warnx("start failed");
			return 1;
		}
  }

  if (!strcmp(argv[1], "listener")) {

		if (ros_2_baro::instance != nullptr) {
			warnx("already running");
		}else{
			ros_2_baro::instance = new ROS2Baro;
	  	rclcpp::init(argc, argv);
		}

		if (OK != ros_2_baro::instance->start_listener()) {
			delete ros_2_baro::instance;
			ros_2_baro::instance = nullptr;
			warnx("start failed");
			return 1;
		}

  }
  return 0;
}
