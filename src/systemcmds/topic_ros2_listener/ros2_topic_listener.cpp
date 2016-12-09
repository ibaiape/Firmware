#include "baro.cpp"

extern "C" __EXPORT int ros2_listener_main(int argc, char *argv[]);

int ros2_listener_main(int argc, char *argv[]) {

  (void)argv;

  if(argc < 2) {
		printf("need at least two arguments: topic name. [optional number of messages to print]\n");
		return 0;
	}

  unsigned num_msgs = (argc > 2) ? atoi(argv[2]) : 1;
  (void) num_msgs;
  auto node = rclcpp::Node::make_shared("ros2_listener_main");

  if (strncmp(argv[1],"sensor_baro", 50) == 0) {
    if (ros_2_baro::instance != nullptr) {
      warnx("already running");
    }else{
      ros_2_baro::instance = new ROS2Baro();
    }

    ros_2_baro::instance->set_num_msgs(num_msgs);

		if (OK != ros_2_baro::instance->start_listener()) {
			delete ros_2_baro::instance;
			ros_2_baro::instance = nullptr;
			warnx("start failed");
			return 1;
		}
  }

  return 0;
}
