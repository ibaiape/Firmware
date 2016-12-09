#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include <px4_posix.h>

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

	 void set_num_msgs(int num_msgs);

	 /**
 	 * Destructor, also kills task.
 	 */
	 ~ROS2Baro();

	 int start_listener();
	 static void	task_main_listener(int argc, char *argv[]);

	 int		_control_task_listener = -1;			/**< task handle for task */
	 int 		_num_msgs;
	 int 		_cont_msgs;
	 int 		_stop;

	 void barometerCallback(const std_msgs::msg::Float32::SharedPtr msg);

	 void stop();
	 rclcpp::node::Node::SharedPtr node;
	 rclcpp::executors::SingleThreadedExecutor executor;
};


ROS2Baro::ROS2Baro(){}

void ROS2Baro::set_num_msgs(int num_msgs)
{
	this->_num_msgs = num_msgs;
	this->_cont_msgs = 0;
}

ROS2Baro::~ROS2Baro(){}

int ROS2Baro::start_listener()
{
	this->_stop = false;

	if(_control_task_listener != -1){
		return OK;
	}

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

	if(ros_2_baro::instance->node == nullptr)
		ros_2_baro::instance->node = rclcpp::Node::make_shared("listener_baro");

	auto cb_float_function
			= std::bind(
        &ROS2Baro::barometerCallback,
				 ros_2_baro::instance,
				 std::placeholders::_1);

	auto sub = ros_2_baro::instance->node->create_subscription<std_msgs::msg::Float32>(
		"sensor_baro", cb_float_function, rmw_qos_profile_sensor_data);

	ros_2_baro::instance->executor.add_node(ros_2_baro::instance->node);
	ros_2_baro::instance->executor.spin_some();
  while (rclcpp::utilities::ok() && !ros_2_baro::instance->_stop) {
    ros_2_baro::instance->executor.spin_some();
  }
	ros_2_baro::instance->stop();
}

void ROS2Baro::stop()
{
	if (_control_task_listener != -1) {

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);
			// warn(" loop stop() _control_task_listener %d", _control_task_listener);

			/* if we have given up, kill it */
			if (++i > 50) {
				int pid_id = _control_task_listener;
				_control_task_listener = -1;
				px4_task_delete(pid_id);
				break;
			}
		} while (_control_task_listener != -1);
	}
}

void ROS2Baro::barometerCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  std::cout << "sensor_baro: [" << msg->data << "] " << std::endl;

	this->_cont_msgs++;

	if(this->_cont_msgs >= this->_num_msgs){
		ros_2_baro::instance->executor.remove_node(ros_2_baro::instance->node);
		ros_2_baro::instance->_stop = true;
	}
}
