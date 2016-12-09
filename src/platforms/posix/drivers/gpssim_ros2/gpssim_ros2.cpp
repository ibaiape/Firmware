/****************************************************************************
 *
 *   Copyright (c) 2015 Roman Bapst. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file gpssim.cpp
 * Simulated GPS driver
 */

#include <px4_defines.h>
/*
#include <sys/types.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
*/
#include <px4_config.h>

#include <drivers/drv_hrt.h>
#include <systemlib/systemlib.h>

#include <drivers/drv_gps.h>

#include <simulator/simulator.h>

#include "DevMgr.hpp"
#include "VirtDevObj.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"

using namespace DriverFramework;

#define GPS_DRIVER_MODE_UBX_SIM
#define GPSSIM_DEVICE_PATH "/dev/gpssim_ros2"

#define TIMEOUT_5HZ 500
#define RATE_MEASUREMENT_PERIOD 5000000

/* class for dynamic allocation of satellite info data
class GPS_Sat_Info
{
public:
	struct satellite_info_s 	_data;
};
*/

class GPSSIM_ROS2 : public VirtDevObj
{
public:
	GPSSIM_ROS2(const char *uart_path, bool fake_gps, bool enable_sat_info);
	virtual ~GPSSIM_ROS2();

	virtual int			init();

	virtual int			devIOCTL(unsigned long cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void				print_info();

protected:
	virtual void			_measure() {}

private:

	bool				_task_should_exit;				///< flag to make the main worker task exit
	int				_serial_fd;					///< serial interface to GPS
	unsigned			_baudrate;					///< current baudrate
	char				_port[20];					///< device / serial port path
	volatile int			_task;						///< worker task
	bool				_healthy;					///< flag to signal if the GPS is ok
	bool				_baudrate_changed;				///< flag to signal that the baudrate with the GPS has changed
	bool				_mode_changed;					///< flag that the GPS mode has changed
	//gps_driver_mode_t		_mode;						///< current mode
	// GPS_Sat_Info			*_Sat_Info;					///< instance of GPS sat info data object
	// struct vehicle_gps_position_s	_report_gps_pos;				///< uORB topic for gps position
	//orb_advert_t			_report_gps_pos_pub;				///< uORB pub for gps position
	//struct satellite_info_s		*_p_report_sat_info;				///< pointer to uORB topic for satellite info
	//orb_advert_t			_report_sat_info_pub;				///< uORB pub for satellite info
	float				_rate;						///< position update rate
	bool				_fake_gps;					///< fake gps output
	SyncObj				_sync;

	/**
	 * Try to configure the GPS, handle outgoing communication to the GPS
	 */
	void			 	config();

	/**
	 * Trampoline to the worker task
	 */
	static void			task_main_trampoline(void *arg);

	/**
	 * Worker task: main GPS thread that configures the GPS and parses incoming data, always running
	 */
	void				task_main(void);

	/**
	 * Set the baudrate of the UART to the GPS
	 */
	int				set_baudrate(unsigned baud);

	/**
	 * Send a reset command to the GPS
	 */
	void				cmd_reset();

	int 				receive(int timeout);

	rclcpp::publisher::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_gps;
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int gpssim_ros2_main(int argc, char *argv[]);

static GPSSIM_ROS2	*g_dev = nullptr;

GPSSIM_ROS2::GPSSIM_ROS2(const char *uart_path, bool fake_gps, bool enable_sat_info) :
	VirtDevObj("gpsros2", GPSSIM_DEVICE_PATH, nullptr, 1e6 / 10),
	_task_should_exit(false),
	// _Sat_Info(nullptr),
	// _report_gps_pos_pub(nullptr),
	// _p_report_sat_info(nullptr),
	// _report_sat_info_pub(nullptr),
	_rate(0.0f),
	_fake_gps(fake_gps)
{
	(void) uart_path;

	// memset(&_report_gps_pos, 0, sizeof(_report_gps_pos));

	/* create satellite info data object if requested */
	if (enable_sat_info) {
		// _Sat_Info = new(GPS_Sat_Info);
		// _p_report_sat_info = &_Sat_Info->_data;
		// memset(_p_report_sat_info, 0, sizeof(*_p_report_sat_info));
	}
}

GPSSIM_ROS2::~GPSSIM_ROS2()
{
	/* tell the task we want it to go away */
	_task_should_exit = true;

	/* spin waiting for the task to stop */
	for (unsigned i = 0; (i < 10) && (_task != -1); i++) {
		/* give it another 100ms */
		usleep(100000);
	}

	/* well, kill it anyway, though this will probably crash */
	if (_task != -1) {
		px4_task_delete(_task);
	}

	g_dev = nullptr;
}

int
GPSSIM_ROS2::init()
{

	int ret = PX4_ERROR;

	/* do regular cdev init */
	if (VirtDevObj::init() != OK) {
		return ret;
	}

	auto node = rclcpp::node::Node::make_shared("talker_gps");

	/* start the GPS driver worker task */
	_task = px4_task_spawn_cmd("gps", SCHED_DEFAULT,
				   SCHED_PRIORITY_DEFAULT, 1500, (px4_main_t)&GPSSIM_ROS2::task_main_trampoline, nullptr);

	if (_task < 0) {
		PX4_ERR("task start failed: %d", errno);
		return -errno;
	}

	rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
	custom_qos_profile.depth = 7;
	pub_gps = node->create_publisher<sensor_msgs::msg::NavSatFix>("sensor_gps", custom_qos_profile);

	ret = OK;
	return ret;
}

int
GPSSIM_ROS2::devIOCTL(unsigned long cmd, unsigned long arg)
{
	_sync.lock();

	int ret = OK;

	switch (cmd) {
	case SENSORIOCRESET:
		cmd_reset();
		break;

	default:
		/* give it to parent if no one wants it */
		ret = VirtDevObj::devIOCTL(cmd, arg);
		break;
	}

	_sync.unlock();

	return ret;
}

void
GPSSIM_ROS2::task_main_trampoline(void *arg)
{
	(void) arg;
	g_dev->task_main();
}

int
GPSSIM_ROS2::receive(int timeout)
{
	(void)timeout;
	Simulator *sim = Simulator::getInstance();
	simulator::RawGPSData gps;
	sim->getGPSSample((uint8_t *)&gps, sizeof(gps));
	PX4_ERR("GPSSIM_ROS2 receive");

	auto msg = std::make_shared<sensor_msgs::msg::NavSatFix>();
	msg->latitude = gps.lat;
	msg->longitude = gps.lon;
	msg->altitude = gps.alt;
	msg->status.status = gps.fix_type;
	pub_gps->publish(msg);

/*
	_report_gps_pos.timestamp = hrt_absolute_time();
	_report_gps_pos.lat = gps.lat;
	_report_gps_pos.lon = gps.lon;
	_report_gps_pos.alt = gps.alt;
	_report_gps_pos.eph = (float)gps.eph * 1e-2f;
	_report_gps_pos.epv = (float)gps.epv * 1e-2f;
	_report_gps_pos.vel_m_s = (float)(gps.vel) / 100.0f;
	_report_gps_pos.vel_n_m_s = (float)(gps.vn) / 100.0f;
	_report_gps_pos.vel_e_m_s = (float)(gps.ve) / 100.0f;
	_report_gps_pos.vel_d_m_s = (float)(gps.vd) / 100.0f;
	_report_gps_pos.cog_rad = (float)(gps.cog) * 3.1415f / (100.0f * 180.0f);
	_report_gps_pos.fix_type = gps.fix_type;
	_report_gps_pos.satellites_used = gps.satellites_visible;
*/
	usleep(200000);
	return 1;
}

void
GPSSIM_ROS2::task_main()
{

	/* loop handling received serial bytes and also configuring in between */
	while (!_task_should_exit) {

		if (_fake_gps) {
			/*
			_report_gps_pos.timestamp = hrt_absolute_time();
			_report_gps_pos.lat = (int32_t)47.378301e7f;
			_report_gps_pos.lon = (int32_t)8.538777e7f;
			_report_gps_pos.alt = (int32_t)1200e3f;
			_report_gps_pos.s_variance_m_s = 10.0f;
			_report_gps_pos.c_variance_rad = 0.1f;
			_report_gps_pos.fix_type = 3;
			_report_gps_pos.eph = 0.9f;
			_report_gps_pos.epv = 1.8f;
			_report_gps_pos.vel_n_m_s = 0.0f;
			_report_gps_pos.vel_e_m_s = 0.0f;
			_report_gps_pos.vel_d_m_s = 0.0f;
			_report_gps_pos.vel_m_s = sqrtf(_report_gps_pos.vel_n_m_s * _report_gps_pos.vel_n_m_s + _report_gps_pos.vel_e_m_s *
							_report_gps_pos.vel_e_m_s + _report_gps_pos.vel_d_m_s * _report_gps_pos.vel_d_m_s);
			_report_gps_pos.cog_rad = 0.0f;
			_report_gps_pos.vel_ned_valid = true;
			*/

			//no time and satellite information simulated
/*
			if (!(m_pub_blocked)) {
				if (_report_gps_pos_pub != nullptr) {
					orb_publish(ORB_ID(vehicle_gps_position), _report_gps_pos_pub, &_report_gps_pos);

				} else {
					_report_gps_pos_pub = orb_advertise(ORB_ID(vehicle_gps_position), &_report_gps_pos);
				}
			}
*/
			usleep(2e5);

		} else {
			/*
			//Publish initial report that we have access to a GPS
			//Make sure to clear any stale data in case driver is reset
			memset(&_report_gps_pos, 0, sizeof(_report_gps_pos));
			_report_gps_pos.timestamp = hrt_absolute_time();
			_report_gps_pos.timestamp_time_relative = 0;

			if (!(m_pub_blocked)) {
				if (_report_gps_pos_pub != nullptr) {
					orb_publish(ORB_ID(vehicle_gps_position), _report_gps_pos_pub, &_report_gps_pos);

				} else {
					_report_gps_pos_pub = orb_advertise(ORB_ID(vehicle_gps_position), &_report_gps_pos);
				}
			}
*/
			// GPS is obviously detected successfully, reset statistics
			//_Helper->reset_update_rates();

			while ((receive(TIMEOUT_5HZ)) > 0 && !_task_should_exit) {
				/* opportunistic publishing - else invalid data would end up on the bus */
/*
				if (!(m_pub_blocked)) {
					orb_publish(ORB_ID(vehicle_gps_position), _report_gps_pos_pub, &_report_gps_pos);

					if (_p_report_sat_info) {
						if (_report_sat_info_pub != nullptr) {
							orb_publish(ORB_ID(satellite_info), _report_sat_info_pub, _p_report_sat_info);

						} else {
							_report_sat_info_pub = orb_advertise(ORB_ID(satellite_info), _p_report_sat_info);
						}
					}
				}
				*/
			}

			// FIXME - if ioctl is called then it will deadlock
			_sync.lock();
		}
	}

	PX4_INFO("exiting");

	/* tell the dtor that we are exiting */
	_task = -1;
	return;
}



void
GPSSIM_ROS2::cmd_reset()
{
}

void
GPSSIM_ROS2::print_info()
{
	//GPS Mode
	if (_fake_gps) {
		PX4_INFO("protocol: faked");
	}else {
		PX4_INFO("protocol: SIM");
	}

	PX4_INFO("port: %s, baudrate: %d, status: %s", _port, _baudrate, (_healthy) ? "OK" : "NOT OK");
	// PX4_INFO("sat info: %s, noise: %d, jamming detected: %s",
	// 	 (_p_report_sat_info != nullptr) ? "enabled" : "disabled",
	// 	 _report_gps_pos.noise_per_ms,
	// 	 _report_gps_pos.jamming_indicator == 255 ? "YES" : "NO");
/*
	if (_report_gps_pos.timestamp != 0) {
		PX4_INFO("position lock: %dD, satellites: %d, last update: %8.4fms ago", (int)_report_gps_pos.fix_type,
			 _report_gps_pos.satellites_used, (double)(hrt_absolute_time() - _report_gps_pos.timestamp) / 1000.0);
		PX4_INFO("lat: %d, lon: %d, alt: %d", _report_gps_pos.lat, _report_gps_pos.lon, _report_gps_pos.alt);
		PX4_INFO("vel: %.2fm/s, %.2fm/s, %.2fm/s", (double)_report_gps_pos.vel_n_m_s,
			 (double)_report_gps_pos.vel_e_m_s, (double)_report_gps_pos.vel_d_m_s);
		PX4_INFO("eph: %.2fm, epv: %.2fm", (double)_report_gps_pos.eph, (double)_report_gps_pos.epv);
		//PX4_INFO("rate position: \t%6.2f Hz", (double)_Helper->get_position_update_rate());
		//PX4_INFO("rate velocity: \t%6.2f Hz", (double)_Helper->get_velocity_update_rate());
		PX4_INFO("rate publication:\t%6.2f Hz", (double)_rate);
	}
	*/
	usleep(100000);
}

/**
 * Local functions in support of the shell command.
 */
namespace gpssim
{

/*
void	start(const char *uart_path, bool fake_gps, bool enable_sat_info);
void	stop();
void	reset();
void	info();
void	usage(const char *reason);
*/
/**
 * Start the driver.
 */
static void
start(const char *uart_path, bool fake_gps, bool enable_sat_info)
{
	DevHandle h;

	/* create the driver */
	g_dev = new GPSSIM_ROS2(uart_path, fake_gps, enable_sat_info);

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	DevMgr::getHandle(GPSSIM_DEVICE_PATH, h);

	if (!h.isValid()) {
		PX4_ERR("getHandle failed: %s", GPSSIM_DEVICE_PATH);
		goto fail;
	}

	return;

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	PX4_ERR("start failed");
}

/**
 * Print the status of the driver.
 */
static void
info()
{
	if (g_dev == nullptr) {
		PX4_ERR("gpssim not running");
		return;
	}

	g_dev->print_info();
}

static void
usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PX4_INFO("usage:");
	PX4_INFO("gpssim {start|stop|test|reset|status}");
	PX4_INFO("       [-d /dev/ttyS0-n][-f (for enabling fake)][-s (to enable sat info)]");
}

} // namespace

int
gpssim_ros2_main(int argc, char *argv[])
{

	/* set to default */
	const char *device_name = GPS_DEFAULT_UART_PORT;
	bool fake_gps = false;
	bool enable_sat_info = false;


	if (argc < 2) {

		gpssim::usage("not enough arguments supplied");
		return 1;
	}

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		if (g_dev != nullptr) {
			PX4_WARN("already started");
			return 0;
		}

		if (argc > 3) {
			if (!strcmp(argv[2], "-d")) {
				device_name = argv[3];
			}
		}

		/* Detect fake gps option */
		for (int i = 2; i < argc; i++) {
			if (!strcmp(argv[i], "-f")) {
				fake_gps = true;
			}
		}

		/* Detect sat info option */
		for (int i = 2; i < argc; i++) {
			if (!strcmp(argv[i], "-s")) {
				enable_sat_info = true;
			}
		}

		gpssim::start(device_name, fake_gps, enable_sat_info);
		return 0;
	}

	/* The following need gpssim running. */
	if (g_dev == nullptr) {
		PX4_WARN("not running");
		return 1;
	}

	/*
	 * Print driver status.
	 */
	if (!strcmp(argv[1], "status")) {
		gpssim::info();
	}

	return 0;
}
