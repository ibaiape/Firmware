#include <px4_defines.h>

#include <drivers/drv_baro.h>
#include <drivers/device/ringbuffer.h>

#include <systemlib/perf_counter.h>

#include <simulator/simulator.h>

#include "VirtDevObj.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

/* helper macro for handling report buffer indices */
#define INCREMENT(_x, _lim)	do { __typeof__(_x) _tmp = _x+1; if (_tmp >= _lim) _tmp = 0; _x = _tmp; } while(0)

/* helper macro for arithmetic - returns the square of the argument */
#define POW2(_x)		((_x) * (_x))

#define BAROSIMROS2_DEV_PATH "/dev/barosim_ros2"
/*
 * BAROSIMROS2 internal constants and data structures.
 */

/* internal conversion time: 9.17 ms, so should not be read at rates higher than 100 Hz */
#define BAROSIMROS2_CONVERSION_INTERVAL	10000	/* microseconds */
#define BAROSIMROS2_MEASUREMENT_RATIO	3	/* pressure measurements per temperature measurement */

#define ADDR_RESET_CMD			0x1E	/* write to this address to reset chip */
#define ADDR_CMD_CONVERT_D1		0x48	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D2		0x58	/* write to this address to start temperature conversion */
#define ADDR_DATA			0x00	/* address of 3 bytes / 32bit pressure data */
#define ADDR_PROM_SETUP			0xA0	/* address of 8x 2 bytes factory and calibration data */
#define ADDR_PROM_C1			0xA2	/* address of 6x 2 bytes calibration data */

/* interface ioctls */
#define IOCTL_RESET			2
#define IOCTL_MEASURE			3

using namespace DriverFramework;

class BAROSIMROS2 : public VirtDevObj
{
public:
	BAROSIMROS2(const char *path);
	~BAROSIMROS2();

	virtual int		init();

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

protected:

	bool			_collect_phase;
	unsigned		_measure_phase;

	/* last report */
	struct baro_report	report;

	/* altitude conversion calibration */
	unsigned		_msl_pressure;	/* in Pa */

	//orb_advert_t		_baro_topic;
	//int			_orb_class_instance;

	perf_counter_t		_sample_perf;
	perf_counter_t		_measure_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t		_buffer_overflows;

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 *
	 * This is the heart of the measurement state machine.  This function
	 * alternately starts a measurement, or collects the data from the
	 * previous measurement.
	 *
	 * When the interval between measurements is greater than the minimum
	 * measurement interval, a gap is inserted between collection
	 * and measurement to provide the most recent measurement possible
	 * at the next interval.
	 */
	void			cycle();

	int			transfer(const uint8_t *send, unsigned send_len, uint8_t *recv, unsigned recv_len);

	/**
	 * Get the internal / external state
	 *
	 * @return true if the sensor is not on the main MCU board
	 */
	bool			is_external() { return 0;}//return (_orb_class_instance == 0); /* XXX put this into the interface class */ }

	/**
	 * Static trampoline from the workq context; because we don't have a
	 * generic workq wrapper yet.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		cycle_trampoline(void *arg);

	/**
	 * Issue a measurement command for the current state.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			measure();

	// Unused
	virtual void _measure();

	/**
	 * Collect the result of the most recent measurement.
	 */
	virtual int		collect();
	rclcpp::publisher::Publisher<std_msgs::msg::Float32>::SharedPtr pub_baro;

};

extern "C" __EXPORT int barosim_ros2_main(int argc, char *argv[]);

static BAROSIMROS2 *g_barosim = nullptr;

BAROSIMROS2::BAROSIMROS2(const char *path) :
	VirtDevObj("BAROSIMROS2", path, BARO_BASE_DEVICE_PATH, 1e6 / 100),
	// _reports(nullptr),
	_collect_phase(false),
	_measure_phase(0),
	report{},
	_msl_pressure(101325),
	_sample_perf(perf_alloc(PC_ELAPSED, "barosim_read")),
	_measure_perf(perf_alloc(PC_ELAPSED, "barosim_measure")),
	_comms_errors(perf_alloc(PC_COUNT, "barosim_comms_errors")),
	_buffer_overflows(perf_alloc(PC_COUNT, "barosim_buffer_overflows"))
{
}

BAROSIMROS2::~BAROSIMROS2()
{
	// free perf counters
	perf_free(_sample_perf);
	perf_free(_measure_perf);
	perf_free(_comms_errors);
	perf_free(_buffer_overflows);

}

int
BAROSIMROS2::init()
{
	int ret;

	ret = VirtDevObj::init();

	if (ret != OK) {
		PX4_ERR("VirtDevObj init failed");
		return ret;
	}

	auto node = rclcpp::node::Node::make_shared("talker_baro");

	pub_baro = node->create_publisher<std_msgs::msg::Float32>("sensor_baro", rmw_qos_profile_sensor_data);

	// this do..while is goto without goto //
	do {
		/* do temperature first */
		if (OK != measure()) {
			ret = -EIO;
			PX4_ERR("temp measure failed");
			break;
		}

		usleep(BAROSIMROS2_CONVERSION_INTERVAL);

		if (OK != collect()) {
			ret = -EIO;
			PX4_ERR("temp collect failed");
			break;
		}

		/* now do a pressure measurement */
		if (OK != measure()) {
			ret = -EIO;
			PX4_ERR("pressure collect failed");
			break;
		}

		usleep(BAROSIMROS2_CONVERSION_INTERVAL);

		if (OK != collect()) {
			ret = -EIO;
			PX4_ERR("pressure collect failed");
			break;
		}

		ret = OK;

	} while (0);

	return ret;
}

void
BAROSIMROS2::_measure()
{
	cycle();
}

void
BAROSIMROS2::cycle()
{
	int ret;

	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		ret = collect();

		if (ret != OK) {
			uint8_t		cmd = ADDR_RESET_CMD;

			/* bump the retry count */
			(void)transfer(&cmd, 1, nullptr, 0);

			return;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/*
		 * Is there a collect->measure gap?
		 * Don't inject one after temperature measurements, so we can keep
		 * doing pressure measurements at something close to the desired rate.
		 */
		if (_measure_phase != 0) {
			return;
		}
	}

	/* measurement phase */
	ret = measure();

	if (ret != OK) {
		return;
	}

	/* next phase is collection */
	_collect_phase = true;

}

int
BAROSIMROS2::measure()
{
	int ret;

	//PX4_WARN("baro measure %llu", hrt_absolute_time());

	perf_begin(_measure_perf);

	/*
	 * In phase zero, request temperature; in other phases, request pressure.
	 */
	unsigned long addr = (_measure_phase == 0) ? ADDR_CMD_CONVERT_D2 : ADDR_CMD_CONVERT_D1;

	/*
	 * Send the command to begin measuring.
	 */
	uint8_t cmd = addr;
	ret = transfer(&cmd, 1, nullptr, 0);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	perf_end(_measure_perf);

	return ret;
}

int
BAROSIMROS2::transfer(const uint8_t *send, unsigned send_len, uint8_t *recv, unsigned recv_len)
{
	if (send_len == 1 && send[0] == ADDR_RESET_CMD) {
		/* reset command */
		return 0;

	} else if (send_len == 1 && (send[0] == ADDR_CMD_CONVERT_D2 || send[0] == ADDR_CMD_CONVERT_D1)) {
		/* measure command */
		if (send[0] == ADDR_CMD_CONVERT_D2) {
		} else {
		}

		return 0;

	} else if (send[0] == 0 && send_len == 1) {
		/* read requested */
		Simulator *sim = Simulator::getInstance();

		if (sim == NULL) {
			PX4_ERR("Error BAROSIM_DEV::transfer no simulator");
			return -ENODEV;
		}

		PX4_DEBUG("BAROSIM_DEV::transfer getting sample");
		sim->getBaroSample(recv, recv_len);
		return recv_len;

	} else {
		PX4_WARN("BAROSIM_DEV::transfer invalid param %u %u %u", send_len, send[0], recv_len);
		return 1;
	}
	return 0;
}

int
BAROSIMROS2::collect()
{

	int ret;

	#pragma pack(push, 1)
		struct raw_baro_s {
			float		pressure;
			float		altitude;
			float		temperature;
		} raw_baro;
	#pragma pack(pop)

		perf_begin(_sample_perf);

		/* this should be fairly close to the end of the conversion, so the best approximation of the time */
		report.timestamp = hrt_absolute_time();
		report.error_count = perf_event_count(_comms_errors);

		/* read the most recent measurement - read offset/size are hardcoded in the interface */
		uint8_t cmd = 0;
		ret = transfer(&cmd, 1, (uint8_t *)(&raw_baro), sizeof(raw_baro));

		if (ret < 0) {
			perf_count(_comms_errors);
			perf_end(_sample_perf);
			return ret;
		}

		/* handle a measurement */
		if (_measure_phase == 0) {
			report.pressure = raw_baro.pressure;
			report.altitude = raw_baro.altitude;
			report.temperature = raw_baro.temperature;

		} else {
			report.pressure = raw_baro.pressure;
			report.altitude = raw_baro.altitude;
			report.temperature = raw_baro.temperature;
			// PX4_ERR("altitude barosim: %f", (double)report.altitude);

			auto msg = std::make_shared<std_msgs::msg::Float32>();
			msg->data = report.altitude;
			pub_baro->publish(msg);

			/* notify anyone waiting for data */
			updateNotify();
		}

		/* update the measurement state machine */
		INCREMENT(_measure_phase, BAROSIMROS2_MEASUREMENT_RATIO + 1);

		perf_end(_sample_perf);

		return OK;
}

void
BAROSIMROS2::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_buffer_overflows);
	PX4_INFO("poll interval:  %u usec", m_sample_interval_usecs);
	PX4_INFO("TEMP:           %f", (double)report.temperature);
	PX4_INFO("P:              %.3f", (double)report.pressure);
}

/**
 * BAROSIM crc4 cribbed from the datasheet
 */
bool
crc4(uint16_t *n_prom)
{
	int16_t cnt;
	uint16_t n_rem;
	uint16_t crc_read;
	uint8_t n_bit;

	n_rem = 0x00;

	/* save the read crc */
	crc_read = n_prom[7];

	/* remove CRC byte */
	n_prom[7] = (0xFF00 & (n_prom[7]));

	for (cnt = 0; cnt < 16; cnt++) {
		/* uneven bytes */
		if (cnt & 1) {
			n_rem ^= (uint8_t)((n_prom[cnt >> 1]) & 0x00FF);

		} else {
			n_rem ^= (uint8_t)(n_prom[cnt >> 1] >> 8);
		}

		for (n_bit = 8; n_bit > 0; n_bit--) {
			if (n_rem & 0x8000) {
				n_rem = (n_rem << 1) ^ 0x3000;

			} else {
				n_rem = (n_rem << 1);
			}
		}
	}

	/* final 4 bit remainder is CRC value */
	n_rem = (0x000F & (n_rem >> 12));
	n_prom[7] = crc_read;

	/* return true if CRCs match */
	return (0x000F & crc_read) == (n_rem ^ 0x00);
}

namespace barosim
{

static void
usage()
{
	PX4_WARN("missing command: try 'start', 'info', 'test', 'test2', 'reset', 'calibrate <altitude>'");
}

/**
 * Start the driver.
 *
 * This function call only returns once the driver
 * is either successfully up and running or failed to start.
 */
static int
start()
{

	g_barosim = new BAROSIMROS2(BAROSIMROS2_DEV_PATH);
  //
	if (g_barosim != nullptr && OK != g_barosim->init()) {
		delete g_barosim;
		g_barosim = NULL;
		PX4_ERR("bus init failed");
		return false;
	}

	DevHandle h;
	DevMgr::getHandle(BAROSIMROS2_DEV_PATH, h);

	/* set the poll rate to default, starts automatic data collection */
	if (!h.isValid()) {
		PX4_ERR("can't open baro device");
		return false;
	}

	if (h.ioctl(SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		DevMgr::releaseHandle(h);
		PX4_ERR("failed setting default poll rate");
		return false;
	}

	DevMgr::releaseHandle(h);
	return 0;
}

}; // namespace barosim

int
barosim_ros2_main(int argc, char *argv[])
{
	int ret;

	rclcpp::init(argc, argv);

	if (argc < 2) {
		barosim::usage();
		return 1;
	}

  const char *verb = argv[1];

  /*
   * Start/load the driver.
   */
  if (!strcmp(verb, "start")) {
    ret = barosim::start();
  }else{
		ret = -1;
	}

  return ret;
}
