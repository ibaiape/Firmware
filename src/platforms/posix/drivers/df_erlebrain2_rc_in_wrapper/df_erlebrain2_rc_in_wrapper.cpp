/****************************************************************************
 *
 * Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file df_erlebrain2_rc_in_wrapper.cpp
 * Lightweight driver to access the RCIN of the DriverFramework.
 *
 * @author @mocibb
 * @author Víctor Mayoral Vilches <victor@erlerobot.com>
 * @author Alejandro Hernández Cordero <alex@erlerobotics.com>
 *
 */

#include <drivers/drv_hrt.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include <px4_config.h>
#include <px4_getopt.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_rc_input.h>

#include <board_config.h>

#include <erlebrain2_rc_in/ERLEBRAIN2_RC_IN.hpp>
#include <DevMgr.hpp>

#include <uORB/uORB.h>
#include <uORB/topics/input_rc.h>

extern "C" { __EXPORT int df_erlebrain2_rc_in_wrapper_main(int argc, char *argv[]); }

using namespace DriverFramework;


class DfErleBrain2RcInputWrapper : public ErleBrain2RcInput
{
public:
    DfErleBrain2RcInputWrapper();
    ~DfErleBrain2RcInputWrapper();

    virtual int init();

    virtual int _publish(struct rc_channel_data &data);
private:
    orb_advert_t _rcinput_pub;
    struct input_rc_s _data;
};

DfErleBrain2RcInputWrapper::DfErleBrain2RcInputWrapper() :
    ErleBrain2RcInput("/dev/input_rc"),
    _rcinput_pub(nullptr),
    _data{}
{
}

DfErleBrain2RcInputWrapper::~DfErleBrain2RcInputWrapper()
{
}

int DfErleBrain2RcInputWrapper::init() {
    int ret = ErleBrain2RcInput::init();

    if (ret < 0) {
        return ret;
    }

    _rcinput_pub = orb_advertise(ORB_ID(input_rc), &_data);

    if (_rcinput_pub == nullptr) {
        PX4_WARN("error: advertise failed");
        return ERROR;
    }

    return OK;
}

int DfErleBrain2RcInputWrapper::_publish(struct rc_channel_data &data){
    uint64_t ts = hrt_absolute_time();
    //_data.timestamp_publication = ts;
    _data.timestamp_last_signal = ts;
    for (int i = 0; i < data.channel_count; ++i) {
        _data.values[i] = data.values[i];
    }
    _data.channel_count = data.channel_count;
    _data.rssi = 100;
    _data.rc_lost_frame_count = 0;
    _data.rc_total_frame_count = 1;
    _data.rc_ppm_frame_length = 100;
    _data.rc_failsafe = false;
    _data.rc_lost = false;
    _data.input_source = input_rc_s::RC_INPUT_SOURCE_PX4IO_PPM;

    orb_publish(ORB_ID(input_rc), _rcinput_pub, &_data);

    return OK;
}

namespace df_erlebrain2_rc_in_wrapper
{

DfErleBrain2RcInputWrapper *g_dev = nullptr;

int start();
int stop();
int info();
void usage();

int start()
{
    g_dev = new DfErleBrain2RcInputWrapper();

	if (g_dev == nullptr) {
        PX4_ERR("failed instantiating DfErleBrain2RcInputWrapper object");
		return -1;
	}

	int ret = g_dev->init();
	if (ret != 0) {
            PX4_ERR("DfErleBrain2RcInputWrapper init failed");
            return ret;
	}

	ret = g_dev->start();

	if (ret != 0) {
        PX4_ERR("DfErleBrain2RcInputWrapper start failed");
		return ret;
	}

    // Open the GPIOLED sensor
	DevHandle h;
    DevMgr::getHandle(RC_INPUT0_DEVICE_PATH, h);

	if (!h.isValid()) {
		DF_LOG_INFO("Error: unable to obtain a valid handle for the receiver at: %s (%d)",
                RC_INPUT0_DEVICE_PATH, h.getError());
		return -1;
	}

	DevMgr::releaseHandle(h);

	return 0;
}

int stop()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return 1;
	}

	int ret = g_dev->stop();

	if (ret != 0) {
		PX4_ERR("driver could not be stopped");
		return ret;
	}

	delete g_dev;
	g_dev = nullptr;
	return 0;
}

/**
 * Print a little info about the driver.
 */
int
info()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return 1;
	}

	PX4_DEBUG("state @ %p", g_dev);

	return 0;
}

void
usage()
{
    PX4_WARN("Usage: df_erlebrain2_rc_in_wrapper 'start', 'info', 'stop'");
}

} // namespace df_erlebrain2_rc_in_wrapper


int
df_erlebrain2_rc_in_wrapper_main(int argc, char *argv[])
{
	int ret = 0;
	int myoptind = 1;

	if (argc <= 1) {
        df_erlebrain2_rc_in_wrapper::usage();
		return 1;
	}

	const char *verb = argv[myoptind];


	if (!strcmp(verb, "start")) {
        ret = df_erlebrain2_rc_in_wrapper::start();
	}

	else if (!strcmp(verb, "stop")) {
        ret = df_erlebrain2_rc_in_wrapper::stop();
	}

	else if (!strcmp(verb, "info")) {
        ret = df_erlebrain2_rc_in_wrapper::info();
	}

	else {
        df_erlebrain2_rc_in_wrapper::usage();
		return 1;
	}

	return ret;
}
