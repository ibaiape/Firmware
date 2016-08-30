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
 * @file df_gpioled_wrapper.cpp
 * Lightweight driver to access the GPIO_LED of the DriverFramework.
 *
 * @author @mocibb
 * @author Víctor Mayoral Vilches <victor@erlerobot.com> 
 *
 */

#include <px4_config.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <px4_getopt.h>
#include <errno.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_led.h>

#include <board_config.h>

#include <gpioled/GPIO_LED.hpp>
#include <DevMgr.hpp>


extern "C" { __EXPORT int df_gpioled_wrapper_main(int argc, char *argv[]); }

using namespace DriverFramework;


class DfGPIOLEDWrapper : public GPIOLED
{
public:
    DfGPIOLEDWrapper();
    ~DfGPIOLEDWrapper();

    virtual int devIOCTL(unsigned long request, unsigned long arg);
};

DfGPIOLEDWrapper::DfGPIOLEDWrapper() :
    GPIOLED()
{
}

DfGPIOLEDWrapper::~DfGPIOLEDWrapper()
{
}

int DfGPIOLEDWrapper::devIOCTL(unsigned long request, unsigned long arg)
{
    switch (request) {

        case LED_ON:
            return on(arg);

        case LED_OFF:
            return off(arg);

        case LED_TOGGLE:
            return toggle(arg);

        default:
            /* give it to the superclass */
            return GPIOLED::devIOCTL(request, arg);
        }
}


namespace df_gpioled_wrapper
{

DfGPIOLEDWrapper *g_dev = nullptr;

int start();
int stop();
int info();
void usage();

int start()
{
    g_dev = new DfGPIOLEDWrapper();

	if (g_dev == nullptr) {
        PX4_ERR("failed instantiating DfGPIOLEDWrapper object");
		return -1;
	}

    //int ret = g_dev->start();
    g_dev->init();
    g_dev->start();

    // don't check return value of start, because GPIO is loop running device it alway return -3
    //if (ret != 0) {
    //    PX4_ERR("DfGPIOLEDWrapper start failed");
    //	return ret;
    //}

    // Open the GPIOLED sensor
	DevHandle h;
    DevMgr::getHandle(LED0_DEVICE_PATH, h);

	if (!h.isValid()) {
		DF_LOG_INFO("Error: unable to obtain a valid handle for the receiver at: %s (%d)",
                LED0_DEVICE_PATH, h.getError());
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

    g_dev->stop();

//	if (ret != 0) {
//		PX4_ERR("driver could not be stopped");
//		return ret;
//	}

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
    PX4_WARN("Usage: df_gpioled_wrapper 'start', 'info', 'stop'");
}

} // namespace df_gpioled_wrapper


int
df_gpioled_wrapper_main(int argc, char *argv[])
{
	int ret = 0;
	int myoptind = 1;

	if (argc <= 1) {
        df_gpioled_wrapper::usage();
		return 1;
	}

	const char *verb = argv[myoptind];


	if (!strcmp(verb, "start")) {
        ret = df_gpioled_wrapper::start();
	}

	else if (!strcmp(verb, "stop")) {
        ret = df_gpioled_wrapper::stop();
	}

	else if (!strcmp(verb, "info")) {
        ret = df_gpioled_wrapper::info();
	}

	else {
        df_gpioled_wrapper::usage();
		return 1;
	}

	return ret;
}
