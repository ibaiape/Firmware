# This file is shared between posix_erlebrain2_native.cmake
# and posix_erlebrain2_native.cmake.

include(posix/px4_impl_posix)

# This definition allows to differentiate if this just the usual POSIX build
# or if it is for the RPi.
add_definitions(
#	-D__PX4_POSIX_RPI
#	-D__PX4_POSIX_ERLEBRAIN2
#	-D__LINUX
#	-D__DF_LINUX # For DriverFramework
#  -D__RPI
	-D__PX4_POSIX_RPI
	-D__DF_LINUX # For DriverFramework
	-D__LINUX
	-D__PX4_POSIX_ERLEBRAIN2

)

set(config_module_list
	#
	# Board support modules
	#
	drivers/device
	modules/sensors
	#platforms/posix/drivers/df_gpioled_wrapper
	platforms/posix/drivers/df_erlebrain2_rc_in_wrapper
  platforms/posix/drivers/df_erlebrain2_pwm_out_wrapper
	platforms/posix/drivers/df_mpu9250_wrapper
	platforms/posix/drivers/df_ms5611_wrapper
	platforms/posix/drivers/df_hmc5883_wrapper

	#
	# System commands
	#
	systemcmds/param
	systemcmds/mixer
	systemcmds/ver
	systemcmds/esc_calib
	systemcmds/topic_listener
	systemcmds/perf

	#
	# Estimation modules (EKF/ SO3 / other filters)
	#
	modules/attitude_estimator_q
	modules/position_estimator_inav
	modules/local_position_estimator
	modules/ekf2

	#
	# Vehicle Control
	#
	modules/mc_att_control
	modules/mc_pos_control
	modules/fw_att_control
	modules/fw_pos_control_l1
	modules/vtol_att_control

	#
	# Library modules
	#
	modules/sdlog2
	modules/logger
	modules/commander
	modules/param
	modules/systemlib
	modules/systemlib/mixer
	modules/uORB
	modules/dataman
	modules/land_detector
	modules/navigator
	modules/mavlink

	#
	# PX4 drivers
	#
	drivers/gps
	#
	# Libraries
	#
	lib/controllib
	lib/mathlib
	lib/mathlib/math/filter
	lib/geo
	lib/ecl
	lib/geo_lookup
	lib/launchdetection
	lib/external_lgpl
	lib/conversion
	lib/terrain_estimation
	lib/runway_takeoff
	lib/tailsitter_recovery
	lib/DriverFramework/framework

	#
	# POSIX
	#
	platforms/common
	platforms/posix/px4_layer
	platforms/posix/work_queue
)

set(config_df_driver_list
	common
	erlebrain2_rc_in
	gpioled
	hmc5883
	mpu9250
	ms5611
	pca9685
)
