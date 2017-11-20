include(posix/px4_impl_posix)

set(CMAKE_TOOLCHAIN_FILE ${PX4_SOURCE_DIR}/cmake/toolchains/Toolchain-native.cmake)

set(config_module_list
	drivers/boards
	drivers/camera_trigger
	drivers/device
	drivers/gps
	drivers/pwm_out_sim
	drivers/vmount
	drivers/linux_gpio
	drivers/airspeed
	drivers/ets_airspeed
	drivers/ms4525_airspeed
	drivers/ms5525_airspeed
	drivers/sdp3x_airspeed

	modules/sensors
	platforms/posix/drivers/accelsim
	platforms/posix/drivers/adcsim
	platforms/posix/drivers/airspeedsim
	platforms/posix/drivers/barosim
	platforms/posix/drivers/gpssim
	platforms/posix/drivers/gyrosim
	platforms/posix/drivers/ledsim
	platforms/posix/drivers/tonealrmsim

	#
	# System commands
	#
	#systemcmds/bl_update
	#systemcmds/config
	#systemcmds/dumpfile
	systemcmds/esc_calib
	systemcmds/led_control
	systemcmds/mixer
	systemcmds/motor_ramp
	#systemcmds/mtd
	#systemcmds/nshterm
	systemcmds/param
	systemcmds/perf
	systemcmds/pwm
	systemcmds/reboot
	systemcmds/sd_bench
	systemcmds/top
	systemcmds/topic_listener
	systemcmds/ver

	#
	# Testing
	#
	drivers/sf0x/sf0x_tests
	#drivers/test_ppm
	lib/rc/rc_tests
	modules/commander/commander_tests
	lib/controllib/controllib_test
	modules/mavlink/mavlink_tests
	modules/mc_pos_control/mc_pos_control_tests
	modules/uORB/uORB_tests
	systemcmds/tests

	platforms/posix/tests/hello
	platforms/posix/tests/hrt_test
	platforms/posix/tests/muorb
	platforms/posix/tests/vcdev_test
	platforms/posix/tests/wqueue

	#
	# General system control
	#
	modules/commander
	modules/events
	#modules/gpio_led
	modules/land_detector
	modules/load_mon
	modules/mavlink
	modules/navigator
	modules/replay
	modules/simulator
	#modules/uavcan

	#
	# Estimation modules
	#
	modules/attitude_estimator_q
	modules/ekf2
	modules/local_position_estimator
	#modules/position_estimator_inav


	#
	# Logging
	#
	modules/logger
	modules/sdlog2

	#
	# Library modules
	#
	modules/dataman
	modules/systemlib/param
	modules/systemlib
	modules/systemlib/mixer
	modules/uORB

	#
	# Libraries
	#
	lib/controllib
	lib/conversion
	lib/DriverFramework/framework
	lib/ecl
	lib/geo
	lib/geo_lookup
	lib/launchdetection
	lib/led
	lib/mathlib
	lib/mathlib/math/filter
	lib/rc
	lib/runway_takeoff
	#lib/tailsitter_recovery
	#lib/terrain_estimation
	lib/version

	#
	# Platform
	#
	platforms/common
	platforms/posix/px4_layer
	platforms/posix/work_queue

	
	# Tutorial code from
	# https://px4.io/dev/px4_simple_app
	examples/px4_simple_app

	# Tutorial code from
	# https://px4.io/dev/daemon
	examples/px4_daemon_app

	# Tutorial code from
	# https://px4.io/dev/debug_values
	examples/px4_mavlink_debug
)

# Default config_sitl_rcS_dir (posix_sitl_default), this is overwritten later
# for the config posix_sitl_efk2 and set again, explicitly, for posix_sitl_lpe,
# which are based on posix_sitl_default.
set(config_sitl_rcS_dir configs/posix-configs/SITL/init/ekf2 CACHE INTERNAL "init script dir for sitl")

set(config_sitl_viewer jmavsim CACHE STRING "viewer for sitl")
set_property(CACHE config_sitl_viewer PROPERTY STRINGS "jmavsim;none")

set(config_sitl_debugger disable CACHE STRING "debugger for sitl")
set_property(CACHE config_sitl_debugger PROPERTY STRINGS "disable;gdb;lldb")

# If the environment variable 'replay' is defined, we are building with replay
# support. In this case, we enable the orb publisher rules.
set(REPLAY_FILE "$ENV{replay}")
if(REPLAY_FILE)
	message("Building with uorb publisher rules support")
	add_definitions(-DORB_USE_PUBLISHER_RULES)
endif()
