include(nuttx/px4_impl_nuttx)
px4_nuttx_configure(HWCLASS m4 CONFIG nsh ROMFS y ROMFSROOT px4fmu_common IO)

set(config_uavcan_num_ifaces 2)

set(config_module_list
	#
	# Board support modules
	#
	drivers/airspeed
	drivers/blinkm
	drivers/bmi160
	drivers/bmp280
	drivers/boards
	drivers/bst
	drivers/camera_trigger
	drivers/device
	drivers/ets_airspeed
	drivers/frsky_telemetry
	drivers/gps
	drivers/hmc5883
	drivers/hott
	drivers/hott/hott_sensors
	drivers/hott/hott_telemetry
	drivers/iridiumsbd
	drivers/l3gd20
	drivers/led
	drivers/lis3mdl
	drivers/ll40ls
	drivers/lsm303d
	drivers/mb12xx
	drivers/mkblctrl
	drivers/mpu6000
	drivers/mpu9250
	drivers/ms4525_airspeed
	drivers/ms5525_airspeed
	drivers/ms5611
	drivers/oreoled
	drivers/pwm_input
	drivers/pwm_out_sim
	drivers/px4flow
	drivers/px4fmu
	drivers/px4io
	drivers/rgbled
	drivers/sdp3x_airspeed
	drivers/sf0x
	drivers/sf1xx
	drivers/srf02
	drivers/stm32
	drivers/stm32/adc
	drivers/stm32/tone_alarm
	drivers/tap_esc
	drivers/teraranger
	drivers/vmount
	modules/sensors

	#
	# System commands
	#
	systemcmds/bl_update
	systemcmds/config
	systemcmds/dumpfile
	systemcmds/esc_calib
	systemcmds/hardfault_log
	systemcmds/led_control
	systemcmds/mixer
	systemcmds/motor_ramp
	systemcmds/motor_test
	systemcmds/mtd
	systemcmds/nshterm
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
	drivers/test_ppm
	#lib/rc/rc_tests
	modules/commander/commander_tests
	lib/controllib/controllib_test
	modules/mavlink/mavlink_tests
	modules/mc_pos_control/mc_pos_control_tests
	modules/uORB/uORB_tests
	systemcmds/tests

	#
	# General system control
	#
	modules/commander
	modules/events
	modules/gpio_led
	modules/land_detector
	modules/load_mon
	modules/mavlink
	modules/navigator
	modules/uavcan
	modules/camera_feedback

	#
	# Estimation modules
	#
	modules/attitude_estimator_q
	modules/ekf2
	modules/local_position_estimator
	modules/position_estimator_inav

	
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
	#lib/runway_takeoff
	#lib/tailsitter_recovery
	#lib/terrain_estimation
	lib/version

	#
	# Platform
	#
	platforms/common
	platforms/nuttx
	platforms/nuttx/px4_layer

	
	# Tutorial code from
	# https://px4.io/dev/px4_simple_app
	examples/px4_simple_app

	# Tutorial code from
	# https://px4.io/dev/daemon
	examples/px4_daemon_app

	# Tutorial code from
	# https://px4.io/dev/debug_values
	examples/px4_mavlink_debug


	# Hardware test
	examples/hwtest
)
