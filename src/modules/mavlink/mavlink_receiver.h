/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file mavlink_receiver.h
 * MAVLink receiver thread
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 */

#pragma once

#include <systemlib/perf_counter.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/gps_inject_data.h>

#include "mavlink_parameters.h"
#include "mavlink_ftp.h"
#include "mavlink_log_handler.h"

#define PX4_EPOCH_SECS 1234567890ULL

class Mavlink;

class MavlinkReceiver
{
public:
	/**
	 * Constructor
	 */
	MavlinkReceiver(Mavlink *parent);

	/**
	 * Destructor, also kills the mavlinks task.
	 */
	~MavlinkReceiver();

	/**
	 * Display the mavlink status.
	 */
	void		print_status();

	/**
	 * Start the receiver thread
	 */
	static void receive_start(pthread_t *thread, Mavlink *parent);

	static void *start_helper(void *context);

private:

	void handle_message(mavlink_message_t *msg);
	void handle_message_command_long(mavlink_message_t *msg);
	void handle_message_command_ack(mavlink_message_t *msg);
	void handle_message_optical_flow_rad(mavlink_message_t *msg);
	void handle_message_hil_optical_flow(mavlink_message_t *msg);
	void handle_message_radio_status(mavlink_message_t *msg);
	void handle_message_heartbeat(mavlink_message_t *msg);
	void handle_message_ping(mavlink_message_t *msg);
	void handle_message_system_time(mavlink_message_t *msg);
	void handle_message_hil_sensor(mavlink_message_t *msg);
	void handle_message_hil_gps(mavlink_message_t *msg);
	void handle_message_distance_sensor(mavlink_message_t *msg);
	void handle_message_battery_status(mavlink_message_t *msg);
	void handle_message_serial_control(mavlink_message_t *msg);
	void handle_message_logging_ack(mavlink_message_t *msg);
	void handle_message_play_tune(mavlink_message_t *msg);
	void handle_message_gps_rtcm_data(mavlink_message_t *msg);

	void *receive_thread(void *arg);

	/**
	 * Set the interval at which the given message stream is published.
	 * The rate is the number of messages per second.
	 *
	 * @param msgId the message ID of to change the interval of
	 * @param interval the interval in us to send the message at
	 * @param data_rate the total link data rate in bytes per second
	 *
	 * @return PX4_OK on success, PX4_ERROR on fail
	 */
	int set_message_interval(int msgId, float interval, int data_rate = -1);
	void get_message_interval(int msgId);

	bool	evaluate_target_ok(int command, int target_system, int target_component);

	Mavlink	*_mavlink;

	MavlinkParametersManager	_parameters_manager;
	MavlinkFTP			_mavlink_ftp;
	MavlinkLogHandler		_mavlink_log_handler;

	mavlink_status_t _status; ///< receiver status, used for mavlink_parse_char()
	orb_advert_t _gps_pub;
	orb_advert_t _gyro_pub;
	orb_advert_t _accel_pub;
	orb_advert_t _mag_pub;
	orb_advert_t _baro_pub;
	orb_advert_t _airspeed_pub;
	orb_advert_t _battery_pub;
	orb_advert_t _cmd_pub;
	orb_advert_t _flow_pub;
	orb_advert_t _hil_distance_sensor_pub;
	orb_advert_t _flow_distance_sensor_pub;
	orb_advert_t _distance_sensor_pub;
	orb_advert_t _telemetry_status_pub;
	orb_advert_t _command_ack_pub;
	static const int _gps_inject_data_queue_size = 6;
	orb_advert_t _gps_inject_data_pub;

	int _hil_frames;
	uint64_t _old_timestamp;
	int	_orb_class_instance;

	param_t _p_bat_emergen_thr;
	param_t _p_bat_crit_thr;
	param_t _p_bat_low_thr;

	MavlinkReceiver(const MavlinkReceiver &) = delete;
	MavlinkReceiver operator=(const MavlinkReceiver &) = delete;
};
