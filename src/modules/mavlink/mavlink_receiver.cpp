/****************************************************************************
 *
 *   Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
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
 * @file mavlink_receiver.cpp
 * MAVLink protocol message receive and dispatch
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 * @author Thomas Gubler <thomas@px4.io>
 */

/* XXX trim includes */
#include <px4_config.h>
#include <px4_time.h>
#include <px4_tasks.h>
#include <px4_defines.h>
#include <px4_posix.h>
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <fcntl.h>
#include <string.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_baro.h>
#include <drivers/drv_range_finder.h>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_tone_alarm.h>
#include <time.h>
#include <float.h>
#include <unistd.h>
#ifndef __PX4_POSIX
#include <termios.h>
#endif
#include <errno.h>
#include <stdlib.h>
#include <poll.h>

#include <mathlib/mathlib.h>

#include <conversion/rotation.h>

#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>
#include <systemlib/mavlink_log.h>
#include <systemlib/err.h>
#include <systemlib/airspeed.h>
//#include <commander/px4_custom_mode.h>
#include <geo/geo.h>

#include "mavlink_bridge_header.h"
#include "mavlink_receiver.h"
#include "mavlink_main.h"
#include "mavlink_command_sender.h"

static const float mg2ms2 = CONSTANTS_ONE_G / 1000.0f;

MavlinkReceiver::MavlinkReceiver(Mavlink *parent) :
	_mavlink(parent),
	_parameters_manager(parent),
	_mavlink_ftp(parent),
	_mavlink_log_handler(parent),
	_status{},
	_gps_pub(nullptr),
	_gyro_pub(nullptr),
	_accel_pub(nullptr),
	_mag_pub(nullptr),
	_baro_pub(nullptr),
	_airspeed_pub(nullptr),
	_battery_pub(nullptr),
	_cmd_pub(nullptr),
	_flow_pub(nullptr),
	_hil_distance_sensor_pub(nullptr),
	_flow_distance_sensor_pub(nullptr),
	_distance_sensor_pub(nullptr),
	_telemetry_status_pub(nullptr),
	_command_ack_pub(nullptr),
	_gps_inject_data_pub(nullptr),
	_hil_frames(0),
	_old_timestamp(0),
	_orb_class_instance(-1),
	_p_bat_emergen_thr(param_find("BAT_EMERGEN_THR")),
	_p_bat_crit_thr(param_find("BAT_CRIT_THR")),
	_p_bat_low_thr(param_find("BAT_LOW_THR"))
{
}

MavlinkReceiver::~MavlinkReceiver()
{
}

void
MavlinkReceiver::handle_message(mavlink_message_t *msg)
{
	if (!_mavlink->get_config_link_on()) {
		if (_mavlink->get_mode() == Mavlink::MAVLINK_MODE_CONFIG) {
			_mavlink->set_config_link_on(true);
		}
	}

	switch (msg->msgid) {
	case MAVLINK_MSG_ID_COMMAND_LONG:
		if (_mavlink->accepting_commands()) {
			handle_message_command_long(msg);
		}
		break;

	case MAVLINK_MSG_ID_COMMAND_ACK:
		handle_message_command_ack(msg);
		break;

	case MAVLINK_MSG_ID_OPTICAL_FLOW_RAD:
		handle_message_optical_flow_rad(msg);
		break;

	case MAVLINK_MSG_ID_PING:
		handle_message_ping(msg);
		break;

	case MAVLINK_MSG_ID_RADIO_STATUS:
		handle_message_radio_status(msg);
		break;

	case MAVLINK_MSG_ID_HEARTBEAT:
		handle_message_heartbeat(msg);
		break;

	case MAVLINK_MSG_ID_SYSTEM_TIME:
		handle_message_system_time(msg);
		break;

	case MAVLINK_MSG_ID_DISTANCE_SENSOR:
		handle_message_distance_sensor(msg);
		break;

	case MAVLINK_MSG_ID_BATTERY_STATUS:
		handle_message_battery_status(msg);
		break;

	case MAVLINK_MSG_ID_SERIAL_CONTROL:
		handle_message_serial_control(msg);
		break;

	case MAVLINK_MSG_ID_LOGGING_ACK:
		handle_message_logging_ack(msg);
		break;

	case MAVLINK_MSG_ID_PLAY_TUNE:
		handle_message_play_tune(msg);
		break;

	case MAVLINK_MSG_ID_GPS_RTCM_DATA:
		handle_message_gps_rtcm_data(msg);
		break;

	default:
		break;
	}

	/*
	 * Only decode hil messages in HIL mode.
	 *
	 * The HIL mode is enabled by the HIL bit flag
	 * in the system mode. Either send a set mode
	 * COMMAND_LONG message or a SET_MODE message
	 *
	 * Accept HIL GPS messages if use_hil_gps flag is true.
	 * This allows to provide fake gps measurements to the system.
	 */
	if (_mavlink->get_hil_enabled()) {

		switch (msg->msgid) {
		case MAVLINK_MSG_ID_HIL_SENSOR:
			handle_message_hil_sensor(msg);
			break;

		case MAVLINK_MSG_ID_HIL_OPTICAL_FLOW:
			handle_message_hil_optical_flow(msg);
			break;

		default:
			break;
		}
	}

	if (_mavlink->get_hil_enabled() || (_mavlink->get_use_hil_gps() && msg->sysid == mavlink_system.sysid)) {

		switch (msg->msgid) {
		case MAVLINK_MSG_ID_HIL_GPS:
			handle_message_hil_gps(msg);
			break;

		default:
			break;
		}

	}

	/* If we've received a valid message, mark the flag indicating so.
	   This is used in the '-w' command-line flag. */
	_mavlink->set_has_received_messages(true);
}

bool
MavlinkReceiver::evaluate_target_ok(int command, int target_system, int target_component)
{
	/* evaluate if this system should accept this command */
	bool target_ok = false;

	switch (command) {

	case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:

	/* fallthrough */
	case MAV_CMD_REQUEST_PROTOCOL_VERSION:
		/* broadcast and ignore component */
		target_ok = (target_system == 0) || (target_system == mavlink_system.sysid);
		break;

	default:
		target_ok = (target_system == mavlink_system.sysid) && ((target_component == mavlink_system.compid)
				|| (target_component == MAV_COMP_ID_ALL));
		break;
	}

	return target_ok;
}

void
MavlinkReceiver::handle_message_command_long(mavlink_message_t *msg)
{
	/* command */
	mavlink_command_long_t cmd_mavlink;
	mavlink_msg_command_long_decode(msg, &cmd_mavlink);

	bool target_ok = evaluate_target_ok(cmd_mavlink.command, cmd_mavlink.target_system, cmd_mavlink.target_component);

	bool send_ack = true;
	int ret = 0;

	if (!target_ok) {
		ret = PX4_ERROR;
		goto out;
	}

	//check for MAVLINK terminate command
	if (cmd_mavlink.command == MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN && ((int)cmd_mavlink.param1) == 10) {
		/* This is the link shutdown command, terminate mavlink */
		warnx("terminated by remote");
		fflush(stdout);
		usleep(50000);

		/* terminate other threads and this thread */
		_mavlink->_task_should_exit = true;

	} else if (cmd_mavlink.command == MAV_CMD_REQUEST_PROTOCOL_VERSION) {
		/* send protocol version message */
		_mavlink->send_protocol_version();

	} else if (cmd_mavlink.command == MAV_CMD_SET_MESSAGE_INTERVAL) {
		ret = set_message_interval((int)(cmd_mavlink.param1 + 0.5f),
					   cmd_mavlink.param2, cmd_mavlink.param3);

	} else if (cmd_mavlink.command == MAV_CMD_GET_MESSAGE_INTERVAL) {
		get_message_interval((int)cmd_mavlink.param1);

	} else {

		send_ack = false;

		if (msg->sysid == mavlink_system.sysid && msg->compid == mavlink_system.compid) {
			PX4_WARN("ignoring CMD with same SYS/COMP (%d/%d) ID",
				 mavlink_system.sysid, mavlink_system.compid);
			return;
		}

		if (cmd_mavlink.command == MAV_CMD_LOGGING_START) {
			// we already instanciate the streaming object, because at this point we know on which
			// mavlink channel streaming was requested. But in fact it's possible that the logger is
			// not even running. The main mavlink thread takes care of this by waiting for an ack
			// from the logger.
			_mavlink->try_start_ulog_streaming(msg->sysid, msg->compid);

		} else if (cmd_mavlink.command == MAV_CMD_LOGGING_STOP) {
			_mavlink->request_stop_ulog_streaming();
		}

		struct vehicle_command_s vcmd;

		memset(&vcmd, 0, sizeof(vcmd));

		vcmd.timestamp = hrt_absolute_time();

		/* Copy the content of mavlink_command_long_t cmd_mavlink into command_t cmd */
		vcmd.param1 = cmd_mavlink.param1;
		vcmd.param2 = cmd_mavlink.param2;
		vcmd.param3 = cmd_mavlink.param3;
		vcmd.param4 = cmd_mavlink.param4;
		vcmd.param5 = cmd_mavlink.param5;
		vcmd.param6 = cmd_mavlink.param6;
		vcmd.param7 = cmd_mavlink.param7;

		// XXX do proper translation
		vcmd.command = cmd_mavlink.command;
		vcmd.target_system = cmd_mavlink.target_system;
		vcmd.target_component = cmd_mavlink.target_component;
		vcmd.source_system = msg->sysid;
		vcmd.source_component = msg->compid;
		vcmd.confirmation =  cmd_mavlink.confirmation;

		if (_cmd_pub == nullptr) {
			_cmd_pub = orb_advertise_queue(ORB_ID(vehicle_command), &vcmd, vehicle_command_s::ORB_QUEUE_LENGTH);

		} else {
			orb_publish(ORB_ID(vehicle_command), _cmd_pub, &vcmd);
		}
	}

out:

	if (send_ack) {

		vehicle_command_ack_s command_ack;
		command_ack.command = cmd_mavlink.command;

		if (ret == PX4_OK) {
			command_ack.result = vehicle_command_ack_s::VEHICLE_RESULT_ACCEPTED;

		} else {
			command_ack.result = vehicle_command_ack_s::VEHICLE_RESULT_FAILED;
		}

		if (_command_ack_pub == nullptr) {
			_command_ack_pub = orb_advertise_queue(ORB_ID(vehicle_command_ack), &command_ack,
							       vehicle_command_ack_s::ORB_QUEUE_LENGTH);

		} else {
			orb_publish(ORB_ID(vehicle_command_ack), _command_ack_pub, &command_ack);
		}
	}
}

void
MavlinkReceiver::handle_message_command_ack(mavlink_message_t *msg)
{
	mavlink_command_ack_t ack;
	mavlink_msg_command_ack_decode(msg, &ack);

	MavlinkCommandSender::instance().handle_mavlink_command_ack(ack, msg->sysid, msg->compid);

	if (ack.result != MAV_RESULT_ACCEPTED && ack.result != MAV_RESULT_IN_PROGRESS) {
		if (msg->compid == MAV_COMP_ID_CAMERA) {
			PX4_WARN("Got unsuccessful result %d from camera", ack.result);
		}
	}
}

void
MavlinkReceiver::handle_message_optical_flow_rad(mavlink_message_t *msg)
{
	/* optical flow */
	mavlink_optical_flow_rad_t flow;
	mavlink_msg_optical_flow_rad_decode(msg, &flow);

	enum Rotation flow_rot;
	param_get(param_find("SENS_FLOW_ROT"), &flow_rot);

	struct optical_flow_s f = {};

	f.timestamp = flow.time_usec;
	f.integration_timespan = flow.integration_time_us;
	f.pixel_flow_x_integral = flow.integrated_x;
	f.pixel_flow_y_integral = flow.integrated_y;
	f.gyro_x_rate_integral = flow.integrated_xgyro;
	f.gyro_y_rate_integral = flow.integrated_ygyro;
	f.gyro_z_rate_integral = flow.integrated_zgyro;
	f.time_since_last_sonar_update = flow.time_delta_distance_us;
	f.ground_distance_m = flow.distance;
	f.quality = flow.quality;
	f.sensor_id = flow.sensor_id;
	f.gyro_temperature = flow.temperature;

	/* rotate measurements according to parameter */
	float zeroval = 0.0f;
	rotate_3f(flow_rot, f.pixel_flow_x_integral, f.pixel_flow_y_integral, zeroval);
	rotate_3f(flow_rot, f.gyro_x_rate_integral, f.gyro_y_rate_integral, f.gyro_z_rate_integral);

	if (_flow_pub == nullptr) {
		_flow_pub = orb_advertise(ORB_ID(optical_flow), &f);

	} else {
		orb_publish(ORB_ID(optical_flow), _flow_pub, &f);
	}

	/* Use distance value for distance sensor topic */
	struct distance_sensor_s d = {};

	if (flow.distance > 0.0f) { // negative values signal invalid data
		d.timestamp = flow.integration_time_us * 1000; /* ms to us */
		d.min_distance = 0.3f;
		d.max_distance = 5.0f;
		d.current_distance = flow.distance; /* both are in m */
		d.type = 1;
		d.id = MAV_DISTANCE_SENSOR_ULTRASOUND;
		d.orientation = 8;
		d.covariance = 0.0;

		if (_flow_distance_sensor_pub == nullptr) {
			_flow_distance_sensor_pub = orb_advertise_multi(ORB_ID(distance_sensor), &d,
						    &_orb_class_instance, ORB_PRIO_HIGH);

		} else {
			orb_publish(ORB_ID(distance_sensor), _flow_distance_sensor_pub, &d);
		}
	}
}

void
MavlinkReceiver::handle_message_hil_optical_flow(mavlink_message_t *msg)
{
	/* optical flow */
	mavlink_hil_optical_flow_t flow;
	mavlink_msg_hil_optical_flow_decode(msg, &flow);

	struct optical_flow_s f;
	memset(&f, 0, sizeof(f));

	f.timestamp = hrt_absolute_time(); // XXX we rely on the system time for now and not flow.time_usec;
	f.integration_timespan = flow.integration_time_us;
	f.pixel_flow_x_integral = flow.integrated_x;
	f.pixel_flow_y_integral = flow.integrated_y;
	f.gyro_x_rate_integral = flow.integrated_xgyro;
	f.gyro_y_rate_integral = flow.integrated_ygyro;
	f.gyro_z_rate_integral = flow.integrated_zgyro;
	f.time_since_last_sonar_update = flow.time_delta_distance_us;
	f.ground_distance_m = flow.distance;
	f.quality = flow.quality;
	f.sensor_id = flow.sensor_id;
	f.gyro_temperature = flow.temperature;

	if (_flow_pub == nullptr) {
		_flow_pub = orb_advertise(ORB_ID(optical_flow), &f);

	} else {
		orb_publish(ORB_ID(optical_flow), _flow_pub, &f);
	}

	/* Use distance value for distance sensor topic */
	struct distance_sensor_s d;
	memset(&d, 0, sizeof(d));

	d.timestamp = hrt_absolute_time();
	d.min_distance = 0.3f;
	d.max_distance = 5.0f;
	d.current_distance = flow.distance; /* both are in m */
	d.type = 1;
	d.id = 0;
	d.orientation = 8;
	d.covariance = 0.0;

	if (_hil_distance_sensor_pub == nullptr) {
		_hil_distance_sensor_pub = orb_advertise_multi(ORB_ID(distance_sensor), &d,
					   &_orb_class_instance, ORB_PRIO_HIGH);

	} else {
		orb_publish(ORB_ID(distance_sensor), _hil_distance_sensor_pub, &d);
	}
}


void
MavlinkReceiver::handle_message_distance_sensor(mavlink_message_t *msg)
{
	/* distance sensor */
	mavlink_distance_sensor_t dist_sensor;
	mavlink_msg_distance_sensor_decode(msg, &dist_sensor);

	struct distance_sensor_s d;
	memset(&d, 0, sizeof(d));

	d.timestamp = dist_sensor.time_boot_ms * 1000; /* ms to us */
	d.min_distance = float(dist_sensor.min_distance) * 1e-2f; /* cm to m */
	d.max_distance = float(dist_sensor.max_distance) * 1e-2f; /* cm to m */
	d.current_distance = float(dist_sensor.current_distance) * 1e-2f; /* cm to m */
	d.type = dist_sensor.type;
	d.id = 	MAV_DISTANCE_SENSOR_LASER;
	d.orientation = dist_sensor.orientation;
	d.covariance = dist_sensor.covariance;

	/// TODO Add sensor rotation according to MAV_SENSOR_ORIENTATION enum

	if (_distance_sensor_pub == nullptr) {
		_distance_sensor_pub = orb_advertise_multi(ORB_ID(distance_sensor), &d,
				       &_orb_class_instance, ORB_PRIO_HIGH);

	} else {
		orb_publish(ORB_ID(distance_sensor), _distance_sensor_pub, &d);
	}
}

void
MavlinkReceiver::handle_message_radio_status(mavlink_message_t *msg)
{
	/* telemetry status supported only on first ORB_MULTI_MAX_INSTANCES mavlink channels */
	if (_mavlink->get_channel() < (mavlink_channel_t)ORB_MULTI_MAX_INSTANCES) {
		mavlink_radio_status_t rstatus;
		mavlink_msg_radio_status_decode(msg, &rstatus);

		struct telemetry_status_s &tstatus = _mavlink->get_rx_status();

		tstatus.timestamp = hrt_absolute_time();
		tstatus.telem_time = tstatus.timestamp;
		/* tstatus.heartbeat_time is set by system heartbeats */
		tstatus.type = telemetry_status_s::TELEMETRY_STATUS_RADIO_TYPE_3DR_RADIO;
		tstatus.rssi = rstatus.rssi;
		tstatus.remote_rssi = rstatus.remrssi;
		tstatus.txbuf = rstatus.txbuf;
		tstatus.noise = rstatus.noise;
		tstatus.remote_noise = rstatus.remnoise;
		tstatus.rxerrors = rstatus.rxerrors;
		tstatus.fixed = rstatus.fixed;
		tstatus.system_id = msg->sysid;
		tstatus.component_id = msg->compid;

		if (_telemetry_status_pub == nullptr) {
			int multi_instance;
			_telemetry_status_pub = orb_advertise_multi(ORB_ID(telemetry_status), &tstatus, &multi_instance, ORB_PRIO_HIGH);

		} else {
			orb_publish(ORB_ID(telemetry_status), _telemetry_status_pub, &tstatus);
		}
	}
}

void
MavlinkReceiver::handle_message_battery_status(mavlink_message_t *msg)
{
	// external battery measurements
	mavlink_battery_status_t battery_mavlink;
	mavlink_msg_battery_status_decode(msg, &battery_mavlink);

	battery_status_s battery_status = {};
	battery_status.timestamp = hrt_absolute_time();

	float voltage_sum = 0.0f;
	uint8_t cell_count = 0;

	while (battery_mavlink.voltages[cell_count] < UINT16_MAX && cell_count < 10) {
		voltage_sum += (float)(battery_mavlink.voltages[cell_count]) / 1000.0f;
		cell_count++;
	}

	battery_status.voltage_v = voltage_sum;
	battery_status.voltage_filtered_v  = voltage_sum;
	battery_status.current_a = battery_status.current_filtered_a = (float)(battery_mavlink.current_battery) / 100.0f;
	battery_status.current_filtered_a = battery_status.current_a;
	battery_status.remaining = (float)battery_mavlink.battery_remaining / 100.0f;
	battery_status.discharged_mah = (float)battery_mavlink.current_consumed;
	battery_status.cell_count = cell_count;
	battery_status.connected = true;

	// Get the battery level thresholds.
	float bat_emergen_thr;
	float bat_crit_thr;
	float bat_low_thr;
	param_get(_p_bat_emergen_thr, &bat_emergen_thr);
	param_get(_p_bat_crit_thr, &bat_crit_thr);
	param_get(_p_bat_low_thr, &bat_low_thr);

	// Set the battery warning based on remaining charge.
	//  Note: Smallest values must come first in evaluation.
	if (battery_status.remaining < bat_emergen_thr) {
		battery_status.warning = battery_status_s::BATTERY_WARNING_EMERGENCY;

	} else if (battery_status.remaining < bat_crit_thr) {
		battery_status.warning = battery_status_s::BATTERY_WARNING_CRITICAL;

	} else if (battery_status.remaining < bat_low_thr) {
		battery_status.warning = battery_status_s::BATTERY_WARNING_LOW;
	}

	if (_battery_pub == nullptr) {
		_battery_pub = orb_advertise(ORB_ID(battery_status), &battery_status);

	} else {
		orb_publish(ORB_ID(battery_status), _battery_pub, &battery_status);
	}
}

void
MavlinkReceiver::handle_message_serial_control(mavlink_message_t *msg)
{
	mavlink_serial_control_t serial_control_mavlink;
	mavlink_msg_serial_control_decode(msg, &serial_control_mavlink);

	// we only support shell commands
	if (serial_control_mavlink.device != SERIAL_CONTROL_DEV_SHELL
	    || (serial_control_mavlink.flags & SERIAL_CONTROL_FLAG_REPLY)) {
		return;
	}

	MavlinkShell *shell = _mavlink->get_shell();

	if (shell) {
		// we ignore the timeout, EXCLUSIVE & BLOCKING flags of the SERIAL_CONTROL message
		if (serial_control_mavlink.count > 0) {
			shell->write(serial_control_mavlink.data, serial_control_mavlink.count);
		}

		// if no response requested, assume the shell is no longer used
		if ((serial_control_mavlink.flags & SERIAL_CONTROL_FLAG_RESPOND) == 0) {
			_mavlink->close_shell();
		}
	}
}

void
MavlinkReceiver::handle_message_logging_ack(mavlink_message_t *msg)
{
	mavlink_logging_ack_t logging_ack;
	mavlink_msg_logging_ack_decode(msg, &logging_ack);

	MavlinkULog *ulog_streaming = _mavlink->get_ulog_streaming();

	if (ulog_streaming) {
		ulog_streaming->handle_ack(logging_ack);
	}
}

void
MavlinkReceiver::handle_message_play_tune(mavlink_message_t *msg)
{
	mavlink_play_tune_t play_tune;
	mavlink_msg_play_tune_decode(msg, &play_tune);

	char *tune = play_tune.tune;

	if ((mavlink_system.sysid == play_tune.target_system ||
	     play_tune.target_system == 0) &&
	    (mavlink_system.compid == play_tune.target_component ||
	     play_tune.target_component == 0)) {

		if (*tune == 'M') {
			int fd = px4_open(TONEALARM0_DEVICE_PATH, PX4_F_WRONLY);

			if (fd >= 0) {
				px4_write(fd, tune, strlen(tune) + 1);
				px4_close(fd);
			}
		}
	}
}

void
MavlinkReceiver::handle_message_heartbeat(mavlink_message_t *msg)
{
	/* telemetry status supported only on first TELEMETRY_STATUS_ORB_ID_NUM mavlink channels */
	if (_mavlink->get_channel() < (mavlink_channel_t)ORB_MULTI_MAX_INSTANCES) {
		mavlink_heartbeat_t hb;
		mavlink_msg_heartbeat_decode(msg, &hb);

		/* ignore own heartbeats, accept only heartbeats from GCS */
		if (msg->sysid != mavlink_system.sysid && hb.type == MAV_TYPE_GCS) {

			struct telemetry_status_s &tstatus = _mavlink->get_rx_status();

			/* set heartbeat time and topic time and publish -
			 * the telem status also gets updated on telemetry events
			 */
			tstatus.timestamp = hrt_absolute_time();
			tstatus.heartbeat_time = tstatus.timestamp;

			if (_telemetry_status_pub == nullptr) {
				int multi_instance;
				_telemetry_status_pub = orb_advertise_multi(ORB_ID(telemetry_status), &tstatus, &multi_instance, ORB_PRIO_HIGH);

			} else {
				orb_publish(ORB_ID(telemetry_status), _telemetry_status_pub, &tstatus);
			}
		}
	}
}

void
MavlinkReceiver::handle_message_ping(mavlink_message_t *msg)
{
	mavlink_ping_t ping;
	mavlink_msg_ping_decode(msg, &ping);

	if ((mavlink_system.sysid == ping.target_system) &&
	    (mavlink_system.compid == ping.target_component)) {
		mavlink_msg_ping_send_struct(_mavlink->get_channel(), &ping);
	}
}

void
MavlinkReceiver::handle_message_system_time(mavlink_message_t *msg)
{
	mavlink_system_time_t time;
	mavlink_msg_system_time_decode(msg, &time);

	timespec tv = {};
	px4_clock_gettime(CLOCK_REALTIME, &tv);

	// date -d @1234567890: Sat Feb 14 02:31:30 MSK 2009
	bool onb_unix_valid = (unsigned long long)tv.tv_sec > PX4_EPOCH_SECS;
	bool ofb_unix_valid = time.time_unix_usec > PX4_EPOCH_SECS * 1000ULL;

	if (!onb_unix_valid && ofb_unix_valid) {
		tv.tv_sec = time.time_unix_usec / 1000000ULL;
		tv.tv_nsec = (time.time_unix_usec % 1000000ULL) * 1000ULL;

		if (px4_clock_settime(CLOCK_REALTIME, &tv)) {
			warn("failed setting clock");

		} else {
			warnx("[timesync] UTC time synced.");
		}
	}

}

void
MavlinkReceiver::handle_message_hil_sensor(mavlink_message_t *msg)
{
	mavlink_hil_sensor_t imu;
	mavlink_msg_hil_sensor_decode(msg, &imu);

	uint64_t timestamp = hrt_absolute_time();

	/* airspeed */
	{
		struct airspeed_s airspeed = {};

		float ias = calc_indicated_airspeed(imu.diff_pressure * 1e2f);
		float tas = calc_true_airspeed_from_indicated(ias, imu.abs_pressure * 100, imu.temperature);

		airspeed.timestamp = timestamp;
		airspeed.indicated_airspeed_m_s = ias;
		airspeed.true_airspeed_m_s = tas;

		if (_airspeed_pub == nullptr) {
			_airspeed_pub = orb_advertise(ORB_ID(airspeed), &airspeed);

		} else {
			orb_publish(ORB_ID(airspeed), _airspeed_pub, &airspeed);
		}
	}

	/* gyro */
	{
		struct gyro_report gyro = {};

		gyro.timestamp = timestamp;
		gyro.x_raw = imu.xgyro * 1000.0f;
		gyro.y_raw = imu.ygyro * 1000.0f;
		gyro.z_raw = imu.zgyro * 1000.0f;
		gyro.x = imu.xgyro;
		gyro.y = imu.ygyro;
		gyro.z = imu.zgyro;
		gyro.temperature = imu.temperature;

		if (_gyro_pub == nullptr) {
			_gyro_pub = orb_advertise(ORB_ID(sensor_gyro), &gyro);

		} else {
			orb_publish(ORB_ID(sensor_gyro), _gyro_pub, &gyro);
		}
	}

	/* accelerometer */
	{
		struct accel_report accel = {};

		accel.timestamp = timestamp;
		accel.x_raw = imu.xacc / mg2ms2;
		accel.y_raw = imu.yacc / mg2ms2;
		accel.z_raw = imu.zacc / mg2ms2;
		accel.x = imu.xacc;
		accel.y = imu.yacc;
		accel.z = imu.zacc;
		accel.temperature = imu.temperature;

		if (_accel_pub == nullptr) {
			_accel_pub = orb_advertise(ORB_ID(sensor_accel), &accel);

		} else {
			orb_publish(ORB_ID(sensor_accel), _accel_pub, &accel);
		}
	}

	/* magnetometer */
	{
		struct mag_report mag = {};

		mag.timestamp = timestamp;
		mag.x_raw = imu.xmag * 1000.0f;
		mag.y_raw = imu.ymag * 1000.0f;
		mag.z_raw = imu.zmag * 1000.0f;
		mag.x = imu.xmag;
		mag.y = imu.ymag;
		mag.z = imu.zmag;

		if (_mag_pub == nullptr) {
			/* publish to the first mag topic */
			_mag_pub = orb_advertise(ORB_ID(sensor_mag), &mag);

		} else {
			orb_publish(ORB_ID(sensor_mag), _mag_pub, &mag);
		}
	}

	/* baro */
	{
		struct baro_report baro = {};

		baro.timestamp = timestamp;
		baro.pressure = imu.abs_pressure;
		baro.altitude = imu.pressure_alt;
		baro.temperature = imu.temperature;

		/* fake device ID */
		baro.device_id = 1234567;

		if (_baro_pub == nullptr) {
			_baro_pub = orb_advertise(ORB_ID(sensor_baro), &baro);

		} else {
			orb_publish(ORB_ID(sensor_baro), _baro_pub, &baro);
		}
	}

	/* battery status */
	{
		struct battery_status_s hil_battery_status = {};

		hil_battery_status.timestamp = timestamp;
		hil_battery_status.voltage_v = 11.5f;
		hil_battery_status.voltage_filtered_v = 11.5f;
		hil_battery_status.current_a = 10.0f;
		hil_battery_status.discharged_mah = -1.0f;

		if (_battery_pub == nullptr) {
			_battery_pub = orb_advertise(ORB_ID(battery_status), &hil_battery_status);

		} else {
			orb_publish(ORB_ID(battery_status), _battery_pub, &hil_battery_status);
		}
	}

	/* increment counters */
	_hil_frames++;

	/* print HIL sensors rate */
	if ((timestamp - _old_timestamp) > 10000000) {
		// printf("receiving HIL sensors at %d hz\n", _hil_frames / 10);
		_old_timestamp = timestamp;
		_hil_frames = 0;
	}
}

void
MavlinkReceiver::handle_message_hil_gps(mavlink_message_t *msg)
{
	mavlink_hil_gps_t gps;
	mavlink_msg_hil_gps_decode(msg, &gps);

	uint64_t timestamp = hrt_absolute_time();

	struct vehicle_gps_position_s hil_gps = {};

	hil_gps.timestamp_time_relative = 0;
	hil_gps.time_utc_usec = gps.time_usec;

	hil_gps.timestamp = timestamp;
	hil_gps.lat = gps.lat;
	hil_gps.lon = gps.lon;
	hil_gps.alt = gps.alt;
	hil_gps.eph = (float)gps.eph * 1e-2f; // from cm to m
	hil_gps.epv = (float)gps.epv * 1e-2f; // from cm to m

	hil_gps.s_variance_m_s = 1.0f;

	hil_gps.vel_m_s = (float)gps.vel * 1e-2f; // from cm/s to m/s
	hil_gps.vel_n_m_s = gps.vn * 1e-2f; // from cm to m
	hil_gps.vel_e_m_s = gps.ve * 1e-2f; // from cm to m
	hil_gps.vel_d_m_s = gps.vd * 1e-2f; // from cm to m
	hil_gps.vel_ned_valid = true;
	hil_gps.cog_rad = _wrap_pi(gps.cog * M_DEG_TO_RAD_F * 1e-2f);

	hil_gps.fix_type = gps.fix_type;
	hil_gps.satellites_used = gps.satellites_visible;  //TODO: rename mavlink_hil_gps_t sats visible to used?

	if (_gps_pub == nullptr) {
		_gps_pub = orb_advertise(ORB_ID(vehicle_gps_position), &hil_gps);

	} else {
		orb_publish(ORB_ID(vehicle_gps_position), _gps_pub, &hil_gps);
	}
}

void MavlinkReceiver::handle_message_gps_rtcm_data(mavlink_message_t *msg)
{
	mavlink_gps_rtcm_data_t gps_rtcm_data_msg = {};
	mavlink_msg_gps_rtcm_data_decode(msg, &gps_rtcm_data_msg);

	gps_inject_data_s gps_inject_data_topic = {};
	gps_inject_data_topic.len = math::min((int)sizeof(gps_rtcm_data_msg.data),
					      (int)sizeof(uint8_t) * gps_rtcm_data_msg.len);
	gps_inject_data_topic.flags = gps_rtcm_data_msg.flags;
	memcpy(gps_inject_data_topic.data, gps_rtcm_data_msg.data,
	       math::min((int)sizeof(gps_inject_data_topic.data), (int)sizeof(uint8_t) * gps_inject_data_topic.len));

	orb_advert_t &pub = _gps_inject_data_pub;

	if (pub == nullptr) {
		pub = orb_advertise_queue(ORB_ID(gps_inject_data), &gps_inject_data_topic,
					  _gps_inject_data_queue_size);

	} else {
		orb_publish(ORB_ID(gps_inject_data), pub, &gps_inject_data_topic);
	}

}

int
MavlinkReceiver::set_message_interval(int msgId, float interval, int data_rate)
{
	if (msgId == 0) {
		return PX4_ERROR;
	}

	if (data_rate > 0) {
		_mavlink->set_data_rate(data_rate);
	}

	// configure_stream wants a rate (msgs/second), so convert here.
	float rate = 0;

	if (interval < 0) {
		// stop the stream.
		rate = 0;

	} else if (interval > 0) {
		rate = 1000000.0f / interval;

	} else {
		// note: mavlink spec says rate == 0 is requesting a default rate but our streams
		// don't publish a default rate so for now let's pick a default rate of zero.
	}

	bool found_id = false;


	// The interval between two messages is in microseconds.
	// Set to -1 to disable and 0 to request default rate
	if (msgId != 0) {
		for (unsigned int i = 0; streams_list[i] != nullptr; i++) {
			const StreamListItem *item = streams_list[i];

			if (msgId == item->get_id()) {
				_mavlink->configure_stream_threadsafe(item->get_name(), rate);
				found_id = true;
				break;
			}
		}
	}

	return (found_id ? PX4_OK : PX4_ERROR);
}

void
MavlinkReceiver::get_message_interval(int msgId)
{
	unsigned interval = 0;

	MavlinkStream *stream = nullptr;
	LL_FOREACH(_mavlink->get_streams(), stream) {
		if (stream->get_id() == msgId) {
			interval = stream->get_interval();
			break;
		}
	}

	// send back this value...
	mavlink_msg_message_interval_send(_mavlink->get_channel(), msgId, interval);
}

/**
 * Receive data from UART.
 */
void *
MavlinkReceiver::receive_thread(void *arg)
{

	/* set thread name */
	{
		char thread_name[24];
		sprintf(thread_name, "mavlink_rcv_if%d", _mavlink->get_instance_id());
		px4_prctl(PR_SET_NAME, thread_name, px4_getpid());
	}

	// poll timeout in ms. Also defines the max update frequency of the mission & param manager, etc.
	const int timeout = 10;

#ifdef __PX4_POSIX
	/* 1500 is the Wifi MTU, so we make sure to fit a full packet */
	uint8_t buf[1600 * 5];
#else
	/* the serial port buffers internally as well, we just need to fit a small chunk */
	uint8_t buf[64];
#endif
	mavlink_message_t msg;

	struct pollfd fds[1] = {};

	if (_mavlink->get_protocol() == SERIAL) {
		fds[0].fd = _mavlink->get_uart_fd();
		fds[0].events = POLLIN;
	}

#ifdef __PX4_POSIX
	struct sockaddr_in srcaddr = {};
	socklen_t addrlen = sizeof(srcaddr);

	if (_mavlink->get_protocol() == UDP || _mavlink->get_protocol() == TCP) {
		// make sure mavlink app has booted before we start using the socket
		while (!_mavlink->boot_complete()) {
			usleep(100000);
		}

		fds[0].fd = _mavlink->get_socket_fd();
		fds[0].events = POLLIN;
	}

#endif
	ssize_t nread = 0;
	hrt_abstime last_send_update = 0;

	while (!_mavlink->_task_should_exit) {
		if (poll(&fds[0], 1, timeout) > 0) {
			if (_mavlink->get_protocol() == SERIAL) {

				/*
				 * to avoid reading very small chunks wait for data before reading
				 * this is designed to target one message, so >20 bytes at a time
				 */
				const unsigned character_count = 20;

				/* non-blocking read. read may return negative values */
				if ((nread = ::read(fds[0].fd, buf, sizeof(buf))) < (ssize_t)character_count) {
					unsigned sleeptime = (1.0f / (_mavlink->get_baudrate() / 10)) * character_count * 1000000;
					usleep(sleeptime);
				}
			}

#ifdef __PX4_POSIX

			if (_mavlink->get_protocol() == UDP) {
				if (fds[0].revents & POLLIN) {
					nread = recvfrom(_mavlink->get_socket_fd(), buf, sizeof(buf), 0, (struct sockaddr *)&srcaddr, &addrlen);
				}

			} else {
				// could be TCP or other protocol
			}

			struct sockaddr_in *srcaddr_last = _mavlink->get_client_source_address();

			int localhost = (127 << 24) + 1;

			if (!_mavlink->get_client_source_initialized()) {

				// set the address either if localhost or if 3 seconds have passed
				// this ensures that a GCS running on localhost can get a hold of
				// the system within the first N seconds
				hrt_abstime stime = _mavlink->get_start_time();

				if ((stime != 0 && (hrt_elapsed_time(&stime) > 3 * 1000 * 1000))
				    || (srcaddr_last->sin_addr.s_addr == htonl(localhost))) {
					srcaddr_last->sin_addr.s_addr = srcaddr.sin_addr.s_addr;
					srcaddr_last->sin_port = srcaddr.sin_port;
					_mavlink->set_client_source_initialized();
					PX4_INFO("partner IP: %s", inet_ntoa(srcaddr.sin_addr));
				}
			}

#endif
			// only start accepting messages once we're sure who we talk to

			if (_mavlink->get_client_source_initialized()) {
				/* if read failed, this loop won't execute */
				for (ssize_t i = 0; i < nread; i++) {
					if (mavlink_parse_char(_mavlink->get_channel(), buf[i], &msg, &_status)) {

						/* check if we received version 2 and request a switch. */
						if (!(_mavlink->get_status()->flags & MAVLINK_STATUS_FLAG_IN_MAVLINK1)) {
							/* this will only switch to proto version 2 if allowed in settings */
							_mavlink->set_proto_version(2);
						}

						/* handle generic messages and commands */
						handle_message(&msg);

						/* handle packet with parameter component */
						_parameters_manager.handle_message(&msg);

						/* handle packet with ftp component */
						_mavlink_ftp.handle_message(&msg);

						/* handle packet with log component */
						_mavlink_log_handler.handle_message(&msg);

						/* handle packet with parent object */
						_mavlink->handle_message(&msg);
					}
				}

				/* count received bytes (nread will be -1 on read error) */
				if (nread > 0) {
					_mavlink->count_rxbytes(nread);
				}
			}
		}

		hrt_abstime t = hrt_absolute_time();

		if (t - last_send_update > timeout * 1000) {
			_parameters_manager.send(t);
			_mavlink_ftp.send(t);
			_mavlink_log_handler.send(t);
			last_send_update = t;
		}

	}

	return nullptr;
}

void MavlinkReceiver::print_status()
{

}

void *MavlinkReceiver::start_helper(void *context)
{

	MavlinkReceiver *rcv = new MavlinkReceiver((Mavlink *)context);

	if (!rcv) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	void *ret = rcv->receive_thread(nullptr);

	delete rcv;

	return ret;
}

void
MavlinkReceiver::receive_start(pthread_t *thread, Mavlink *parent)
{
	pthread_attr_t receiveloop_attr;
	pthread_attr_init(&receiveloop_attr);

	struct sched_param param;
	(void)pthread_attr_getschedparam(&receiveloop_attr, &param);
	param.sched_priority = SCHED_PRIORITY_MAX - 80;
	(void)pthread_attr_setschedparam(&receiveloop_attr, &param);

	pthread_attr_setstacksize(&receiveloop_attr, PX4_STACK_ADJUSTED(2140));
	pthread_create(thread, &receiveloop_attr, MavlinkReceiver::start_helper, (void *)parent);

	pthread_attr_destroy(&receiveloop_attr);
}
