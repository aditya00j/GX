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
 * @file mavlink_messages.cpp
 * MAVLink 2.0 message formatters implementation.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <stdio.h>
#include <errno.h>

#include "mavlink_main.h"
#include "mavlink_messages.h"
#include "mavlink_command_sender.h"

//#include <commander/px4_custom_mode.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_rc_input.h>
#include <lib/geo/geo.h>
#include <mathlib/mathlib.h>
#include <px4_time.h>
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>

#include <uORB/uORB.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/mavlink_log.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_status.h>

static uint16_t cm_uint16_from_m_float(float m);
static void get_mavlink_mode_state(struct vehicle_status_s *status, uint8_t *mavlink_state,
				   uint8_t *mavlink_base_mode, uint32_t *mavlink_custom_mode);

uint16_t
cm_uint16_from_m_float(float m)
{
	if (m < 0.0f) {
		return 0;

	} else if (m > 655.35f) {
		return 65535;
	}

	return (uint16_t)(m * 100.0f);
}

void get_mavlink_mode_state(struct vehicle_status_s *status, uint8_t *mavlink_state,
			    uint8_t *mavlink_base_mode, uint32_t *mavlink_custom_mode)
{
	*mavlink_state = 0;
	*mavlink_base_mode = 0;
	*mavlink_custom_mode = 0;

	/* HIL */
	if (status->hil_state == vehicle_status_s::HIL_STATE_ON) {
		*mavlink_base_mode |= MAV_MODE_FLAG_HIL_ENABLED;
	}

	/* arming state */
	if (status->arming_state == vehicle_status_s::ARMING_STATE_ARMED
	    || status->arming_state == vehicle_status_s::ARMING_STATE_ARMED_ERROR) {
		*mavlink_base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
	}

	/* main state */
	*mavlink_base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

	union px4_custom_mode custom_mode;
	custom_mode.data = 0;

	const uint8_t auto_mode_flags	= MAV_MODE_FLAG_AUTO_ENABLED
					  | MAV_MODE_FLAG_STABILIZE_ENABLED
					  | MAV_MODE_FLAG_GUIDED_ENABLED;

	switch (status->nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_MANUAL:
		*mavlink_base_mode	|= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
					   | (status->is_rotary_wing ? MAV_MODE_FLAG_STABILIZE_ENABLED : 0);
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_MANUAL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ACRO:
		*mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_ACRO;
		break;

	case vehicle_status_s::NAVIGATION_STATE_RATTITUDE:
		*mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_RATTITUDE;
		break;

	case vehicle_status_s::NAVIGATION_STATE_STAB:
		*mavlink_base_mode	|= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
					   | MAV_MODE_FLAG_STABILIZE_ENABLED;
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_STABILIZED;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
		*mavlink_base_mode	|= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
					   | MAV_MODE_FLAG_STABILIZE_ENABLED;
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_ALTCTL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_POSCTL:
		*mavlink_base_mode	|= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
					   | MAV_MODE_FLAG_STABILIZE_ENABLED
					   | MAV_MODE_FLAG_GUIDED_ENABLED; // TODO: is POSCTL GUIDED?
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_POSCTL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_MISSION;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_LOITER;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:

	/* fallthrough */
	case vehicle_status_s::NAVIGATION_STATE_AUTO_RCRECOVER:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_RTL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDGPSFAIL:

	/* fallthrough */
	case vehicle_status_s::NAVIGATION_STATE_DESCEND:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_LAND;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTGS:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_RTGS;
		break;

	case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
		*mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_MANUAL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_OFFBOARD;
		break;

	case vehicle_status_s::NAVIGATION_STATE_MAX:
		/* this is an unused case, ignore */
		break;

	}

	*mavlink_custom_mode = custom_mode.data;

	/* set system state */
	if (status->arming_state == vehicle_status_s::ARMING_STATE_INIT
	    || status->arming_state == vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE
	    || status->arming_state == vehicle_status_s::ARMING_STATE_STANDBY_ERROR) {	// TODO review
		*mavlink_state = MAV_STATE_UNINIT;

	} else if (status->arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		*mavlink_state = MAV_STATE_ACTIVE;

	} else if (status->arming_state == vehicle_status_s::ARMING_STATE_ARMED_ERROR) {
		*mavlink_state = MAV_STATE_CRITICAL;

	} else if (status->arming_state == vehicle_status_s::ARMING_STATE_STANDBY) {
		*mavlink_state = MAV_STATE_STANDBY;

	} else if (status->arming_state == vehicle_status_s::ARMING_STATE_REBOOT) {
		*mavlink_state = MAV_STATE_POWEROFF;

	} else {
		*mavlink_state = MAV_STATE_CRITICAL;
	}
}


class MavlinkStreamHeartbeat : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamHeartbeat::get_name_static();
	}

	static const char *get_name_static()
	{
		return "HEARTBEAT";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_HEARTBEAT;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamHeartbeat(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_HEARTBEAT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

	bool const_rate()
	{
		return true;
	}

private:
	MavlinkOrbSubscription *_status_sub;

	/* do not allow top copying this class */
	MavlinkStreamHeartbeat(MavlinkStreamHeartbeat &);
	MavlinkStreamHeartbeat &operator = (const MavlinkStreamHeartbeat &);

protected:
	explicit MavlinkStreamHeartbeat(Mavlink *mavlink) : MavlinkStream(mavlink),
		_status_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_status)))
	{}

	void send(const hrt_abstime t)
	{
		struct vehicle_status_s status = {};

		/* always send the heartbeat, independent of the update status of the topics */
		if (!_status_sub->update(&status)) {
			/* if topic update failed fill it with defaults */
			memset(&status, 0, sizeof(status));
		}

		uint8_t base_mode = 0;
		uint32_t custom_mode = 0;
		uint8_t system_status = 0;
		get_mavlink_mode_state(&status, &system_status, &base_mode, &custom_mode);

		mavlink_msg_heartbeat_send(_mavlink->get_channel(), _mavlink->get_system_type(), MAV_AUTOPILOT_PX4,
					   base_mode, custom_mode, system_status);
	}
};

class MavlinkStreamStatustext : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamStatustext::get_name_static();
	}

	static const char *get_name_static()
	{
		return "STATUSTEXT";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_STATUSTEXT;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamStatustext(mavlink);
	}

	unsigned get_size()
	{
		return _mavlink->get_logbuffer()->empty() ? 0 : (MAVLINK_MSG_ID_STATUSTEXT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES);
	}

private:
	/* do not allow top copying this class */
	MavlinkStreamStatustext(MavlinkStreamStatustext &);
	MavlinkStreamStatustext &operator = (const MavlinkStreamStatustext &);

protected:
	explicit MavlinkStreamStatustext(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	~MavlinkStreamStatustext() {}


	void send(const hrt_abstime t)
	{
		if (!_mavlink->get_logbuffer()->empty() && _mavlink->is_connected()) {

			struct mavlink_log_s mavlink_log = {};

			if (_mavlink->get_logbuffer()->get(&mavlink_log)) {

				mavlink_statustext_t msg;
				msg.severity = mavlink_log.severity;
				strncpy(msg.text, (const char *)mavlink_log.text, sizeof(msg.text));
				msg.text[sizeof(msg.text) - 1] = '\0';

				mavlink_msg_statustext_send_struct(_mavlink->get_channel(), &msg);
			}
		}
	}
};

class MavlinkStreamCommandLong : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamCommandLong::get_name_static();
	}

	static const char *get_name_static()
	{
		return "COMMAND_LONG";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_COMMAND_LONG;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamCommandLong(mavlink);
	}

	unsigned get_size()
	{
		return 0;	// commands stream is not regular and not predictable
	}

private:
	MavlinkOrbSubscription *_cmd_sub;
	uint64_t _cmd_time;

	/* do not allow top copying this class */
	MavlinkStreamCommandLong(MavlinkStreamCommandLong &);
	MavlinkStreamCommandLong &operator = (const MavlinkStreamCommandLong &);

protected:
	explicit MavlinkStreamCommandLong(Mavlink *mavlink) : MavlinkStream(mavlink),
		_cmd_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_command))),
		_cmd_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct vehicle_command_s cmd;

		if (_cmd_sub->update(&_cmd_time, &cmd)) {

			/* only send commands for other systems/components, don't forward broadcast commands */
			if ((cmd.target_system != mavlink_system.sysid || cmd.target_component != mavlink_system.compid) &&
			    (cmd.target_system != 0)) {

				if (_mavlink->verbose()) {
					PX4_INFO("sending command %d to %d/%d", cmd.command, cmd.target_system, cmd.target_component);
				}

				MavlinkCommandSender::instance().handle_vehicle_command(cmd, _mavlink->get_channel());

			} else {
				if (_mavlink->verbose()) {
					PX4_INFO("not forwarding command %d to %d/%d", cmd.command, cmd.target_system, cmd.target_component);
				}
			}
		}

		MavlinkCommandSender::instance().check_timeout(_mavlink->get_channel());
	}
};

class MavlinkStreamSysStatus : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamSysStatus::get_name_static();
	}

	static const char *get_name_static()
	{
		return "SYS_STATUS";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_SYS_STATUS;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamSysStatus(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_SYS_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_status_sub;
	MavlinkOrbSubscription *_cpuload_sub;
	MavlinkOrbSubscription *_battery_status_sub;

	/* do not allow top copying this class */
	MavlinkStreamSysStatus(MavlinkStreamSysStatus &);
	MavlinkStreamSysStatus &operator = (const MavlinkStreamSysStatus &);

protected:
	explicit MavlinkStreamSysStatus(Mavlink *mavlink) : MavlinkStream(mavlink),
		_status_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_status))),
		_cpuload_sub(_mavlink->add_orb_subscription(ORB_ID(cpuload))),
		_battery_status_sub(_mavlink->add_orb_subscription(ORB_ID(battery_status)))
	{}

	void send(const hrt_abstime t)
	{
		struct vehicle_status_s status = {};
		struct cpuload_s cpuload = {};
		struct battery_status_s battery_status = {};

		const bool updated_status = _status_sub->update(&status);
		const bool updated_cpuload = _cpuload_sub->update(&cpuload);
		const bool updated_battery = _battery_status_sub->update(&battery_status);

		if (updated_status || updated_battery || updated_cpuload) {
			mavlink_sys_status_t msg = {};

			msg.onboard_control_sensors_present = status.onboard_control_sensors_present;
			msg.onboard_control_sensors_enabled = status.onboard_control_sensors_enabled;
			msg.onboard_control_sensors_health = status.onboard_control_sensors_health;
			msg.load = cpuload.load * 1000.0f;
			msg.voltage_battery = (battery_status.connected) ? battery_status.voltage_filtered_v * 1000.0f : UINT16_MAX;
			msg.current_battery = (battery_status.connected) ? battery_status.current_filtered_a * 100.0f : -1;
			msg.battery_remaining = (battery_status.connected) ? battery_status.remaining * 100.0f : -1;
			// TODO: fill in something useful in the fields below
			msg.drop_rate_comm = 0;
			msg.errors_comm = 0;
			msg.errors_count1 = 0;
			msg.errors_count2 = 0;
			msg.errors_count3 = 0;
			msg.errors_count4 = 0;

			mavlink_msg_sys_status_send_struct(_mavlink->get_channel(), &msg);

			/* battery status message with higher resolution */
			mavlink_battery_status_t bat_msg = {};
			bat_msg.id = 0;
			bat_msg.battery_function = MAV_BATTERY_FUNCTION_ALL;
			bat_msg.type = MAV_BATTERY_TYPE_LIPO;
			bat_msg.current_consumed = (battery_status.connected) ? battery_status.discharged_mah : -1;
			bat_msg.energy_consumed = -1;
			bat_msg.current_battery = (battery_status.connected) ? battery_status.current_filtered_a * 100 : -1;
			bat_msg.battery_remaining = (battery_status.connected) ? battery_status.remaining * 100.0f : -1;
			bat_msg.temperature = INT16_MAX;

			for (unsigned int i = 0; i < (sizeof(bat_msg.voltages) / sizeof(bat_msg.voltages[0])); i++) {
				if ((int)i < battery_status.cell_count && battery_status.connected) {
					bat_msg.voltages[i] = (battery_status.voltage_v / battery_status.cell_count) * 1000.0f;

				} else {
					bat_msg.voltages[i] = UINT16_MAX;
				}
			}

			mavlink_msg_battery_status_send_struct(_mavlink->get_channel(), &bat_msg);
		}
	}
};


class MavlinkStreamHighresIMU : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamHighresIMU::get_name_static();
	}

	static const char *get_name_static()
	{
		return "HIGHRES_IMU";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_HIGHRES_IMU;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamHighresIMU(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_HIGHRES_IMU_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_sensor_sub;
	uint64_t _sensor_time;

	MavlinkOrbSubscription *_differential_pressure_sub;
	uint64_t _differential_pressure_time;

	uint64_t _accel_timestamp;
	uint64_t _gyro_timestamp;
	uint64_t _mag_timestamp;
	uint64_t _baro_timestamp;

	/* do not allow top copying this class */
	MavlinkStreamHighresIMU(MavlinkStreamHighresIMU &);
	MavlinkStreamHighresIMU &operator = (const MavlinkStreamHighresIMU &);

protected:
	explicit MavlinkStreamHighresIMU(Mavlink *mavlink) : MavlinkStream(mavlink),
		_sensor_sub(_mavlink->add_orb_subscription(ORB_ID(sensor_combined))),
		_sensor_time(0),
		_differential_pressure_sub(_mavlink->add_orb_subscription(ORB_ID(differential_pressure))),
		_differential_pressure_time(0),
		_accel_timestamp(0),
		_gyro_timestamp(0),
		_mag_timestamp(0),
		_baro_timestamp(0)
	{}

	void send(const hrt_abstime t)
	{
		struct sensor_combined_s sensor = {};
		struct differential_pressure_s differential_pressure = {};

		if (_sensor_sub->update(&_sensor_time, &sensor)) {
			uint16_t fields_updated = 0;

			if (_accel_timestamp != sensor.timestamp + sensor.accelerometer_timestamp_relative) {
				/* mark first three dimensions as changed */
				fields_updated |= (1 << 0) | (1 << 1) | (1 << 2);
				_accel_timestamp = sensor.timestamp + sensor.accelerometer_timestamp_relative;
			}

			if (_gyro_timestamp != sensor.timestamp) {
				/* mark second group dimensions as changed */
				fields_updated |= (1 << 3) | (1 << 4) | (1 << 5);
				_gyro_timestamp = sensor.timestamp;
			}

			if (_mag_timestamp != sensor.timestamp + sensor.magnetometer_timestamp_relative) {
				/* mark third group dimensions as changed */
				fields_updated |= (1 << 6) | (1 << 7) | (1 << 8);
				_mag_timestamp = sensor.timestamp + sensor.magnetometer_timestamp_relative;
			}

			if (_baro_timestamp != sensor.timestamp + sensor.baro_timestamp_relative) {
				/* mark last group dimensions as changed */
				fields_updated |= (1 << 9) | (1 << 11) | (1 << 12);
				_baro_timestamp = sensor.timestamp + sensor.baro_timestamp_relative;
			}

			_differential_pressure_sub->update(&_differential_pressure_time, &differential_pressure);

			mavlink_highres_imu_t msg = {};

			msg.time_usec = sensor.timestamp;
			msg.xacc = sensor.accelerometer_m_s2[0];
			msg.yacc = sensor.accelerometer_m_s2[1];
			msg.zacc = sensor.accelerometer_m_s2[2];
			msg.xgyro = sensor.gyro_rad[0];
			msg.ygyro = sensor.gyro_rad[1];
			msg.zgyro = sensor.gyro_rad[2];
			msg.xmag = sensor.magnetometer_ga[0];
			msg.ymag = sensor.magnetometer_ga[1];
			msg.zmag = sensor.magnetometer_ga[2];
			msg.abs_pressure = 0;
			msg.diff_pressure = differential_pressure.differential_pressure_raw_pa;
			msg.pressure_alt = sensor.baro_alt_meter;
			msg.temperature = sensor.baro_temp_celcius;
			msg.fields_updated = fields_updated;

			mavlink_msg_highres_imu_send_struct(_mavlink->get_channel(), &msg);
		}
	}
};

class MavlinkStreamGPSRawInt : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamGPSRawInt::get_name_static();
	}

	static const char *get_name_static()
	{
		return "GPS_RAW_INT";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_GPS_RAW_INT;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamGPSRawInt(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_GPS_RAW_INT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_gps_sub;
	uint64_t _gps_time;

	/* do not allow top copying this class */
	MavlinkStreamGPSRawInt(MavlinkStreamGPSRawInt &);
	MavlinkStreamGPSRawInt &operator = (const MavlinkStreamGPSRawInt &);

protected:
	explicit MavlinkStreamGPSRawInt(Mavlink *mavlink) : MavlinkStream(mavlink),
		_gps_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_gps_position))),
		_gps_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct vehicle_gps_position_s gps;

		if (_gps_sub->update(&_gps_time, &gps)) {
			mavlink_gps_raw_int_t msg = {};

			msg.time_usec = gps.timestamp;
			msg.fix_type = gps.fix_type;
			msg.lat = gps.lat;
			msg.lon = gps.lon;
			msg.alt = gps.alt;
			msg.eph = gps.hdop * 100; //cm_uint16_from_m_float(gps.eph);
			msg.epv = gps.vdop * 100; //cm_uint16_from_m_float(gps.epv);
			msg.vel = cm_uint16_from_m_float(gps.vel_m_s),
			    msg.cog = _wrap_2pi(gps.cog_rad) * M_RAD_TO_DEG_F * 1e2f,
				msg.satellites_visible = gps.satellites_used;

			mavlink_msg_gps_raw_int_send_struct(_mavlink->get_channel(), &msg);
		}
	}
};

class MavlinkStreamSystemTime : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamSystemTime::get_name_static();
	}

	static const char *get_name_static()
	{
		return "SYSTEM_TIME";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_SYSTEM_TIME;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamSystemTime(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_SYSTEM_TIME_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	/* do not allow top copying this class */
	MavlinkStreamSystemTime(MavlinkStreamSystemTime &);
	MavlinkStreamSystemTime &operator = (const MavlinkStreamSystemTime &);

protected:
	explicit MavlinkStreamSystemTime(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	void send(const hrt_abstime t)
	{
		mavlink_system_time_t msg = {};
		timespec tv;

		px4_clock_gettime(CLOCK_REALTIME, &tv);

		msg.time_boot_ms = hrt_absolute_time() / 1000;
		msg.time_unix_usec = (uint64_t)tv.tv_sec * 1000000 + tv.tv_nsec / 1000;

		mavlink_msg_system_time_send_struct(_mavlink->get_channel(), &msg);
	}
};

template <int N>
class MavlinkStreamServoOutputRaw : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamServoOutputRaw<N>::get_name_static();
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_SERVO_OUTPUT_RAW;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static const char *get_name_static()
	{
		switch (N) {
		case 0:
			return "SERVO_OUTPUT_RAW_0";

		case 1:
			return "SERVO_OUTPUT_RAW_1";

		case 2:
			return "SERVO_OUTPUT_RAW_2";

		case 3:
			return "SERVO_OUTPUT_RAW_3";
		}
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamServoOutputRaw<N>(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_act_sub;
	uint64_t _act_time;

	/* do not allow top copying this class */
	MavlinkStreamServoOutputRaw(MavlinkStreamServoOutputRaw &);
	MavlinkStreamServoOutputRaw &operator = (const MavlinkStreamServoOutputRaw &);

protected:
	explicit MavlinkStreamServoOutputRaw(Mavlink *mavlink) : MavlinkStream(mavlink),
		_act_sub(nullptr),
		_act_time(0)
	{
		_act_sub = _mavlink->add_orb_subscription(ORB_ID(actuator_outputs), N);
	}

	void send(const hrt_abstime t)
	{
		struct actuator_outputs_s act;

		if (_act_sub->update(&_act_time, &act)) {
			mavlink_servo_output_raw_t msg = {};

			msg.time_usec = act.timestamp;
			msg.port = N;
			msg.servo1_raw = act.output[0];
			msg.servo2_raw = act.output[1];
			msg.servo3_raw = act.output[2];
			msg.servo4_raw = act.output[3];
			msg.servo5_raw = act.output[4];
			msg.servo6_raw = act.output[5];
			msg.servo7_raw = act.output[6];
			msg.servo8_raw = act.output[7];

			mavlink_msg_servo_output_raw_send_struct(_mavlink->get_channel(), &msg);
		}
	}
};

class MavlinkStreamHILActuatorControls : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamHILActuatorControls::get_name_static();
	}

	static const char *get_name_static()
	{
		return "HIL_ACTUATOR_CONTROLS";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamHILActuatorControls(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_status_sub;
	uint64_t _status_time;

	MavlinkOrbSubscription *_act_sub;
	uint64_t _act_time;

	/* do not allow top copying this class */
	MavlinkStreamHILActuatorControls(MavlinkStreamHILActuatorControls &);
	MavlinkStreamHILActuatorControls &operator = (const MavlinkStreamHILActuatorControls &);

protected:
	explicit MavlinkStreamHILActuatorControls(Mavlink *mavlink) : MavlinkStream(mavlink),
		_status_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_status))),
		_status_time(0),
		_act_sub(_mavlink->add_orb_subscription(ORB_ID(actuator_outputs))),
		_act_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct vehicle_status_s status;
		struct actuator_outputs_s act;

		bool updated = _act_sub->update(&_act_time, &act);
		updated |= _status_sub->update(&_status_time, &status);

		if (updated && (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED)) {
			/* translate the current syste state to mavlink state and mode */
			uint8_t mavlink_state;
			uint8_t mavlink_base_mode;
			uint32_t mavlink_custom_mode;
			mavlink_hil_actuator_controls_t msg = {};

			get_mavlink_mode_state(&status, &mavlink_state, &mavlink_base_mode, &mavlink_custom_mode);

			const float pwm_center = (PWM_DEFAULT_MAX + PWM_DEFAULT_MIN) / 2;

			unsigned system_type = _mavlink->get_system_type();

			/* scale outputs depending on system type */
			if (system_type == MAV_TYPE_QUADROTOR ||
			    system_type == MAV_TYPE_HEXAROTOR ||
			    system_type == MAV_TYPE_OCTOROTOR ||
			    system_type == MAV_TYPE_VTOL_DUOROTOR ||
			    system_type == MAV_TYPE_VTOL_QUADROTOR ||
			    system_type == MAV_TYPE_VTOL_RESERVED2) {

				/* multirotors: set number of rotor outputs depending on type */

				unsigned n;

				switch (system_type) {
				case MAV_TYPE_QUADROTOR:
					n = 4;
					break;

				case MAV_TYPE_HEXAROTOR:
					n = 6;
					break;

				case MAV_TYPE_VTOL_DUOROTOR:
					n = 2;
					break;

				case MAV_TYPE_VTOL_QUADROTOR:
					n = 4;
					break;

				case MAV_TYPE_VTOL_RESERVED2:
					n = 8;
					break;

				default:
					n = 8;
					break;
				}

				for (unsigned i = 0; i < 16; i++) {
					if (act.output[i] > PWM_DEFAULT_MIN / 2) {
						if (i < n) {
							/* scale PWM out 900..2100 us to 0..1 for rotors */
							msg.controls[i] = (act.output[i] - PWM_DEFAULT_MIN) / (PWM_DEFAULT_MAX - PWM_DEFAULT_MIN);

						} else {
							/* scale PWM out 900..2100 us to -1..1 for other channels */
							msg.controls[i] = (act.output[i] - pwm_center) / ((PWM_DEFAULT_MAX - PWM_DEFAULT_MIN) / 2);
						}

					} else {
						/* send 0 when disarmed and for disabled channels */
						msg.controls[i] = 0.0f;
					}
				}

			} else {
				/* fixed wing: scale throttle to 0..1 and other channels to -1..1 */

				for (unsigned i = 0; i < 16; i++) {
					if (act.output[i] > PWM_DEFAULT_MIN / 2) {
						if (i != 3) {
							/* scale PWM out 900..2100 us to -1..1 for normal channels */
							msg.controls[i] = (act.output[i] - pwm_center) / ((PWM_DEFAULT_MAX - PWM_DEFAULT_MIN) / 2);

						} else {
							/* scale PWM out 900..2100 us to 0..1 for throttle */
							msg.controls[i] = (act.output[i] - PWM_DEFAULT_MIN) / (PWM_DEFAULT_MAX - PWM_DEFAULT_MIN);
						}

					} else {
						/* set 0 for disabled channels */
						msg.controls[i] = 0.0f;
					}
				}
			}

			msg.time_usec = hrt_absolute_time();
			msg.mode = mavlink_base_mode;
			msg.flags = 0;

			mavlink_msg_hil_actuator_controls_send_struct(_mavlink->get_channel(), &msg);
		}
	}
};

class MavlinkStreamRCChannels : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamRCChannels::get_name_static();
	}

	static const char *get_name_static()
	{
		return "RC_CHANNELS";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_RC_CHANNELS;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamRCChannels(mavlink);
	}

	unsigned get_size()
	{
		return _rc_sub->is_published() ? (MAVLINK_MSG_ID_RC_CHANNELS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	MavlinkOrbSubscription *_rc_sub;
	uint64_t _rc_time;

	/* do not allow top copying this class */
	MavlinkStreamRCChannels(MavlinkStreamRCChannels &);
	MavlinkStreamRCChannels &operator = (const MavlinkStreamRCChannels &);

protected:
	explicit MavlinkStreamRCChannels(Mavlink *mavlink) : MavlinkStream(mavlink),
		_rc_sub(_mavlink->add_orb_subscription(ORB_ID(input_rc))),
		_rc_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct rc_input_values rc = {};

		if (_rc_sub->update(&_rc_time, &rc)) {

			/* send RC channel data and RSSI */
			mavlink_rc_channels_t msg = {};

			msg.time_boot_ms = rc.timestamp / 1000;
			msg.chancount = rc.channel_count;
			msg.chan1_raw = (rc.channel_count > 0) ? rc.values[0] : UINT16_MAX;
			msg.chan2_raw = (rc.channel_count > 1) ? rc.values[1] : UINT16_MAX;
			msg.chan3_raw = (rc.channel_count > 2) ? rc.values[2] : UINT16_MAX;
			msg.chan4_raw = (rc.channel_count > 3) ? rc.values[3] : UINT16_MAX;
			msg.chan5_raw = (rc.channel_count > 4) ? rc.values[4] : UINT16_MAX;
			msg.chan6_raw = (rc.channel_count > 5) ? rc.values[5] : UINT16_MAX;
			msg.chan7_raw = (rc.channel_count > 6) ? rc.values[6] : UINT16_MAX;
			msg.chan8_raw = (rc.channel_count > 7) ? rc.values[7] : UINT16_MAX;
			msg.chan9_raw = (rc.channel_count > 8) ? rc.values[8] : UINT16_MAX;
			msg.chan10_raw = (rc.channel_count > 9) ? rc.values[9] : UINT16_MAX;
			msg.chan11_raw = (rc.channel_count > 10) ? rc.values[10] : UINT16_MAX;
			msg.chan12_raw = (rc.channel_count > 11) ? rc.values[11] : UINT16_MAX;
			msg.chan13_raw = (rc.channel_count > 12) ? rc.values[12] : UINT16_MAX;
			msg.chan14_raw = (rc.channel_count > 13) ? rc.values[13] : UINT16_MAX;
			msg.chan15_raw = (rc.channel_count > 14) ? rc.values[14] : UINT16_MAX;
			msg.chan16_raw = (rc.channel_count > 15) ? rc.values[15] : UINT16_MAX;
			msg.chan17_raw = (rc.channel_count > 16) ? rc.values[16] : UINT16_MAX;
			msg.chan18_raw = (rc.channel_count > 17) ? rc.values[17] : UINT16_MAX;

			msg.rssi = (rc.channel_count > 0) ? rc.rssi : 0;

			mavlink_msg_rc_channels_send_struct(_mavlink->get_channel(), &msg);

			/* send override message - harmless if connected to GCS, allows to connect a board to a Linux system */
			/* http://mavlink.org/messages/common#RC_CHANNELS_OVERRIDE */
			mavlink_rc_channels_override_t over;
			over.target_system = mavlink_system.sysid;
			over.target_component = 0;
			over.chan1_raw = msg.chan1_raw;
			over.chan2_raw = msg.chan2_raw;
			over.chan3_raw = msg.chan3_raw;
			over.chan4_raw = msg.chan4_raw;
			over.chan5_raw = msg.chan5_raw;
			over.chan6_raw = msg.chan6_raw;
			over.chan7_raw = msg.chan7_raw;
			over.chan8_raw = msg.chan8_raw;

			mavlink_msg_rc_channels_override_send_struct(_mavlink->get_channel(), &over);
		}
	}
};

class MavlinkStreamOpticalFlowRad : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamOpticalFlowRad::get_name_static();
	}

	static const char *get_name_static()
	{
		return "OPTICAL_FLOW_RAD";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_OPTICAL_FLOW_RAD;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamOpticalFlowRad(mavlink);
	}

	unsigned get_size()
	{
		return _flow_sub->is_published() ? (MAVLINK_MSG_ID_OPTICAL_FLOW_RAD_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	MavlinkOrbSubscription *_flow_sub;
	uint64_t _flow_time;

	/* do not allow top copying this class */
	MavlinkStreamOpticalFlowRad(MavlinkStreamOpticalFlowRad &);
	MavlinkStreamOpticalFlowRad &operator = (const MavlinkStreamOpticalFlowRad &);

protected:
	explicit MavlinkStreamOpticalFlowRad(Mavlink *mavlink) : MavlinkStream(mavlink),
		_flow_sub(_mavlink->add_orb_subscription(ORB_ID(optical_flow))),
		_flow_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct optical_flow_s flow = {};

		if (_flow_sub->update(&_flow_time, &flow)) {
			mavlink_optical_flow_rad_t msg = {};

			msg.time_usec = flow.timestamp;
			msg.sensor_id = flow.sensor_id;
			msg.integrated_x = flow.pixel_flow_x_integral;
			msg.integrated_y = flow.pixel_flow_y_integral;
			msg.integrated_xgyro = flow.gyro_x_rate_integral;
			msg.integrated_ygyro = flow.gyro_y_rate_integral;
			msg.integrated_zgyro = flow.gyro_z_rate_integral;
			msg.distance = flow.ground_distance_m;
			msg.quality = flow.quality;
			msg.integration_time_us = flow.integration_timespan;
			msg.sensor_id = flow.sensor_id;
			msg.time_delta_distance_us = flow.time_since_last_sonar_update;
			msg.temperature = flow.gyro_temperature;

			mavlink_msg_optical_flow_rad_send_struct(_mavlink->get_channel(), &msg);
		}
	}
};

class MavlinkStreamNamedValueFloat : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamNamedValueFloat::get_name_static();
	}

	static const char *get_name_static()
	{
		return "NAMED_VALUE_FLOAT";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_NAMED_VALUE_FLOAT;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamNamedValueFloat(mavlink);
	}

	unsigned get_size()
	{
		return (_debug_time > 0) ? MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	MavlinkOrbSubscription *_debug_sub;
	uint64_t _debug_time;

	/* do not allow top copying this class */
	MavlinkStreamNamedValueFloat(MavlinkStreamNamedValueFloat &);
	MavlinkStreamNamedValueFloat &operator = (const MavlinkStreamNamedValueFloat &);

protected:
	explicit MavlinkStreamNamedValueFloat(Mavlink *mavlink) : MavlinkStream(mavlink),
		_debug_sub(_mavlink->add_orb_subscription(ORB_ID(debug_key_value))),
		_debug_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct debug_key_value_s debug = {};

		if (_debug_sub->update(&_debug_time, &debug)) {
			mavlink_named_value_float_t msg = {};

			msg.time_boot_ms = debug.timestamp_ms;
			memcpy(msg.name, debug.key, sizeof(msg.name));
			/* enforce null termination */
			msg.name[sizeof(msg.name) - 1] = '\0';
			msg.value = debug.value;

			mavlink_msg_named_value_float_send_struct(_mavlink->get_channel(), &msg);
		}
	}
};

class MavlinkStreamDistanceSensor : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamDistanceSensor::get_name_static();
	}

	static const char *get_name_static()
	{
		return "DISTANCE_SENSOR";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_DISTANCE_SENSOR;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamDistanceSensor(mavlink);
	}

	unsigned get_size()
	{
		return _distance_sensor_sub->is_published() ? (MAVLINK_MSG_ID_DISTANCE_SENSOR_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	MavlinkOrbSubscription *_distance_sensor_sub;
	uint64_t _dist_sensor_time;

	/* do not allow top copying this class */
	MavlinkStreamDistanceSensor(MavlinkStreamDistanceSensor &);
	MavlinkStreamDistanceSensor &operator = (const MavlinkStreamDistanceSensor &);

protected:
	explicit MavlinkStreamDistanceSensor(Mavlink *mavlink) : MavlinkStream(mavlink),
		_distance_sensor_sub(_mavlink->add_orb_subscription(ORB_ID(distance_sensor))),
		_dist_sensor_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct distance_sensor_s dist_sensor = {};

		if (_distance_sensor_sub->update(&_dist_sensor_time, &dist_sensor)) {

			mavlink_distance_sensor_t msg = {};

			msg.time_boot_ms = dist_sensor.timestamp / 1000; /* us to ms */

			/* TODO: use correct ID here */
			msg.id = 0;

			switch (dist_sensor.type) {
			case MAV_DISTANCE_SENSOR_ULTRASOUND:
				msg.type = MAV_DISTANCE_SENSOR_ULTRASOUND;
				break;

			case MAV_DISTANCE_SENSOR_LASER:
				msg.type = MAV_DISTANCE_SENSOR_LASER;
				break;

			case MAV_DISTANCE_SENSOR_INFRARED:
				msg.type = MAV_DISTANCE_SENSOR_INFRARED;
				break;

			default:
				msg.type = MAV_DISTANCE_SENSOR_LASER;
				break;
			}

			msg.orientation = dist_sensor.orientation;
			msg.min_distance = dist_sensor.min_distance * 100.0f; /* m to cm */
			msg.max_distance = dist_sensor.max_distance * 100.0f; /* m to cm */
			msg.current_distance = dist_sensor.current_distance * 100.0f; /* m to cm */
			msg.covariance = dist_sensor.covariance;

			mavlink_msg_distance_sensor_send_struct(_mavlink->get_channel(), &msg);
		}
	}
};

const StreamListItem *streams_list[] = {
	new StreamListItem(&MavlinkStreamHeartbeat::new_instance, &MavlinkStreamHeartbeat::get_name_static, &MavlinkStreamHeartbeat::get_id_static),
	new StreamListItem(&MavlinkStreamStatustext::new_instance, &MavlinkStreamStatustext::get_name_static, &MavlinkStreamStatustext::get_id_static),
	new StreamListItem(&MavlinkStreamCommandLong::new_instance, &MavlinkStreamCommandLong::get_name_static, &MavlinkStreamCommandLong::get_id_static),
	new StreamListItem(&MavlinkStreamSysStatus::new_instance, &MavlinkStreamSysStatus::get_name_static, &MavlinkStreamSysStatus::get_id_static),
	new StreamListItem(&MavlinkStreamHighresIMU::new_instance, &MavlinkStreamHighresIMU::get_name_static, &MavlinkStreamHighresIMU::get_id_static),
	new StreamListItem(&MavlinkStreamGPSRawInt::new_instance, &MavlinkStreamGPSRawInt::get_name_static, &MavlinkStreamGPSRawInt::get_id_static),
	new StreamListItem(&MavlinkStreamSystemTime::new_instance, &MavlinkStreamSystemTime::get_name_static, &MavlinkStreamSystemTime::get_id_static),
	new StreamListItem(&MavlinkStreamServoOutputRaw<0>::new_instance, &MavlinkStreamServoOutputRaw<0>::get_name_static, &MavlinkStreamServoOutputRaw<0>::get_id_static),
	new StreamListItem(&MavlinkStreamServoOutputRaw<1>::new_instance, &MavlinkStreamServoOutputRaw<1>::get_name_static, &MavlinkStreamServoOutputRaw<1>::get_id_static),
	new StreamListItem(&MavlinkStreamServoOutputRaw<2>::new_instance, &MavlinkStreamServoOutputRaw<2>::get_name_static, &MavlinkStreamServoOutputRaw<2>::get_id_static),
	new StreamListItem(&MavlinkStreamServoOutputRaw<3>::new_instance, &MavlinkStreamServoOutputRaw<3>::get_name_static, &MavlinkStreamServoOutputRaw<3>::get_id_static),
	new StreamListItem(&MavlinkStreamHILActuatorControls::new_instance, &MavlinkStreamHILActuatorControls::get_name_static, &MavlinkStreamHILActuatorControls::get_id_static),
	new StreamListItem(&MavlinkStreamRCChannels::new_instance, &MavlinkStreamRCChannels::get_name_static, &MavlinkStreamRCChannels::get_id_static),
	new StreamListItem(&MavlinkStreamOpticalFlowRad::new_instance, &MavlinkStreamOpticalFlowRad::get_name_static, &MavlinkStreamOpticalFlowRad::get_id_static),
	new StreamListItem(&MavlinkStreamNamedValueFloat::new_instance, &MavlinkStreamNamedValueFloat::get_name_static, &MavlinkStreamNamedValueFloat::get_id_static),
	new StreamListItem(&MavlinkStreamDistanceSensor::new_instance, &MavlinkStreamDistanceSensor::get_name_static, &MavlinkStreamDistanceSensor::get_id_static),
	nullptr
};
