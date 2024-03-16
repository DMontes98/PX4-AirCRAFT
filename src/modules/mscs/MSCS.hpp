/****************************************************************************
 *
 *   Copyright (c) 2024 Daniel Montes <daniel.montestolon@slu.edu>
 *   AirCRAFT Lab - Saint Louis University
 *
 ****************************************************************************/

/**
 * @file MSCS.hpp
 * Implementation of the Aircraft Lab's Modular Sensor and Computing System
 *
 * @author Daniel Montes 	<daniel.montestolon@slu.edu>
 *
 */


#ifndef MSCS_HPP_
#define MSCS_HPP_

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/log.h>				// Includes PX4_INFO()

#include <cstdlib>
#include <cmath>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>

#include <stdio.h>
#include <string.h>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <poll.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/key_events.h>
#include <uORB/topics/mscs_status.h>
#include <uORB/topics/mscs_event.h>
#include <uORB/topics/mscs_calibration_event.h>
#include <uORB/topics/mscs_command_transaction.h>
#include <uORB/topics/mscs_left_adp_calibrated_static_pressure_data.h>
#include <uORB/topics/mscs_left_adp_calibrated_temperature_data.h>
#include <uORB/topics/mscs_left_adp_calibrated_angle_of_attack_data.h>
#include <uORB/topics/mscs_left_adp_calibrated_angle_of_sideslip_data.h>
#include <uORB/topics/mscs_left_adp_ias_data.h>
#include <uORB/topics/mscs_right_adp_calibrated_static_pressure_data.h>
#include <uORB/topics/mscs_right_adp_calibrated_temperature_data.h>
#include <uORB/topics/mscs_right_adp_calibrated_angle_of_attack_data.h>
#include <uORB/topics/mscs_right_adp_calibrated_angle_of_sideslip_data.h>
#include <uORB/topics/mscs_right_adp_ias_data.h>

#include "MSCS_poll.hpp"

#define MSCS_ID_LEFT_ADP			22
#define MSCS_ID_RIGHT_ADP			23

#define MSCS_ADP_SENSOR_ANGLE_OF_ATTACK		2
#define MSCS_ADP_SENSOR_ANGLE_OF_SIDESLIP	3

#define MSCS_EVENT_HANDLING_TIMEOUT		(uint64_t) 5e5
#define MSCS_STANDARD_QUEUE_SIZE		32

#define MSCS_EVENT_INIT				0
#define MSCS_EVENT_DEVICE_CONNECTED		1
#define MSCS_EVENT_DEVICE_DISCONNECTED		2
#define MSCS_EVENT_PARAMETERS_UPDATED		10

#define MSCS_POLL_TASK_ID			1

#define MSCS_NODE_MODE_IDLE			1

#define uavcan_node_ExecuteCommand_Request_1_2_COMMAND_DATA_ACQUISITION_START   (110U)
#define uavcan_node_ExecuteCommand_Request_1_2_COMMAND_DATA_ACQUISITION_STOP    (111U)
#define uavcan_node_ExecuteCommand_Request_1_2_COMMAND_DATA_ACQUISITION_CONFIG  (112U)
#define uavcan_node_ExecuteCommand_Request_1_2_COMMAND_DATA_CALIBRATION_START   (130U)
#define uavcan_node_ExecuteCommand_Request_1_2_COMMAND_DATA_CALIBRATION_SUBMIT  (131U)
#define uavcan_node_ExecuteCommand_Request_1_2_COMMAND_DATA_CALIBRATION_CANCEL  (132U)

#define MSCS_COMMAND_DATA_ACQUISITION_START 					uavcan_node_ExecuteCommand_Request_1_2_COMMAND_DATA_ACQUISITION_START
#define MSCS_COMMAND_DATA_ACQUISITION_STOP  					uavcan_node_ExecuteCommand_Request_1_2_COMMAND_DATA_ACQUISITION_STOP
#define MSCS_COMMAND_DATA_ACQUISITION_CONFIG  					uavcan_node_ExecuteCommand_Request_1_2_COMMAND_DATA_ACQUISITION_CONFIG
#define MSCS_COMMAND_DATA_CALIBRATION_START  					uavcan_node_ExecuteCommand_Request_1_2_COMMAND_DATA_CALIBRATION_START
#define MSCS_COMMAND_DATA_CALIBRATION_SUBMIT  					uavcan_node_ExecuteCommand_Request_1_2_COMMAND_DATA_CALIBRATION_SUBMIT
#define MSCS_COMMAND_DATA_CALIBRATION_CANCEL  					uavcan_node_ExecuteCommand_Request_1_2_COMMAND_DATA_CALIBRATION_CANCEL

#define MSCS_STATE_STARTUP			0
#define MSCS_STATE_READY			1
#define MSCS_STATE_CALIBRATION			2

#define MSCS_CALIBRATION_BAD                    0
#define MSCS_CALIBRATION_OK                     1
#define MSCS_CALIBRATION_CONNECTION_ESTABLISHED 2

#define MSCS_CALIBRATION_MOVE_TO_NEXT           1
#define MSCS_CALIBRATION_OVERFLOW               2

#define KEY_EVENTS_PRIORITY_DEBUG  		0
#define KEY_EVENTS_PRIORITY_LOW  		1
#define KEY_EVENTS_PRIORITY_MEDIUM 		2
#define KEY_EVENTS_PRIORITY_HIGH 		3
#define KEY_EVENTS_MSCS_SYSTEM_STARTUP 		1
#define KEY_EVENTS_MSCS_SYSTEM_SHUTDOWN 	2
#define KEY_EVENTS_MSCS_DEVICE_CONNECTED 	10
#define KEY_EVENTS_MSCS_DEVICE_DISCONNECTED 	11
#define KEY_EVENTS_MSCS_CALIBRATION_START 	30
#define KEY_EVENTS_MSCS_CALIBRATION_END		31
#define KEY_EVENTS_MSCS_COMMAND_SENT		50
#define KEY_EVENTS_SOURCE_MSCS 			1
#define KEY_EVENTS_SOURCE_CLI 			2
#define KEY_EVENTS_DEVICE_FLIGHT_CONTROLLER 	0
#define KEY_EVENTS_DEVICE_LEFT_ADP 		22
#define KEY_EVENTS_DEVICE_RIGHT_ADP 		23

#define MSCS_CALIBRATION_LOOP_REFRESH_RATE      (int) 30
#define MSCS_CALIBRATION_LOOP_REFRESH_PERIOD_US ((int) ((int) 1e6)/(MSCS_CALIBRATION_LOOP_REFRESH_RATE))

using namespace time_literals;

class MSCS_poll;
class MSCS_calibration;
typedef struct adp_config_s {
	uint8_t data_publishing_rate;
	uint8_t pressure_data_acquisition_rate;
	uint8_t angle_data_acquisition_rate;
	uint8_t airspeed_divisor;
	uint8_t altitude_divisor;
	uint8_t data_publishing_mode_raw;
	uint8_t data_publishing_mode_calibrated;
	uint8_t differential_pressure_output_enable;
	uint8_t static_pressure_output_enable;
	uint8_t angle_of_attack_output_enable;
	uint8_t angle_of_sideslip_output_enable;
	uint8_t temperature_output_enable;
	uint8_t pressure_altitude_output_enable;
	uint8_t density_altitude_output_enable;
	uint8_t agl_altitude_output_enable;
	uint8_t msl_altitude_output_enable;
	uint8_t ias_output_enable;
	uint8_t cas_output_enable;
	uint8_t tas_output_enable;
} adp_config_s;

template <typename T> class MSCS_queue
{
public:
	MSCS_queue(){};
	~MSCS_queue(){};

	/* Adding and removing */
	void push(T& message) {
		/* FUNCTION BODY */
		/* Save Event at correct position and increment tail pointer */
		_data_buffer[_tail_pointer++] = message;

		/* Handle wrap-around */
		if (_tail_pointer >= MSCS_STANDARD_QUEUE_SIZE) {
			_tail_pointer = 0;
		}

		//PX4_INFO("[MSCS_event_queue::push()] New Element pushed to Queue - HP:%i TP: %i", _head_pointer, _tail_pointer);
	};
	void peek(T* message_buffer) {
		/* FUNCTION BODY */
		memcpy(message_buffer, &_data_buffer[_head_pointer], sizeof(T));

		//PX4_INFO("[MSCS_event_queue::peek()] Element peeked from queue - HP:%i TP: %i", _head_pointer, _tail_pointer);
	};
	void pop() {
		/* FUNCTION BODY */
		_head_pointer++;

		/* Handle wrap-around */
		if (_head_pointer >= MSCS_STANDARD_QUEUE_SIZE) {
			_head_pointer = 0;
		}

		//PX4_INFO("[MSCS_event_queue::pop()] Element Popped from Queue - HP:%i TP: %i", _head_pointer, _tail_pointer);
	};
	int empty() {
		/* VARIABLE DEFINITIONS */
		int return_value = 0;

		/* FUNCTION BODY */
		if (_head_pointer == _tail_pointer) {
			return_value = 1;
		}

		//PX4_INFO("[MSCS_event_queue::empty()] Element Queue State - HP:%i TP: %i", _head_pointer, _tail_pointer);

		return return_value;
	};

private:
	/* Control variables */
	int _head_pointer{0};
	int _tail_pointer{0};

	/* Data Buffer */
	T _data_buffer[MSCS_STANDARD_QUEUE_SIZE];
};
class MSCS : public ModuleBase<MSCS>, public ModuleParams, public px4::ScheduledWorkItem {
public:
	MSCS();
	~MSCS() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

	/* MSCS Functions */
	void register_poll(MSCS_poll* MSCS_poll_ref);

	static int start_data_acquisition_command_handler(int argc, char *argv[]);
	int  start_data_acquisition_command(int argc, char *argv[]);
	void start_data_acquisition(uint16_t node_id);

	static int stop_data_acquisition_command_handler(int argc, char *argv[]);
	int  stop_data_acquisition_command(int argc, char *argv[]);
	void stop_data_acquisition(uint16_t node_id);

	static int adp_calibration_command_handler(int argc, char *argv[]);
	int  adp_calibration_handler(int argc, char *argv[]);
	static int list_command_handler(int argc, char *argv[]);
	int  list_command(int argc, char *argv[]);

	void key_event(uint8_t priority, uint16_t event_type, uint16_t device, const char* event_data);

private:
	static uint8_t _get_node_id_from_string(const char *argv);

	void _update_parameters();
	void Run() override;
	void _handle_event(mscs_event_s MSCS_event_s);
	void _command_handler();
	void _config_node(uint16_t node_id);

	int _load_device_config(uint8_t node_id);

	/* Calibration */
	uint16_t _wait_calibration_response(uint16_t node_id, uint64_t timeout_ms);
	uint16_t _wait_calibration_connection(uint16_t node_id, uint64_t timeout_ms);

	void _submit_calibration(uint16_t node_id);
	void _start_angle_of_attack_calibration(uint16_t node_id);
	void _start_angle_of_sideslip_calibration(uint16_t node_id);

	void _adp_airspeed_calibration(uint16_t node_id);
	void _adp_altitude_calibration(uint16_t node_id);
	void _adp_angle_calibration(uint16_t node_id, uint16_t angle);

	void _adp_airspeed_calibration_loop(uint16_t node_id);

	/* MSCS Objects */
	static MSCS* _instance;
	MSCS_queue<mscs_command_transaction_s>* _command_queue{};
	MSCS_poll* _MSCS_poll;

	/* MSCS Functions */
	void _spawn_poll();

	/* MSCS Device Configuration */
	adp_config_s _left_adp_config {0};
	adp_config_s _right_adp_config{0};

	/* Sample Interval */
	static const hrt_abstime SAMPLE_INTERVAL{200_ms};

	/* uORB Buffers */
	mscs_status_s                                     _mscs_status{0};
	mscs_event_s                                      _mscs_event_msg{0};
	mscs_calibration_event_s                          _mscs_calibration_event_msg{0};
	mscs_command_transaction_s                        _mscs_command_transaction_msg{0};
	mscs_left_adp_ias_data_s                          _mscs_left_adp_ias_data_msg{0};
	mscs_left_adp_calibrated_static_pressure_data_s   _mscs_left_adp_calibrated_static_pressure_data_msg{0};
	mscs_left_adp_calibrated_temperature_data_s       _mscs_left_adp_calibrated_temperature_data_msg{0};
	mscs_left_adp_calibrated_angle_of_attack_data_s   _mscs_left_adp_calibrated_angle_of_attack_data_msg{0};
	mscs_left_adp_calibrated_angle_of_sideslip_data_s _mscs_left_adp_calibrated_angle_of_sideslip_data_msg{0};

	/* uORB Subscriptions */
	uORB::Subscription _mscs_status_sub{ORB_ID(mscs_status)};
	uORB::Subscription _mscs_event_sub{ORB_ID(mscs_event)};
	uORB::Subscription _mscs_calibration_event_sub{ORB_ID(mscs_calibration_event)};
	uORB::Subscription _mscs_command_transaction_sub{ORB_ID(mscs_command_transaction)};
	uORB::Subscription _mscs_left_adp_ias_data_sub                          {ORB_ID(mscs_left_adp_ias_data)};
	uORB::Subscription _mscs_left_adp_calibrated_static_pressure_data_sub   {ORB_ID(mscs_left_adp_calibrated_static_pressure_data)};
	uORB::Subscription _mscs_left_adp_calibrated_temperature_data_sub       {ORB_ID(mscs_left_adp_calibrated_temperature_data)};
	uORB::Subscription _mscs_left_adp_calibrated_angle_of_attack_data_sub   {ORB_ID(mscs_left_adp_calibrated_angle_of_attack_data)};
	uORB::Subscription _mscs_left_adp_calibrated_angle_of_sideslip_data_sub {ORB_ID(mscs_left_adp_calibrated_angle_of_sideslip_data)};

	/* uORB Publications */
	uORB::Publication<key_events_s>               _key_events_pub {ORB_ID(key_events)};
	uORB::Publication<mscs_status_s>              _mscs_status_pub{ORB_ID(mscs_status)};
	uORB::Publication<mscs_calibration_event_s>   _mscs_calibration_event_pub{ORB_ID(mscs_calibration_event)};
	uORB::Publication<mscs_event_s>               _mscs_event_pub{ORB_ID(mscs_event)};
	uORB::Publication<mscs_command_transaction_s> _mscs_command_transaction_pub{ORB_ID(mscs_command_transaction)};

	/* Performance counters */
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MSCS_ID_ADP_L>)    _left_adp_node_id,
		(ParamInt<px4::params::MSCS_ID_ADP_R>)    _right_adp_node_id,
		(ParamInt<px4::params::MSCS_ADP_RAW>)     _adp_raw_mode,
		(ParamInt<px4::params::MSCS_ADP_CAL>)     _adp_calibrated_mode,
		(ParamInt<px4::params::MSCS_ADP_RT>)      _adp_data_rate,
		(ParamInt<px4::params::MSCS_ADP_ANG_RT>)  _adp_angle_sensor_rate,
		(ParamInt<px4::params::MSCS_ADP_PRS_RT>)  _adp_pressure_sensor_rate,
		(ParamInt<px4::params::MSCS_ADP_DIF_PRS>) _adp_differential_pressure_enable,
		(ParamInt<px4::params::MSCS_ADP_STA_PRS>) _adp_static_pressure_enable,
		(ParamInt<px4::params::MSCS_ADP_AOA>)     _adp_angle_of_attack_enable,
		(ParamInt<px4::params::MSCS_ADP_BETA>)    _adp_angle_of_sideslip_enable,
		(ParamInt<px4::params::MSCS_ADP_TEMP>)    _adp_temperature_enable,
		(ParamInt<px4::params::MSCS_ADP_ASPD_DV>) _adp_airspeed_divisor,
		(ParamInt<px4::params::MSCS_ADP_ALT_DV>)  _adp_altitude_divisor,
		(ParamInt<px4::params::MSCS_ADP_IAS>)     _adp_ias_enable,
		(ParamInt<px4::params::MSCS_ADP_CAS>)     _adp_cas_enable,
		(ParamInt<px4::params::MSCS_ADP_TAS>)     _adp_tas_enable,
		(ParamInt<px4::params::MSCS_ADP_PRS_ALT>) _adp_pressure_altitude_enable,
		(ParamInt<px4::params::MSCS_ADP_DEN_ALT>) _adp_density_altitude_enable,
		(ParamInt<px4::params::MSCS_ADP_MSL_ALT>) _adp_msl_altitude_enable,
		(ParamInt<px4::params::MSCS_ADP_AGL_ALT>) _adp_agl_altitude_enable,
		(ParamInt<px4::params::MSCS_KEY_EVT_LVL>) _mscs_key_event_priority
	)
};

extern "C" __EXPORT int mscs_main(int argc, char *argv[]);

#endif /* MSCS_HPP_ */
