/****************************************************************************
 *
 *   Copyright (c) 2024 Daniel Montes <daniel.montestolon@slu.edu>
 *   AirCRAFT Lab - Saint Louis University
 *
 ****************************************************************************/

/**
 * @file MSCS.cpp
 * Implementation of the Aircraft Lab's Modular Sensor and Computing System
 *
 * @author Daniel Montes 	<daniel.montestolon@slu.edu>
 *
 */

#include "MSCS.hpp"

#define POLL_TASK_STACK_SIZE			10000
#define CALIBRATION_TASK_STACK_SIZE     	10000

#define MAX_COMMAND_DEVICES			8

#define CALIBRATION_RESPONSE_TIMEOUT_MS		1000

#define ABS_BAR					0
#define SIGNED_BAR				1

#define BAR_SIZE				51

#define ENTER_KEY                               0x0D

MSCS* MSCS::_instance;
static uint8_t mscs_key_event_priority_level{0};

void _printbar(float min, float max, float value, uint8_t bar_type) {
	/* VARIABLE DEFINITION */
	float    bar_size = (float) BAR_SIZE;
	float    step     = (max-min)/bar_size;
	float    number_of_steps_f;
	uint16_t number_of_steps;
	uint16_t step_counter;

	/* FUNCTION BODY */
	/* Trim input */
	if (value < min) {
		value = min;
	} else if (value > max) {
		value = max;
	}

	/* Print first bracket */
	printf("[");

	/* Check bar type */
	switch (bar_type)
	{
		case ABS_BAR:
			number_of_steps_f = (value-min)/step;
			number_of_steps   = (uint16_t) roundf(number_of_steps_f);

			for (step_counter = 0; step_counter < number_of_steps; step_counter++) {
				printf("=");
			}

			for (; step_counter < BAR_SIZE; step_counter++) {
				printf(" ");
			}

			break;

		case SIGNED_BAR:
			/* code */
			break;

		default:
			break;
	}

	/* Print last bracket */
	printf("]");

}

void _printfloat(float value) {
	if (value < 10.f) {
		printf("  ");
	} else if (value < 100.f) {
		printf(" ");
	}
	printf("%.2f", (double) value);
};

void _wait_keypress(char key, uint64_t timeout_ms) {
	/* VARIABLE DEFINITION */
	const hrt_abstime system_timestamp = hrt_absolute_time();
	struct termios old_termios, new_termios;
	struct timeval timeout;
	char pressed_key;
	fd_set set;

	/* FUNCTION BODY */
	/* Save termios state */
    	tcgetattr(STDIN_FILENO, &old_termios);

	/* Configure new termios*/
	new_termios = old_termios;
	new_termios.c_lflag &= ~(ICANON | ECHO);
    	tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);

	/* Set timeout to a value depending on refresh rate so that we get wait time from select() */
	timeout.tv_sec  = 0;
	timeout.tv_usec = MSCS_CALIBRATION_LOOP_REFRESH_PERIOD_US;

	/* Main Loop */
	while (1) {
		/* Check User Input*/
		if (select(STDIN_FILENO + 1, &set, NULL, NULL, &timeout) > 0) {
			read(STDIN_FILENO, &pressed_key, 1);

			// Exit loop if target key is pressed
			if (pressed_key == key) {
				printf("\n");
				break;
			}
		}
		/* Check for Response Timeout */
		if ((hrt_absolute_time()-system_timestamp) > (timeout_ms*((uint64_t)1e3))) {
			break;
		}

	}
}


MSCS::MSCS() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

MSCS::~MSCS()
{
	key_event(KEY_EVENTS_PRIORITY_MEDIUM,
	          KEY_EVENTS_MSCS_SYSTEM_SHUTDOWN,
	          KEY_EVENTS_DEVICE_FLIGHT_CONTROLLER,
	          "AirCRAFT Lab's Modular Sensor and Computing System Shutdown");

	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

void MSCS::key_event(uint8_t priority, uint16_t event_type, uint16_t device, const char* event_data) {

	/* VARIABLE DEFINITIONS */
	const hrt_abstime system_timestamp = hrt_absolute_time();
	key_events_s key_event_msg{0};

	/* FUNCTION BODY */
	/* Check current priority level */
	if (priority >= mscs_key_event_priority_level) {
		/* Build Key Event Message */
		key_event_msg.timestamp    = system_timestamp;
		key_event_msg.priority     = priority;
		key_event_msg.event_type   = event_type;
		key_event_msg.event_source = KEY_EVENTS_SOURCE_MSCS;
		key_event_msg.device 	   = device;

		/* Clear event data buffer */
		memset (key_event_msg.event_data,       '\0', sizeof(key_event_msg.event_data));
		strncpy(key_event_msg.event_data, event_data, sizeof(key_event_msg.event_data));

		/* Publish Message */
		_key_events_pub.publish(key_event_msg);

		//PX4_INFO("[MSCS::key_event()] Publishing Key Event");
	}
}

bool MSCS::init()
{
	/* VARIABLE DEFINITIONS */
	const hrt_abstime system_timestamp = hrt_absolute_time();
	mscs_command_transaction_s mscs_command_transaction{0};

	/* FUNCTION BODY */
	/* Update Parameters */
	_update_parameters();

	/* Create Command Queue */
	_command_queue = new MSCS_queue<mscs_command_transaction_s>;

	/* Load default configuration to each of the active devices */
	(void) _load_device_config(MSCS_ID_LEFT_ADP);
	(void) _load_device_config(MSCS_ID_RIGHT_ADP);

	/* Spawn all MSCS Tasks */
	_spawn_poll();

	PX4_INFO("AirCRAFT Lab's Modular Sensor and Computing System Initialized");

	key_event(KEY_EVENTS_PRIORITY_MEDIUM,
	 	  KEY_EVENTS_MSCS_SYSTEM_STARTUP,
		  KEY_EVENTS_DEVICE_FLIGHT_CONTROLLER,
		  "AirCRAFT Lab's Modular Sensor and Computing System Initialized");

	/* Publish empty command to initialize commander */
	mscs_command_transaction.timestamp = system_timestamp;

	_command_queue->push(mscs_command_transaction);
	_command_handler();

	ScheduleOnInterval(SAMPLE_INTERVAL);

	return true;
}

void MSCS::Run()
{
	/* FUNCTION BODY */
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	/* Poll Events */
	if (_mscs_event_sub.updated()) {

		/* Copy parameter update topic */
		_mscs_event_sub.copy(&_mscs_event_msg);

		/* Handle Events */
		_handle_event(_mscs_event_msg);
	}

	/* Check if command queue is empty */
	//PX4_INFO_RAW("[MSCS::Run()] - Checking if event queue is empty\n");
	if (!_command_queue->empty()) {
		//PX4_INFO_RAW("[MSCS::Run()] - Command Queue is not empty\n");
		_command_handler();
	}
}

int MSCS::task_spawn(int argc, char *argv[])
{
	/* Create New MSCS Instance */
	MSCS *instance = new MSCS();

	_instance = instance;

	/* Check if instance was properly created */
	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		/* Initialize MSCS instance */
		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MSCS::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int MSCS::custom_command(int argc, char *argv[])
{
	return 0;
}

int MSCS::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
		### Description
		Aircraft Lab's Modular Sensor and Computing System.

		)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("MSCS", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

void MSCS::_command_handler() {
	/* VARIABLE DEFINITIONS */
	const hrt_abstime system_timestamp = hrt_absolute_time();
	mscs_command_transaction_s current_command;
	mscs_command_transaction_s next_command;

	/* FUNCTION_BODY */
	/* Check if current Event has not yet been handled */
	if (_mscs_command_transaction_sub.copy(&current_command)) {
		/* Check if event has been handled already or has timed out */
		if ((current_command.command_handled != 0) || ((system_timestamp - current_command.timestamp) > ((uint64_t) 1e3 * (uint64_t) current_command.command_timeout_ms))) {
			//PX4_INFO_RAW("[MSCS::_command_handler()] - Current Command Handled or Timed Out\n");
			/* Pop if current queue'd event is the same as handled event */
			_command_queue->peek(&next_command);
			//PX4_INFO("[MSCS_poll::_event_handler()] - NEXT COMMAND - Timestamp:  %llu",  next_command.timestamp);
			//PX4_INFO("[MSCS_poll::_event_handler()] - NEXT COMMAND - Command: %hu",   next_command.command);
			//PX4_INFO("[MSCS_poll::_event_handler()] - NEXT COMMAND - Node ID:    %hu",   next_event.node_id);
			//PX4_INFO("[MSCS_poll::_event_handler()] - NEXT COMMAND - Timeout:    %lums", next_event.event_timeout_ms);

			if (current_command.timestamp == next_command.timestamp) {/* Event no longer needs to be handled */
				_command_queue->pop();
				//PX4_INFO("[MSCS_poll::_event_handler()] - Event Removed from event Queue");
			}

			/* Is there another event in line? */
			if (!_command_queue->empty()) {

				/* Publish Event */
				_command_queue->peek(&next_command);
				_mscs_command_transaction_pub.publish(next_command);

				//PX4_INFO_RAW("[MSCS::_command_handler()] - Publishing Command\n");
			}
		} else {
			//PX4_INFO_RAW("[MSCS::_command_handler()] - Current Command Not Handled yet\n");
		}

	} else {
		/* No first event has been transmitted */
		/* Get next event in line */
		_command_queue->peek(&next_command);

		/* Publish Event */
		_mscs_command_transaction_pub.publish(next_command);

		//PX4_INFO_RAW("[MSCS::_command_handler()] - Publishing First Command\n");
	}
};

void MSCS::_handle_event(mscs_event_s MSCS_event_s) {
	/* FUNCTION BODY */
	switch (MSCS_event_s.source_module)
	{
	case MSCS_POLL_TASK_ID:
		/* Check Event Type */
		switch (MSCS_event_s.event_type)
		{
		case MSCS_EVENT_INIT:
			//PX4_INFO("[MSCS::_handle_event()] - MSCS::poll() Initialized");
			break;
		case MSCS_EVENT_DEVICE_CONNECTED:
			//PX4_INFO("[MSCS::_handle_event()] - New Device Detected");
			/* Check MSCS node ID*/
			switch (MSCS_event_s.node_id)
			{
			case MSCS_ID_LEFT_ADP:
				PX4_INFO("[MSCS::_handle_event()] - Left ADP Device Detected");

				key_event(KEY_EVENTS_PRIORITY_MEDIUM,
					KEY_EVENTS_MSCS_DEVICE_CONNECTED,
					KEY_EVENTS_DEVICE_LEFT_ADP,
					"Left ADP Device Connection Detected");

				/* Check Device Mode*/
				switch (MSCS_event_s.event_data[0])
				{
				case MSCS_NODE_MODE_IDLE:
					//PX4_INFO("[MSCS::_handle_event()] - Device Mode: IDLE");
					//PX4_INFO("[MSCS::_handle_event()] - Initializing Device...");

					/* Send Configuration Command */
					//PX4_INFO("[MSCS::_handle_command_transaction()] - Preparing DATA ACQUISITION CONFIG command for LEFT ADP");
					_config_node(MSCS_ID_LEFT_ADP);

					/* Send Data Acquisition Start Command */
					//PX4_INFO("[MSCS::_handle_command_transaction()] - Preparing DATA ACQUISITION START command for LEFT ADP");
					start_data_acquisition(MSCS_event_s.dynamic_node_id);
					/* Set Event Handled Flag */
					MSCS_event_s.event_handled = 1;
					break;

				default:
					break;
				}
				break;

			case MSCS_ID_RIGHT_ADP:
				//PX4_INFO("[MSCS::_handle_event()] - Right ADP Device Detected");

				key_event(KEY_EVENTS_PRIORITY_MEDIUM,
					KEY_EVENTS_MSCS_DEVICE_CONNECTED,
					KEY_EVENTS_DEVICE_RIGHT_ADP,
					"Right ADP Device Connection Detected");

				/* Check Device Mode*/
				switch (MSCS_event_s.event_data[0])
				{
				case MSCS_NODE_MODE_IDLE:
					//PX4_INFO("[MSCS::_handle_event()] - Device Mode: IDLE");
					//PX4_INFO("[MSCS::_handle_event()] - Initializing Device...");

					/* Send Configuration Command */
					//PX4_INFO("[MSCS::_handle_command_transaction()] - Preparing DATA ACQUISITION CONFIG command for RIGHT ADP");
					_config_node(MSCS_ID_RIGHT_ADP);

					/* Send Data Acquisition Start Command */
					//PX4_INFO("[MSCS::_handle_command_transaction()] - Preparing DATA ACQUISITION START command for RIGHT ADP");
					start_data_acquisition(MSCS_event_s.dynamic_node_id);
					/* Set Event Handled Flag */
					MSCS_event_s.event_handled = 1;
					break;

				default:
					break;
				}
				break;
			default:
				break;
			}
			break;

		case MSCS_EVENT_DEVICE_DISCONNECTED:
			//PX4_INFO("[MSCS::_handle_event()] - New Device Detected");
			/* Check MSCS node ID*/
			switch (MSCS_event_s.node_id)
			{
			case MSCS_ID_LEFT_ADP:
				//PX4_INFO("[MSCS::_handle_event()] - Left ADP Device Disconnected");

				key_event(KEY_EVENTS_PRIORITY_HIGH,
					KEY_EVENTS_MSCS_DEVICE_DISCONNECTED,
					KEY_EVENTS_DEVICE_LEFT_ADP,
					"Left ADP Device Disconnected");

				/* Check Device Mode*/
				switch (MSCS_event_s.event_data[0])
				{
				case MSCS_NODE_MODE_IDLE:
					//PX4_INFO("[MSCS::_handle_event()] - Device Mode: IDLE");
					//PX4_INFO("[MSCS::_handle_event()] - Initializing Device...");

					/* Send Configuration Command */
					//PX4_INFO("[MSCS::_handle_command_transaction()] - Preparing DATA ACQUISITION CONFIG command for LEFT ADP");
					_config_node(MSCS_ID_LEFT_ADP);

					/* Send Data Acquisition Start Command */
					//PX4_INFO("[MSCS::_handle_command_transaction()] - Preparing DATA ACQUISITION START command for LEFT ADP");
					start_data_acquisition(MSCS_event_s.dynamic_node_id);
					/* Set Event Handled Flag */
					MSCS_event_s.event_handled = 1;
					break;

				default:
					break;
				}
				break;

			case MSCS_ID_RIGHT_ADP:
				//PX4_INFO("[MSCS::_handle_event()] - Right ADP Device Detected");

				key_event(KEY_EVENTS_PRIORITY_HIGH,
					KEY_EVENTS_MSCS_DEVICE_DISCONNECTED,
					KEY_EVENTS_DEVICE_RIGHT_ADP,
					"Right ADP Device Disconnected");

				/* Check Device Mode*/
				switch (MSCS_event_s.event_data[0])
				{
				case MSCS_NODE_MODE_IDLE:
					//PX4_INFO("[MSCS::_handle_event()] - Device Mode: IDLE");
					//PX4_INFO("[MSCS::_handle_event()] - Initializing Device...");

					/* Send Configuration Command */
					//PX4_INFO("[MSCS::_handle_command_transaction()] - Preparing DATA ACQUISITION CONFIG command for RIGHT ADP");
					_config_node(MSCS_ID_RIGHT_ADP);

					/* Send Data Acquisition Start Command */
					//PX4_INFO("[MSCS::_handle_command_transaction()] - Preparing DATA ACQUISITION START command for RIGHT ADP");
					start_data_acquisition(MSCS_event_s.dynamic_node_id);
					/* Set Event Handled Flag */
					MSCS_event_s.event_handled = 1;
					break;
				default:
					break;
				}
				break;

			default:
				break;
			}
			break;
		case MSCS_EVENT_PARAMETERS_UPDATED:
			PX4_INFO("[MSCS::_handle_event()] - Updating MSCS Parameters");

			/* Update Parameters */
			_update_parameters();

			/* Set Event Handled Flag */
			MSCS_event_s.event_handled = 1;

			break;

		default:
			break;
		}
		break;

	default:
		break;
	}

	if (MSCS_event_s.event_handled) {
		/* Set event handled flag */
		_mscs_event_pub.publish(MSCS_event_s);

		/* Copy parameter to remove updated flag */
		_mscs_event_sub.copy(&MSCS_event_s);
	};

}

void MSCS::_update_parameters() {
	/* FUNCTION BODY */
	/* Update Parameters */
	updateParams();

	/* Update Key Event priority level */
	mscs_key_event_priority_level = _mscs_key_event_priority.get();

	/* Load current MSCS status */
	if (_mscs_status_sub.updated()) {
		_mscs_status_sub.copy(&_mscs_status);
	}

	/* Reconfigure Sensors depending on updated parameters in Real Time */
	if (_load_device_config(MSCS_ID_LEFT_ADP)) {
		/* Check if device is found in bus */
		if (_mscs_status.left_adp_active) {
			_config_node(MSCS_ID_LEFT_ADP);
		}
	}
	if (_load_device_config(MSCS_ID_RIGHT_ADP)) {
		/* Check if device is found in bus */
		if (_mscs_status.right_adp_active) {
			_config_node(MSCS_ID_RIGHT_ADP);
		}
	}
}

int MSCS::_load_device_config(uint8_t node_id) {
	/* VARIABLE DEFINITION */
	int config_updated = 0;

	/* FUNCTION BODY */
	/* Switch device */
	switch (node_id)
	{
	case MSCS_ID_LEFT_ADP:
		/* Left ADP */
		if (_left_adp_config.data_publishing_rate != _adp_data_rate.get()) {
			_left_adp_config.data_publishing_rate = _adp_data_rate.get();
			config_updated = 1;
		}
		if (_left_adp_config.pressure_data_acquisition_rate != _adp_pressure_sensor_rate.get()) {
			_left_adp_config.pressure_data_acquisition_rate = _adp_pressure_sensor_rate.get();
			config_updated = 1;
		}
		if (_left_adp_config.angle_data_acquisition_rate != _adp_angle_sensor_rate.get()) {
			_left_adp_config.angle_data_acquisition_rate = _adp_angle_sensor_rate.get();
			config_updated = 1;
		}
		if (_left_adp_config.airspeed_divisor != _adp_airspeed_divisor.get()) {
			_left_adp_config.airspeed_divisor = _adp_airspeed_divisor.get();
			config_updated = 1;
		}
		if (_left_adp_config.altitude_divisor != _adp_altitude_divisor.get()) {
			_left_adp_config.altitude_divisor = _adp_altitude_divisor.get();
			config_updated = 1;
		}
		if (_left_adp_config.data_publishing_mode_raw != _adp_raw_mode.get()) {
			_left_adp_config.data_publishing_mode_raw = _adp_raw_mode.get();
			config_updated = 1;
		}
		if (_left_adp_config.data_publishing_mode_calibrated != _adp_calibrated_mode.get()) {
			_left_adp_config.data_publishing_mode_calibrated = _adp_calibrated_mode.get();
			config_updated = 1;
		}
		if (_left_adp_config.differential_pressure_output_enable != _adp_differential_pressure_enable.get()) {
			_left_adp_config.differential_pressure_output_enable = _adp_differential_pressure_enable.get();
			config_updated = 1;
		}
		if (_left_adp_config.static_pressure_output_enable != _adp_static_pressure_enable.get()) {
			_left_adp_config.static_pressure_output_enable = _adp_static_pressure_enable.get();
			config_updated = 1;
		}
		if (_left_adp_config.angle_of_attack_output_enable != _adp_angle_of_attack_enable.get()) {
			_left_adp_config.angle_of_attack_output_enable = _adp_angle_of_attack_enable.get();
			config_updated = 1;
		}
		if (_left_adp_config.angle_of_sideslip_output_enable != _adp_angle_of_sideslip_enable.get()) {
			_left_adp_config.angle_of_sideslip_output_enable = _adp_angle_of_sideslip_enable.get();
			config_updated = 1;
		}
		if (_left_adp_config.temperature_output_enable != _adp_temperature_enable.get()) {
			_left_adp_config.temperature_output_enable = _adp_temperature_enable.get();
			config_updated = 1;
		}
		if (_left_adp_config.pressure_altitude_output_enable != _adp_pressure_altitude_enable.get()) {
			_left_adp_config.pressure_altitude_output_enable = _adp_pressure_altitude_enable.get();
			config_updated = 1;
		}
		if (_left_adp_config.density_altitude_output_enable != _adp_density_altitude_enable.get()) {
			_left_adp_config.density_altitude_output_enable = _adp_density_altitude_enable.get();
			config_updated = 1;
		}
		if (_left_adp_config.agl_altitude_output_enable != _adp_agl_altitude_enable.get()) {
			_left_adp_config.agl_altitude_output_enable = _adp_agl_altitude_enable.get();
			config_updated = 1;
		}
		if (_left_adp_config.msl_altitude_output_enable != _adp_msl_altitude_enable.get()) {
			_left_adp_config.msl_altitude_output_enable = _adp_msl_altitude_enable.get();
			config_updated = 1;
		}
		if (_left_adp_config.ias_output_enable != _adp_ias_enable.get()) {
			_left_adp_config.ias_output_enable = _adp_ias_enable.get();
			config_updated = 1;
		}
		if (_left_adp_config.cas_output_enable != _adp_cas_enable.get()) {
			_left_adp_config.cas_output_enable = _adp_cas_enable.get();
			config_updated = 1;
		}
		if (_left_adp_config.tas_output_enable != _adp_tas_enable.get()) {
			_left_adp_config.tas_output_enable = _adp_tas_enable.get();
			config_updated = 1;
		}
		break;

	case MSCS_ID_RIGHT_ADP:
		/* Right ADP */
		if (_right_adp_config.data_publishing_rate != _adp_data_rate.get()) {
			_right_adp_config.data_publishing_rate = _adp_data_rate.get();
			config_updated = 1;
		}
		if (_right_adp_config.pressure_data_acquisition_rate != _adp_pressure_sensor_rate.get()) {
			_right_adp_config.pressure_data_acquisition_rate = _adp_pressure_sensor_rate.get();
			config_updated = 1;
		}
		if (_right_adp_config.angle_data_acquisition_rate != _adp_angle_sensor_rate.get()) {
			_right_adp_config.angle_data_acquisition_rate = _adp_angle_sensor_rate.get();
			config_updated = 1;
		}
		if (_right_adp_config.airspeed_divisor != _adp_airspeed_divisor.get()) {
			_right_adp_config.airspeed_divisor = _adp_airspeed_divisor.get();
			config_updated = 1;
		}
		if (_right_adp_config.altitude_divisor != _adp_altitude_divisor.get()) {
			_right_adp_config.altitude_divisor = _adp_altitude_divisor.get();
			config_updated = 1;
		}
		if (_right_adp_config.data_publishing_mode_raw != _adp_raw_mode.get()) {
			_right_adp_config.data_publishing_mode_raw = _adp_raw_mode.get();
			config_updated = 1;
		}
		if (_right_adp_config.data_publishing_mode_calibrated != _adp_calibrated_mode.get()) {
			_right_adp_config.data_publishing_mode_calibrated = _adp_calibrated_mode.get();
			config_updated = 1;
		}
		if (_right_adp_config.differential_pressure_output_enable != _adp_differential_pressure_enable.get()) {
			_right_adp_config.differential_pressure_output_enable = _adp_differential_pressure_enable.get();
			config_updated = 1;
		}
		if (_right_adp_config.static_pressure_output_enable != _adp_static_pressure_enable.get()) {
			_right_adp_config.static_pressure_output_enable = _adp_static_pressure_enable.get();
			config_updated = 1;
		}
		if (_right_adp_config.angle_of_attack_output_enable != _adp_angle_of_attack_enable.get()) {
			_right_adp_config.angle_of_attack_output_enable = _adp_angle_of_attack_enable.get();
			config_updated = 1;
		}
		if (_right_adp_config.angle_of_sideslip_output_enable != _adp_angle_of_sideslip_enable.get()) {
			_right_adp_config.angle_of_sideslip_output_enable = _adp_angle_of_sideslip_enable.get();
			config_updated = 1;
		}
		if (_right_adp_config.temperature_output_enable != _adp_temperature_enable.get()) {
			_right_adp_config.temperature_output_enable = _adp_temperature_enable.get();
			config_updated = 1;
		}
		if (_right_adp_config.pressure_altitude_output_enable != _adp_pressure_altitude_enable.get()) {
			_right_adp_config.pressure_altitude_output_enable = _adp_pressure_altitude_enable.get();
			config_updated = 1;
		}
		if (_right_adp_config.density_altitude_output_enable != _adp_density_altitude_enable.get()) {
			_right_adp_config.density_altitude_output_enable = _adp_density_altitude_enable.get();
			config_updated = 1;
		}
		if (_right_adp_config.agl_altitude_output_enable != _adp_agl_altitude_enable.get()) {
			_right_adp_config.agl_altitude_output_enable = _adp_agl_altitude_enable.get();
			config_updated = 1;
		}
		if (_right_adp_config.msl_altitude_output_enable != _adp_msl_altitude_enable.get()) {
			_right_adp_config.msl_altitude_output_enable = _adp_msl_altitude_enable.get();
			config_updated = 1;
		}
		if (_right_adp_config.ias_output_enable != _adp_ias_enable.get()) {
			_right_adp_config.ias_output_enable = _adp_ias_enable.get();
			config_updated = 1;
		}
		if (_right_adp_config.cas_output_enable != _adp_cas_enable.get()) {
			_right_adp_config.cas_output_enable = _adp_cas_enable.get();
			config_updated = 1;
		}
		if (_right_adp_config.tas_output_enable != _adp_tas_enable.get()) {
			_right_adp_config.tas_output_enable = _adp_tas_enable.get();
			config_updated = 1;
		}
		break;

	default:
		break;
	}

	return config_updated;
};

void MSCS::_spawn_poll() {
	/* FUNCTION BODY */
	/* Spawn MSCS_poll() task using px4_task_spawn_cmd() */
	px4_task_spawn_cmd("MSCS_poll",
			   SCHED_DEFAULT,
			   SCHED_PRIORITY_DEFAULT - 50,
			   POLL_TASK_STACK_SIZE,
			   (px4_main_t)&MSCS_poll::task_spawn,
			   (char * const *) this
			   );
};

void MSCS::register_poll(MSCS_poll* MSCS_poll_ref) {
	/* FUNCTION BODY */
	_MSCS_poll = MSCS_poll_ref;
};

uint8_t MSCS::_get_node_id_from_string(const char *argv) {
	/* VARIABLE DEFINITIONS */
	uint8_t node_id = 0;

	/* FUNCTION BODY */
	/* Check if node name is both recognized and active */
	if (!strcmp("left_adp", argv)) {
		node_id = MSCS_ID_LEFT_ADP;
	} else if (!strcmp("right_adp", argv)) {
		node_id = MSCS_ID_RIGHT_ADP;
	}

	return node_id;
};

int MSCS::start_data_acquisition_command_handler(int argc, char *argv[]) {
	/* VARIABLE DEFINITIONS */
	int return_value = 0;

	/* FUNCTION BODY */
	if (_instance != nullptr) {
		return_value = _instance->start_data_acquisition_command(argc, argv);
	} else {
		PX4_INFO_RAW("MSCS has not yet been initialized!\n");
	}
	return return_value;
}

int MSCS::stop_data_acquisition_command_handler(int argc, char *argv[]) {
	/* VARIABLE DEFINITIONS */
	int return_value = 0;

	/* FUNCTION BODY */
	if (_instance != nullptr) {
		return_value = _instance->stop_data_acquisition_command(argc, argv);
	} else {
		PX4_INFO_RAW("MSCS has not yet been initialized!\n");
	}

	return return_value;
}

int MSCS::adp_calibration_command_handler(int argc, char *argv[]) {
	/* VARIABLE DEFINITIONS */
	int return_value = 0;

	/* FUNCTION BODY */
	if (_instance != nullptr) {
		return_value = _instance->adp_calibration_handler(argc, argv);
	} else {
		PX4_INFO_RAW("MSCS has not yet been initialized!\n");
	}

	return return_value;
}

int MSCS::adp_calibration_handler(int argc, char *argv[]) {
	/* VARIABLE DEFINITIONS */
	int return_value = 0;
	uint8_t node_id;

	/* FUNCTION BODY */
	/* Check if command is ok */
	if (argc < 2) {
		PX4_INFO_RAW("Usage: mscs calibrate %s <calibration_commmand>\n", argv[0]);
	} else {
		/* Check which ADP to calibrate */
		node_id = _get_node_id_from_string(argv[0]);

		/* Check which sensor to calibrate */
		if (argc > 3) {
			PX4_INFO_RAW("\033[1m[MSCS]\033[0m Too many arguments for 'mscs calibrate %s", argv[0]);
		} else {
			if (!strcmp(argv[1], "airspeed")) {
				_adp_airspeed_calibration(node_id);
			} else if (!strcmp(argv[1], "altitude")) {
			} else if (!strcmp(argv[1], "angle_of_attack")) {
				_adp_angle_calibration(node_id, MSCS_ADP_SENSOR_ANGLE_OF_ATTACK);
			} else if (!strcmp(argv[1], "angle_of_sideslip")) {
				_adp_angle_calibration(node_id, MSCS_ADP_SENSOR_ANGLE_OF_SIDESLIP);
			} else {
				PX4_INFO_RAW("\033[1m[MSCS]\033[0m Command \"%s\" not recognized\n", argv[1]);
				return 1;
			}
		}

	}

	return return_value;
}



uint16_t MSCS::_wait_calibration_response(uint16_t node_id, uint64_t timeout_ms) {
	/* VARIABLE DEFINITION */
	const hrt_abstime system_timestamp = hrt_absolute_time();
	uint16_t calibration_error = 0;

	/* FUNCTION BODY */
	/* Poll Response From ADP to start calibration */
	while (1) {
		/* Check Calibration Events */
		if (_mscs_calibration_event_sub.updated()) {
			/* Get Message */
			_mscs_calibration_event_sub.copy(&_mscs_calibration_event_msg);

			/* Make sure event has not already been handled */
			if (_mscs_calibration_event_msg.event_handled == 0) {
				/* Check if message is a calibration OK message */
				if ((_mscs_calibration_event_msg.node_id == MSCS_ID_LEFT_ADP) && (_mscs_calibration_event_msg.event_type == MSCS_CALIBRATION_OK)) {
					/* Publish Event Handled */
					_mscs_calibration_event_msg.event_handled = 1;
					_mscs_calibration_event_pub.publish(_mscs_calibration_event_msg);

					/* Remove Updated Flag */
					_mscs_calibration_event_sub.copy(&_mscs_calibration_event_msg);

					/* Break out of loop */
					break;
				} else if ((_mscs_calibration_event_msg.node_id == MSCS_ID_LEFT_ADP) && (_mscs_calibration_event_msg.event_type == MSCS_CALIBRATION_BAD)) {
					/* Check if we had an overflow */
					if (_mscs_calibration_event_msg.event_data[0] == MSCS_CALIBRATION_OVERFLOW) {
						printf("\033[1m[CALIBRATION ERROR]\033[0m ADC Overflow detected! Change Voltage Divider resistors to stay within range.");
					} else {
						printf("\033[1m[CALIBRATION ERROR]\033[0m Calibration Failed.");
					}

					/* Set Error Flag */
					calibration_error = 1;

					/* Break out of loop */
					break;
				}
			}
		}

		/* Check for Response Timeout */
		if (timeout_ms && (hrt_absolute_time()-system_timestamp) > (timeout_ms*((uint64_t)1e3))) {
			break;
		}
	}

	return calibration_error;
}

uint16_t MSCS::_wait_calibration_connection(uint16_t node_id, uint64_t timeout_ms) {
	/* VARIABLE DEFINITION */
	const hrt_abstime system_timestamp = hrt_absolute_time();
	uint16_t calibration_connection_established = 0;

	/* FUNCTION BODY */
	/* Poll Response From ADP to start calibration */
	while (1) {
		/* Check Calibration Events */
		if (_mscs_calibration_event_sub.updated()) {
			/* Get Message */
			_mscs_calibration_event_sub.copy(&_mscs_calibration_event_msg);

			/* Make sure event has not already been handled */
			if (_mscs_calibration_event_msg.event_handled == 0) {
				/* Check if message is a connection established message */
				if ((_mscs_calibration_event_msg.node_id == node_id) && (_mscs_calibration_event_msg.event_type == MSCS_CALIBRATION_CONNECTION_ESTABLISHED)) {
					/* Publish Event Handled */
					_mscs_calibration_event_msg.event_handled = 1;
					_mscs_calibration_event_pub.publish(_mscs_calibration_event_msg);

					/* Remove Updated Flag */
					_mscs_calibration_event_sub.copy(&_mscs_calibration_event_msg);

					/* Sent calibration connection established flag to 1 */
					calibration_connection_established = 1;

					/* Break out of loop */
					break;
				}
			}
		}

		/* Check for Response Timeout */
		if ((hrt_absolute_time()-system_timestamp) > (timeout_ms*((uint64_t)1e3))) {
			printf("\033[1m[CALIBRATION ERROR]\033[0m Calibration Session could not be established with Remote Node.\n");
			break;
		}
	}

	return calibration_connection_established;
}

void MSCS::_adp_airspeed_calibration_loop(uint16_t node_id) {
	/* VARIABLE DEFINITION */
	struct termios old_termios, new_termios;
	struct timeval timeout;
	char pressed_key;
	fd_set set;

	float ias;
	float static_pressure;
	//float angle_of_attack;
	//float angle_of_sideslip;

	/* FUNCTION BODY */
	/* Save termios state */
    	tcgetattr(STDIN_FILENO, &old_termios);

	/* Configure new termios*/
	new_termios = old_termios;
	new_termios.c_lflag &= ~(ICANON | ECHO);
    	tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);

	/* Set timeout to a value depending on refresh rate so that we get wait time from select() */
	timeout.tv_sec  = 0;
	timeout.tv_usec = MSCS_CALIBRATION_LOOP_REFRESH_PERIOD_US;

	/* Initialize values */
	_mscs_left_adp_ias_data_sub.copy(&_mscs_left_adp_ias_data_msg);
	//_mscs_left_adp_calibrated_temperature_data_sub.copy(&_mscs_left_adp_calibrated_temperature_data_msg);
	_mscs_left_adp_calibrated_static_pressure_data_sub.copy(&_mscs_left_adp_calibrated_static_pressure_data_msg);
	//_mscs_left_adp_calibrated_angle_of_attack_data_sub.copy(&mscs_left_adp_calibrated_static_pressure_data_msg);
	//_mscs_left_adp_calibrated_angle_of_sideslip_data_sub.copy(&_mscs_left_adp_calibrated_angle_of_sideslip_data_msg);

	ias               = _mscs_left_adp_ias_data_msg.data;
	static_pressure   = _mscs_left_adp_calibrated_static_pressure_data_msg.data/1000.f;
	//angle_of_attack   = _mscs_left_adp_calibrated_angle_of_attack_data_msg.data;
	//angle_of_sideslip = _mscs_left_adp_calibrated_angle_of_sideslip_data_msg.data;

	/* Run Introductory Message depending on node */
	switch (node_id)
	{
	case MSCS_ID_LEFT_ADP:
		/* Clear Screen */
		printf("\033[2J");

		/* Print initial system status */
		printf("\n\033[1m****************              LEFT ADP AIRSPEED CALIBRATION              ****************\033[0m\n\n");

		/* Print System Status */
		printf("Indicated Airspeed [IAS]: ");
		_printbar(0, 200, ias, ABS_BAR);
		printf(" ");
		_printfloat(ias);
		printf("m/s\n");

		printf("Static Pressure      [P]: ");
		_printbar(60, 160, static_pressure, ABS_BAR);
		printf(" ");
		_printfloat(static_pressure);
		printf("kPa\n");

		printf("Angle of Attack      [\u03B1]: [                        |==                       ]   2.02deg\n");
		printf("Angle of Sideslip    [\u03B2]: [                        \033[1m|\033[0m                         ]   0.46deg\n");
		printf("\nPress Enter Key to exit calibration.\n");

		/* Calibration Loop */
		while (1) {
			/* Read ADP Data */
			_mscs_left_adp_ias_data_sub.copy(&_mscs_left_adp_ias_data_msg);
			//_mscs_left_adp_calibrated_temperature_data_sub.copy(&_mscs_left_adp_calibrated_temperature_data_msg);
			_mscs_left_adp_calibrated_static_pressure_data_sub.copy(&_mscs_left_adp_calibrated_static_pressure_data_msg);
			//_mscs_left_adp_calibrated_angle_of_attack_data_sub.copy(&mscs_left_adp_calibrated_static_pressure_data_msg);
			//_mscs_left_adp_calibrated_angle_of_sideslip_data_sub.copy(&_mscs_left_adp_calibrated_angle_of_sideslip_data_msg);

			ias               = _mscs_left_adp_ias_data_msg.data;
			static_pressure   = _mscs_left_adp_calibrated_static_pressure_data_msg.data/1000.f;
			//angle_of_attack   = _mscs_left_adp_calibrated_angle_of_attack_data_msg.data;
			//angle_of_sideslip = _mscs_left_adp_calibrated_angle_of_sideslip_data_msg.data;

			/* Move cursor up */
			printf("\033[6A");

			/* Print System Status */
			printf("Indicated Airspeed [IAS]: ");
			_printbar(0, 200, ias, ABS_BAR);
			printf(" ");
			_printfloat(ias);
			printf("m/s\n");

			printf("Static Pressure      [P]: ");
			_printbar(60, 160, static_pressure, ABS_BAR);
			printf(" ");
			_printfloat(static_pressure);
			printf("kPa\n");

			printf("Angle of Attack      [\u03B1]: [                        |==                       ]   2.02deg\n");
			printf("Angle of Sideslip    [\u03B2]: [                        \033[1m|\033[0m                         ]   0.46deg\n");
			printf("\nPress Enter Key to exit calibration.\n");

			/* IO Multiplexing */
			FD_ZERO(&set);
			FD_SET(STDIN_FILENO, &set);

			/* Check User Input*/
			if (select(STDIN_FILENO + 1, &set, NULL, NULL, &timeout) > 0) {

				read(STDIN_FILENO, &pressed_key, 1);

				// Exit loop if ENTER Key is pressed
				if (pressed_key == ENTER_KEY) {
					printf("\n");
					break;
				}
			}

		}

		break;

	case MSCS_ID_RIGHT_ADP:
		printf("\n****  AIRSPEED CALIBRATION STARTED FOR RIGHT ADP  ****\n");
		break;

	default:
		break;
	}

	/* Reset termios */
    	tcsetattr(STDIN_FILENO, TCSANOW, &old_termios);
}


void MSCS::_adp_airspeed_calibration(uint16_t node_id) {
	/* VARIABLE DEFINITION */

	/* FUNCTION BODY */
	/* Read System Status */
	_mscs_status_sub.copy(&_mscs_status);

	/* Run calibration depending on target node */
	switch (node_id)
	{
		case MSCS_ID_LEFT_ADP:
			if (!_mscs_status.left_adp_active) {
				PX4_INFO_RAW("\033[1m[MSCS]\033[0m Left ADP not found in Bus!\n");
			} else {
				/* Set Status as Calibration */
				_mscs_status.state = MSCS_STATE_CALIBRATION;
				_mscs_status_pub.publish(_mscs_status);

				/* Configure ADP for airspeed calibration */
				_left_adp_config.data_publishing_rate 			  = 0;
				_left_adp_config.pressure_data_acquisition_rate           = 0;
				_left_adp_config.angle_data_acquisition_rate              = 2;
				_left_adp_config.airspeed_divisor			  = 1;
				_left_adp_config.altitude_divisor                         = 1;
				_left_adp_config.data_publishing_mode_raw                 = 0;
				_left_adp_config.data_publishing_mode_calibrated          = 1;
				_left_adp_config.differential_pressure_output_enable      = 0;
				_left_adp_config.static_pressure_output_enable            = 1;
				_left_adp_config.angle_of_attack_output_enable            = 1;
				_left_adp_config.angle_of_sideslip_output_enable          = 1;
				_left_adp_config.temperature_output_enable                = 1;
				_left_adp_config.pressure_altitude_output_enable 	  = 0;
				_left_adp_config.density_altitude_output_enable 	  = 0;
				_left_adp_config.msl_altitude_output_enable 		  = 0;
				_left_adp_config.agl_altitude_output_enable 		  = 0;
				_left_adp_config.ias_output_enable 			  = 1;
				_left_adp_config.cas_output_enable 			  = 0;
				_left_adp_config.tas_output_enable  			  = 0;

				/* Configure ADP */
				_config_node(node_id);
				start_data_acquisition(node_id);

				key_event(KEY_EVENTS_PRIORITY_HIGH,
					KEY_EVENTS_MSCS_CALIBRATION_START,
					KEY_EVENTS_DEVICE_LEFT_ADP,
					"Airspeed Calibration Started for Left ADP");

				/* Run Calibration Loop */
				_adp_airspeed_calibration_loop(MSCS_ID_LEFT_ADP);

				/* Set Node Back to Previous Configuration */
				(void) _load_device_config(MSCS_ID_LEFT_ADP);
				_config_node(node_id);

				key_event(KEY_EVENTS_PRIORITY_HIGH,
					KEY_EVENTS_MSCS_CALIBRATION_END,
					KEY_EVENTS_DEVICE_LEFT_ADP,
					"Airspeed Calibration Ended for Left ADP");
			}

			break;
		case MSCS_ID_RIGHT_ADP:
			if (!_mscs_status.right_adp_active) {
				PX4_INFO_RAW("\033[1m[MSCS]\033[0m Right ADP not found in Bus!\n");
			} else {
				/* Set Status as Calibration */
				_mscs_status_sub.copy(&_mscs_status);
				_mscs_status.state = MSCS_STATE_CALIBRATION;
				_mscs_status_pub.publish(_mscs_status);

				/* Configure ADP for airspeed calibration */
				_right_adp_config.data_publishing_rate 			  = 0;
				_right_adp_config.pressure_data_acquisition_rate          = 0;
				_right_adp_config.angle_data_acquisition_rate             = 2;
				_right_adp_config.airspeed_divisor			  = 1;
				_right_adp_config.altitude_divisor                        = 1;
				_right_adp_config.data_publishing_mode_raw                = 0;
				_right_adp_config.data_publishing_mode_calibrated         = 1;
				_right_adp_config.differential_pressure_output_enable     = 0;
				_right_adp_config.static_pressure_output_enable           = 1;
				_right_adp_config.angle_of_attack_output_enable           = 1;
				_right_adp_config.angle_of_sideslip_output_enable         = 1;
				_right_adp_config.temperature_output_enable               = 1;
				_right_adp_config.pressure_altitude_output_enable 	  = 0;
				_right_adp_config.density_altitude_output_enable 	  = 0;
				_right_adp_config.msl_altitude_output_enable 		  = 0;
				_right_adp_config.agl_altitude_output_enable 		  = 0;
				_right_adp_config.ias_output_enable 			  = 1;
				_right_adp_config.cas_output_enable 			  = 0;
				_right_adp_config.tas_output_enable  			  = 0;

				/* Configure ADP */
				_config_node(node_id);
				start_data_acquisition(node_id);

				key_event(KEY_EVENTS_PRIORITY_HIGH,
					KEY_EVENTS_MSCS_CALIBRATION_START,
					KEY_EVENTS_DEVICE_RIGHT_ADP,
					"Airspeed Calibration Started for Right ADP");

				/* Run Calibration Loop */
				_adp_airspeed_calibration_loop(MSCS_ID_LEFT_ADP);

				/* Set Node Back to Previous Configuration */
				(void) _load_device_config(MSCS_ID_RIGHT_ADP);
				_config_node(node_id);

				key_event(KEY_EVENTS_PRIORITY_HIGH,
					KEY_EVENTS_MSCS_CALIBRATION_END,
					KEY_EVENTS_DEVICE_RIGHT_ADP,
					"Airspeed Calibration Ended for Right ADP");
			}
			break;

		default:
			break;
	}
}

void MSCS::_adp_altitude_calibration(uint16_t node_id) {
	/* TO DO */
}

void MSCS::_adp_angle_calibration(uint16_t node_id, uint16_t angle) {
	/* VARIABLE DEFINITION */
	uint16_t calibration_connection_established = 0;
	uint16_t calibration_error = 0;
	uint16_t adp_active  = 1;
	uint16_t angle_valid = 0;
	char adp_name[16];
	char angle_name[32];

	/* FUNCTION BODY */
	/* Clear Names */
	memset(adp_name, '\0', sizeof(adp_name));
	memset(angle_name, '\0', sizeof(angle_name));

	/* Read System Status */
	_mscs_status_sub.copy(&_mscs_status);

	/* Run calibration depending on target node */
	switch (node_id)
	{
		case MSCS_ID_LEFT_ADP:
			if (!_mscs_status.left_adp_active) {
				PX4_INFO_RAW("\033[1m[MSCS]\033[0m Left ADP not found in Bus!\n");
				adp_active = 0;
			} else {
				strncpy(adp_name, "Left ADP", sizeof(adp_name));
			}
			break;

		case MSCS_ID_RIGHT_ADP:
			if (!_mscs_status.right_adp_active) {
				PX4_INFO_RAW("\033[1m[MSCS]\033[0m Right ADP not found in Bus!\n");
				adp_active = 0;
			} else {
				strncpy(adp_name, "Left ADP", sizeof(adp_name));
			}
			break;

		default:
			break;
	}

	/* Decode which angle we are calibrating */
	switch (angle)
	{
		case MSCS_ADP_SENSOR_ANGLE_OF_ATTACK:
			strncpy(angle_name, "Angle of Attack", sizeof(angle_name));
			angle_valid = 1;
			break;

		case MSCS_ADP_SENSOR_ANGLE_OF_SIDESLIP:
			strncpy(angle_name, "Angle of Sideslip", sizeof(angle_name));
			angle_valid = 1;
			break;

		default:
			break;
	}

	/* Check if ADP is active */
	if (adp_active && angle_valid) {
		/* Set Status as Calibration */
		_mscs_status.state = MSCS_STATE_CALIBRATION;
		_mscs_status_pub.publish(_mscs_status);

		/* Send command to ADP to start angle calibration */
		if (angle == MSCS_ADP_SENSOR_ANGLE_OF_ATTACK) {
			_start_angle_of_attack_calibration(node_id);
		} else {
			_start_angle_of_sideslip_calibration(node_id);
		}

		/* Wait for Calibration Connection to be established */
		calibration_connection_established = _wait_calibration_connection(node_id, CALIBRATION_RESPONSE_TIMEOUT_MS);

		/* Check if calibration connection has been established */
		if (calibration_connection_established) {
			/* STAGE 1 */
			switch (node_id)
			{
				case MSCS_ID_LEFT_ADP:
					if (angle == MSCS_ADP_SENSOR_ANGLE_OF_ATTACK) {
						key_event(KEY_EVENTS_PRIORITY_HIGH,
							KEY_EVENTS_MSCS_CALIBRATION_START,
							KEY_EVENTS_DEVICE_LEFT_ADP,
							"Angle of Attack Calibration Started for Left ADP");
						printf("\n\033[1m****************          LEFT ADP ANGLE OF ATTACK CALIBRATION           ****************\033[0m\n\n");
					} else {
						key_event(KEY_EVENTS_PRIORITY_HIGH,
							KEY_EVENTS_MSCS_CALIBRATION_START,
							KEY_EVENTS_DEVICE_LEFT_ADP,
							"Angle of Sideslip Calibration Started for Left ADP");
						printf("\n\033[1m****************         LEFT ADP ANGLE OF SIDESLIP CALIBRATION          ****************\033[0m\n\n");
					}
					break;
				case MSCS_ID_RIGHT_ADP:
					if (angle == MSCS_ADP_SENSOR_ANGLE_OF_ATTACK) {
						key_event(KEY_EVENTS_PRIORITY_HIGH,
							KEY_EVENTS_MSCS_CALIBRATION_START,
							KEY_EVENTS_DEVICE_LEFT_ADP,
							"Angle of Attack Calibration Started for Right ADP");
						printf("\n\033[1m****************          RIGHT ADP ANGLE OF ATTACK CALIBRATION          ****************\033[0m\n\n");
					} else {
						key_event(KEY_EVENTS_PRIORITY_HIGH,
							KEY_EVENTS_MSCS_CALIBRATION_START,
							KEY_EVENTS_DEVICE_LEFT_ADP,
							"Angle of Sideslip Calibration Started for Right ADP");
						printf("\n\033[1m****************         RIGHT ADP ANGLE OF SIDESLIP CALIBRATION         ****************\033[0m\n\n");
					}
					break;

				default:
					break;
			}

			printf("\033[1m[1]\033[0m Slowly Rotate %s Vane in the CLOCKWISE direction.", angle_name);

			calibration_error = _wait_calibration_response(MSCS_ID_LEFT_ADP, 0);

			/* If there has been a calibration error, exit */
			if (calibration_error) {
				return;
			}

			/* STAGE 2 */
			printf("\033[1m[2]\033[0m Slowly Rotate %s Vane in the COUNTERCLOCKWISE direction.", angle_name);

			calibration_error = _wait_calibration_response(MSCS_ID_LEFT_ADP, 0);

			/* If there has been a calibration error, exit */
			if (calibration_error) {
				return;
			}

			/* STAGE 3 */
			printf("\033[1m[3]\033[0m Set %s Vane to   0deg and press the ENTER key.", angle_name);

			/* Wait on ENTER */
			_wait_keypress(ENTER_KEY, 0);

			/* Wait for Confirmation from ADP */
			calibration_error = _wait_calibration_response(MSCS_ID_LEFT_ADP, 0);

			/* If there has been a calibration error, exit */
			if (calibration_error) {
				return;
			}

			/* STAGE 4 */
			printf("\033[1m[4]\033[0m Set %s Vane to  +5deg and press the ENTER key.", angle_name);

			/* Wait on ENTER */
			_wait_keypress(ENTER_KEY, 0);

			/* Wait for Confirmation from ADP */
			calibration_error = _wait_calibration_response(MSCS_ID_LEFT_ADP, 0);

			/* If there has been a calibration error, exit */
			if (calibration_error) {
				return;
			}

			/* STAGE 5 */
			printf("\033[1m[5]\033[0m Set %s Vane to  -5deg and press the ENTER key.", angle_name);

			/* Wait on ENTER */
			_wait_keypress(ENTER_KEY, 0);

			/* Wait for Confirmation from ADP */
			calibration_error = _wait_calibration_response(MSCS_ID_LEFT_ADP, 0);

			/* If there has been a calibration error, exit */
			if (calibration_error) {
				return;
			}

			/* STAGE 6 */
			printf("\033[1m[6]\033[0m Set %s Vane to +15deg and press the ENTER key.", angle_name);

			/* Wait on ENTER */
			_wait_keypress(ENTER_KEY, 0);

			/* Wait for Confirmation from ADP */
			calibration_error = _wait_calibration_response(MSCS_ID_LEFT_ADP, 0);

			/* If there has been a calibration error, exit */
			if (calibration_error) {
				return;
			}

			/* STAGE 7 */
			printf("\033[1m[7]\033[0m Set %s Vane to -15deg and press the ENTER key.", angle_name);

			/* Wait on ENTER */
			_wait_keypress(ENTER_KEY, 0);

			/* Wait for Confirmation from ADP */
			calibration_error = _wait_calibration_response(MSCS_ID_LEFT_ADP, 0);

			/* If there has been a calibration error, exit */
			if (calibration_error) {
				return;
			}

			/* STAGE 8 */
			printf("\033[1m[8]\033[0m Set %s Vane to +35deg and press the ENTER key.", angle_name);

			/* Wait on ENTER */
			_wait_keypress(ENTER_KEY, 0);

			/* Wait for Confirmation from ADP */
			calibration_error = _wait_calibration_response(MSCS_ID_LEFT_ADP, 0);

			/* If there has been a calibration error, exit */
			if (calibration_error) {
				return;
			}

			/* STAGE 9 */
			printf("\033[1m[9]\033[0m Set %s Vane to -35deg and press the ENTER key.", angle_name);

			/* Wait on ENTER */
			_wait_keypress(ENTER_KEY, 0);

			/* Wait for Confirmation from ADP */
			calibration_error = _wait_calibration_response(MSCS_ID_LEFT_ADP, 0);

			/* If there has been a calibration error, exit */
			if (calibration_error) {
				return;
			}

			/* STAGE 10 */
			printf("\033[1m[10]\033[0m Set %s Vane to +65deg and press the ENTER key.", angle_name);

			/* Wait on ENTER */
			_wait_keypress(ENTER_KEY, 0);

			/* Wait for Confirmation from ADP */
			calibration_error = _wait_calibration_response(MSCS_ID_LEFT_ADP, 0);

			/* If there has been a calibration error, exit */
			if (calibration_error) {
				return;
			}

			/* STAGE 11 */
			printf("\033[1m[11]\033[0m Set %s Vane to -65deg and press the ENTER key.", angle_name);

			/* Wait on ENTER */
			_wait_keypress(ENTER_KEY, 0);

			/* Wait for Confirmation from ADP */
			calibration_error = _wait_calibration_response(MSCS_ID_LEFT_ADP, 0);

			/* If there has been a calibration error, exit */
			if (calibration_error) {
				return;
			}

			/* STAGE 13 */
			printf("\n\033[1m****************                  CALIBRATION  COMPLETE                  ****************\033[0m\n\n");

		}

		switch (node_id)
		{
			case MSCS_ID_LEFT_ADP:
				if (angle == MSCS_ADP_SENSOR_ANGLE_OF_ATTACK) {
					key_event(KEY_EVENTS_PRIORITY_HIGH,
						KEY_EVENTS_MSCS_CALIBRATION_START,
						KEY_EVENTS_DEVICE_LEFT_ADP,
						"Angle of Attack Calibration Ended for Left ADP");
				} else {
					key_event(KEY_EVENTS_PRIORITY_HIGH,
						KEY_EVENTS_MSCS_CALIBRATION_START,
						KEY_EVENTS_DEVICE_LEFT_ADP,
						"Angle of Sideslip Calibration Ended for Left ADP");
				}
				break;
			case MSCS_ID_RIGHT_ADP:
				if (angle == MSCS_ADP_SENSOR_ANGLE_OF_ATTACK) {
					key_event(KEY_EVENTS_PRIORITY_HIGH,
						KEY_EVENTS_MSCS_CALIBRATION_START,
						KEY_EVENTS_DEVICE_LEFT_ADP,
						"Angle of Attack Calibration Ended for Right ADP");
				} else {
					key_event(KEY_EVENTS_PRIORITY_HIGH,
						KEY_EVENTS_MSCS_CALIBRATION_START,
						KEY_EVENTS_DEVICE_LEFT_ADP,
						"Angle of Sideslip Calibration Ended for Right ADP");
				}
				break;

			default:
				break;
		}
	}
}

int MSCS::list_command_handler(int argc, char *argv[]) {
	/* VARIABLE DEFINITIONS */
	int return_value = 0;

	/* FUNCTION BODY */
	if (_instance != nullptr) {
		return_value = _instance->list_command(argc, argv);
	} else {
		PX4_INFO_RAW("MSCS has not yet been initialized!\n");
	}

	return return_value;
}

int MSCS::start_data_acquisition_command(int argc, char *argv[]) {
	/* VARIABLE DEFINITION */
	//const hrt_abstime system_timestamp = hrt_absolute_time();
	uint8_t string_iterator_index = 0;
	uint8_t device_index = 0;
	uint8_t device_iterator_index ;
	uint8_t device_array[MAX_COMMAND_DEVICES];
	int8_t current_device_id;

	/* FUNCTION BODY*/
	/* Decode node */
	if (argc < 1) {
		PX4_INFO_RAW("Usage: mscs command start_data_acquisition_command <device_name>  [...]\n");
		PX4_INFO_RAW("Device not specified!\n");
	} else {
		for (device_index = 0; string_iterator_index < argc; string_iterator_index++) {
			current_device_id = _get_node_id_from_string(argv[string_iterator_index]);
			if (current_device_id == 0) {
				PX4_INFO_RAW("Device name \"%s\" not recognized. Use 'mscs list' to see a list of available nodes.\n", argv[string_iterator_index]);
			} else {
				device_array[device_index++] = current_device_id;
				if (device_index >= MAX_COMMAND_DEVICES) {
					break;
				}
			}
		}

		/* Update MSCS status */
		_mscs_status_sub.copy(&_mscs_status);

		/* Loop through device array */
		for (device_iterator_index = 0; device_iterator_index < device_index; device_iterator_index++) {
			switch (device_array[device_iterator_index])
			{
			case MSCS_ID_LEFT_ADP:
				/* Check if device is active */
				if (_mscs_status.left_adp_active) {
					/* Send Start command to Device */
					start_data_acquisition(_mscs_status.left_adp_node_id);

					key_event(KEY_EVENTS_PRIORITY_HIGH,
						KEY_EVENTS_MSCS_COMMAND_SENT,
						KEY_EVENTS_DEVICE_FLIGHT_CONTROLLER,
						"DATA_ACQUISITION_START Command Sent to Left ADP from CLI");
				} else {
					PX4_INFO_RAW("Left ADP not found in Bus!\n");
				}
				break;
			case MSCS_ID_RIGHT_ADP:
				/* Check if device is active */
				if (_mscs_status.right_adp_active) {
					/* Send Start command to Device */
					start_data_acquisition(_mscs_status.right_adp_node_id);

					key_event(KEY_EVENTS_PRIORITY_HIGH,
						KEY_EVENTS_MSCS_COMMAND_SENT,
						KEY_EVENTS_DEVICE_FLIGHT_CONTROLLER,
						"DATA_ACQUISITION_START Command Sent to Right ADP from CLI");
				} else {
					PX4_INFO_RAW("Right ADP not found in Bus!\n");
				}
				break;

			default:
				break;
			}
		}
	}

	return 0;
};

int MSCS::stop_data_acquisition_command(int argc, char *argv[]) {
	/* VARIABLE DEFINITION */
	//const hrt_abstime system_timestamp = hrt_absolute_time();
	uint8_t string_iterator_index = 0;
	uint8_t device_index = 0;
	uint8_t device_iterator_index ;
	uint8_t device_array[MAX_COMMAND_DEVICES];
	int8_t current_device_id;

	/* FUNCTION BODY*/
	/* Decode node */
	if (argc < 1) {
		PX4_INFO_RAW("Usage: mscs command stop_data_acquisition_command <device_name> [...]\n");
		PX4_INFO_RAW("Device not specified!\n");
	} else {
		for (device_index = 0; string_iterator_index < argc; string_iterator_index++) {
			current_device_id = _get_node_id_from_string(argv[string_iterator_index]);
			if (current_device_id == 0) {
				PX4_INFO_RAW("Device name \"%s\" not recognized. Use 'mscs list' to see a list of available nodes.\n", argv[string_iterator_index]);
			} else {
				device_array[device_index++] = current_device_id;
				if (device_index >= MAX_COMMAND_DEVICES) {
					break;
				}
			}
		}

		/* Update MSCS status */
		_mscs_status_sub.copy(&_mscs_status);

		/* Loop through device array */
		for (device_iterator_index = 0; device_iterator_index < device_index; device_iterator_index++) {
			switch (device_array[device_iterator_index])
			{
			case MSCS_ID_LEFT_ADP:
				/* Check if device is active */
				if (_mscs_status.left_adp_active) {
					/* Send Stop command to Device */
					stop_data_acquisition(_mscs_status.left_adp_node_id);
				} else {
					PX4_INFO_RAW("Left ADP not found in Bus!\n");
				}

				key_event(KEY_EVENTS_PRIORITY_HIGH,
					KEY_EVENTS_MSCS_COMMAND_SENT,
					KEY_EVENTS_DEVICE_FLIGHT_CONTROLLER,
					"DATA_ACQUISITION_STOP Command Sent to Left ADP from CLI");
				break;
			case MSCS_ID_RIGHT_ADP:
				/* Check if device is active */
				if (_mscs_status.right_adp_active) {
					/* Send Stop command to Device */
					stop_data_acquisition(_mscs_status.right_adp_node_id);
				} else {
					PX4_INFO_RAW("Right ADP not found in Bus!\n");
				}

				key_event(KEY_EVENTS_PRIORITY_HIGH,
					KEY_EVENTS_MSCS_COMMAND_SENT,
					KEY_EVENTS_DEVICE_FLIGHT_CONTROLLER,
					"DATA_ACQUISITION_STOP Command Sent to Right ADP from CLI");
				break;

			default:
				break;
			}
		}
	}

	return 0;
};

int MSCS::list_command(int argc, char *argv[]) {
	/* VARIABLE DEFINITIONS */
	int return_value = 0;

	/* FUNCTION BODY */
	/* Update MSCS status */
	_mscs_status_sub.copy(&_mscs_status);

	/* Print Left ADP */
	PX4_INFO_RAW("     [%hhu] Left  ADP - 'left_adp'            ", _mscs_status.left_adp_node_id);
	if (_mscs_status.left_adp_active) {
		/* Send Start command to Device */
		PX4_INFO_RAW("[ACTIVE]\n");
	} else {
		PX4_INFO_RAW("     [X]\n");
	}

	/* Print Right ADP */
	PX4_INFO_RAW("     [%hhu] Right ADP - 'right_adp'           ", _mscs_status.right_adp_node_id);
	if (_mscs_status.right_adp_active) {
		/* Send Start command to Device */
		PX4_INFO_RAW("[ACTIVE]\n");
	} else {
		PX4_INFO_RAW("     [X]\n");
	}

	return return_value;
}

void MSCS::start_data_acquisition(uint16_t node_id) {
	/* VARIABLE DEFINITION */
	const hrt_abstime system_timestamp = hrt_absolute_time();
	mscs_command_transaction_s mscs_command_transaction_msg{0};

	/* FUNCTION BODY */
	/* Build Command Transaction Message */
	mscs_command_transaction_msg.node_id            = node_id;
	mscs_command_transaction_msg.timestamp          = system_timestamp;
	mscs_command_transaction_msg.command            = MSCS_COMMAND_DATA_ACQUISITION_START;
	mscs_command_transaction_msg.command_timeout_ms = 1000;

	/* Push Command to queue */
	_command_queue->push(mscs_command_transaction_msg);

	/* Handle Commmand */
	_command_handler();
};

void MSCS::_start_angle_of_attack_calibration(uint16_t node_id) {
	/* VARIABLE DEFINITION */
	const hrt_abstime system_timestamp = hrt_absolute_time();
	mscs_command_transaction_s mscs_command_transaction_msg{0};

	/* FUNCTION BODY */
	/* Build Command Transaction Message */
	mscs_command_transaction_msg.node_id            = node_id;
	mscs_command_transaction_msg.timestamp          = system_timestamp;
	mscs_command_transaction_msg.command            = MSCS_COMMAND_DATA_CALIBRATION_START;
	mscs_command_transaction_msg.command_args[0]    = MSCS_ADP_SENSOR_ANGLE_OF_ATTACK;
	mscs_command_transaction_msg.command_timeout_ms = 1000;

	/* Push Command to queue */
	_command_queue->push(mscs_command_transaction_msg);

	/* Handle Commmand */
	_command_handler();
};

void MSCS::_start_angle_of_sideslip_calibration(uint16_t node_id) {
	/* VARIABLE DEFINITION */
	const hrt_abstime system_timestamp = hrt_absolute_time();
	mscs_command_transaction_s mscs_command_transaction_msg{0};

	/* FUNCTION BODY */
	/* Build Command Transaction Message */
	mscs_command_transaction_msg.node_id            = node_id;
	mscs_command_transaction_msg.timestamp          = system_timestamp;
	mscs_command_transaction_msg.command            = MSCS_COMMAND_DATA_CALIBRATION_START;
	mscs_command_transaction_msg.command_args[0]    = MSCS_ADP_SENSOR_ANGLE_OF_SIDESLIP;
	mscs_command_transaction_msg.command_timeout_ms = 1000;

	/* Push Command to queue */
	_command_queue->push(mscs_command_transaction_msg);

	/* Handle Commmand */
	_command_handler();
};

void MSCS::_submit_calibration(uint16_t node_id) {
	/* VARIABLE DEFINITION */
	const hrt_abstime system_timestamp = hrt_absolute_time();
	mscs_command_transaction_s mscs_command_transaction_msg{0};

	/* FUNCTION BODY */
	/* Build Command Transaction Message */
	mscs_command_transaction_msg.node_id            = node_id;
	mscs_command_transaction_msg.timestamp          = system_timestamp;
	mscs_command_transaction_msg.command            = MSCS_COMMAND_DATA_CALIBRATION_SUBMIT;
	mscs_command_transaction_msg.command_timeout_ms = 1000;

	/* Push Command to queue */
	_command_queue->push(mscs_command_transaction_msg);

	/* Handle Commmand */
	_command_handler();
};

void MSCS::stop_data_acquisition(uint16_t node_id) {
	/* VARIABLE DEFINITION */
	const hrt_abstime system_timestamp = hrt_absolute_time();
	mscs_command_transaction_s mscs_command_transaction_msg{0};

	/* FUNCTION BODY */
	/* Build Command Transaction Message */
	mscs_command_transaction_msg.node_id            = node_id;
	mscs_command_transaction_msg.timestamp          = system_timestamp;
	mscs_command_transaction_msg.command            = MSCS_COMMAND_DATA_ACQUISITION_STOP;
	mscs_command_transaction_msg.command_timeout_ms = 1000;

	/* Push Command to queue */
	_command_queue->push(mscs_command_transaction_msg);

	/* Handle Commmand */
	_command_handler();
};

void MSCS::_config_node(uint16_t node_id) {
	/* VARIABLE DEFINITION */
	const hrt_abstime system_timestamp = hrt_absolute_time();
	mscs_command_transaction_s mscs_command_transaction_msg{0};

	/* FUNCTION BODY */
	/* Setup Command Transaction */
	mscs_command_transaction_msg.node_id            = node_id;
	mscs_command_transaction_msg.timestamp          = system_timestamp;
	mscs_command_transaction_msg.command            = MSCS_COMMAND_DATA_ACQUISITION_CONFIG;
	mscs_command_transaction_msg.command_timeout_ms = 1000;

	/* Assemble Configuration from struct */
	switch (node_id)
	{
	case MSCS_ID_LEFT_ADP:
		/* Configuration Page 0 */
		mscs_command_transaction_msg.command_args[0]  = 0;

		/* Data Publishing Mode and Rate*/
		mscs_command_transaction_msg.command_args[1]  = ((_left_adp_config.data_publishing_rate                      & 0x0F));
		mscs_command_transaction_msg.command_args[1] |= ((_left_adp_config.data_publishing_mode_raw                  & 0x01) << 4);
		mscs_command_transaction_msg.command_args[1] |= ((_left_adp_config.data_publishing_mode_calibrated           & 0x01) << 5);

		/* Data Acquisition Rate for Pressure Sensors */
		mscs_command_transaction_msg.command_args[2]  = ((_left_adp_config.pressure_data_acquisition_rate            & 0x0F));
		/* Data Acquisition Rate for Angle Sensors */
		mscs_command_transaction_msg.command_args[2] |= ((_left_adp_config.angle_data_acquisition_rate               & 0x0F) << 4);

		/* Sensor Publication Mask */
		mscs_command_transaction_msg.command_args[3]  = ((_left_adp_config.differential_pressure_output_enable       & 0x01));
		mscs_command_transaction_msg.command_args[3] |= ((_left_adp_config.static_pressure_output_enable             & 0x01) << 1);
		mscs_command_transaction_msg.command_args[3] |= ((_left_adp_config.angle_of_attack_output_enable             & 0x01) << 2);
		mscs_command_transaction_msg.command_args[3] |= ((_left_adp_config.angle_of_sideslip_output_enable           & 0x01) << 3);
		mscs_command_transaction_msg.command_args[3] |= ((_left_adp_config.temperature_output_enable                 & 0x01) << 4);

		/* Push to command queue */
		_command_queue->push(mscs_command_transaction_msg);

		/* Handle Command */
		_command_handler();

		/* Configuration Page 1 */
		mscs_command_transaction_msg.command_args[0]  = 1;

		/* Airspeed Publication Mask */
		mscs_command_transaction_msg.command_args[1]  = ((_left_adp_config.ias_output_enable                         & 0x01));
		mscs_command_transaction_msg.command_args[1] |= ((_left_adp_config.cas_output_enable                         & 0x01) << 1);
		mscs_command_transaction_msg.command_args[1] |= ((_left_adp_config.tas_output_enable                         & 0x01) << 2);

		/* Altitude Publication Mask */
		mscs_command_transaction_msg.command_args[2]  = ((_left_adp_config.pressure_altitude_output_enable           & 0x01));
		mscs_command_transaction_msg.command_args[2] |= ((_left_adp_config.density_altitude_output_enable            & 0x01) << 1);
		mscs_command_transaction_msg.command_args[2] |= ((_left_adp_config.agl_altitude_output_enable                & 0x01) << 2);
		mscs_command_transaction_msg.command_args[2] |= ((_left_adp_config.msl_altitude_output_enable                & 0x01) << 3);

		/* Airspeed and Altitude Publication Divisor  */
		mscs_command_transaction_msg.command_args[3]  = ((_left_adp_config.airspeed_divisor                          & 0x0F));
		mscs_command_transaction_msg.command_args[3] |= ((_left_adp_config.altitude_divisor                          & 0x0F) << 4);

		/* Push to command queue */
		_command_queue->push(mscs_command_transaction_msg);

		/* Handle Command */
		_command_handler();

		key_event(KEY_EVENTS_PRIORITY_DEBUG,
			KEY_EVENTS_MSCS_COMMAND_SENT,
			KEY_EVENTS_DEVICE_FLIGHT_CONTROLLER,
			"DATA_ACQUISITION_CONFIG Command Sent to Left ADP");

		break;

	case MSCS_ID_RIGHT_ADP:
		/* Configuration Page 0 */
		mscs_command_transaction_msg.command_args[0]  = 0;

		/* Data Publishing Mode and Rate*/
		mscs_command_transaction_msg.command_args[1]  = ((_right_adp_config.data_publishing_rate                      & 0x0F));
		mscs_command_transaction_msg.command_args[1] |= ((_right_adp_config.data_publishing_mode_raw                  & 0x01) << 4);
		mscs_command_transaction_msg.command_args[1] |= ((_right_adp_config.data_publishing_mode_calibrated           & 0x01) << 5);

		/* Data Acquisition Rate for Pressure Sensors */
		mscs_command_transaction_msg.command_args[2]  = ((_right_adp_config.pressure_data_acquisition_rate            & 0x0F));
		/* Data Acquisition Rate for Angle Sensors */
		mscs_command_transaction_msg.command_args[2] |= ((_right_adp_config.angle_data_acquisition_rate               & 0x0F) << 4);

		/* Sensor Publication Mask */
		mscs_command_transaction_msg.command_args[3]  = ((_right_adp_config.differential_pressure_output_enable       & 0x01));
		mscs_command_transaction_msg.command_args[3] |= ((_right_adp_config.static_pressure_output_enable             & 0x01) << 1);
		mscs_command_transaction_msg.command_args[3] |= ((_right_adp_config.angle_of_attack_output_enable             & 0x01) << 2);
		mscs_command_transaction_msg.command_args[3] |= ((_right_adp_config.angle_of_sideslip_output_enable           & 0x01) << 3);
		mscs_command_transaction_msg.command_args[3] |= ((_right_adp_config.temperature_output_enable                 & 0x01) << 4);

		/* Push to command queue */
		_command_queue->push(mscs_command_transaction_msg);

		/* Handle Command */
		_command_handler();

		/* Configuration Page 1 */
		mscs_command_transaction_msg.command_args[0]  = 1;

		/* Airspeed Publication Mask */
		mscs_command_transaction_msg.command_args[1]  = ((_right_adp_config.ias_output_enable                         & 0x01));
		mscs_command_transaction_msg.command_args[1] |= ((_right_adp_config.cas_output_enable                         & 0x01) << 1);
		mscs_command_transaction_msg.command_args[1] |= ((_right_adp_config.tas_output_enable                         & 0x01) << 2);

		/* Altitude Publication Mask */
		mscs_command_transaction_msg.command_args[2]  = ((_right_adp_config.pressure_altitude_output_enable           & 0x01));
		mscs_command_transaction_msg.command_args[2] |= ((_right_adp_config.density_altitude_output_enable            & 0x01) << 1);
		mscs_command_transaction_msg.command_args[2] |= ((_right_adp_config.agl_altitude_output_enable                & 0x01) << 2);
		mscs_command_transaction_msg.command_args[2] |= ((_right_adp_config.msl_altitude_output_enable                & 0x01) << 3);

		/* Airspeed and Altitude Publication Divisor  */
		mscs_command_transaction_msg.command_args[3]  = ((_right_adp_config.airspeed_divisor                          & 0x0F));
		mscs_command_transaction_msg.command_args[3] |= ((_right_adp_config.altitude_divisor                          & 0x0F) << 4);

		/* Push to command queue */
		_command_queue->push(mscs_command_transaction_msg);

		/* Handle Command */
		_command_handler();

		key_event(KEY_EVENTS_PRIORITY_DEBUG,
			KEY_EVENTS_MSCS_COMMAND_SENT,
			KEY_EVENTS_DEVICE_FLIGHT_CONTROLLER,
			"DATA_ACQUISITION_CONFIG Command Sent to Right ADP");
		break;

	default:
		break;
	}

};

static void usage()
{

	PX4_INFO_RAW("\n### Description\nThis module implements the Aircraft Lab's Modular Sensor and Computing System.\n\n");
	PX4_INFO_RAW("  start                                             Start new MSCS Instance\n\n");
	PX4_INFO_RAW("  stop                                              Stop MSCS Instance\n\n");
	PX4_INFO_RAW("  status                                            Print MSCS Status\n\n");
	PX4_INFO_RAW("  command <command>                                 Send command to specific device/devices\n");
	PX4_INFO_RAW("      start_data_acquisition <device_name> [...]    Start data acquisition on specific devices\n");
	PX4_INFO_RAW("      stop_data_acquisition  <device_name> [...]    Stop data acquisition on specific devices\n\n");
	PX4_INFO_RAW("  calibrate <device_name>                           Calibrate specific device\n");
	PX4_INFO_RAW("      left_adp <calibration_commmand>	          Calibrate specific variable in Left ADP\n");
	PX4_INFO_RAW("          airspeed 		    	          Calibrate Airspeed\n");
	PX4_INFO_RAW("          altitude 		    	          Calibrate Altitude\n");
	PX4_INFO_RAW("          angle_of_attack        		          Calibrate Angle of Attack\n");
	PX4_INFO_RAW("          angle_of_sideslip    		          Calibrate Angle of Attack\n");
	PX4_INFO_RAW("      right_adp <calibration_commmand>	          Calibrate specific variable in Right ADP\n");
	PX4_INFO_RAW("          airspeed			          Calibrate Airspeed\n");
	PX4_INFO_RAW("          altitude 		                  Calibrate Altitude\n");
	PX4_INFO_RAW("          angle_of_attack        		          Calibrate Angle of Attack\n");
	PX4_INFO_RAW("          angle_of_sideslip    		          Calibrate Angle of Attack\n");
}

extern "C" __EXPORT int mscs_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
		return 1;
	}

	if (!strcmp(argv[1], "start")) {
		return MSCS::main(argc, argv);

	} else if (!strcmp(argv[1], "command")) {
		if (argc < 3) {
			PX4_INFO_RAW("Usage: mscs command <command>\n");
		} else {
			if (!strcmp(argv[2], "start_data_acquisition")) {
				return MSCS::start_data_acquisition_command_handler(argc-3, &argv[3]);
			} else if (!strcmp(argv[2], "stop_data_acquisition")) {
				return MSCS::stop_data_acquisition_command_handler(argc-3, &argv[3]);
			} else {
				PX4_INFO_RAW("\033[1m[MSCS]\033[0m Command '%s' not found.\n", argv[2]);
			}
		}
	} else if (!strcmp(argv[1], "calibrate")) {
		if (argc < 3) {
			PX4_INFO_RAW("Usage: mscs calibrate <device_name>\n");
		} else {
			if (!strcmp(argv[2], "left_adp") || !strcmp(argv[2], "right_adp")) {
				return MSCS::adp_calibration_command_handler(argc-2, &argv[2]);
			} else {
				PX4_INFO_RAW("\033[1m[MSCS]\033[0m Device name \"%s\" not recognized. Use 'mscs list' to see a list of available nodes.\n", argv[2]);
			}
		}
	} else if (!strcmp(argv[1], "list")) {
		return MSCS::list_command_handler(argc-2, &argv[2]);
	} else {
		usage();
		return 1;
	}
	return 1;
}
