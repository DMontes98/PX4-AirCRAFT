/****************************************************************************
 *
 *   Copyright (c) 2024 Daniel Montes <daniel.montestolon@slu.edu>
 *   AirCRAFT Lab - Saint Louis University
 *
 ****************************************************************************/

/**
 * @file MSCS_poll.cpp
 * Implementation of the system status polling task in the Aircraft Lab's
 * Modular Sensor and Computing System
 *
 * @author Daniel Montes 	<daniel.montestolon@slu.edu>
 *
 */

#include "MSCS_poll.hpp"

static int instance_count = 0;

MSCS_poll::MSCS_poll() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

MSCS_poll::~MSCS_poll()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool MSCS_poll::init()
{
	/* VARIABLE DEFINITIONS */
	const hrt_abstime system_timestamp = hrt_absolute_time();

	/* FUNCTION BODY */
	/* Create Event Queue */
	_event_queue = new MSCS_queue<mscs_event_s>;

	/* Initialize heartbeat timeout */
	_hearbeat_timeout_us = _cyphal_hearbeat_timeout.get()*1e3;

	/* Initialize MSCS Status Structure in the uORB */
	_mscs_status_msg.timestamp          = system_timestamp;
	_mscs_status_msg.module_active      = true;
	_mscs_status_msg.state              = MSCS_STATE_READY;

	_mscs_status_msg.left_adp_node_id   = (uint8_t) _left_adp_node_id.get();
	_mscs_status_msg.right_adp_node_id  = (uint8_t) _right_adp_node_id.get();
	_mscs_status_msg.left_adp_active    = false;
	_mscs_status_msg.right_adp_active   = false;

	/* Publish Initial MSCS Status*/
	_mscs_status_pub.publish(_mscs_status_msg);
	_mscs_event_msg.node_id             = 0;
	_mscs_event_msg.source_module       = MSCS_POLL_TASK_ID;
	_mscs_event_msg.event_type          = MSCS_EVENT_INIT;
	_mscs_event_msg.event_handled       = -1;
	_mscs_event_msg.event_timeout_ms    = 0;

	/* Initialize MSCS Initialization Event Message */
	_mscs_event_msg.timestamp = system_timestamp;

	/* Publish Initialization Event */
	//PX4_INFO("[MSCS_poll::init()] Pushing first event");
	_event_queue->push(_mscs_event_msg);
	_event_handler();

	/* Initialize MSCS Status Structure so we can detect active sensors in the first cycle */
	_mscs_status_s = _mscs_status_msg;

	/* Schedule */
	ScheduleOnInterval(SAMPLE_INTERVAL);

	return true;
}

void MSCS_poll::Run()
{
	/* FUNCTION BODY */
	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	/* Check if task should exit */
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	/* Check if there has been a parameter update */
	_parameters_update();

	/* Poll Status Structure */
	_poll_status();

	/* Check if event queue is empty */
	//PX4_INFO("[MSCS_poll::Run()] - Checking if event queue is empty");
	if (!_event_queue->empty()) {
		//PX4_INFO("[MSCS_poll::Run()] - Event Queue is not empty");
		_event_handler();
	}
}

int MSCS_poll::task_spawn(int argc, char *argv[])
{
	/* VARIABLE DEFINITION */
	MSCS* MSCS_ref = static_cast<MSCS*>(static_cast<void*>(argv[0]));
	MSCS_poll* instance;

	/* FUNCTION BODY*/
	//PX4_INFO("[MSCS_poll::task_spawn()] - Call()");
	instance = new MSCS_poll();

	/* Create New MSCS Instance */
	if (instance_count == 0) {

		//PX4_INFO("[MSCS_poll::task_spawn()] - New Instance Created");

		instance_count++;

		/* Check if instance was properly created */
		if (instance) {
			_object.store(instance);
			_task_id = task_id_is_work_queue;

			/* Save pointer to MSCS */
			instance->register_MSCS(MSCS_ref);

			/* Initialize MSCS instance */
			if (instance->init()) {
				return PX4_OK;
			}

		} else {
			PX4_ERR("alloc failed");
		}
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MSCS_poll::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

void MSCS_poll::register_MSCS(MSCS* MSCS_ptr)
{
	/* FUNCTION BODY */
	/* Save Pointer to MSCS */
	_MSCS = MSCS_ptr;

	/* Register current task object in MSCS */
	_MSCS->register_poll(this);
}

void MSCS_poll::_poll_status()
{
	/* VARIABLE DEFINITIONS */
	const hrt_abstime system_timestamp = hrt_absolute_time();
	bool mscs_status_updated = false;

	/* FUNCTION BODY */
	/* Check if uORB Message has been updated */
	if (_mscs_status_sub.updated()) {
		/* Get MSCS Status Structure from the uORB */
		_mscs_status_sub.copy(&_mscs_status_msg);
	} else {
		/* Current message is the same as the previous iteration */
		_mscs_status_msg = _mscs_status_s;
	}

	/* If there has been a change in parameters, reload all possible changes */
	if (_parameters_updated) {
		/* Reset Cyphal Node IDs */
		if (_mscs_status_msg.left_adp_node_id      != (uint8_t) _left_adp_node_id.get()) {
			_mscs_status_msg.left_adp_node_id   = (uint8_t) _left_adp_node_id.get();
			mscs_status_updated = true;
		}
		if (_mscs_status_msg.right_adp_node_id     != (uint8_t) _right_adp_node_id.get()) {
			_mscs_status_msg.right_adp_node_id  = (uint8_t) _right_adp_node_id.get();
			mscs_status_updated = true;
		}
		/* Reset parameters updated flag */
		_parameters_updated = false;
	}

	/* Check all nodes for heartbeat timeouts */
	if (_mscs_status_msg.left_adp_active && (((system_timestamp - _mscs_status_msg.left_adp_last_heartbeat_timestamp) > _hearbeat_timeout_us))) {
		PX4_INFO("[MSCS_poll::_poll_status()] Left ADP Disconnected.");
		/* Set node as inactive */
		_mscs_status_msg.left_adp_active = false;
		/* Set MSCS Status Updated Flag */
		mscs_status_updated = true;

		_notify_sensor_disconnected(MSCS_ID_LEFT_ADP, _mscs_status_msg.left_adp_node_id, _mscs_status_msg.left_adp_mode, _mscs_status_msg.left_adp_status);

	}
	if (_mscs_status_msg.right_adp_active && (((system_timestamp - _mscs_status_msg.right_adp_last_heartbeat_timestamp) > _hearbeat_timeout_us))) {
		//PX4_INFO("[MSCS_poll::_poll_status()] Right ADP Disconnected.");
		/* Set node as inactive */
		_mscs_status_msg.right_adp_active = false;
		/* Set MSCS Status Updated Flag */
		mscs_status_updated = true;

		_notify_sensor_disconnected(MSCS_ID_RIGHT_ADP, _mscs_status_msg.right_adp_node_id, _mscs_status_msg.right_adp_mode, _mscs_status_msg.right_adp_status);

	}

	/* Detect new sensor connections */
	if (!_mscs_status_s.left_adp_active && _mscs_status_msg.left_adp_active) {
		/* Notify MSCS of new sensor connection */
		_notify_sensor_connected(MSCS_ID_LEFT_ADP, _mscs_status_msg.left_adp_node_id, _mscs_status_msg.left_adp_mode, _mscs_status_msg.left_adp_status);

	}
	if (!_mscs_status_s.right_adp_active && _mscs_status_msg.right_adp_active) {
		/* Notify MSCS of new sensor connection */
		_notify_sensor_connected(MSCS_ID_RIGHT_ADP, _mscs_status_msg.right_adp_node_id, _mscs_status_msg.right_adp_mode, _mscs_status_msg.right_adp_status);
	}

	/* If there have been changes, publish new MSCS Status*/
	if (mscs_status_updated) {
		_mscs_status_pub.publish(_mscs_status_msg);
	}

	/* Save message into status struct to be compared in next cycle */
	_mscs_status_s = _mscs_status_msg;
}

void MSCS_poll::_notify_sensor_connected(uint8_t MSCS_sensor_id, uint8_t dynamic_node_id, uint8_t sensor_mode, uint8_t sensor_status)
{
	/* VARIABLE DEFINITIONS */
	const hrt_abstime system_timestamp = hrt_absolute_time();

	/* FUNCTION_BODY */
	/* Assemble Event Message */
	_mscs_event_msg.timestamp        = system_timestamp;
	_mscs_event_msg.node_id          = MSCS_sensor_id;
	_mscs_event_msg.dynamic_node_id  = dynamic_node_id;
	_mscs_event_msg.event_type       = MSCS_EVENT_DEVICE_CONNECTED;
	_mscs_event_msg.event_data[0]    = sensor_mode;
	_mscs_event_msg.event_data[1]    = sensor_status;
	_mscs_event_msg.event_handled    = 0;
	_mscs_event_msg.event_timeout_ms = 1000;

	//PX4_INFO("[MSCS_poll::_notify_sensor_new()] - Pushing MSCS_EVENT_DEVICE_CONNECTED to Event Queue");

	/* Publish Event */
	_event_queue->push(_mscs_event_msg);
	_event_handler();
}

void MSCS_poll::_notify_sensor_disconnected(uint8_t MSCS_sensor_id, uint8_t dynamic_node_id, uint8_t sensor_mode, uint8_t sensor_status)
{
	/* VARIABLE DEFINITIONS */
	const hrt_abstime system_timestamp = hrt_absolute_time();

	/* FUNCTION_BODY */
	/* Assemble Event Message */
	_mscs_event_msg.timestamp        = system_timestamp;
	_mscs_event_msg.node_id          = MSCS_sensor_id;
	_mscs_event_msg.dynamic_node_id  = dynamic_node_id;
	_mscs_event_msg.event_type       = MSCS_EVENT_DEVICE_DISCONNECTED;
	_mscs_event_msg.event_data[0]    = sensor_mode;
	_mscs_event_msg.event_data[1]    = sensor_status;
	_mscs_event_msg.event_handled    = 0;
	_mscs_event_msg.event_timeout_ms = 1000;

	//PX4_INFO("[MSCS_poll::_notify_sensor_new()] - Pushing MSCS_EVENT_DEVICE_DISCONNECTED to Event Queue");

	/* Publish Event */
	_event_queue->push(_mscs_event_msg);
	_event_handler();
}
void MSCS_poll::_event_handler()
{
	/* VARIABLE DEFINITIONS */
	const hrt_abstime system_timestamp = hrt_absolute_time();
	mscs_event_s current_event;
	mscs_event_s next_event;

	/* FUNCTION_BODY */
	/* Check if current Event has not yet been handled */
	if (_mscs_event_sub.copy(&current_event)) {
		//PX4_INFO("[MSCS_poll::_event_handler()] - CURRENT EVENT - Timestamp:  %llu",  current_event.timestamp);
		//PX4_INFO("[MSCS_poll::_event_handler()] - CURRENT EVENT - Event Type: %hu",   current_event.event_type);
		//PX4_INFO("[MSCS_poll::_event_handler()] - CURRENT EVENT - Node ID:    %hu",   current_event.node_id);
		//PX4_INFO("[MSCS_poll::_event_handler()] - CURRENT EVENT - Timeout:    %lums", current_event.event_timeout_ms);

		/* Check if event has been handled already or has timed out */
		if ((current_event.event_handled != 0) || ((system_timestamp - current_event.timestamp) > ((uint64_t) 1e3 * (uint64_t) current_event.event_timeout_ms))) {
			//PX4_INFO("[MSCS_poll::_event_handler()] - Current Event Handled or Timed Out");
			/* Pop if current queue'd event is the same as handled event */
			_event_queue->peek(&next_event);
			//PX4_INFO("[MSCS_poll::_event_handler()] - NEXT EVENT - Timestamp:  %llu",  next_event.timestamp);
			//PX4_INFO("[MSCS_poll::_event_handler()] - NEXT EVENT - Event Type: %hu",   next_event.event_type);
			//PX4_INFO("[MSCS_poll::_event_handler()] - NEXT EVENT - Node ID:    %hu",   next_event.node_id);
			//PX4_INFO("[MSCS_poll::_event_handler()] - NEXT EVENT - Timeout:    %lums", next_event.event_timeout_ms);

			if (current_event.timestamp == next_event.timestamp) {/* Event no longer needs to be handled */
				_event_queue->pop();
				//PX4_INFO("[MSCS_poll::_event_handler()] - Event Removed from event Queue");
			}

			/* Is there another event in line? */
			if (!_event_queue->empty()) {
				/* Publish Event */
				_event_queue->peek(&next_event);
				_mscs_event_pub.publish(next_event);

				//PX4_INFO("[MSCS_poll::_event_handler()] - Publishing Event");
			}
		} else {
			//PX4_INFO("[MSCS_poll::_event_handler()] - Current Event Not Handled yet");
		}

	} else {
		/* No first event has been transmitted */
		//PX4_INFO("[MSCS_poll::_event_handler()] - Handling first event");

		/* Get next event in line */
		_event_queue->peek(&current_event);

		/* Publish Event */
		_mscs_event_pub.publish(current_event);

		//PX4_INFO("[MSCS_poll::_event_handler()] - Publishing Event");

	}
}

void MSCS_poll::_parameters_update()
{
	/* VARIABLE DEFINITIONS */
	const hrt_abstime system_timestamp = hrt_absolute_time();
	parameter_update_s parameter_update_msg;

	/* FUNCTION_BODY */
	/* Check if there has been an update to the parameters through the parameter update subscription */
	if (_parameter_update_sub.updated()) {
		//PX4_INFO("[MSCS_poll::_parameters_update()] - Update in parameters detected");

		/* Assemble Event Message */
		_mscs_event_msg.timestamp        = system_timestamp;
		_mscs_event_msg.node_id          = 0;
		_mscs_event_msg.dynamic_node_id  = 0;
		_mscs_event_msg.event_type       = MSCS_EVENT_PARAMETERS_UPDATED;
		_mscs_event_msg.event_handled    = 0;
		_mscs_event_msg.event_timeout_ms = 1000;

		/* Push to event queue */
		_event_queue->push(_mscs_event_msg);

		/* Copy parameter update topic */
		_parameter_update_sub.copy(&parameter_update_msg);

		/* If any parameter updated, call updateParams() to check if this class attributes need updating (and do so) */
		updateParams();

		/* There has been a parameter change, so return true */
		_parameters_updated = true;
	}

	/* Update heartbeat timeout */
	_hearbeat_timeout_us = _cyphal_hearbeat_timeout.get()*1e3;

}
