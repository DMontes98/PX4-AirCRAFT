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

MSCS::MSCS() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

MSCS::~MSCS()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool MSCS::init()
{
	PX4_INFO("Modular Sensor and Computing System Initialized");

	/* Schedule Now */
	ScheduleNow();

	return true;
}

void MSCS::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);
}

int MSCS::task_spawn(int argc, char *argv[])
{
	/* Create New MSCS Instance */
	MSCS *instance = new MSCS();

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
	return print_usage("unknown command");
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

extern "C" __EXPORT int mscs_main(int argc, char *argv[])
{

	return MSCS::main(argc, argv);
}
