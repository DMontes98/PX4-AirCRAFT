/*
 * "MSCS.hpp"
 *
 * ** Picluster Controller **
 * Saint Louis University
 *
 * Author: Daniel Montes <daniel.montestolon@slu.edu>
 *
 * Description: Data Acquisition Interface for Picluster
 *
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/log.h>				// Includes PX4_INFO()

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <poll.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/aircraft_state_raw.h>

#include <uORB/topics/adp_data.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/vehicle_air_data.h>

using namespace time_literals;

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

private:
	void Run() override;

	// uORB Publications
        //uORB::Publication<aircraft_state_raw_s> _to_aircraft_state_raw{ORB_ID(aircraft_state_raw)};

	// uORB Subscriptions
	/*uORB::Subscription _imu_sub{ORB_ID(sensor_combined)};
	uORB::Subscription _pilot_input_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _barometer_sub{ORB_ID(sensor_baro)};
	uORB::Subscription _adp_data_sub{ORB_ID(adp_data)};
	uORB::Subscription _airspeed_sub{ORB_ID(airspeed_validated)};
	uORB::Subscription _air_data_sub{ORB_ID(vehicle_air_data)};*/

	// uORB Messages
	//aircraft_state_raw_s _aircraft_state_raw_msg{};

	// Sampling
	//static const hrt_abstime	SAMPLE_INTERVAL{10_ms};

	// Performance (perf) counters
	//perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	//perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

};

extern "C" __EXPORT int mscs_main(int argc, char *argv[]);
