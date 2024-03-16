/****************************************************************************
 *
 *   Copyright (c) 2024 Daniel Montes <daniel.montestolon@slu.edu>
 *   AirCRAFT Lab - Saint Louis University
 *
 ****************************************************************************/

/**
 * @file MSCS_poll.hpp
 * Implementation of the system status polling task in the Aircraft Lab's
 * Modular Sensor and Computing System
 *
 * @author Daniel Montes 	<daniel.montestolon@slu.edu>
 *
 */

#ifndef MSCS_POLL_HPP_
#define MSCS_POLL_HPP_

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
#include <uORB/topics/mscs_status.h>
#include <uORB/topics/mscs_event.h>
#include <uORB/topics/parameter_update.h>

#include "MSCS.hpp"

using namespace time_literals;

class MSCS;
template <typename T> class MSCS_queue;

class MSCS_poll : public ModuleBase<MSCS_poll>, public ModuleParams, public px4::ScheduledWorkItem {
public:
	MSCS_poll();
	~MSCS_poll() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	void register_MSCS(MSCS* MSCS_ptr);

	int print_status() override;

private:
	/* Private Functions */
	void Run() override;
	void _poll_status();
	void _notify_sensor_connected(uint8_t MSCS_sensor_id, uint8_t dynamic_node_id, uint8_t sensor_mode, uint8_t sensor_status);
	void _notify_sensor_disconnected(uint8_t MSCS_sensor_id, uint8_t dynamic_node_id, uint8_t sensor_mode, uint8_t sensor_status);
	void _event_handler();

	/* MSCS */
	MSCS* _MSCS;
	MSCS_queue<mscs_event_s>* _event_queue{};

	/* Sampling */
	static const hrt_abstime SAMPLE_INTERVAL{100_ms};

	/* Timeouts */
	uint64_t _hearbeat_timeout_us{0};

	/* Status Structs */
	mscs_status_s _mscs_status_s{0};

	/* uORB Buffers */
	mscs_status_s _mscs_status_msg{0};
	mscs_event_s  _mscs_event_msg{0};

	/* uORB Subscriptions */
	uORB::Subscription _mscs_status_sub{ORB_ID(mscs_status)};
	uORB::Subscription _mscs_event_sub{ORB_ID(mscs_event)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	/* uORB Publications */
	uORB::Publication<mscs_status_s> _mscs_status_pub{ORB_ID(mscs_status)};
	uORB::Publication<mscs_event_s>  _mscs_event_pub{ORB_ID(mscs_event)};

	/* Performance counters */
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	/* Parameters */
	bool _parameters_updated{false};
	void _parameters_update();

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MSCS_HB_TIMEOUT>)  _cyphal_hearbeat_timeout,
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

#endif /* MSCS_POLL_HPP_ */
