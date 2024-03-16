/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file MSCSRawData.hpp
 *
 * Defines basic functionality of MSCS Data Subscriptions
 *
 * @author Daniel Montes <daniel.montestolon@slu.edu>
 */

#pragma once

#include "../NodeManager.hpp"

#include <AirCRAFT/RawDataPoint_1_0.h>

#include "BaseSubscriber.hpp"

#include <px4_platform_common/module_params.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/mscs_status.h>
#include <uORB/topics/mscs_left_adp_raw_angle_of_attack_data.h>
#include <uORB/topics/mscs_left_adp_raw_angle_of_sideslip_data.h>
#include <uORB/topics/mscs_left_adp_raw_differential_pressure_data.h>
#include <uORB/topics/mscs_left_adp_raw_static_pressure_data.h>
//#include <uORB/topics/mscs_left_adp_raw_temperature_data.h>
#include <uORB/topics/mscs_right_adp_raw_angle_of_attack_data.h>
#include <uORB/topics/mscs_right_adp_raw_angle_of_sideslip_data.h>
#include <uORB/topics/mscs_right_adp_raw_differential_pressure_data.h>
#include <uORB/topics/mscs_right_adp_raw_static_pressure_data.h>
//#include <uORB/topics/mscs_right_adp_raw_temperature_data.h>

#define MSCS_RAW_DATA_COLUMNS			16U

#define MSCS_ID_LEFT_ADP			22
#define MSCS_ID_RIGHT_ADP			23

#define MSCS_ADP_DIFFERENTIAL_PRESSURE_SOURCE	(uint8_t) 0x00
#define MSCS_ADP_STATIC_PRESSURE_SOURCE		(uint8_t) 0x01
#define MSCS_ADP_ANGLE_OF_ATTACK_SOURCE		(uint8_t) 0x02
#define MSCS_ADP_ANGLE_OF_SIDESLIP_SOURCE	(uint8_t) 0x03

#define MSCS_ADP_IAS	                        (uint8_t) 0x04
#define MSCS_ADP_CAS	                        (uint8_t) 0x05
#define MSCS_ADP_TAS	                        (uint8_t) 0x06

#define MSCS_ADP_PRESSURE_ALTITUDE		(uint8_t) 0x07
#define MSCS_ADP_DENSITY_ALTITUDE		(uint8_t) 0x08
#define MSCS_ADP_MSL_ALTITUDE		        (uint8_t) 0x09
#define MSCS_ADP_AGL_ALTITUDE		        (uint8_t) 0x0A

#define MSCS_ADP_TEMPERATURE_SOURCE             (uint8_t) 0x0B

class MSCSRawDataSubscriber : public UavcanBaseSubscriber, public ModuleParams
{
public:
	MSCSRawDataSubscriber(CanardHandle &handle) : UavcanBaseSubscriber(handle, "", "MSCSRawData", 0), ModuleParams(nullptr)
	{

	};

	void subscribe() override
	{
		/* Subscribe to MSCS messages */
		_canard_handle.RxSubscribe(CanardTransferKindMessage,
					   AirCRAFT_RawDataPoint_1_0_FIXED_PORT_ID_,  // The fixed Subject-ID
					   AirCRAFT_RawDataPoint_1_0_EXTENT_BYTES_,
					   CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
					   &_subj_sub._canard_sub);

	};

	void callback(const CanardRxTransfer &receive) override
	{
		/* VARIABLE DEFINITIONS */
		const hrt_abstime system_timestamp = hrt_absolute_time();

		/* FUNCTION BODY */
		/* Check if uORB has been updated */
		if (_mscs_status_sub.updated()) {
			/* Copy uORB MSCS Status into local buffer */
			_mscs_status_sub.copy(&_mscs_status_msg);

			/* Get all the possibly updated parameters */
			_left_adp_node_id  = _mscs_status_msg.left_adp_node_id;
			_right_adp_node_id = _mscs_status_msg.right_adp_node_id;
		}

		/* Check if MSCS is active */
		if (_mscs_status_msg.module_active == true) {
			/* Deserialize Raw Data Structure */
			_raw_data_message_buffer_size = AirCRAFT_RawDataPoint_1_0_EXTENT_BYTES_;

			if (AirCRAFT_RawDataPoint_1_0_deserialize_(&_raw_data_message, (uint8_t*) receive.payload, &_raw_data_message_buffer_size) == 0) {
				/* Identify Sending Node Message */
				switch (receive.metadata.remote_node_id) {
					case MSCS_ID_LEFT_ADP:

						/* For the Left ADP, the flag field identifies the data source */
						_raw_data_source_identifier = (_raw_data_message.flags & 0x00FF);

						/* Save data to proper message depending on source */
						switch (_raw_data_source_identifier){
							case MSCS_ADP_ANGLE_OF_ATTACK_SOURCE:
								/* Save Data to Message Buffer */
								_mscs_left_adp_raw_angle_of_attack_data_msg.data_acquisition_timestamp[_mscs_left_adp_raw_angle_of_attack_data_point_counter]   = _raw_data_message.timestamp;
								_mscs_left_adp_raw_angle_of_attack_data_msg.data                      [_mscs_left_adp_raw_angle_of_attack_data_point_counter++] = _raw_data_message.data;

								//PX4_INFO("RAW LA [%lu] 0x%04x", _mscs_left_adp_raw_angle_of_attack_data_msg.data_acquisition_timestamp[_mscs_left_adp_raw_angle_of_attack_data_point_counter-1], _mscs_left_adp_raw_angle_of_attack_data_msg.data[_mscs_left_adp_raw_angle_of_attack_data_point_counter-1]);
								/* Check if Data should be published */
								if (_mscs_left_adp_raw_angle_of_attack_data_point_counter == MSCS_RAW_DATA_COLUMNS) {
									/* Get current timestamp */
									_mscs_left_adp_raw_angle_of_attack_data_msg.timestamp = system_timestamp;

									/* Publish data to uORB */
									_mscs_left_adp_raw_angle_of_attack_data_pub.publish(_mscs_left_adp_raw_angle_of_attack_data_msg);

									/* Reset Pointer */
									_mscs_left_adp_raw_angle_of_attack_data_point_counter = 0;
								}
								break;

							case MSCS_ADP_ANGLE_OF_SIDESLIP_SOURCE:
								/* Save Data to Message Buffer */
								_mscs_left_adp_raw_angle_of_sideslip_data_msg.data_acquisition_timestamp[_mscs_left_adp_raw_angle_of_sideslip_data_point_counter]   = _raw_data_message.timestamp;
								_mscs_left_adp_raw_angle_of_sideslip_data_msg.data                      [_mscs_left_adp_raw_angle_of_sideslip_data_point_counter++] = _raw_data_message.data;

								//PX4_INFO("RAW LB [%lu] 0x%04x", _mscs_left_adp_raw_angle_of_sideslip_data_msg.data_acquisition_timestamp[_mscs_left_adp_raw_angle_of_sideslip_data_point_counter-1], _mscs_left_adp_raw_angle_of_sideslip_data_msg.data[_mscs_left_adp_raw_angle_of_sideslip_data_point_counter-1]);
								/* Check if Data should be published */
								if (_mscs_left_adp_raw_angle_of_sideslip_data_point_counter == MSCS_RAW_DATA_COLUMNS) {
									/* Get current timestamp */
									_mscs_left_adp_raw_angle_of_sideslip_data_msg.timestamp = system_timestamp;

									/* Publish data to uORB */
									_mscs_left_adp_raw_angle_of_sideslip_data_pub.publish(_mscs_left_adp_raw_angle_of_sideslip_data_msg);

									/* Reset Pointer */
									_mscs_left_adp_raw_angle_of_sideslip_data_point_counter = 0;
								}
								break;

							case MSCS_ADP_DIFFERENTIAL_PRESSURE_SOURCE:
								/* Save Data to Message Buffer */
								_mscs_left_adp_raw_differential_pressure_data_msg.data_acquisition_timestamp[_mscs_left_adp_raw_differential_pressure_data_point_counter]   = _raw_data_message.timestamp;
								_mscs_left_adp_raw_differential_pressure_data_msg.data                      [_mscs_left_adp_raw_differential_pressure_data_point_counter++] = _raw_data_message.data;

								//PX4_INFO("RAW LD [%lu] 0x%04x", _mscs_left_adp_raw_differential_pressure_data_msg.data_acquisition_timestamp[_mscs_left_adp_raw_differential_pressure_data_point_counter-1], _mscs_left_adp_raw_differential_pressure_data_msg.data[_mscs_left_adp_raw_differential_pressure_data_point_counter-1]);
								if (_mscs_left_adp_raw_differential_pressure_data_point_counter == MSCS_RAW_DATA_COLUMNS) {
									/* Get current timestamp */
									_mscs_left_adp_raw_differential_pressure_data_msg.timestamp = system_timestamp;

									/* Publish data to uORB */
									_mscs_left_adp_raw_differential_pressure_data_pub.publish(_mscs_left_adp_raw_differential_pressure_data_msg);

									/* Reset Pointer */
									_mscs_left_adp_raw_differential_pressure_data_point_counter = 0;
								}
								break;

							case MSCS_ADP_STATIC_PRESSURE_SOURCE:
								/* Save Data to Message Buffer */
								_mscs_left_adp_raw_static_pressure_data_msg.data_acquisition_timestamp[_mscs_left_adp_raw_static_pressure_data_point_counter]   = _raw_data_message.timestamp;
								_mscs_left_adp_raw_static_pressure_data_msg.data                      [_mscs_left_adp_raw_static_pressure_data_point_counter++] = _raw_data_message.data;

								//PX4_INFO("RAW LS [%lu] 0x%04x", _mscs_left_adp_raw_static_pressure_data_msg.data_acquisition_timestamp[_mscs_left_adp_raw_static_pressure_data_point_counter-1], _mscs_left_adp_raw_static_pressure_data_msg.data[_mscs_left_adp_raw_static_pressure_data_point_counter-1]);
								if (_mscs_left_adp_raw_static_pressure_data_point_counter == MSCS_RAW_DATA_COLUMNS) {
									/* Get current timestamp */
									_mscs_left_adp_raw_static_pressure_data_msg.timestamp = system_timestamp;

									/* Publish data to uORB */
									_mscs_left_adp_raw_static_pressure_data_pub.publish(_mscs_left_adp_raw_static_pressure_data_msg);

									/* Reset Pointer */
									_mscs_left_adp_raw_static_pressure_data_point_counter = 0;
								}
								break;

							default:
								break;
						}

						break;
					case MSCS_ID_RIGHT_ADP:
						/* For the Right ADP, the flag field identifies the data source */
						_raw_data_source_identifier = (_raw_data_message.flags & 0x00FF);

						/* Save data to proper message depending on source */
						switch (_raw_data_source_identifier){
							case MSCS_ADP_ANGLE_OF_ATTACK_SOURCE:
								/* Save Data to Message Buffer */
								_mscs_right_adp_raw_angle_of_attack_data_msg.data_acquisition_timestamp[_mscs_right_adp_raw_angle_of_attack_data_point_counter]   = _raw_data_message.timestamp;
								_mscs_right_adp_raw_angle_of_attack_data_msg.data                      [_mscs_right_adp_raw_angle_of_attack_data_point_counter++] = _raw_data_message.data;

								/* Check if Data should be published */
								if (_mscs_right_adp_raw_angle_of_attack_data_point_counter == MSCS_RAW_DATA_COLUMNS) {
									/* Get current timestamp */
									_mscs_right_adp_raw_angle_of_attack_data_msg.timestamp = system_timestamp;

									/* Publish data to uORB */
									_mscs_right_adp_raw_angle_of_attack_data_pub.publish(_mscs_right_adp_raw_angle_of_attack_data_msg);

									/* Reset Pointer */
									_mscs_right_adp_raw_angle_of_attack_data_point_counter = 0;
								}
								break;

							case MSCS_ADP_ANGLE_OF_SIDESLIP_SOURCE:
								/* Save Data to Message Buffer */
								_mscs_right_adp_raw_angle_of_sideslip_data_msg.data_acquisition_timestamp[_mscs_right_adp_raw_angle_of_sideslip_data_point_counter]   = _raw_data_message.timestamp;
								_mscs_right_adp_raw_angle_of_sideslip_data_msg.data                      [_mscs_right_adp_raw_angle_of_sideslip_data_point_counter++] = _raw_data_message.data;

								/* Check if Data should be published */
								if (_mscs_right_adp_raw_angle_of_sideslip_data_point_counter == MSCS_RAW_DATA_COLUMNS) {
									/* Get current timestamp */
									_mscs_right_adp_raw_angle_of_sideslip_data_msg.timestamp = system_timestamp;

									/* Publish data to uORB */
									_mscs_right_adp_raw_angle_of_sideslip_data_pub.publish(_mscs_right_adp_raw_angle_of_sideslip_data_msg);

									/* Reset Pointer */
									_mscs_right_adp_raw_angle_of_sideslip_data_point_counter = 0;
								}
								break;

							case MSCS_ADP_DIFFERENTIAL_PRESSURE_SOURCE:
								/* Save Data to Message Buffer */
								_mscs_right_adp_raw_differential_pressure_data_msg.data_acquisition_timestamp[_mscs_right_adp_raw_differential_pressure_data_point_counter]   = _raw_data_message.timestamp;
								_mscs_right_adp_raw_differential_pressure_data_msg.data                      [_mscs_right_adp_raw_differential_pressure_data_point_counter++] = _raw_data_message.data;

								/* Check if Data should be published */
								if (_mscs_right_adp_raw_differential_pressure_data_point_counter == MSCS_RAW_DATA_COLUMNS) {
									/* Get current timestamp */
									_mscs_right_adp_raw_differential_pressure_data_msg.timestamp = system_timestamp;

									/* Publish data to uORB */
									_mscs_right_adp_raw_differential_pressure_data_pub.publish(_mscs_right_adp_raw_differential_pressure_data_msg);

									/* Reset Pointer */
									_mscs_right_adp_raw_differential_pressure_data_point_counter = 0;
								}
								break;

							case MSCS_ADP_STATIC_PRESSURE_SOURCE:
								/* Save Data to Message Buffer */
								_mscs_right_adp_raw_static_pressure_data_msg.data_acquisition_timestamp[_mscs_right_adp_raw_static_pressure_data_point_counter]   = _raw_data_message.timestamp;
								_mscs_right_adp_raw_static_pressure_data_msg.data                      [_mscs_right_adp_raw_static_pressure_data_point_counter++] = _raw_data_message.data;

								/* Check if Data should be published */
								if (_mscs_right_adp_raw_static_pressure_data_point_counter == MSCS_RAW_DATA_COLUMNS) {
									/* Get current timestamp */
									_mscs_right_adp_raw_static_pressure_data_msg.timestamp = system_timestamp;

									/* Publish data to uORB */
									_mscs_right_adp_raw_static_pressure_data_pub.publish(_mscs_right_adp_raw_static_pressure_data_msg);

									/* Reset Pointer */
									_mscs_left_adp_raw_static_pressure_data_point_counter = 0;
								}
								break;

							default:
								break;
						}

						break;
					default:
						break;
				}
			}
		}
	}

private:
	/* Raw Data Message Buffer */
	AirCRAFT_RawDataPoint_1_0 _raw_data_message{};
	size_t  _raw_data_message_buffer_size;
	uint8_t _raw_data_source_identifier;

	/* uORB */
	uint8_t _mscs_left_adp_raw_angle_of_attack_data_point_counter{0};
	uint8_t _mscs_left_adp_raw_angle_of_sideslip_data_point_counter{0};
	uint8_t _mscs_left_adp_raw_differential_pressure_data_point_counter{0};
	uint8_t _mscs_left_adp_raw_static_pressure_data_point_counter{0};
	uint8_t _mscs_right_adp_raw_angle_of_attack_data_point_counter{0};
	uint8_t _mscs_right_adp_raw_angle_of_sideslip_data_point_counter{0};
	uint8_t _mscs_right_adp_raw_differential_pressure_data_point_counter{0};
	uint8_t _mscs_right_adp_raw_static_pressure_data_point_counter{0};

	mscs_status_s 					                     _mscs_status_msg{0};
	mscs_left_adp_raw_angle_of_attack_data_s                             _mscs_left_adp_raw_angle_of_attack_data_msg{0};
	mscs_left_adp_raw_angle_of_sideslip_data_s                           _mscs_left_adp_raw_angle_of_sideslip_data_msg{0};
	mscs_left_adp_raw_differential_pressure_data_s                       _mscs_left_adp_raw_differential_pressure_data_msg{0};
	mscs_left_adp_raw_static_pressure_data_s                             _mscs_left_adp_raw_static_pressure_data_msg{0};
	mscs_right_adp_raw_angle_of_attack_data_s                            _mscs_right_adp_raw_angle_of_attack_data_msg{0};
	mscs_right_adp_raw_angle_of_sideslip_data_s                          _mscs_right_adp_raw_angle_of_sideslip_data_msg{0};
	mscs_right_adp_raw_differential_pressure_data_s                      _mscs_right_adp_raw_differential_pressure_data_msg{0};
	mscs_right_adp_raw_static_pressure_data_s                            _mscs_right_adp_raw_static_pressure_data_msg{0};

	uORB::Subscription _mscs_status_sub                                  {ORB_ID(mscs_status)};
	uORB::Subscription _mscs_left_adp_raw_angle_of_attack_data_sub       {ORB_ID(mscs_left_adp_raw_angle_of_attack_data)};
	uORB::Subscription _mscs_left_adp_raw_angle_of_sideslip_data_sub     {ORB_ID(mscs_left_adp_raw_angle_of_sideslip_data)};
	uORB::Subscription _mscs_left_adp_raw_differential_pressure_data_sub {ORB_ID(mscs_left_adp_raw_differential_pressure_data)};
	uORB::Subscription _mscs_left_adp_raw_static_pressure_data_sub       {ORB_ID(mscs_left_adp_raw_static_pressure_data)};
	uORB::Subscription _mscs_right_adp_raw_angle_of_attack_data_sub      {ORB_ID(mscs_right_adp_raw_angle_of_attack_data)};
	uORB::Subscription _mscs_right_adp_raw_angle_of_sideslip_data_sub    {ORB_ID(mscs_right_adp_raw_angle_of_sideslip_data)};
	uORB::Subscription _mscs_right_adp_raw_differential_pressure_data_sub{ORB_ID(mscs_right_adp_raw_differential_pressure_data)};
	uORB::Subscription _mscs_right_adp_raw_static_pressure_data_sub      {ORB_ID(mscs_right_adp_raw_static_pressure_data)};

	uORB::Publication<mscs_left_adp_raw_angle_of_attack_data_s>          _mscs_left_adp_raw_angle_of_attack_data_pub       {ORB_ID(mscs_left_adp_raw_angle_of_attack_data)};
	uORB::Publication<mscs_left_adp_raw_angle_of_sideslip_data_s>        _mscs_left_adp_raw_angle_of_sideslip_data_pub     {ORB_ID(mscs_left_adp_raw_angle_of_sideslip_data)};
	uORB::Publication<mscs_left_adp_raw_differential_pressure_data_s>    _mscs_left_adp_raw_differential_pressure_data_pub {ORB_ID(mscs_left_adp_raw_differential_pressure_data)};
	uORB::Publication<mscs_left_adp_raw_static_pressure_data_s>          _mscs_left_adp_raw_static_pressure_data_pub       {ORB_ID(mscs_left_adp_raw_static_pressure_data)};
	uORB::Publication<mscs_right_adp_raw_angle_of_attack_data_s>         _mscs_right_adp_raw_angle_of_attack_data_pub      {ORB_ID(mscs_right_adp_raw_angle_of_attack_data)};
	uORB::Publication<mscs_right_adp_raw_angle_of_sideslip_data_s>       _mscs_right_adp_raw_angle_of_sideslip_data_pub    {ORB_ID(mscs_right_adp_raw_angle_of_sideslip_data)};
	uORB::Publication<mscs_right_adp_raw_differential_pressure_data_s>   _mscs_right_adp_raw_differential_pressure_data_pub{ORB_ID(mscs_right_adp_raw_differential_pressure_data)};
	uORB::Publication<mscs_right_adp_raw_static_pressure_data_s>         _mscs_right_adp_raw_static_pressure_data_pub      {ORB_ID(mscs_right_adp_raw_static_pressure_data)};

	/* Node IDs */
	CanardNodeID _left_adp_node_id;
	CanardNodeID _right_adp_node_id;
};
