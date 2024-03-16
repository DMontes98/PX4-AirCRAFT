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
 * @file MSCSExecuteCommand.hpp
 *
 * Defines basic functionality of MSCS Execute Comand Subscriptions
 *
 * @author Daniel Montes <daniel.montestolon@slu.edu>
 */

#pragma once

#include "../NodeManager.hpp"

#include <uavcan/node/ExecuteCommand_1_2.h>

#include "BaseSubscriber.hpp"

#include <px4_platform_common/module_params.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/mscs_status.h>
#include <uORB/topics/mscs_calibration_event.h>

#define MSCS_ID_LEFT_ADP			22
#define MSCS_ID_RIGHT_ADP			23

#define uavcan_node_ExecuteCommand_Request_1_2_COMMAND_DATA_ACQUISITION_START (110U)
#define uavcan_node_ExecuteCommand_Request_1_2_COMMAND_DATA_ACQUISITION_STOP (111U)
#define uavcan_node_ExecuteCommand_Request_1_2_COMMAND_DATA_ACQUISITION_CONFIG (112U)
#define uavcan_node_ExecuteCommand_Request_1_2_COMMAND_DATA_CALIBRATION_START (130U)
#define uavcan_node_ExecuteCommand_Request_1_2_COMMAND_DATA_CALIBRATION_SUBMIT (131U)
#define uavcan_node_ExecuteCommand_Request_1_2_COMMAND_DATA_CALIBRATION_CANCEL (132U)

#define MSCS_CALIBRATION_BAD                     0
#define MSCS_CALIBRATION_OK                      1
#define MSCS_CALIBRATION_CONNECTION_ESTABLISHED  2

#define MSCS_CALIBRATION_MOVE_TO_NEXT            1
#define MSCS_CALIBRATION_OVERFLOW                5

class MSCSExecuteCommandSubscriber : public UavcanBaseSubscriber, public ModuleParams
{
public:
	MSCSExecuteCommandSubscriber(CanardHandle &handle) : UavcanBaseSubscriber(handle, "", "MSCSExecuteCommand", 0), ModuleParams(nullptr)
	{

	};

	void subscribe() override
	{
		/* Subscribe to MSCS messages */
		_canard_handle.RxSubscribe(CanardTransferKindRequest,
					   uavcan_node_ExecuteCommand_1_2_FIXED_PORT_ID_,  // The fixed Subject-ID
					   uavcan_node_ExecuteCommand_1_2_FIXED_PORT_ID_,
					   CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
					   &_subj_sub._canard_sub);

	};

	void callback(const CanardRxTransfer &receive) override
	{
		/* VARIABLE DEFINITIONS */
		const hrt_abstime system_timestamp = hrt_absolute_time();
		uint8_t* command_payload;

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
			_command_buffer_size = uavcan_node_ExecuteCommand_Request_1_2_EXTENT_BYTES_;

			command_payload = (uint8_t*) receive.payload;

			if (uavcan_node_ExecuteCommand_Request_1_2_deserialize_(&_command_msg, command_payload, &_command_buffer_size) == 0) {
				/* Identify Sending Node Message */
				switch (receive.metadata.remote_node_id) {
					case MSCS_ID_LEFT_ADP:
						/* Save data to proper message depending on source */
						switch (_command_msg.command){
							case uavcan_node_ExecuteCommand_Request_1_2_COMMAND_DATA_CALIBRATION_SUBMIT:
								/* Save Data to Message Buffer */
								_mscs_calibration_event_msg.timestamp     = system_timestamp;
								_mscs_calibration_event_msg.node_id       = MSCS_ID_LEFT_ADP;
								_mscs_calibration_event_msg.event_type    = uavcan_node_ExecuteCommand_Request_1_2_COMMAND_DATA_CALIBRATION_SUBMIT;
								_mscs_calibration_event_msg.event_data[0] = command_payload[3];
								_mscs_calibration_event_msg.event_data[1] = command_payload[4];
								_mscs_calibration_event_msg.event_data[2] = command_payload[5];
								_mscs_calibration_event_msg.event_handled = 0;

								/* Publish Calibration Event */
								_mscs_calibration_event_pub.publish(_mscs_calibration_event_msg);

								break;

							default:
								break;
						}

						break;

					case MSCS_ID_RIGHT_ADP:
						/* Save data to proper message depending on source */
						switch (_command_msg.command){
							case uavcan_node_ExecuteCommand_Request_1_2_COMMAND_DATA_CALIBRATION_SUBMIT:
								/* Save Data to Message Buffer */
								_mscs_calibration_event_msg.timestamp     = system_timestamp;
								_mscs_calibration_event_msg.node_id       = MSCS_ID_RIGHT_ADP;
								_mscs_calibration_event_msg.event_type    = uavcan_node_ExecuteCommand_Request_1_2_COMMAND_DATA_CALIBRATION_SUBMIT;
								_mscs_calibration_event_msg.event_data[0] = command_payload[3];
								_mscs_calibration_event_msg.event_data[1] = command_payload[4];
								_mscs_calibration_event_msg.event_data[2] = command_payload[5];
								_mscs_calibration_event_msg.event_handled = 0;

								/* Publish Calibration Event */
								_mscs_calibration_event_pub.publish(_mscs_calibration_event_msg);

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
	uavcan_node_ExecuteCommand_Request_1_2 _command_msg{};
	size_t  _command_buffer_size;
	uint8_t _command_source_identifier;

	/* uORB */
	mscs_status_s 					                     _mscs_status_msg{0};
	mscs_calibration_event_s                                             _mscs_calibration_event_msg{0};

	uORB::Subscription _mscs_status_sub                                  {ORB_ID(mscs_status)};

	uORB::Publication<mscs_calibration_event_s>                          _mscs_calibration_event_pub       {ORB_ID(mscs_calibration_event)};

	/* Node IDs */
	CanardNodeID _left_adp_node_id;
	CanardNodeID _right_adp_node_id;
};
