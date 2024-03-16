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
 * @file Heartbeat.hpp
 *
 * Defines basic functionality of UAVCAN Heartbeat.1.0 subscription
 *
 * @author Peter van der Perk <peter.vanderperk@nxp.com>
 * @author Daniel Montes <daniel.montestolon@slu.edu>
 */

#pragma once

#include "../NodeManager.hpp"

#include <uavcan/node/Heartbeat_1_0.h>

#include "BaseSubscriber.hpp"

#include <px4_platform_common/module_params.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/mscs_status.h>
#include <uORB/topics/parameter_update.h>

class UavcanHeartbeatSubscriber : public UavcanBaseSubscriber, public ModuleParams
{
public:
	UavcanHeartbeatSubscriber(CanardHandle &handle) : UavcanBaseSubscriber(handle, "", "Heartbeat", 0), ModuleParams(nullptr)
	{

	};

	void subscribe() override
	{
		/* Subscribe to Cyphal Heartbeat messages */
		_canard_handle.RxSubscribe(CanardTransferKindMessage,
					   uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,  // The fixed Subject-ID
					   uavcan_node_Heartbeat_1_0_EXTENT_BYTES_,
					   CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
					   &_subj_sub._canard_sub);

	};

	void callback(const CanardRxTransfer &receive) override
	{
		/* VARIABLE DEFINITIONS */
		CanardNodeID heartbeat_node;
		bool updated = false;
		const hrt_abstime system_timestamp = hrt_absolute_time();

		/* FUNCTION BODY */
		/* Check if uORB has been updated */
		if (_mscs_status_sub.updated()) {
			/* Copy uORB MSCS Status into local buffer */
			_mscs_status_sub.copy(&_mscs_status_msg);

			/* Get all the possibly updated parameters */
			_left_adp_node_id = _mscs_status_msg.left_adp_node_id;
			_right_adp_node_id = _mscs_status_msg.right_adp_node_id;
		}

		//PX4_INFO("HEARTBEAT RECEIVED");

		/* Check if MSCS is active */
		if (_mscs_status_msg.module_active == true) {
			/* Deserialize Heartbeat Structure */
			_heartbeat_message_buffer_size = uavcan_node_Heartbeat_1_0_EXTENT_BYTES_;

			if (uavcan_node_Heartbeat_1_0_deserialize_(&_heartbeat_message, (uint8_t*) receive.payload, &_heartbeat_message_buffer_size) == 0) {
				/* Get current timestamp */
				_mscs_status_msg.timestamp = system_timestamp;

				/* Identify Heartbeat Message */
				heartbeat_node = receive.metadata.remote_node_id;

				/* Save data to proper fields */
				if (heartbeat_node == _left_adp_node_id) {

					/* Set Left ADP as active */
					_mscs_status_msg.left_adp_active = true;

					/* Save all parameters to Status structure */
					_mscs_status_msg.left_adp_last_heartbeat_timestamp = system_timestamp;
					_mscs_status_msg.left_adp_uptime = _heartbeat_message.uptime;
					_mscs_status_msg.left_adp_mode   = _heartbeat_message.mode.value;
					_mscs_status_msg.left_adp_status = _heartbeat_message.vendor_specific_status_code;

					//PX4_INFO("HB - %llu", system_timestamp);

					updated = true;

				} else if (heartbeat_node == _right_adp_node_id) {
					/* Set Right ADP as active */
					_mscs_status_msg.right_adp_active = true;

					/* Save all parameters to Status structure */
					_mscs_status_msg.right_adp_last_heartbeat_timestamp = system_timestamp;
					_mscs_status_msg.right_adp_uptime = _heartbeat_message.uptime;
					_mscs_status_msg.right_adp_mode   = _heartbeat_message.mode.value;
					_mscs_status_msg.right_adp_status = _heartbeat_message.vendor_specific_status_code;

					updated = true;

				}

				/* Publish uORB Message */
				if (updated) {
					_mscs_status_pub.publish(_mscs_status_msg);
				}
			}

		}
	}

private:
	/* Heartbeat Message Buffer */
	uavcan_node_Heartbeat_1_0 _heartbeat_message{};
	size_t _heartbeat_message_buffer_size;

	/* uORB */
	mscs_status_s _mscs_status_msg{};
	uORB::Subscription _mscs_status_sub{ORB_ID(mscs_status)};
	uORB::Publication<mscs_status_s> _mscs_status_pub{ORB_ID(mscs_status)};

	/* Node IDs */
	CanardNodeID _left_adp_node_id;
	CanardNodeID _right_adp_node_id;
};
