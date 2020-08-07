/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#pragma once

#include "../mavlink_messages.h"

#include <uORB/topics/vehicle_roi.h>

class MavlinkStreamVehicleROI : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamVehicleROI::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "VEHICLE_ROI";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_COMMAND_LONG;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamVehicleROI(mavlink);
	}

	unsigned get_size() override
	{
		return 0;	// commands stream is not regular and not predictable
	}

private:
	uORB::Subscription _roi_sub{ORB_ID(vehicle_roi)};

	/* do not allow top copying this class */
	MavlinkStreamVehicleROI(MavlinkStreamVehicleROI &) = delete;
	MavlinkStreamVehicleROI &operator = (const MavlinkStreamVehicleROI &) = delete;

protected:
	explicit MavlinkStreamVehicleROI(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		struct vehicle_roi_s _vroi;
		bool sent = false;

		if (_roi_sub.update(&_vroi)) {
			if (_vroi.mode == vehicle_roi_s::ROI_LOCATION) {
				mavlink_command_long_t msg = {};
				msg.target_system = 0;
				msg.target_component = 0;
				msg.command = MAV_CMD_DO_SET_ROI_LOCATION;
				msg.confirmation = 0;
				msg.param1 = 0;
				msg.param2 = (double)NAN;
				msg.param3 = (double)NAN;
				msg.param4 = (double)NAN;
				msg.param5 = _vroi.lat;
				msg.param6 = _vroi.lon;
				msg.param7 = _vroi.alt;
				mavlink_msg_command_long_send_struct(_mavlink->get_channel(), &msg);
				sent = true;

			}
		}

		return sent;
	}
};
