/*
 * WiVRn VR streaming
 * Copyright (C) 2022  Guillaume Meunier <guillaume.meunier@centraliens.net>
 * Copyright (C) 2022  Patrick Nicolas <patricknicolas@laposte.net>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "application.h"
#include "stream.h"
#include "utils/ranges.h"
#include "xr/check.h"
#include <ranges>
#include <spdlog/spdlog.h>
#include <thread>

static uint8_t cast_flags(XrSpaceLocationFlags location, XrSpaceVelocityFlags velocity)
{
	uint8_t flags = 0;
	if (location & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT)
		flags |= from_headset::tracking::orientation_valid;

	if (location & XR_SPACE_LOCATION_POSITION_VALID_BIT)
		flags |= from_headset::tracking::position_valid;

	if (velocity & XR_SPACE_VELOCITY_LINEAR_VALID_BIT)
		flags |= from_headset::tracking::linear_velocity_valid;

	if (velocity & XR_SPACE_VELOCITY_ANGULAR_VALID_BIT)
		flags |= from_headset::tracking::angular_velocity_valid;

	if (location & XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT)
		flags |= from_headset::tracking::orientation_tracked;

	if (location & XR_SPACE_LOCATION_POSITION_TRACKED_BIT)
		flags |= from_headset::tracking::position_tracked;

	return flags;
}

static from_headset::tracking::pose locate_space(device_id device, XrSpace space, XrSpace reference, XrTime time)
{
	XrSpaceVelocity velocity{
	        .type = XR_TYPE_SPACE_VELOCITY,
	};

	XrSpaceLocation location{
	        .type = XR_TYPE_SPACE_LOCATION,
	        .next = &velocity,
	};

	xrLocateSpace(space, reference, time, &location);

	return from_headset::tracking::pose{
	        .device = device,
	        .pose = location.pose,
	        .linear_velocity = velocity.linearVelocity,
	        .angular_velocity = velocity.angularVelocity,
	        .flags = cast_flags(location.locationFlags, velocity.velocityFlags),
	};
}

namespace
{
class timer
{
	xr::instance & instance;
	XrTime start = instance.now();
	XrDuration duration = 0;

public:
	timer(xr::instance & instance) :
	        instance(instance) {}
	void pause()
	{
		duration += instance.now() - start;
	}
	void resume()
	{
		start = instance.now();
	}
	XrDuration count()
	{
		pause();
		return duration;
	}
};
} // namespace

static std::optional<std::array<from_headset::hand_tracking::pose, XR_HAND_JOINT_COUNT_EXT>> locate_hands(xr::hand_tracker & hand, XrSpace space, XrTime time)
{
	auto joints = hand.locate(space, time);

	if (joints)
	{
		std::array<from_headset::hand_tracking::pose, XR_HAND_JOINT_COUNT_EXT> poses;
		for (int i = 0; i < XR_HAND_JOINT_COUNT_EXT; i++)
		{
			poses[i] = {
			        .pose = (*joints)[i].first.pose,
			        .linear_velocity = (*joints)[i].second.linearVelocity,
			        .angular_velocity = (*joints)[i].second.angularVelocity,
			        .radius = uint16_t((*joints)[i].first.radius * 10'000),
			};

			if ((*joints)[i].first.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT)
				poses[i].flags |= from_headset::hand_tracking::orientation_valid;

			if ((*joints)[i].first.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT)
				poses[i].flags |= from_headset::hand_tracking::position_valid;

			if ((*joints)[i].second.velocityFlags & XR_SPACE_VELOCITY_LINEAR_VALID_BIT)
				poses[i].flags |= from_headset::hand_tracking::linear_velocity_valid;

			if ((*joints)[i].second.velocityFlags & XR_SPACE_VELOCITY_ANGULAR_VALID_BIT)
				poses[i].flags |= from_headset::hand_tracking::angular_velocity_valid;

			if ((*joints)[i].first.locationFlags & XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT)
				poses[i].flags |= from_headset::hand_tracking::orientation_tracked;

			if ((*joints)[i].first.locationFlags & XR_SPACE_LOCATION_POSITION_TRACKED_BIT)
				poses[i].flags |= from_headset::hand_tracking::position_tracked;
		}

		return poses;
	}
	else
		return std::nullopt;
}

void scenes::stream::tracking()
{
#ifdef __ANDROID__
	// Runtime may use JNI and needs the thread to be attached
	application::instance().setup_jni();
#endif

	std::vector<XrSpace> spaces = {
	        application::view(),
	        application::left_aim(),
	        application::left_grip(),
	        application::right_aim(),
	        application::right_grip(),
	};
	std::vector<device_id> space_devices = {
	        device_id::HEAD,
	        device_id::LEFT_AIM,
	        device_id::LEFT_GRIP,
	        device_id::RIGHT_AIM,
	        device_id::RIGHT_GRIP,
	};

#ifdef XR_KHR_locate_spaces
	PFN_xrLocateSpaces xrLocateSpaces = nullptr;
	auto & xr_instance = application::get_xr_instance();
	if (xr_instance.has_extension(XR_KHR_LOCATE_SPACES_EXTENSION_NAME))
	{
		xrLocateSpaces = xr_instance.get_proc<PFN_xrLocateSpacesKHR>("xrLocateSpacesKHR");
	}
	std::vector<XrSpaceVelocityDataKHR> velocities(spaces.size());
	std::vector<XrSpaceLocationDataKHR> locations(spaces.size());
	XrSpacesLocateInfoKHR spaces_locate_info{
	        .type = XR_TYPE_SPACES_LOCATE_INFO_KHR,
	        .baseSpace = local_floor,
	        .spaceCount = uint32_t(spaces.size()),
	        .spaces = spaces.data(),
	};
	XrSpaceVelocitiesKHR velocity{
	        .type = XR_TYPE_SPACE_VELOCITIES_KHR,
	        .velocityCount = uint32_t(velocities.size()),
	        .velocities = velocities.data(),
	};
	XrSpaceLocationsKHR location{
	        .type = XR_TYPE_SPACE_LOCATIONS_KHR,
	        .next = &velocity,
	        .locationCount = uint32_t(locations.size()),
	        .locations = locations.data(),
	};
#endif

	XrSpace view_space = application::view();
	XrDuration tracking_period = 1'000'000; // Send tracking data every 1ms
	const XrDuration dt = 100'000;          // Wake up 0.1ms before measuring the position

	XrTime t0 = instance.now();
	from_headset::tracking packet{};

	while (not exiting)
	{
		try
		{
			XrTime now = instance.now();
			if (now < t0)
				std::this_thread::sleep_for(std::chrono::nanoseconds(t0 - now - dt));

			// If thread can't keep up, skip timestamps
			t0 = std::max(t0, now);

			timer t(instance);

			XrDuration prediction = tracking_prediction_offset;
			// 1 or 2 samples
			for (XrDuration Δt = 0; Δt <= prediction; Δt += std::max<XrDuration>(1, prediction))
			{
				from_headset::hand_tracking hands{};

				packet.production_timestamp = t0;
				hands.production_timestamp = t0;

				packet.timestamp = t0 + Δt;
				hands.timestamp = t0 + Δt;

				try
				{
					auto [flags, views] = session.locate_views(XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, t0 + Δt, view_space);
					assert(views.size() == packet.views.size());

					for (auto [i, j]: utils::zip(views, packet.views))
					{
						j.pose = i.pose;
						j.fov = i.fov;
					}

					packet.flags = flags;
					packet.device_poses.clear();

#ifdef XR_KHR_locate_spaces
					if (xrLocateSpaces)
					{
						std::lock_guard lock(local_floor_mutex);
						spaces_locate_info.baseSpace = local_floor;
						spaces_locate_info.time = packet.timestamp;
						check(xrLocateSpaces(session, &spaces_locate_info, &location), "xrLocateSpaces");
						for (size_t i = 0; i < spaces.size(); ++i)
						{
							const auto & location = locations[i];
							const auto & velocity = velocities[i];
							const auto device = space_devices[i];
							packet.device_poses.push_back(from_headset::tracking::pose{
							        .device = device,
							        .pose = location.pose,
							        .linear_velocity = velocity.linearVelocity,
							        .angular_velocity = velocity.angularVelocity,
							        .flags = cast_flags(location.locationFlags, velocity.velocityFlags),
							});
						}
					}
					else
#endif
					{
						std::lock_guard lock(local_floor_mutex);
						for (auto [device, space]: utils::zip(space_devices, spaces))
						{
							packet.device_poses.push_back(locate_space(device, space, local_floor, t0 + Δt));
						}
					}

					t.pause();
					network_session->send_control(packet);
					t.resume();

					if (application::get_hand_tracking_supported())
					{
						hands.hand = xrt::drivers::wivrn::from_headset::hand_tracking::left;
						hands.joints = locate_hands(application::get_left_hand(), local_floor, hands.timestamp);
						t.pause();
						network_session->send_stream(hands);
						t.resume();

						hands.hand = xrt::drivers::wivrn::from_headset::hand_tracking::right;
						hands.joints = locate_hands(application::get_right_hand(), local_floor, hands.timestamp);
						t.pause();
						network_session->send_stream(hands);
						t.resume();
					}

					XrDuration busy_time = t.count();
					// Target: polling between 1 and 5ms, with 20% busy time
					tracking_period = std::clamp<XrDuration>(std::lerp(tracking_period, busy_time * 5, 0.2), 1'000'000, 5'000'000);
				}
				catch (const std::system_error & e)
				{
					if (e.code().category() != xr::error_category() or
					    e.code().value() != XR_ERROR_TIME_INVALID)
						throw;
				}
			}

			t0 += tracking_period;
		}
		catch (std::exception & e)
		{
			spdlog::info("Exception in tracking thread, exiting: {}", e.what());
			exit();
		}
	}
}

void scenes::stream::operator()(to_headset::prediction_offset && packet)
{
	if (packet.offset.count() >= 0)
		tracking_prediction_offset = std::lerp(packet.offset.count(), tracking_prediction_offset.load(), 0.2);
}
