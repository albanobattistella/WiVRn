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

#include "view_list.h"
#include "pose_list.h"
#include "xrt_cast.h"
#include <cmath>

tracked_views view_list::interpolate(const tracked_views & a, const tracked_views & b, float t)
{
	tracked_views result = b;
	result.relation = pose_list::interpolate(a.relation, b.relation, t);
	return result;
}

tracked_views view_list::extrapolate(const tracked_views & a, const tracked_views & b, uint64_t ta, uint64_t tb, uint64_t t)
{
	tracked_views result = t < ta ? a : b;
	result.relation = pose_list::extrapolate(a.relation, b.relation, ta, tb, t);
	return result;
}

void view_list::update_tracking(const from_headset::tracking & tracking, const clock_offset & offset)
{
	for (const auto & pose: tracking.device_poses)
	{
		if (pose.device != device_id::HEAD)
			continue;

		tracked_views view{};

		view.relation = pose_list::convert_pose(pose);
		view.flags = tracking.flags;

		for (size_t eye = 0; eye < 2; ++eye)
		{
			view.poses[eye] = xrt_cast(tracking.views[eye].pose);
			view.fovs[eye] = xrt_cast(tracking.views[eye].fov);

			if (tracking.production_timestamp < tracking.timestamp and tracking.flags & from_headset::tracking::flags::angular_velocity_valid)
			{
				auto & fov = view.fovs[eye];
				double t = (tracking.timestamp - tracking.production_timestamp) / 1'000'000'000.;
				t *= 0.3;
				fov.angle_left -= std::abs(pose.angular_velocity.y * t);
				fov.angle_right += std::abs(pose.angular_velocity.y * t);
				fov.angle_up += std::abs(pose.angular_velocity.x * t);
				fov.angle_down -= std::abs(pose.angular_velocity.x * t);

				// clamp to roughly 85°
				fov.angle_left = std::max(fov.angle_left, -1.5f);
				fov.angle_down = std::max(fov.angle_down, -1.5f);
				fov.angle_up = std::min(fov.angle_up, 1.5f);
				fov.angle_right = std::min(fov.angle_right, 1.5f);
			}
		}

		add_sample(tracking.production_timestamp, tracking.timestamp, view, offset);
		return;
	}
}
