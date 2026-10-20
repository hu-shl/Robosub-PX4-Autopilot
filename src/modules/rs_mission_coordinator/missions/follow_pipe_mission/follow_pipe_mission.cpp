#include "follow_pipe_mission.hpp"

#include <px4_platform_common/log.h>
#include <uORB/topics/vehicle_status.h>

uint8_t FollowPipeMission::required_nav_state() const
{
	return vehicle_status_s::NAVIGATION_STATE_OFFBOARD;
}

void FollowPipeMission::start()
{
	PX4_INFO("Follow pipe mission started");
}

void FollowPipeMission::update(MissionContext &)
{
}

void FollowPipeMission::stop()
{
	PX4_INFO("Follow pipe mission stopped");
}
