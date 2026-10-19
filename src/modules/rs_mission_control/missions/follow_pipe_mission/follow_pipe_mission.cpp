#include "follow_pipe_mission.hpp"

#include <px4_platform_common/log.h>

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
