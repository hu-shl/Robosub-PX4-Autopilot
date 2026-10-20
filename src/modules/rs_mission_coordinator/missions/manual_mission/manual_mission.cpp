#include "manual_mission.hpp"

#include <px4_platform_common/log.h>
#include <uORB/topics/vehicle_status.h>

uint8_t ManualMission::required_nav_state() const
{
	return vehicle_status_s::NAVIGATION_STATE_MANUAL;
}

void ManualMission::start()
{
	PX4_INFO("Manual mission started");
}

void ManualMission::update(MissionContext &context)
{
	_controller.update(context.manual_control, context.thrust, context.torque, context.buoyancy, context.arm, context.now);
}

void ManualMission::stop()
{
	PX4_INFO("Manual mission stopped");
}
