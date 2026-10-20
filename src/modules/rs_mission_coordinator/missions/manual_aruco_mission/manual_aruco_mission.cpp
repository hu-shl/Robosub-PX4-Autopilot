#include "manual_aruco_mission.hpp"

#include <px4_platform_common/log.h>
#include <uORB/topics/vehicle_status.h>

uint8_t ManualArucoMission::required_nav_state() const
{
	return vehicle_status_s::NAVIGATION_STATE_MANUAL;
}

void ManualArucoMission::start()
{
	PX4_INFO("Manual ArUco mission started");
}

void ManualArucoMission::update(MissionContext &context)
{
	_controller.update(context.manual_control, context.thrust, context.torque, context.buoyancy, context.arm, context.now);
}

void ManualArucoMission::stop()
{
	PX4_INFO("Manual ArUco mission stopped");
}
