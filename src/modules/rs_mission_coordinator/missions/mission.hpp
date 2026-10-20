#pragma once

#include <cstdint>

#include <drivers/drv_hrt.h>

#include <uORB/topics/arm_control.h>
#include <uORB/topics/buoyancy_control.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

#include "../controllers/position_controller/position_controller.hpp"

struct MissionContext {
	const manual_control_setpoint_s &manual_control;
	const vehicle_odometry_s &odometry;
	PositionController &position_controller;
	PositionController::AxisGains x_gains;
	PositionController::AxisGains y_gains;
	PositionController::AxisGains z_gains;
	vehicle_thrust_setpoint_s &thrust;
	vehicle_torque_setpoint_s &torque;
	buoyancy_control_s &buoyancy;
	arm_control_s &arm;
	hrt_abstime now;
};

class Mission
{
public:
	virtual ~Mission() = default;

	virtual uint8_t required_nav_state() const = 0;
	virtual void start() = 0;
	virtual void update(MissionContext &context) = 0;
	virtual void stop() = 0;
};
