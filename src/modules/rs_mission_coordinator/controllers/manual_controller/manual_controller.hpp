#pragma once

#include <drivers/drv_hrt.h>

#include <uORB/topics/arm_control.h>
#include <uORB/topics/buoyancy_control.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

class ManualController
{
public:
	void update(const manual_control_setpoint_s &manual, vehicle_thrust_setpoint_s &thrust,
		    vehicle_torque_setpoint_s &torque, buoyancy_control_s &buoyancy, arm_control_s &arm, hrt_abstime now);
};
