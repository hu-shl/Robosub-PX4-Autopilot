#pragma once

#include <drivers/drv_hrt.h>

#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>

#include "../pid_controller/pid_controller.hpp"

class PositionController
{
public:
	struct AxisGains {
		float kp;
		float ki;
		float kd;
	};

	void update(const vehicle_odometry_s &odom, vehicle_thrust_setpoint_s &thrust, hrt_abstime now,
		    const AxisGains &x_gains, const AxisGains &y_gains, const AxisGains &z_gains);
	void reset();

private:
	PIDController _pid_x{1.f};
	PIDController _pid_y{1.f};
	PIDController _pid_z{1.f};
	bool _hold_position_set{false};
	float _hold_x{0.f};
	float _hold_y{0.f};
	float _hold_z{0.f};
	hrt_abstime _last_run{0};
};
