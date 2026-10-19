#include "position_controller.hpp"

#include <px4_platform_common/log.h>

void PositionController::reset()
{
	_hold_position_set = false;
}

void PositionController::update(const vehicle_odometry_s &odom, vehicle_thrust_setpoint_s &thrust, hrt_abstime now,
				const AxisGains &x_gains, const AxisGains &y_gains, const AxisGains &z_gains)
{
	if (!_hold_position_set) {
		_hold_x = odom.position[0];
		_hold_y = odom.position[1];
		_hold_z = odom.position[2];

		_pid_x.reset();
		_pid_y.reset();
		_pid_z.reset();

		_hold_position_set = true;
		_last_run = now;
		PX4_INFO("Custom PID Hold Start");
	}

	const float dt = (now - _last_run) / 1e6f;

	if (dt > 0.099f) {
		_pid_x.update(x_gains.kp, x_gains.ki, x_gains.kd, _hold_x, odom.position[0], dt);
		_pid_y.update(y_gains.kp, y_gains.ki, y_gains.kd, _hold_y, odom.position[1], dt);
		_pid_z.update(z_gains.kp, z_gains.ki, z_gains.kd, _hold_z, odom.position[2], dt);

		_last_run = now;
		PX4_INFO("Hold Active - X_Err: %.2f | X_Thrust: %.2f",
			 (double)(_hold_x - odom.position[0]), (double)thrust.xyz[0]);
		PX4_INFO("Hold sp %.2f", (double)odom.position[0]);
	}

	thrust.xyz[0] = -_pid_x.output();
	thrust.xyz[1] = -_pid_y.output();
	thrust.xyz[2] = -_pid_z.output();
	thrust.timestamp = now;
}
