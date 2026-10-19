#include "manual_controller.hpp"

#include <math.h>

#include <px4_platform_common/log.h>

void ManualController::update(const manual_control_setpoint_s &manual, vehicle_thrust_setpoint_s &thrust,
			      vehicle_torque_setpoint_s &torque, buoyancy_control_s &buoyancy, arm_control_s &arm,
			      hrt_abstime now)
{
	torque.timestamp = now;
	torque.timestamp_sample = now;
	thrust.timestamp = now;
	thrust.timestamp_sample = now;

	const int manual_mode = static_cast<int>(lroundf(manual.aux1));
	PX4_INFO("manual mode: %d", manual_mode);

	thrust.xyz[0] = 0.f;
	thrust.xyz[1] = 0.f;
	thrust.xyz[2] = 0.f;
	torque.xyz[0] = 0.f;
	torque.xyz[1] = 0.f;
	torque.xyz[2] = 0.f;

	for (int i = 0; i < 4; i++) {
		buoyancy.tank_command[i] = 0.f;
	}

	for (int i = 0; i < 6; i++) {
		arm.servo_command[i] = 0.f;
	}

	switch (manual_mode) {
	case -1:
		buoyancy.tank_command[0] = manual.roll;
		buoyancy.tank_command[1] = manual.pitch;
		buoyancy.tank_command[2] = manual.yaw;
		buoyancy.tank_command[3] = manual.throttle;
		buoyancy.tank_command[4] = manual.aux3;
		buoyancy.tank_command[5] = manual.aux4;
		buoyancy.tank_command[6] = manual.aux5;
		buoyancy.tank_command[7] = manual.aux6;
		PX4_INFO("buoyancy mode: %d", manual_mode);
		break;

	case 0:
		torque.xyz[0] = manual.aux4;
		torque.xyz[1] = manual.pitch;
		torque.xyz[2] = manual.roll * 0.2f;

		thrust.xyz[0] = manual.throttle;
		thrust.xyz[1] = 0.f;
		thrust.xyz[2] = manual.yaw * 0.6f;
		PX4_INFO("thrusters mode: %d", manual_mode);
		break;

	case 1:
		arm.servo_command[0] = manual.roll;
		arm.servo_command[1] = manual.pitch;
		arm.servo_command[2] = manual.yaw;
		arm.servo_command[3] = manual.throttle;
		arm.servo_command[4] = manual.aux3;
		arm.servo_command[5] = manual.aux4;
		arm.servo_command[6] = manual.aux5;
		arm.servo_command[7] = manual.aux6;
		PX4_INFO("arm mode: %d", manual_mode);
		break;
	}
}
