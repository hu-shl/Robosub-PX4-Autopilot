#include "pid_controller.hpp"

#include <mathlib/math/Functions.hpp>

void PIDController::update(float kp, float ki, float kd, float setpoint, float current_value, float dt)
{
	if (dt <= 0.f) {
		return;
	}

	const float error = setpoint - current_value;
	const float proportional = kp * error;

	_integral += error * dt;
	_integral = math::constrain(_integral, -0.5f, 0.5f);
	const float integral = ki * _integral;

	const float derivative = (error - _last_error) / dt;
	const float derivative_output = kd * derivative;

	_output = math::constrain(proportional + integral + derivative_output, -_output_limit, _output_limit);
	_last_error = error;
}

void PIDController::reset()
{
	_integral = 0.f;
	_last_error = 0.f;
	_output = 0.f;
}
