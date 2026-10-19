#pragma once

class PIDController
{
public:
	explicit PIDController(float output_limit = 1.f) : _output_limit(output_limit) {}

	void update(float kp, float ki, float kd, float setpoint, float current_value, float dt);
	float output() const { return _output; }
	void reset();

private:
	float _output_limit;
	float _integral{0.f};
	float _last_error{0.f};
	float _output{0.f};
};
