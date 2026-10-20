#pragma once

#include "../mission.hpp"
#include "../../controllers/manual_controller/manual_controller.hpp"

class ManualArucoMission : public Mission
{
public:
	uint8_t required_nav_state() const override;
	void start() override;
	void update(MissionContext &context) override;
	void stop() override;

private:
	ManualController _controller;
};
