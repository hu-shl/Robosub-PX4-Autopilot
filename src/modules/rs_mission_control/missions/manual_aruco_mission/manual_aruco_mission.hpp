#pragma once

#include "../mission.hpp"
#include "../../controllers/manual_controller/manual_controller.hpp"

class ManualArucoMission : public Mission
{
public:
	RequiredMode required_mode() const override { return RequiredMode::Manual; }
	void start() override;
	void update(MissionContext &context) override;
	void stop() override;

private:
	ManualController _controller;
};
