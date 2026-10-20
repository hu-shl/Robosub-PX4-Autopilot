#pragma once

#include "../mission.hpp"

class FollowPipeMission : public Mission
{
public:
	uint8_t required_nav_state() const override;
	void start() override;
	void update(MissionContext &context) override;
	void stop() override;
};
