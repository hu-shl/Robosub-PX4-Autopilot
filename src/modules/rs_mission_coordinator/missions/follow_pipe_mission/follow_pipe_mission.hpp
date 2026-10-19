#pragma once

#include "../mission.hpp"

class FollowPipeMission : public Mission
{
public:
	RequiredMode required_mode() const override { return RequiredMode::Offboard; }
	void start() override;
	void update(MissionContext &context) override;
	void stop() override;
};
