add_subdirectory(missions)
add_subdirectory(controllers)

px4_add_module(
	MODULE modules__rs_mission_coordinator
	MAIN rs_mission_coordinator
	STACK_MAIN 1200
	COMPILE_FLAGS
	SRCS
		rs_mission_coordinator.cpp
		rs_mission_coordinator.hpp
	MODULE_CONFIG
		rs_pid_gains.yaml
		rs_mission_id.yaml
	DEPENDS
		px4_work_queue
		rs_missions
		rs_controllers
	)
