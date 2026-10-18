px4_add_module(
	MODULE modules__rs_mission_control
	MAIN rs_mission_control
	STACK_MAIN 1200
	COMPILE_FLAGS
	SRCS
		rs_mission_control.cpp
		rs_mission_control.hpp
	MODULE_CONFIG
		rs_mission_control.yaml
	DEPENDS
		px4_work_queue
	)
