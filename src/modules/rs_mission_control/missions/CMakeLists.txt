px4_add_library(rs_missions
	mission.hpp
	manual_mission/manual_mission.cpp
	manual_mission/manual_mission.hpp
	manual_aruco_mission/manual_aruco_mission.cpp
	manual_aruco_mission/manual_aruco_mission.hpp
	follow_pipe_mission/follow_pipe_mission.cpp
	follow_pipe_mission/follow_pipe_mission.hpp
)

target_include_directories(rs_missions PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/..
)
