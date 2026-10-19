px4_add_library(rs_controllers
	manual_controller/manual_controller.cpp
	manual_controller/manual_controller.hpp
	pid_controller/pid_controller.cpp
	pid_controller/pid_controller.hpp
	position_controller/position_controller.cpp
	position_controller/position_controller.hpp
)

target_include_directories(rs_controllers PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/..
)
