/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "rs_mission_coordinator.hpp"

#include <drivers/drv_hrt.h>
#include <systemlib/mavlink_log.h>

#include "missions/manual_mission/manual_mission.hpp"
#include "missions/manual_aruco_mission/manual_aruco_mission.hpp"
#include "missions/follow_pipe_mission/follow_pipe_mission.hpp"

static constexpr hrt_abstime MODE_REQUEST_RETRY_INTERVAL_US = 1'000'000;
static constexpr hrt_abstime MISSION_UPDATE_INTERVAL_US = 20'000;

ModuleBase::Descriptor RS_MissionCoordinator::desc{task_spawn, custom_command, print_usage};

/**
 * @brief Class constructor
 */
RS_MissionCoordinator::RS_MissionCoordinator() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{}

/**
 * @brief Class destructor
 */
RS_MissionCoordinator::~RS_MissionCoordinator()
{
	perf_free(_loop_perf);
}

/**
 * @brief static function to instantiate an instance of the class
 *
 * @param argc Number of arguments
 * @param argv Argument Array
 *
 * @return PX4_OK if successfull, otherwise PX4_ERROR
 */
int RS_MissionCoordinator::task_spawn(int argc, char *argv[])
{
	RS_MissionCoordinator *instance = new RS_MissionCoordinator();

	if (!instance) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	desc.object.store(instance);
	desc.task_id = task_id_is_work_queue;

	if (instance->init()) {
		instance->ScheduleNow();
		return PX4_OK;
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

/**
 * @brief init function part of the constructor to be able to have error handling.
 *
 * @return PX4_OK if successfull, otherwise PX4_ERROR
 */
bool RS_MissionCoordinator::init()
{
	updateParams();

	// Register the callbacks
	if (!_manual_control_setpoint_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	if (!_vehicle_health_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		_manual_control_setpoint_sub.unregisterCallback();
		return false;
	}

	if (!_vehicle_status_sub.registerCallback()) {
		PX4_ERR("vehicle status callback registration failed");
		_manual_control_setpoint_sub.unregisterCallback();
		_vehicle_health_sub.unregisterCallback();
		return false;
	}

	if (!_parameter_update_sub.registerCallback()) {
		PX4_ERR("parameter update callback registration failed");
		_manual_control_setpoint_sub.unregisterCallback();
		_vehicle_health_sub.unregisterCallback();
		_vehicle_status_sub.unregisterCallback();
		return false;
	}

	// advertise to be published topics
	_vehicle_torque_setpoint_pub.advertise();
	_vehicle_thrust_setpoint_pub.advertise();
	_buoyancy_control_pub.advertise();
	_arm_control_pub.advertise();
	_rs_mission_id_pub.advertise();
	publish_mission_id();

	_last_mission_tick = hrt_absolute_time();
	ScheduleOnInterval(MISSION_UPDATE_INTERVAL_US);

	return true;
}

/**
 * @brief Main Module loop
 */
void RS_MissionCoordinator::Run()
{
	// Check if this module should still exist
	if (should_exit()) {
		_manual_control_setpoint_sub.unregisterCallback();
		_vehicle_health_sub.unregisterCallback();
		_vehicle_status_sub.unregisterCallback();
		_parameter_update_sub.unregisterCallback();
		exit_and_cleanup(desc);
		return;
	}

	// begin performance monitoring
	perf_begin(_loop_perf);

	orb_advert_t mavlink_log_pub = nullptr;
	hrt_abstime now = hrt_absolute_time();  // get current time


	const bool mission_tick_due = now - _last_mission_tick >= MISSION_UPDATE_INTERVAL_US;

	if (mission_tick_due) {
		_last_mission_tick = now;
	}

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
		publish_mission_id();
	}

	if (_vehicle_health_sub.updated()) {
		vehicle_health_s vehicleHealth{};
		_vehicle_health_sub.copy(&vehicleHealth);
		if (vehicleHealth.battery[0] > 50.0f) {
			mavlink_log_info(&mavlink_log_pub, "temp battery high!: %f", static_cast<double>(vehicleHealth.battery[0]));
		}
	}

	// Check if flight-mode has changed
	if (_vehicle_status_sub.updated()) {
		vehicle_status_s status{};
		_vehicle_status_sub.copy(&status);
		update_mission_selection(status);

		if (_coordinator_state == CoordinatorState::WaitingForMode && _active_mission != nullptr) {
			const uint8_t required_nav_state = _active_mission->required_nav_state();

			if (status.nav_state == required_nav_state) {
				_active_mission->start();
				_coordinator_state = CoordinatorState::Running;
				PX4_INFO("Mission mode confirmed");

			} else if (now - _last_mode_request >= MODE_REQUEST_RETRY_INTERVAL_US) {
				request_mission_mode(status);
			}
		}

		if (_coordinator_state == CoordinatorState::Running && _active_mission != nullptr) {
			const uint8_t required_nav_state = _active_mission->required_nav_state();

			if (status.nav_state != required_nav_state) {
				_active_mission->stop();
				enter_safe_state("vehicle left the mission's required mode");
			}
		}
	}

	// Run the mission update and publish the results
	if (_coordinator_state == CoordinatorState::Running
	    && _active_mission != nullptr
	    && (_manual_control_setpoint_sub.updated() || mission_tick_due)) {
		manual_control_setpoint_s manualControlInput{};
		vehicle_odometry_s vehicle_odo_data{};
		vehicle_thrust_setpoint_s thrustSetpoint{};
		vehicle_torque_setpoint_s torqueSetpoint{};
		buoyancy_control_s buoyancyControl{};
		arm_control_s armControl{};
		_vehicle_odo_sub.copy(&vehicle_odo_data);
		_manual_control_setpoint_sub.copy(&manualControlInput);

		MissionContext context{
			manualControlInput,
			vehicle_odo_data,
			_position_controller,
			{_rs_x_kp.get(), _rs_x_ki.get(), _rs_x_kd.get()},
			{_rs_y_kp.get(), _rs_y_ki.get(), _rs_y_kd.get()},
			{_rs_z_kp.get(), _rs_z_ki.get(), _rs_z_kd.get()},
			thrustSetpoint,
			torqueSetpoint,
			buoyancyControl,
			armControl,
			now
		};
		_active_mission->update(context);

		_vehicle_thrust_setpoint_pub.publish(thrustSetpoint);
		_vehicle_torque_setpoint_pub.publish(torqueSetpoint);
		_buoyancy_control_pub.publish(buoyancyControl);
		_arm_control_pub.publish(armControl);
	}

	perf_end(_loop_perf);
}

void RS_MissionCoordinator::update_mission_selection(const vehicle_status_s &status)
{
	const int32_t mission_id = _param_rs_mission_id.get();

	if (mission_id == _active_mission_id) {
		return;
	}

	if (_active_mission != nullptr && _coordinator_state == CoordinatorState::Running) {
		_active_mission->stop();
	}

	_active_mission = nullptr;
	_active_mission_id = mission_id;
	_coordinator_state = CoordinatorState::NoMission;

	switch (static_cast<MissionId>(mission_id)) {
	case MissionId::Off:
		PX4_INFO("No mission selected");
		return;

	case MissionId::Manual: {
			static ManualMission mission;
			_active_mission = &mission;
		}
		break;

	case MissionId::ManualAruco: {
			static ManualArucoMission mission;
			_active_mission = &mission;
		}
		break;

	case MissionId::FollowPipe: {
			static FollowPipeMission mission;
			_active_mission = &mission;
		}
		break;

	default:
		enter_safe_state("unsupported mission ID");
		return;
	}

	const uint8_t required_nav_state = _active_mission->required_nav_state();

	// If PX4 is already in the selected mission's required mode, start it
	// immediately; otherwise, request the mode and wait for confirmation.
	if (status.nav_state == required_nav_state) {
		_active_mission->start();
		_coordinator_state = CoordinatorState::Running;
		PX4_INFO("Selected mission ID: %d is ready", static_cast<int>(mission_id));

	} else {
		_coordinator_state = CoordinatorState::WaitingForMode;
		request_mission_mode(status);

		if (_coordinator_state == CoordinatorState::WaitingForMode) {
			PX4_INFO("Selected mission ID: %d, waiting for required mode", static_cast<int>(mission_id));
		}
	}
}

void RS_MissionCoordinator::request_mission_mode(const vehicle_status_s &status)
{
	vehicle_command_s command{};
	command.timestamp = hrt_absolute_time();
	command.command = vehicle_command_s::VEHICLE_CMD_SET_NAV_STATE;
	command.param1 = static_cast<float>(_active_mission->required_nav_state());
	_last_mode_request = command.timestamp;
	command.target_system = status.system_id;
	command.target_component = status.component_id;
	command.source_system = status.system_id;
	command.source_component = status.component_id;

	if (!_vehicle_command_pub.publish(command)) {
		enter_safe_state("failed to publish mission mode request");
	}
}

void RS_MissionCoordinator::enter_safe_state(const char *reason)
{
	_active_mission = nullptr;
	_coordinator_state = CoordinatorState::Safe;
	PX4_WARN("Safe-state stub active: %s; no UUV-specific safe action is implemented", reason);
}

void RS_MissionCoordinator::publish_mission_id()
{
	const int32_t mission_id = _param_rs_mission_id.get();

	if (mission_id != static_cast<int32_t>(MissionId::Off)
	    && mission_id != static_cast<int32_t>(MissionId::Manual)
	    && mission_id != static_cast<int32_t>(MissionId::ManualAruco)
	    && mission_id != static_cast<int32_t>(MissionId::FollowPipe)) {
		PX4_ERR("unsupported mission ID: %d", static_cast<int>(mission_id));
		return;
	}

	if (mission_id == _last_published_mission_id) {
		return;
	}

	rs_mission_id_s msg{};
	msg.timestamp = hrt_absolute_time();
	msg.mission_id = static_cast<uint8_t>(mission_id);
	_rs_mission_id_pub.publish(msg);

	_last_published_mission_id = mission_id;
	PX4_INFO("Selected mission ID: %d", static_cast<int>(mission_id));
}

/**
 * @brief function run when a custom flag or command other than the standard ones defined in module.h is passed
 *
 * @param argc Number of arguments
 * @param argv Argument Array
 *
 * @return PX4_OK if successfull, otherwise PX4_ERROR
 */
int RS_MissionCoordinator::custom_command(int argc, char *argv[])
{
	// for now we don't define any custom flags / commands
	return print_usage("unknown command");
}

/**
 * @brief function run when a help command or -h flag is passed
 *
 * @param argc Number of arguments
 * @param argv Argument Array
 *
 * @return PX4_OK if successfull, otherwise PX4_ERROR
 */
int RS_MissionCoordinator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_USAGE_NAME("rs_mission_coordinator", "controller");
	PRINT_MODULE_USAGE_COMMAND("start")
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

/**
 * @brief C Export for compaitibility with NuttX shell
 */
extern "C" __EXPORT int rs_mission_coordinator_main(int argc, char *argv[])
{
	return ModuleBase::main(RS_MissionCoordinator::desc, argc, argv);
}
