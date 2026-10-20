
#pragma once

// ################################################################################
// #	PX4 Libraries
// ################################################################################

// PX4 standard functionality
#include <px4_platform_common/defines.h>				// general macro's
#include <px4_platform_common/module.h>					// Module Base class using CRTP
#include <px4_platform_common/module_params.h>				// Module parameters functionality
#include <uORB/topics/parameter_update.h>
#include <uORB/Subscription.hpp>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>	// workQueue functionality for our module

// Helpers and utilities
#include <lib/perf/perf_counter.h>					// performance counters for top command
#include <mathlib/math/Functions.hpp>					// math helper functions

// uORB pub&sub stuff
#include <uORB/Publication.hpp>						// uORB publication functionality
#include <uORB/SubscriptionCallback.hpp>				// interrupt instead of polling

// Legacy member declarations below require these topic headers.
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>

// ################################################################################
// #	uORB Topics
// ################################################################################

// Setpoints
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/jetson_control.h>			// manual control inputs
#include <uORB/topics/rs_mission_id.h>

// Vehicle status
#include <uORB/topics/vehicle_status.h>					// vehicle status for arming state
#include <uORB/topics/vehicle_control_mode.h>				// vehicle control mode for checking
#include <uORB/topics/vehicle_health.h>					// vehicle health for checking

// Sensor EKF subscriptions
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_odometry.h>		// position, velocity and acceleration

// Thruster & Buoyancy control outputs & arm control
#include <uORB/topics/vehicle_torque_setpoint.h>			// for controlling torque in thruster system
#include <uORB/topics/vehicle_thrust_setpoint.h>			// for controlling thrust in thruster system
#include <uORB/topics/buoyancy_control.h>
#include <uORB/topics/arm_control.h>

#include "missions/mission.hpp"

// ################################################################################
// #	Task Class
// ################################################################################
#include "controllers/position_controller/position_controller.hpp"

class RS_MissionCoordinator : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
	// standard PX4 module boilerplate
public:
	static Descriptor desc;

	/** @brief standard constructor / destructor */
	RS_MissionCoordinator();
	~RS_MissionCoordinator() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @brief not part of ModuleBase, standard convention */
	bool init();

private:

	/** @see ModuleBase
	 *  @brief main running loop
	*/
	void Run() override;

	enum class MissionId : int32_t {
		Off = 0,
		Manual = 1,
		ManualAruco = 2,
		FollowPipe = 3
	};

	enum class CoordinatorState : uint8_t {
		NoMission,
		WaitingForMode,
		Running,
		Safe
	};

	void publish_mission_id();
	void update_mission_selection(const vehicle_status_s &status);
	void request_mission_mode(const vehicle_status_s &status);
	void enter_safe_state(const char *reason);

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::RS_MISSION_ID>) _param_rs_mission_id,
		(ParamFloat<px4::params::RS_X_KP>) _rs_x_kp,
		(ParamFloat<px4::params::RS_X_KI>) _rs_x_ki,
		(ParamFloat<px4::params::RS_X_KD>) _rs_x_kd,
		(ParamFloat<px4::params::RS_Y_KP>) _rs_y_kp,
		(ParamFloat<px4::params::RS_Y_KI>) _rs_y_ki,
		(ParamFloat<px4::params::RS_Y_KD>) _rs_y_kd,
		(ParamFloat<px4::params::RS_Z_KP>) _rs_z_kp,
		(ParamFloat<px4::params::RS_Z_KI>) _rs_z_ki,
		(ParamFloat<px4::params::RS_Z_KD>) _rs_z_kd
		// TODO: Extend this list with roll, pitch, yaw parameters for pid

	)

	// ################################################################################
	// #	uORB Subscriptions & Publications
	// ################################################################################

	// maybe add these later as needed
	// uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
	// uORB::Subscription _control_allocator_status_sub{ORB_ID(control_allocator_status)};
	// uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};

	// Custom parameters
	uORB::SubscriptionCallbackWorkItem _parameter_update_sub{this, ORB_ID(parameter_update)};

	// Setpoints subscriptions
	uORB::SubscriptionCallbackWorkItem _manual_control_setpoint_sub{this, ORB_ID(manual_control_setpoint)};		// should be converted to normal later on

	// Vehicle status subscriptions
	uORB::SubscriptionCallbackWorkItem _vehicle_status_sub{this, ORB_ID(vehicle_status)};
	uORB::SubscriptionCallbackWorkItem _vehicle_health_sub{this, ORB_ID(vehicle_health)};

	// Legacy members retained temporarily; verify their purpose before removing.
	uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _jetson_control_sub{ORB_ID(jetson_control)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	vehicle_control_mode_s _vehicle_control_mode{};
	vehicle_status_s _vehicle_status{};
	vehicle_health_s _vehicle_health{};
	jetson_control_s _jetson_control{};

	// Sensor EKF subscriptions used by the current control paths.
	// TODO: Change this subscription: This is now using vehicle_visal_odometry (but that is the dvl data not the ekf2)
	uORB::Subscription _vehicle_odo_sub{ORB_ID(vehicle_visual_odometry)};  // Coumd maybe be converted to Callback later on

	// Thruster & Buoyancy control outputs publications
	uORB::Publication<vehicle_torque_setpoint_s>	_vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};
	uORB::Publication<vehicle_thrust_setpoint_s>	_vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Publication<buoyancy_control_s>		_buoyancy_control_pub{ORB_ID(buoyancy_control)};
	uORB::Publication<arm_control_s>			_arm_control_pub{ORB_ID(arm_control)};
	uORB::Publication<rs_mission_id_s> _rs_mission_id_pub{ORB_ID(rs_mission_id)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};
	int32_t _last_published_mission_id{-1};
	int32_t _active_mission_id{-1};
	hrt_abstime _last_mode_request{0};
	hrt_abstime _last_mission_tick{0};
	Mission *_active_mission{nullptr};
	CoordinatorState _coordinator_state{CoordinatorState::NoMission};

	perf_counter_t	_loop_perf;			/**< loop duration performance counter */

	PositionController _position_controller;

};
