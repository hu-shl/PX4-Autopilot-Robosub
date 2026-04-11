
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
#include <px4_platform_common/posix.h>					// standard POSIX functionality
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>	// workQueue functionality for our module




// Helpers and utilities
#include <lib/pid/PID.hpp>						// PID functionality
#include <lib/matrix/matrix/math.hpp>					// matrix library
#include <lib/perf/perf_counter.h>					// performance counters for top command
#include <lib/systemlib/mavlink_log.h>					// MAVLink logging

// uORB pub&sub stuff
#include <uORB/Publication.hpp>						// uORB publication functionality
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>					// uORB subscription functionality
#include <uORB/SubscriptionCallback.hpp>				// interrupt instead of polling


// ################################################################################
// #	uORB Topics
// ################################################################################

// Setpoints
#include <uORB/topics/vehicle_attitude_setpoint.h>			// contains thrust and attitude setpoints
#include <uORB/topics/manual_control_setpoint.h>			// manual control inputs

// Vehicle status
#include <uORB/topics/vehicle_status.h>					// vehicle status for arming state
#include <uORB/topics/vehicle_control_mode.h>				// vehicle control mode for checking

// Sensor EKF subscriptions
#include <uORB/topics/vehicle_angular_velocity.h>			// attitude rates
#include <uORB/topics/vehicle_attitude.h>				// attitude
#include <uORB/topics/vehicle_local_position.h>				// position, velocity and acceleration

// Thruster & Buoyancy control outputs
#include <uORB/topics/vehicle_torque_setpoint.h>			// for controlling torque in thruster system
#include <uORB/topics/vehicle_thrust_setpoint.h>			// for controlling thrust in thruster system
#include <uORB/topics/buoyancy_control.h>

// Topics to test communication
#include <uORB/topics/my_custom_topic.h>


using namespace time_literals;


// ################################################################################
// #	Task Class
// ################################################################################

class RS_MovementControl : public ModuleBase<RS_MovementControl>, public ModuleParams, public px4::WorkItem
{
	// standard PX4 module boilerplate
public:

	/** @brief standard constructor / destructor */
	RS_MovementControl();
	~RS_MovementControl() override;

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

	void loadParams();	// extra function to handle parameter updates

	void control_manual(const manual_control_setpoint_s &manual, vehicle_thrust_setpoint_s &thrust, vehicle_torque_setpoint_s &torque, buoyancy_control_s &buoyancy, hrt_abstime now);
	//void control_hold(const manual_control_setpoint_s &manual, vehicle_thrust_setpoint_s &thrust, vehicle_torque_setpoint_s &torque, hrt_abstime now);
	//void control_offboard();

	void parameters_update();			//functie om custom parameters te maken

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::RS_X_KP>) _rs_x_kp,
		(ParamFloat<px4::params::RS_X_KI>) _rs_x_ki
	)

	// ################################################################################
	// #	uORB Subscriptions & Publications
	// ################################################################################

	// maybe add these later as needed
	// uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
	// uORB::Subscription _control_allocator_status_sub{ORB_ID(control_allocator_status)};
	// uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};

	// Custom parameters
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};


	// Setpoints subscriptions
	uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::SubscriptionCallbackWorkItem _manual_control_setpoint_sub{this, ORB_ID(manual_control_setpoint)};		// should be converted to normal later on

	// Vehicle status subscriptions
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};

	// Sensor EKF subscriptions
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};					// should be converted to Callback later on
	// uORB::SubscriptionCallbackWorkItem _vehicle_local_position_sub{this, ORB_ID(vehicle_local_position)};
	// Run the module when new position data is available

	// Thruster & Buoyancy control outputs publications
	uORB::Publication<vehicle_torque_setpoint_s>	_vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};
	uORB::Publication<vehicle_thrust_setpoint_s>	_vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Publication<buoyancy_control_s>		_buoyancy_control_pub{ORB_ID(buoyancy_control)};

	// Topics to test publications
	uORB::Publication<my_custom_topic_s>		_my_custom_topics_pub{ORB_ID(my_custom_topic)};

	// Global topic objects
	vehicle_control_mode_s	_vehicle_control_mode{};
	vehicle_status_s	_vehicle_status{};



	// ################################################################################
	// #	Module Global Variables
	// ################################################################################

	hrt_abstime _last_run{0};			/**< last run time for pid dt calculation*/
	perf_counter_t	_loop_perf;			/**< loop duration performance counter */



	// ################################################################################
	// #	Module Parameters
	// ################################################################################

	// define module parameters for configuring module in-flight
	// DEFINE_PARAMETERS(
	// 	(ParamBool<px4::params::RS_MOVE_CTRL_EN>) _param_rs_move_ctrl_en
	// )
};
