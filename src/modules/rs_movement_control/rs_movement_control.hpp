
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
#include <lib/systemlib/mavlink_log.h>
#include <mathlib/math/Functions.hpp>					// MAVLink logging

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
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/jetson_control.h>			// manual control inputs

// Vehicle status
#include <uORB/topics/vehicle_status.h>					// vehicle status for arming state
#include <uORB/topics/vehicle_control_mode.h>				// vehicle control mode for checking
#include <uORB/topics/vehicle_health.h>					// vehicle health for checking

// Sensor EKF subscriptions
#include <uORB/topics/vehicle_angular_velocity.h>			// attitude rates
#include <uORB/topics/vehicle_attitude.h>				// attitude
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_odometry.h>		// position, velocity and acceleration

// Thruster & Buoyancy control outputs & arm control
#include <uORB/topics/vehicle_torque_setpoint.h>			// for controlling torque in thruster system
#include <uORB/topics/vehicle_thrust_setpoint.h>			// for controlling thrust in thruster system
#include <uORB/topics/buoyancy_control.h>
#include <uORB/topics/arm_control.h>

// Topics to test communication
#include <uORB/topics/my_custom_topic.h>


using namespace time_literals;


// ################################################################################
// #	Task Class
// ################################################################################
class RS_PID {
public:
	RS_PID(float limit = 1.0f) : _limit(limit) {}

	void update(float kp, float ki, float kd, float setpoint, float current_value, float dt){
	if (dt <= 0.0f) return;

	float error = setpoint - current_value;

	float p_out = kp * error;

	_integral += error * dt;
	_integral = math::constrain(_integral, -0.5f, 0.5f);
	float i_out = ki * _integral;

	float derivative = (error - _last_error) / dt;
	float d_out = kd * derivative;

	float total = p_out + i_out + d_out;
	_last_output = math::constrain(total, -_limit, _limit);

	_last_error = error;
	}

	float get_output() const { return _last_output; }

	void reset(){
		_integral = 0.0f;
		_last_error = 0.0f;
		_last_output = 0.0f;
	}

private:
	float _limit;
	float _integral{0.0f};
	float _last_error{0.0f};
	float _last_output{0.0f};


};



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

	void control_manual(const manual_control_setpoint_s &manual, vehicle_thrust_setpoint_s &thrust, vehicle_torque_setpoint_s &torque, buoyancy_control_s &buoyancy, arm_control_s &arm, hrt_abstime now);
	void control_hold(const vehicle_odometry_s &odom, vehicle_thrust_setpoint_s &thrust, vehicle_torque_setpoint_s &torque, buoyancy_control_s &buoyancy, hrt_abstime now);
	void control_jetson(const vehicle_local_position_s &lp, vehicle_thrust_setpoint_s &thrust, vehicle_torque_setpoint_s &torque, hrt_abstime now);


	void parameters_update();			//functie om custom parameters te maken

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RS_X_KP>) _rs_x_kp,
		(ParamFloat<px4::params::RS_X_KI>) _rs_x_ki,
		(ParamFloat<px4::params::RS_X_KD>) _rs_x_kd,
		(ParamFloat<px4::params::RS_Y_KP>) _rs_y_kp,
		(ParamFloat<px4::params::RS_Y_KI>) _rs_y_ki,
		(ParamFloat<px4::params::RS_Y_KD>) _rs_y_kd,
		(ParamFloat<px4::params::RS_Z_KP>) _rs_z_kp,
		(ParamFloat<px4::params::RS_Z_KI>) _rs_z_ki,
		(ParamFloat<px4::params::RS_Z_KD>) _rs_z_kd,
		(ParamFloat<px4::params::RS_MAN_MODE>) _rs_man_mode

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
	uORB::Subscription _jetson_control_sub{ORB_ID(jetson_control)};			// to monitor health status of the vehicle
	uORB::SubscriptionCallbackWorkItem _vehicle_health_sub{this, ORB_ID(vehicle_health)};

	// Sensor EKF subscriptions
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_odo_sub{ORB_ID(vehicle_visual_odometry)};		// should be converted to Callback later on
	// uORB::SubscriptionCallbackWorkItem _vehicle_local_position_sub{this, ORB_ID(vehicle_local_position)};
	// Run the module when new position data is available

	// Thruster & Buoyancy control outputs publications
	uORB::Publication<vehicle_torque_setpoint_s>	_vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};
	uORB::Publication<vehicle_thrust_setpoint_s>	_vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Publication<buoyancy_control_s>		_buoyancy_control_pub{ORB_ID(buoyancy_control)};
	uORB::Publication<arm_control_s>			_arm_control_pub{ORB_ID(arm_control)};

	// Topics to test publications
	uORB::Publication<my_custom_topic_s>		_my_custom_topics_pub{ORB_ID(my_custom_topic)};

	// Global topic objects
	vehicle_control_mode_s	_vehicle_control_mode{};
	vehicle_status_s	_vehicle_status{};
	vehicle_health_s	_vehicle_health{};
	jetson_control_s	_jetson_control{};


	// ################################################################################
	// #	Module Global Variables
	// ################################################################################

	hrt_abstime _last_run{0};			/**< last run time for pid dt calculation*/
	perf_counter_t	_loop_perf;			/**< loop duration performance counter */

	RS_PID _pid_x{1.0f}; // Limit op 1.0 thrust
	RS_PID _pid_y{1.0f};
	RS_PID _pid_z{1.0f};

	bool _hold_position_set{false};
	float _hold_x{0.0f};
	float _hold_y{0.0f};
	float _hold_z{0.0f};


	// ################################################################################
	// #	Module Parameters
	// ################################################################################

	// define module parameters for configuring module in-flight
	// DEFINE_PARAMETERS(
	// 	(ParamBool<px4::params::RS_MOVE_CTRL_EN>) _param_rs_move_ctrl_en
	// )
};
