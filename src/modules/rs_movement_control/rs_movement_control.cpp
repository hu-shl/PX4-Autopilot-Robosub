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

#include "rs_movement_control.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/events.h>
#include <systemlib/mavlink_log.h>

using namespace matrix;
using namespace time_literals;
using math::radians;



/**
 * @brief Class constructor
 */
RS_MovementControl::RS_MovementControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	// load parameters when the task begins
	// updateParams();
	// loadParams();
}



/**
 * @brief Class destructor
 */
RS_MovementControl::~RS_MovementControl()
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
int RS_MovementControl::task_spawn(int argc, char *argv[])
{
	// Create new instance of the class
	RS_MovementControl *instance = new RS_MovementControl();

	// Return Error if allocation in memory of instance failed
	if (!instance) {			// check if the instance exists
		PX4_ERR("alloc failed");	// log the error
		return PX4_ERROR;		// return as an error
	}

	// Return Error and delete instance if init is not successful
	if (!instance->init()) {		// run the initialize function
		delete instance;		// if failed, delete the instance
		return PX4_ERROR;		// return as an error
	}

	// If all is successfull:

	// Store the instance and mark as a work queue
	_object.store(instance);
	_task_id = task_id_is_work_queue;

	// Force the module to wake up immediately to process (potential) parameters.
	// Future runs will be triggered automatically by the uORB subscription callbacks.
	instance->ScheduleNow();

	// return ok
	return PX4_OK;
}



/**
 * @brief init function part of the constructor to be able to have error handling.
 *
 * @return PX4_OK if successfull, otherwise PX4_ERROR
 */
bool RS_MovementControl::init()
{
	// Register the callback
	if (!_manual_control_setpoint_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	if(!_vehicle_health_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	// advertise to be published topics
	_vehicle_torque_setpoint_pub.advertise();
	_vehicle_thrust_setpoint_pub.advertise();
	_buoyancy_control_pub.advertise();
	_arm_control_pub.advertise();

	return true;
}



/**
 * @brief helper function to update variables from parameters
 *
 * @return void
 */
void RS_MovementControl::loadParams()
{
	// Add parameters here to update in the future.
	// like: pid.load(kp.get(), ki.get(), kd.get());
}



/**
 * @brief Main Module loop
 */
void RS_MovementControl::Run()
{

	// Check if this module should still exist
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	// begin performance monitoring
	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
	// 	// clear update
	 	parameter_update_s param_update;
	 	_parameter_update_sub.copy(&param_update);

	 	 updateParams();
	// 	// loadParams();
	 }

	// create objects to copy data to and from
	manual_control_setpoint_s manualControlInput;
	vehicle_local_position_s vehicle_local_position;
	vehicle_odometry_s vehicle_odo_data;
	vehicle_thrust_setpoint_s thrustSetpoint{};
	vehicle_torque_setpoint_s torqueSetpoint{};
	buoyancy_control_s	  buoyancyControl{};
	arm_control_s		  armControl{};
	vehicle_status_s	  status;			// to monitor flightmode
	vehicle_health_s	  vehicleHealth{};
	jetson_control_s	  jetsonControl{};		// to monitor health status of the vehicle

	// only run if there is new data in the manual control setpoint topic
	_manual_control_setpoint_sub.copy(&manualControlInput);
	_vehicle_status_sub.copy(&status);
	_vehicle_local_position_sub.copy(&vehicle_local_position);
	_vehicle_odo_sub.copy(&vehicle_odo_data);

	_vehicle_health_sub.copy(&vehicleHealth);
	_jetson_control_sub.copy(&jetsonControl);

	orb_advert_t mavlink_log_pub = nullptr;
	if (vehicleHealth.battery[0] > 50.0f) {
		mavlink_log_info(&mavlink_log_pub, "temp battery high!: %f", static_cast<double>(vehicleHealth.battery[0]));
	}
		// get current time
		hrt_abstime now = hrt_absolute_time();


	switch (status.nav_state){						// switchcase with logic based on flight mode

		case vehicle_status_s::NAVIGATION_STATE_MANUAL:
		_hold_position_set = false;			//controller inputs linked to thrusters en buoyancy
		control_manual(manualControlInput, thrustSetpoint, torqueSetpoint, buoyancyControl, armControl, now);
		break;


	case vehicle_status_s::NAVIGATION_STATE_POSCTL:			// Thruster control in hold mode

		control_hold(vehicle_odo_data, thrustSetpoint, torqueSetpoint, buoyancyControl, now);
		//pid in for x,y,z, roll, pitch, yaw thrusters
		//evt later langzame pid voor buoyancy voor Z,roll,pitch
		break;


	case vehicle_status_s::NAVIGATION_STATE_ACRO:			//Thruster control by ROS2
		// volledig naar locatie-setpoint luisteren van ROS2.
		// In dit stuk code zit dus ook code verwerkt.
	break;
}
		// publish the new data
		_vehicle_torque_setpoint_pub.publish(torqueSetpoint);
        	_vehicle_thrust_setpoint_pub.publish(thrustSetpoint);
		_buoyancy_control_pub.publish(buoyancyControl);
		_arm_control_pub.publish(armControl);


	perf_end(_loop_perf);
}



/**
 * @brief function run when a custom flag or command other than the standard ones defined in module.h is passed
 *
 * @param argc Number of arguments
 * @param argv Argument Array
 *
 * @return PX4_OK if successfull, otherwise PX4_ERROR
 */
int RS_MovementControl::custom_command(int argc, char *argv[])
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
int RS_MovementControl::print_usage(const char *reason)
{
	return print_usage("No commands defined");
}



/**
 * @brief C Export for compaitibility with NuttX shell
 */
extern "C" __EXPORT int rs_movement_control_main(int argc, char *argv[])
{
	return RS_MovementControl::main(argc, argv);
}


void RS_MovementControl::control_manual(const manual_control_setpoint_s &manual, vehicle_thrust_setpoint_s &thrust,
	 vehicle_torque_setpoint_s &torque, buoyancy_control_s &buoyancy, arm_control_s &arm, hrt_abstime now)
{

// set timestamp variables
		torque.timestamp = now;
       		torque.timestamp_sample = now;
        	thrust.timestamp = now;
        	thrust.timestamp_sample = now;

		float manual_mode_float = manual.aux1;
		// if (manual_mode_float < 0) manual_mode_float -= 0.5f;		// round to nearest int
		// else manual_mode_float += 0.5f;
		// int manual_mode = (int)(manual_mode_float);

		//float manual_mode_float = _rs_man_mode.get();
		int manual_mode = (int)lroundf(manual_mode_float);
		PX4_INFO("manual mode: %d", manual_mode);

		thrust.xyz[0] = 0.f; thrust.xyz[1] = 0.f; thrust.xyz[2] = 0.f;
   		torque.xyz[0] = 0.f; torque.xyz[1] = 0.f; torque.xyz[2] = 0.f;
    		for(int i=0; i<4; i++) buoyancy.tank_command[i] = 0.f;
		for(int j=0; j<6; j++) arm.servo_command[j] = 0.f;
		// load data from manual control to output
		// NED frame

		switch(manual_mode){

			case -1:
			buoyancy.tank_command[0] = manual.roll;
			buoyancy.tank_command[1] = manual.pitch;
			buoyancy.tank_command[2] = manual.yaw;
			buoyancy.tank_command[3] = manual.throttle;
			buoyancy.tank_command[4] = manual.aux3;
			buoyancy.tank_command[5] = manual.aux4;
			buoyancy.tank_command[6] = manual.aux5;
			buoyancy.tank_command[7] = manual.aux6;
			PX4_INFO("buoyancy mode: %d", manual_mode);
			break;

			case 0:
			torque.xyz[0] = manual.aux4;					//Controller allocation based on rc controller setup
			torque.xyz[1] = manual.pitch;
			torque.xyz[2] = manual.roll * 0.2f;

			thrust.xyz[0] = manual.throttle;
			thrust.xyz[1] = 0.f;
			thrust.xyz[2] = manual.yaw * 0.6f;
			PX4_INFO("thrusters mode: %d", manual_mode);
			break;

			case 1:
			arm.servo_command[0] = manual.roll;
			arm.servo_command[1] = manual.pitch;
			arm.servo_command[2] = manual.yaw;
			arm.servo_command[3] = manual.throttle;
			arm.servo_command[4] = manual.aux3;
			arm.servo_command[5] = manual.aux4;
			arm.servo_command[6] = manual.aux5;
			arm.servo_command[7] = manual.aux6;
			PX4_INFO("arm mode: %d", manual_mode);

		}


}

void RS_MovementControl::control_hold(const vehicle_odometry_s &odom, vehicle_thrust_setpoint_s &thrust,
vehicle_torque_setpoint_s &torque, buoyancy_control_s &buoyancy, hrt_abstime now)
{
    if (!_hold_position_set) {
        _hold_x = odom.position[0];
        _hold_y = odom.position[1];
        _hold_z = odom.position[2];

	_pid_x.reset();
	_pid_y.reset();
	_pid_z.reset();

        _hold_position_set = true;
        _last_run = now;

        PX4_INFO("Custom PID Hold Start");
    }

    float dt = (now - _last_run) / 1e6f;

    if (dt > 0.099f) {

	_pid_x.update(_rs_x_kp.get(), _rs_x_ki.get(), _rs_x_kd.get(), _hold_x, odom.position[0], dt);

	_pid_y.update(_rs_y_kp.get(), _rs_y_ki.get(), _rs_y_kd.get(), _hold_y, odom.position[1], dt);

	_pid_z.update(_rs_z_kp.get(), _rs_z_ki.get(), _rs_z_kd.get(), _hold_z, odom.position[2], dt);

        _last_run = now;

        PX4_INFO("Hold Active - X_Err: %.2f | X_Thrust: %.2f", (double)(_hold_x - odom.position[0]), (double)thrust.xyz[0]);
	PX4_INFO("Hold sp %.2f", (double)odom.position[0]);
    }
    thrust.xyz[0] = -_pid_x.get_output();
    thrust.xyz[1] = _pid_y.get_output();
    thrust.xyz[2] = _pid_z.get_output();
    thrust.timestamp = now;
}
