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

	// advertise to be published topics
	_vehicle_torque_setpoint_pub.advertise();
	_vehicle_thrust_setpoint_pub.advertise();
	_buoyancy_control_pub.advertise();

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
	// if (_parameter_update_sub.updated()) {
	// 	// clear update
	// 	parameter_update_s param_update;
	// 	_parameter_update_sub.copy(&param_update);

	// 	// updateParams();
	// 	// loadParams();
	// }

	// create objects to copy data to and from
	manual_control_setpoint_s manualControlInput;
	vehicle_thrust_setpoint_s thrustSetpoint;
	vehicle_torque_setpoint_s torqueSetpoint;
	buoyancy_control_s	  buoyancyControl;

	// only run if there is new data in the manual control setpoint topic
	_manual_control_setpoint_sub.copy(&manualControlInput);

		// get current time
		hrt_abstime now = hrt_absolute_time();

		// set timestamp variables
		torqueSetpoint.timestamp = now;
       		torqueSetpoint.timestamp_sample = now;
        	thrustSetpoint.timestamp = now;
        	thrustSetpoint.timestamp_sample = now;
		buoyancyControl.timestamp = now;

		// load data from manual control to output
		// NED frame
		torqueSetpoint.xyz[0] = 0.f;
		torqueSetpoint.xyz[1] = manualControlInput.throttle;
		torqueSetpoint.xyz[2] = manualControlInput.roll;

		thrustSetpoint.xyz[0] = manualControlInput.pitch;
		thrustSetpoint.xyz[1] = 0.f;
		thrustSetpoint.xyz[2] = manualControlInput.yaw;

		//thrustSetpoint.xyz[0] = manualControlInput.pitch;		//thrustsetpoint code Wesse
		//thrustSetpoint.xyz[1] = manualControlInput.roll;
		//thrustSetpoint.xyz[2] = manualControlInput.throttle;



		buoyancyControl.tank_command[0] = manualControlInput.roll;
		buoyancyControl.tank_command[1] = manualControlInput.pitch;
		buoyancyControl.tank_command[2] = manualControlInput.yaw;
		buoyancyControl.tank_command[3] = manualControlInput.throttle;


		// publish the new data
		_vehicle_torque_setpoint_pub.publish(torqueSetpoint);
        	_vehicle_thrust_setpoint_pub.publish(thrustSetpoint);
		_buoyancy_control_pub.publish(buoyancyControl);


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
