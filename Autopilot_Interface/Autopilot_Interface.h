/****************************************************************************
*
*   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
*   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
*           Jaycee Lock,    <jaycee.lock@gmail.com>
*           Lorenz Meier,   <lm@inf.ethz.ch>
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

/**
* @file autopilot_interface.h
*
* @brief Autopilot interface definition
*
* Functions for sending and recieving commands to an autopilot via MAVlink
*
* @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
* @author Jaycee Lock,    <jaycee.lock@gmail.com>
* @author Lorenz Meier,   <lm@inf.ethz.ch>
*
* Modified by:
*	Spencer Watza, <s.g.watza@gmail.com>
* For Windows Use
*/

#pragma once
#ifndef AUTOPILOT_INTERFACE_H_
#define AUTOPILOT_INTERFACE_H_


#include "common/mavlink.h"
#include "serial_port.h"


struct Time_Stamps
{
	Time_Stamps()
	{
		reset_timestamps();
	}

	uint64_t heartbeat;
	uint64_t sys_status;
	uint64_t battery_status;
	uint64_t radio_status;
	uint64_t local_position_ned;
	uint64_t global_position_int;
	uint64_t position_target_local_ned;
	uint64_t position_target_global_int;
	uint64_t highres_imu;
	uint64_t attitude;

	void
		reset_timestamps()
	{
		heartbeat = 0;
		sys_status = 0;
		battery_status = 0;
		radio_status = 0;
		local_position_ned = 0;
		global_position_int = 0;
		position_target_local_ned = 0;
		position_target_global_int = 0;
		highres_imu = 0;
		attitude = 0;
	}

};

// Struct containing information on the MAV we are currently connected to

struct Mavlink_Messages {

	int sysid;
	int compid;

	// Heartbeat
	mavlink_heartbeat_t heartbeat;

	// System Status
	mavlink_sys_status_t sys_status;

	// Battery Status
	mavlink_battery_status_t battery_status;

	// Radio Status
	mavlink_radio_status_t radio_status;

	// Local Position
	mavlink_local_position_ned_t local_position_ned;

	// Global Position
	mavlink_global_position_int_t global_position_int;

	// Local Position Target
	mavlink_position_target_local_ned_t position_target_local_ned;

	// Global Position Target
	mavlink_position_target_global_int_t position_target_global_int;

	// HiRes IMU
	mavlink_highres_imu_t highres_imu;

	// Attitude
	mavlink_attitude_t attitude;

	// System Parameters?

	// Time Stamps
	Time_Stamps time_stamps;

	void
		reset_timestamps()
	{
		time_stamps.reset_timestamps();
	}

};

DWORD WINAPI start_autopilot_interface_read_thread(void *args);
DWORD WINAPI start_autopilot_interface_write_thread(void *args);
void ErrorHandler(LPTSTR lpszFunction);

// ********************
//   Autopilot Interface Class
// ********************
/*
* Autopilot Interface Class
*
* This starts two threads for read and write over MAVlink. The read thread
* listens for any MAVlink message and pushes it to the current_messages
* attribute.  The write thread at the moment only streams a position target
* in the local NED frame (mavlink_set_position_target_local_ned_t), which
* is changed by using the method update_setpoint().  Sending these messages
* are only half the requirement to get response from the autopilot, a signal
* to enter "offboard_control" mode is sent by using the enable_offboard_control()
* method.  Signal the exit of this mode with disable_offboard_control().  It's
* important that one way or another this program signals offboard mode exit,
* otherwise the vehicle will go into failsafe.
*/
class AutopilotInterface
{

public:

	AutopilotInterface(SerialPort *serial_port_);
	~AutopilotInterface();

	char reading_status;
	char writing_status;
	bool offboard_mode;
	uint64_t write_count;

	int system_id;
	int autopilot_id;
	int companion_id;

	Mavlink_Messages current_messages;
	mavlink_set_position_target_local_ned_t initial_position;

	void update_setpoint(mavlink_set_position_target_local_ned_t setpoint);
	void read_messages();
	int  write_message(mavlink_message_t message);

	void enable_offboard_control();
	void disable_offboard_control();

	void start();
	void stop();

	void start_read_thread(void);
	void start_write_thread(void);

	void handle_quit(int sig);


private:

	SerialPort *serial_port;

	bool time_to_exit;

	HANDLE hread_thread;
	HANDLE hwrite_thread;

	DWORD hread_tid;
	DWORD hwrite_tid;
	//pthread_t read_tid;
	//pthread_t write_tid;

	mavlink_set_position_target_local_ned_t current_setpoint;

	void read_thread();
	void write_thread(void);

	int toggle_offboard_control(bool flag);
	void write_setpoint();

};
#endif // AUTOPILOT_INTERFACE_H_