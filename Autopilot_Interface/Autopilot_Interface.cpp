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
* @file autopilot_interface.cpp
*
* @brief Autopilot interface functions
*
* Functions for sending and recieving commands to an autopilot via MAVlink
*
* @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
* @author Jaycee Lock,    <jaycee.lock@gmail.com>
* @author Lorenz Meier,   <lm@inf.ethz.ch>
*
* Modified by:
*	Spencer Watza, <s.g.watza@gmail.com>
*	James Jenkins, <james.p.jenkins@wmich.edu>
* For Windows Use
*/


#include <stdio.h>
#include <stdint.h>
#include <strsafe.h>
#include <Windows.h>

#include "stdafx.h"
#include "BaseTsd.h"

#include "common/mavlink.h"

#include "Autopilot_Interface.h"
#include "serial_port.h"
#include "utils.h"
#include "mavlink_utils.h"


AutopilotInterface::AutopilotInterface(SerialPort *serial_port_)
{
	//initialize attributes
	write_count = 0;

	reading_status = 0;    // whether the read thread is running
	writing_status = 0;    // whether the write thread is running
	offboard_mode = 0;     // whether the autopilot is in offboard control mode
	time_to_exit = false;  // flag to signal thread exit

	system_id = 0;    // system id
	autopilot_id = 0; // autopilot component id
	companion_id = 0; // companion computer component id

	current_messages.sysid = system_id;
	current_messages.compid = autopilot_id;

	serial_port = serial_port_; // serial port management object
}

AutopilotInterface::~AutopilotInterface()
{}


void AutopilotInterface::update_setpoint(mavlink_set_position_target_local_ned_t setpoint)
{
	current_setpoint = setpoint;
}


void AutopilotInterface::read_messages()
{
	int success;
	bool received_all = false;  // receive only one message
	Time_Stamps this_timestamps;

	// Blocking wait for new data
	while (!received_all && !time_to_exit) {
		// Read message
		mavlink_message_t message;
		success = serial_port->read_message(message);

		if (success) {
			// Store message sysid and compid.
			// Note this doesn't handle multiple message sources.
			current_messages.sysid = message.sysid;
			current_messages.compid = message.compid;

			switch (message.msgid) {
				case MAVLINK_MSG_ID_HEARTBEAT:
				{
					fprintf(stderr, "MAVLINK_MSG_ID_HEARTBEAT\n");
					mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
					fprintf(stderr, "In mode: %d\n", current_messages.heartbeat.base_mode);
					current_messages.time_stamps.heartbeat = get_time_usec();
					this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;
					break;
				}

				case MAVLINK_MSG_ID_SYS_STATUS:
				{
					//printf("MAVLINK_MSG_ID_SYS_STATUS\n");
					mavlink_msg_sys_status_decode(&message, &(current_messages.sys_status));
					current_messages.time_stamps.sys_status = get_time_usec();
					this_timestamps.sys_status = current_messages.time_stamps.sys_status;
					break;
				}

				case MAVLINK_MSG_ID_BATTERY_STATUS:
				{
					//printf("MAVLINK_MSG_ID_BATTERY_STATUS\n");
					mavlink_msg_battery_status_decode(&message, &(current_messages.battery_status));
					current_messages.time_stamps.battery_status = get_time_usec();
					this_timestamps.battery_status = current_messages.time_stamps.battery_status;
					break;
				}

				case MAVLINK_MSG_ID_RADIO_STATUS:
				{
					//printf("MAVLINK_MSG_ID_RADIO_STATUS\n");
					mavlink_msg_radio_status_decode(&message, &(current_messages.radio_status));
					current_messages.time_stamps.radio_status = get_time_usec();
					this_timestamps.radio_status = current_messages.time_stamps.radio_status;
					break;
				}

				case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
				{
					//printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
					mavlink_msg_local_position_ned_decode(&message, &(current_messages.local_position_ned));
					current_messages.time_stamps.local_position_ned = get_time_usec();
					this_timestamps.local_position_ned = current_messages.time_stamps.local_position_ned;
					break;
				}

				case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
				{
					//printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
					mavlink_msg_global_position_int_decode(&message, &(current_messages.global_position_int));
					current_messages.time_stamps.global_position_int = get_time_usec();
					this_timestamps.global_position_int = current_messages.time_stamps.global_position_int;
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
				{
					//printf("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
					mavlink_msg_position_target_local_ned_decode(&message, &(current_messages.position_target_local_ned));
					current_messages.time_stamps.position_target_local_ned = get_time_usec();
					this_timestamps.position_target_local_ned = current_messages.time_stamps.position_target_local_ned;
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
				{
					//printf("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
					mavlink_msg_position_target_global_int_decode(&message, &(current_messages.position_target_global_int));
					current_messages.time_stamps.position_target_global_int = get_time_usec();
					this_timestamps.position_target_global_int = current_messages.time_stamps.position_target_global_int;
					break;
				}

				case MAVLINK_MSG_ID_HIGHRES_IMU:
				{
					//printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
					mavlink_msg_highres_imu_decode(&message, &(current_messages.highres_imu));
					current_messages.time_stamps.highres_imu = get_time_usec();
					this_timestamps.highres_imu = current_messages.time_stamps.highres_imu;
					break;
				}

				case MAVLINK_MSG_ID_ATTITUDE:
				{
					//printf("MAVLINK_MSG_ID_ATTITUDE\n");
					mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
					current_messages.time_stamps.attitude = get_time_usec();
					this_timestamps.attitude = current_messages.time_stamps.attitude;
					break;
				}

				case MAVLINK_MSG_ID_HIL_STATE:
				{
					printf("Got a HIL");
				}
				default:
				{
					// printf("Warning, did not handle message id %i\n",message.msgid);
					break;
				}
			} // end: switch msgid

		} // end: if read message

		  // Check for receipt of all items
		received_all =
			this_timestamps.heartbeat                  &&
			//				this_timestamps.battery_status             &&
			//				this_timestamps.radio_status               &&
			//				this_timestamps.local_position_ned         &&
			//				this_timestamps.global_position_int        &&
			//				this_timestamps.position_target_local_ned  &&
			//				this_timestamps.position_target_global_int &&
							this_timestamps.highres_imu                &&
			//				this_timestamps.attitude                   &&
			this_timestamps.sys_status
			;

		// give the write thread time to use the port
		if (writing_status > 0) {
			usleep(100); // look for components of batches at 10kHz
		}

	}

	return;
}


int
AutopilotInterface::write_message(mavlink_message_t message)
{
	int len = serial_port->write_message(message);

	// book keeping
	write_count++;

	return len;
}


void
AutopilotInterface::write_setpoint()
{
	static int i;
	fprintf(stderr, "Writing setpoint %d\n", i);
	i++;
	// pull from position target
	mavlink_set_position_target_local_ned_t sp = current_setpoint;

	// double check some system parameters
	if (!sp.time_boot_ms)
		sp.time_boot_ms = (uint32_t)(get_time_usec() / 1000);

	sp.target_system = system_id;
	sp.target_component = autopilot_id;

	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------

	mavlink_message_t message;
	mavlink_msg_set_position_target_local_ned_encode(system_id, companion_id, &message, &sp);
	// do the write
	int len = write_message(message);

	// check the write
	if (len <= 0)
		fprintf(stderr, "WARNING: could not send POSITION_TARGET_LOCAL_NED \n");
	//	else
	//		printf("%lu POSITION_TARGET  = [ %f , %f , %f ] \n", write_count, position_target.x, position_target.y, position_target.z);

}


void
AutopilotInterface::enable_offboard_control()
{
	// Should only send this command once
	if (offboard_mode == false) {
		printf("ENABLE OFFBOARD MODE\n");

		int success = toggle_offboard_control(true);

		//Check the next status message from Pixhawk; through the read thread

		//THis only checks if the string was written; not if px4 rejected command
		if (success) {
			offboard_mode = true;
		}
		else {
			fprintf(stderr, "Error: off-board mode not set, could not write message\n");
			//throw EXIT_FAILURE;
		}
	}
}


void
AutopilotInterface::disable_offboard_control()
{
	// Should only send this command once
	if (offboard_mode == true) {
		printf("DISABLE OFFBOARD MODE\n");

		int success = toggle_offboard_control(false);

		//Check the next status message from Pixhawk; through the read thread

		// Check the command was written
		if (success) {
			offboard_mode = false;
		}
		else {
			fprintf(stderr, "Error: off-board mode not set, could not write message\n");
			//throw EXIT_FAILURE;
		}
	}
}


int AutopilotInterface::toggle_offboard_control(bool flag)
{
	// Prepare command for off-board mode
	mavlink_command_long_t com = {0};
	com.target_system = system_id;
	com.target_component = autopilot_id;
	com.command = MAV_CMD_NAV_GUIDED_ENABLE;
	com.confirmation = true;
	com.param1 = (float)flag; // flag >0.5 => start, <0.5 => stop

							  // Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

	// Send the message
	int len = serial_port->write_message(message);

	// Done!
	return len;
}


void AutopilotInterface::start()
{
	if (serial_port->isOpen() != true) {
		fprintf(stderr, "ERROR: serial port not open\n");
		throw 1;
	}

	// Start a read thread
	printf("START READ THREAD \n");
	//int result = pthread_create(&read_tid, NULL, &start_autopilot_interface_read_thread, this);
	hread_thread = CreateThread(NULL, 0, start_autopilot_interface_read_thread, this, 0, &hread_tid);
	if (hread_thread == NULL) {
		ErrorHandler(TEXT("CreateThread"));
		ExitProcess(3);
	}
	printf("\n");

	// Verify messages are coming in
	printf("CHECK FOR MESSAGES\n");

	while (!current_messages.sysid) {
		if (time_to_exit)
			return;
		usleep(500000); // check at 2Hz
	}

	// now we know autopilot is sending messages
	printf("Found\n\n");

	// This comes from the heartbeat. If there is more than one
	// vehicle then this won't work. In which case set the id's manually.

	// System ID
	if (!system_id) {
		system_id = current_messages.sysid;
		printf("GOT VEHICLE SYSTEM ID: %i\n", system_id);
	}

	// Component ID
	if (!autopilot_id) {
		autopilot_id = current_messages.compid;
		printf("GOT AUTOPILOT COMPONENT ID: %i\n", autopilot_id);
		printf("\n");
	}


	// Wait for initial position NED
	while (!(current_messages.time_stamps.local_position_ned &&
			 current_messages.time_stamps.attitude)) {
		if (time_to_exit)
			return;
		usleep(500000);
	}

	// copy initial position ned
	Mavlink_Messages local_data = current_messages;
	initial_position.x = local_data.local_position_ned.x;
	initial_position.y = local_data.local_position_ned.y;
	initial_position.z = local_data.local_position_ned.z;
	initial_position.vx = local_data.local_position_ned.vx;
	initial_position.vy = local_data.local_position_ned.vy;
	initial_position.vz = local_data.local_position_ned.vz;
	initial_position.yaw = local_data.attitude.yaw;
	initial_position.yaw_rate = local_data.attitude.yawspeed;

	printf("INITIAL POSITION XYZ = [ %.4f , %.4f , %.4f ] \n", initial_position.x, initial_position.y, initial_position.z);
	printf("INITIAL POSITION YAW = %.4f \n", initial_position.yaw);
	printf("\n");

	// Start a write thread
	printf("START WRITE THREAD \n");
	//result = pthread_create(&write_tid, NULL, &start_autopilot_interface_write_thread, this);
	hread_thread = CreateThread(NULL, 0, start_autopilot_interface_write_thread, this, 0, &hwrite_tid);
	if (hread_thread == NULL) {
		ErrorHandler(TEXT("CreateThread"));
		ExitProcess(3);
	}

	// wait for it to be started
	while (writing_status != 0)
		usleep(100000); // 10Hz

	// now we're streaming setpoint commands
}


void AutopilotInterface::stop()
{
	printf("CLOSE THREADS\n");

	// signal exit
	time_to_exit = true;

	// Wait to close read and write threads
	//pthread_join(read_tid, NULL);
	//pthread_join(write_tid, NULL);
	CloseHandle(hread_thread);
	CloseHandle(hwrite_thread);
	
	printf("Autopilot stopped.\n");
}

void AutopilotInterface::handle_quit(int sig)
{
	disable_offboard_control();

	try {
		stop();
	} catch (int error) {
		fprintf(stderr, "Warning, could not stop autopilot interface - errno: %d\n", error);
	}
}


// Read thread
void AutopilotInterface::start_read_thread()
{
	if (reading_status)
		fprintf(stderr, "read thread already running\n");
	else
		read_thread();
}

void AutopilotInterface::read_thread()
{
	reading_status = true;

	while (!time_to_exit) {
		read_messages();
		// Read batches at 10Hz
		usleep(100000);
	}

	reading_status = false;
}


// Write thread
void AutopilotInterface::start_write_thread(void)
{
	if (writing_status)
		fprintf(stderr, "write thread already running\n");
	else
		write_thread();
}

void AutopilotInterface::write_thread(void)
{
	printf("We're in the writing thread\n");
	writing_status = true;

	// prepare an initial setpoint, just stay put
	mavlink_set_position_target_local_ned_t sp;
	sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY &
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE &MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
	sp.vx = 0.0;
	sp.vy = 0.0;
	sp.vz = 0.0;
	sp.yaw_rate = 0.0;
	sp.yaw = 0.0;

	// set position target
	current_setpoint = sp;

	// write a message and signal writing
	write_setpoint();

	// Pixhawk needs to see off-board commands at minimum 2Hz,
	// otherwise it will go into fail safe
	while (!time_to_exit) {
		usleep(250000);   // Stream at 4Hz
		write_setpoint();
	}

	writing_status = false;
}


/*
 * End Autopilot_Interface
 */


DWORD WINAPI start_autopilot_interface_read_thread(void *args)
{
	AutopilotInterface *autopilot_interface = (AutopilotInterface *)args;
	autopilot_interface->start_read_thread();
	return NULL;
}

DWORD WINAPI start_autopilot_interface_write_thread(void *args)
{
	AutopilotInterface *autopilot_interface = (AutopilotInterface *)args;
	autopilot_interface->start_write_thread();
	return NULL;
}

void ErrorHandler(LPTSTR lpszFunction)
{
	// Retrieve the system error message for the last-error code.

	LPVOID lpMsgBuf;
	LPVOID lpDisplayBuf;
	DWORD dw = GetLastError();

	FormatMessage(
		FORMAT_MESSAGE_ALLOCATE_BUFFER |
		FORMAT_MESSAGE_FROM_SYSTEM |
		FORMAT_MESSAGE_IGNORE_INSERTS,
		NULL,
		dw,
		MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
		(LPTSTR)&lpMsgBuf,
		0, NULL);

	// Display the error message.

	lpDisplayBuf = (LPVOID)LocalAlloc(LMEM_ZEROINIT,
		(lstrlen((LPCTSTR)lpMsgBuf) + lstrlen((LPCTSTR)lpszFunction) + 40) * sizeof(TCHAR));
	StringCchPrintf((LPTSTR)lpDisplayBuf,
					LocalSize(lpDisplayBuf) / sizeof(TCHAR),
					TEXT("%s failed with error %d: %s"),
					lpszFunction, dw, lpMsgBuf);
	MessageBox(NULL, (LPCTSTR)lpDisplayBuf, TEXT("Error"), MB_OK);

	// Free error-handling buffer allocations.

	LocalFree(lpMsgBuf);
	LocalFree(lpDisplayBuf);
}