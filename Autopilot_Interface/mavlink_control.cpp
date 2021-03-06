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
* @file mavlink_control.cpp
*
* @brief An example offboard control process via mavlink
*
* This process connects an external MAVLink UART device to send an receive data
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

#include "stdafx.h"

#include "mavlink_control.h"
#include "utils.h"
#include "mavlink_utils.h"


int
top(int argc, char **argv)
{
	// Defaults
	int baudrate = 57600;
	int portNum = 4;

	parse_commandline(argc, argv, portNum, baudrate);

	/*
	* This object handles the opening and closing of the offboard computer's
	* serial port over which it will communicate to an autopilot.  It has
	* methods to read and write a mavlink_message_t object.  To help with read
	* and write in the context of pthreading, it gaurds port operations with a
	* pthread mutex lock.
	*/
	SerialPort serial_port(portNum, baudrate);
	//serial_port.debug = TRUE;
	std::cout << "Finished creating serial port" << std::endl;

	/*
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
	AutopilotInterface autopilot_interface(&serial_port);
	std::cout << "Finished creating autopilot interface" << std::endl;

	// Setup interrupt signal handler
	serial_port_quit = &serial_port;
	autopilot_interface_quit = &autopilot_interface;
	signal(SIGINT, quit_handler);

	// Start autopilot interface
	std::cout << "Starting autopilot interface" << std::endl;
	autopilot_interface.start();

	// Now we can implement the algorithm we want on top of the autopilot interface
	std::cout << "Running commands" << std::endl;
	commands(autopilot_interface);

	// Wrap up
	autopilot_interface.stop();
	serial_port.close();

	return 0;
}


void
commands(AutopilotInterface &api)
{
	usleep(26000000);
	printf("Enabling offboard control\n");

	// Start offboard mode so pixhawk doesn't go to failsafe
	api.enable_offboard_control();
	// give some time to let it sink in
	usleep(100);
	// autopilot is now accepting setpoint commands

	printf("SEND OFFBOARD COMMANDS\n");

	mavlink_message_t message;
	printf("api.system_id = %d; api.companion_id = %d", api.system_id, api.companion_id);
	mavlink_rc_channels_override_t rcco;
	rcco.chan1_raw = 0;
	rcco.chan4_raw = 1015;
	mavlink_msg_rc_channels_override_encode(api.system_id, api.companion_id, &message, &rcco);
	api.write_message(message);
	Sleep(500);
	
	/*
	mavlink_servo_output_raw_t servo_output;
	servo_output.servo3_raw = 800;
	mavlink_msg_servo_output_raw_encode(api.system_id, api.companion_id, &servo_output, &message);
	for(int i=0; i < 1000; i++){
		int len = api.write_message(message);
		if (len <= 0) {
			fprintf(stderr, "COULD NOT SEND MANUAL SERVO OUT");
		}
		Sleep(2);
	}
	*/
	
	mavlink_set_position_target_local_ned_t sp;
	mavlink_set_position_target_local_ned_t ip = api.initial_position;

	// autopilot_interface.h provides some helper functions to build the command
	// e.g. set_velocity(), set_position(), set_yaw()

	// rad/sec
	//set_yaw(ip.yaw, sp);
	set_position(ip.x - 10.0, // [m]
		ip.y - 10.0, // [m]
		ip.z -10, // [m]
		sp);

	set_yaw(ip.yaw-10, sp);

	api.update_setpoint(sp);
	// NOW pixhawk will try to move

	for (int i = 0; i < 8; i++) {
		mavlink_local_position_ned_t pos = api.current_messages.local_position_ned;
		printf("%i CURRENT VELOCITY XYZ = [ % .4f , % .4f , % .4f ] \n", i, pos.vx, pos.vy, pos.vz);
		Sleep(1);
	}

	api.disable_offboard_control();

	printf("\nREAD SOME MESSAGES \n");

	// copy current messages
	uint64_t big = 0;
	int msgs = 0;
	while(msgs < 1){
		Mavlink_Messages messages = api.current_messages;

		if (messages.highres_imu.time_usec != big) {
			big = messages.highres_imu.time_usec;
			// hires imu

			mavlink_highres_imu_t imu = messages.highres_imu;
			printf("Got message HIGHRES_IMU (spec: https://pixhawk.ethz.ch/mavlink/#HIGHRES_IMU)\n");
			printf("    ap time:     %llu \n", imu.time_usec);
			printf("    acc  (NED):  % f % f % f (m/s^2)\n", imu.xacc, imu.yacc, imu.zacc);
			printf("    gyro (NED):  % f % f % f (rad/s)\n", imu.xgyro, imu.ygyro, imu.zgyro);
			printf("    mag  (NED):  % f % f % f (Ga)\n", imu.xmag, imu.ymag, imu.zmag);
			printf("    altitude:    %f (m) \n", imu.pressure_alt);
			printf("    temperature: %f C \n", imu.temperature);

			printf("\n");
			msgs += 1;
		}
	}
}


// throws EXIT_FAILURE if could not open the port
void
parse_commandline(int argc, char **argv, int &portNum, int &baudrate)
{

	// string for command line usage
	const char *commandline_usage = "usage: mavlink_serial -d <devicename> -b <baudrate>";

	// Read input arguments
	for (int i = 1; i < argc; i++) { // argv[0] is "mavlink"

									 // Help
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			printf("%s\n", commandline_usage);
			throw EXIT_FAILURE;
		}

		// UART device ID
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--deviceNum") == 0) {
			if (argc > i + 1) {
				portNum = atoi(argv[i + 1]);

			} else {
				printf("%s\n", commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// Baud rate
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
			if (argc > i + 1) {
				baudrate = atoi(argv[i + 1]);

			} else {
				printf("%s\n", commandline_usage);
				throw EXIT_FAILURE;
			}
		}
	}
}

// Signal handler - this function is called when you press Ctrl-C
void
quit_handler(int sig)
{
	printf("\nTERMINATING AT USER REQUEST\n\n");

	// autopilot interface
	try {
		autopilot_interface_quit->handle_quit(sig);
	} catch (int error) {}

	// serial port
	try {
		serial_port_quit->handle_quit(sig);
	} catch (int error) {}

	exit(0);
}


int
main(int argc, char **argv)
{
	// This program uses throw, wrap one big try/catch here
	try {
		int result = top(argc, argv);
		return result;
	} catch (int error) {
		fprintf(stderr, "mavlink_control threw exception %i \n", error);
		return error;
	}
}