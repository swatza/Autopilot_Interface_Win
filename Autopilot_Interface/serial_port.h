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
* @file serial_port.h
*
* @brief Serial interface definition
*
* Functions for opening, closing, reading and writing via serial ports
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

#pragma once
#ifndef SERIAL_PORT_H_
#define SERIAL_PORT_H_


#include <cstdlib>
#include <stdio.h>
//#include <unistd.h>  // UNIX standard function definitions
#include <fcntl.h>     // File control definitions
//#include <termios.h> // POSIX terminal control definitions
//#include <pthread.h> // This uses POSIX Threads
#include <signal.h>
#include <windows.h>
#include "cserial.h"
#include "utils.h"

#include "common/mavlink.h"


// The following two non-standard baudrates should have been defined by the system
// If not, just fallback to number
#ifndef B460800
#define B460800 460800
#endif

#ifndef B921600
#define B921600 921600
#endif


// Status flags
#define SERIAL_PORT_OPEN   1;
#define SERIAL_PORT_CLOSED 0;
#define SERIAL_PORT_ERROR -1;


/*
* This object handles the opening and closing of the offboard computer's
* serial port over which we'll communicate.  It also has methods to write
* a byte stream buffer.  MAVlink is not used in this object yet, it's just
* a serialization interface.  To help with read and write pthreading, it
* gaurds any port operation with a pthread mutex.
*/
class SerialPort
{

public:

	SerialPort(int portNum_, int baudrate_);
	~SerialPort();

	CSerial port;

	bool debug;
	int  status;

	bool isOpen();

	int read_message(mavlink_message_t &message);
	int write_message(const mavlink_message_t &message);

	void close();

	void handle_quit(int sig);

	

private:

	int  fd;
	mavlink_status_t lastStatus;
	HANDLE lock;
};

#endif // SERIAL_PORT_H_