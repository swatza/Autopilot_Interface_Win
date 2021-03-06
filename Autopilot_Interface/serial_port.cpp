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
* @file serial_port.cpp
*
* @brief Serial interface functions
*
* Functions for opening, closing, reading and writing via serial ports
*
* @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
* @author Jaycee Lock,    <jaycee.lock@gmail.com>
* @author Lorenz Meier,   <lm@inf.ethz.ch>\
*
* Modified by:
*	Spencer Watza, <s.g.watza@gmail.com>
*	James Jenkins, <james.p.jenkins@wmich.edu>
* For Windows Use
*/


#include "serial_port.h"
#include "stdafx.h"


SerialPort::SerialPort(int portNum_, int baudrate_)
{
	port = CSerial();
	port.Open(portNum_, baudrate_);

	lock = CreateMutex(NULL, FALSE, NULL);
}

SerialPort::~SerialPort()
{
	// destroy mutex
}

bool
SerialPort::isOpen()
{
	return port.IsOpened();
}


int
SerialPort::read_message(mavlink_message_t &message)
{
	uint8_t          cp;
	mavlink_status_t status;
	uint8_t          msgReceived = false;

	WaitForSingleObject(lock, INFINITE);
	int result = port.ReadData(&cp, 1);
	ReleaseMutex(lock);

	if (result > 0) {
		// Parse the message
		msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);

		// check for dropped packets
		if ((lastStatus.packet_rx_drop_count != status.packet_rx_drop_count) && debug) {
			printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
			unsigned char v = cp;
			fprintf(stderr, "%02x ", v);
		}

		lastStatus = status;
	}

	// Debugging reports
	if (msgReceived && debug) {
		// Report info
		printf("Received message from serial with ID #%d (sys:%d|comp:%d):\n", message.msgid, message.sysid, message.compid);

		fprintf(stderr, "Received serial data: ");
		unsigned int i;
		uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

		// check message is write length
		unsigned int messageLength = mavlink_msg_to_send_buffer(buffer, &message);

		// message length error
		//	        or
		// print out the buffer
		if (messageLength > MAVLINK_MAX_PACKET_LEN) {
			fprintf(stderr, "\nFATAL ERROR: MESSAGE LENGTH IS LARGER THAN BUFFER SIZE\n");
		} else {
			
			for (i = 0; i < messageLength; i++) {
				unsigned char v = buffer[i];
				fprintf(stderr, "%02x ", v);
			}
			fprintf(stderr, "\n");
		}
	}

	return msgReceived;
}


int SerialPort::write_message(const mavlink_message_t &message)
{
	char buf[300];

	// Translate message to buffer
	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);

	// Write buffer to serial port, locks port while writing
	WaitForSingleObject(lock, INFINITE);
	int bytesWritten = port.SendData(buf, len);
	ReleaseMutex(lock);

	return bytesWritten;
}

void SerialPort::close()
{
	//close_serial();
	port.Close();
}

void SerialPort::handle_quit(int sig)
{
	try {
		close();
	} catch (int error) {
		fprintf(stderr, "Warning, could not stop serial port\n");
	}
}