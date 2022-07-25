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
 */

// ------------------------------------------------------------------------------
//   Defines
// ------------------------------------------------------------------------------


// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "SID_control.h"


// ------------------------------------------------------------------------------
//   TOP
// ------------------------------------------------------------------------------
int
top (int argc, char **argv)
{

	// --------------------------------------------------------------------------
	//   PARSE THE COMMANDS
	// --------------------------------------------------------------------------

	// Default input arguments
#ifdef __APPLE__
	char *uart_name = (char*)"/dev/tty.usbmodem1";
#else
	char *uart_name = (char*)"/dev/ttyUSB0";
#endif
	int baudrate = 57600;

	bool use_udp = false;
	char *udp_ip = (char*)"127.0.0.1";
	int udp_port = 14540;
	bool autotakeoff = false;
	string logfile_directory = "/home/stefan/Documents/PX4-SID/tests/";
	int buffer_length = 100;

	// do the parse, will throw an int if it fails
	parse_commandline(argc, argv, uart_name, baudrate, use_udp, udp_ip, udp_port, autotakeoff, logfile_directory, &buffer_length);
	
	// --------------------------------------------------------------------------
	//   PORT and THREAD STARTUP
	// --------------------------------------------------------------------------

	/*
	 * Instantiate a generic port object
	 *
	 * This object handles the opening and closing of the offboard computer's
	 * port over which it will communicate to an autopilot.  It has
	 * methods to read and write a mavlink_message_t object.  To help with read
	 * and write in the context of pthreading, it guards port operations with a
	 * pthread mutex lock. It can be a serial or an UDP port.
	 *
	 */
	Generic_Port *port;
	if(use_udp)
	{
		port = new UDP_Port(udp_ip, udp_port);
	}
	else
	{
		port = new Serial_Port(uart_name, baudrate);
	}

	/*
	 * Instantiate buffer objects
	 *
	 * This object handles the synchronization between the producer (autopilot interface)
	 * and consumer (SINDy). It creates a buffer for the desired data of length BUFFER_LENGTH.
	 * The buffer implements mutexing and condition variables which allows the separate
	 * Producer and Consumer threads to access it without data races. Each buffer instance is 
	 * associated with a Mavlink type which is passed into the SID
	 *
	 */
	Buffer input_buffer(buffer_length);
	
	/*
	 * Instantiate a system identification object
	 *
	 * This object takes data from the input buffer and implements the SINDy algorithm on it
	 *
	 */
	SID SINDy(&input_buffer);

	/*
	 * Instantiate an autopilot interface object
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
	 * 
	 * The input buffer pointer is passed into its constructor so it can write to the buffer
	 *
	 */
	Autopilot_Interface autopilot_interface(port, &input_buffer);

	/*
	 * Setup interrupt signal handler
	 *
	 * Responds to early exits signaled with Ctrl-C.  The handler will command
	 * to exit offboard mode if required, and close threads and the port.
	 * The handler in this example needs references to the above objects.
	 *
	 */
	port_quit         = port;
	autopilot_interface_quit = &autopilot_interface;
	SINDy_quit = &SINDy;
	signal(SIGINT,quit_handler);

	/*
	 * Start the port, autopilot_interface, and logger
	 * This is where the port is opened, and read and write threads are started.
	 */
	port->start();
	autopilot_interface.start();
	SINDy.start();


	// --------------------------------------------------------------------------
	//   RUN COMMANDS
	// --------------------------------------------------------------------------

	/*
	 * Now we can implement the algorithm we want on top of the autopilot interface
	 */
	commands(autopilot_interface, SINDy, autotakeoff, logfile_directory);


	// --------------------------------------------------------------------------
	//   THREAD and PORT SHUTDOWN
	// --------------------------------------------------------------------------

	/*
	 * Now that we are done we can stop the threads and close the port
	 */
	autopilot_interface.stop();
	port->stop();

	delete port;

	// --------------------------------------------------------------------------
	//   DONE
	// --------------------------------------------------------------------------

	// woot!
	return 0;

}


// ------------------------------------------------------------------------------
//   COMMANDS
// ------------------------------------------------------------------------------

void
commands(Autopilot_Interface &api, SID &SINDy, bool autotakeoff, string logfile_directory)
{
	// Primary event variables
	int flights_since_reboot = 0;

	//Request mavlink streams in addition to defaults

	// Prepare command for setting message interval
	// TODO put command generation into helper function
	
	mavlink_command_int_t com = { 0 };
	com.target_system    = api.system_id; //Companion system id
	com.target_component = api.autopilot_id; //Autopilot system id
	com.command          = MAV_CMD_SET_MESSAGE_INTERVAL; //Command to send
	com.param1           = MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS; //Requested Message
	com.param2           = 100000; //Default message interval

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_int_encode(api.system_id, api.companion_id, &message, &com);

	// Send the message
	int len = api.write_message(message);

	if(!len)
	{
		fprintf(stderr, "Failed to set message interval\n");
	}
	
	com = { 0 };
	com.target_system    = api.system_id; //Companion system id
	com.target_component = api.autopilot_id; //Autopilot system id
	com.command          = MAV_CMD_SET_MESSAGE_INTERVAL; //Command to send
	com.param1           = MAVLINK_MSG_ID_GLOBAL_POSITION_INT; //Requested Message
	com.param2           = 100000; //Default message interval

	mavlink_msg_command_int_encode(api.system_id, api.companion_id, &message, &com);

	// Send the message
	len = api.write_message(message);

	if(!len)
	{
		fprintf(stderr, "Failed to set message interval\n");
	}

	// copy current messages
	Mavlink_Messages messages = api.current_messages;

	// local position in ned frame
	mavlink_local_position_ned_t pos = messages.local_position_ned;
	printf("Initial LOCAL_POSITION_NED (spec: https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED)\n");
	printf("    pos  (NED):  %f %f %f (m)\n", pos.x, pos.y, pos.z );

	// Primary event loop
	while(1)
	{
		int isarmed = api.current_messages.heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED;
		switch(system_state)
		{
			case GROUND_IDLE_STATE:
				// If autopilot mode is armed, switch to FLIGHT_LOG_STATE
				if(api.autopilot_armed)
				{
					system_state = FLIGHT_LOG_STATE;
					flights_since_reboot++;
					SINDy.filename = logfile_directory + "Flight Number: " + to_string(flights_since_reboot) + ".csv";
					printf("Autopilot armed. Starting system identification for flight number %d\n", flights_since_reboot);
					SINDy.disarmed = false;
				}
			break;

			case FLIGHT_LOG_STATE:
				// If autopilot is disarmed, switch to GROUND_IDLE_STATE
				if(!(api.autopilot_armed))
				{
					system_state = GROUND_IDLE_STATE;
					printf("Autopilot disarmed. Stopping system identification\n");
					// Stop the system identification thread
					SINDy.disarmed = true;
				}
			break;

			case FLIGHT_LOG_SID_STATE:
				//TODO implement switch to passive system identification state
			break;

			case FLIGHT_LOG_SID_CMD_STATE:
				//TODO implement switch to active system identification state
			break;
		}
		//printf("Base Mode: %d\n", api.current_messages.heartbeat.base_mode);
	}
	printf("\n");


	// --------------------------------------------------------------------------
	//   END OF COMMANDS
	// --------------------------------------------------------------------------

	return;

}


// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
// throws EXIT_FAILURE if could not open the port
void
parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate,
		bool &use_udp, char *&udp_ip, int &udp_port, bool &autotakeoff, string &filename, int *buffer_length)
{

	// string for command line usage
	const char *commandline_usage = "usage: mavlink_control [-d <devicename> -b <baudrate>] [-u <udp_ip> -p <udp_port>] [-a ]";
		char *val;
	// Read input arguments
	for (int i = 1; i < argc; i++) { // argv[0] is "mavlink"
		val = argv[i];
		printf("%s\n",val);
		// Help
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			printf("%s\n",commandline_usage);
			throw EXIT_FAILURE;
		}

		// UART device ID
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
			if (argc > i + 1) {
				i++;
				uart_name = argv[i];
			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// Baud rate
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
			if (argc > i + 1) {
				i++;
				baudrate = atoi(argv[i]);
			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// UDP ip
		if (strcmp(argv[i], "-u") == 0 || strcmp(argv[i], "--udp_ip") == 0) {
			if (argc > i + 1) {
				i++;
				udp_ip = argv[i];
				use_udp = true;
			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}
		
		// UDP port
		if (strcmp(argv[i], "-p") == 0 || strcmp(argv[i], "--port") == 0) {
			if (argc > i + 1) {
				i++;
				udp_port = atoi(argv[i]);
			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// Autotakeoff
		if (strcmp(argv[i], "-a") == 0 || strcmp(argv[i], "--autotakeoff") == 0) {
			autotakeoff = true;
		}

		// logfile
		if (strcmp(argv[i], "-l") == 0 || strcmp(argv[i], "--log") == 0) {
			if (argc > i + 1) {
				i++;
				filename = (argv[i]);
			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// buffer
		if (strcmp(argv[i], "-s") == 0 || strcmp(argv[i], "--buffer") == 0) {
			if (argc > i + 1) {
				i++;
				*buffer_length = atoi(argv[i]);
			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

	}
	// end: for each input argument

	// Done!
	return;
}


// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void
quit_handler( int sig )
{
	printf("\n");
	printf("TERMINATING AT USER REQUEST\n");
	printf("\n");

	// autopilot interface
	try {
		autopilot_interface_quit->handle_quit(sig);
	}
	catch (int error){}

	// port
	try {
		port_quit->stop();
	}
	catch (int error){}

	// SID
	try {
		SINDy_quit->stop();
	}
	catch (int error){}
	// end program here
	exit(0);

}


// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int
main(int argc, char **argv)
{
	// This program uses throw, wrap one big try/catch here
	try
	{
		int result = top(argc,argv);
		return result;
	}

	catch ( int error )
	{
		fprintf(stderr,"mavlink_control threw exception %i \n" , error);
		return error;
	}

}

