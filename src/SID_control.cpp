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
	char *uart_name = (char*)"/dev/ttyUSB0";
	int baudrate = 57600;
	bool use_udp = false;
	char *udp_ip = (char*)"127.0.0.1";
	int udp_port = 14540;
	bool autotakeoff = false;
	string logfile_directory = "/home/stefan/Documents/PX4-SID/tests/";
	string buffer_mode = "length";
	int buffer_length = 100;

	// do the parse, will throw an int if it fails
	parse_commandline(argc, argv, uart_name, baudrate, use_udp, udp_ip, udp_port, 
						autotakeoff, logfile_directory, buffer_length, buffer_mode);
	
	// --------------------------------------------------------------------------
	//   Instantiate MAVSDK Object
	// --------------------------------------------------------------------------
	using namespace mavsdk;

    Mavsdk mavsdk;
    ConnectionResult connection_result = mavsdk.add_any_connection(argv[1]);

	// Wait for the system to connect
	while (mavsdk.systems().size() == 0) {
	}

	// make sure we have connected successfully to the autopilot
	if (connection_result != ConnectionResult::Success) {
		std::cerr << "Connection failed: " << connection_result << '\n';
		return 1;
	}

	// assuming one autopilot connected, get a pointer to the first one connected
	std::shared_ptr<System> system = mavsdk.systems()[0];

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
	Buffer input_buffer(buffer_length, buffer_mode);
	
	/*
	 * Instantiate a system identification object
	 *
	 * This object takes data from the input buffer and implements the SINDy algorithm on it
	 *
	 */
	SID SINDy(&input_buffer);

	/*
	 * Setup interrupt signal handler
	 *
	 * Responds to early exits signaled with Ctrl-C.  The handler will command
	 * to exit offboard mode if required, and close threads and the port.
	 * The handler in this example needs references to the above objects.
	 *
	 */

	SINDy_quit = &SINDy;
	signal(SIGINT,quit_handler);

	/*
	 * Start the port, autopilot_interface, and logger
	 * This is where the port is opened, and read and write threads are started.
	 */
	SINDy.start();


	// --------------------------------------------------------------------------
	//   RUN COMMANDS
	// --------------------------------------------------------------------------
	commands(system, SINDy, autotakeoff, logfile_directory);


	// --------------------------------------------------------------------------
	//   THREAD and PORT SHUTDOWN
	// --------------------------------------------------------------------------


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
commands(std::shared_ptr<mavsdk::System> system, SID &SINDy, Buffer &input_buffer, string logfile_directory)
{
	using namespace mavsdk;

	// Primary event variables
	int flights_since_reboot = 0;

	// instantiate telemetry object
	Telemetry telemetry = Telemetry{system};

	telemetry.subscribe_attitude_euler(input_buffer.insert(Telemetry::attitude_euler attitude));


	// Prepare command for setting message interval
	// TODO put command generation into helper function
	
	//Request actuator output status messages
	mavlink_command_int_t com = { 0 };
	com.target_system    = api.system_id; //Companion system id
	com.target_component = api.autopilot_id; //Autopilot system id
	com.command          = MAV_CMD_SET_MESSAGE_INTERVAL; //Command to send
	com.param1           = MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS; //Requested Message
	com.param2           = 100000; //Default message interval

	// Request wind messages
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
	com.param1           = MAVLINK_MSG_ID_WIND_COV; //Requested Message
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
		bool &use_udp, char *&udp_ip, int &udp_port, bool &autotakeoff, string &filename, int &buffer_length, string &buffer_mode)
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

		// buffer length
		if (strcmp(argv[i], "-s") == 0 || strcmp(argv[i], "--buffer") == 0) {
			if (argc > i + 1) {
				i++;
				buffer_length = atoi(argv[i]);
			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// buffer mode
		if (strcmp(argv[i], "-m") == 0 || strcmp(argv[i], "--mode") == 0) {
			if (argc > i + 1) {
				i++;
				buffer_mode = (argv[i]);
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

