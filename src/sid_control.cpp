// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "sid_control.h"


// ------------------------------------------------------------------------------
//   TOP
// ------------------------------------------------------------------------------
int
setup (int argc, char **argv)
{

	// Set program defaults
	string autopilot_path = "udp://:14540";
	string logfile_directory = "../logs/";
	string buffer_mode = "length";
	int buffer_length = 100;

	// Parse command line arguments
	parse_commandline(argc, argv, autopilot_path, logfile_directory, buffer_length, buffer_mode);
	
	using namespace mavsdk;

	// Instantiate MAVSDK Object
    Mavsdk mavsdk;
	// Find autopilot system using UDP or Serial device path
    ConnectionResult connection_result = mavsdk.add_any_connection(autopilot_path);

	// make sure we have connected successfully to the autopilot
	if (connection_result != ConnectionResult::Success) {
		std::cerr << "Connection failed: " << connection_result << '\n';
	}

	// Wait for system to connect
	while (mavsdk.systems().size() == 0) {
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	// System got discovered.
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
	 * Start the system identification thread
	 */
	SINDy.start();

	// instantiate telemetry object
	
	Telemetry telemetry = Telemetry{system};

	// set a base time at which the telemetry items are timestamped
	auto program_epoch = std::chrono::high_resolution_clock::now();

	// Subscribe to telemetry sources, inserting into the buffer on every new telemetry item
	// Each subscription dispatches a thread which listens for a new item, calling the lambda function when one is received
	telemetry.subscribe_attitude_euler([&input_buffer, program_epoch](Telemetry::EulerAngle attitude) {
        auto now = std::chrono::high_resolution_clock::now();
		uint64_t sample_time = std::chrono::duration_cast<std::chrono::milliseconds>(now - program_epoch).count();
		input_buffer.insert(attitude, sample_time);
    });

	telemetry.subscribe_attitude_angular_velocity_body([&input_buffer, program_epoch](Telemetry::AngularVelocityBody angular_velocity) {
        auto now = std::chrono::high_resolution_clock::now();
		uint64_t sample_time = std::chrono::duration_cast<std::chrono::milliseconds>(now - program_epoch).count();
		input_buffer.insert(angular_velocity, sample_time);
    });

	telemetry.subscribe_odometry([&input_buffer, program_epoch](Telemetry::Odometry state) {
        auto now = std::chrono::high_resolution_clock::now();
		uint64_t sample_time = std::chrono::duration_cast<std::chrono::milliseconds>(now - program_epoch).count();
		input_buffer.insert(state, sample_time);
    });

	telemetry.subscribe_actuator_control_target([&input_buffer, program_epoch](Telemetry::ActuatorControlTarget actuator){
        auto now = std::chrono::high_resolution_clock::now();
		uint64_t sample_time = std::chrono::duration_cast<std::chrono::milliseconds>(now - program_epoch).count();
		input_buffer.insert(actuator, sample_time);
	});
	
	// Run the main event loop
	flight_loop(system, telemetry, SINDy, input_buffer, logfile_directory);

	// woot!
	return 0;
}

// ------------------------------------------------------------------------------
//   COMMANDS
// ------------------------------------------------------------------------------

void flight_loop(std::shared_ptr<mavsdk::System> system, mavsdk::Telemetry &telemetry, SID &SINDy, Buffer &input_buffer, std::string logfile_directory)
{
	using namespace mavsdk;

	// Keep track of number of times the autopilot has been armed to separate flight tests into different files
	int flights_since_reboot = 0;
	
	// Main event loop
	while(1)
	{
		
		switch(system_state)
		{
			case GROUND_IDLE_STATE:
				// If autopilot mode is armed, switch to FLIGHT_LOG_STATE
				if(telemetry.armed())
				{
					system_state = FLIGHT_LOG_STATE;
					SINDy.flight_number++;
					printf("Autopilot armed. Starting system identification for flight number %d\n", SINDy.flight_number);
					SINDy.armed = true;
				}
			break;

			case FLIGHT_LOG_STATE:
				// If autopilot is disarmed, switch to GROUND_IDLE_STATE
				if(!telemetry.armed())
				{
					system_state = GROUND_IDLE_STATE;
					printf("Autopilot disarmed. Stopped logging\n");
					SINDy.armed = false;
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
void parse_commandline(int argc, char **argv, string &autopilot_path, string &logfile_directory, int &buffer_length, string &buffer_mode)
{

	// string for command line usage
	std::string commandline_usage = "usage: SID_offboard -d <Device Path>\n udp://[host][:port]\ntcp://[host][:port]\n serial://[path][:baudrate]";
	commandline_usage += "-l <logfile directory>\n -b <buffer length>\n -m <buffer mode>\n time or length";
	char *val;
	// Read input arguments
	for (int i = 1; i < argc; i++) { // argv[0] is "mavlink"
		val = argv[i];
		printf("%s\n",val);
		// Help
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			std::cout << commandline_usage;
			throw EXIT_FAILURE;
		}

		// logfile
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--devpath") == 0) {
			if (argc > i + 1) {
				i++;
				autopilot_path = (argv[i]);
			} else {
				std::cout << commandline_usage;
				throw EXIT_FAILURE;
			}
		}

		// logfile
		if (strcmp(argv[i], "-l") == 0 || strcmp(argv[i], "--log") == 0) {
			if (argc > i + 1) {
				i++;
				logfile_directory = (argv[i]);
			} else {
				std::cout << commandline_usage;
				throw EXIT_FAILURE;
			}
		}

		// buffer length
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--buffer") == 0) {
			if (argc > i + 1) {
				i++;
				buffer_length = atoi(argv[i]);
			} else {
				std::cout << commandline_usage;
				throw EXIT_FAILURE;
			}
		}

		// buffer mode
		if (strcmp(argv[i], "-m") == 0 || strcmp(argv[i], "--mode") == 0) {
			if (argc > i + 1) {
				i++;
				buffer_mode = (argv[i]);
			} else {
				//std::cout << commandline_usage;
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
void quit_handler( int sig )
{
	printf("\n");
	printf("TERMINATING AT USER REQUEST\n");
	printf("\n");

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
		int result = setup(argc,argv);
		return result;
	}

	catch ( int error )
	{
		fprintf(stderr,"exception thrown: %i \n" , error);
		return error;
	}

}

