// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "sid_control.h"

// ------------------------------------------------------------------------------
//   TOP
// ------------------------------------------------------------------------------
int setup(int argc, char **argv)
{
	using namespace std;
	// Set program defaults
	string autopilot_path = "udp://:14540";
	string coefficient_logfile_directory = "../logs/coefficients.csv";
	string debug_logfile_path = "../logs/mavlink_debug_log.csv";

	buffer_mode mode = buffer_mode::length_mode;
	int buffer_length = 100;
	float ridge_regression_penalty = 0.1;
	float stlsq_threshold = 0.1;
	bool debug = false;

	// Parse command line arguments
	parse_commandline(argc, argv, autopilot_path, coefficient_logfile_directory, buffer_length, mode, ridge_regression_penalty,
					  stlsq_threshold, debug, debug_logfile_path);

	// set a base time at which the telemetry items are timestamped
	std::chrono::_V2::system_clock::time_point program_epoch = std::chrono::high_resolution_clock::now();

	using namespace mavsdk;

	// Instantiate MAVSDK Object
	Mavsdk mavsdk;
	// Find autopilot system using UDP or Serial device path
	ConnectionResult connection_result = mavsdk.add_any_connection(autopilot_path);

	// make sure we have connected successfully to the autopilot
	if (connection_result != ConnectionResult::Success)
	{
		std::cerr << "Connection failed: " << connection_result << '\n';
	}

	// Wait for system to connect
	while (mavsdk.systems().size() == 0)
	{
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	// System got discovered.
	std::shared_ptr<System> system = mavsdk.systems()[0];

	/*
	 * Instantiate buffer objects
	 *
	 * This object handles the synchronization between the producer (px4 autopilot)
	 * and consumer (SINDy). It creates a buffer for the desired data of length BUFFER_LENGTH.
	 * The buffer implements mutexing and condition variables which allows the separate
	 * Producer and Consumer threads to access it without data races. Each buffer instance is
	 * associated with a Mavlink type which is passed into the SID
	 *
	 */
	Buffer input_buffer(buffer_length, mode);

	/*
	 * Instantiate a system identification object
	 *
	 * This object takes data from the input buffer and implements the SINDy algorithm on it
	 *
	 */
	SID SINDy(&input_buffer, program_epoch, stlsq_threshold, ridge_regression_penalty, coefficient_logfile_directory, debug);

	/*
	 * Setup interrupt signal handler
	 *
	 * Responds to early exits signaled with Ctrl-C.  The handler will command
	 * to exit offboard mode if required, and close threads and the port.
	 * The handler in this example needs references to the above objects.
	 *
	 */

	SINDy_quit = &SINDy;
	signal(SIGINT, quit_handler);

	// instantiate telemetry object

	Telemetry telemetry = Telemetry{system};
	auto info = Info{system};

	const Info::Identification &systemInformation = info.get_identification().second;
	const Info::Product &systemProduct = info.get_product().second;
	const Info::Version &systemVersion = info.get_version().second;

	std::cout << " Hardware uid: " << systemInformation.hardware_uid << '\n';
	std::cout << " Legacy Hardware uid: " << systemInformation.legacy_uid << '\n';

	// Print out the vehicle version information.
	std::cout << "  flight_sw_major: " << systemVersion.flight_sw_major << '\n'
			  << "  flight_sw_minor: " << systemVersion.flight_sw_minor << '\n'
			  << "  flight_sw_patch: " << systemVersion.flight_sw_patch << '\n'
			  << "  flight_sw_vendor_major: " << systemVersion.flight_sw_vendor_major << '\n'
			  << "  flight_sw_vendor_minor: " << systemVersion.flight_sw_vendor_minor << '\n'
			  << "  flight_sw_vendor_patch: " << systemVersion.flight_sw_vendor_patch << '\n'
			  << "  flight_sw_git_hash: " << systemVersion.flight_sw_git_hash << '\n'
			  << "  os_sw_major: " << systemVersion.os_sw_major << '\n'
			  << "  os_sw_minor: " << systemVersion.os_sw_minor << '\n'
			  << "  os_sw_patch: " << systemVersion.os_sw_patch << '\n'
			  << "  os_sw_git_hash: " << systemVersion.os_sw_git_hash << '\n';

	// Print out the vehicle product information.
	std::cout << "  vendor_id: " << systemProduct.vendor_id << '\n'
			  << "  vendor_name: " << systemProduct.vendor_name << '\n'
			  << "  product_id: " << systemProduct.product_id << '\n'
			  << "  product_name: " << systemProduct.product_id << '\n';

	// Subscribe to mavlink logs
	mavsdk::log::subscribe([debug_logfile_path](mavsdk::log::Level level,	// message severity level
												const std::string &message, // message text
												const std::string &file,	// source file from which the message was sent
												int line) {					// line number in the source file
		// process the log message in a way you like
		log_mavlink_info(level, message, debug_logfile_path);

		// returning true from the callback disables printing the message to stdout
		return level < mavsdk::log::Level::Warn;
	});

	// Subscribe to telemetry sources, inserting into the buffer on every new telemetry item
	// Each subscription dispatches a thread which listens for a new item, calling the lambda function when one is received
	telemetry.subscribe_attitude_euler([&input_buffer, program_epoch](Telemetry::EulerAngle attitude)
									   {
        auto now = std::chrono::high_resolution_clock::now();
		uint64_t sample_time = std::chrono::duration_cast<std::chrono::milliseconds>(now - program_epoch).count();
		input_buffer.insert(attitude, sample_time); });

	telemetry.subscribe_attitude_angular_velocity_body([&input_buffer, program_epoch](Telemetry::AngularVelocityBody angular_velocity)
													   {
        auto now = std::chrono::high_resolution_clock::now();
		uint64_t sample_time = std::chrono::duration_cast<std::chrono::milliseconds>(now - program_epoch).count();
		input_buffer.insert(angular_velocity, sample_time); });

	telemetry.subscribe_odometry([&input_buffer, program_epoch](Telemetry::Odometry state)
								 {
        auto now = std::chrono::high_resolution_clock::now();
		uint64_t sample_time = std::chrono::duration_cast<std::chrono::milliseconds>(now - program_epoch).count();
		input_buffer.insert(state, sample_time); });

	telemetry.subscribe_actuator_control_target([&input_buffer, program_epoch](Telemetry::ActuatorControlTarget actuator)
												{
        auto now = std::chrono::high_resolution_clock::now();
		uint64_t sample_time = std::chrono::duration_cast<std::chrono::milliseconds>(now - program_epoch).count();
		input_buffer.insert(actuator, sample_time); });

	// Run the main event loop
	flight_loop(system, telemetry, SINDy, input_buffer, coefficient_logfile_directory);

	// woot!
	return 0;
}

// ------------------------------------------------------------------------------
//   COMMANDS
// ------------------------------------------------------------------------------

void flight_loop(std::shared_ptr<mavsdk::System> system, mavsdk::Telemetry &telemetry, SID &SINDy, Buffer &input_buffer, std::string logfile_directory)
{
	using namespace mavsdk;

	// Main event loop
	while (1)
	{
		SINDy.start();
		// Top level state machine will go here
	}
	printf("\n");

	return;
}

// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
void parse_commandline(int argc, char **argv, std::string &autopilot_path, std::string &coefficient_logfile_path, int &buffer_length, buffer_mode &mode,
					   float &stlsq_threshold, float &ridge_regression_penalty, bool &debug, std::string &debug_logfile_path)
{
	using namespace std;
	// string for command line usage
	string commandline_usage = "usage: SID_offboard\nOptions:\n-p <Device Path>\n\tudp://[host][:port]\n\ttcp://[host][:port]\n\tserial://[path][:baudrate]\n";
	commandline_usage += "-l <logfile directory>\n-b <buffer length>\n-m <buffer mode>\n\ttime or length\n-t <STLSQ threshold>\n-r <Ridge regression penalty>\n-d <debug output>\n";
	char *val;
	// Read input arguments
	for (int i = 1; i < argc; i++)
	{ // argv[0] is "SINDy_offboard"
		printf("%s\n", argv[0]);
		val = argv[i];
		printf("%s\n", val);
		// Help
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0)
		{
			std::cout << commandline_usage;
			throw EXIT_FAILURE;
		}

		// devicepath
		if (strcmp(argv[i], "-p") == 0 || strcmp(argv[i], "--devpath") == 0)
		{
			if (argc > i + 1)
			{
				i++;
				autopilot_path = (argv[i]);
			}
			else
			{
				std::cout << commandline_usage;
				throw EXIT_FAILURE;
			}
		}

		// coefficient logfile path
		if (strcmp(argv[i], "-l") == 0 || strcmp(argv[i], "--log") == 0)
		{
			if (argc > i + 1)
			{
				i++;
				coefficient_logfile_path = (argv[i]);
			}
			else
			{
				std::cout << commandline_usage;
				throw EXIT_FAILURE;
			}
		}

		// buffer length
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--buffer") == 0)
		{
			if (argc > i + 1)
			{
				i++;
				buffer_length = atoi(argv[i]);
			}
			else
			{
				std::cout << commandline_usage;
				throw EXIT_FAILURE;
			}
		}

		// buffer mode
		if (strcmp(argv[i], "-m") == 0 || strcmp(argv[i], "--mode") == 0)
		{
			if (argc > i + 1)
			{
				i++;
				string buffer_mode_string = (argv[i]);
				if (buffer_mode_string == "length")
				{
					mode = buffer_mode::length_mode;
				}
				else if (buffer_mode_string == "time")
				{
					mode = buffer_mode::time_mode;
				}
				else
				{
					std::cout << "Invalid argument for -m option, use length or time\n";
					throw EXIT_FAILURE;
				}
			}
			else
			{
				std::cout << commandline_usage;
				throw EXIT_FAILURE;
			}
		}

		// STLSQ thresholding parameter
		if (strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--threshold") == 0)
		{
			if (argc > i + 1)
			{
				i++;
				stlsq_threshold = atoi(argv[i]);
			}
			else
			{
				// std::cout << commandline_usage;
				throw EXIT_FAILURE;
			}
		}

		// ridge regression penalty
		if (strcmp(argv[i], "-r") == 0 || strcmp(argv[i], "--lambda") == 0)
		{
			if (argc > i + 1)
			{
				i++;
				ridge_regression_penalty = atoi(argv[i]);
			}
			else
			{
				// std::cout << commandline_usage;
				throw EXIT_FAILURE;
			}
		}

		// debug option
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--debug") == 0)
		{
			debug = true;
			if (argc > i + 1)
			{
				i++;
				debug_logfile_path = argv[i];
			}
			else
			{
				// std::cout << commandline_usage;
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
void quit_handler(int sig)
{
	printf("\n");
	printf("TERMINATING AT USER REQUEST\n");
	printf("\n");

	// SID
	try
	{
		SINDy_quit->stop();
	}
	catch (int error)
	{
	}
	// end program here
	exit(0);
}

// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int main(int argc, char **argv)
{
	// This program uses throw, wrap one big try/catch here
	try
	{
		int result = setup(argc, argv);
		return result;
	}

	catch (int error)
	{
		fprintf(stderr, "exception thrown: %i \n", error);
		return error;
	}
}
