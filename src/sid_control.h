#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <string.h>
#include <array>
#include <chrono>
#include<utility>

#include "toml.hpp"
#include <mavsdk/mavsdk.h> // general mavlink header
#include <mavsdk/plugins/telemetry/telemetry.h> // telemetry plugin
#include <mavsdk/plugins/info/info.h> // system info plugin
#include <mavsdk/log_callback.h> // mavlink logging
#include "buffer.h"
#include "system_identification.h"
#include "logging.h"

// Top state machine logic states
enum system_states
{
	GROUND_IDLE_STATE = 0, // Aircraft is on the ground and disarmed, no logging or system identification is needed
	FLIGHT_LOG_STATE = 1, // Aircraft is armed and ready for flight, start logging flight data only
	FLIGHT_LOG_SID_STATE = 2, // Aircraft is armed and ready for flight, system identification without commands will occur
	FLIGHT_LOG_SID_CMD_STATE = 3 // Aircraft is armed and ready for flight, system identification with commands will occur
};

// Struct to contain program runtime options
struct ProgramOptions{
	std::string autopilot_path;
	std::string coefficient_logfile_directory;
	std::string debug_logfile_path;
	BufferMode buffer_mode;
	int buffer_length;
	float ridge_regression_penalty;
	float stlsq_threshold;
	bool debug;

	void print_options()
	{
		std::cout << "autopilot device path: " << autopilot_path << "\n";
		std::cout << "coefficient log path: " <<  coefficient_logfile_directory << "\n";
		std::cout << "debug log path: " <<  debug_logfile_path << "\n";
		std::cout << "buffer mode: " <<  buffer_mode << "\n";
		std::cout << "buffer length: " <<  buffer_length << "\n";
		std::cout << "ridge regression penalty: " <<  ridge_regression_penalty << "\n";
		std::cout << "stlsq threshold: " <<  stlsq_threshold << "\n";
		std::cout << "debug to console: " <<  debug << "\n";
	}
};

int main(int argc, char **argv);
//Device connection and configuration
int setup(int argc, char **argv);
//Runtime command handling
void flight_loop(std::shared_ptr<mavsdk::System> system, mavsdk::Telemetry &telemetry, SID &SINDy, Buffer &input_buffer, std::string logfile_directory);
void parse_commandline(int argc, char **argv, std::string &config_file_path);
void parse_toml_config(const std::string config_file_path, ProgramOptions &options);
//Interrupt handling
SID *SINDy_quit;
int system_state = GROUND_IDLE_STATE;
void quit_handler( int sig );

//Filename creation
char* generate_filename(int flights_since_reboot);