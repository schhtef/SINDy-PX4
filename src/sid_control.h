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

int main(int argc, char **argv);
//Device connection and configuration
int setup(int argc, char **argv);
//Runtime command handling
void flight_loop(std::shared_ptr<mavsdk::System> system, mavsdk::Telemetry &telemetry, SID &SINDy, Buffer &input_buffer, std::string logfile_directory);
void parse_commandline(int argc, char **argv, std::string &autopilot_path, std::string &logfile_directory, int &buffer_length, buffer_mode &mode, 
						float &stlsq_threshold, float &ridge_regression_penalty, bool &debug, std::string &debug_logfile_path);
//Interrupt handling
SID *SINDy_quit;
int system_state = GROUND_IDLE_STATE;
void quit_handler( int sig );

//Filename creation
char* generate_filename(int flights_since_reboot);