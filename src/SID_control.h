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

using std::string;
using namespace std;

#include "buffer.h"
#include "autopilot_interface.h"
#include "c_library_v2/common/mavlink.h"
#include "udp_port.h"
#include "serial_port.h"
#include "SID.h"
#include "plog/Log.h"
#include "plog/Initializers/RollingFileInitializer.h"

// Top state machine logic states
enum system_states
{
	GROUND_IDLE_STATE = 0, // Aircraft is on the ground and disarmed, no logging or system identification is needed
	FLIGHT_LOG_STATE = 1, // Aircraft is armed and ready for flight, start logging flight data only
	FLIGHT_LOG_SID_STATE = 2, // Aircraft is armed and ready for flight, system identification without commands will occur
	FLIGHT_LOG_SID_CMD_STATE = 3 // Aircraft is armed and ready for flight, system identification with commands will occur
};

int main(int argc, char **argv);
int top(int argc, char **argv);

//Runtime command handling
void commands(Autopilot_Interface &autopilot_interface, bool autotakeoff);
void parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate,
		bool &use_udp, char *&udp_ip, int &udp_port, bool &autotakeoff, string &filename, int *buffer_length);

//Interrupt handling
Autopilot_Interface *autopilot_interface_quit;
Generic_Port *port_quit;
SID *SINDy_quit;
int system_state = GROUND_IDLE_STATE;
void quit_handler( int sig );