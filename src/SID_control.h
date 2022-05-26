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

using std::string;
using namespace std;

#include "buffer.h"
#include "autopilot_interface.h"
#include "c_library_v2/common/mavlink.h"
#include "udp_port.h"
#include "serial_port.h"
#include "logger.h"
#include "SID.h"

int main(int argc, char **argv);
int top(int argc, char **argv);

//Runtime command handling
void commands(Autopilot_Interface &autopilot_interface, bool autotakeoff);
void parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate,
		bool &use_udp, char *&udp_ip, int &udp_port, bool &autotakeoff, string &filename, int buffer_length);

//Interrupt handling
Autopilot_Interface *autopilot_interface_quit;
Generic_Port *port_quit;
Logger *logger_quit;
void quit_handler( int sig );