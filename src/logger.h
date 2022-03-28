/**
 * @file logger.h
 *
 * @brief data logger definition
 *
 * Functions for opening, closing, reading and writing telemetry files
 *
 * @author Stefan Bichlmaier, <bichlmaier.stef@gmail.com>
 *
 */

#ifndef LOGGER_H_
#define LOGGER_H_

//This module should allow me to write telemetry to a type of log file and storage medium/location chosen at runtime.
// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------
#include <iostream>
#include <fstream>
#include <unistd.h>
#include "autopilot_interface.h"
#include "c_library_v2/common/mavlink.h"

using namespace std;

// ------------------------------------------------------------------------------
//   Function prototypes
// ------------------------------------------------------------------------------
void *start_logger_write_thread(void *args); //pthread helper function

// ----------------------------------------------------------------------------------
//   Logger Class
// ----------------------------------------------------------------------------------
/*
 * Logger Class
 *
 */
class Logger
{

public:
    //Take parameters filename, dir, file specifics
    Logger(ofstream *file_, Mavlink_Messages *messages_); //Initialize file attributes to be ready for writing
    ~Logger();

    int start();
    void stop();
    void handle_quit( int sig );
    int write_thread();

    bool time_to_exit;
	pthread_t write_tid;

    int writingRate;
private:
    ofstream *file; //we don't want to change attributes of the file once the writing process has started
    Mavlink_Messages *messages;
};

#endif  //LOGGER_H_