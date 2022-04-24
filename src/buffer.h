/**
 * @file buffer.h
 *
 * @brief data buffer definition
 *
 * Functions for collecting telemetry in a buffer suitable for SINDy
 *
 * @author Stefan Bichlmaier, <bichlmaier.stef@gmail.com>
 *
 */

#ifndef BUFFER_H_
#define BUFFER_H_

//This module should allow me to write telemetry to a type of log file and storage medium/location chosen at runtime.
// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------
#include "autopilot_interface.h"
#include "c_library_v2/common/mavlink.h"

using namespace std;

// ------------------------------------------------------------------------------
//   Function prototypes
// ------------------------------------------------------------------------------
void* start_buffer_read_thread(void *args); //pthread helper function

// ----------------------------------------------------------------------------------
//   Buffer Class
// ----------------------------------------------------------------------------------
/*
 * Buffer Class
 *
 */
class Buffer
{

public:
    // Pass mavlink messages object so we can read from it
    Buffer(Mavlink_Messages *messages_);
    ~Buffer();

    int start();
    void stop();
    void handle_quit( int sig );
    int read_thread();

    bool time_to_exit;
	pthread_t write_tid;

    int readingRate;
private:
    Mavlink_Messages *messages;
};

#endif  //LOGGER_H_