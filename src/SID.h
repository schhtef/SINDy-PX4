/**
 * @file SID.h
 *
 * @brief system identification computational definition
 *
 * Functions for implementing SINDy
 *
 * @author Stefan Bichlmaier, <bichlmaier.stef@gmail.com>
 *
 */

#ifndef SID_H_
#define SID_H_

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------
#include "autopilot_interface.h"
#include "c_library_v2/common/mavlink.h"
#include "buffer.h"
#include "plog/Log.h"
#include "plog/Initializers/RollingFileInitializer.h"
#include "logger.h"
#include <string>

using namespace std;

// ----------------------------------------------------------------------------------
//   System Identification Class
// ----------------------------------------------------------------------------------

class SID
{
private:
    Buffer *input_buffer;
    Mavlink_Message_Buffers data;
    bool time_to_exit = false;
    pthread_t compute_tid = 0;
public:
    SID(Buffer *input_buffer_);
    ~SID();

    void compute_thread();
    void interpolate();
    void start();
    void stop();
    void handle_quit(int sig);

    bool compute_status;
    bool disarmed;
    string filename;
};

#endif