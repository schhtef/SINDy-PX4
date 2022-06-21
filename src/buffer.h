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

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------
#include "c_library_v2/common/mavlink.h"
#include <utility>
#include <mutex>
#include <condition_variable>
#include <vector>       // std::vector
#include <algorithm>    // std::copy

using namespace std;

// ------------------------------------------------------------------------------
//   Data structures
// ------------------------------------------------------------------------------

// Package the individual arrays into a struct
struct Mavlink_Message_Buffers {
	// Local Position
	std::vector<mavlink_local_position_ned_t> buffered_local_position_ned;

	// Global Position
	std::vector<mavlink_global_position_int_t> buffered_global_position_int;

	// HiRes IMU
	std::vector<mavlink_highres_imu_t> buffered_highres_imu;

	// Attitude
	std::vector<mavlink_attitude_t> buffered_attitude;

	// Actuator Setpoint
	std::vector<mavlink_actuator_output_status_t> buffered_actuator_status;

    // erase all elements in each buffer
    void clear_buffers()
    {
        buffered_local_position_ned.clear();
        buffered_global_position_int.clear();
        buffered_highres_imu.clear();
        buffered_attitude.clear();
        buffered_actuator_status.clear();
    }
};

// ------------------------------------------------------------------------------
//   Function prototypes
// ------------------------------------------------------------------------------
void* start_buffer_read_thread(void *args); //pthread helper function
int find_max_length(Mavlink_Message_Buffers buffer); //buffer helper function

// ----------------------------------------------------------------------------------
//   Buffer Class
// ----------------------------------------------------------------------------------
/*
 * Generic Buffer Class so that buffers of multiple mavlink datatypes can be used
 *
 */
class Buffer
{
    int buffer_length;
    int buffer_counter = 0;
    Mavlink_Message_Buffers input_buffer;

    std::mutex mtx;
    std::condition_variable full;
    std::condition_variable not_full;

public:
    Buffer();
    Buffer(int buffer_length_);
    ~Buffer();


    int start();
    void stop();
    void handle_quit( int sig );
    int read_thread();

    void insert(mavlink_message_t message);
    
    Mavlink_Message_Buffers clear();

    bool time_to_exit;
	//pthread_t write_tid;
    
};



#endif  //Buffer_H_