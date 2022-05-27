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
//   Function prototypes
// ------------------------------------------------------------------------------
void* start_buffer_read_thread(void *args); //pthread helper function

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
    std::vector<pair <mavlink_highres_imu_t, uint64_t>> buffer;
    int buffer_counter = 0;

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
    
    void insert(pair <mavlink_highres_imu_t, uint64_t> element);
    std::vector<pair <mavlink_highres_imu_t, uint64_t>> clear();



    bool time_to_exit;
	//pthread_t write_tid;
    
};

#endif  //Buffer_H_