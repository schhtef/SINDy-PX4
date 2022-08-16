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
struct Data_Buffer {
    std::vector<uint32_t> time_boot_ms; /*< [ms] Common Timestamp (time since system boot) used after interpolation.*/

    std::vector<uint32_t> attitude_time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
    std::vector<float> roll; /*< [rad] Roll angle (-pi..+pi)*/
    std::vector<float> pitch; /*< [rad] Pitch angle (-pi..+pi)*/
    std::vector<float> yaw; /*< [rad] Yaw angle (-pi..+pi)*/
    std::vector<float> rollspeed; /*< [rad/s] Roll angular speed*/
    std::vector<float> pitchspeed; /*< [rad/s] Pitch angular speed*/
    std::vector<float> yawspeed; /*< [rad/s] Yaw angular speed*/

    std::vector<uint32_t> local_time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
    std::vector<float> x; /*< [m] X Position*/
    std::vector<float> y; /*< [m] Y Position*/
    std::vector<float> z; /*< [m] Z Position*/
    std::vector<float> lvx; /*< [m/s] X Speed*/
    std::vector<float> lvy; /*< [m/s] Y Speed*/
    std::vector<float> lvz; /*< [m/s] Z Speed*/

    std::vector<uint32_t> wind_time_boot_ms; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
    std::vector<float> wind_x; /*< [m/s] Wind in X (NED) direction*/
    std::vector<float> wind_y; /*< [m/s] Wind in Y (NED) direction*/
    std::vector<float> wind_z; /*< [m/s] Wind in Z (NED) direction*/

    void clear_buffers()
    {
        attitude_time_boot_ms.clear(); /*< [ms] Timestamp (time since system boot).*/
        roll.clear(); /*< [rad] Roll angle (-pi..+pi)*/
        pitch.clear(); /*< [rad] Pitch angle (-pi..+pi)*/
        yaw.clear(); /*< [rad] Yaw angle (-pi..+pi)*/
        rollspeed.clear(); /*< [rad/s] Roll angular speed*/
        pitchspeed.clear(); /*< [rad/s] Pitch angular speed*/
        yawspeed.clear(); /*< [rad/s] Yaw angular speed*/

        local_time_boot_ms.clear(); /*< [ms] Timestamp (time since system boot).*/
        x.clear(); /*< [m] X Position*/
        y.clear(); /*< [m] Y Position*/
        z.clear(); /*< [m] Z Position*/
        lvx.clear(); /*< [m/s] X Speed*/
        lvy.clear(); /*< [m/s] Y Speed*/
        lvz.clear(); /*< [m/s] Z Speed*/

        wind_time_boot_ms.clear();
        wind_x.clear();
        wind_y.clear();
        wind_z.clear();
    }

    int find_max_length()
    {
        int max_length = 0;
        if(attitude_time_boot_ms.size() > max_length)
        {
            max_length = attitude_time_boot_ms.size();
        }
        if(local_time_boot_ms.size() > max_length)
        {
            max_length = local_time_boot_ms.size();
        }
        if(wind_time_boot_ms.size() > max_length)
        {
            max_length = wind_time_boot_ms.size();
        }
        return max_length;
    }
};

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

    Data_Buffer buffer;
    std::mutex mtx;
    std::condition_variable full;
    std::condition_variable not_full;

public:
    Buffer();
    Buffer(int buffer_length_);
    ~Buffer();

    void insert(mavlink_message_t message);
    Data_Buffer clear();
};



#endif  //Buffer_H_