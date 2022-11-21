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
#include <utility>
#include <mutex>
#include <condition_variable>
#include <vector>       // std::vector
#include <algorithm>    // std::copy
#include <assert.h>
#include <chrono>
#include <mavsdk/mavsdk.h> // general mavlink header
#include <mavsdk/plugins/telemetry/telemetry.h> // telemetry plugin

using namespace std;

// ------------------------------------------------------------------------------
//   Data structures
// ------------------------------------------------------------------------------

// Package the individual arrays into a struct
struct Data_Buffer {
    std::vector<uint64_t> time_boot_ms; /*< [ms] Common Timestamp (time since system boot) used after interpolation.*/

    // Body coordinates
    std::vector<uint64_t> attitude_time_boot_ms;    
    std::vector<float> roll; /*< [rad] Roll angle (-pi..+pi)*/
    std::vector<float> pitch; /*< [rad] Pitch angle (-pi..+pi)*/
    std::vector<float> yaw; /*< [rad] Yaw angle (-pi..+pi)*/

    // Body coordinates
    std::vector<uint64_t> angular_velocity_time_boot_ms;
    std::vector<float> rollspeed; /*< [rad/s] Roll angular speed*/
    std::vector<float> pitchspeed; /*< [rad/s] Pitch angular speed*/
    std::vector<float> yawspeed; /*< [rad/s] Yaw angular speed*/

    // Body coordinates
    std::vector<uint64_t> position_time_boot_ms;
    std::vector<float> x; /*< [m] X Position*/
    std::vector<float> y; /*< [m] Y Position*/
    std::vector<float> z; /*< [m] Z Position*/

    // Body coordinates
    std::vector<uint64_t> velocity_time_boot_ms;
    std::vector<float> lvx; /*< [m/s] X Speed*/
    std::vector<float> lvy; /*< [m/s] Y Speed*/
    std::vector<float> lvz; /*< [m/s] Z Speed*/

    void clear_buffers()
    {
        time_boot_ms.clear();
        attitude_time_boot_ms.clear();
        angular_velocity_time_boot_ms.clear();
        position_time_boot_ms.clear();
        velocity_time_boot_ms.clear();

        roll.clear(); /*< [rad] Roll angle (-pi..+pi)*/
        pitch.clear(); /*< [rad] Pitch angle (-pi..+pi)*/
        yaw.clear(); /*< [rad] Yaw angle (-pi..+pi)*/
        rollspeed.clear(); /*< [rad/s] Roll angular speed*/
        pitchspeed.clear(); /*< [rad/s] Pitch angular speed*/
        yawspeed.clear(); /*< [rad/s] Yaw angular speed*/

        x.clear(); /*< [m] X Position*/
        y.clear(); /*< [m] Y Position*/
        z.clear(); /*< [m] Z Position*/
        lvx.clear(); /*< [m/s] X Speed*/
        lvy.clear(); /*< [m/s] Y Speed*/
        lvz.clear(); /*< [m/s] Z Speed*/
    }

    int find_max_length()
    {
        int max_length = 0;
        if(attitude_time_boot_ms.size() > max_length)
        {
            max_length = attitude_time_boot_ms.size();
        }
        if(angular_velocity_time_boot_ms.size() > max_length)
        {
            max_length = angular_velocity_time_boot_ms.size();
        }
        if(position_time_boot_ms.size() > max_length)
        {
            max_length = position_time_boot_ms.size();
        }
        if(velocity_time_boot_ms.size() > max_length)
        {
            max_length = velocity_time_boot_ms.size();
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
    int clear_time;
    string buffer_mode = "";

    Data_Buffer buffer;
    std::mutex mtx;
    std::condition_variable full;
    std::condition_variable not_full;

public:
    Buffer();
    Buffer(int buffer_length_, string buffer_mode);
    ~Buffer();

    void Buffer::insert(mavsdk::Telemetry::PositionBody, uint64_t timestamp);
    void Buffer::insert(mavsdk::Telemetry::EulerAngle, uint64_t timestamp);
    void Buffer::insert(mavsdk::Telemetry::AngularVelocityBody, uint64_t timestamp);
    void Buffer::insert(mavsdk::Telemetry::VelocityBody, uint64_t timestamp);
    
    Data_Buffer clear();
};

#endif  //Buffer_H_