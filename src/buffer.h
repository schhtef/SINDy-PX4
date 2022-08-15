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

    std::vector<uint32_t> global_time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
    std::vector<int32_t> lat; /*< [degE7] Latitude, expressed*/
    std::vector<int32_t> lon; /*< [degE7] Longitude, expressed*/
    std::vector<int32_t> alt; /*< [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.*/
    std::vector<int32_t> relative_alt; /*< [mm] Altitude above ground*/
    std::vector<int16_t> gvx; /*< [cm/s] Ground X Speed (Latitude, positive north)*/
    std::vector<int16_t> gvy; /*< [cm/s] Ground Y Speed (Longitude, positive east)*/
    std::vector<int16_t> gvz; /*< [cm/s] Ground Z Speed (Altitude, positive down)*/
    std::vector<uint16_t> hdg; /*< [cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/

    std::vector<uint64_t> imu_time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
    std::vector<float> xacc; /*< [m/s/s] X acceleration*/
    std::vector<float> yacc; /*< [m/s/s] Y acceleration*/
    std::vector<float> zacc; /*< [m/s/s] Z acceleration*/
    std::vector<float> xgyro; /*< [rad/s] Angular speed around X axis*/
    std::vector<float> ygyro; /*< [rad/s] Angular speed around Y axis*/
    std::vector<float> zgyro; /*< [rad/s] Angular speed around Z axis*/
    std::vector<float> xmag; /*< [gauss] X Magnetic field*/
    std::vector<float> ymag; /*< [gauss] Y Magnetic field*/
    std::vector<float> zmag; /*< [gauss] Z Magnetic field*/
    std::vector<float> abs_pressure; /*< [hPa] Absolute pressure*/
    std::vector<float> diff_pressure; /*< [hPa] Differential pressure*/
    std::vector<float> pressure_alt; /*<  Altitude calculated from pressure*/
    std::vector<float> temperature; /*< [degC] Temperature*/
    std::vector<uint16_t> fields_updated; /*<  Bitmap for fields that have updated since last message*/
    std::vector<uint8_t> id; /*<  Id. Ids are numbered from 0 and map to IMUs numbered from 1 (e.g. IMU1 will have a message with id=0)*/

    std::vector<uint32_t> local_time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
    std::vector<float> x; /*< [m] X Position*/
    std::vector<float> y; /*< [m] Y Position*/
    std::vector<float> z; /*< [m] Z Position*/
    std::vector<float> lvx; /*< [m/s] X Speed*/
    std::vector<float> lvy; /*< [m/s] Y Speed*/
    std::vector<float> lvz; /*< [m/s] Z Speed*/

    void clear_buffers()
    {
        attitude_time_boot_ms.clear(); /*< [ms] Timestamp (time since system boot).*/
        roll.clear(); /*< [rad] Roll angle (-pi..+pi)*/
        pitch.clear(); /*< [rad] Pitch angle (-pi..+pi)*/
        yaw.clear(); /*< [rad] Yaw angle (-pi..+pi)*/
        rollspeed.clear(); /*< [rad/s] Roll angular speed*/
        pitchspeed.clear(); /*< [rad/s] Pitch angular speed*/
        yawspeed.clear(); /*< [rad/s] Yaw angular speed*/

        global_time_boot_ms.clear(); /*< [ms] Timestamp (time since system boot).*/
        lat.clear(); /*< [degE7] Latitude, expressed*/
        lon.clear(); /*< [degE7] Longitude, expressed*/
        alt.clear(); /*< [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.*/
        relative_alt.clear(); /*< [mm] Altitude above ground*/
        gvx.clear(); /*< [cm/s] Ground X Speed (Latitude, positive north)*/
        gvy.clear(); /*< [cm/s] Ground Y Speed (Longitude, positive east)*/
        gvz.clear(); /*< [cm/s] Ground Z Speed (Altitude, positive down)*/
        hdg.clear(); /*< [cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/

        imu_time_usec.clear(); /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
        xacc.clear(); /*< [m/s/s] X acceleration*/
        yacc.clear(); /*< [m/s/s] Y acceleration*/
        zacc.clear(); /*< [m/s/s] Z acceleration*/
        xgyro.clear(); /*< [rad/s] Angular speed around X axis*/
        ygyro.clear(); /*< [rad/s] Angular speed around Y axis*/
        zgyro.clear(); /*< [rad/s] Angular speed around Z axis*/
        xmag.clear(); /*< [gauss] X Magnetic field*/
        ymag.clear(); /*< [gauss] Y Magnetic field*/
        zmag.clear(); /*< [gauss] Z Magnetic field*/
        abs_pressure.clear(); /*< [hPa] Absolute pressure*/
        diff_pressure.clear(); /*< [hPa] Differential pressure*/
        pressure_alt.clear(); /*<  Altitude calculated from pressure*/
        temperature.clear(); /*< [degC] Temperature*/
        fields_updated.clear(); /*<  Bitmap for fields that have updated since last message*/
        id.clear(); /*<  Id. Ids are numbered from 0 and map to IMUs numbered from 1 (e.g. IMU1 will have a message with id=0)*/

        local_time_boot_ms.clear(); /*< [ms] Timestamp (time since system boot).*/
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
        if(local_time_boot_ms.size() > max_length)
        {
            max_length = local_time_boot_ms.size();
        }
        if(global_time_boot_ms.size() > max_length)
        {
            max_length = global_time_boot_ms.size();
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