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

#ifndef INTERPOLATE_H_
#define INTERPOLATE_H_

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------
#include "autopilot_interface.h"
#include "c_library_v2/common/mavlink.h"
#include "buffer.h"

struct Telemetry{
    //TIME
    std::vector<uint32_t> time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
    //TIME

    //ATTITUDE
    std::vector<uint32_t> time_boot_ms_attitude; /*< [ms] Timestamp (time since system boot).*/
    std::vector<float> roll; /*< [rad] Roll angle (-pi..+pi)*/
    std::vector<float> pitch; /*< [rad] Pitch angle (-pi..+pi)*/
    std::vector<float> yaw; /*< [rad] Yaw angle (-pi..+pi)*/
    std::vector<float> rollspeed; /*< [rad/s] Roll angular speed*/
    std::vector<float> pitchspeed; /*< [rad/s] Pitch angular speed*/
    std::vector<float> yawspeed; /*< [rad/s] Yaw angular speed*/
    //ATTITUDE

    //GLOBAL POSITION
    std::vector<uint32_t> time_boot_ms_global; /*< [ms] Timestamp (time since system boot).*/
    std::vector<int32_t> lat; /*< [degE7] Latitude, expressed*/
    std::vector<int32_t> lon; /*< [degE7] Longitude, expressed*/
    std::vector<int32_t> alt; /*< [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.*/
    std::vector<int32_t> relative_alt; /*< [mm] Altitude above ground*/
    std::vector<int16_t> vx; /*< [cm/s] Ground X Speed (Latitude, positive north)*/
    std::vector<int16_t> vy; /*< [cm/s] Ground Y Speed (Longitude, positive east)*/
    std::vector<int16_t> vz; /*< [cm/s] Ground Z Speed (Altitude, positive down)*/
    std::vector<uint16_t> hdg; /*< [cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
    //GLOBAL POSITION

    
};

Telemetry interpolate(Mavlink_Message_Buffers &data, uint32_t sample_rate);

template <typename T, typename U>
void lerp_vector(std::vector<T> &y, std::vector<U> &x, std::vector<T> &y_result, std::vector<U> &x_result, U start, U end, uint32_t sample_rate);

template <typename T, typename U>
T linear_interp(T y0, T y1, U x0, U x1, U x);


#endif