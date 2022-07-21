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

using namespace std;

void interpolate(Mavlink_Message_Buffers &data);

void align_time_series(std::list<mavlink_local_position_ned_t> &local_position_ned, uint64_t first_sample_time, uint64_t last_sample_time);
void align_time_series(std::list<mavlink_global_position_int_t> &global_position_int, uint64_t first_sample_time, uint64_t last_sample_time);
void align_time_series(std::list<mavlink_highres_imu_t> &highres_imu, uint64_t first_sample_time, uint64_t last_sample_time);
void align_time_series(std::list<mavlink_attitude_t> &attitude, uint64_t first_sample_time, uint64_t last_sample_time);
void align_time_series(std::list<mavlink_actuator_output_status_t> &actuator_output_status, uint64_t first_sample_time, uint64_t last_sample_time);

#endif