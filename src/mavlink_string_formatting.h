/**
 * @file mavlink_string_formatting.h
 *
 * @brief helper functions for string formatting of mavlink message types 
 *
 * @author Stefan Bichlmaier, <bichlmaier.stef@gmail.com>
 *
 */

#ifndef mavlink_string_formatting_H_
#define mavlink_string_formatting_H_

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------
#include "c_library_v2/common/mavlink.h"

using namespace std;

void mavlink_msg_format_local_position_ned(mavlink_local_position_ned_t data);
void mavlink_msg_format_global_position_int(mavlink_global_position_int_t data);
void mavlink_msg_format_highres_imu(mavlink_highres_imu_t data);
void mavlink_msg_format_attitude(mavlink_attitude_t data);
void mavlink_msg_format_actuator_status(mavlink_actuator_output_status_t data);

#endif