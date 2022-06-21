/**
 * @file mavlink_string_formatting.cpp
 *
 * @brief helper functions for string formatting of mavlink message types 
 *
 * @author Stefan Bichlmaier, <bichlmaier.stef@gmail.com>
 *
 */

#include "mavlink_string_formatting.h"

void mavlink_msg_format_local_position_ned(mavlink_local_position_ned_t data)
{
    printf("LOCAL_POSITION_NED");
    printf("    ap time:     %lu \n", data.time_boot_ms);
    printf("    Local Position  (NED):  X: % f Y: % f Z: % f (m)\n", data.x , data.y , data.z );
    printf("    Local Velocity  (NED):  X: % f Y: % f Z: % f (m/s)\n", data.vx , data.vy , data.vz );
    return;
}

void mavlink_msg_format_highres_imu(mavlink_highres_imu_t data)
{
    printf("HIGHRES_IMU\n");
    printf("    ap time:     %lu \n", data.time_usec);
    printf("    acc  (NED):  % f % f % f (m/s^2)\n", data.xacc , data.yacc , data.zacc );
    printf("    gyro (NED):  % f % f % f (rad/s)\n", data.xgyro, data.ygyro, data.zgyro);
    printf("    mag  (NED):  % f % f % f (Ga)\n"   , data.xmag , data.ymag , data.zmag );
    printf("    baro:        %f (mBar) \n"  , data.abs_pressure);
    printf("    altitude:    %f (m) \n"     , data.pressure_alt);
    printf("    temperature: %f C \n"       , data.temperature );
    return;
}
void mavlink_msg_format_attitude(mavlink_attitude_t data)
{
    return;
}
void mavlink_msg_format_actuator_status(mavlink_actuator_output_status_t data)
{
    return;
}