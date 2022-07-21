/**
 * @file logger.h
 *
 * @brief helper functions for logging data buffer to csv 
 *
 * @author Stefan Bichlmaier, <bichlmaier.stef@gmail.com>
 *
 */

#include "logger.h"

void log_buffer_to_csv(Mavlink_Message_Buffers buffer, string filename)
{
    int max_length = find_max_length(buffer);

    //create an array of strings, size max length
    //iterate through buffers, appending element + ","
    //append " "+ "," if we have reached the end
    std::ofstream myfile;
    myfile.open (filename);

    string header;
    header += "Attitude_time_ms, Pitch, Pitchspeed, Roll, Rollspeed, Yaw, Yawspeed,";
    header += "Global Position Time ms, Altitude, Heading, Latitude, Longitude, Relative Altitude, Velocity X, Velocity Y, Velocity Z,";
    header += "IMU time us, Acceleration X, Acceleration Y, Gyro X, Gyro Y,";
    header += "Local Position Time ms, Velocity X, Velocity Y, Velocity Z, Position X, Position Y, Position Z";
    header += "Actuator Output Status Time us, Active actuators\n";
    myfile << header;

    std::list<mavlink_local_position_ned_t>::iterator local_position_iterator = buffer.buffered_local_position_ned.begin();
    std::list<mavlink_global_position_int_t>::iterator global_position_iterator = buffer.buffered_global_position_int.begin();
    std::list<mavlink_highres_imu_t>::iterator imu_iterator = buffer.buffered_highres_imu.begin();
    std::list<mavlink_attitude_t>::iterator attitude_iterator = buffer.buffered_attitude.begin();
    std::list<mavlink_actuator_output_status_t>::iterator actuator_iterator = buffer.buffered_actuator_status.begin();
/*
    while(local_position_iterator != buffer.buffered_local_position_ned.end())
    {

    }

    for(int i = 0; i < max_length; i++)
    {
        string row;
        if(i >= buffer.buffered_attitude.size())
        {
            //append empty cells
            row += " , , , , , , ,";
        }
        else
        {
            //append each element to the row
            row += to_string((*buffer.buffered_attitude.begin()).time_boot_ms) + ",";
            row += to_string((*buffer.buffered_attitude.begin().pitch) + ",";
            row += to_string((*buffer.buffered_attitude.begin()).pitchspeed) + ",";
            row += to_string((*buffer.buffered_attitude.begin()).roll) + ",";
            row += to_string((*buffer.buffered_attitude.begin()).rollspeed) + ",";
            row += to_string((*buffer.buffered_attitude.begin()).yaw) + ",";
            row += to_string((*buffer.buffered_attitude.begin()).yawspeed) + ",";
        }

        if(i >= buffer.buffered_global_position_int.size())
        {
            //append empty cells
            row += " , , , , , , , , ,";           
        }
        else
        {
            row += to_string((*buffer.buffered_global_position_int.begin()).time_boot_ms) + ",";
            row += to_string((*buffer.buffered_global_position_int.begin()).alt) + ",";
            row += to_string((*buffer.buffered_global_position_int.begin()).hdg) + ",";
            row += to_string((*buffer.buffered_global_position_int.begin()).lat) + ",";
            row += to_string((*buffer.buffered_global_position_int.begin()).lon) + ",";
            row += to_string((*buffer.buffered_global_position_int.begin()).relative_alt) + ",";
            row += to_string((*buffer.buffered_global_position_int.begin()).vx) + ",";
            row += to_string((*buffer.buffered_global_position_int.begin()).vy) + ",";
            row += to_string((*buffer.buffered_global_position_int.begin()).vz) + ",";
        }

        if(i >= buffer.buffered_highres_imu.size())
        {
            //append empty cells
            row += " , , , , ,";
        }
        else
        {
            row += to_string((*buffer.buffered_highres_imu.begin()).time_usec) + ",";
            row += to_string((*buffer.buffered_highres_imu.begin()).xacc) + ",";
            row += to_string((*buffer.buffered_highres_imu.begin()).yacc) + ",";
            row += to_string((*buffer.buffered_highres_imu.begin()).xgyro) + ",";
            row += to_string((*buffer.buffered_highres_imu.begin()).ygyro) + ",";
        }

        if(i >= buffer.buffered_local_position_ned.size())
        {
            //append empty cells
            row += " , , , , , , \n";
        }
        else
        {
            row += to_string((*buffer.buffered_local_position_ned.begin()).time_boot_ms) + ",";
            row += to_string((*buffer.buffered_local_position_ned.begin()).vx) + ",";
            row += to_string((*buffer.buffered_local_position_ned.begin()).vy) + ",";
            row += to_string((*buffer.buffered_local_position_ned.begin()).vz) + ",";
            row += to_string((*buffer.buffered_local_position_ned.begin()).x) + ",";
            row += to_string((*buffer.buffered_local_position_ned.begin()).y) + ",";
            row += to_string((*buffer.buffered_local_position_ned.begin()).z) + "\n";
        }

        if(i >= buffer.buffered_actuator_status.size())
        {
            //append empty cells
            row += " , , , , , , \n";
        }
        else
        {
            row += to_string((*buffer.buffered_actuator_status.begin()).time_usec) + ",";
            row += to_string((*buffer.buffered_actuator_status.begin()).active) + ",";
            // TODO: Add logic to determine which actuators to log
        }
        myfile << row;
    }
    */
    myfile.close();
    return;
}