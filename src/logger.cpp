/**
 * @file logger.h
 *
 * @brief helper functions for logging data buffer to csv 
 *
 * @author Stefan Bichlmaier, <bichlmaier.stef@gmail.com>
 *
 */

#include "logger.h"

void log_buffer_to_csv(Mavlink_Message_Buffers buffer)
{
    int max_length = find_max_length(buffer);

    //create an array of strings, size max length
    //iterate through buffers, appending element + ","
    //append " "+ "," if we have reached the end
    //std::string output[max_length];
    std::ofstream myfile;
    myfile.open ("log.csv");

    string header;
    header += "Attitude_time_ms, Pitch, Pitchspeed, Roll, Rollspeed, Yaw, Yawspeed,";
    header += "Global Position Time ms, Altitude, Heading, Latitude, Longitude, Relative Altitude, Velocity X, Velocity Y, Velocity Z,";
    header += "IMU time us, Acceleration X, Acceleration Y, Gyro X, Gyro Y,";
    header += "Local Position Time ms, Velocity X, Velocity Y, Velocity Z, Position X, Position Y, Position Z\n";
    myfile << header;
    
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
            row += to_string(buffer.buffered_attitude.at(i).time_boot_ms) + ",";
            row += to_string(buffer.buffered_attitude.at(i).pitch) + ",";
            row += to_string(buffer.buffered_attitude.at(i).pitchspeed) + ",";
            row += to_string(buffer.buffered_attitude.at(i).roll) + ",";
            row += to_string(buffer.buffered_attitude.at(i).rollspeed) + ",";
            row += to_string(buffer.buffered_attitude.at(i).yaw) + ",";
            row += to_string(buffer.buffered_attitude.at(i).yawspeed) + ",";
        }

        if(i >= buffer.buffered_global_position_int.size())
        {
            //append empty cells
            row += " , , , , , , , , ,";           
        }
        else
        {
            row += to_string(buffer.buffered_global_position_int.at(i).time_boot_ms) + ",";
            row += to_string(buffer.buffered_global_position_int.at(i).alt) + ",";
            row += to_string(buffer.buffered_global_position_int.at(i).hdg) + ",";
            row += to_string(buffer.buffered_global_position_int.at(i).lat) + ",";
            row += to_string(buffer.buffered_global_position_int.at(i).lon) + ",";
            row += to_string(buffer.buffered_global_position_int.at(i).relative_alt) + ",";
            row += to_string(buffer.buffered_global_position_int.at(i).vx) + ",";
            row += to_string(buffer.buffered_global_position_int.at(i).vy) + ",";
            row += to_string(buffer.buffered_global_position_int.at(i).vz) + ",";
        }

        if(i >= buffer.buffered_highres_imu.size())
        {
            //append empty cells
            row += " , , , , ,";
        }
        else
        {
            row += to_string(buffer.buffered_highres_imu.at(i).time_usec) + ",";
            row += to_string(buffer.buffered_highres_imu.at(i).xacc) + ",";
            row += to_string(buffer.buffered_highres_imu.at(i).yacc) + ",";
            row += to_string(buffer.buffered_highres_imu.at(i).xgyro) + ",";
            row += to_string(buffer.buffered_highres_imu.at(i).ygyro) + ",";
        }

        if(i >= buffer.buffered_local_position_ned.size())
        {
            //append empty cells
            row += " , , , , , , \n";
        }
        else
        {
            row += to_string(buffer.buffered_local_position_ned.at(i).time_boot_ms) + ",";
            row += to_string(buffer.buffered_local_position_ned.at(i).vx) + ",";
            row += to_string(buffer.buffered_local_position_ned.at(i).vy) + ",";
            row += to_string(buffer.buffered_local_position_ned.at(i).vz) + ",";
            row += to_string(buffer.buffered_local_position_ned.at(i).x) + ",";
            row += to_string(buffer.buffered_local_position_ned.at(i).y) + ",";
            row += to_string(buffer.buffered_local_position_ned.at(i).z) + "\n";
        }
        myfile << row;
    }
    myfile.close();
    return;
}

