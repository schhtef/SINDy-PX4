/**
 * @file logger.h
 *
 * @brief helper functions for logging data buffer to csv 
 *
 * @author Stefan Bichlmaier, <bichlmaier.stef@gmail.com>
 *
 */

#include "logging.h"

void log_buffer_to_csv(Data_Buffer telemetry, std::string filename){
/*
    using namespace std;
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
    
    string row;

    for(int i = 0; i < telemetry.alt.size(); i++)
    {
        row += to_string(telemetry.time_boot_ms[i]) + ",";

        row += to_string(telemetry.pitch[i]) + ",";
        row += to_string(telemetry.pitchspeed[i]) + ",";
        row += to_string(telemetry.roll[i]) + ",";
        row += to_string(telemetry.rollspeed[i]) + ",";
        row += to_string(telemetry.yaw[i]) + ",";
        row += to_string(telemetry.yawspeed[i]) + ",";

        //row += to_string(telemetry.time_boot_ms_global[i]) + ",";
        row += to_string(telemetry.alt[i]) + ",";
        row += to_string(telemetry.hdg[i]) + ",";
        row += to_string(telemetry.lat[i]) + ",";
        row += to_string(telemetry.lon[i]) + ",";
        row += to_string(telemetry.relative_alt[i]) + ",";
        row += to_string(telemetry.vx[i]) + ",";
        row += to_string(telemetry.vy[i]) + ",";
        row += to_string(telemetry.vz[i]) + "\n";

        row += to_string((*highres_imu_iterator).time_usec) + ",";
        row += to_string((*highres_imu_iterator).xacc) + ",";
        row += to_string((*highres_imu_iterator).yacc) + ",";
        row += to_string((*highres_imu_iterator).xgyro) + ",";
        row += to_string((*highres_imu_iterator).ygyro) + ",";
        highres_imu_iterator++;


        row += to_string((*local_position_iterator).time_boot_ms) + ",";
        row += to_string((*local_position_iterator).vx) + ",";
        row += to_string((*local_position_iterator).vy) + ",";
        row += to_string((*local_position_iterator).vz) + ",";
        row += to_string((*local_position_iterator).x) + ",";
        row += to_string((*local_position_iterator).y) + ",";
        row += to_string((*local_position_iterator).z) + "\n";
        local_position_iterator++;

        myfile << row;
        row = "";
    }
    myfile.close();
    return;
*/
}

void log_mavlink_info(mavsdk::log::Level level, const std::string& message, const std::string &filename){
	using namespace std;
	ofstream myfile;
    myfile.open (filename, ios_base::app);
	myfile << static_cast<int>(level) << ',' << message << '\n';
	myfile.close();
}

void log_coeff(arma::mat matrix, std::string filename, std::chrono::microseconds sample_time)
{
	using namespace std;
	ofstream myfile;
    myfile.open (filename, ios_base::app);
	arma::rowvec vectorized_matrix = vectorise(matrix, 1); //row-wise vectorization of the coefficient matrix for writing to csv
	
	arma::rowvec::iterator coefficient_iterator = vectorized_matrix.begin();
	myfile << sample_time.count() << ",";
	for(; coefficient_iterator != vectorized_matrix.end(); ++coefficient_iterator)
	{
		myfile << (*coefficient_iterator) << ",";
	}
	myfile << "\n";
	myfile.close();
}