/**
 * @file interpolate.cpp
 *
 * @brief interpolation functions
 *
 * Performs zero insertion to align time series of different data sources
 * and linear or quadratic interpolation
 * 
 * @author Stefan Bichlmaier, <bichlmaier.stef@gmail.com>
 *
 */

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "interpolate.h"


/**
 * Interpolate the samples in Mavlink Message Buffer
 *
 * @param data Struct containing vectors of mavlink messages
 * @return void
 */
void interpolate(Mavlink_Message_Buffers &data)
{ 
	// Find earliest sample, remember the time. This will be the time origin
	uint64_t first_sample_time = data.buffered_actuator_status.at(0).time_usec/1000; //Convert to ms
	// Use generic pointer to point to the vector with first sample
	void *first_sample_vector = &data.buffered_actuator_status; 

	if(data.buffered_attitude.at(0).time_boot_ms < first_sample_time)
	{
		first_sample_time = data.buffered_attitude.at(0).time_boot_ms;
		first_sample_vector = &data.buffered_attitude;
	}
	if(data.buffered_global_position_int.at(0).time_boot_ms < first_sample_time)
	{
		first_sample_time = data.buffered_global_position_int.at(0).time_boot_ms;
		first_sample_vector = &data.buffered_global_position_int;
	}
	if(data.buffered_highres_imu.at(0).time_usec/1000 < first_sample_time) //Convert to ms
	{
		first_sample_time = data.buffered_highres_imu.at(0).time_usec/1000;
		first_sample_vector = &data.buffered_highres_imu;
	}
	if(data.buffered_local_position_ned.at(0).time_boot_ms < first_sample_time)
	{
		first_sample_time = data.buffered_local_position_ned.at(0).time_boot_ms;
		first_sample_vector = &data.buffered_local_position_ned;
	}
	// For each vector, find difference in time between consecutive samples
	// Calculate the number of zeros to add in, (time in ms)/(time resolution in ms)
	// Pad with zeros
	align_time_series(data.buffered_actuator_status, first_sample_time);
    align_time_series(data.buffered_local_position_ned, first_sample_time);
    align_time_series(data.buffered_global_position_int, first_sample_time);
	align_time_series(data.buffered_highres_imu, first_sample_time);
	align_time_series(data.buffered_attitude, first_sample_time);
}

void align_time_series(std::vector<mavlink_actuator_output_status_t> &actuator_output_status, uint64_t first_sample_time)
{
	std::vector<mavlink_actuator_output_status_t>::iterator it = actuator_output_status.begin();
	while(it != actuator_output_status.end())
	{
		//Currently spacing is 1ms, divide result by desired timebase
		uint64_t number_of_interpolants = ((*it).time_usec/1000 - first_sample_time); 
		//insert number of interpolants into the list
		for(int i = 0; i <= number_of_interpolants; i++)
		{
			mavlink_actuator_output_status_t empty_actuator_status;
			empty_actuator_status.time_usec = ((*it).time_usec)-1000; //Inserting behind current element, so previous ms is needed 
			actuator_output_status.insert(it, empty_actuator_status);
		}
		it = it+number_of_interpolants+1;
	}
}

void align_time_series(std::vector<mavlink_local_position_ned_t> &local_position_ned, uint64_t first_sample_time)
{
    std::vector<mavlink_local_position_ned_t>::iterator it = local_position_ned.begin();
	while(it != local_position_ned.end())
	{
		//Currently spacing is 1ms, divide result by desired timebase
		uint64_t number_of_interpolants = ((*it).time_boot_ms - first_sample_time); 
		//insert number of interpolants into the list
		for(int i = 0; i <= number_of_interpolants; i++)
		{
			mavlink_local_position_ned_t empty_local_position;
			empty_local_position.time_boot_ms = ((*it).time_boot_ms)-1; //Inserting behind current element, so previous ms is needed 
			local_position_ned.insert(it, empty_local_position);
		}
		it = it+number_of_interpolants+1;
	}
}
void align_time_series(std::vector<mavlink_global_position_int_t> &global_position_int, uint64_t first_sample_time)
{
	std::vector<mavlink_global_position_int_t>::iterator it = global_position_int.begin();
	while(it != global_position_int.end())
	{
		//Currently spacing is 1ms, divide result by desired timebase
		uint64_t number_of_interpolants = ((*it).time_boot_ms - first_sample_time); 
		//insert number of interpolants into the list
		for(int i = 0; i <= number_of_interpolants; i++)
		{
			mavlink_global_position_int_t empty_actuator_status;
			empty_actuator_status.time_boot_ms = ((*it).time_boot_ms)-1; //Inserting behind current element, so previous ms is needed 
			global_position_int.insert(it, empty_actuator_status);
		}
		it = it+number_of_interpolants+1;
	}
}
void align_time_series(std::vector<mavlink_highres_imu_t> &highres_imu, uint64_t first_sample_time)
{
	std::vector<mavlink_highres_imu_t>::iterator it = highres_imu.begin();
	while(it != highres_imu.end())
	{
		//Currently spacing is 1ms, divide result by desired timebase
		uint64_t number_of_interpolants = ((*it).time_usec/1000 - first_sample_time); 
		//insert number of interpolants into the list
		for(int i = 0; i <= number_of_interpolants; i++)
		{
			mavlink_highres_imu_t empty_actuator_status;
			empty_actuator_status.time_usec = ((*it).time_usec)-1000; //Inserting behind current element, so previous ms is needed 
			highres_imu.insert(it, empty_actuator_status);
		}
		it = it+number_of_interpolants+1;
	}
}
void align_time_series(std::vector<mavlink_attitude_t> &attitude, uint64_t first_sample_time)
{
	std::vector<mavlink_attitude_t>::iterator it = attitude.begin();
	while(it != attitude.end())
	{
		//Currently spacing is 1ms, divide result by desired timebase
		uint64_t number_of_interpolants = ((*it).time_boot_ms - first_sample_time); 
		//insert number of interpolants into the list
		for(int i = 0; i <= number_of_interpolants; i++)
		{
			mavlink_attitude_t empty_actuator_status;
			empty_actuator_status.time_boot_ms = ((*it).time_boot_ms)-1; //Inserting behind current element, so previous ms is needed 
			attitude.insert(it, empty_actuator_status);
		}
		it = it+number_of_interpolants+1;
	}
}