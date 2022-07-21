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
    std::list<mavlink_actuator_output_status_t>::iterator it = data.buffered_actuator_status.begin();

	uint64_t first_sample_time = (*it).time_usec/1000; //Convert to ms

	// Use generic pointer to point to the vector with first sample
	void *first_sample_vector = &data.buffered_actuator_status; 

	if((*data.buffered_attitude.begin()).time_boot_ms < first_sample_time)
	{
		first_sample_time = (*data.buffered_attitude.begin()).time_boot_ms;
		first_sample_vector = &data.buffered_attitude;
	}
	if((*data.buffered_global_position_int.begin()).time_boot_ms < first_sample_time)
	{
		first_sample_time = (*data.buffered_global_position_int.begin()).time_boot_ms;
		first_sample_vector = &data.buffered_global_position_int;
	}
	if((*data.buffered_highres_imu.begin()).time_usec/1000 < first_sample_time) //Convert to ms
	{
		first_sample_time = (*data.buffered_highres_imu.begin()).time_usec/1000;
		first_sample_vector = &data.buffered_highres_imu;
	}
	if((*data.buffered_local_position_ned.begin()).time_boot_ms < first_sample_time)
	{
		first_sample_time = (*data.buffered_local_position_ned.begin()).time_boot_ms;
		first_sample_vector = &data.buffered_local_position_ned;
	}

    // Find the buffer with the last collected sample, all buffers will be zero padded to this time
    it = data.buffered_actuator_status.end();
 	uint64_t last_sample_time = (*it).time_usec/1000; //Convert to ms

	if((*data.buffered_attitude.end()).time_boot_ms > last_sample_time)
	{
		last_sample_time = (*data.buffered_attitude.end()).time_boot_ms;
		first_sample_vector = &data.buffered_attitude;
	}
	if((*data.buffered_global_position_int.end()).time_boot_ms > last_sample_time)
	{
		last_sample_time = (*data.buffered_global_position_int.end()).time_boot_ms;
		first_sample_vector = &data.buffered_global_position_int;
	}
	if((*data.buffered_highres_imu.end()).time_usec/1000 > last_sample_time) //Convert to ms
	{
		last_sample_time = (*data.buffered_highres_imu.end()).time_usec/1000;
		first_sample_vector = &data.buffered_highres_imu;
	}
	if((*data.buffered_local_position_ned.end()).time_boot_ms > last_sample_time)
	{
		last_sample_time = (*data.buffered_local_position_ned.end()).time_boot_ms;
		first_sample_vector = &data.buffered_local_position_ned;
	}

	// For each vector, find difference in time between consecutive samples
	// Calculate the number of zeros to add in, (time in ms)/(time resolution in ms)
	// Pad with zeros
	align_time_series(data.buffered_actuator_status, first_sample_time, last_sample_time);
    align_time_series(data.buffered_local_position_ned, first_sample_time, last_sample_time);
    align_time_series(data.buffered_global_position_int, first_sample_time, last_sample_time);
	align_time_series(data.buffered_highres_imu, first_sample_time, last_sample_time);
	align_time_series(data.buffered_attitude, first_sample_time, last_sample_time);
}

void align_time_series(std::list<mavlink_actuator_output_status_t> &actuator_output_status, uint64_t first_sample_time, uint64_t last_sample_time)
{
	std::list<mavlink_actuator_output_status_t>::iterator it = actuator_output_status.end();
    mavlink_actuator_output_status_t empty_actuator_status;
    init_actuator_status(empty_actuator_status);

    // Add zeros between samples from end of list to the beginning
	while(it != actuator_output_status.begin())
	{
		// Currently spacing is 1ms, divide result by desired timebase
		uint64_t number_of_interpolants = ((*it).time_usec/1000 - (*(it--)).time_usec/1000); 
		// insert number of interpolants into the list
        empty_actuator_status.time_usec = ((*it).time_usec)-1000; //Inserting behind current element, so previous ms is needed 
        actuator_output_status.insert(it, number_of_interpolants, empty_actuator_status);
	}

    // Extend true beginning of list to the common time origin, if this list is the time origin, number of interpolants is 0 
    uint64_t number_of_interpolants = ((*it).time_usec/1000 - first_sample_time); 
    // insert number of interpolants into the list
    for(int i = 0; i < number_of_interpolants; i++)
    {
        empty_actuator_status.time_usec = ((*it).time_usec)-1000*i; //Inserting behind current element, so previous ms is needed
        actuator_output_status.insert(it, empty_actuator_status);
    }
 
    // Extend true end of list to the common time completion
    it = actuator_output_status.end();
    number_of_interpolants = (last_sample_time - (*it).time_usec/1000);
    for(int i = 0; i < number_of_interpolants; i++)
    {
        empty_actuator_status.time_usec = ((*it).time_usec)+1000*i; //Inserting ahead of current element, so next ms is needed
        actuator_output_status.push_back(empty_actuator_status);
    }
}

void align_time_series(std::list<mavlink_local_position_ned_t> &local_position_ned, uint64_t first_sample_time, uint64_t last_sample_time)
{
	std::list<mavlink_local_position_ned_t>::iterator it = local_position_ned.end();
    mavlink_local_position_ned_t empty_local_position;
    init_local_position_ned(empty_local_position);

    //Add zeros from end of list to the beginning
	while(it != local_position_ned.begin())
	{
		//Currently spacing is 1ms, divide result by desired timebase
		uint64_t number_of_interpolants = ((*it).time_boot_ms - (*(it--)).time_boot_ms); 
		//insert number of interpolants into the list
        empty_local_position.time_boot_ms = ((*it).time_boot_ms)-1; //Inserting behind current element, so previous ms is needed 
        local_position_ned.insert(it, number_of_interpolants, empty_local_position);
	}
    //Extend true beginning of list to the common time origin, if this list is the time origin, number of interpolants is 0 
    uint64_t number_of_interpolants = ((*it).time_boot_ms - first_sample_time); 
    //insert number of interpolants into the list
    empty_local_position.time_boot_ms = ((*it).time_boot_ms)-1; //Inserting behind current element, so previous ms is needed 
    local_position_ned.insert(it, number_of_interpolants, empty_local_position);
}

void align_time_series(std::list<mavlink_global_position_int_t> &global_position_int, uint64_t first_sample_time, uint64_t last_sample_time)
{
	std::list<mavlink_global_position_int_t>::iterator it = global_position_int.end();
    mavlink_global_position_int_t empty_global_position;
    init_global_position_ned(empty_global_position);
    //Add zeros from end of list to the beginning
	while(it != global_position_int.begin())
	{
		//Currently spacing is 1ms, divide result by desired timebase
		uint64_t number_of_interpolants = ((*it).time_boot_ms - (*(it--)).time_boot_ms); 
		//insert number of interpolants into the list
        empty_global_position.time_boot_ms = ((*it).time_boot_ms)-1; //Inserting behind current element, so previous ms is needed 
        global_position_int.insert(it, number_of_interpolants, empty_global_position);
	}
    //Extend true beginning of list to the common time origin, if this list is the time origin, number of interpolants is 0 
    uint64_t number_of_interpolants = ((*it).time_boot_ms - first_sample_time); 
    //insert number of interpolants into the list
    empty_global_position.time_boot_ms = ((*it).time_boot_ms)-1; //Inserting behind current element, so previous ms is needed 
    global_position_int.insert(it, number_of_interpolants, empty_global_position);
}

void align_time_series(std::list<mavlink_highres_imu_t> &highres_imu, uint64_t first_sample_time, uint64_t last_sample_time)
{
	std::list<mavlink_highres_imu_t>::iterator it = highres_imu.end();
    mavlink_highres_imu_t empty_highres_imu;

    //Add zeros from end of list to the beginning
	while(it != highres_imu.begin())
	{
		//Currently spacing is 1ms, divide result by desired timebase
		uint64_t number_of_interpolants = ((*it).time_usec/1000 - (*(it--)).time_usec/1000); 
		//insert number of interpolants into the list
        empty_highres_imu.time_usec = ((*it).time_usec)-1000; //Inserting behind current element, so previous ms is needed 
        highres_imu.insert(it, number_of_interpolants, empty_highres_imu);
	}
    //Extend true beginning of list to the common time origin, if this list is the time origin, number of interpolants is 0 
    uint64_t number_of_interpolants = ((*it).time_usec/1000 - first_sample_time); 
    //insert number of interpolants into the list
    empty_highres_imu.time_usec = ((*it).time_usec)-1000; //Inserting behind current element, so previous ms is needed 
    highres_imu.insert(it, number_of_interpolants, empty_highres_imu);
}

void align_time_series(std::list<mavlink_attitude_t> &attitude, uint64_t first_sample_time, uint64_t last_sample_time)
{
	std::list<mavlink_attitude_t>::iterator it = attitude.end();
    mavlink_attitude_t empty_attitude;

    //Add zeros from end of list to the beginning
	while(it != attitude.begin())
	{
		//Currently spacing is 1ms, divide result by desired timebase
		uint64_t number_of_interpolants = ((*it).time_boot_ms - (*(it--)).time_boot_ms); 
		//insert number of interpolants into the list
        empty_attitude.time_boot_ms = ((*it).time_boot_ms)-1; //Inserting behind current element, so previous ms is needed 
        attitude.insert(it, number_of_interpolants, empty_attitude);
	}
    //Extend true beginning of list to the common time origin, if this list is the time origin, number of interpolants is 0 
    uint64_t number_of_interpolants = ((*it).time_boot_ms - first_sample_time); 
    //insert number of interpolants into the list
    empty_attitude.time_boot_ms = ((*it).time_boot_ms)-1; //Inserting behind current element, so previous ms is needed 
    attitude.insert(it, number_of_interpolants, empty_attitude);
}