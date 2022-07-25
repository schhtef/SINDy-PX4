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
 * @param sample_rate rate for resampling data buffer
 * @return void
 */
Telemetry interpolate(Mavlink_Message_Buffers &data, uint32_t sample_rate)
{     
    // Find latest first sample, remember the time. This will be the time origin
    // Avoids extrapolating data at the beginning
    std::vector<mavlink_actuator_output_status_t>::iterator it = data.buffered_actuator_status.begin();

	uint32_t first_sample_time = (*it).time_usec/1000; //Convert to ms

	if((*data.buffered_attitude.begin()).time_boot_ms > first_sample_time)
	{
		first_sample_time = (*data.buffered_attitude.begin()).time_boot_ms;
	}
	if((*data.buffered_global_position_int.begin()).time_boot_ms > first_sample_time)
	{
		first_sample_time = (*data.buffered_global_position_int.begin()).time_boot_ms;
	}
	if((*data.buffered_highres_imu.begin()).time_usec/1000 > first_sample_time) //Convert to ms
	{
		first_sample_time = (*data.buffered_highres_imu.begin()).time_usec/1000;
	}
	if((*data.buffered_local_position_ned.begin()).time_boot_ms > first_sample_time)
	{
		first_sample_time = (*data.buffered_local_position_ned.begin()).time_boot_ms;
	}

    // Find the buffer with the earliest, last sample.
    // Avoids extrapolating data at the end
    it = data.buffered_actuator_status.end();
 	uint32_t last_sample_time = (*it).time_usec/1000; //Convert to ms

	if((*data.buffered_attitude.end()).time_boot_ms < last_sample_time)
	{
		last_sample_time = (*data.buffered_attitude.end()).time_boot_ms;
	}
	if((*data.buffered_global_position_int.end()).time_boot_ms < last_sample_time)
	{
		last_sample_time = (*data.buffered_global_position_int.end()).time_boot_ms;
	}
	if((*data.buffered_highres_imu.end()).time_usec/1000 < last_sample_time) //Convert to ms
	{
		last_sample_time = (*data.buffered_highres_imu.end()).time_usec/1000;
	}
	if((*data.buffered_local_position_ned.end()).time_boot_ms < last_sample_time)
	{
		last_sample_time = (*data.buffered_local_position_ned.end()).time_boot_ms;
	}

    // Separate vectors of mavlink structs into vectors containing each data source
    std::vector<mavlink_attitude_t>::iterator attitude_iterator = data.buffered_attitude.begin();
    Telemetry uninterpolated_data;
    Telemetry interpolated_data;

    while(attitude_iterator != data.buffered_attitude.end())
    {
        uninterpolated_data.time_boot_ms_attitude.push_back((*attitude_iterator).time_boot_ms);
        uninterpolated_data.roll.push_back((*attitude_iterator).roll);
        uninterpolated_data.pitch.push_back((*attitude_iterator).pitch);
        uninterpolated_data.yaw.push_back((*attitude_iterator).yaw);
        uninterpolated_data.rollspeed.push_back((*attitude_iterator).rollspeed);
        uninterpolated_data.pitchspeed.push_back((*attitude_iterator).pitchspeed);
        uninterpolated_data.yawspeed.push_back((*attitude_iterator).yawspeed);
    }

    lerp_vector(uninterpolated_data.roll, uninterpolated_data.time_boot_ms_attitude, interpolated_data.roll, interpolated_data.time_boot_ms_attitude, first_sample_time, last_sample_time, sample_rate);
    lerp_vector(uninterpolated_data.pitch, uninterpolated_data.time_boot_ms_attitude, interpolated_data.pitch, interpolated_data.time_boot_ms_attitude, first_sample_time, last_sample_time, sample_rate);
    lerp_vector(uninterpolated_data.yaw, uninterpolated_data.time_boot_ms_attitude, interpolated_data.yaw, interpolated_data.time_boot_ms_attitude, first_sample_time, last_sample_time, sample_rate);
    lerp_vector(uninterpolated_data.rollspeed, uninterpolated_data.time_boot_ms_attitude, interpolated_data.rollspeed, interpolated_data.time_boot_ms_attitude, first_sample_time, last_sample_time, sample_rate);
    lerp_vector(uninterpolated_data.pitchspeed, uninterpolated_data.time_boot_ms_attitude, interpolated_data.pitchspeed, interpolated_data.time_boot_ms_attitude, first_sample_time, last_sample_time, sample_rate);
    lerp_vector(uninterpolated_data.yawspeed, uninterpolated_data.time_boot_ms_attitude, interpolated_data.yawspeed, interpolated_data.time_boot_ms_attitude, first_sample_time, last_sample_time, sample_rate);

    // Create vectors from global position
    std::vector<mavlink_global_position_int_t>::iterator global_position_iterator = data.buffered_global_position_int.begin();

    while(global_position_iterator != data.buffered_global_position_int.end())
    {
        uninterpolated_data.time_boot_ms_global.push_back((*global_position_iterator).time_boot_ms);
        uninterpolated_data.lat.push_back((*global_position_iterator).lat);
        uninterpolated_data.lon.push_back((*global_position_iterator).lon);
        uninterpolated_data.alt.push_back((*global_position_iterator).alt);
        uninterpolated_data.relative_alt.push_back((*global_position_iterator).relative_alt);
        uninterpolated_data.vx.push_back((*global_position_iterator).vx);
        uninterpolated_data.vy.push_back((*global_position_iterator).vy);
        uninterpolated_data.vz.push_back((*global_position_iterator).vz);
        uninterpolated_data.hdg.push_back((*global_position_iterator).hdg);
    }

    lerp_vector(uninterpolated_data.lat, uninterpolated_data.time_boot_ms_global, interpolated_data.lat, interpolated_data.time_boot_ms_global, first_sample_time, last_sample_time, sample_rate);
    lerp_vector(uninterpolated_data.lon, uninterpolated_data.time_boot_ms_global, interpolated_data.lon, interpolated_data.time_boot_ms_global, first_sample_time, last_sample_time, sample_rate);
    lerp_vector(uninterpolated_data.alt, uninterpolated_data.time_boot_ms_global, interpolated_data.alt, interpolated_data.time_boot_ms_global, first_sample_time, last_sample_time, sample_rate);
    lerp_vector(uninterpolated_data.relative_alt, uninterpolated_data.time_boot_ms_global, interpolated_data.relative_alt, interpolated_data.time_boot_ms_global, first_sample_time, last_sample_time, sample_rate);
    lerp_vector(uninterpolated_data.vx, uninterpolated_data.time_boot_ms_global, interpolated_data.vx, interpolated_data.time_boot_ms_global, first_sample_time, last_sample_time, sample_rate);
    lerp_vector(uninterpolated_data.vy, uninterpolated_data.time_boot_ms_global, interpolated_data.vy, interpolated_data.time_boot_ms_global, first_sample_time, last_sample_time, sample_rate);
    lerp_vector(uninterpolated_data.vz, uninterpolated_data.time_boot_ms_global, interpolated_data.vz, interpolated_data.time_boot_ms_global, first_sample_time, last_sample_time, sample_rate);
    lerp_vector(uninterpolated_data.hdg, uninterpolated_data.time_boot_ms_global, interpolated_data.hdg, interpolated_data.time_boot_ms_global, first_sample_time, last_sample_time, sample_rate);

    return interpolated_data;
}


template <typename T, typename U>
void lerp_vector(std::vector<T> &y, std::vector<U> &x, std::vector<T> &y_result, std::vector<U> &x_result, U start, U end, uint32_t sample_rate)
{
    if(x.size() != y.size())
    {
        fprintf(stderr, "Caution: X and Y series are different sizes\n");
    }
    // Linearly interpolate local position from first_sample_time, if this vector has the first sample time,
    // first interpolant will just be the first sample
	typename std::vector<T>::iterator y_iterator = y.begin()++;
	typename std::vector<U>::iterator x_iterator = x.begin()++;

    float sample_period = 1/sample_rate;
    U interpolant_time = start;
    // Outputs
    //std::vector<T> y_result;
    //std::vector<T> x_result;
    // Loop variables
    T y_interpolant;
    U x_interpolant;

    // Interpolate until we reach the last sample time, or the end of the vector is reached, which is unexpected
    while((y_iterator != y.end()) || (x_iterator != x.end()) || interpolant_time <= end)
    {
        // Interpolate consecutive variables
        x_interpolant = interpolant_time;
        y_interpolant = linear_interp((*y_iterator--), (*y_iterator),(*x_iterator--), (*x_iterator), interpolant_time);

        // Push interpolated value into a new vector
        y_result.push_back(y_interpolant);
        x_result.push_back(x_interpolant);

        // Increment interpolant time
        interpolant_time = interpolant_time+sample_period;
        // If interpolant time is greater than the next sample, increment the iterator
        // If the interpolant time is equal, the next loop will just result in the true sample, no interpolation
        if(interpolant_time > (*x_iterator))
        {
            y_iterator++;
            x_iterator++;
        }
    }
    // Sanity Checks
    if(x_result.size() != y_result.size())
    {
        fprintf(stderr, "Caution: X and Y results are different sizes\n");
    }
    if(x_result.size() != (start-end)*sample_rate)
    {
        fprintf(stderr, "Caution: size of interpolated vector is different than expected.\n Expected: %d, Actual %d", (start-end)*sample_rate, x.size());
    }
}

template <typename T, typename U>
T linear_interp(T y0, T y1, U x0, U x1, U x)
{
    return y0*((x1-x)/(x1-x0))+y1*((x-x0)/(x1-x0));
}