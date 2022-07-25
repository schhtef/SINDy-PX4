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

 	uint32_t first_sample_time = data.buffered_actuator_status.front().time_usec/1000; //Convert to ms

	if((data.buffered_attitude.front()).time_boot_ms > first_sample_time)
	{
		first_sample_time = (data.buffered_attitude.front()).time_boot_ms;
	}
	if((data.buffered_global_position_int.front()).time_boot_ms > first_sample_time)
	{
		first_sample_time = (data.buffered_global_position_int.front()).time_boot_ms;
	}
	if((data.buffered_highres_imu.front()).time_usec/1000 > first_sample_time) //Convert to ms
	{
		first_sample_time = (data.buffered_highres_imu.front()).time_usec/1000;
	}
	if((data.buffered_local_position_ned.front()).time_boot_ms > first_sample_time)
	{
		first_sample_time = (data.buffered_local_position_ned.front()).time_boot_ms;
	}

    // Find the buffer with the earliest, last sample.
    // Avoids extrapolating data at the end
 	uint32_t last_sample_time = data.buffered_actuator_status.back().time_usec/1000; //Convert to ms

	if(data.buffered_attitude.back().time_boot_ms < last_sample_time)
	{
		last_sample_time = data.buffered_attitude.back().time_boot_ms;
	}
	if(data.buffered_global_position_int.back().time_boot_ms < last_sample_time)
	{
		last_sample_time = data.buffered_global_position_int.back().time_boot_ms;
	}
	if(data.buffered_highres_imu.back().time_usec/1000 < last_sample_time) //Convert to ms
	{
		last_sample_time = data.buffered_highres_imu.back().time_usec/1000;
	}
	if(data.buffered_local_position_ned.back().time_boot_ms < last_sample_time)
	{
		last_sample_time = data.buffered_local_position_ned.back().time_boot_ms;
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
        attitude_iterator++;
    }

    lerp_vector(uninterpolated_data.roll, uninterpolated_data.time_boot_ms_attitude, interpolated_data.roll, interpolated_data.time_boot_ms_attitude, first_sample_time, last_sample_time, sample_rate);
    interpolated_data.time_boot_ms_attitude.clear();
    lerp_vector(uninterpolated_data.pitch, uninterpolated_data.time_boot_ms_attitude, interpolated_data.pitch, interpolated_data.time_boot_ms_attitude, first_sample_time, last_sample_time, sample_rate);
    interpolated_data.time_boot_ms_attitude.clear();
    lerp_vector(uninterpolated_data.yaw, uninterpolated_data.time_boot_ms_attitude, interpolated_data.yaw, interpolated_data.time_boot_ms_attitude, first_sample_time, last_sample_time, sample_rate);
    interpolated_data.time_boot_ms_attitude.clear();
    lerp_vector(uninterpolated_data.rollspeed, uninterpolated_data.time_boot_ms_attitude, interpolated_data.rollspeed, interpolated_data.time_boot_ms_attitude, first_sample_time, last_sample_time, sample_rate);
    interpolated_data.time_boot_ms_attitude.clear();
    lerp_vector(uninterpolated_data.pitchspeed, uninterpolated_data.time_boot_ms_attitude, interpolated_data.pitchspeed, interpolated_data.time_boot_ms, first_sample_time, last_sample_time, sample_rate);
    interpolated_data.time_boot_ms_attitude.clear();
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
        global_position_iterator++;
    }

    lerp_vector(uninterpolated_data.lat, uninterpolated_data.time_boot_ms_global, interpolated_data.lat, interpolated_data.time_boot_ms_global, first_sample_time, last_sample_time, sample_rate);
    interpolated_data.time_boot_ms_global.clear(); // There needs to be a better way to handle this
    lerp_vector(uninterpolated_data.lon, uninterpolated_data.time_boot_ms_global, interpolated_data.lon, interpolated_data.time_boot_ms_global, first_sample_time, last_sample_time, sample_rate);
    interpolated_data.time_boot_ms_global.clear();
    lerp_vector(uninterpolated_data.alt, uninterpolated_data.time_boot_ms_global, interpolated_data.alt, interpolated_data.time_boot_ms_global, first_sample_time, last_sample_time, sample_rate);
    interpolated_data.time_boot_ms_global.clear();
    lerp_vector(uninterpolated_data.relative_alt, uninterpolated_data.time_boot_ms_global, interpolated_data.relative_alt, interpolated_data.time_boot_ms_global, first_sample_time, last_sample_time, sample_rate);
    interpolated_data.time_boot_ms_global.clear();
    lerp_vector(uninterpolated_data.vx, uninterpolated_data.time_boot_ms_global, interpolated_data.vx, interpolated_data.time_boot_ms_global, first_sample_time, last_sample_time, sample_rate);
    interpolated_data.time_boot_ms_global.clear();
    lerp_vector(uninterpolated_data.vy, uninterpolated_data.time_boot_ms_global, interpolated_data.vy, interpolated_data.time_boot_ms_global, first_sample_time, last_sample_time, sample_rate);
    interpolated_data.time_boot_ms_global.clear();
    lerp_vector(uninterpolated_data.vz, uninterpolated_data.time_boot_ms_global, interpolated_data.vz, interpolated_data.time_boot_ms_global, first_sample_time, last_sample_time, sample_rate);
    interpolated_data.time_boot_ms_global.clear();
    lerp_vector(uninterpolated_data.hdg, uninterpolated_data.time_boot_ms_global, interpolated_data.hdg, interpolated_data.time_boot_ms_global, first_sample_time, last_sample_time, sample_rate);

    return interpolated_data;
}


template <typename T, typename U>
void lerp_vector(std::vector<T> y, std::vector<U> x, std::vector<T> &y_result, std::vector<U> &x_result, U start, U end, uint32_t sample_rate)
{
    if(x.size() != y.size())
    {
        fprintf(stderr, "Caution: X and Y series are different sizes\n");
    }
    // Linearly interpolate local position from first_sample_time, if this vector has the first sample time,
    // first interpolant will just be the first sample
	typename std::vector<T>::iterator y_iterator = y.begin();
	typename std::vector<U>::iterator x_iterator = x.begin();
    y_iterator++;
    x_iterator++;

    double sample_period = 1/(float)sample_rate;
    U interpolant_time = start;
    // Outputs
    //std::vector<T> y_result;
    //std::vector<T> x_result;
    // Loop variables
    T y_interpolant;
    U x_interpolant;

    // Find two consecutive samples which straddle start
    // This ensures interpolation starts between the right samples.
    while(!(*x_iterator >= start && *(x_iterator-1) <= start))
    {
        x_iterator++;
        y_iterator++;
    }

    // Interpolate until we reach the last sample time, or the end of the vector is reached, which is unexpected
    while((y_iterator != y.end()) || (x_iterator != x.end()) || interpolant_time <= end)
    {
        // Interpolate consecutive variables
        x_interpolant = interpolant_time;
        y_interpolant = linear_interp(*(y_iterator-1), (*y_iterator),*(x_iterator-1), (*x_iterator), interpolant_time);

        // Push interpolated value into a new vector
        y_result.push_back(y_interpolant);
        x_result.push_back(x_interpolant);

        // Increment interpolant time
        interpolant_time = interpolant_time+(sample_period*1000);
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
    int x_size = x_result.size();
    if(x_size != ((end-start)/1000)*sample_rate)
    {
        fprintf(stderr, "Caution: size of interpolated vector is different than expected. Expected: %u, Actual %d\n", ((end-start)/((U)1000)*sample_rate, x_size));
    }
}

template <typename T, typename U>
T linear_interp(T y0, T y1, U x0, U x1, U x)
{
    // Returns y0*(x1-x)/(x1-x0) + y1*(x-x0)/(x1-x0)
    // Type casting with (T) to ensure output is of type T
    T retval = y0*(((float)x1-(float)x)/((float)x1-(float)x0))+y1*(((float)x-(float)x0)/((float)x1-(float)x0));
    return retval;
}