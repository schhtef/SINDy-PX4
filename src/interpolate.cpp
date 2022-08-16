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
Data_Buffer interpolate(Data_Buffer data, int sample_rate)
{
    using namespace arma;

    Data_Buffer interpolated_data;

    // Find first sample This will be the time origin
 	uint32_t first_sample_time = data.attitude_time_boot_ms.front();

	if(data.local_time_boot_ms.front() < first_sample_time)
	{
		first_sample_time = data.local_time_boot_ms.front();
	}
    /*
	if((data.wind_time_boot_ms.front()) < first_sample_time)
	{
		first_sample_time = data.wind_time_boot_ms.front();
	}
    */

    // Find the buffer with the last sample.
    //We will extrapolate to this value
 	uint32_t last_sample_time = data.attitude_time_boot_ms.back();

	if(data.local_time_boot_ms.back() > last_sample_time)
	{
		last_sample_time = data.local_time_boot_ms.back();
	}
    /*
	if((data.wind_time_boot_ms.back()) > last_sample_time)
	{
		last_sample_time = data.wind_time_boot_ms.back();
	}
    */

    // Interpolate each parameter of interest
    lerp_vector(data.roll, data.attitude_time_boot_ms, interpolated_data.roll, interpolated_data.time_boot_ms, first_sample_time, last_sample_time, sample_rate);
    lerp_vector(data.pitch, data.attitude_time_boot_ms, interpolated_data.pitch, interpolated_data.time_boot_ms, first_sample_time, last_sample_time, sample_rate);
    lerp_vector(data.yaw, data.attitude_time_boot_ms, interpolated_data.yaw, interpolated_data.time_boot_ms, first_sample_time, last_sample_time, sample_rate);
    lerp_vector(data.pitchspeed, data.attitude_time_boot_ms, interpolated_data.pitchspeed, interpolated_data.time_boot_ms, first_sample_time, last_sample_time, sample_rate);
    lerp_vector(data.rollspeed, data.attitude_time_boot_ms, interpolated_data.rollspeed, interpolated_data.time_boot_ms, first_sample_time, last_sample_time, sample_rate);
    lerp_vector(data.yawspeed, data.attitude_time_boot_ms, interpolated_data.yawspeed, interpolated_data.time_boot_ms, first_sample_time, last_sample_time, sample_rate);
    lerp_vector(data.lvx, data.local_time_boot_ms, interpolated_data.lvx, interpolated_data.time_boot_ms, first_sample_time, last_sample_time, sample_rate);
    lerp_vector(data.lvy, data.local_time_boot_ms, interpolated_data.lvy, interpolated_data.time_boot_ms, first_sample_time, last_sample_time, sample_rate);
    lerp_vector(data.lvz, data.local_time_boot_ms, interpolated_data.lvz, interpolated_data.time_boot_ms, first_sample_time, last_sample_time, sample_rate);
    //lerp_vector(data.wind_x, data.wind_time_boot_ms, interpolated_data.wind_x, interpolated_data.time_boot_ms, first_sample_time, last_sample_time, sample_rate);
    //lerp_vector(data.wind_y, data.wind_time_boot_ms, interpolated_data.wind_y, interpolated_data.time_boot_ms, first_sample_time, last_sample_time, sample_rate);
    //lerp_vector(data.wind_z, data.wind_time_boot_ms, interpolated_data.wind_z, interpolated_data.time_boot_ms, first_sample_time, last_sample_time, sample_rate);

    return interpolated_data;
}

template <typename T, typename U>
void lerp_vector(std::vector<T> y, std::vector<U> x, std::vector<T> &y_result, std::vector<U> &x_result, U start, U end, int sample_rate)
{
    if(x.size() != y.size())
    {
        fprintf(stderr, "Caution: X and Y series are different sizes\n");
    }
    //Since we are interpolating to a common time series, the result should be reset until the very last interpolation
    if(x_result.size() != 0)
    {
        x_result.clear();
    }
    // Linearly interpolate local position from first_sample_time, if this vector has the first sample time,
    // first interpolant will just be the first sample
	typename std::vector<T>::iterator y_iterator = y.begin();
	typename std::vector<U>::iterator x_iterator = x.begin();

    float sample_period = 1/(float)sample_rate;
    U interpolant_time = start;
    // Outputs
    //std::vector<T> y_result;
    //std::vector<T> x_result;
    // Loop variables
    T y_interpolant;
    U x_interpolant;

    // Interpolate until we reach the last sample time, or the end of the vector is reached, which is unexpected
    while(interpolant_time <= end)
    {
        // Interpolate consecutive variables
        x_interpolant = interpolant_time;
        y_interpolant = linear_interp(*(y_iterator), (*y_iterator+1),*(x_iterator), (*x_iterator+1), interpolant_time);

        // Push interpolated value into a new vector
        y_result.push_back(y_interpolant);
        x_result.push_back(x_interpolant);

        // Increment interpolant time
        interpolant_time = interpolant_time+(sample_period*1000);

        // If interpolant time is greater than the next sample, increment the iterator
        // If the interpolant time is equal, the next loop will just result in the true sample, no interpolation
        // This covers the extrapolation case a the beginning and end of the vector
        // If the vector's first sample is after the start time, interpolant_time will keep increasing.
        // If the vector's last sample is before the end time, interpolant_time will keep increasing but the iterators will not
        if((interpolant_time > (*x_iterator)) && x_iterator != x.end())
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

    assert(x_result.size() == ((end-start))*sample_rate);
}

template <typename T, typename U>
T linear_interp(T y0, T y1, U x0, U x1, U x)
{
    // Returns y0*(x1-x)/(x1-x0) + y1*(x-x0)/(x1-x0)
    // Type casting with (T) to ensure output is of type T
    T retval = y0*(((float)x1-(float)x)/((float)x1-(float)x0))+y1*(((float)x-(float)x0)/((float)x1-(float)x0));
    return retval;
}