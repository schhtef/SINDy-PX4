/**
 * @file interpolate.cpp
 *
 * @brief interpolation functions
 *
 * Uses arma's linear interpolation function to lerp a Data_Buffer type
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
// Interpolates the data buffer and performs state transformations
Vehicle_States linear_interpolate(Data_Buffer data, int sample_rate)
{
	//Perform the coordinate conversions to obtain the desired states

	//Euler angles
	arma::rowvec psi = arma::conv_to<arma::rowvec>::from(data.roll);
	arma::rowvec theta = arma::conv_to<arma::rowvec>::from(data.pitch);
	arma::rowvec phi = arma::conv_to<arma::rowvec>::from(data.yaw);
	arma::rowvec attitude_time_ms = arma::conv_to<arma::rowvec>::from(data.attitude_time_boot_ms);

	//Angular Velocities
	arma::rowvec p = arma::conv_to<arma::rowvec>::from(data.rollspeed);
	arma::rowvec q = arma::conv_to<arma::rowvec>::from(data.pitchspeed);
	arma::rowvec r = arma::conv_to<arma::rowvec>::from(data.yawspeed);
	arma::rowvec angular_velocity_time_boot_ms = arma::conv_to<arma::rowvec>::from(data.angular_velocity_time_boot_ms);

	//Linear Velocities
	arma::rowvec x_m_s = arma::conv_to<arma::rowvec>::from(data.x_m_s);
	arma::rowvec y_m_s = arma::conv_to<arma::rowvec>::from(data.y_m_s);
	arma::rowvec z_m_s = arma::conv_to<arma::rowvec>::from(data.z_m_s);

	//Linear Positions
	arma::rowvec x = arma::conv_to<arma::rowvec>::from(data.x);
	arma::rowvec y = arma::conv_to<arma::rowvec>::from(data.y);
	arma::rowvec z = arma::conv_to<arma::rowvec>::from(data.z);
	arma::rowvec position_time_boot_ms = arma::conv_to<arma::rowvec>::from(data.position_time_boot_ms);

	// Find latest first sample sample time, this will be the time origin
	// Using latest so no extrapolation occurs
 	uint64_t first_sample_time = data.attitude_time_boot_ms.front();

	if(data.angular_velocity_time_boot_ms.front() > first_sample_time)
	{
		first_sample_time = data.angular_velocity_time_boot_ms.front();
	}

	if((data.position_time_boot_ms.front()) > first_sample_time)
	{
		first_sample_time = data.position_time_boot_ms.front();
	}

    // Find the buffer with the earliest last sample to avoid extrapolation
 	uint64_t last_sample_time = data.attitude_time_boot_ms.back();

	if(data.angular_velocity_time_boot_ms.back() < last_sample_time)
	{
		last_sample_time = data.angular_velocity_time_boot_ms.back();
	}

	if((data.position_time_boot_ms.back()) < last_sample_time)
	{
		last_sample_time = data.position_time_boot_ms.back();
	}

	int number_of_samples = (last_sample_time-first_sample_time)*(sample_rate)/1000;
	
	// Generate a common time base
	arma::rowvec time_ms = arma::linspace<arma::rowvec>(first_sample_time, last_sample_time, number_of_samples);

	arma::rowvec psi_interp(number_of_samples);
	arma::rowvec theta_interp(number_of_samples);
	arma::rowvec phi_interp(number_of_samples);
	arma::rowvec p_interp(number_of_samples);
	arma::rowvec q_interp(number_of_samples);
	arma::rowvec r_interp(number_of_samples);
	arma::rowvec lvx_interp(number_of_samples);
	arma::rowvec lvy_interp(number_of_samples);
	arma::rowvec lvz_interp(number_of_samples);
	arma::rowvec x_interp(number_of_samples);
	arma::rowvec y_interp(number_of_samples);
	arma::rowvec z_interp(number_of_samples);

	arma::interp1(attitude_time_ms, psi, time_ms, psi_interp);
	arma::interp1(attitude_time_ms, theta, time_ms, theta_interp);
	arma::interp1(attitude_time_ms, phi, time_ms, phi_interp);
	arma::interp1(angular_velocity_time_boot_ms, p, time_ms, p_interp);
	arma::interp1(angular_velocity_time_boot_ms, q, time_ms, q_interp);
	arma::interp1(angular_velocity_time_boot_ms, r, time_ms, r_interp);
	arma::interp1(position_time_boot_ms, x_m_s, time_ms, lvx_interp);
	arma::interp1(position_time_boot_ms, y_m_s, time_ms, lvy_interp);
	arma::interp1(position_time_boot_ms, z_m_s, time_ms, lvz_interp);
	arma::interp1(position_time_boot_ms, x, time_ms, x_interp);
	arma::interp1(position_time_boot_ms, y, time_ms, y_interp);
	arma::interp1(position_time_boot_ms, z, time_ms, z_interp);
	
	Vehicle_States state_buffer;

	state_buffer.p = p_interp;
	state_buffer.q = q_interp;
	state_buffer.r = r_interp;
	state_buffer.psi = psi_interp;
	state_buffer.theta = theta_interp;
	state_buffer.phi = phi_interp;
	state_buffer.u = lvx_interp;
	state_buffer.v = lvy_interp;
	state_buffer.w = lvz_interp;
	state_buffer.x = x_interp;
	state_buffer.y = y_interp;
	state_buffer.z = z_interp;
	state_buffer.num_samples = number_of_samples;

	return state_buffer;
}