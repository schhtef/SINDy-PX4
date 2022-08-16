/**
 * @file SID.cpp
 *
 * @brief System Identification Process
 *
 * Performs the SINDy algorithm on the input buffer
 * 
 * @author Stefan Bichlmaier, <bichlmaier.stef@gmail.com>
 *
 */

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "system_identification.h"

void* start_SID_compute_thread(void *args);

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
SID::
SID()
{
}

SID::
SID(Buffer *input_buffer_)
{
    input_buffer = input_buffer_;
}

SID::
~SID()
{
}

void SID::
compute_thread()
{
    compute_status = true;
    while ( ! time_to_exit )
	{
        Data_Buffer data = input_buffer->clear();
		Vehicle_States states = interpolate(data, 200); // Resample input buffer and compute desired states
		arma::mat candidate_functions = compute_candidate_functions(states); //Generate Candidate Functions
		arma::mat derivatives = get_derivatives(states); //Get state derivatives for SINDy
		arma::mat coefficients = STLSQ(derivatives, candidate_functions, STLSQ_threshold, lambda); //Run STLSQ
		//Log Results
		
		// might cause a race condition where the SINDy thread checks disarmed just before the main thread
		// sets the disarmed flag. Worst case scenario the SINDy thread logs one more buffer
		if(!disarmed)
		{
			//log_buffer_to_csv(interpolated_telemetry, filename);
		}
	}
	compute_status = false;

	return;
}

Vehicle_States SID::
interpolate(Data_Buffer data, int sample_rate)
{
	using namespace arma;
	//Perform the coordinate conversions to obtain the desired states

	//Euler angles
	arma::rowvec psi = arma::conv_to<arma::rowvec>::from(data.roll);
	arma::rowvec theta = arma::conv_to<arma::rowvec>::from(data.pitch);
	arma::rowvec phi = arma::conv_to<arma::rowvec>::from(data.yaw);

	//Angular Velocities
	arma::rowvec p = arma::conv_to<arma::rowvec>::from(data.rollspeed);
	arma::rowvec q = arma::conv_to<arma::rowvec>::from(data.pitchspeed);
	arma::rowvec r = arma::conv_to<arma::rowvec>::from(data.yawspeed);

	arma::rowvec attitude_time_ms = arma::conv_to<arma::rowvec>::from(data.attitude_time_boot_ms);

	//Linear Velocities
	arma::rowvec lvx = arma::conv_to<arma::rowvec>::from(data.lvx);
	arma::rowvec lvy = arma::conv_to<arma::rowvec>::from(data.lvy);
	arma::rowvec lvz = arma::conv_to<arma::rowvec>::from(data.lvz);

	arma::rowvec local_position_time_ms = arma::conv_to<arma::rowvec>::from(data.local_time_boot_ms);

	//Interpolate using arma 1D interpolate
	//Generate common time series

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
	int number_of_samples = (last_sample_time-first_sample_time)*(sample_rate)/1000;
	arma::rowvec time_ms = linspace<arma::rowvec>(first_sample_time, last_sample_time, number_of_samples);

	arma::rowvec psi_interp(number_of_samples);
	arma::rowvec theta_interp(number_of_samples);
	arma::rowvec phi_interp(number_of_samples);
	arma::rowvec p_interp(number_of_samples);
	arma::rowvec q_interp(number_of_samples);
	arma::rowvec r_interp(number_of_samples);
	arma::rowvec lvx_interp(number_of_samples);
	arma::rowvec lvy_interp(number_of_samples);
	arma::rowvec lvz_interp(number_of_samples);

	arma::interp1(attitude_time_ms, psi, time_ms, psi_interp);
	arma::interp1(attitude_time_ms, theta, time_ms, theta_interp);
	arma::interp1(attitude_time_ms, phi, time_ms, phi_interp);
	arma::interp1(attitude_time_ms, p, time_ms, p_interp);
	arma::interp1(attitude_time_ms, q, time_ms, q_interp);
	arma::interp1(attitude_time_ms, r, time_ms, r_interp);
	arma::interp1(local_position_time_ms, lvx, time_ms, lvx_interp);
	arma::interp1(local_position_time_ms, lvy, time_ms, lvy_interp);
	arma::interp1(local_position_time_ms, lvz, time_ms, lvz_interp);
	
	Vehicle_States state_buffer;
	//Linear Body Velocity Computations
	arma::fmat rotation_matrix(3,3);
	arma::rowvec body_speeds(3);
	arma::rowvec linear_speeds(3);

	//Linear body speeds
	arma::rowvec u(number_of_samples);
	arma::rowvec v(number_of_samples);
	arma::rowvec w(number_of_samples);

	float theta_i;
	float psi_i;
	float phi_i;

	for(int i = 0; i< number_of_samples; i++)
	{
		psi_i = psi_interp(i);
		theta_i = theta_interp(i);
		phi_i = phi_interp(i);

		//Fill rotation matrix http://eecs.qmul.ac.uk/~gslabaugh/publications/euler.pdf
		rotation_matrix = {{cos(theta_i)*cos(phi_i), sin(psi_i)*sin(theta_i)*cos(phi_i) - cos(psi_i)*sin(phi_i), cos(psi_i)*sin(theta_i)*cos(phi_i) + sin(psi_i)*sin(phi_i)},
							{cos(theta_i)*sin(phi_i), sin(psi_i)*sin(theta_i)*sin(phi_i)+cos(psi_i)*cos(phi_i), cos(psi_i)*sin(theta_i)*sin(phi_i)-sin(psi_i)*cos(theta_i)},
							{-sin(theta_i), sin(phi_i)*cos(theta_i), cos(psi_i)*cos(theta_i)}};
		//Fill row vector with linear intertial frame velocities (xyz)
		//Intertial frame velocities are computed by local velocity (NED) - wind velocity (NED)
		//linear_speeds = {data.lvx[i]-data.wind_x[i], data.lvy[i]-data.wind_y[i], data.lvz[i]-data.wind_z[i]};
		linear_speeds = {lvx_interp(i), lvy_interp(i), lvz_interp(i)};
		//Rotate to bring intertial frame into body frame
		body_speeds = linear_speeds*rotation_matrix;
		//Insert results into the state buffer
		u(i) = body_speeds(0);
		v(i) = body_speeds(1);
		w(i) = body_speeds(2);
	}

	state_buffer.p = p_interp;
	state_buffer.q = q_interp;
	state_buffer.r = r_interp;
	state_buffer.psi = psi_interp;
	state_buffer.theta = theta_interp;
	state_buffer.phi = phi_interp;
	state_buffer.u = u;
	state_buffer.v = v;
	state_buffer.w = w;
	state_buffer.num_samples = number_of_samples;

	return state_buffer;
}

arma::mat SID::
STLSQ(arma::mat states, arma::mat candidate_functions, float threshold, float lambda)
{
	//states are row indexes
	//features are row indexes
	//time domain samples are column indexes
	bool notConverged = true;
	int iteration = 0;
	int max_iterations = 10;

	using namespace mlpack::regression;
	arma::mat coefficients(candidate_functions.n_rows, states.n_rows);

	//Do STLSQ for each state of interest
	for(int i = 0; i < states.n_rows; i++)
	{
		arma::rowvec state = states.row(i); //Get derivatives for current state
		LinearRegression lr(candidate_functions, state, 0.1, false); //Initial regression on the candidate functions
		arma::vec loop_coefficients = lr.Parameters();
		arma::uvec index = arma::find(loop_coefficients<threshold); //Find values of the coefficients below the threshold
		//Do subsequent regressions until converged 
		while(notConverged && iteration < max_iterations)
		{
			lr.Train(candidate_functions.rows(index), state, false); //Regress again on thresholded candidate functions
			//Check if relative error of subsequent solutions have changed much
			loop_coefficients = lr.Parameters();
			index = arma::find(loop_coefficients<threshold);
			if(arma::approx_equal(lr.Parameters(), loop_coefficients, "reldiff", 0.1))
			{
				notConverged = false;
			}
			iteration++;
		}
		//loop coefficients is too small for the coefficient matrix
		coefficients.col(i) = loop_coefficients; //Solution for current state
	}
	return coefficients;
}

arma::mat SID::
compute_candidate_functions(Vehicle_States states)
{
	//Compute desired candidate functions from input buffer
	int num_features = 28; //8*(8-1)/2
	//Generate ones
	arma::rowvec bias(states.num_samples);
	bias.ones();
	states.bias = bias;
	//join states into matrix so we can iterate over them all
	arma::mat state_matrix = arma::join_vert(states.bias, states.u);
	state_matrix = arma::join_vert(state_matrix, states.v, states.w, states.psi);
	state_matrix = arma::join_vert(state_matrix, states.theta, states.phi);

	arma::mat candidate_functions(num_features, states.num_samples);
	//Do for all columns
	for(int i = 0; i <state_matrix.n_cols; i++)
	{
		//For each state, multiply by all others
		for(int j = 0; j < state_matrix.n_rows; j++)
		{
			for(int k = 0; k < state_matrix.n_rows; k++)
			{
				candidate_functions(j,i) = state_matrix(j,i)*state_matrix(k,i);
			}
		}
	}
	assert(candidate_functions.n_rows == num_features);
	return candidate_functions;
}

arma::mat SID::
get_derivatives(Vehicle_States states)
{
	arma::mat derivatives = arma::join_vert(states.p, states.q, states.r);
	return derivatives;
}

void SID::
start()
{
	printf("START SINDy COMPUTE THREAD \n");
	int result = pthread_create( &compute_tid, NULL, &start_SID_compute_thread, this);
	if ( result ) throw result;
}

void SID::
stop()
{
	// --------------------------------------------------------------------------
	//   CLOSE THREADS
	// --------------------------------------------------------------------------
	printf("STOP SINDy THREAD\n");

	// signal exit
	time_to_exit = true;

	// wait for exit
	pthread_join(compute_tid ,NULL);

	// now the read and write threads are closed
	printf("\n");
}

// ------------------------------------------------------------------------------
//   Quit Handler
// ------------------------------------------------------------------------------
// Separate quit handler for interrupt
void
SID::
handle_quit( int sig )
{
	// 
	try {
		stop();
	}
	catch (int error) {
		fprintf(stderr,"Warning, could not stop SINDy\n");
	}

}

// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------

void*
start_SID_compute_thread(void *args)
{
	// takes a SID object
	SID *SINDy = (SID *)args;

	// run the object's read thread
	SINDy->compute_thread();

	// done!
	return NULL;
}