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
        data = input_buffer->clear();
		interpolated_data = interpolate(data, 500); // Resample input buffer and interpolate
		Vehicle_States states = compute_states(interpolated_data);
		arma::mat candidate_functions = compute_candidate_functions(states); //Generate Candidate Functions
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

arma::mat SID::
STLSQ(arma::mat states, arma::mat candidate_functions, float threshold, float lambda)
{
	//states are row indexes
	//features are row indexes
	//time domain samples are column indexes
	bool notConverged = true;

	using namespace mlpack::regression;
	arma::mat coefficients(candidate_functions.n_rows, states.n_rows);

	//Do STLSQ for each state of interest
	for(int i = 0; i < states.n_rows; i++)
	{
		arma::rowvec state = states.row(i); //Get derivatives for current state
		LinearRegression lr(candidate_functions, state, 0.1, false); //Initial regression on the candidate functions
		arma::vec state_coefficients = lr.Parameters();
		//Do subsequent regressions until converged 
		while(notConverged)
		{
			arma::uvec index = arma::find(coefficients<threshold); //Find values of the coefficients below the threshold
			arma::vec loop_coefficients = lr.Parameters();
			lr.Train(candidate_functions.rows(index), state, false); //Regress again on thresholded candidate functions
			//Check if relative error of subsequent solutions have changed much
			if(arma::approx_equal(lr.Parameters(), loop_coefficients, "reldiff", 0.1))
			{
				notConverged = false;
			}
		}
		coefficients.col(i) = lr.Parameters(); //Solution for current state
	}
	return coefficients;
}

Vehicle_States SID::
compute_states(Data_Buffer data)
{
	Vehicle_States state_buffer;
	//Perform the coordinate conversions to obtain the desired states
	//Euler angles
	state_buffer.psi = conv_to<rowvec>::from(data.roll);
	state_buffer.theta = conv_to<rowvec>::from(data.pitch);
	state_buffer.phi = conv_to<rowvec>::from(data.yaw);

	//Angular Velocities
	state_buffer.p = conv_to<rowvec>::from(data.rollspeed);
	state_buffer.q = conv_to<rowvec>::from(data.pitchspeed);
	state_buffer.r = conv_to<rowvec>::from(data.yawspeed);

	//Linear Body Velocity Computations
	arma::mat rotation_matrix(3,3);
	arma::rowvec body_speeds(3);
	arma::rowvec linear_speeds(3);
	float theta;
	float psi;
	float phi;

	for(int i = 0; i< state_buffer.num_samples; i++)
	{
		psi = state_buffer.psi(i);
		theta = state_buffer.theta(i);
		phi = state_buffer.phi(i);

		//Fill rotation matrix http://eecs.qmul.ac.uk/~gslabaugh/publications/euler.pdf
		rotation_matrix = {{cos(theta)*cos(phi), sin(psi)*sin(theta)*cos(phi) - cos(psi)sin(phi), cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi)},
							{cos(theta)*sin(phi), sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi), cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(theta)},
							{-sin(theta), sin(phi)*cos(theta), cos(psi)*cos(theta)}};
		//Fill row vector with linear intertial frame velocities (xyz)
		//Intertial frame velocities are computed by local velocity (NED) - wind velocity (NED)
		linear_speeds = {data.lvx(i)-data.wind_x(i), data.lvy(i)-data.wind_y(i), data.lvz(i)-data.wind_z(i)};
		//Rotate to bring intertial frame into body frame
		body_speeds = linear_speeds*rotation_matrix;
		//Insert results into the state buffer
		state_buffer.u(i) = body_speeds(0);
		state_buffer.v(i) = body_speeds(1);
		state_buffer.w(i) = body_speeds(2);
	}

}

arma::mat SID::
compute_candidate_functions(Vehicle_States states)
{
	//Compute desired candidate functions from input buffer
	int num_features = 28; //8*(8-1)/2
	//join states into matrix so we can iterate over them all
	arma::mat state_matrix = arma::join_vert(bias, states.u);
	state_matrix = arma::join_vert(state_matrix, states.v, states.w, states.psi);
	state_matrix = arma::join_vert(state_matrix, states.theta, states.phi);
	assert(state_matrix.n_rows == num_features);

	arma::mat candidate_functions(num_features, states.num_samples);
	//Do for all columns
	for(int i = 0; i <states.n_cols; i++)
	{
		//For each state, multiply by all others
		for(int j = 0; j < states.n_rows; j++)
		{
			for(int k = 0; k < states.n_rows; k++)
			{
				candidate_functions(j,i) = states(j,i)*states(k,i);
			}
		}
	}
	return candidate_functions;
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