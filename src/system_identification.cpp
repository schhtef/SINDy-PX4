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
	STLSQ_threshold = 0.01;
	lambda = 0.5;
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
		auto t1 = std::chrono::high_resolution_clock::now();
        Data_Buffer data = input_buffer->clear();
		auto t2 = std::chrono::high_resolution_clock::now();
		Vehicle_States states = interpolate(data, 200); // Resample input buffer and compute desired states
		auto t3 = std::chrono::high_resolution_clock::now();
		arma::mat candidate_functions = compute_candidate_functions(states); //Generate Candidate Function
		auto t4 = std::chrono::high_resolution_clock::now();
		arma::mat derivatives = get_derivatives(states); //Get state derivatives for SINDy
		auto t5 = std::chrono::high_resolution_clock::now();
		arma::mat coefficients = STLSQ(derivatives, candidate_functions, STLSQ_threshold, lambda); //Run STLSQ
		auto t6 = std::chrono::high_resolution_clock::now();

    	auto clear_buffer_time = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
    	auto interpolation_time = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2);
		auto candidate_computation_time = std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3);
		auto derivative_time = std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4);
		auto SINDy_time = std::chrono::duration_cast<std::chrono::microseconds>(t6 - t5);

		log_coeff(coefficients, "coefficients.csv");

		std::cout << "Buffer Clear: " << clear_buffer_time.count() << "ms\n";
		std::cout << "Interpolation: " << interpolation_time.count() << "us\n";
		std::cout << "Candidate Functions: " << candidate_computation_time.count() << "us\n";
		std::cout << "Derivative Parse: " << derivative_time.count() << "us\n";
		std::cout << "SINDy: " << SINDy_time.count() << "us\n";

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

void SID::
log_coeff(arma::mat matrix, string filename)
{
	std::ofstream myfile;
    myfile.open (filename);
	myfile << ",p, q, r, u, v, w" << endl;
	std::vector<string> candidates = {"1", "x", "y", "z", "psi", "theta", "phi",
									"x^2", "xy", "xz", "xpsi", "xtheta", "xphi", 
									"y^2", "yz", "ypsi", "ytheta", "yphi", "z^2",
									"zpsi", "ztheta", "zphi","psi^2","psitheta",
									"psiphi", "theta^2", "thetaphi", "phi^2"};
	assert(candidates.size() == matrix.n_rows);
	for(int i = 0; i < matrix.n_rows; i++)
	{
		myfile << candidates[i] << ",";
		for(int j = 0; j < matrix.n_cols; j++)
		{
			myfile << matrix(i,j) << ",";
		}
		myfile << "\n";
	}
	myfile.close();
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
	arma::rowvec x = arma::conv_to<arma::rowvec>::from(data.x);
	arma::rowvec y = arma::conv_to<arma::rowvec>::from(data.y);
	arma::rowvec z = arma::conv_to<arma::rowvec>::from(data.z);

	arma::rowvec local_position_time_ms = arma::conv_to<arma::rowvec>::from(data.local_time_boot_ms);

	//Interpolate using arma 1D interpolate
	//Generate common time series

	// Find latest first sample sample time, this will be the time origin
	// Using latest so no extrapolation occurs
 	uint32_t first_sample_time = data.attitude_time_boot_ms.front();

	if(data.local_time_boot_ms.front() > first_sample_time)
	{
		first_sample_time = data.local_time_boot_ms.front();
	}
    /*
	if((data.wind_time_boot_ms.front()) < first_sample_time)
	{
		first_sample_time = data.wind_time_boot_ms.front();
	}
    */

    // Find the buffer with the earliest last sample to avoid extrapolation
 	uint32_t last_sample_time = data.attitude_time_boot_ms.back();

	if(data.local_time_boot_ms.back() < last_sample_time)
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
	arma::rowvec x_interp(number_of_samples);
	arma::rowvec y_interp(number_of_samples);
	arma::rowvec z_interp(number_of_samples);

	arma::interp1(attitude_time_ms, psi, time_ms, psi_interp);
	arma::interp1(attitude_time_ms, theta, time_ms, theta_interp);
	arma::interp1(attitude_time_ms, phi, time_ms, phi_interp);
	arma::interp1(attitude_time_ms, p, time_ms, p_interp);
	arma::interp1(attitude_time_ms, q, time_ms, q_interp);
	arma::interp1(attitude_time_ms, r, time_ms, r_interp);
	arma::interp1(local_position_time_ms, lvx, time_ms, lvx_interp);
	arma::interp1(local_position_time_ms, lvy, time_ms, lvy_interp);
	arma::interp1(local_position_time_ms, lvz, time_ms, lvz_interp);
	arma::interp1(local_position_time_ms, x, time_ms, x_interp);
	arma::interp1(local_position_time_ms, y, time_ms, y_interp);
	arma::interp1(local_position_time_ms, z, time_ms, z_interp);
	
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
	state_buffer.x = x_interp;
	state_buffer.y = y_interp;
	state_buffer.z = z_interp;
	state_buffer.num_samples = number_of_samples;

	return state_buffer;
}

arma::uvec SID::
threshold_vector(arma::vec vector, float threshold, string mode)
{
    std::vector<uint> index;
    if(mode == "below")
    {
        for(int i = 0; i < vector.size(); i++)
        {
            if(std::abs(vector(i)) < threshold)
            {
                index.push_back(i);
            }
        }
    }
    else if(mode == "above")
    {
        for(int i = 0; i < vector.size(); i++)
        {
            if(std::abs(vector(i)) > threshold)
            {
                index.push_back(i);
            }
        }        
    }
    arma::uvec thresholded_indexes = arma::conv_to<arma::uvec>::from(index);
    return thresholded_indexes;
}

arma::mat 
SID::STLSQ(arma::mat states, arma::mat candidate_functions, float threshold, float lambda)
{
	//states are row indexes
	//features are row indexes
	//time domain samples are column indexes
	bool notConverged = true;
	int iteration;
	int max_iterations = 10;

    //Normalize candidates with respect to stdev
    //candidate_functions = candidate_functions/candidate_functions.max();

	using namespace mlpack::regression;
	//To store result of STLSQ
	arma::mat coefficients(candidate_functions.n_rows, states.n_rows);
	
	//Do STLSQ for each state
	for(int i = 0; i < states.n_rows; i++)
	{
        iteration = 0;
        notConverged = true;
		//Keep track of which candidate functions have been discarded, so we can match resulting coefficents to candidate functions
		arma::uvec coefficient_indexes(candidate_functions.n_rows);
		//For each state, fill the column with indexes 0 to number of candidate functions
		for(int j = 0; j < candidate_functions.n_rows; j++)
		{
			coefficient_indexes(j) = j;
		}
        arma::mat loop_candidate_functions = candidate_functions; //Keep copy since we will delete portions when thresholding
		arma::rowvec state = states.row(i); //Get derivatives for current state
		LinearRegression lr(candidate_functions, state, lambda, false); //Initial regression on the candidate functions
		arma::vec loop_coefficients = lr.Parameters();
		//Do subsequent regressions until converged
		while(notConverged && iteration < max_iterations)
		{
			arma::uvec below_index = threshold_vector(loop_coefficients, threshold, "below"); //Find indexes of coefficients which are lower than the threshold value
			coefficient_indexes.shed_rows(below_index); //Remove indexes which correspond to thresholded values
            loop_candidate_functions.shed_rows(below_index);
			arma::uvec above_index = threshold_vector(loop_coefficients, threshold, "above"); //Find indexes of coefficients which are higher than the threshold value
            //lr.Train(candidate_functions.rows(above_index), state, false); //Regress again on thresholded candidate functions
			lr.Train(loop_candidate_functions, state, false); //Initial regression on the candidate functions
            //Check if coefficient vector has changed in size since last iteration
			if(lr.Parameters().size() == loop_coefficients.size())
			{
				//notConverged = false; //If thresholding hasn't shrunk the coefficient vector, we have converged
			}
			loop_coefficients = lr.Parameters();
			iteration++; //Keep track of iteration number
		}

		if(loop_coefficients.size() == 0)
		{
			fprintf(stderr, "Thresholding parameter set too low and removed all coefficients in state %d\n", i);
			coefficients.col(i).zeros(); //Set all coefficients to zero and don't attempt to match indexes
			break;
		}
		
		//Match coefficients to their candidate functions
		arma::vec state_coefficients = arma::zeros(candidate_functions.n_rows);
        //std::assert(coefficient_indexes.n_rows == loop_coefficients.n_rows);
		for(int k = 0; k < coefficient_indexes.n_rows; k++)
		{
			state_coefficients(coefficient_indexes(k)) = loop_coefficients(k);
		}
		//loop coefficients is too small for the coefficient matrix
		coefficients.col(i) = state_coefficients; //Solution for current state
	}
	return coefficients;
}

arma::mat SID::
compute_candidate_functions(Vehicle_States states)
{
	//Compute desired candidate functions from input buffer
	//Generate ones
	arma::rowvec bias(states.num_samples);
	bias.ones();
	states.bias = bias;
	//join states into matrix so we can iterate over them all
	arma::mat state_matrix = arma::join_vert(states.bias, states.x);
	state_matrix = arma::join_vert(state_matrix, states.y, states.z, states.psi);
	state_matrix = arma::join_vert(state_matrix, states.theta, states.phi);
	int num_features = state_matrix.n_rows*(state_matrix.n_rows+1)/2; //8*(8-1)/2
	arma::mat candidate_functions(num_features, states.num_samples);
	//Index to keep track of insertion into kandidate function6
	int candidate_index = 0;
	//Do for all columns
	for(int i = 0; i <state_matrix.n_cols; i++)
	{
		//For each state, multiply by all others
		for(int j = 0; j < state_matrix.n_rows; j++)
		{
			for(int k = j; k < state_matrix.n_rows; k++)
			{
				candidate_functions(candidate_index,i) = state_matrix(j,i)*state_matrix(k,i);
				candidate_index++;
			}
		}
		candidate_index = 0;
	}
	assert(candidate_functions.n_rows == num_features);
	return candidate_functions;
}

arma::mat SID::
get_derivatives(Vehicle_States states)
{
	arma::mat derivatives = arma::join_vert(states.p, states.q, states.r);
	derivatives = arma::join_vert(derivatives, states.u, states.v, states.w);
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