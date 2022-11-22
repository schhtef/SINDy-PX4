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
	arma::running_stat<double> stats;
    while ( ! time_to_exit )
	{ 
		auto t1 = std::chrono::high_resolution_clock::now();
        Data_Buffer data = input_buffer->clear();
		printf("Buffer Cleared \n");
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

		stats(SINDy_time.count());
		//std::cout << "Buffer Clear: " << clear_buffer_time.count() << "ms\n";
		//std::cout << "Interpolation: " << interpolation_time.count() << "us\n";
		//std::cout << "Candidate Functions: " << candidate_computation_time.count() << "us\n";
		//std::cout << "Derivative Parse: " << derivative_time.count() << "us\n";
		std::cout << "SINDy: " << SINDy_time.count() << "us\n";
		std::cout << "SINDy Average: " << stats.mean() << "us\n";
		std::cout << "SINDy: " << stats.stddev() << "us\n";
		std::cout << "Buffer Size: " << states.num_samples << " samples\n";

		//coefficients.print();

		//Log Results
		
		// might cause a race condition where the SINDy thread checks disarmed just before the main thread
		// sets the disarmed flag. Worst case scenario the SINDy thread logs one more buffer
		if(armed)
		{
			//log_buffer_to_csv(interpolated_telemetry, filename);
			log_coeff(coefficients, logfile_directory + "Flight Number: " + to_string(flight_number) + ".csv");
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

// Interpolates the data buffer and performs state transformations
Vehicle_States SID::
interpolate(Data_Buffer data, int sample_rate)
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
	arma::rowvec lvx = arma::conv_to<arma::rowvec>::from(data.lvx);
	arma::rowvec lvy = arma::conv_to<arma::rowvec>::from(data.lvy);
	arma::rowvec lvz = arma::conv_to<arma::rowvec>::from(data.lvz);

	//Linear Positions
	arma::rowvec x = arma::conv_to<arma::rowvec>::from(data.x);
	arma::rowvec y = arma::conv_to<arma::rowvec>::from(data.y);
	arma::rowvec z = arma::conv_to<arma::rowvec>::from(data.z);
	arma::rowvec position_time_boot_ms = arma::conv_to<arma::rowvec>::from(data.position_time_boot_ms);

	//Interpolate using arma 1D interpolate
	//Generate common time series

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
	arma::interp1(position_time_boot_ms, lvx, time_ms, lvx_interp);
	arma::interp1(position_time_boot_ms, lvy, time_ms, lvy_interp);
	arma::interp1(position_time_boot_ms, lvz, time_ms, lvz_interp);
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

// Returns indeces of vector which correspond to values which are above or below a threshold value
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

// Sequentially thresholded least squares algorithm
arma::mat 
SID::STLSQ(arma::mat states, arma::mat candidate_functions, float threshold, float lambda)
{
	//states are row indexes
	//features are row indexes
	//time domain samples are column indexes
	bool converged = false;
	int iteration = 0;
	int coefficientSize = 0;
	int max_iterations = 10;

    //Normalize candidates with respect to stdev
    //candidate_functions = candidate_functions/candidate_functions.max();

	//To store result of STLSQ
	arma::mat coefficients(candidate_functions.n_rows, states.n_rows);
	
	//Do STLSQ for each state
	for(int i = 0; i < states.n_rows; i++)
	{
        iteration = 0;
        converged = false;
		coefficientSize = 0;

		//Keep track of which candidate functions have been discarded, so we can match resulting coefficents to candidate functions
		arma::uvec coefficient_indexes(candidate_functions.n_rows);
		//For each state, fill the column with indexes 0 to number of candidate functions
		for(int j = 0; j < candidate_functions.n_rows; j++)
		{
			coefficient_indexes(j) = j;
		}
        arma::mat loop_candidate_functions = candidate_functions; //Keep copy since we will delete portions when thresholding
		arma::rowvec state = states.row(i); //Get derivatives for current state
		arma::vec loop_coefficients = ridge_regression(candidate_functions, state, lambda); //Initial regression on the candidate functions
		coefficientSize = loop_coefficients.size();
		//Do subsequent regressions until converged
		while(!converged && iteration < max_iterations)
		{
			arma::uvec below_index = threshold_vector(loop_coefficients, threshold, "below"); //Find indexes of coefficients which are lower than the threshold value
			coefficient_indexes.shed_rows(below_index); //Remove indexes which correspond to thresholded values
            loop_candidate_functions.shed_rows(below_index); //Remove indexes which correspond to thresholded values
			arma::uvec above_index = threshold_vector(loop_coefficients, threshold, "above"); //Find indexes of coefficients which are higher than the threshold value
            loop_coefficients = ridge_regression(candidate_functions.rows(above_index), state, lambda); //Regress again on thresholded candidate functions
            //Check if coefficient vector has changed in size since last iteration
			if(coefficientSize == loop_coefficients.size())
			{
				converged = true; //If thresholding hasn't shrunk the coefficient vector, we have converged
			}
			coefficientSize = loop_coefficients.size(); //update with size of coefficient vector
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

// Ridge regression. Essentially least squares when lambda = 0
arma::vec SID::
ridge_regression(arma::mat candidate_functions, arma::rowvec state, float lambda)
{
	//Ridge regression is defined by the coefficients \beta = ((X'X+\lamdaI)^-1)X'y
	//Where X is the design matrix (candidate functions), y are the responses (states), 
	//and lambda is the regression penalty
	//Armadillo's linear solver is used to solve for \beta by arranging the expression as
	//(X'X+\lamdaI)\beta = X'y
	arma::mat A = candidate_functions * candidate_functions.t() + lambda * arma::eye<arma::mat>(candidate_functions.n_rows, candidate_functions.n_rows);
	arma::vec coefficients = arma::solve(A, candidate_functions*state.t());
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