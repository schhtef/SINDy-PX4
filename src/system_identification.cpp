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
SID(Buffer *input_buffer_, std::chrono::_V2::system_clock::time_point program_epoch, float stlsq_threshold, float ridge_regression_penalty, std::string coefficient_logfile_path_, bool debug_)
{
    input_buffer = input_buffer_;
	STLSQ_threshold = stlsq_threshold;
	lambda = ridge_regression_penalty;
	coefficient_logfile_path = coefficient_logfile_path_;
	flight_number = 0;
	epoch = program_epoch;
	debug = debug_;
}

SID::
~SID()
{
}

void SID::
start()
{
	compute_thread = std::thread(&SID::sindy_compute, this);
}

void SID::
sindy_compute()
{
	using namespace std;
	cout << "Performing sindy\n";
    compute_status = true;
	arma::running_stat<double> stats;
	initialize_logfile(coefficient_logfile_path); //Write header to coefficient logfile
    while ( ! time_to_exit )
	{
		auto t1 = std::chrono::high_resolution_clock::now();
        Data_Buffer data = input_buffer->clear();
		//std::cout << "Cleared Buffer\n";
		auto t2 = std::chrono::high_resolution_clock::now();
		Vehicle_States states = linear_interpolate(data, 200); // Resample input buffer and compute desired states
		auto t3 = std::chrono::high_resolution_clock::now();
		//std::cout << "Interpolated Buffer\n";
		arma::mat candidate_functions = compute_candidate_functions(states); //Generate Candidate Function
		auto t4 = std::chrono::high_resolution_clock::now();
		//std::cout << "Computed Candidates\n";
		arma::mat derivatives = get_derivatives(states); //Get state derivatives for SINDy
		auto t5 = std::chrono::high_resolution_clock::now();
		//std::cout << "Computed Derivatives\n";
		arma::mat coefficients = STLSQ(derivatives, candidate_functions, STLSQ_threshold, lambda); //Run STLSQ
		auto t6 = std::chrono::high_resolution_clock::now();
		//std::cout << "Completed STLSQ\n";

		assert(states.num_samples == candidate_functions.n_cols); // Check that number of samples are preserved after computing candidate functions
		assert(candidate_functions.n_cols == derivatives.n_cols); // Check that number of samples in candidate functions and derivatives are equal
		assert(candidate_functions.n_rows == coefficients.n_rows); // Check that number of features is equal in the candidate functions and solved coefficients

    	auto clear_buffer_time = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
    	auto interpolation_time = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2);
		auto candidate_computation_time = std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3);
		auto derivative_time = std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4);
		auto SINDy_time = std::chrono::duration_cast<std::chrono::microseconds>(t6 - t5);

		std::chrono::microseconds coefficient_sample_time = std::chrono::duration_cast<std::chrono::microseconds>(t6 - epoch);

		stats(SINDy_time.count());

		if(debug){
			std::cout << "Buffer Clear: " << clear_buffer_time.count() << "ms\n";
			std::cout << "Interpolation: " << interpolation_time.count() << "us\n";
			std::cout << "Candidate Functions: " << candidate_computation_time.count() << "us\n";
			std::cout << "Derivative Parse: " << derivative_time.count() << "us\n";
			std::cout << "SINDy: " << SINDy_time.count() << "us\n";
			std::cout << "SINDy Average: " << stats.mean() << "us\n";
			std::cout << "SINDy: " << stats.stddev() << "us\n";
			std::cout << "Buffer Size: " << states.num_samples << " samples\n";
			coefficients.print();
		}

		//Log Results

		//log_buffer_to_csv(interpolated_telemetry, filename);
		log_coeff(coefficients, coefficient_logfile_path, coefficient_sample_time);
		//coefficients.save(arma::hdf5_name(logfile_directory + "Flight Number: " + to_string(flight_number)+".hdf5", "coefficients", arma::hdf5_opts::append));
	}
	compute_status = false;
	return;
}

void SID::
initialize_logfile(std::string filename)
{
	using namespace std;
	ofstream myfile;
    myfile.open (filename, ios_base::trunc);
	//std::array<string, 11> first_order_candidates = {"1", "x", "y", "z", "psi", "theta", "phi", "u0", "u1", "u2", "u3"};
	vector<string> second_order_candidates = {"1", "x", "y", "z", "psi", "theta", "phi", "u0", "u1", "u2", "u3",
								"x^2", "xy", "xz", "xpsi", "xtheta", "xphi", "xu0", "xu1", "xu2", "xu3",
								"y^2", "yz", "ypsi", "ytheta","yphi", "yu0", "yu1", "yu2", "yu3",
								"z^2", "zpsi", "ztheta", "zphi","zu0", "zu1", "zu2", "zu3",
								"psi^2","psitheta",	"psiphi", "psiu0", "psiu1", "psiu2", "psiu3", 							
								"theta^2", "thetaphi", "thetau0", "thetau1", "thetau2", "thetau3",
								"phi^2", "phiu0", "phiu1", "phiu2", "phiu3", 
								"u0^2", "u0u1", "u0u2", "u0u3",
								"u1^2", "u1u2", "u1u3",
								"u2^2", "u2u3",
								"u3^2"						
								}; //would like to generate this programmatically at some point
	
	vector<string> states = {"p", "q", "r", "u", "v", "w"};

	myfile << "Time (us)" << ",";
	//Generate header and write to file
	//For each candidate, create a column which is the candidate multiplied by a state variable
	for(auto candidate_iterator = second_order_candidates.begin(); candidate_iterator != second_order_candidates.end(); ++candidate_iterator)
	{
		for(auto state_iterator = states.begin(); state_iterator != states.end(); ++state_iterator)
		{
			myfile << *candidate_iterator << "-" << *state_iterator << ",";
		}
	}
	myfile << "\n";
	myfile.close();
}

// Returns indeces of vector which correspond to values which are above or below a threshold value
arma::uvec SID::
threshold_vector(arma::vec vector, float threshold, std::string mode)
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
            loop_coefficients = ridge_regression(loop_candidate_functions, state, lambda); //Regress again on thresholded candidate functions
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
			//fprintf(stderr, "Thresholding parameter set too high and removed all coefficients in state %d\n", i);
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

// Compute 2nd order candidate functions given that states are rows, samples are columns
// Overloaded function allows you to just pass in a plain arma matrix. The actual candidate function computation
// Is identical to the function which takes a Vehicle_States struct, without the repackaging into matrices
arma::mat SID::
compute_candidate_functions(arma::mat states)
{
	int num_samples = states.n_cols;
	arma::rowvec bias = arma::ones<arma::rowvec>(num_samples);
	states = arma::join_cols(bias, states);
	int num_features = (states.n_rows)*(states.n_rows+1)/2;
	int candidate_index = 0; //Index to keep track of insertion into candidate functions
	arma::mat candidate_functions(num_features, num_samples);
	//Compute second order combinations for each column
	for(int i = 0; i <states.n_cols; i++)
	{
		//For each state, multiply by all others
		for(int j = 0; j < states.n_rows; j++)
		{
			for(int k = j; k < states.n_rows; k++)
			{
				candidate_functions(candidate_index,i) = states(j,i)*states(k,i);
				candidate_index++;
			}
		}
		candidate_index = 0;
	}
	return candidate_functions;
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
	state_matrix = arma::join_vert(state_matrix, states.actuator0, states.actuator1);
	state_matrix = arma::join_vert(state_matrix, states.actuator2, states.actuator3);

	int num_features = state_matrix.n_rows*(state_matrix.n_rows+1)/2; //Compute total number of combinations of vehicle states
	arma::mat candidate_functions(num_features, states.num_samples);
	int candidate_index = 0; //Index to keep track of insertion into candidate functions
	
	//Compute second order combinations for each column
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
	//compute_thread.join();

	// now the read and write threads are closed
	printf("\n");
}