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
		//Generate Candidate Functions
		arma::mat candidate_functions = compute_candidate_functions(interpolated_data);
		//Construct arma::mat and arma::vec for SINDy
		arma::mat derivatives = get_state_derivatives(interpolated_data);
		//Pass into STLSQ
		arma::mat coefficients = STLSQ(derivatives, candidate_functions, STLSQ_threshold, lambda);
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

arma::rowvec SID::
threshold(arma::vec coefficients, arma::mat candidate_functions, float threshold)
{
	//Find indexes of coefficient vector which are smaller than the threshold

}

arma::mat SID::
get_state_derivatives(Data_Buffer data)
{
	//Select and compute state derivatives from input buffer
}

arma::mat SID::
compute_candidate_functions(Data_Buffer data)
{
	//Compute desired candidate functions from input buffer
	
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