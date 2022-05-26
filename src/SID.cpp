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

#include "SID.h"

void* start_SID_compute_thread(void *args);

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
SID::
SID(Buffer *input_buffer_)
{
    input_buffer = input_buffer_;
    data = new pair <mavlink_highres_imu_t, uint64_t> [input_buffer->buffer_length];
}

SID::
~SID()
{
    delete data;
}

void SID::
compute_thread()
{
    compute_status = true;
	plog::init(plog::debug, "demo.csv", 5000, 3); // Initialize logging to the file.

    while ( ! time_to_exit )
	{
        input_buffer->clear(data);
		for(int i = 0; i < input_buffer->buffer_length, i++;)
		{
			PLOGD << "Test\n";
			//PLOGD << data[i].second;
		}
        mavlink_highres_imu_t imu = data[input_buffer->buffer_length - 1].first;
        //printf("Final acc  (NED):  % f % f % f (m/s^2)\n", imu.xacc , imu.yacc , imu.zacc );
        //printf("Final timestamp = %ld \n", data[input_buffer->buffer_length - 1].second);
	}

	compute_status = false;

	return;
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

	// still need to close the port separately
}

// ------------------------------------------------------------------------------
//   Quit Handler
// ------------------------------------------------------------------------------
void
SID::
handle_quit( int sig )
{
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