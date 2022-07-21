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
		interpolate(data); // Bring data to a common timebase and interpolate it
		// might cause a race condition where the SINDy thread checks disarmed just before the main thread
		// sets the disarmed flag. Worst case scenario the SINDy thread logs one more buffer
		if(!disarmed)
		{
			log_buffer_to_csv(data, filename);
		}
		usleep(5000000);
	}
	compute_status = false;

	return;
}

/**
 * Interpolate the samples the Mavlink Message Buffer
 *
 * @param data Struct containing vectors of mavlink messages
 * @return void
 */
void SID::
interpolate(Mavlink_Message_Buffers data)
{ 
	// Find earliest sample, remember the time. This will be the time origin
	uint64_t first_sample_time = data.buffered_actuator_status.at(0).time_usec/1000; //Convert to ms
	// Use generic pointer to point to the vector with first sample
	void *first_sample_vector = &data.buffered_actuator_status; 

	if(data.buffered_attitude.at(0).time_boot_ms < first_sample_time)
	{
		first_sample_time = data.buffered_attitude.at(0).time_boot_ms;
		first_sample_vector = &data.buffered_attitude;
	}
	if(data.buffered_global_position_int.at(0).time_boot_ms < first_sample_time)
	{
		first_sample_time = data.buffered_global_position_int.at(0).time_boot_ms;
		first_sample_vector = &data.buffered_global_position_int;
	}
	if(data.buffered_highres_imu.at(0).time_usec/1000 < first_sample_time) //Convert to ms
	{
		first_sample_time = data.buffered_highres_imu.at(0).time_usec/1000;
		first_sample_vector = &data.buffered_highres_imu;
	}
	if(data.buffered_local_position_ned.at(0).time_boot_ms < first_sample_time)
	{
		first_sample_time = data.buffered_local_position_ned.at(0).time_boot_ms;
		first_sample_vector = &data.buffered_local_position_ned;
	}
	// For each vector, find difference in time between consecutive samples
	// Calculate the number of zeros to add in, (time in ms)/(time resolution in ms)
	// Pad with zeros

	std::vector<mavlink_actuator_output_status_t>::iterator it = data.buffered_actuator_status.begin();
	while(it != data.buffered_actuator_status.end())
	{
		//Currently spacing is 1ms, divide result by desired timebase
		uint64_t number_of_interpolants = ((*it).time_usec/1000 - first_sample_time); 
		//insert number of interpolants into the list
		for(int i = 0; i <= number_of_interpolants; i++)
		{
			mavlink_actuator_output_status_t empty_actuator_status;
			empty_actuator_status.time_usec = ((*it).time_usec)-1000; //Inserting behind current element, so previous ms is needed 
			data.buffered_actuator_status.insert(it, empty_actuator_status);
		}
		it = it+number_of_interpolants+1;
	}
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