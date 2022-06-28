/**
 * @file logger.cpp
 *
 * @brief Telemetry file logging functions
 *
 * Functions for reading/writing mavlink messages from/to files
 * 
 * @author Stefan Bichlmaier, <bichlmaier.stef@gmail.com>
 *
 */

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "buffer.h"

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
Buffer::
Buffer()
{

}


Buffer::
Buffer(int buffer_length_)
{
	buffer_length = buffer_length_;
}


Buffer::
~Buffer()
{

}

void Buffer::insert(mavlink_message_t message)
{
	// thread safe insertion into the buffer, ensures that no insertion occurs when buffer is full
	// will cause the calling thread to wait if it is full

	std::unique_lock<std::mutex> unique_lock(mtx);

	// use lambda function to determine if the buffer is currently full
	// the calling thread waits on the not_full condition variable until the consumer notifies it is empty
	not_full.wait(unique_lock, [this]()
	{
		return buffer_counter != buffer_length; 
	});

	// Switch on message type to put into the appropriate vector
	switch(message.msgid)
	{
		case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
		{
			mavlink_local_position_ned_t element;
			mavlink_msg_local_position_ned_decode(&message, &element);
			input_buffer.buffered_local_position_ned.push_back(element);
			break;
		}

		case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
		{
			mavlink_global_position_int_t element;
			mavlink_msg_global_position_int_decode(&message, &element);
			input_buffer.buffered_global_position_int.push_back(element);
			break;
		}

		case MAVLINK_MSG_ID_HIGHRES_IMU:
		{
			mavlink_highres_imu_t element;
			mavlink_msg_highres_imu_decode(&message, &element);
			input_buffer.buffered_highres_imu.push_back(element);
			break;
		}

		case MAVLINK_MSG_ID_ATTITUDE:
		{
			mavlink_attitude_t element;
			mavlink_msg_attitude_decode(&message, &element);
			input_buffer.buffered_attitude.push_back(element);
			break;
		}

		case MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS:
		{
			mavlink_actuator_output_status_t element;
			mavlink_msg_actuator_output_status_decode(&message, &element);
			input_buffer.buffered_actuator_status.push_back(element);
			break;
		}

		default:
		{
			fprintf(stderr, "Warning, did not insert message id %i into buffer\n", message.msgid);
			break;
		}
	}

	buffer_counter = find_max_length(input_buffer);
	// If one of the input buffers has reached the maximum length, notify that the buffer is full

	if(buffer_counter == buffer_length)
	{
		// Notify blocked thread that buffer is full
		printf("Buffer is full!\n");
		full.notify_one();
	}
	//unlock mutex
	unique_lock.unlock();
}

Mavlink_Message_Buffers
Buffer::clear()
{
	// This block of code is locked to the calling thread
	// ie. the buffer is emptied without having any data added

	// Acquire a unique lock on the mutex
    std::unique_lock<std::mutex> unique_lock(mtx);
    
    // Wait if buffer is not full
	// Will wait as long as predicate is false
	// buffer will not notify consumer until it has filled up
    full.wait(unique_lock, [this]{return buffer_counter == buffer_length;});
    
    // Copy buffer contents to new vector, because we will unlock the mutex
	// before returning the buffer
	Mavlink_Message_Buffers *data = new Mavlink_Message_Buffers;
	*data = input_buffer;

	printf("Buffer has been emptied!\n");
	printf("Buffer Counter: %d\n", buffer_counter);
	
	// Empty buffer
	input_buffer.clear_buffers();
	buffer_counter = 0;

    // Notify a single thread that the buffer isn't full
	not_full.notify_one();

    // Unlock unique lock
    unique_lock.unlock();
    
	// Return copy of buffer, shouldn't need to deallocate because the compiler does this on function return
	return *data;
}

// Go through all buffers and find the longest one
int find_max_length(Mavlink_Message_Buffers buffer)
{
    int max_length = buffer.buffered_actuator_status.size();

    if(buffer.buffered_attitude.size() > max_length)
    {
        max_length = buffer.buffered_attitude.size();
    }

    if(buffer.buffered_global_position_int.size() > max_length)
    {
        max_length = buffer.buffered_global_position_int.size();
    }

    if(buffer.buffered_highres_imu.size() > max_length)
    {
        max_length = buffer.buffered_highres_imu.size();
    }

    if(buffer.buffered_local_position_ned.size() > max_length)
    {
        max_length = buffer.buffered_local_position_ned.size();
    }

    return max_length;
}


int Buffer::
start()
{
	/*
    int result;

    printf("START BUFFER READ THREAD \n");

	result = pthread_create( &write_tid, NULL, &start_buffer_read_thread, this );
	if ( result ) throw result;
	// Initialize buffer tuple arrays according to config
	// now we're writing to the buffer
	printf("\n");
	*/
	
}


int Buffer::
read_thread()
{
	/*
    while(!time_to_exit)
    {
		//Write specified parameters to tuple arrays at sampling rate
		//If buffer is full, send to SID process, set ready flag?
		//Once received, clear buffer and reset ready flag.
		//Mutex? ie, how do I know that the SID process received the buffer
		
		mavlink_highres_imu_t imu = messages->highres_imu;
        usleep(250000); //4hz
    }
	*/
}

// ------------------------------------------------------------------------------
//   SHUTDOWN
// ------------------------------------------------------------------------------

void Buffer::
stop()
{
	/*
	// --------------------------------------------------------------------------
	//   CLOSE THREADS
	// --------------------------------------------------------------------------
	printf("CLOSE THREADS\n");

	// signal exit
	time_to_exit = true;

	// wait for exit
	pthread_join(write_tid,NULL);

	// now the read and write threads are closed
	printf("\n");
	*/
}

// ------------------------------------------------------------------------------
//   Quit Handler
// ------------------------------------------------------------------------------

void Buffer::
handle_quit( int sig )
{
	try {
		stop();
	}
	catch (int error) {
		fprintf(stderr,"Warning, could not stop Buffer\n");
	}

}

//this function exists because it doesn't depend on the state of the Buffer object so
//it doesn't need to be a member function
void* start_buffer_read_thread(void *args)
{
	/*
    Buffer *buffer = (Buffer *)args;

    buffer->read_thread();

    return NULL;
	*/
}