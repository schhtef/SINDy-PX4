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
	//initialize first row of the buffer
	buffer_length = buffer_length_;
}

Buffer::
~Buffer()
{

}

void Buffer::insert(pair <mavlink_highres_imu_t, uint64_t> element)
{
	// thread safe insertion into the buffer, ensures that no insertion occurs when buffer is full
	//will cause the calling thread to wait if it is full

	std::unique_lock<std::mutex> unique_lock(mtx);

	//use lambda function to determine if the buffer is currently full
	//the calling thread waits on the not_full condition variable until the consumer notifies it is empty
	not_full.wait(unique_lock, [this]()
	{
		return buffer_counter != buffer_length; 
	});

	//insert element into buffer
	buffer.push_back(element);

	//increment buffer counter
	buffer_counter++;

	//unlock mutex
	unique_lock.unlock();

	if(buffer_counter == buffer_length)
	{
		//notify blocked thread that buffer is full
		printf("Buffer is full!\n");
		full.notify_one();
	}

}

std::vector<pair <mavlink_highres_imu_t, uint64_t>> Buffer::clear()
{
	// This block of code is locked to the calling thread
	// ie. the buffer is emptied without having any data added

	// Acquire a unique lock on the mutex
    std::unique_lock<std::mutex> unique_lock(mtx);
    
    // Wait if buffer is not full
	// buffer will not notify consumer until it has filled up
    full.wait(unique_lock, [this]() {
        return buffer_counter != buffer_length;
    });
    
    //copy buffer contents to new vector
	std::vector<pair <mavlink_highres_imu_t, uint64_t>> data (buffer);

	//empty buffer
	buffer.clear();
	printf("Buffer has been emptied!\n");

	buffer_counter = 0;

    // Unlock unique lock
    unique_lock.unlock();
    
    // Notify a single thread that the buffer isn't full
    not_full.notify_one();

	// Return copy of buffer
	return data;
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
    Buffer<T> *buffer = (Buffer<T> *)args;

    buffer->read_thread();

    return NULL;
	*/
}