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

#include "logger.h"

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
Logger::
Logger(ofstream *file_, Mavlink_Messages *messages_)
{
    file = file_; // file management object
    write_tid = 0; // write thread id
	messages = messages_;
}

Logger::
~Logger()
{
}

int
Logger::
start()
{
    int result;

    //check that file is open and ready to write to
    if ( !file->is_open() ) // error if file not open
	{
		fprintf(stderr,"ERROR: file not open\n");
		throw 1;
	}
    
    
    //start the file write thread
    printf("START FILE WRITE THREAD \n");

	result = pthread_create( &write_tid, NULL, &start_logger_write_thread, this );
	if ( result ) throw result;

	// now we're writing to file
	printf("\n");
    
}

int
Logger::
write_thread()
{
    while(!time_to_exit)
    {
		mavlink_highres_imu_t imu = messages->highres_imu;
		//printf("    acc  (NED):  % f % f % f (m/s^2)\n", imu.xacc , imu.yacc , imu.zacc );
		*file << imu.xacc << endl;
		//*file << printf();
        //write messages to file at writingRate
        usleep(250000); //4hz
    }
}

// ------------------------------------------------------------------------------
//   SHUTDOWN
// ------------------------------------------------------------------------------
void
Logger::
stop()
{
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

    // close the logfile
    file->close();

}

// ------------------------------------------------------------------------------
//   Quit Handler
// ------------------------------------------------------------------------------
void
Logger::
handle_quit( int sig )
{
	try {
		stop();
	}
	catch (int error) {
		fprintf(stderr,"Warning, could not stop logger\n");
	}

}

//this function exists because it doesn't depend on the state of the logger object so
//it doesn't need to be a member function
void*
start_logger_write_thread(void *args)
{
    Logger *logger = (Logger *)args;

    logger->write_thread();

    return NULL;
}