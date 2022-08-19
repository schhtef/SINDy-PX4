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
Buffer(int buffer_length_, string buffer_mode_)
{
	assert(buffer_mode_ == "time" || buffer_mode_ == "length");
	buffer_mode = buffer_mode_;
	buffer_length = buffer_length_;
	//Get time of construction to keep track of training time
	auto now = std::chrono::high_resolution_clock::now();
 	clear_time = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
}


Buffer::
~Buffer()
{

}

// Insert a generic mavlink message into the buffer. Parse the desired data points into the data buffer.
void Buffer::insert(mavlink_message_t message)
{
	// thread safe insertion into the buffer, ensures that no insertion occurs when buffer is full
	// will cause the calling thread to wait if it is full

	std::unique_lock<std::mutex> unique_lock(mtx);

	// use lambda function to determine if the buffer is currently full
	// the calling thread waits on the not_full condition variable until the consumer notifies it is empty
	not_full.wait(unique_lock, [this]()
	{
		return buffer_counter < buffer_length; 
	});

	// Switch on message type to put into the appropriate vector
	switch(message.msgid)
	{
		case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
		{
			mavlink_local_position_ned_t element;
			mavlink_msg_local_position_ned_decode(&message, &element);
			//Insert struct elements into local position buffer
			buffer.local_time_boot_ms.push_back(element.time_boot_ms);
			buffer.lvx.push_back(element.vx);
			buffer.lvy.push_back(element.vy);
			buffer.lvz.push_back(element.vz);
			buffer.x.push_back(element.x);
			buffer.y.push_back(element.y);
			buffer.z.push_back(element.z);
			break;
		}
/*
		case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
		{
			mavlink_global_position_int_t element;
			mavlink_msg_global_position_int_decode(&message, &element);
			//Insert struct elements into global position buffer
			buffer.global_time_boot_ms.push_back(element.time_boot_ms);
			buffer.alt.push_back(element.alt);
			buffer.hdg.push_back(element.hdg);
			buffer.lat.push_back(element.lat);
			buffer.lon.push_back(element.lon);
			buffer.relative_alt.push_back(element.relative_alt);
			buffer.gvx.push_back(element.vx);
			buffer.gvy.push_back(element.vy);
			buffer.gvz.push_back(element.vz);
			break;
		}
*/

/*
		case MAVLINK_MSG_ID_HIGHRES_IMU:
		{
			mavlink_highres_imu_t element;
			mavlink_msg_highres_imu_decode(&message, &element);
			input_buffer.buffered_highres_imu.push_back(element);
			break;
		}
*/
		case MAVLINK_MSG_ID_ATTITUDE:
		{
			mavlink_attitude_t element;
			mavlink_msg_attitude_decode(&message, &element);
			//Insert struct elements into attitude buffer
			buffer.pitch.push_back(element.pitch);
			buffer.pitchspeed.push_back(element.pitchspeed);
			buffer.roll.push_back(element.roll);
			buffer.rollspeed.push_back(element.rollspeed);
			buffer.attitude_time_boot_ms.push_back(element.time_boot_ms);
			buffer.yaw.push_back(element.yaw);
			buffer.yawspeed.push_back(element.yawspeed);
			break;
		}
/*
		case MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS:
		{
			mavlink_actuator_output_status_t element;
			mavlink_msg_actuator_output_status_decode(&message, &element);
			input_buffer.buffered_actuator_status.push_back(element);
			break;
		}
*/
		case MAVLINK_MSG_ID_WIND_COV:
		{
			mavlink_wind_cov_t element;
			mavlink_msg_wind_cov_decode(&message, &element);
			//Insert struct elements into attitude buffer
			buffer.wind_time_boot_ms.push_back(element.time_usec/1000); //us to ms
			buffer.wind_x.push_back(element.wind_x);
			buffer.wind_y.push_back(element.wind_y);
			buffer.wind_z.push_back(element.wind_z);
			break;
		}

		default:
		{
			//fprintf(stderr, "Warning, did not insert message id %i into buffer\n", message.msgid);
			break;
		}
	}

	if(buffer_mode == "length")
	{
		// If one of the input buffers has reached the maximum length, notify that the buffer is full
		buffer_counter = buffer.find_max_length();
	}
	else if(buffer_mode == "time")
	{
		//Find current time in s
		auto now = std::chrono::high_resolution_clock::now();
		//Buffer counter is the time since
		buffer_counter = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count() - clear_time; 
	}


	if(buffer_counter >= buffer_length)
	{
		// Notify blocked thread that buffer is full
		full.notify_one();
	}
	//unlock mutex
	unique_lock.unlock();
}

// Give the accessing thread a copy of the data buffer then empty it
Data_Buffer
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
	Data_Buffer *data = new Data_Buffer;
	*data = buffer;
	
	// Empty buffer
	buffer.clear_buffers();
	if(buffer_mode == "length")
	{
		buffer_counter = 0;
	}
	else if(buffer_mode == "time")
	{
		//Set buffer counter to 0 and save clearing time
		auto now = std::chrono::high_resolution_clock::now();
		clear_time = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
		buffer_counter = 0;
	}

    // Notify a single thread that the buffer isn't full
	not_full.notify_one();

    // Unlock unique lock
    unique_lock.unlock();
    
	// Return copy of buffer, shouldn't need to deallocate because the compiler does this on function return
	return *data;
}



