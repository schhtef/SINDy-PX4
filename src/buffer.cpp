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
Buffer(int buffer_length_, buffer_mode mode_)
{
	//assert(buffer_mode == time || buffer_mode == length);
	mode = mode_;
	buffer_length = buffer_length_;
	//Get time of construction to keep track of training time
	auto now = std::chrono::high_resolution_clock::now();
 	clear_time = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
}


Buffer::
~Buffer()
{

}

// Insert an angular velocity message into the buffer
void Buffer::insert(mavsdk::Telemetry::Odometry message, uint64_t timestamp)
{
	// thread safe insertion into the buffer, ensures that no insertion occurs when buffer is full
	// will cause the calling thread to wait if it is full

	std::unique_lock<std::mutex> unique_lock(mtx);

	// use lambda function to determine if the buffer is currently full
	// the calling thread waits on the not_full condition variable until the consumer notifies it is empty
	// in summary, if the buffer is full, wait to insert until it has been emptied
	not_full.wait(unique_lock, [this]()
	{
		return buffer_counter < buffer_length; 
	});

	buffer.position_time_boot_ms.push_back(timestamp);

	buffer.x.push_back(message.position_body.x_m);
	buffer.y.push_back(message.position_body.y_m);
	buffer.z.push_back(message.position_body.z_m);

	buffer.x_m_s.push_back(message.velocity_body.x_m_s);
	buffer.y_m_s.push_back(message.velocity_body.y_m_s);
	buffer.z_m_s.push_back(message.velocity_body.z_m_s);


	if(mode == buffer_mode::length_mode)
	{
		// If one of the input buffers has reached the maximum length, notify that the buffer is full
		buffer_counter = buffer.find_max_length();
	}
	else if(mode == buffer_mode::time_mode)
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


// Insert an angular velocity message into the buffer
void Buffer::insert(mavsdk::Telemetry::AngularVelocityBody message, uint64_t timestamp)
{
	// thread safe insertion into the buffer, ensures that no insertion occurs when buffer is full
	// will cause the calling thread to wait if it is full

	std::unique_lock<std::mutex> unique_lock(mtx);

	// use lambda function to determine if the buffer is currently full
	// the calling thread waits on the not_full condition variable until the consumer notifies it is empty
	// in summary, if the buffer is full, wait to insert until it has been emptied
	not_full.wait(unique_lock, [this]()
	{
		return buffer_counter < buffer_length; 
	});

	buffer.angular_velocity_time_boot_ms.push_back(timestamp);
	buffer.pitchspeed.push_back(message.pitch_rad_s);
	buffer.rollspeed.push_back(message.roll_rad_s);
	buffer.yawspeed.push_back(message.yaw_rad_s);

	if(mode == buffer_mode::length_mode)
	{
		// If one of the input buffers has reached the maximum length, notify that the buffer is full
		buffer_counter = buffer.find_max_length();
	}
	else if(mode == buffer_mode::time_mode)
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

// Insert an angular attitude message into the buffer
void Buffer::insert(mavsdk::Telemetry::EulerAngle message, uint64_t timestamp)
{
	// thread safe insertion into the buffer, ensures that no insertion occurs when buffer is full
	// will cause the calling thread to wait if it is full

	std::unique_lock<std::mutex> unique_lock(mtx);

	// use lambda function to determine if the buffer is currently full
	// the calling thread waits on the not_full condition variable until the consumer notifies it is empty
	// in summary, if the buffer is full, wait to insert until it has been emptied
	not_full.wait(unique_lock, [this]()
	{
		return buffer_counter < buffer_length; 
	});

	buffer.attitude_time_boot_ms.push_back(timestamp);
	buffer.roll.push_back(message.roll_deg);
	buffer.pitch.push_back(message.pitch_deg);
	buffer.yaw.push_back(message.yaw_deg);

	if(mode == buffer_mode::length_mode)
	{
		// If one of the input buffers has reached the maximum length, notify that the buffer is full
		buffer_counter = buffer.find_max_length();
	}
	else if(mode == buffer_mode::time_mode)
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

// Insert an ActuatorControlTarget into the buffer
void Buffer::insert(mavsdk::Telemetry::ActuatorControlTarget actuator_message, uint64_t timestamp)
{
	// thread safe insertion into the buffer, ensures that no insertion occurs when buffer is full
	// will cause the calling thread to wait if it is full

	std::unique_lock<std::mutex> unique_lock(mtx);

	// use lambda function to determine if the buffer is currently full
	// the calling thread waits on the not_full condition variable until the consumer notifies it is empty
	// in summary, if the buffer is full, wait to insert until it has been emptied
	not_full.wait(unique_lock, [this]()
	{
		return buffer_counter < buffer_length; 
	});

	buffer.actuator_output_ms.push_back(timestamp);
	//Insert the first four actuator outputs into respective actuators
	buffer.actuator0.push_back(actuator_message.controls.at(0));
	buffer.actuator1.push_back(actuator_message.controls.at(1));
	buffer.actuator2.push_back(actuator_message.controls.at(2));
	buffer.actuator3.push_back(actuator_message.controls.at(3));

	if(mode == buffer_mode::length_mode)
	{
		// If one of the input buffers has reached the maximum length, notify that the buffer is full
		buffer_counter = buffer.find_max_length();
	}
	else if(mode == buffer_mode::time_mode)
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
	if(mode == buffer_mode::length_mode)
	{
		buffer_counter = 0;
	}
	else if(mode == buffer_mode::time_mode)
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



