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
compute()
{
    while(1)
    {
        //copy input buffer into data array, will block until buffer is full
        input_buffer->clear(data);
        printf("Final Element = %.4f \n", data[input_buffer->buffer_length - 1].first);
        printf("Final timestamp = %d \n", data[input_buffer->buffer_length - 1].second);
    }
}