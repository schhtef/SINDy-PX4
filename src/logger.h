/**
 * @file logger.h
 *
 * @brief helper functions for logging data buffer to csv 
 *
 * @author Stefan Bichlmaier, <bichlmaier.stef@gmail.com>
 *
 */

#ifndef logger_H_
#define logger_H_

#define NUMBER_OF_BUFFERS 5
// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------
#include "c_library_v2/common/mavlink.h"
#include "buffer.h"
#include <string>
#include <fstream>
#include <iostream>

using namespace std;

void log_buffer_to_csv(Mavlink_Message_Buffers buffer, string filename);

#endif