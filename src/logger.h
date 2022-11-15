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
#include "buffer.h"
#include <string>
#include <fstream>
#include <iostream>
#include "interpolate.h"

using namespace std;

void log_buffer_to_csv(Data_Buffer telemetry, string filename);

#endif