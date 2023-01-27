/**
 * @file logger.h
 *
 * @brief helper functions for logging data buffer to csv 
 *
 * @author Stefan Bichlmaier, <bichlmaier.stef@gmail.com>
 *
 */

#ifndef LOGGING_H_
#define LOGGING_H_

#define NUMBER_OF_BUFFERS 5
// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------
#include "buffer.h"
#include <string>
#include <fstream>
#include <iostream>
#include <mavsdk/log_callback.h>
#include <armadillo>

void log_buffer_to_csv(Data_Buffer telemetry, std::string filename);
void log_mavlink_info(mavsdk::log::Level level, const std::string& message, const std::string &filename);
void log_coeff(arma::mat matrix, std::string filename, std::chrono::microseconds sample_time);
#endif