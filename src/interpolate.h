/**
 * @file interpolate.h
 *
 * @brief Interpolation definition
 *
 * Functions for interpolating telemetry
 *
 * @author Stefan Bichlmaier, <bichlmaier.stef@gmail.com>
 *
 */

#ifndef INTERPOLATE_H_
#define INTERPOLATE_H_

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------
#include "buffer.h"
#include "assert.h"
#include <armadillo>
#include "system_identification.h"

Vehicle_States linear_interpolate(Data_Buffer data, int sample_rate);

#endif