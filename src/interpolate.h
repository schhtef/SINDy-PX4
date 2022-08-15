/**
 * @file SID.h
 *
 * @brief system identification computational definition
 *
 * Functions for implementing SINDy
 *
 * @author Stefan Bichlmaier, <bichlmaier.stef@gmail.com>
 *
 */

#ifndef INTERPOLATE_H_
#define INTERPOLATE_H_

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------
#include "autopilot_interface.h"
#include "c_library_v2/common/mavlink.h"
#include "buffer.h"
#include "assert.h"

Data_Buffer interpolate(Data_Buffer data, int sample_rate);

template <typename T, typename U>
void lerp_vector(std::vector<T> y, std::vector<U> x, std::vector<T> &y_result, std::vector<U> &x_result, U start, U end, int sample_rate);

template <typename T, typename U>
T linear_interp(T y0, T y1, U x0, U x1, U x);

#endif