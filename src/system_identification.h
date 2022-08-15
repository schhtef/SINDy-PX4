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

#ifndef SYSTEM_IDENTIFICATION_H_
#define SYSTEM_IDENTIFICATION_H_

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------
#include "autopilot_interface.h"
#include "c_library_v2/common/mavlink.h"
#include "buffer.h"
#include "interpolate.h"
#include <string>
#include <math.h>       /* pow */
#include <mlpack/methods/linear_regression/linear_regression.hpp>

// ----------------------------------------------------------------------------------
//   System Identification Class
// ----------------------------------------------------------------------------------

class SID
{
private:
    Buffer *input_buffer;
    Data_Buffer data;
    Data_Buffer interpolated_data;
    bool time_to_exit = false;
    pthread_t compute_tid = 0;
public:
    SID();
    SID(Buffer *input_buffer_);
    ~SID();

    void compute_thread();
    void start();
    void stop();
    void handle_quit(int sig);

    arma::mat compute_candidate_functions(Data_Buffer data);
    arma::mat get_state_derivatives(Data_Buffer data);
    arma::mat STLSQ(arma::mat states, arma::mat candidate_functions, float threshold, float lambda);
    arma::rowvec threshold(arma::vec coefficients, arma::mat candidate_functions, float threshold);

    bool compute_status;
    bool disarmed;

    string filename;
    uint32_t resampling_rate;
    float STLSQ_threshold; //STLSQ thresholding parameter
    float lambda; //Ridge regression parameter
};

#endif