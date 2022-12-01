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
#include "buffer.h"
#include "regression.h"
#include "interpolate.h"
#include <string>
#include <math.h>
#include <chrono>
#include <armadillo>
#include <thread>



// ----------------------------------------------------------------------------------
//   System Identification Class
// ----------------------------------------------------------------------------------

class SID
{
private:
    Buffer *input_buffer;
    bool time_to_exit = false;
    std::thread compute_thread;

    arma::uvec threshold_vector(arma::vec vector, float threshold, string mode);
    arma::mat compute_candidate_functions(Vehicle_States states);
    arma::mat STLSQ(arma::mat states, arma::mat candidate_functions, float threshold, float lambda);
    arma::rowvec threshold(arma::vec coefficients, arma::mat candidate_functions, float threshold);
    arma::mat get_derivatives(Vehicle_States states);
    void log_coeff(arma::mat matrix, string filename);

public:
    SID();
    SID(Buffer *input_buffer_);
    ~SID();

    void stop();
    void start();
    void handle_quit(int sig);
    void sindy_compute();

    bool compute_status;
    std::atomic<bool> armed;

    string logfile_directory = "../logs";
    int flight_number = 0; //current flight number used for generating a filename
    float STLSQ_threshold; //STLSQ thresholding parameter
    float lambda; //Ridge regression parameter
};

#endif