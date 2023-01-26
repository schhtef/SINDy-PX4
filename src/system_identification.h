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
#include <array>



// ----------------------------------------------------------------------------------
//   System Identification Class
// ----------------------------------------------------------------------------------

class SID
{
private:
    Buffer *input_buffer;
    bool time_to_exit = false;
    std::thread compute_thread;
    std::chrono::_V2::system_clock::time_point epoch;

public:
    SID();
    SID(Buffer *input_buffer_, std::chrono::_V2::system_clock::time_point program_epoch, float stlsq_threshold, float ridge_regression_penalty);
    ~SID();

    void stop();
    void start();
    void handle_quit(int sig);
    void sindy_compute();
    arma::uvec threshold_vector(arma::vec vector, float threshold, std::string mode);
    arma::mat compute_candidate_functions(Vehicle_States states);
    arma::mat compute_candidate_functions(arma::mat states);
    arma::mat STLSQ(arma::mat states, arma::mat candidate_functions, float threshold, float lambda);
    arma::rowvec threshold(arma::vec coefficients, arma::mat candidate_functions, float threshold);
    arma::mat get_derivatives(Vehicle_States states);
    void log_coeff(arma::mat matrix, std::string filename, std::chrono::microseconds sample_time);
    void initialize_logfile(std::string filename);

    bool compute_status;
    std::atomic<bool> armed;

    std::string logfile_directory;
    int flight_number; //current flight number used for generating a filename
    float STLSQ_threshold; //STLSQ thresholding parameter
    float lambda; //Ridge regression parameter
};

#endif