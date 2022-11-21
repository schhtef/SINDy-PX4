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
#include <string>
#include <math.h>
#include <chrono>
#include <armadillo>

// Vehicle states
struct Vehicle_States{
    int num_samples;

    //Common time base
    arma::rowvec time_boot_ms;

    //Bias
    arma::rowvec bias; //Body velocity in X

    //Linear body velocities
    arma::rowvec u; //Body velocity in X
    arma::rowvec v; //Body velocity in Y
    arma::rowvec w; //Body velocity in Z

    //Linear body positions
    arma::rowvec x; //Body position in X
    arma::rowvec y; //Body position in Y
    arma::rowvec z; //Body position in Z

    //Angular body velocities
    arma::rowvec p; //Roll angular velocity [rad/s] 
    arma::rowvec q; //Pitch angular velocity [rad/s]
    arma::rowvec r; //Yaw angular velocity [rad/s]

    //Euler body angles
    arma::rowvec psi; //Roll angle [rad]
    arma::rowvec theta; //Pitch angle [rad]
    arma::rowvec phi; //Yaw angle [rad]

    //Misc
    arma::rowvec alpha; //Angle of attack [rad]
    arma::rowvec beta;
};

// ----------------------------------------------------------------------------------
//   System Identification Class
// ----------------------------------------------------------------------------------

class SID
{
private:
    Buffer *input_buffer;
    bool time_to_exit = false;
    pthread_t compute_tid = 0;

    arma::uvec threshold_vector(arma::vec vector, float threshold, string mode);
    Vehicle_States interpolate(Data_Buffer data, int sample_rate);
    arma::mat compute_candidate_functions(Vehicle_States states);
    arma::mat STLSQ(arma::mat states, arma::mat candidate_functions, float threshold, float lambda);
    arma::rowvec threshold(arma::vec coefficients, arma::mat candidate_functions, float threshold);
    arma::mat get_derivatives(Vehicle_States states);
    void log_coeff(arma::mat matrix, string filename);
    arma::vec SID::ridge_regression(arma::mat candidate_functions, arma::rowvec state, float lambda);

public:
    SID();
    SID(Buffer *input_buffer_);
    ~SID();

    void start();
    void stop();
    void handle_quit(int sig);
    void compute_thread();

    bool compute_status;
    std::atomic<bool> armed;

    string logfile_directory = "../logs";
    int flight_number = 0; //current flight number used for generating a filename
    float STLSQ_threshold; //STLSQ thresholding parameter
    float lambda; //Ridge regression parameter
};

#endif