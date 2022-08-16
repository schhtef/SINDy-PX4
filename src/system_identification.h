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
#include <string>
#include <math.h>
#include <mlpack/methods/linear_regression/linear_regression.hpp>

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
public:
    SID();
    SID(Buffer *input_buffer_);
    ~SID();

    void compute_thread();
    void start();
    void stop();
    void handle_quit(int sig);

    Vehicle_States interpolate(Data_Buffer data, int sample_rate);
    arma::mat compute_candidate_functions(Vehicle_States states);
    arma::mat STLSQ(arma::mat states, arma::mat candidate_functions, float threshold, float lambda);
    arma::rowvec threshold(arma::vec coefficients, arma::mat candidate_functions, float threshold);
    arma::mat get_derivatives(Vehicle_States states);
    
    bool compute_status;
    bool disarmed;

    string filename;
    uint32_t resampling_rate;
    float STLSQ_threshold; //STLSQ thresholding parameter
    float lambda; //Ridge regression parameter
};

#endif