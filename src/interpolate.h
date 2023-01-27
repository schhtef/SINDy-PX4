/**
 * @file interpolate.h
 *
 * @brief data buffer definition
 *
 * Functions for collecting telemetry in a buffer suitable for SINDy
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
#include <vector>       // std::vector
#include <armadillo>    // std::copy
#include <mavsdk/mavsdk.h> // general mavlink header
#include <mavsdk/plugins/telemetry/telemetry.h> // telemetry plugin

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

    //Actuator output duty cycles, 1000us -> 0% 2000us -> 100% 
    arma::rowvec actuator0; //Actuator 0 duty cycle [us]
    arma::rowvec actuator1; //Actuator 1 duty cycle [us]
    arma::rowvec actuator2; //Actuator 2 duty cycle [us]
    arma::rowvec actuator3; //Actuator 3 duty cycle [us]

    arma::rowvec alpha; //Angle of attack [rad]
    arma::rowvec beta; //Sideslip angle [rad]
};

Vehicle_States linear_interpolate(Data_Buffer data, int sample_rate);

#endif