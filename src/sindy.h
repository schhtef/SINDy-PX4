/**
 * @file sindy.h
 *
 * @brief System Identification of Nonlinear Dynamics
 * 
 * Implements the SINDy algorithm, providing choice of optimizer and candidate functions 
 *
 * @author Stefan Bichlmaier, <bichlmaier.stef@gmail.com>
 *
 */

#ifndef SINDY_H_
#define SINDY_H_

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------
//#include "autopilot_interface.h"
//#include "c_library_v2/common/mavlink.h"
#include "lsmr.h" // Least squares algorithm
#include <vector>
#include <iostream>

using namespace std;
// ----------------------------------------------------------------------------------
//   SINDy Class
// ----------------------------------------------------------------------------------

class SINDy
{
private:

public:
    SINDy();
    ~SINDy();

    std::vector<vector<double>> STLSQ(vector<vector<double>> candidate_functions, vector<vector<double>> state_derivatives, int max_iterations, float threshold);
    std::vector<double*>  threshold(vector<double> &coefficients, std::vector<double*> theta_matrix, double coefficient_threshold);
};

#endif