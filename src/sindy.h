/**
 * @file sindy.h
 *
 * @brief System Identification of Nonlinear Dynamics
 * 
 * Implements the SINDy algorithm, providing choice of candidate functions 
 *
 * @author Stefan Bichlmaier, <bichlmaier.stef@gmail.com>
 *
 */

#ifndef SINDY_H_
#define SINDY_H_

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------
#include "lsmr.h" // Least squares algorithm
#include <vector>
#include <iostream>
#include <list>

// ----------------------------------------------------------------------------------
//   SINDy Functions
// ----------------------------------------------------------------------------------

std::vector<double> STLSQ(std::vector<std::vector<double>> candidate_functions, std::vector<double> state_derivatives, int max_iterations, double threshold);
void threshold(std::vector<std::vector<double>> &candidate_functions, std::vector<double> &coefficients, std::vector<int> &candidate_coefficients, double coefficient_threshold);

#endif