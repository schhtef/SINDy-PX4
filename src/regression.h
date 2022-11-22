/**
 * @file regression.h
 *
 * @brief Regression definition
 *
 * Functions for performing regressions for use in STLSQ 
 *
 * @author Stefan Bichlmaier, <bichlmaier.stef@gmail.com>
 *
 */

#ifndef REGRESSION_H_
#define REGRESSION_H_

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------
#include "buffer.h"
#include "assert.h"
#include <armadillo>

arma::vec ridge_regression(arma::mat candidate_functions, arma::rowvec state, float lambda);

#endif