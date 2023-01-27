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

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------
#include "regression.h"

// Ridge regression. Least squares when lambda = 0
arma::vec ridge_regression(arma::mat candidate_functions, arma::rowvec state, float lambda)
{
	//Ridge regression is defined by the coefficients \beta = ((X'X+\lamdaI)^-1)X'y
	//Where X is the design matrix (candidate functions), y are the responses (states), 
	//and lambda is the regression penalty
	//Armadillo's linear solver is used to solve for \beta by arranging the expression as
	//(X'X+\lamdaI)\beta = X'y
	arma::mat A = candidate_functions * candidate_functions.t() + lambda * arma::eye<arma::mat>(candidate_functions.n_rows, candidate_functions.n_rows);
	arma::vec coefficients = arma::solve(A, candidate_functions*state.t());
	return coefficients;
}