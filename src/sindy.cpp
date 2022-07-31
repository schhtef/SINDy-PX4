/**
 * @file sindy.cpp
 *
 * @brief Implements the SINDy algorithm with the desired optimization process
 * 
 * @author Stefan Bichlmaier, <bichlmaier.stef@gmail.com>
 *
 */

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "sindy.h"

SINDy::SINDy()
{

}

SINDy::~SINDy()
{
    
}


/** Performs the Sequentially Thresholded Least Squares Algorithm from https://arxiv.org/abs/1711.05501
 * 
 *  @param candidate_functions A 2D vector, size lxm containing time series solutions to a set of candidate functions
 *  @param state_derivates A 2D vector, size lxn containing measured time series data
 *  @param max_iterations A limit to the number of iterations STLSQ will run
 *  @param coefficient_threshold The value such that elements smaller than that in the vector will be set to 0
 *  
 *  @return coefficients A 2D vector, size mxn which contains the weights associated with the candidate functions which describe the state_derivates
 */
vector<vector<double>> SINDy::STLSQ(vector<vector<double>> candidate_functions, vector<vector<double>> state_derivatives, int max_iterations, float coefficient_threshold)
{
    int number_of_states = state_derivatives.size();
    int number_of_samples = state_derivatives.at(0).size();
    int number_of_features = candidate_functions.size();

    //LSMR library uses double**, convert vector<vector<double>> to double**
    std::vector<double*> theta_matrix(number_of_features, nullptr);
    int i = 0;
    for(auto &ptr : candidate_functions)
        theta_matrix[i++] = ptr.data();

    lsmr least_squares;
    least_squares.SetMatrix(theta_matrix.data());

    // Initialize an n x m vector for the coefficients
    std::vector<vector<double>> coefficients(number_of_states, std::vector<double>(candidate_functions.size(), 0));
    //least_squares.Solve(number_of_samples, number_of_features, state_derivatives.at(0).data(), coefficients.at(0).data());

    for(int i = 0; i < number_of_states; i++)
    {
        for(int j = 0; j < max_iterations; j++)
        {
            //Perform least squares regression
            least_squares.Solve(number_of_samples, number_of_features, state_derivatives.at(i).data(), coefficients.at(i).data());
            //Threshold coefficient vector for this current state and update theta_matrix
            least_squares.SetMatrix(threshold(coefficients.at(i), theta_matrix, coefficient_threshold).data());
        }
    }
    return coefficients;
}

/** Thresholds the values of a 2D vector
 * 
 *  @param coefficients The 2D vector to be thresholded
 *  @param coefficient_threshold The value such that elements smaller than that in the vector will be set to 0
 * 
 * @return thresholded_theta_matrix The matrix of candidate functions with terms removed associated with thresholded coefficients
 */
std::vector<double*> SINDy::threshold(vector<double> &coefficients, std::vector<double*> theta_matrix, double coefficient_threshold)
{
    if(theta_matrix.size() != coefficients.size())
    {
        fprintf(stderr, "Warning, number of columns in theta and rows in coefficient matrix are unequal\n");
    }
    //Threshold values in coefficient matrix and keep columns in theta which are not thresholded
    std::vector<double*> thresholded_theta_matrix;
    for(int i = 0 ; i < coefficients.size(); i++)
    {
        if(coefficients.at(i) > coefficient_threshold)
        {
            thresholded_theta_matrix.push_back(theta_matrix.at(i));
        }
        else
        {
            coefficients.at(i) = 0;
        }
    }
    return thresholded_theta_matrix;
}

