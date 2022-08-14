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

/** Performs the Sequentially Thresholded Least Squares Algorithm from https://arxiv.org/abs/1711.05501
 * 
 *  @param candidate_functions A 2D vector, size lxm containing time series solutions to a set of candidate functions
 *  @param state_derivates A 2D vector, size lxn containing measured time series data
 *  @param max_iterations A limit to the number of iterations STLSQ will run
 *  @param coefficient_threshold The value such that elements smaller than that in the vector will be set to 0
 *  
 *  @return coefficients A 2D vector, size mxn which contains the weights associated with the candidate functions which describe the state_derivates
 */
std::vector<double> STLSQ(std::vector<std::vector<double>> candidate_functions, std::vector<double> state_derivatives, int max_iterations, double coefficient_threshold)
{
    int number_of_states = state_derivatives.size();
    int number_of_samples = state_derivatives.size();
    int number_of_features = candidate_functions.at(0).size(); // number of candidate functions

    lsmr least_squares;
    //vector_to_pointer(candidate_functions);
    least_squares.SetMatrix(candidate_functions);

    // Initialize an nx1 feature (col x row) vector for the coefficients
    std::vector<double> coefficients(number_of_features);
        
    //Keep track of the index corresponding to the coefficients we're keeping
    std::vector<int> coefficient_index;
    for(int k = 0; k < number_of_features; k++)
    {
        coefficient_index.push_back(k);
    }

    for(int j = 0; j < max_iterations; j++)
    {
        //Perform least squares regression
        //TODO need to handle case when candidate_function_State_i is empty
        least_squares.Solve(number_of_samples, candidate_functions.at(0).size(), state_derivatives.data(), coefficients.data());
        //Threshold coefficient vector for this current state and update theta_matrix
        threshold(candidate_functions, coefficients, coefficient_index, coefficient_threshold);
        least_squares.SetMatrix(candidate_functions);
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
void threshold(std::vector<std::vector<double>> &candidate_functions, std::vector<double> &coefficients, std::vector<int> &coefficient_index, double coefficient_threshold)
{
    //Threshold values in coefficient matrix and keep columns in theta which are not thresholded

    std::vector<double>::iterator coeff_it = coefficients.begin();
    std::vector<int>::iterator candidate_it = coefficient_index.begin();

    while(coeff_it != coefficients.end())
    {
        if(std::abs(*coeff_it) < coefficient_threshold)
        {
            std::vector<std::vector<double>>::iterator row_it = candidate_functions.begin();
            std::vector<double>::iterator col_it = (*row_it).begin();

            //Iterate through each row of the matrix and remove the ith element from each one
            while(row_it != candidate_functions.end())
            {
                col_it = col_it+(coeff_it-coefficients.begin());
                (*row_it).erase(col_it);
                row_it++;
                col_it = (*row_it).begin();
            }
            coefficients.erase(coeff_it); //Threshold coefficient
            coefficient_index.erase(candidate_it);
        }
        else{
            //Dont increment if we delete, since the next item becomes the current one
            coeff_it++;
            candidate_it++;
        }
    }
}

