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
    int number_of_features = candidate_functions.at(0).size(); // number of candidate functions

    lsmr least_squares;
    //vector_to_pointer(candidate_functions);
    least_squares.SetMatrix(candidate_functions);

    // Initialize an m state by n feature (col x row) vector for the coefficients
    std::vector<vector<double>> coefficients(number_of_states, std::vector<double>(number_of_features));
    //least_squares.Solve(number_of_samples, number_of_features, state_derivatives.at(0).data(), coefficients.at(0).data());

    for(int i = 0; i < number_of_states; i++)
    {
        vector<vector<double>> candidate_function_state_i = candidate_functions;
        
        vector<int> coefficient_index;
        //Keep track of which coefficients we're keeping
        for(int k = 0; k < number_of_features; k++)
        {
            coefficient_index.push_back(k);
        }

        for(int j = 0; j < max_iterations; j++)
        {
            //Perform least squares regression
            //TODO need to handle case when candidate_function_State_i is empty
            least_squares.Solve(number_of_samples, candidate_function_state_i.at(0).size(), state_derivatives.at(i).data(), coefficients.at(i).data());
            //Threshold coefficient vector for this current state and update theta_matrix
            threshold(candidate_function_state_i, coefficients.at(i), coefficient_index, 0.01);
            least_squares.SetMatrix(candidate_function_state_i);
        }
    }
    return coefficients;
}

/** Converts a vector of vectors (matrix) into a double pointer (2D array).
 *  This is because the LSMR library uses double**
 * 
 *  @param matrix The 2D vector to be converted
 * 
 * @return 2D double array
 */
double ** SINDy::vector_to_pointer(vector<vector<double>> matrix)
{
    // Initialize vector of pointers
    std::vector<double*> ptr_matrix(matrix.size(), nullptr);
    int i = 0;
    // Iterate through the pointers of matrix with ptr and assign pointer to each row of matrix
    // with data() function, which returns true pointer to a vector element
    for(auto &ptr : matrix)
        ptr_matrix[i++] = ptr.data();
    // Return true pointer
    return ptr_matrix.data();
}

/** Thresholds the values of a 2D vector
 * 
 *  @param coefficients The 2D vector to be thresholded
 *  @param coefficient_threshold The value such that elements smaller than that in the vector will be set to 0
 * 
 * @return thresholded_theta_matrix The matrix of candidate functions with terms removed associated with thresholded coefficients
 */
void SINDy::threshold(vector<vector<double>> &candidate_functions, vector<double> &coefficients, vector<int> &coefficient_index, double coefficient_threshold)
{
    //Threshold values in coefficient matrix and keep columns in theta which are not thresholded

    vector<double>::iterator coeff_it = coefficients.begin();
    vector<int>::iterator candidate_it = coefficient_index.begin();

    while(coeff_it != coefficients.end())
    {
        if(*coeff_it < coefficient_threshold)
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

