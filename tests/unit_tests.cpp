#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <vector>
#include <algorithm>

using namespace std;
//void matrix_to_csv(vector<vector<double>> matrix, string filename);
Eigen::VectorXf STLSQ(Eigen::MatrixXf candidate_functions, Eigen::VectorXf state_derivative, int max_iterations, float coefficient_threshold);
void threshold(Eigen::MatrixXf &candidate_functions, Eigen::VectorXf &coefficients, double coefficient_threshold);

int main()
{
    /*
    * Generate derivatives for simple harmonic oscillator
    * dx/dt = -0.1x + 2y
    * dy/dt = -2x -0.1y
    */
    int num_iterations = 2500;
    int num_states = 4;

    float x = 2;
    float y = 0;

    Eigen::MatrixXf candidate_functions(num_iterations, num_states);
    Eigen::VectorXf x_dot(num_iterations);
    Eigen::VectorXf y_dot(num_iterations);

    float t = 0.01; 
    int i;
    //Iterate and update x,y and z locations
    //based upon the Lorenz equations
    for ( i = 0; i < num_iterations; i++ )
    {
        float d_x = -0.1*x+2*y;
        float d_y = -2*x-0.1*y;
        
        x_dot(i) = d_x;
        y_dot(i) = d_y;

        candidate_functions(i, 0) = 1;
        candidate_functions(i, 1) = x;
        candidate_functions(i, 2) = y;
        candidate_functions(i, 3) = x*y;

        x = x+t*d_x;
        y = y+t*d_y;
    }

    Eigen::VectorXf x_coeff = STLSQ(candidate_functions, x_dot, 10, 0.05);
    Eigen::VectorXf y_coeff = STLSQ(candidate_functions, y_dot, 10, 0.05);

    //matrix_to_csv(state, "SHO_STATE.csv");
    //matrix_to_csv(state_derivatives, "SHO_DERIVATIVES.csv");
    //matrix_to_csv(coeff, "coeff.csv");

    return 0;
}

/*
void matrix_to_csv(vector<vector<double>> matrix, string filename)
{
    std::ofstream myfile;
    myfile.open (filename);

    string header;
    header+="x, y\n";
    myfile << header;
    string row;
    for(int i = 0; i < matrix.at(0).size(); i++)
    {
        row = "";
        for(int j = 0; j < matrix.size(); j++)
        {
            if(j == (matrix.size() - 1))
            {
                row += to_string(matrix.at(j).at(i)) + "\n";
                break;
            }
            row += to_string(matrix.at(j).at(i)) + ",";
        }
        myfile <<  row;
    }
    myfile.close();
}
*/

Eigen::VectorXf STLSQ(Eigen::MatrixXf candidate_functions, Eigen::VectorXf state_derivative, int max_iterations, float coefficient_threshold)
{
	Eigen::VectorXf coefficients(candidate_functions.cols());

	for(int i = 0; i < max_iterations; i++)
	{
        std::cout <<state_derivative.size()<< "\n" <<std::endl;
        std::cout <<candidate_functions.size()<< "\n" <<std::endl;
		coefficients= candidate_functions.bdcSvd().solve(state_derivative);
		threshold(candidate_functions, coefficients, coefficient_threshold);
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
void threshold(Eigen::MatrixXf &candidate_functions, Eigen::VectorXf &coefficients, double coefficient_threshold)
{
    //Threshold values in coefficient matrix and keep columns in theta which are not thresholded
    std::vector<int> indices_to_keep;
	int index = 0;
	for(int i = 0; i < coefficients.size(); i++)
    {
		if(std::abs(coefficients(i)) > coefficient_threshold)
		{
            indices_to_keep.push_back(i);
		}
		index++;
    }

    Eigen::VectorXi indicesToKeepVector(indices_to_keep.size());
    for(size_t j = 0; j < indices_to_keep.size(); j++)
    {
        indicesToKeepVector(j) = indices_to_keep.at(j);
    }
	candidate_functions = candidate_functions(Eigen::all, indicesToKeepVector); // keep columns associated with non-zero indeces, discard others
	coefficients = coefficients(Eigen::all, indicesToKeepVector); // keep columns associated with non-zero indeces, discard others
}