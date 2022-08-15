#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <math.h>       /* pow */
#include <mlpack/methods/linear_regression/linear_regression.hpp>
#include "csv.h"
#include "system_identification.h"

using namespace std;
void matrix_to_csv(vector<vector<double>> matrix, string filename, string header);
void matrix_to_csv(vector<double> matrix, string filename, string header);


int main()
{
    /*
    * Generate derivatives for simple harmonic oscillator
    * dx/dt = -0.1x + 2y
    * dy/dt = -2x -0.1y
    */

    int num_iterations = 100;
    int num_states = 2;
    int num_features = 4;

    using namespace mlpack::regression;
    arma::mat data(num_features, num_iterations); // The dataset itself.
    arma::rowvec responses(num_iterations); // The responses, one row for each row in data.
    // Regress.

    double x = 2;
    double y = 0;
    double dt = 0.01;

    for ( int i = 0; i < num_iterations; i++ )
    {
        double d_x = -1*x+10*x*y;
        double d_y = -2*x-0.1*x*y;

        x = x+dt*d_x;
        y = y+dt*d_y;

        data(0, i) = 1;
        data(1, i) = x;
        data(2, i) = y;
        data(3, i) = x*y;

        responses(i) = d_x;
    }

    LinearRegression lr(data, responses, 0.1, false);
    // Get the parameters, or coefficients.
    arma::vec parameters = lr.Parameters();
    parameters.print();

    //Generate data to interpolate

    return 0;
}

void matrix_to_csv(std::vector<double> matrix, string filename, string header)
{
    std::ofstream myfile;
    myfile.open (filename);

    myfile << header + "\n";
    string row;
    for(int i = 0; i < matrix.size(); i++)
    {
        row = to_string(matrix.at(i));
        myfile <<  row + "\n";
    }
    myfile.close();
}

void matrix_to_csv(std::vector<std::vector<double>> matrix, string filename)
{
    std::ofstream myfile;
    myfile.open (filename);

    string header;
    myfile << header + "\n";
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