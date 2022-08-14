#include <iostream>
#include <sindy.h>
#include <fstream>
#include <vector>
#include <algorithm>
#include "csv.h"
#include <math.h>       /* pow */
#include <mlpack/methods/linear_regression/linear_regression.hpp>


using namespace std;
void matrix_to_csv(vector<vector<double>> matrix, string filename, string header);
void matrix_to_csv(vector<double> matrix, string filename, string header);


int main()
{
    int num_iterations = 100;
    int num_states = 2;
    int num_features = 4;

    using namespace mlpack::regression;
    arma::mat data(num_features, num_iterations); // The dataset itself.
    arma::rowvec responses(num_iterations); // The responses, one row for each row in data.
    // Regress.

    /*
    * Generate derivatives for simple harmonic oscillator
    * dx/dt = -0.1x + 2y
    * dy/dt = -2x -0.1y
    */

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

/*


    std::vector<double> x_dot;
    std::vector<double> y_dot;
    std::vector<double> x_state;
    std::vector<double> y_state;
    std::vector<std::vector<double>> candidate_functions;

    int i;
    //Iterate and update x,y and z locations
    //based upon the Lorenz equations


    std::vector<double> x_coeff = STLSQ(candidate_functions, x_dot, 100, 0.05);
    std::vector<double> y_coeff = STLSQ(candidate_functions, y_dot, 100, 0.01);

    std::vector<double> x_dot;
    std::vector<double> y_dot;
    std::vector<double> x_state;
    std::vector<double> y_state;
    std::vector<std::vector<double>> candidate_functions;

    io::CSVReader<2> in("SHO.csv");
    //in.read_header(io::ignore_no_column, "x", "y");
    double x; double y;
    while (in.read_row(x,y))
    {
        x_state.push_back(x);
        y_state.push_back(y);
    }
    
    std::vector<double> x_coeff = STLSQ(candidate_functions, x_dot, 10, 0.05);
    std::vector<double> y_coeff = STLSQ(candidate_functions, y_dot, 10, 0.05);
    

    std::vector<std::vector<double>> coeff;
    coeff.push_back(x_coeff);
    coeff.push_back(y_coeff);

    std::vector<std::vector<double>> state_derivatives;
    state_derivatives.push_back(x_dot);
    state_derivatives.push_back(y_dot);


    matrix_to_csv(x_dot, "cubic_x_dot.csv", "x_dot");
    matrix_to_csv(x_state, "cubic_x.csv", "x");

    Least squares sanity check

    lsmr least_squares;
    //vector_to_pointer(candidate_functions);
    std::vector<vector<double>> A;
    A.push_back({1,0});
    A.push_back({0,1}); 
    least_squares.SetMatrix(A);
    std::vector<double> b = {10, 5};
    std::vector<double> x(2);
    least_squares.Solve(2, 2, b.data(),x.data());
*/



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