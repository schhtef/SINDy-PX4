#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <math.h>
#include "csv.h"
#include <armadillo>
#include "system_identification.h"

using namespace std;
void matrix_to_csv(vector<vector<float>> matrix, string filename, string header);
void matrix_to_csv(vector<float> matrix, string filename, string header);
arma::mat STLSQ(arma::mat states, arma::mat candidate_functions, float threshold, float lambda);
arma::uvec threshold_vector(arma::vec vector, float , string mode);
arma::mat compute_candidate_functions(arma::mat states);

int main()
{

    //Generate simple y = x dataset to verify regression implementation
    arma::mat x(1,100);
    arma::rowvec y(1,100);

    for(int i = 0; i < x.size(); i++)
    {
        x(i) = i;
        y(i) = i;
    }

    arma::vec result = ridge_regression(x,y,0);
    /*
    * Generate derivatives for simple harmonic oscillator
    * dx/dt = -0.1x + 2y
    * dy/dt = -2x -0.1y
    */
/*
    int num_iterations = 1000;
    int num_states = 3;
    int num_features = 10;

    using namespace mlpack::regression;
    arma::mat derivatives(num_states, num_iterations); // The responses, one row for each row in data.
    arma::mat states(num_iterations, num_states); // The dataset itself.
    // Regress.

    double x = -8;
    double y = 8;
    double z = 27;

    double dt = 0.01;

    for ( int i = 0; i < num_iterations; i++ )
    {
        double d_x = 10*(y-x);
        double d_y = x*(28-z)-y;
        double d_z = x*y-(8/3)*z;

        x = x+dt*d_x;
        y = y+dt*d_y;
        z = z+dt*d_z;

        states(i, 0) = x;
        states(i, 1) = y;
        states(i, 2) = z;

        derivatives(0,i) = d_x;
        derivatives(1,i) = d_y;
        derivatives(2,i) = d_z;
    }
*/
    /*
    arma::field<std::string> header(data.n_rows);
    header(0) = "dx";
    header(1) = "dy";
    arma::mat datat = responses.t(); // The dataset itself.
    datat.save("derivatives.csv",arma::csv_ascii);
    states.save("states.csv", arma::csv_ascii);
    

    //LinearRegression lr(data, responses.row(0), 0.05, false); //Initial regression on the candidate functions 
    //lr.Parameters().print();
    //arma::mat candidate_functions = compute_candidate_functions(states);
   //arma::mat ceofficients = STLSQ(derivatives, candidate_functions, 0.5, 0.1);
    //ceofficients.print();


    arma::mat states;
    states.load("lorenz_states.csv");
    arma::mat derivatives;
    derivatives.load("lorenz_derivatives.csv");
    arma::mat candidate_functions = compute_candidate_functions(states);
    arma::mat candt = candidate_functions.t();
    candt.save("candidates.csv", arma::csv_ascii);
    arma::vec time(50);
    arma::mat ceofficients;
    for(int i = 0; i < 50; i++)
    {
        auto t1 = std::chrono::high_resolution_clock::now();
        ceofficients = STLSQ(derivatives.t(), candidate_functions, 0.05, 0.05);
        auto t2 = std::chrono::high_resolution_clock::now();
        auto SINDy_time = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
        time(i) = SINDy_time.count();
        //printf("%d\n", i);
    }
    ceofficients.print();
	std::cout << "Mean SINDy: " << mean(time) << "us\n";
	std::cout << "StDev SINDy: " << stddev(time) << "us\n";



    //Generate data to interpolate
    float dt = 0.1;
    float t = 0.2;
    float start = 0;
    float end = 8;

    std::vector<float> y;
    std::vector<float> x;
    std::vector<float> y0;
    std::vector<float> x0;
    while(t <= 6.28)
    {
        x.push_back(t);
        y.push_back(sin(t));
        t = t+dt;
    }
    std::vector<std::vector<float>> sine({x,y});
    matrix_to_csv(sine, "sine.csv", "x,y,x0,y0");
    lerp_vector(y,x,y0,x0,start,end,20);
    std::vector<std::vector<float>> result({x0,y0});
    matrix_to_csv(result, "interpolation.csv", "x0,y0");
*/
    return 0;
}

void matrix_to_csv(std::vector<float> matrix, string filename, string header)
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

void matrix_to_csv(std::vector<std::vector<float>> matrix, string filename, string header)
{
    std::ofstream myfile;
    myfile.open (filename);
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