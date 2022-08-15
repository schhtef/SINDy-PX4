#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <math.h>
#include <mlpack/methods/linear_regression/linear_regression.hpp>
#include "csv.h"
#include "system_identification.h"
//#include "interpolate.h"

using namespace std;
void matrix_to_csv(vector<vector<float>> matrix, string filename, string header);
void matrix_to_csv(vector<float> matrix, string filename, string header);
template <typename T, typename U>
void lerp_vector(std::vector<T> y, std::vector<U> x, std::vector<T> &y_result, std::vector<U> &x_result, U start, U end, int sample_rate);

template <typename T, typename U>
T linear_interp(T y0, T y1, U x0, U x1, U x);

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
    SID SINDy();
    SINDy.STLSQ();


/*
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

template <typename T, typename U>
void lerp_vector(std::vector<T> y, std::vector<U> x, std::vector<T> &y_result, std::vector<U> &x_result, U start, U end, int sample_rate)
{
    if(x.size() != y.size())
    {
        fprintf(stderr, "Caution: X and Y series are different sizes\n");
    }
    //Since we are interpolating to a common time series, the result should be reset until the very last interpolation
    if(x_result.size() != 0)
    {
        x_result.clear();
    }
    // Linearly interpolate local position from first_sample_time, if this vector has the first sample time,
    // first interpolant will just be the first sample
	typename std::vector<T>::iterator y_iterator = y.begin();
	typename std::vector<U>::iterator x_iterator = x.begin();

    float sample_period = 1/(float)sample_rate;
    U interpolant_time = start;
    // Outputs
    //std::vector<T> y_result;
    //std::vector<T> x_result;
    // Loop variables
    T y_interpolant;
    U x_interpolant;

    // Interpolate until we reach the last sample time, or the end of the vector is reached, which is unexpected
    while(interpolant_time <= end)
    {
        // Interpolate consecutive variables
        x_interpolant = interpolant_time;
        y_interpolant = linear_interp(*(y_iterator), (*y_iterator+1),*(x_iterator), (*x_iterator+1), interpolant_time);

        // Push interpolated value into a new vector
        y_result.push_back(y_interpolant);
        x_result.push_back(x_interpolant);

        // Increment interpolant time
        interpolant_time = interpolant_time+(sample_period);

        // If interpolant time is greater than the next sample, increment the iterator
        // If the interpolant time is equal, the next loop will just result in the true sample, no interpolation
        // This covers the extrapolation case a the beginning and end of the vector
        // If the vector's first sample is after the start time, interpolant_time will keep increasing.
        // If the vector's last sample is before the end time, interpolant_time will keep increasing but the iterators will not
        if((interpolant_time > (*x_iterator)) && x_iterator != x.end())
        {
            y_iterator++;
            x_iterator++;
        }
    }
    // Sanity Checks
    if(x_result.size() != y_result.size())
    {
        fprintf(stderr, "Caution: X and Y results are different sizes\n");
    }

    assert(x_result.size() == ((end-start))*sample_rate);
}

template <typename T, typename U>
T linear_interp(T y0, T y1, U x0, U x1, U x)
{
    // Returns y0*(x1-x)/(x1-x0) + y1*(x-x0)/(x1-x0)
    // Type casting with (T) to ensure output is of type T
    T retval = y0*(((float)x1-(float)x)/((float)x1-(float)x0))+y1*(((float)x-(float)x0)/((float)x1-(float)x0));
    return retval;
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