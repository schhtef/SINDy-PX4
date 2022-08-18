#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <math.h>
#include <mlpack/methods/linear_regression/linear_regression.hpp>
#include "csv.h"
//#include "interpolate.h"

using namespace std;
void matrix_to_csv(vector<vector<float>> matrix, string filename, string header);
void matrix_to_csv(vector<float> matrix, string filename, string header);
arma::mat STLSQ(arma::mat states, arma::mat candidate_functions, float threshold, float lambda);
arma::uvec threshold_vector(arma::vec vector, float , string mode);
arma::mat compute_candidate_functions(arma::mat states);

int main()
{
    /*
    * Generate derivatives for simple harmonic oscillator
    * dx/dt = -0.1x + 2y
    * dy/dt = -2x -0.1y
    */
/*
    int num_iterations = 25;
    int num_states = 2;
    int num_features = 3;

    using namespace mlpack::regression;
    arma::mat data(num_features, num_iterations); // The dataset itself.
    arma::mat responses(num_states, num_iterations); // The responses, one row for each row in data.
    arma::mat states(num_iterations, num_states); // The dataset itself.
    // Regress.

    double x = 2;
    double y = 0;
    double dt = 0.01;

    for ( int i = 0; i < num_iterations; i++ )
    {
        double d_x = -0.1*x + 2*y;
        double d_y = -2*x - 0.1*y;

        x = x+dt*d_x;
        y = y+dt*d_y;

        data(0, i) = 1;
        data(1, i) = x;
        data(2, i) = y;
        //data(3, i) = x*y;

        states(i, 0) = x;
        states(i, 1) = y;

        responses(0,i) = d_x;
        responses(1,i) = d_y;

    }
    */
    /*
    arma::field<std::string> header(data.n_rows);
    header(0) = "dx";
    header(1) = "dy";
    arma::mat datat = responses.t(); // The dataset itself.
    datat.save("derivatives.csv",arma::csv_ascii);
    states.save("states.csv", arma::csv_ascii);
    */
    //LinearRegression lr(data, responses.row(0), 0.05, false); //Initial regression on the candidate functions 
    //lr.Parameters().print();

    //arma::mat ceofficients = STLSQ(responses, data, 0.05, 0.05);
    //ceofficients.print();


    arma::mat states;
    states.load("lorenz_states.csv");
    arma::mat derivatives;
    derivatives.load("lorenz_derivatives.csv");
    arma::mat candidate_functions = compute_candidate_functions(states);
    arma::mat candt = candidate_functions.t();
    candt.save("candidates.csv", arma::csv_ascii);
    arma::vec time(1000);
    for(int i = 0; i < 1000; i++)
    {
        auto t1 = std::chrono::high_resolution_clock::now();
        arma::mat ceofficients = STLSQ(derivatives.t(), candidate_functions, 0.05, 0.05);
        auto t2 = std::chrono::high_resolution_clock::now();
        auto SINDy_time = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
        time(i) = SINDy_time.count();
        printf("%d\n", i);
    }

	std::cout << "Mean SINDy: " << mean(time) << "us\n";
	std::cout << "StDev SINDy: " << stddev(time) << "us\n";
    

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

arma::mat
compute_candidate_functions(arma::mat states)
{
	//Compute desired candidate functions from input buffer
	//Generate ones
	int num_features = (states.n_cols+1)*(states.n_cols+2)/2; //8*(8-1)/2
    arma::mat candidate_functions(num_features, states.n_rows);

	arma::rowvec bias(states.n_rows);
	bias.ones();
	//join states into matrix so we can iterate over them all
	arma::mat state_matrix = states.t();
    state_matrix = arma::join_vert(bias, state_matrix);
	//Index to keep track of insertion into kandidate function6
	int candidate_index = 0;
	//Do for all columns
	for(int i = 0; i <state_matrix.n_cols; i++)
	{
		//For each state, multiply by all others
		for(int j = 0; j < state_matrix.n_rows; j++)
		{
			for(int k = j; k < state_matrix.n_rows; k++)
			{
				candidate_functions(candidate_index,i) = state_matrix(j,i)*state_matrix(k,i);
				candidate_index++;
			}
		}
		candidate_index = 0;
	}
	assert(candidate_functions.n_rows == num_features);
	return candidate_functions;
}

arma::mat STLSQ(arma::mat states, arma::mat candidate_functions, float threshold, float lambda)
{
	//states are row indexes
	//features are row indexes
	//time domain samples are column indexes
	bool notConverged = true;
	int iteration;
	int max_iterations = 10;

    //Normalize candidates with respect to stdev
    //candidate_functions = candidate_functions/candidate_functions.max();

	using namespace mlpack::regression;
	//To store result of STLSQ
	arma::mat coefficients(candidate_functions.n_rows, states.n_rows);
	
	//Do STLSQ for each state
	for(int i = 0; i < states.n_rows; i++)
	{
        iteration = 0;
        notConverged = true;
		//Keep track of which candidate functions have been discarded, so we can match resulting coefficents to candidate functions
		arma::uvec coefficient_indexes(candidate_functions.n_rows);
		//For each state, fill the column with indexes 0 to number of candidate functions
		for(int j = 0; j < candidate_functions.n_rows; j++)
		{
			coefficient_indexes(j) = j;
		}
        arma::mat loop_candidate_functions = candidate_functions; //Keep copy since we will delete portions when thresholding
		arma::rowvec state = states.row(i); //Get derivatives for current state
		LinearRegression lr(candidate_functions, state, lambda, false); //Initial regression on the candidate functions
		arma::vec loop_coefficients = lr.Parameters();
		//Do subsequent regressions until converged
		while(notConverged && iteration < max_iterations)
		{
			arma::uvec below_index = threshold_vector(loop_coefficients, threshold, "below"); //Find indexes of coefficients which are lower than the threshold value
			coefficient_indexes.shed_rows(below_index); //Remove indexes which correspond to thresholded values
            loop_candidate_functions.shed_rows(below_index);
			arma::uvec above_index = threshold_vector(loop_coefficients, threshold, "above"); //Find indexes of coefficients which are higher than the threshold value
            //lr.Train(candidate_functions.rows(above_index), state, false); //Regress again on thresholded candidate functions
			lr.Train(loop_candidate_functions, state, false); //Initial regression on the candidate functions
            //Check if coefficient vector has changed in size since last iteration
			if(lr.Parameters().size() == loop_coefficients.size())
			{
				//notConverged = false; //If thresholding hasn't shrunk the coefficient vector, we have converged
			}
			loop_coefficients = lr.Parameters();
			iteration++; //Keep track of iteration number
		}

		if(loop_coefficients.size() == 0)
		{
			fprintf(stderr, "Thresholding parameter set too low and removed all coefficients in state %d\n", i);
			coefficients.col(i).zeros(); //Set all coefficients to zero and don't attempt to match indexes
			break;
		}
		
		//Match coefficients to their candidate functions
		arma::vec state_coefficients = arma::zeros(candidate_functions.n_rows);
        //std::assert(coefficient_indexes.n_rows == loop_coefficients.n_rows);
		for(int k = 0; k < coefficient_indexes.n_rows; k++)
		{
			state_coefficients(coefficient_indexes(k)) = loop_coefficients(k);
		}
		//loop coefficients is too small for the coefficient matrix
		coefficients.col(i) = state_coefficients; //Solution for current state
	}
	return coefficients;
}

arma::uvec
threshold_vector(arma::vec vector, float threshold, string mode)
{
    std::vector<uint> index;
    if(mode == "below")
    {
        for(int i = 0; i < vector.size(); i++)
        {
            if(std::abs(vector(i)) < threshold)
            {
                index.push_back(i);
            }
        }
    }
    else if(mode == "above")
    {
        for(int i = 0; i < vector.size(); i++)
        {
            if(std::abs(vector(i)) > threshold)
            {
                index.push_back(i);
            }
        }        
    }
    arma::uvec thresholded_indexes = arma::conv_to<arma::uvec>::from(index);
    return thresholded_indexes;
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