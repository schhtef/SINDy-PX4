#include <catch2/catch_test_macros.hpp>
#include <armadillo>
#include "regression.h"
#include "system_identification.h"

TEST_CASE( "Regression of linear inputs is 1") {
    arma::mat x(1,100);
    arma::rowvec y(1,100);

    for(int i = 0; i < x.size(); i++)
    {
        x(i) = i;
        y(i) = i;
    }

    arma::vec result = ridge_regression(x,y,0);
    REQUIRE(result.n_rows == 1);
    REQUIRE(result.n_cols == 1);
    REQUIRE(result(0) == 1 );
}

TEST_CASE( "Regression of scaled linear inputs is 2") {
    arma::mat x(1,100);
    arma::rowvec y(1,100);

    for(int i = 0; i < x.size(); i++)
    {
        x(i) = i;
        y(i) = 2*i;
    }

    arma::vec result = ridge_regression(x,y,0);
    REQUIRE(result.n_rows == 1);
    REQUIRE(result.n_cols == 1);
    REQUIRE(result(0) == 2 );
}

TEST_CASE( "Regression of scaled linear inputs is 100") {
    arma::mat x(1,100);
    arma::rowvec y(1,100);

    for(int i = 0; i < x.size(); i++)
    {
        x(i) = i;
        y(i) = i*100;
    }

    arma::vec result = ridge_regression(x,y,0);
    REQUIRE(result.n_rows == 1);
    REQUIRE(result.n_cols == 1);
    REQUIRE(result(0) == 100 );
}

TEST_CASE( "Interpolation of data buffer") {
    Data_Buffer test_buffer;
    std::vector<float> attitude_test_input;
    std::vector<float> angular_velocity_test_input;
    std::vector<float> position_test_input;
    std::vector<float> actuator_test_input;
    std::vector<uint64_t> attitude_test_time;
    std::vector<uint64_t> angular_velocity_test_time;
    std::vector<uint64_t> position_velocity_test_time;
    std::vector<uint64_t> actuator_test_time;

    for(int i = 0; i < 100; i++)
    {
        attitude_test_input.push_back(1);
        attitude_test_time.push_back(i);
        if(i > 10 && i < 90)
        {
            angular_velocity_test_time.push_back(i);
            angular_velocity_test_input.push_back(i);
        }
        if(i > 20 && i < 80)
        {
            position_velocity_test_time.push_back(i);
            position_test_input.push_back(i);
        }
        if(i > 30 && i < 70)
        {
            actuator_test_time.push_back(i);
            actuator_test_input.push_back(i);
        }
    }
    SID test_sindy();
    Vehicle_States test_result = test_sindy.linear_interpolate(test_buffer, 1000);
    //REQUIRE(result(0) == 100 );
}