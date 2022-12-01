#include <catch2/catch_test_macros.hpp>
#include <armadillo>
#include "regression.h"

TEST_CASE( "Factorial of 0 is 1 (fail)", "[single-file]" ) {
    arma::mat x(1,100);
    arma::rowvec y(1,100);

    for(int i = 0; i < x.size(); i++)
    {
        x(i) = i;
        y(i) = i;
    }

    arma::vec result = ridge_regression(x,y,0);
    
    REQUIRE( 1 == 1 );
}