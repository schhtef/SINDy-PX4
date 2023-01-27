#include <catch2/catch_test_macros.hpp>
#include <armadillo>
#include "regression.h"
#include "system_identification.h"
#include "interpolate.h"
//To integrate ODEs to verify STLSQ
#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>
#include <math.h>

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

TEST_CASE( "Size limiting of data buffer" ) {
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
        if(i >= 10 && i <= 90)
        {
            angular_velocity_test_time.push_back(i);
            angular_velocity_test_input.push_back(i);
        }
        if(i >= 20 && i <= 80)
        {
            position_velocity_test_time.push_back(i);
            position_test_input.push_back(i);
        }
        if(i >= 30 && i <= 70)
        {
            actuator_test_time.push_back(i);
            actuator_test_input.push_back(i);
        }
    }

    test_buffer.actuator0 = actuator_test_input;
    test_buffer.actuator1 = actuator_test_input;
    test_buffer.actuator2 = actuator_test_input;
    test_buffer.actuator3 = actuator_test_input;
    test_buffer.actuator_output_ms = actuator_test_time;

    test_buffer.pitchspeed = angular_velocity_test_input;
    test_buffer.rollspeed = angular_velocity_test_input;
    test_buffer.yawspeed = angular_velocity_test_input;
    test_buffer.angular_velocity_time_boot_ms = angular_velocity_test_time;

    test_buffer.roll = attitude_test_input;
    test_buffer.pitch = attitude_test_input;
    test_buffer.yaw = attitude_test_input;
    test_buffer.attitude_time_boot_ms = attitude_test_time;

    test_buffer.x = position_test_input;
    test_buffer.y = position_test_input;
    test_buffer.z = position_test_input;
    test_buffer.position_time_boot_ms = position_velocity_test_time;

    test_buffer.x_m_s = position_test_input;
    test_buffer.y_m_s = position_test_input;
    test_buffer.z_m_s = position_test_input;
    test_buffer.position_time_boot_ms = position_velocity_test_time;

    Vehicle_States test_result = linear_interpolate(test_buffer, 1000);
    REQUIRE(test_result.num_samples == 40); //Check that limiting the range to latest and earliest samples works
    REQUIRE(test_result.time_boot_ms.front() == 30); //Check that first sample time is correct
    REQUIRE(test_result.time_boot_ms.back() == 70); //Check that last sample time is correct
}

TEST_CASE( "STLSQ of lorenz system") {
    using namespace std;
    using namespace boost::numeric::odeint;

    std::vector<float> dx_dt;
    std::vector<float> dy_dt;
    std::vector<float> dz_dt;

    std::vector<float> x_state;
    std::vector<float> y_state;
    std::vector<float> z_state;

    const double sigma = 10.0;
    const double R = 28.0;
    const double b = 8.0 / 3.0;

    typedef boost::array< double , 3 > state_type;

    state_type func = {{ -8.0 , 8.0 , 27 }}; // initial conditions
    runge_kutta_dopri5<state_type> rk;

    integrate_const(
    rk,[&dx_dt, &dy_dt, &dz_dt, sigma, R, b, &x_state, &y_state, &z_state]
    ( const state_type &func , state_type &dxdt , double t )
    {
        dxdt[0] = sigma * ( func[1] - func[0] );
        dxdt[1] = R * func[0] - func[1] - func[0] * func[2];
        dxdt[2] = -b * func[2] + func[0] * func[1];

        dx_dt.push_back(dxdt[0]);
        dy_dt.push_back(dxdt[1]);
        dz_dt.push_back(dxdt[2]);

        x_state.push_back(func[0]);
        y_state.push_back(func[1]);
        z_state.push_back(func[2]);
    }
    , func , 0.0 , 100.0 , 0.001);

	arma::rowvec dx = arma::conv_to<arma::rowvec>::from(dx_dt);
	arma::rowvec dy = arma::conv_to<arma::rowvec>::from(dy_dt);
	arma::rowvec dz = arma::conv_to<arma::rowvec>::from(dz_dt);

	arma::rowvec x = arma::conv_to<arma::rowvec>::from(x_state);
	arma::rowvec y = arma::conv_to<arma::rowvec>::from(y_state);
	arma::rowvec z = arma::conv_to<arma::rowvec>::from(z_state);

    arma::mat derivatives = join_cols(dx, dy, dz);
    arma::mat states = join_cols(x, y, z);

    SID test_sindy;
    arma::mat candidate_functions = test_sindy.compute_candidate_functions(states);
    arma::mat test_result = test_sindy.STLSQ(derivatives, candidate_functions, 0.1, 0.1);

    //Verify dimensions of result are correct
    REQUIRE(test_result.n_cols == 3); //Number of states
    REQUIRE(test_result.n_rows == 10); //Number of candidates

    //Verify results are correct to the nearest integer. This should be reproducible
    REQUIRE(std::round(test_result(1,0)) == -10.0);
    REQUIRE(std::round(test_result(2,0)) == 10.0);
    REQUIRE(std::round(test_result(1,1)) == 28.0);
    REQUIRE(std::round(test_result(2,1)) == -1.0);
    REQUIRE(std::round(test_result(6,1)) == -1.0);
    REQUIRE(std::round(test_result(3,2)) == -3.0);
    REQUIRE(std::round(test_result(5,2)) == 1.0);
}

TEST_CASE( "STLSQ of linear system") {
    using namespace std;
    using namespace boost::numeric::odeint;

    std::vector<float> dx_dt;
    std::vector<float> dy_dt;

    std::vector<float> x_state;
    std::vector<float> y_state;

    //system is dx/dt = -2y
    //          dy/dt = x

    const double alpha = -2;

    typedef boost::array< double , 2 > state_type;

    state_type func = {{ 3.0 , 1.0}}; // initial conditions
    runge_kutta_dopri5<state_type> rk;

    integrate_const(
    rk ,[&dx_dt, &dy_dt, alpha, &x_state, &y_state]
    ( const state_type &func , state_type &dxdt , double t )
    {
        dxdt[0] = alpha * ( func[1] );
        dxdt[1] = func[0];

        dx_dt.push_back(dxdt[0]);
        dy_dt.push_back(dxdt[1]);

        x_state.push_back(func[0]);
        y_state.push_back(func[1]);
    }
    , func , 0.0 , 100.0 , 0.01);

	arma::rowvec dx = arma::conv_to<arma::rowvec>::from(dx_dt);
	arma::rowvec dy = arma::conv_to<arma::rowvec>::from(dy_dt);

	arma::rowvec x = arma::conv_to<arma::rowvec>::from(x_state);
	arma::rowvec y = arma::conv_to<arma::rowvec>::from(y_state);

    arma::mat derivatives = join_cols(dx, dy);
    arma::mat states = join_cols(x, y);

    SID test_sindy;
    arma::mat candidate_functions = test_sindy.compute_candidate_functions(states);
    arma::mat test_result = test_sindy.STLSQ(derivatives, candidate_functions, 0.1, 0.1);

    //Verify dimensions of result are correct
    REQUIRE(test_result.n_cols == 2); //Number of states
    REQUIRE(test_result.n_rows == 6); //Number of candidates

    //Verify results are correct to the nearest integer. This should be reproducible
    REQUIRE(std::round(test_result(2,0)) == -2.0);
    REQUIRE(std::round(test_result(1,1)) == 1.0);
}