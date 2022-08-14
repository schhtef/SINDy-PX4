all: git_submodule SID_control

Test: unit_tests.cpp ../src/sindy.cpp ../src/lsmr.cpp
	g++ -std=c++2a -g -Wall -I ../src/ unit_tests.cpp ../src/sindy.cpp ../src/lsmr.cpp  -o unit_tests -lpthread -fopenmp -lmlpack -larmadillo -lboost_serialization

clean:
	 rm -rf *o tests/unit_tests