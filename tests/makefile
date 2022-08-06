all: git_submodule SID_control

Test: unit_tests.cpp
	g++ -std=c++2a -g -Wall unit_tests.cpp -I ../src/include/eigen-3.4.0 -o unit_tests

clean:
	 rm -rf *o tests/unit_tests