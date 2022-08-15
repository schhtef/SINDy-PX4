all: git_submodule SID_control

SID_control_SIL: src/SID_control.cpp src/serial_port.cpp src/udp_port.cpp src/buffer.cpp src/autopilot_interface.cpp src/interpolate.cpp src/system_identification.cpp 
	g++ -std=c++2a -g -w src/*.cpp -I src/include -o build/SID_control_SIL -lpthread -fopenmp -lmlpack -larmadillo -lboost_serialization

SID_control_HIL: src/SID_control.cpp src/serial_port.cpp src/udp_port.cpp src/buffer.cpp src/autopilot_interface.cpp src/SID.cpp src/interpolate.cpp src/lsmr.cpp
	arm-linux-gnueabihf-g++ -std=c++2a -g -w src/*.cpp -I src/include -o build/SID_control_HIL -lpthread -fopenmp -lmlpack -larmadillo -lboost_serialization

git_submodule:
	git submodule update --init --recursive

clean:
	 rm -rf *o build/SID_control