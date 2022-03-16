all: git_submodule SID_control

SID_control: src/SID_control.cpp src/serial_port.cpp src/udp_port.cpp src/autopilot_interface.cpp
	g++ -g -Wall -I src/c_library_v2/ src/SID_control.cpp src/serial_port.cpp src/udp_port.cpp src/autopilot_interface.cpp -o build/SID_control -lpthread

git_submodule:
	git submodule update --init --recursive

clean:
	 rm -rf *o SID_control