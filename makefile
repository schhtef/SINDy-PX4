all: git_submodule SID_control

SID_control: SID_control.cpp serial_port.cpp udp_port.cpp autopilot_interface.cpp
	g++ -g -Wall -I c_library_v2/ SID_control.cpp serial_port.cpp udp_port.cpp autopilot_interface.cpp -o SID_control -lpthread

git_submodule:
	git submodule update --init --recursive

clean:
	 rm -rf *o SID_control