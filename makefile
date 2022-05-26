all: git_submodule SID_control

SID_control: src/SID_control.cpp src/serial_port.cpp src/udp_port.cpp src/autopilot_interface.cpp  src/buffer.cpp src/SID.cpp
	g++ -std=c++2a -g -w -I src/include src/SID_control.cpp src/serial_port.cpp src/udp_port.cpp src/autopilot_interface.cpp src/buffer.cpp src/SID.cpp -o build/SID_control -lpthread

git_submodule:
	git submodule update --init --recursive

clean:
	 rm -rf *o build/SID_control