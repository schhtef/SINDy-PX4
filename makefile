all: git_submodule SID_control

SID_control_SIL: src/SID_control.cpp src/serial_port.cpp src/udp_port.cpp src/buffer.cpp src/autopilot_interface.cpp src/SID.cpp src/logger.cpp src/interpolate.cpp src/lsmr.cpp
	g++ -std=c++2a -g -w -I src/include src/SID_control.cpp src/serial_port.cpp src/udp_port.cpp src/buffer.cpp src/autopilot_interface.cpp  src/SID.cpp src/logger.cpp src/interpolate.cpp src/lsmr.cpp -o build/SID_control_SIL -lpthread

SID_control_HIL: src/SID_control.cpp src/serial_port.cpp src/udp_port.cpp src/buffer.cpp src/autopilot_interface.cpp src/SID.cpp src/logger.cpp src/interpolate.cpp src/lsmr.cpp
	arm-linux-gnueabihf-g++ -std=c++2a -g -w -I src/include src/SID_control.cpp src/serial_port.cpp src/udp_port.cpp src/buffer.cpp src/autopilot_interface.cpp src/SID.cpp src/logger.cpp src/interpolate.cpp src/lsmr.cpp -o build/SID_control_HIL -lpthread

git_submodule:
	git submodule update --init --recursive

clean:
	 rm -rf *o build/SID_control