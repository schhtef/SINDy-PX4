# SINDy-PX4
Companion computer software for UAV system identification using the [Sparse Identification of Nonlinear Dynamics](https://www.pnas.org/doi/10.1073/pnas.1517384113) (SINDy) method. This project was submitted as my honors thesis in Computer Engineering at the University of Victoria. The thesis can be found in `docs/` for more information.

## Dependencies
Run the provision_vm.sh script in SINDY-PX4/tools to install the necessary packages and dependencies. Running this should install everything you need for development, SIL, and HIL simulations.

## Building
This project uses CMAKE as a build manager. You must generate the build files by creating a build directory `mkdir build`, navigating to `/build` then running `cmake ../`

With the buildfiles generated, run `cmake --build .` while in `/build` to generate the executable.

To enable/disable compiling a suite of test cases, change the `SIL_BUILD_TEST` option to `off` in the root directory `CMakeLists.txt`.

## SIL Run Instructions
1. Download the [QGroundControl .appimage](http://qgroundcontrol.com/downloads/)
2. Clone the PX4-Autopilot repository and complete installation of dependencies by following this [guide](https://dev.px4.io/v1.10_noredirect/en/simulation/gazebo.html)
3. Startup the gazebo simulation by running `make px4_sitl gazebo` within the root directory
4. Run the QGroundControl.appimage
5. Build the project by running `make SID_control_SIL` to build using the c++20 compiler
6. Start SID_Control_SIL by running `./build/SID_control_SIL -u 127.0.0.1 -p 14540 -s 100 -m length -l logs/` from the PX4-SID root directory to run with a buffer size of 100 items and a logfile in the logs/ folder.

## HIL Run Instructions
1. Follow steps 1-4 of the SIL instructions.
2. Connect PX4 to the raspberry pi using a UART.
5. Build using `make SID_control_HIL` in order to use the arm-linux-gnueabihf-g++ compiler for the raspberry pi.
5. Start SID_Control by running `./build/SID_control -u 127.0.0.1 -p 14540 -s 100` from the PX4-SID root directory

## CMD Line Options
For simulation with gazebo, the five important arguments are the UDP port, IP address, buffer length, buffer mode, and logfile location.

### UDP IP
`-u <udp ip address>`

IP address for the UDP connection to a virtual or physical PX4. For simulation on a local machine, use 127.0.0.1
### UDP Port Number
`-p <udp port number>`

Port number for the UDP connection to a virtual or physical PX4. Use 14540 as this is what PX4 uses for auxiliary communication.

### SINDy Buffer Length
`-s <buffer length>`

An integer which denotes how long the input buffer for SINDy is.

### Buffer Mode
`-m <file location>`

Specifies whether to fill the shared buffer with -s number of items or -s number of seconds

### Log File Location
`-l <file location>`

Specifies the file location for the logged model coefficients. It currently names the files automatically based on the flight number. 
