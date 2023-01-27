# SINDy-PX4
Companion computer software for UAV system identification using the [Sparse Identification of Nonlinear Dynamics](https://www.pnas.org/doi/10.1073/pnas.1517384113) (SINDy) method. This project was submitted as my honors thesis in Computer Engineering at the University of Victoria. The thesis can be found in `docs/` for more information.

## Dependencies
SINDy-PX4 has two primary dependencies, armadillo and mavsdk. It assumes at least c++17.

## Building
This project uses CMake as a build manager. You must generate the build files by creating a build directory `mkdir build`, navigating to `/build` then running `cmake ../ <options>`. No options will default to the SIL build.

### CMake Options
There are three main options you'll probably care about. These are SIL_BUILD_TEST, SIL_BUILD, and CMAKE_BUILD_TYPE. These are specified when generating build files by passing the `-D` flag after `cmake ../`. For example, building the for debugging would look like `cmake ../ -DCMAKE_BUILD_TYPE=debug`. 

Setting either SIL_BUILD_TEST=true allows you to build and run unit test, while SIL_BUILD=true will allow you to build the standard SIL model. There are currently plans to specify a HIL_BUILD option once a toolchain for this project is created. 

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
Most runtime arguments are parsed from a .toml file. the only cmd line argument needed is this configuration file's location. It defaults to `config.toml`.

### Config File Location
`-c <file location>`

