# SINDy-PX4
Companion computer software for UAV system identification using the [Sparse Identification of Nonlinear Dynamics](https://www.pnas.org/doi/10.1073/pnas.1517384113) (SINDy) method. This project was submitted as my honors thesis in Computer Engineering at the University of Victoria. The thesis can be found in `docs/` for more information.

## Getting Started
Clone the SINDy-PX4 repo using `git clone --recurse-submodules https://github.com/sbichlmaier/SINDy-PX4.git`. This guide assumes a Ubuntu 20.04 installation.

### Install Dependencies
Install the following packages onto your machine using apt-get.

`sudo apt-get install cmake`

`sudo apt-get install libarmadillo-dev`

Mavsdk must also be installed, however it is not distributed using the default ubuntu sources. Download the latest libmavsdk release for amd64 from the [Mavsdk github](https://github.com/mavlink/MAVSDK/releases). Once downloaded, navigate to the file and run
`sudo apt-get install ./libmavsdk-dev_1.4.10_ubuntu20.04_amd64.deb`

SINDy-PX4 uses armadillo for matrix operations MAVSDK for communication with PX4 autopilots, and CMake as a build system.

### Building
This project uses CMake as a build manager. You must generate the build files by creating a build directory `mkdir build`, navigating to `/build` then running `cmake ../ <options>`. No options will default to the SIL build.

#### CMake Options
There are three main options you'll probably care about. These are SIL_BUILD_TEST, SIL_BUILD, and CMAKE_BUILD_TYPE. These are specified when generating build files by passing the `-D` flag after `cmake ../`. For example, building the for debugging would look like `cmake ../ -DCMAKE_BUILD_TYPE=debug`. 

Setting either SIL_BUILD_TEST=true allows you to build and run unit test, while SIL_BUILD=true will allow you to build the standard SIL model. There are currently plans to specify a HIL_BUILD option once a toolchain for this project is created. 

With the buildfiles generated, run `cmake --build .` while in `/build` to generate the executable.

To enable/disable compiling a suite of test cases, change the `SIL_BUILD_TEST` option to `off` in the root directory `CMakeLists.txt`.

## SIL Run Instructions
1. Download the [QGroundControl .appimage](http://qgroundcontrol.com/downloads/)
2. Clone the PX4-Autopilot repository and complete installation of dependencies by following this [guide](https://dev.px4.io/v1.10_noredirect/en/simulation/gazebo.html)
3. Startup the gazebo simulation by running `make px4_sitl gazebo` within the root directory
4. Run the QGroundControl.appimage

## HIL Run Instructions
1. Follow steps 1-4 of the SIL instructions.
2. Connect PX4 to the raspberry pi using a UART.

## CMD Line Options
Most runtime arguments are parsed from a .toml file. the only cmd line argument needed is this configuration file's location. It defaults to `config.toml`.

### Config File Location
`-c <file location>`

