#!/bin/bash -ex

#We'll need git
sudo apt install git

#Move out of repo directory
cd ~/Documents

#Check if PX4 repo has been cloned yet, if it has, pull from the repo
PX4_REPO="https://github.com/PX4/PX4-Autopilot.git"
if [ -d "PX4-Autopilot" ]; then
    cd PX4-Autopilot
    git pull ${PX4_REPO}
else
    git clone ${PX4_REPO}
    cd PX4-Autopilot
fi

#Run PX4 install script
./Tools/setup/ubuntu.sh --no-nuttx

cd ~/Documents

#Download QGroundControl and set executable permissions
if [-e "QGroundControl.AppImage" ]; then
#if it already exists, do nothing
    :
else
    wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
fi

chmod +x ./QGroundControl.AppImage

#Install armadillo and its dependencies
sudo apt install libarmadillo-dev

#Install boost, needed for gazebo
sudo apt-get install libboost-all-dev

#Install gazebo, needed for sitl testing
sudo apt-get install libgazebo-dev

#Install opencv, needed for gazebo
sudo apt-get install libopencv-dev

#Install raspberry pi compilation toolchain for 64bit
sudo apt-get install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu

#Install openblas for cross compiling armadillo
wget https://github.com/xianyi/OpenBLAS/archive/refs/tags/v0.3.21.zip

#Install cmake
sudo apt install cmake

#Download and install MAVSDK
wget https://github.com/mavlink/MAVSDK/releases/download/v1.4.7/libmavsdk-dev_1.4.7_ubuntu20.04_amd64.deb
sudo dpkg -i libmavsdk-dev_1.4.7_ubuntu20.04_amd64.deb









