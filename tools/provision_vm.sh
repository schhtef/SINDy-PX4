#!/bin/bash -ex

#We'll need git
sudo apt-get install git

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

#Download QGroundControl and set executable permissions
curl https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
chmod +x ./QGroundControl.AppImage

#Install armadillo
sudo apt get armadillo







