#!/bin/bash
scp build/SID_control_HIL stefan@192.168.0.120:~/PX4-SID
ssh px4-sid-hil "cd ~/PX4-SID; ./SID_control_HIL -d /dev/serial1 -b 57600 -s 100 -l ~/PX4-SID/logs/"