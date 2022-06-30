#!/bin/bash
scp build/SID_control_HIL stefan@192.168.0.120:~/PX4-SID
ssh px4-sid-hil "cd ~/PX4-SID; ./SID_control_HIL -u 127.0.0.1 -p 14540 -s 100 -l ~/PX4-SID/logs/"