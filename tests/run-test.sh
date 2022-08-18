#!/bin/bash
scp unit_tests px4-sid-hil:~/Tests
ssh px4-sid-hil "cd ~/PX4-SID; ./unit_tests"