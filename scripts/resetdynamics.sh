#!/bin/sh


rostopic pub /dolphin/dynamics/state_control uwsim_msgs/state_control '{reset_dynamics: true}'
