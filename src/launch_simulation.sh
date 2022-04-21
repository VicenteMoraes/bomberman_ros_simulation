#!/bin/bash

morse import bomberman_ws/bomberman
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/bomberman_ws/bomberman/
morse run bomberman & roslaunch morse_2dnav nav.launch
