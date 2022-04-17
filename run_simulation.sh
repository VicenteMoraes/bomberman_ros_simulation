#!/bin/bash

XAUTH_LIST=$(xauth list)

# the name bomberman_bomberman will have to be fixed later
			 #-e "GAZEBO_MODEL_PATH=/opt/ros/galactic/share/turtlebot3_gazebo/models" \
			 #-e "TURTLEBOT3_MODEL=burger" \

docker run -ti \
       --net=host \
       -e DISPLAY \
       -v /tmp/.X11-unix \
			 --runtime=nvidia \
			 -v $(pwd)/src:/bomberman_ws \
       bomberman_bomberman bash -c "xauth add $XAUTH_LIST && roscore |& : & bash"
