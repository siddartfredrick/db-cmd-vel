#!/bin/bash

set -e

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------
# echo "This launch file may be changed as needed to launch specific nodes."
#roscore &
#sleep 5

## Launched individual nodes
#rosrun my_package my_node.py &
#rosrun my_package my_node_subscriber.py &
#rosrun my_package cmd_vels.py



## Calls .launch file to invoke the basic cmd_vels node via roslaunch
#roslaunch my_package cmd_vels.launch veh:=$VEHICLE_NAME



## This launches the DT basic publisher/subscribers 
## and the cmd_vels node
#roslaunch my_package multiple_nodes.launch veh:=$VEHICLE_NAME



#----------------------------------------------------------------------------
# Calls the encoder_control node via roslaunch
# Note: The db-read-encoders node MUST be running first.
# See README.md for more information on the db-read-encoders node.
roslaunch my_package encoder_control.launch veh:=$VEHICLE_NAME






