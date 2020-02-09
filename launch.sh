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

## Calls .launch file to invoke via roslaunch
#roslaunch my_package cmd_vels.launch

# This launches the DT basic publisher/subscribers 
# and the cmd_vels node
roslaunch my_package multiple_nodes.launch
