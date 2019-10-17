#!/bin/bash

set -e

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------
echo "This the baitenberg launch."


roslaunch my_package node.launch veh:=$VEHICLE_NAME
