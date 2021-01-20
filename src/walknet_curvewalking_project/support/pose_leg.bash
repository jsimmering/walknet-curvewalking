#!/bin/bash

echo "set following leg: $1"

echo "set tibia to $2"
rostopic pub --once /phantomx/j_tibia_${1}_position_controller/command std_msgs/Float64 "data: $2"

echo "set thigh to $3"
rostopic pub --once /phantomx/j_thigh_${1}_position_controller/command std_msgs/Float64 "data: $3"

echo "set c1 to $4"
rostopic pub --once /phantomx/j_c1_${1}_position_controller/command std_msgs/Float64 "data: $4"