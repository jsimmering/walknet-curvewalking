#!/bin/bash

echo 'reset following legs: ' "$@"

echo 'reset tibia'
for name in "$@"
do
  rostopic pub --once /phantomx/j_tibia_"${name}"_position_controller/command std_msgs/Float64 "data: -1.2"
done

echo 'reset thigh'
for name in "$@"
do
  rostopic pub --once /phantomx/j_thigh_"${name}"_position_controller/command std_msgs/Float64 "data: -0.5"
done

echo 'reset c1'
for name in "$@"
do
  if [ "$name" = "lf" ] || [ "$name" = "rr" ]; then
    rostopic pub --once /phantomx/j_c1_"${name}"_position_controller/command std_msgs/Float64 "data: 0.0"
  elif [ "$name" = "rf" ] || [ "$name" = "lr" ]; then
    rostopic pub --once /phantomx/j_c1_"${name}"_position_controller/command std_msgs/Float64 "data: -0.6"
  elif [ "$name" = "lm" ] || [ "$name" = "rm" ]; then
    rostopic pub --once /phantomx/j_c1_"${name}"_position_controller/command std_msgs/Float64 "data: 0.3"
  fi
done

# usage:
# ./reset.bash <leg1_name> <leg2_name> ...
# ./reset.bash lm rm lf rr rf lr