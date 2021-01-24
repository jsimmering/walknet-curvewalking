#!/bin/bash

echo 'reset following legs: ' "$@"

echo 'reset tibia'
for name in "$@"
do
  rostopic pub --once /phantomx/j_tibia_${name}_position_controller/command std_msgs/Float64 "data: -1.2"
done
#for name in "$@"
#do
#  #echo "reset ${name}"
#  if [ "$name" = "lf" ] || [ "$name" = "rr" ]; then
#    rostopic pub --once /phantomx/j_tibia_${name}_position_controller/command std_msgs/Float64 "data: -1.0"
#  elif [ "$name" = "rf" ] || [ "$name" = "lr" ]; then
#    rostopic pub --once /phantomx/j_tibia_${name}_position_controller/command std_msgs/Float64 "data: -1.5"
#  elif [ "$name" = "lm" ] || [ "$name" = "rm" ]; then
#    rostopic pub --once /phantomx/j_tibia_${name}_position_controller/command std_msgs/Float64 "data: -1.2"
#  fi
#done

echo 'reset thigh'
for name in "$@"
do
  rostopic pub --once /phantomx/j_thigh_${name}_position_controller/command std_msgs/Float64 "data: -0.5"
done
#for name in "$@"
#do
#  #echo "reset ${name}"
#  if [ "$name" = "lf" ] || [ "$name" = "rr" ]; then
#    rostopic pub --once /phantomx/j_tibia_${name}_position_controller/command std_msgs/Float64 "data: -1.0"
#  elif [ "$name" = "rf" ] || [ "$name" = "lr" ]; then
#    rostopic pub --once /phantomx/j_tibia_${name}_position_controller/command std_msgs/Float64 "data: -0.5"
#  elif [ "$name" = "lm" ] || [ "$name" = "rm" ]; then
#    rostopic pub --once /phantomx/j_tibia_${name}_position_controller/command std_msgs/Float64 "data: -0.75"
#  fi
#done

echo 'reset c1'
for name in "$@"
do
  #echo "reset ${name}"
  if [ "$name" = "lf" ] || [ "$name" = "rr" ]; then
    rostopic pub --once /phantomx/j_c1_${name}_position_controller/command std_msgs/Float64 "data: 0.0"
    #rostopic pub --once /phantomx/j_thigh_${name}_position_controller/command std_msgs/Float64 "data: -0.25"
    #rostopic pub --once /phantomx/j_tibia_${name}_position_controller/command std_msgs/Float64 "data: -1.25"
  elif [ "$name" = "rf" ] || [ "$name" = "lr" ]; then
    rostopic pub --once /phantomx/j_c1_${name}_position_controller/command std_msgs/Float64 "data: -0.6"
    #rostopic pub --once /phantomx/j_thigh_${name}_position_controller/command std_msgs/Float64 "data: -1.0"
    #rostopic pub --once /phantomx/j_tibia_${name}_position_controller/command std_msgs/Float64 "data: -1.5"
  elif [ "$name" = "lm" ] || [ "$name" = "rm" ]; then
    rostopic pub --once /phantomx/j_c1_${name}_position_controller/command std_msgs/Float64 "data: 0.3"
    #rostopic pub --once /phantomx/j_thigh_${name}_position_controller/command std_msgs/Float64 "data: -0.5"
    #rostopic pub --once /phantomx/j_tibia_${name}_position_controller/command std_msgs/Float64 "data: -1.2"
  fi
done