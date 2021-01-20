#!/usr/bin/env python3
import time
from threading import Lock

import rospy
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
from walknet_curvewalking_project.support.constants import CONTROLLER_FREQUENCY

TIMEOUT = 15
tibia_mutex = Lock()
thigh_mutex = Lock()
tibia_joint_values = {}
thigh_joint_values = {}
started = False
c1_pub_list = [rospy.Publisher('/phantomx/j_c1_lf_position_controller/command', Float64, queue_size=1),
               rospy.Publisher('/phantomx/j_c1_lm_position_controller/command', Float64, queue_size=1),
               rospy.Publisher('/phantomx/j_c1_lr_position_controller/command', Float64, queue_size=1),
               rospy.Publisher('/phantomx/j_c1_rf_position_controller/command', Float64, queue_size=1),
               rospy.Publisher('/phantomx/j_c1_rm_position_controller/command', Float64, queue_size=1),
               rospy.Publisher('/phantomx/j_c1_rr_position_controller/command', Float64, queue_size=1)]
thigh_pub_list = [rospy.Publisher('/phantomx/j_thigh_lf_position_controller/command', Float64, queue_size=1),
                  rospy.Publisher('/phantomx/j_thigh_lm_position_controller/command', Float64, queue_size=1),
                  rospy.Publisher('/phantomx/j_thigh_lr_position_controller/command', Float64, queue_size=1),
                  rospy.Publisher('/phantomx/j_thigh_rf_position_controller/command', Float64, queue_size=1),
                  rospy.Publisher('/phantomx/j_thigh_rm_position_controller/command', Float64, queue_size=1),
                  rospy.Publisher('/phantomx/j_thigh_rr_position_controller/command', Float64, queue_size=1)]
tibia_pub_list = [rospy.Publisher('/phantomx/j_tibia_lf_position_controller/command', Float64, queue_size=1),
                  rospy.Publisher('/phantomx/j_tibia_lm_position_controller/command', Float64, queue_size=1),
                  rospy.Publisher('/phantomx/j_tibia_lr_position_controller/command', Float64, queue_size=1),
                  rospy.Publisher('/phantomx/j_tibia_rf_position_controller/command', Float64, queue_size=1),
                  rospy.Publisher('/phantomx/j_tibia_rm_position_controller/command', Float64, queue_size=1),
                  rospy.Publisher('/phantomx/j_tibia_rr_position_controller/command', Float64, queue_size=1)]


def tibia_callback(data, args):
    global tibia_joint_values
    # rospy.loginfo(rospy.get_caller_id() + "I heard from %s that %s", args, data.error)
    tibia_mutex.acquire()
    try:
        tibia_joint_values[args] = data
    finally:
        tibia_mutex.release()


def thigh_callback(data, args):
    global thigh_joint_values
    # rospy.loginfo(rospy.get_caller_id() + "I heard from %s that %s", args, data.error)
    thigh_mutex.acquire()
    try:
        thigh_joint_values[args] = data
    finally:
        thigh_mutex.release()


def make_move(target, joint_values, pub_list, mutex, rate):
    if not rospy.is_shutdown():
        for x in pub_list:
            x.publish(target)
    timeout = time.time() + TIMEOUT
    while not rospy.is_shutdown():
        done = True
        mutex.acquire()
        for key in joint_values:
            if abs(joint_values[key].set_point - target) >= 0.05 or 0.1 < joint_values[key].error or joint_values[
                key].error < -0.1:
                done = False
        mutex.release()
        if done:
            rospy.loginfo("finish movement...")
            break
        if time.time() > timeout:
            rospy.loginfo("timeout...")
            return False
        rate.sleep()
    return True


def talker():
    finished = False
    rate = rospy.Rate(CONTROLLER_FREQUENCY)
    while not rospy.is_shutdown() and not finished:
        finished = True

        rospy.loginfo("move thigh up:")
        if not make_move(-1.0, thigh_joint_values, thigh_pub_list, thigh_mutex, rate):
            finished = False

        rospy.loginfo("move tibia down:")
        if not make_move(-1.2, tibia_joint_values, tibia_pub_list, tibia_mutex, rate):
            finished = False

        rospy.loginfo("straighten thigh:")
        if not make_move(0.5, thigh_joint_values, thigh_pub_list, thigh_mutex, rate):
            finished = False


if __name__ == '__main__':
    rospy.init_node('standup_controller', anonymous=True)
    rospy.Subscriber('/phantomx/j_thigh_lf_position_controller/state', JointControllerState, thigh_callback,
                     callback_args="thigh_lf")
    rospy.Subscriber('/phantomx/j_thigh_lm_position_controller/state', JointControllerState, thigh_callback,
                     callback_args="thigh_lm")
    rospy.Subscriber('/phantomx/j_thigh_lr_position_controller/state', JointControllerState, thigh_callback,
                     callback_args="thigh_lr")
    rospy.Subscriber('/phantomx/j_thigh_rf_position_controller/state', JointControllerState, thigh_callback,
                     callback_args="thigh_rf")
    rospy.Subscriber('/phantomx/j_thigh_rm_position_controller/state', JointControllerState, thigh_callback,
                     callback_args="thigh_rm")
    rospy.Subscriber('/phantomx/j_thigh_rr_position_controller/state', JointControllerState, thigh_callback,
                     callback_args="thigh_rr")
    rospy.Subscriber('/phantomx/j_tibia_lf_position_controller/state', JointControllerState, tibia_callback,
                     callback_args="tibia_lf")
    rospy.Subscriber('/phantomx/j_tibia_lm_position_controller/state', JointControllerState, tibia_callback,
                     callback_args="tibia_lm")
    rospy.Subscriber('/phantomx/j_tibia_lr_position_controller/state', JointControllerState, tibia_callback,
                     callback_args="tibia_lr")
    rospy.Subscriber('/phantomx/j_tibia_rf_position_controller/state', JointControllerState, tibia_callback,
                     callback_args="tibia_rf")
    rospy.Subscriber('/phantomx/j_tibia_rm_position_controller/state', JointControllerState, tibia_callback,
                     callback_args="tibia_rm")
    rospy.Subscriber('/phantomx/j_tibia_rr_position_controller/state', JointControllerState, tibia_callback,
                     callback_args="tibia_rr")

    try:
        talker()
    except rospy.ROSInterruptException:
        pass
