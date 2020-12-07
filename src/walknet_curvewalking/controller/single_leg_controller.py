#!/usr/bin/env python

import numpy
import rospy
# from walknet_curvewalking.motion_primitives.swing_movement_bezier import SwingMovementBezier
from control_msgs.msg import JointControllerState
from walknet_curvewalking.phantomx.SingleLeg import SingleLeg

from walknet_curvewalking.motion_primitives.SimpleSwingTrajectoryGen import SimpleSwingTrajectoryGen


class SingleLegController:
    def __init__(self, name):
        self.name = name
        self.leg = SingleLeg(name, self.compute_segment_length(), True)
        self.swing_trajectory_gen = SimpleSwingTrajectoryGen(self.leg)
        self.alpha_sub = rospy.Subscriber('/phantomx/j_c1_' + self.name + '_position_controller/state',
            JointControllerState, self.c1_callback)
        self.beta_sub = rospy.Subscriber('/phantomx/j_thigh_' + self.name + '_position_controller/state',
            JointControllerState, self.thigh_callback)
        self.gamma_sub = rospy.Subscriber('/phantomx/j_tibia_' + self.name + '_position_controller/state',
            JointControllerState, self.tibia_callback)

    def c1_callback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard that %s", data)
        self.leg.set_c1(data)

    def thigh_callback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard that %s", data)
        self.leg.set_thigh(data)

    def tibia_callback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard that %s", data)
        self.leg.set_tibia(data)

    def manage_swing(self):
        rate = rospy.Rate(100)  # 100Hz
        while not self.leg.is_ready():
            rospy.loginfo("leg not connected yet! wait...")
            rate.sleep()
        rospy.loginfo("leg connected start swing")
        swing = True
        self.swing_trajectory_gen.set_start_point = self.leg.ee_position()
        self.swing_trajectory_gen.set_target_point = [0.1314795, 0.2563255, -0.1365]  # + TODO
        self.swing_trajectory_gen.compute_trajectory_points()
        while not rospy.is_shutdown():
            self.swing_trajectory_gen.move_to_next_point()
            rate.sleep()  # swing_net.swing_start_point = numpy.array([0., 0., 0.])  # the point where the swing  #
            # phase starts  # swing_net.swing_target_point = numpy.array([1., 0., 0.])  # the point where the swing
            # phase should end  # at which position of the interval between the start and the end point the middle  #
            # point should be placed  # swing_net.apex_point_ratio = 0.5  # the offset that is added to the middle  #
            # point that was computed on the connecting line between start and  # end point using the  #
            # apex_point_ratio concept.  # swing_net.apex_point_offset = numpy.array([0, 0,  # 0.4])  #
            # swing_net.collision_point = numpy.array([0.8, 0, 0.256])  # bezier_points =  #
            # swing_net.compute_bezier_points()  # rospy.loginfo(str(bezier_points))

            # swing_net.move_to_next_point(0.5)

    def compute_segment_length(self):
        # TODO get from TF or URDF?
        c1_pos = [0, 0.1034, 0.001116]
        thigh_pos = [0, 0.1574, 0.001116]
        tibia_pos = [0.0001, 0.22275, -0.00885]
        ee_pos = [0.0015, 0.38631, -0.02038]
        c1_length = numpy.sqrt(
            thigh_pos[0] - c1_pos[0] * thigh_pos[0] - c1_pos[0] + thigh_pos[1] - c1_pos[1] * thigh_pos[1] - c1_pos[1] +
            thigh_pos[2] - c1_pos[2] * thigh_pos[2] - c1_pos[2])
        thigh_length = numpy.sqrt(
            tibia_pos[0] - thigh_pos[0] * tibia_pos[0] - thigh_pos[0] + tibia_pos[1] - thigh_pos[1] * tibia_pos[1] -
            thigh_pos[1] + tibia_pos[2] - thigh_pos[2] * tibia_pos[2] - thigh_pos[2])
        tibia_length = numpy.sqrt(
            ee_pos[0] - tibia_pos[0] * ee_pos[0] - tibia_pos[0] + ee_pos[1] - tibia_pos[1] * ee_pos[1] - tibia_pos[1] +
            ee_pos[2] - tibia_pos[2] * ee_pos[2] - tibia_pos[2])
        return [c1_length, thigh_length, tibia_length]


if __name__ == '__main__':
    rospy.init_node('single_leg_controller', anonymous=True)
    legController = SingleLegController('lm')
    try:
        legController.manage_swing()
    except rospy.ROSInterruptException:
        pass
