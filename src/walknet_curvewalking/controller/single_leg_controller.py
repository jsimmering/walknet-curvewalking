#!/usr/bin/env python

import numpy
import rospy
import tf
# from walknet_curvewalking.motion_primitives.swing_movement_bezier import SwingMovementBezier
from control_msgs.msg import JointControllerState
from std_msgs.msg import Bool
from walknet_curvewalking.phantomx.SingleLeg import SingleLeg
from walknet_curvewalking.motion_primitives.SimpleSwingTrajectoryGen import SimpleSwingTrajectoryGen


class SingleLegController:
    def __init__(self, name):
        self.name = name
        self.leg = SingleLeg(name, [0.052, 0.065, 0.13], True, tf.TransformListener())
        self.swing_trajectory_gen = SimpleSwingTrajectoryGen(self.leg)
        self.alpha_sub = rospy.Subscriber('/phantomx/j_c1_' + self.name + '_position_controller/state',
            JointControllerState, self.leg.c1_callback)
        self.beta_sub = rospy.Subscriber('/phantomx/j_thigh_' + self.name + '_position_controller/state',
            JointControllerState, self.leg.thigh_callback)
        self.gamma_sub = rospy.Subscriber('/phantomx/j_tibia_' + self.name + '_position_controller/state',
            JointControllerState, self.leg.tibia_callback)
        self.kinematic_sub = rospy.Subscriber('/kinematic', Bool, self.kinematic_callback)

    # def kinematic_callback(self, data):
    #     rospy.loginfo(rospy.get_caller_id() + "kinematic_callback heard that %s", data)
    #     if data:
    #         ee_pos = self.leg.ee_position()
    #         rospy.loginfo("ee_pos: " + str(ee_pos))
    #     rospy.loginfo("end of callbach")

    def kinematic_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "kinematic_callback heard that %s", data)
        if data:
            # tf_pos = self.leg.body_c1_transform(self.leg.alpha_forward_kinematics())
            # pos = self.leg.body_c1_transformation(0.3, self.leg.c1_thigh_transformation(-1.0))
            # rospy.loginfo("tf                    thigh pos in body frame: " + str(tf_pos))
            # rospy.loginfo("transformation matrix thigh pos in body frame: " + str(pos))
            # tf_pos = self.leg.body_c1_transform(
            #     self.leg.alpha_forward_kinematics(self.leg.beta_forward_kinematics()))
            # pos = self.leg.body_c1_transformation(0.3, self.leg.c1_thigh_transformation(-1.0,
            #     self.leg.thigh_tibia_transformation(0.3)))
            # rospy.loginfo("tf                    tibia pos in body frame: " + str(tf_pos))
            # rospy.loginfo("transformation matrix tibia pos in body frame: " + str(pos))
            # tf_pos = self.leg.body_c1_transform(
            #     self.leg.alpha_forward_kinematics(self.leg.beta_forward_kinematics([0, 0, 0.13, 1])))
            # pos = self.leg.body_c1_transformation(0.3, self.leg.c1_thigh_transformation(-1.0,
            #     self.leg.thigh_tibia_transformation(0.3, self.leg.tibia_ee_transformation())))
            # rospy.loginfo("tf                    ee pos in body frame: " + str(tf_pos))
            # rospy.loginfo("transformation matrix ee pos in body frame: " + str(pos))
            # tf_pos = self.leg.compute_forward_kinematics_tf()
            # pos = self.leg.compute_forward_kinematics()
            # rospy.loginfo("tf                    ee pos in body frame: " + str(tf_pos))
            # rospy.loginfo("transformation matrix ee pos in body frame: " + str(pos))
            ##################################
            cur_angles = self.leg.compute_inverse_kinematics()
            rospy.loginfo('current alpha = ' + str(cur_angles[0]) +
                          ' current beta = ' + str(cur_angles[1]) + ' current gamma = ' + str(cur_angles[2]))
            ee_pos = self.leg.ee_position()
            rospy.loginfo('current ee_pos = ' + str(ee_pos))
            angles = self.leg.compute_inverse_kinematics(numpy.array([-0.1, 0.277, -0.11, 1]))
            rospy.loginfo('angles to reach position [0.1, 0.277, -0.115, 1]: alpha = ' + str(angles[0]) +
                          ' beta = ' + str(angles[1]) + ' gamma = ' + str(angles[2]))
            self.leg.set_command(angles)
            rate = rospy.Rate(10)
            rate.sleep()
            pos = self.leg.compute_forward_kinematics(angles)
            rospy.loginfo("transformation matrix ee pos in body frame: " + str(pos))
            ee_pos = self.leg.ee_position()
            rospy.loginfo('ee_pos = ' + str(ee_pos))

            tf_pos = self.leg.compute_forward_kinematics_tf()
            pos = self.leg.compute_forward_kinematics()
            rospy.loginfo("tf                    ee pos in body frame: " + str(tf_pos))
            rospy.loginfo("transformation matrix ee pos in body frame: " + str(pos))
        rospy.loginfo("end of callback")


    def manage_swing(self):
        rate = rospy.Rate(100)  # 100Hz
        while not self.leg.is_ready():
            rospy.loginfo("leg not connected yet! wait...")
            rate.sleep()
        rospy.loginfo("leg connected start swing")
        swing = True
        self.swing_trajectory_gen.set_start_point = self.leg.ee_position()
        self.swing_trajectory_gen.set_target_point = [0.1, 0.277, -0.11, 1]  # + TODO
        self.swing_trajectory_gen.compute_trajectory_points()
        rospy.loginfo('trajectory: ' + str(self.swing_trajectory_gen.trajectory))
        #while not rospy.is_shutdown():
        #    self.swing_trajectory_gen.move_to_next_point()
        #    rate.sleep()  # swing_net.swing_start_point = numpy.array([0., 0., 0.])  # the point where the swing  #
            # phase starts  # swing_net.swing_target_point = numpy.array([1., 0., 0.])  # the point where the swing
            # phase should end  # at which position of the interval between the start and the end point the middle  #
            # point should be placed  # swing_net.apex_point_ratio = 0.5  # the offset that is added to the middle  #
            # point that was computed on the connecting line between start and  # end point using the  #
            # apex_point_ratio concept.  # swing_net.apex_point_offset = numpy.array([0, 0,  # 0.4])  #
            # swing_net.collision_point = numpy.array([0.8, 0, 0.256])  # bezier_points =  #
            # swing_net.compute_bezier_points()  # rospy.loginfo(str(bezier_points))

            # swing_net.move_to_next_point(0.5)


# def compute_segment_length(self):
# in meter
# c1_pos = [0, 0.1034, 0.001116]
# thigh_pos = [0, 0.1574, 0.001116]
# tibia_pos = [0.0001, 0.22275, -0.00885]
# ee_pos = [0.0015, 0.38631, -0.02038]
# c1_length = numpy.sqrt(
#    thigh_pos[0] - c1_pos[0] * thigh_pos[0] - c1_pos[0] + thigh_pos[1] - c1_pos[1] * thigh_pos[1] - c1_pos[1] +
#    thigh_pos[2] - c1_pos[2] * thigh_pos[2] - c1_pos[2])
# thigh_length = numpy.sqrt(
#    tibia_pos[0] - thigh_pos[0] * tibia_pos[0] - thigh_pos[0] + tibia_pos[1] - thigh_pos[1] * tibia_pos[1] -
#    thigh_pos[1] + tibia_pos[2] - thigh_pos[2] * tibia_pos[2] - thigh_pos[2])
# tibia_length = numpy.sqrt(
#    ee_pos[0] - tibia_pos[0] * ee_pos[0] - tibia_pos[0] + ee_pos[1] - tibia_pos[1] * ee_pos[1] - tibia_pos[1] +
#    ee_pos[2] - tibia_pos[2] * ee_pos[2] - tibia_pos[2])
# return [c1_length, thigh_length, tibia_length]


if __name__ == '__main__':
    rospy.init_node('single_leg_controller', anonymous=True)
    legController = SingleLegController('lm')
    #rospy.spin()
    try:
        legController.manage_swing()
    except rospy.ROSInterruptException:
        pass
