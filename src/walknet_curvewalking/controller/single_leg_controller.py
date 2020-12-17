#!/usr/bin/env python3

import rospy
import tf
from control_msgs.msg import JointControllerState
from std_msgs.msg import Bool
from walknet_curvewalking.phantomx.SingleLeg import SingleLeg
# from walknet_curvewalking.motion_primitives.swing_movement_bezier import SwingMovementBezier
from walknet_curvewalking.motion_primitives.SimpleSwingTrajectoryGen import SimpleSwingTrajectoryGen
from walknet_curvewalking.motion_primitives.stance_movment_simple import StanceMovementSimple


class SingleLegController:
    def __init__(self, name, note_handle, swing):
        # rospy.init_node('single_leg_controller', anonymous=True)
        self.nh = note_handle
        self.name = name
        self.movement_dir = 1
        if 'l' in self.name:
            rospy.loginfo("leg on left side movement_dir -1")
            self.movement_dir = -1
        self.leg = SingleLeg(name, [0.054, 0.066, 0.16], tf.TransformListener())
        self.swing = swing
        self.swing_trajectory_gen = SimpleSwingTrajectoryGen(self.leg)
        self.stance_trajectory_gen = StanceMovementSimple(self.leg)
        self.alpha_sub = rospy.Subscriber('/phantomx/j_c1_' + self.name + '_position_controller/state',
            JointControllerState, self.leg.c1_callback)
        self.beta_sub = rospy.Subscriber('/phantomx/j_thigh_' + self.name + '_position_controller/state',
            JointControllerState, self.leg.thigh_callback)
        self.gamma_sub = rospy.Subscriber('/phantomx/j_tibia_' + self.name + '_position_controller/state',
            JointControllerState, self.leg.tibia_callback)
        self.kinematic_sub = rospy.Subscriber('/kinematic', Bool, self.kinematic_callback)

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
            rospy.loginfo('inverse kinematic angles for current ee_pos current alpha = ' + str(cur_angles[0]) +
                          ' current beta = ' + str(cur_angles[1]) + ' current gamma = ' + str(cur_angles[2]))
            actual_angles = self.leg.get_current_angles()
            rospy.loginfo('actually set angles: ' + str(actual_angles))
            ee_pos = self.leg.ee_position()
            rospy.loginfo('current ee_pos (forward kinematic) = ' + str(ee_pos))
            ee_pos = self.leg.compute_forward_kinematics(cur_angles)
            rospy.loginfo('ee_pos (inverse kinematic angles) = ' + str(ee_pos))
            # angles = self.leg.compute_inverse_kinematics(numpy.array([-0.1, 0.277, -0.11, 1]))
            # rospy.loginfo('angles to reach position [0.1, 0.277, -0.115, 1]: alpha = ' + str(angles[0]) +
            #               ' beta = ' + str(angles[1]) + ' gamma = ' + str(angles[2]))
            # self.leg.set_command(angles)
            # rate = rospy.Rate(10)
            # rate.sleep()
            # pos = self.leg.compute_forward_kinematics(angles)
            # rospy.loginfo("transformation matrix ee pos in body frame: " + str(pos))
            # ee_pos = self.leg.ee_position()
            # rospy.loginfo('ee_pos = ' + str(ee_pos))
            #
            # tf_pos = self.leg.compute_forward_kinematics_tf()
            # pos = self.leg.compute_forward_kinematics()
            # rospy.loginfo("tf                    ee pos in body frame: " + str(tf_pos))
            # rospy.loginfo("transformation matrix ee pos in body frame: " + str(pos))
        rospy.loginfo("end of callback")

    def manage_walk(self):
        rate = rospy.Rate(10)  # 100Hz
        while not self.leg.is_ready():
            rospy.loginfo("leg not connected yet! wait...")
            rate.sleep()
        rospy.loginfo("leg connected start swing")
        mid_point = self.leg.compute_forward_kinematics([0, -0.75, -1.0])
        self.swing_trajectory_gen.set_mid_point(mid_point)
        end_point = self.leg.compute_forward_kinematics([self.movement_dir * 0.59, 0, -1.0])
        self.swing_trajectory_gen.set_target_point(end_point)
        rospy.loginfo('trajectory: ' + str(self.swing_trajectory_gen.trajectory))
        rospy.loginfo('current angles ' + str(self.leg.get_current_angles()))
        #start_angles = self.leg.compute_inverse_kinematics(cur_ee)
        #mid_angles = self.leg.compute_inverse_kinematics(mid_point)
        #end_angles = self.leg.compute_inverse_kinematics(end_point)
        #rospy.loginfo('should be (IK) ' + str(start_angles))
        #rospy.loginfo('mid point angles: [0, -1, -0.9] set to (IK) ' + str(mid_angles))
        #rospy.loginfo('end point angles: [-0.59, 0, -0.9, 1] set to (IK) ' + str(end_angles))

        alpha = 0.59
        if self.movement_dir == 1:
            alpha = -0.59
        end_point = self.leg.compute_forward_kinematics([alpha, 0, -1.0])
        self.stance_trajectory_gen.set_target_point(end_point)
        while not rospy.is_shutdown():
            if self.swing:
                if self.swing_trajectory_gen.start_point is None:
                    self.swing_trajectory_gen.set_start_point(self.leg.ee_position())
                    self.swing_trajectory_gen.compute_trajectory_points()
                self.swing_trajectory_gen.move_to_next_point()
                rate.sleep()
                rospy.loginfo('swing finished is: ' + str(self.swing_trajectory_gen.is_finished()))
                if self.swing_trajectory_gen.is_finished():
                    self.swing = False
                    self.swing_trajectory_gen.set_start_point(None)
            else:
                if self.stance_trajectory_gen.start_point is None:
                    self.stance_trajectory_gen.set_start_point(self.leg.ee_position())
                self.stance_trajectory_gen.stance()
                rate.sleep()
                if self.stance_trajectory_gen.is_finished():
                    self.swing = True
                    self.stance_trajectory_gen.set_start_point(None)
            # swing_net.swing_start_point = numpy.array([0., 0., 0.])  # the point where the swing  #
            # phase starts  # swing_net.swing_target_point = numpy.array([1., 0., 0.])  # the point where the swing
            # phase should end  # at which position of the interval between the start and the end point the middle  #
            # point should be placed  # swing_net.apex_point_ratio = 0.5  # the offset that is added to the middle  #
            # point that was computed on the connecting line between start and  # end point using the  #
            # apex_point_ratio concept.  # swing_net.apex_point_offset = numpy.array([0, 0,  # 0.4])  #
            # swing_net.collision_point = numpy.array([0.8, 0, 0.256])  # bezier_points =  #
            # swing_net.compute_bezier_points()  # rospy.loginfo(str(bezier_points))
            # swing_net.move_to_next_point(0.5)


if __name__ == '__main__':
    rospy.init_node('single_leg_controller', anonymous=True)
    legController = SingleLegController('lm', True)
    #rospy.spin()
    try:
        legController.manage_walk()
    except rospy.ROSInterruptException:
        pass
