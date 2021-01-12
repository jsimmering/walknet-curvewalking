#!/usr/bin/env python3

import rospy
import tf
from control_msgs.msg import JointControllerState
from std_msgs.msg import Bool

from walknet_curvewalking_project.motion_primitives.stance_movement_body_model import StanceMovementBodyModel
from walknet_curvewalking_project.phantomx.SingleLeg import SingleLeg
from walknet_curvewalking_project.motion_primitives.swing_movement_bezier import SwingMovementBezier, bezier
from walknet_curvewalking_project.motion_primitives.SimpleSwingTrajectoryGen import SimpleSwingTrajectoryGen
from walknet_curvewalking_project.motion_primitives.stance_movment_simple import StanceMovementSimple
from walknet_curvewalking_project.support.constants import CONTROLLER_FREQUENCY


class SingleLegController:
    def __init__(self, name, note_handle, swing, robot):
        self.robot = robot
        # rospy.init_node('single_leg_controller', anonymous=True)
        self.nh = note_handle
        self.name = name
        self.movement_dir = 1
        if 'l' in self.name:
            rospy.loginfo("leg on left side movement_dir -1")
            self.movement_dir = -1
        self.leg = SingleLeg(name, [0.054, 0.066, 0.16], tf.TransformListener(), self.movement_dir)
        self.temp = SwingMovementBezier(self.leg)
        self.swing = swing
        # self.swing_trajectory_gen = SimpleSwingTrajectoryGen(self.leg)
        self.stance_trajectory_gen = StanceMovementSimple(self.leg)
        self.stance_net = StanceMovementBodyModel(self)
        self.alpha_sub = rospy.Subscriber('/phantomx/j_c1_' + self.name + '_position_controller/state',
            JointControllerState, self.leg.c1_callback)
        self.beta_sub = rospy.Subscriber('/phantomx/j_thigh_' + self.name + '_position_controller/state',
            JointControllerState, self.leg.thigh_callback)
        self.gamma_sub = rospy.Subscriber('/phantomx/j_tibia_' + self.name + '_position_controller/state',
            JointControllerState, self.leg.tibia_callback)

    def bezier_swing(self):
        rate = rospy.Rate(CONTROLLER_FREQUENCY)
        while not self.leg.is_ready():
            rospy.loginfo("leg not connected yet! wait...")
            rate.sleep()
        self.temp.swing_start_point = self.leg.ee_position()[0:3]
        self.temp.swing_target_point = self.leg.compute_forward_kinematics([self.movement_dir * 0.3, 0, -1.0])[0:3]
        # at which position of the interval between the start and the end point the middle point should be placed
        self.temp.apex_point_ratio = 0.05
        # the offset that is added to the middle point that was computed on the connecting line between start and
        # end point using the apex_point_ratio concept.
        #temp.apex_point_offset = numpy.array([0, 0, 0.4]) # constant is used
        # temp.collision_point = numpy.array([0.8, 0, 0.256])
        # bezier_points = temp.compute_bezier_points()
        # self.temp.trajectory_generator.bezier_points = self.temp.compute_bezier_points()
        self.temp.trajectory_generator.bezier_points = self.temp.compute_bezier_points_with_joint_angles()
        print(self.temp.trajectory_generator.bezier_points)
        while not rospy.is_shutdown() and not self.leg.predictedGroundContact():
            self.temp.move_to_next_point(1)
            rate.sleep()
        self.temp.move_to_next_point(0)
        rate.sleep()
        self.swing = False

    # function for moving a leg alternating between swing and stance.
    def manage_walk_bezier_body_model(self):
        rate = rospy.Rate(CONTROLLER_FREQUENCY)
        # while not self.leg.is_ready():
        #     rospy.loginfo("leg not connected yet! wait...")
        #     rate.sleep()
        # rospy.loginfo("leg connected start walking")
        while not self.robot.walk_motivation and not rospy.is_shutdown():
            rospy.loginfo("no walking motivation...")
            rate.sleep()
        rospy.loginfo("leg connected start walking")

        while not rospy.is_shutdown():
            self.robot.updateStanceBodyModel()
            if self.swing:
                if self.temp.swing_start_point is None:
                    rospy.loginfo("##############################reset swing")
                    self.temp.swing_start_point = self.leg.ee_position()[0:3]
                    self.temp.swing_target_point = self.leg.compute_forward_kinematics(
                        [self.movement_dir * 0.3, 0, -1.0])[0:3]
                    self.temp.apex_point_ratio = 0.05
                    # self.temp.trajectory_generator.bezier_points = self.temp.compute_bezier_points()
                    self.temp.trajectory_generator.bezier_points = self.temp.compute_bezier_points_with_joint_angles()
                self.temp.move_to_next_point(1)
                rate.sleep()
                if self.leg.predictedGroundContact():
                    self.temp.move_to_next_point(0)
                    self.temp.swing_start_point = None
                    rate.sleep()
                    self.swing = False
                # rospy.loginfo('swing finished is: ' + str(self.swing_trajectory_gen.is_finished()))
            else:
                self.stance_net.modulated_routine_function_call()

    # function for moving a leg alternating between swing and stance.
    def manage_walk_bezier(self):
        rate = rospy.Rate(CONTROLLER_FREQUENCY)
        while not self.leg.is_ready() and not rospy.is_shutdown():
            rospy.loginfo("leg not connected yet! wait...")
            rate.sleep()
        rospy.loginfo("leg connected")

        alpha = 0.3
        if self.movement_dir == 1:
            alpha = -0.3
        end_point = self.leg.compute_forward_kinematics([alpha, 0, -1.0])
        self.stance_trajectory_gen.set_target_point(end_point)
        while not rospy.is_shutdown():
            if self.swing:
                rospy.loginfo(str(self.name) + " in swing phase")
                if self.temp.swing_start_point is None:
                    rospy.loginfo("##############################reset swing")
                    self.temp.swing_start_point = self.leg.ee_position()[0:3]
                    self.temp.swing_target_point = self.leg.compute_forward_kinematics(
                        [self.movement_dir * 0.3, 0, -1.0])[0:3]
                    self.temp.apex_point_ratio = 0.05
                    # self.temp.trajectory_generator.bezier_points = self.temp.compute_bezier_points()
                    self.temp.trajectory_generator.bezier_points = self.temp.compute_bezier_points_with_joint_angles()
                self.temp.move_to_next_point(1)
                rate.sleep()
                if self.leg.predictedGroundContact():
                    self.temp.move_to_next_point(0)
                    self.temp.swing_start_point = None
                    rate.sleep()
                    self.swing = False
                # rospy.loginfo('swing finished is: ' + str(self.swing_trajectory_gen.is_finished()))
            else:
                rospy.loginfo(str(self.name) + " in stance phase")
                if self.stance_trajectory_gen.start_point is None:
                    self.stance_trajectory_gen.set_start_point(self.leg.ee_position())
                self.stance_trajectory_gen.stance()
                rate.sleep()
                if self.stance_trajectory_gen.is_finished():
                    self.swing = True
                    self.stance_trajectory_gen.set_start_point(None)

    # function for executing a single stance movement.
    def manage_simple_stance(self):
        rate = rospy.Rate(CONTROLLER_FREQUENCY)
        while not self.leg.is_ready():
            rospy.loginfo("leg not connected yet! wait...")
            rate.sleep()
        rospy.loginfo("leg connected start swing")
        self.stance_trajectory_gen.set_start_point(self.leg.ee_position())
        alpha = 0.3
        if self.movement_dir == 1:
            alpha = -0.3
        end_point = self.leg.compute_forward_kinematics([alpha, 0, -1.0])
        self.stance_trajectory_gen.set_target_point(end_point)
        while not rospy.is_shutdown() and not self.stance_trajectory_gen.is_finished():
            self.stance_trajectory_gen.stance()
            rate.sleep()

    # function for executing a single stance movement.
    def manage_stance(self):
        rate = rospy.Rate(CONTROLLER_FREQUENCY)
        # while not self.leg.is_ready():
        #     rospy.loginfo("leg not connected yet! wait...")
        #     rate.sleep()
        # rospy.loginfo("leg connected start walking")
        while not self.robot.walk_motivation and not rospy.is_shutdown():
            rospy.loginfo("no moving motivation...")
            rate.sleep()
        rospy.loginfo("leg connected start walking")
        input("press any key to performe the next step")
        self.robot.updateStanceBodyModel()
        self.stance_net.modulated_routine_function_call()


if __name__ == '__main__':
    nh = rospy.init_node('single_leg_controller', anonymous=True)
    legController = SingleLegController('rr', nh, True)
    # rospy.spin()
    try:
        # legController.manage_walk()
        legController.manage_walk_bezier()
        # legController.bezier_swing()
        # legController.manage_swing()
        # legController.manage_simple_stance()
    except rospy.ROSInterruptException:
        pass
