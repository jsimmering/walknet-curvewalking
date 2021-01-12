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


class TestController:
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
        self.swing_trajectory_gen = SimpleSwingTrajectoryGen(self.leg)
        self.stance_trajectory_gen = StanceMovementSimple(self.leg)
        # self.stance_net = StanceMovementBodyModel(self)
        self.alpha_sub = rospy.Subscriber('/phantomx/j_c1_' + self.name + '_position_controller/state',
            JointControllerState, self.leg.c1_callback)
        self.beta_sub = rospy.Subscriber('/phantomx/j_thigh_' + self.name + '_position_controller/state',
            JointControllerState, self.leg.thigh_callback)
        self.gamma_sub = rospy.Subscriber('/phantomx/j_tibia_' + self.name + '_position_controller/state',
            JointControllerState, self.leg.tibia_callback)
        self.kinematic_sub = rospy.Subscriber('/kinematic', Bool, self.kinematic_callback)

    # function for testing the kinematics calculations
    def kinematic_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "kinematic_callback heard that %s", data)
        if data:
            cur_angles = self.leg.compute_inverse_kinematics()
            rospy.loginfo('inverse kinematic angles for current ee_pos current alpha = ' + str(cur_angles[0]) +
                          ' current beta = ' + str(cur_angles[1]) + ' current gamma = ' + str(cur_angles[2]))
            actual_angles = self.leg.get_current_angles()
            rospy.loginfo('actually set angles: ' + str(actual_angles))
            ee_pos = self.leg.ee_position()
            rospy.loginfo('current ee_pos (forward kinematic) = ' + str(ee_pos))
            ee_pos = self.leg.compute_forward_kinematics(cur_angles)
            rospy.loginfo('ee_pos (inverse kinematic angles) = ' + str(ee_pos))
        rospy.loginfo("end of callback")

    def bezier_swing_test_no_vel(self):
        rate = rospy.Rate(CONTROLLER_FREQUENCY)
        while not self.leg.is_ready():
            rospy.loginfo("leg not connected yet! wait...")
            rate.sleep()
        temp = SwingMovementBezier(self.leg)
        temp.swing_start_point = self.leg.ee_position()[0:3]
        temp.swing_target_point = self.leg.compute_forward_kinematics([self.movement_dir * 0.3, 0, -1.0])[0:3]
        # at which position of the interval between the start and the end point the middle point should be placed
        temp.apex_point_ratio = 0.05
        #temp.apex_point_offset = numpy.array([0, 0, 0.4])
        # temp.collision_point = numpy.array([0.8, 0, 0.256])
        # temp.trajectory_generator.bezier_points = temp.compute_bezier_points()
        temp.trajectory_generator.bezier_points = temp.compute_bezier_points_with_joint_angles()
        print(temp.trajectory_generator.bezier_points)
        target_position = bezier(temp.trajectory_generator.bezier_points, 0)
        next_angles = self.leg.compute_inverse_kinematics(target_position)
        rospy.loginfo("target position is: " + str(target_position))
        rospy.loginfo("computed next angles as: " + str(next_angles))
        rospy.loginfo("would reach pos: " + str(self.leg.compute_forward_kinematics(next_angles)))
        self.leg.set_command(next_angles)
        rate.sleep()
        target_position = bezier(temp.trajectory_generator.bezier_points, 0.25)
        next_angles = self.leg.compute_inverse_kinematics(target_position)
        rospy.loginfo("target position is: " + str(target_position))
        rospy.loginfo("computed next angles as: " + str(next_angles))
        rospy.loginfo("would reach pos: " + str(self.leg.compute_forward_kinematics(next_angles)))
        self.leg.set_command(next_angles)
        rate.sleep()
        target_position = bezier(temp.trajectory_generator.bezier_points, 0.5)
        next_angles = self.leg.compute_inverse_kinematics(target_position)
        rospy.loginfo("target position is: " + str(target_position))
        rospy.loginfo("computed next angles as: " + str(next_angles))
        rospy.loginfo("would reach pos: " + str(self.leg.compute_forward_kinematics(next_angles)))
        self.leg.set_command(next_angles)
        rate.sleep()
        target_position = bezier(temp.trajectory_generator.bezier_points, 0.75)
        next_angles = self.leg.compute_inverse_kinematics(target_position)
        rospy.loginfo("target position is: " + str(target_position))
        rospy.loginfo("computed next angles as: " + str(next_angles))
        rospy.loginfo("would reach pos: " + str(self.leg.compute_forward_kinematics(next_angles)))
        self.leg.set_command(next_angles)
        rate.sleep()
        target_position = bezier(temp.trajectory_generator.bezier_points, 1)
        next_angles = self.leg.compute_inverse_kinematics(target_position)
        rospy.loginfo("target position is: " + str(target_position))
        rospy.loginfo("computed next angles as: " + str(next_angles))
        rospy.loginfo("would reach pos: " + str(self.leg.compute_forward_kinematics(next_angles)))
        self.leg.set_command(next_angles)
        rate.sleep()

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
    def manage_walk_bezier(self):
        rate = rospy.Rate(CONTROLLER_FREQUENCY)
        while not self.leg.is_ready():
            rospy.loginfo("leg not connected yet! wait...")
            rate.sleep()
        rospy.loginfo("leg connected start walking")

        alpha = 0.3
        if self.movement_dir == 1:
            alpha = -0.3
        end_point = self.leg.compute_forward_kinematics([alpha, 0, -1.0])
        self.stance_trajectory_gen.set_target_point(end_point)
        while not rospy.is_shutdown():
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
                if self.stance_trajectory_gen.start_point is None:
                    self.stance_trajectory_gen.set_start_point(self.leg.ee_position())
                self.stance_trajectory_gen.stance()
                rate.sleep()
                if self.stance_trajectory_gen.is_finished():
                    self.swing = True
                    self.stance_trajectory_gen.set_start_point(None)

    # function for moving a leg alternating between swing and stance.
    def manage_walk(self):
        rate = rospy.Rate(CONTROLLER_FREQUENCY)
        while not self.leg.is_ready():
            rospy.loginfo("leg not connected yet! wait...")
            rate.sleep()
        rospy.loginfo("leg connected start swing")
        mid_point = self.leg.compute_forward_kinematics([0, -0.75, -1.0])
        self.swing_trajectory_gen.set_mid_point(mid_point)
        end_point = self.leg.compute_forward_kinematics([self.movement_dir * 0.3, 0, -1.0])
        self.swing_trajectory_gen.set_target_point(end_point)
        rospy.loginfo('trajectory: ' + str(self.swing_trajectory_gen.trajectory))
        rospy.loginfo('current angles ' + str(self.leg.get_current_angles()))

        alpha = 0.3
        if self.movement_dir == 1:
            alpha = -0.3
        end_point = self.leg.compute_forward_kinematics([alpha, 0, -1.0])
        self.stance_trajectory_gen.set_target_point(end_point)
        while not rospy.is_shutdown():
            if self.swing:
                if self.swing_trajectory_gen.start_point is None:
                    self.swing_trajectory_gen.set_start_point(self.leg.ee_position())
                    self.swing_trajectory_gen.compute_trajectory_points()
                self.swing_trajectory_gen.move_to_next_point()
                rate.sleep()
                # rospy.loginfo('swing finished is: ' + str(self.swing_trajectory_gen.is_finished()))
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

    # function for executing a single swing movement.
    def manage_swing(self):
        rate = rospy.Rate(CONTROLLER_FREQUENCY)
        while not self.leg.is_ready():
            rospy.loginfo("leg not connected yet! wait...")
            rate.sleep()
        rospy.loginfo("leg connected start swing")
        self.swing_trajectory_gen.set_start_point(self.leg.ee_position())
        mid_point = self.leg.compute_forward_kinematics([0, -0.75, -1.0])
        self.swing_trajectory_gen.set_mid_point(mid_point)
        end_point = self.leg.compute_forward_kinematics([self.movement_dir * 0.3, 0, -1.0])
        self.swing_trajectory_gen.set_target_point(end_point)
        self.swing_trajectory_gen.compute_trajectory_points()
        rospy.loginfo('trajectory: ' + str(self.swing_trajectory_gen.trajectory))
        rospy.loginfo('current angles ' + str(self.leg.get_current_angles()))

        self.swing_trajectory_gen.move_to_next_point()
        #while not rospy.is_shutdown() and not self.swing_trajectory_gen.is_finished():
            #self.swing_trajectory_gen.move_to_next_point()
            #rate.sleep()
            # rospy.loginfo('swing finished is: ' + str(self.swing_trajectory_gen.is_finished()))

    # function for executing a single stance movement.
    def manage_stance(self):
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


if __name__ == '__main__':
    nh = rospy.init_node('test_controller', anonymous=True)
    legController = TestController('rr', nh, True)
    # rospy.spin()
    try:
        # legController.manage_walk()
        legController.manage_walk_bezier()
        # legController.bezier_swing()
        # legController.manage_swing()
        # legController.manage_stance()
    except rospy.ROSInterruptException:
        pass
