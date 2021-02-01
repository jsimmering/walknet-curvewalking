#!/usr/bin/env python3
import rospy
import tf
from control_msgs.msg import JointControllerState

import walknet_curvewalking_project.phantomx.RobotSettings as RSTATIC
from walknet_curvewalking_project.motion_primitives.stance_movement_body_model import StanceMovementBodyModel
from walknet_curvewalking_project.motion_primitives.stance_movment_simple import StanceMovementSimple
from walknet_curvewalking_project.motion_primitives.swing_movement_bezier import SwingMovementBezier
from walknet_curvewalking_project.phantomx.SingleLeg import SingleLeg


class SingleLegController:
    def __init__(self, name, note_handle, swing, robot):
        self.robot = robot
        self.name = name
        self.nh = note_handle
        self.rate = rospy.Rate(RSTATIC.controller_frequency)
        if 'l' in self.name:
            rospy.loginfo("leg on left side movement_dir 1")
            self.movement_dir = 1
        else:
            rospy.loginfo("leg on left side movement_dir -1")
            self.movement_dir = -1
        self.leg = SingleLeg(name, [0.054, 0.066, 0.16], tf.TransformListener(), self.movement_dir)
        self.temp = SwingMovementBezier(self.leg)
        self.swing = swing
        self.stance_trajectory_gen = StanceMovementSimple(self.leg)
        self.init_pos = None

        # self.target_pos = None
        if self.name == "lf" or self.name == "rf":
            self.target_pos = RSTATIC.front_initial_aep.copy()
        elif self.name == "lm" or self.name == "rm":
            self.target_pos = RSTATIC.middle_initial_aep.copy()
        elif self.name == "lr" or self.name == "rr":
            self.target_pos = RSTATIC.hind_initial_aep.copy()
        self.target_pos[1] = self.target_pos[1] * self.movement_dir
        rospy.loginfo("leg " + str(self.name) + " target_pos = " + str(self.target_pos))

        if self.robot is None:
            self.stance_net = None
        else:
            self.stance_net = StanceMovementBodyModel(self)
        self.alpha_sub = rospy.Subscriber('/phantomx/j_c1_' + self.name + '_position_controller/state',
            JointControllerState, self.leg.c1_callback)
        self.beta_sub = rospy.Subscriber('/phantomx/j_thigh_' + self.name + '_position_controller/state',
            JointControllerState, self.leg.thigh_callback)
        self.gamma_sub = rospy.Subscriber('/phantomx/j_tibia_' + self.name + '_position_controller/state',
            JointControllerState, self.leg.tibia_callback)

    def set_init_pos(self, p):
        self.init_pos = p
        rospy.loginfo(self.name + ": set init pos to P = " + str(p))
        rospy.loginfo(self.name + ": set init pos = " + str(self.init_pos))

    def bezier_swing(self):
        while not self.leg.is_ready() and not rospy.is_shutdown():
            rospy.loginfo("leg not connected yet! wait...")
            self.rate.sleep()
        self.temp.swing_start_point = self.leg.ee_position()[0:3]
        # self.temp.swing_target_point = self.leg.compute_forward_kinematics([self.movement_dir * 0.3, 0, -1.0])[0:3]

        # self.temp.swing_target_point = self.leg.compute_inverse_kinematics(target_pos)
        self.temp.swing_target_point = self.target_pos
        # at which position of the interval between the start and the end point the middle point should be placed
        self.temp.apex_point_ratio = 0.015
        # the offset that is added to the middle point that was computed on the connecting line between start and
        # end point using the apex_point_ratio concept.
        # temp.apex_point_offset = numpy.array([0, 0, 0.4]) # constant is used
        # temp.collision_point = numpy.array([0.8, 0, 0.256])
        # bezier_points = temp.compute_bezier_points()
        # self.temp.trajectory_generator.bezier_points = self.temp.compute_bezier_points()
        self.temp.trajectory_generator.bezier_points = self.temp.compute_bezier_points_with_joint_angles()
        print(self.temp.trajectory_generator.bezier_points)
        while not rospy.is_shutdown() and not self.leg.predictedGroundContact():
            self.temp.move_to_next_point(1)
            self.rate.sleep()
        self.temp.move_to_next_point(0)
        self.rate.sleep()
        self.swing = False

    # function for moving a leg alternating between swing and stance.
    def manage_walk_bezier_body_model(self):
        # while not self.leg.is_ready():
        #     rospy.loginfo("leg not connected yet! wait...")
        #     rate.sleep()
        # rospy.loginfo("leg connected start walking")
        while not self.robot.walk_motivation and not rospy.is_shutdown():
            rospy.loginfo("no walking motivation...")
            self.rate.sleep()
        rospy.loginfo("leg connected start walking")

        while not rospy.is_shutdown():
            self.robot.updateStanceBodyModel()
            if self.swing:
                if self.temp.swing_start_point is None:
                    rospy.loginfo("##############################reset swing")
                    self.temp.swing_start_point = self.leg.ee_position()[0:3]
                    # self.temp.swing_target_point = self.leg.compute_forward_kinematics(
                    #    [self.movement_dir * 0.3, 0, -1.0])[0:3]
                    self.temp.swing_target_point = self.target_pos
                    self.temp.apex_point_ratio = 0.015
                    # self.temp.trajectory_generator.bezier_points = self.temp.compute_bezier_points()
                    self.temp.trajectory_generator.bezier_points = self.temp.compute_bezier_points_with_joint_angles()
                self.temp.move_to_next_point(1)
                self.rate.sleep()
                if self.leg.predictedGroundContact():
                    self.temp.move_to_next_point(0)
                    self.temp.swing_start_point = None
                    self.rate.sleep()
                    self.swing = False
                # rospy.loginfo('swing finished is: ' + str(self.swing_trajectory_gen.is_finished()))
            else:
                self.stance_net.modulated_routine_function_call()

    # function for executing a single step in a stance movement.
    def manage_walk(self):
        if not self.robot.walk_motivation or rospy.is_shutdown():
            rospy.loginfo("no moving motivation or shutdown...")
            return
        else:
            if self.swing:
                rospy.loginfo(self.name + ": execute swing step.")
                if self.temp.swing_start_point is None:
                    rospy.loginfo("##############################reset swing")
                    self.temp.swing_start_point = self.leg.ee_position()[0:3]
                    self.temp.swing_target_point = self.target_pos
                    # self.temp.swing_target_point = self.leg.compute_forward_kinematics(
                    #                                [self.movement_dir * 0.3, -0.5, -1.2])[0:3]
                    self.temp.apex_point_ratio = 0.015
                    # self.temp.trajectory_generator.bezier_points = self.temp.compute_bezier_points()
                    self.temp.trajectory_generator.bezier_points = self.temp.compute_bezier_points_with_joint_angles()
                self.temp.move_to_next_point(1)
                self.rate.sleep()
                if self.leg.predictedGroundContact():
                    self.temp.move_to_next_point(0)
                    self.temp.swing_start_point = None
                    # self.rate.sleep()
                    self.swing = False
                # rospy.loginfo('swing finished is: ' + str(self.swing_trajectory_gen.is_finished()))
            else:
                rospy.loginfo(self.name + ": execute stance step.")
                self.stance_net.modulated_routine_function_call()
                if self.leg.reached_pep():
                    rospy.loginfo(self.name + ": reached_pep. switch to swing mode.")
                    self.stance_net.reset_stance_trajectory()
                    # self.rate.sleep()
                    self.swing = True

    # function for moving a leg alternating between swing and stance.
    def manage_walk_bezier(self):
        while not self.leg.is_ready() and not rospy.is_shutdown():
            rospy.loginfo("leg not connected yet! wait...")
            self.rate.sleep()
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
                    self.temp.apex_point_ratio = 0.015
                    # self.temp.trajectory_generator.bezier_points = self.temp.compute_bezier_points()
                    self.temp.trajectory_generator.bezier_points = self.temp.compute_bezier_points_with_joint_angles()
                self.temp.move_to_next_point(1)
                self.rate.sleep()
                if self.leg.predictedGroundContact():
                    self.temp.move_to_next_point(0)
                    self.temp.swing_start_point = None
                    self.rate.sleep()
                    self.swing = False
                # rospy.loginfo('swing finished is: ' + str(self.swing_trajectory_gen.is_finished()))
            else:
                rospy.loginfo(str(self.name) + " in stance phase")
                if self.stance_trajectory_gen.start_point is None:
                    self.stance_trajectory_gen.set_start_point(self.leg.ee_position())
                self.stance_trajectory_gen.stance()
                self.rate.sleep()
                if self.stance_trajectory_gen.is_finished():
                    self.swing = True
                    self.stance_trajectory_gen.set_start_point(None)

    # function for executing a single stance movement.
    def manage_simple_stance(self):
        while not self.leg.is_ready():
            rospy.loginfo("leg not connected yet! wait...")
            self.rate.sleep()
        rospy.loginfo("leg connected start swing")
        self.stance_trajectory_gen.set_start_point(self.leg.ee_position())
        alpha = 0.3
        if self.movement_dir == 1:
            alpha = -0.3
        end_point = self.leg.compute_forward_kinematics([alpha, 0, -1.0])
        self.stance_trajectory_gen.set_target_point(end_point)
        while not rospy.is_shutdown() and not self.stance_trajectory_gen.is_finished():
            self.stance_trajectory_gen.stance()
            self.rate.sleep()

    # function for executing a single stance movement.
    def manage_stance_movement(self):
        # while not self.leg.is_ready():
        #     rospy.loginfo("leg not connected yet! wait...")
        #     rate.sleep()
        # rospy.loginfo("leg connected start walking")
        while not self.robot.walk_motivation and not rospy.is_shutdown():
            rospy.loginfo("no moving motivation...")
            self.rate.sleep()
        rospy.loginfo("leg connected start walking")
        while not rospy.is_shutdown():
            # input("press any key to performe the next step for " + str(self.name) + " leg.")
            # self.robot.updateStanceBodyModel()
            self.stance_net.modulated_routine_function_call()

    # function for executing a single step in a stance movement.
    def manage_stance(self):
        if not self.robot.walk_motivation or rospy.is_shutdown():
            rospy.loginfo("no moving motivation or shutdown...")
            return
        else:
            rospy.loginfo(self.name + ": leg connected start walking. Swing = " + str(self.swing))
            self.stance_net.modulated_routine_function_call()
            if self.leg.reached_pep():
                rospy.loginfo(self.name + ": reached pep. swing will be set to True")
                self.stance_net.reset_stance_trajectory()
                self.rate.sleep()
                self.swing = True

    # function for executing a single step in a stance movement.
    def move_leg_to(self, p=None):
        # rate = rospy.Rate(RSRATIC.controller_frequency)
        # while not rospy.is_shutdown() and not self.leg.is_target_reached():
        #     angles = self.leg.compute_inverse_kinematics(p)
        #     self.leg.set_command(angles)
        #     rate.sleep()
        if rospy.is_shutdown():
            return
        else:
            if p is None:
                p = self.init_pos
                rospy.loginfo(self.name + ": move_leg_to p = " + str(p) + " = init_pos = " + str(self.init_pos))
            else:
                rospy.loginfo(self.name + ": move_leg_to p = " + str(p))
            # TODO what does array.any() or array.all() do?
            if self.init_pos[0] != p[0] or self.init_pos[1] != p[1] or self.init_pos[2] != p[2]:
                rospy.logerr("move leg to " + str(p) + " but init pose is set to " + str(self.init_pos))
            angles = self.leg.compute_inverse_kinematics(p)
            rospy.loginfo(
                self.name + ": move_leg_to inverse kinematic. for position " + str(p) + " angles = " + str(angles))
            self.leg.set_command(angles)


if __name__ == '__main__':
    nh = rospy.init_node('single_leg_controller', anonymous=True)
    # lm, rf, rr
    legController = SingleLegController('lf', nh, True, None)
    # rospy.spin()
    try:
        # legController.manage_walk()
        # legController.manage_walk_bezier()
        legController.bezier_swing()
        # legController.manage_swing()
        # legController.manage_simple_stance()
    except rospy.ROSInterruptException:
        pass
