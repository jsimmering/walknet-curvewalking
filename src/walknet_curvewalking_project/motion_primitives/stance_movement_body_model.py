#!/usr/bin/env python3

# Took Code from
# https://github.com/malteschilling/cognitiveWalker/blob/master/controller/reaCog/Movements/StanceMovementBodyModel.py
# modified for phantomX
import math

import numpy
import rospy


##
# 	Stance Network.
#
# This stance movement network is driven by the body model.
# When active it pulls as an input a new target vector from the body model
# for the connected leg.
# And from this it produces joint positions which move the leg from
# the current joint configuration towards this goal posture.
#
# The target joint angles are computed using the direct computation of inverse kinematics which
# provides corresponding joint angles for a given target position.
#
# A movement is derived from ModulatedRoutine which defines an interface.
# It implements the required modulatedRoutineFunctionCall -
# as a consequence a movement can be connected to a MotivationUnit which
# (when active) always executes the connected modulatedRoutineFunctionCall.
##
class StanceMovementBodyModel:

    ##	Initialisation of the Stance Movement. Connecting to the bodyModelStance and
    #	setting the inverseKinematic_provider.
    #	@param leg single leg controller of the leg
    def __init__(self, leg):
        self.leg_controller = leg
        self.init_stance_footpoint = False
        self.bodyModelStance = self.leg_controller.robot.body_model
        self.inverseKinematic_provider = self.leg_controller.leg
        self.valueError_count = 0

    def reset_stance_trajectory(self):
        self.init_stance_footpoint = False

    def modulated_routine_function_call(self):
        if not self.init_stance_footpoint:
            self.bodyModelStance.put_leg_on_ground(self.leg_controller.name, self.leg_controller.leg.ee_position())
            #       self.leg_controller.leg.ee_position() - self.leg_controller.leg.apply_c1_static_transform())
            self.init_stance_footpoint = True
        try:
            body_angle = math.atan2(self.bodyModelStance.segm_post_ant[1], self.bodyModelStance.segm_post_ant[0])
            rotation_matrix_bm_robot = numpy.array([[math.cos(-body_angle), -math.sin(-body_angle), 0],
                                                    [math.sin(-body_angle), math.cos(-body_angle), 0], [0, 0, 1]])
            rotated_leg_vec = rotation_matrix_bm_robot.dot(
                self.bodyModelStance.get_leg_vector(self.leg_controller.leg.name))

            # target_vec = self.leg_controller.leg.apply_c1_static_transform() + self.bodyModelStance.get_leg_vector(
            #         self.leg_controller.leg.name)
            target_vec = self.leg_controller.leg.apply_c1_static_transform() + (rotated_leg_vec)

            next_angles = self.inverseKinematic_provider.compute_inverse_kinematics(target_vec)
            self.leg_controller.leg.set_command(next_angles)
        except ValueError as ve:
            rospy.logerr("STANCE: ValueError during inverse kinematics computation: " + str(ve))
            #             "\n Tried to reach position " + str(target_vec) +
            #             "\ncurrent position is: " + str(self.leg_controller.leg.ee_position()) +
            #             "\ncurrent angles are: " + str(self.leg_controller.leg.get_current_angles()) +
            #             "\nMaintaining current angles.")
            self.valueError_count += 1
            self.leg_controller.leg.set_command(self.leg_controller.leg.get_current_angles())
