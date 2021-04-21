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

    def __del__(self):
        pass

    def reset_stance_trajectory(self):
        self.init_stance_footpoint = False

    def modulated_routine_function_call(self):
        if not self.init_stance_footpoint:
            self.bodyModelStance.put_leg_on_ground(self.leg_controller.name, self.leg_controller.leg.ee_position())
            #       self.leg_controller.leg.ee_position() - self.leg_controller.leg.apply_c1_static_transform())
            self.init_stance_footpoint = True
        try:
            # if self.leg_controller.name == "lm":
            #     rospy.loginfo(self.leg_controller.name + ": body model leg vector = " + str(self.bodyModelStance.get_leg_vector(
            #         self.leg_controller.leg.name)))

            # ee_target_bm_frame = self.bodyModelStance.get_leg_vector(self.leg_controller.leg.name)
            #ee_target_bm_frame = (self.bodyModelStance.segm_post_ant / 2) + self.bodyModelStance.get_front_vector(self.leg_controller.leg.name)

            # leg_vec = self.leg_controller.leg.ee_position() - self.leg_controller.leg.apply_c1_static_transform()
            # ee_x_in_bm_frame = leg_vec[0] * (self.bodyModelStance.segm_post_ant / self.bodyModelStance.segm_post_ant_norm)
            # rospy.loginfo("ee_x_in_bm_frame = " + str(ee_x_in_bm_frame))
            # ee_y_in_bm_frame = leg_vec[1] * numpy.cross((self.bodyModelStance.segm_post_ant / self.bodyModelStance.segm_post_ant_norm), [0, 0, 1])
            # rospy.loginfo("ee_y_in_bm_frame = " + str(ee_y_in_bm_frame))
            # ee_in_bm_frame = numpy.array(
            #         [ee_x_in_bm_frame[0] - ee_y_in_bm_frame[0], ee_x_in_bm_frame[1] - ee_y_in_bm_frame[1], leg_vec[2]])
            # rospy.logwarn("leg vect = " + str(ee_in_bm_frame) + " type = " + str(type(ee_in_bm_frame[2])))
            #
            # # self.front_vect[leg_nr] = -self.segm_post_ant / 2 + leg_vec
            # front_vect = (-self.bodyModelStance.segm_post_ant / 2) + ee_in_bm_frame
            #
            # # self.leg_vect[leg_nr] = numpy.array(leg_vec - self.c1_positions[leg_nr])
            # leg_vect_bm_frame = self.bodyModelStance.segm_leg_ant[leg_nr] + front_vect

            # TODO change back to robot frame where x axis lies along body model segment!

            # TODO manual translation approach (projection)
            #v = self.bodyModelStance.segm_post_ant
            #ee_x_bm_projected = ((ee_target_bm_frame * v)/math.pow(numpy.linalg.norm(v), 2)) * v
            #rospy.loginfo("ee_x_bm_projected = " + str(ee_x_bm_projected))
            #rospy.loginfo("ee_x_bm_projected norm = " + str(numpy.linalg.norm(ee_x_bm_projected)))
            #rospy.loginfo("current ee_x_robot = " + str(self.inverseKinematic_provider.ee_position()[0]))

            #v1 = numpy.cross((self.bodyModelStance.segm_post_ant / self.bodyModelStance.segm_post_ant_norm), [0, 0, 1])
            #ee_y_bm_projected = ((ee_target_bm_frame * v1) / math.pow(numpy.linalg.norm(v1), 2)) * v1
            #rospy.loginfo("ee_y_bm_projected = " + str(ee_y_bm_projected))
            #rospy.loginfo("ee_y_bm_projected norm = " + str(numpy.linalg.norm(ee_y_bm_projected)))
            #rospy.loginfo("current ee_y_robot = " + str(self.inverseKinematic_provider.ee_position()[1]))

            #ee_target_robot_frame = numpy.array([numpy.linalg.norm(ee_x_bm_projected), numpy.linalg.norm(ee_y_bm_projected), ee_target_bm_frame[2]])

            # ee_x_in_bm_frame = leg_vec[0] * (self.segm_post_ant / self.segm_post_ant_norm)
            # rospy.loginfo("ee_x_in_bm_frame = " + str(ee_x_in_bm_frame))
            # ee_y_in_bm_frame = leg_vec[1] * numpy.cross((self.segm_post_ant / self.segm_post_ant_norm), [0, 0, 1])
            # rospy.loginfo("ee_y_in_bm_frame = " + str(ee_y_in_bm_frame))
            # ee_in_bm_frame = numpy.array(
            #         [ee_x_in_bm_frame[0] - ee_y_in_bm_frame[0], ee_x_in_bm_frame[1] - ee_y_in_bm_frame[1], ee_target_bm_frame[2]])
            # #rospy.logwarn("leg vect = " + str(ee_in_bm_frame) + " type = " + str(type(ee_in_bm_frame[2])))
            #
            body_angle = -math.atan2(self.bodyModelStance.segm_post_ant[1], self.bodyModelStance.segm_post_ant[0])
            #rospy.loginfo(self.leg_controller.name + ": stance negative body angle = " + str(body_angle))
            bm_frame_rotation_matrix = numpy.array([[math.cos(body_angle), -math.sin(body_angle), 0],
                                                    [math.sin(body_angle), math.cos(body_angle), 0], [0, 0, 1]])

            rotated_leg_vec = bm_frame_rotation_matrix.dot(self.bodyModelStance.get_leg_vector(self.leg_controller.leg.name))
            # target_vec = (-self.bodyModelStance.segm_post_ant / 2) + ee_target_robot_frame
            # target_vec = ee_target_robot_frame
            target_vec = self.leg_controller.leg.apply_c1_static_transform() + (rotated_leg_vec)

            if self.leg_controller.name == "lf":
                rospy.loginfo("lf: leg vec (BM) = " + str(self.bodyModelStance.get_leg_vector(self.leg_controller.leg.name)))
                rospy.loginfo("lf: rotated leg vec (robot)= " + str(rotated_leg_vec))
                rospy.loginfo(self.leg_controller.name + ": current ee_x_robot = " + str(
                        self.inverseKinematic_provider.ee_position()))
                rospy.loginfo("lf: target leg vec (robot)= " + str(target_vec))
            # rospy.logwarn("target vec = "+str(target_vec))
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
