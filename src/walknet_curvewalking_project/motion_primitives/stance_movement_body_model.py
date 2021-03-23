#!/usr/bin/env python

# Took Code from
# https://github.com/malteschilling/cognitiveWalker/blob/master/controller/reaCog/Movements/StanceMovementBodyModel.py
# modified for phantomX
import rospy

import walknet_curvewalking_project.phantomx.RobotSettings as RSTATIC
import numpy


##
# 	Stance Network.
#
# This stance movement network is driven by the body model.
# When active it pulls as an input a new target vector from the body model
# for the connected leg.
# And from this it produces joint velocities which move the leg from
# the current joint configuration towards this goal posture.
#
# There are different possible ways how the target joint angles are computed
# (this is defined in WalknetSettings):
# 	0 -	using the direct computation of inverse kinematics
# 	1 - using an angular MMC network and iterating for a couple of steps
# 	2 - using a dual quaternion MMC network for solving inverse kinematics.
# The different approaches each are encapsulated as the inverseKinematic_provider
# and each of these objects has a computeInverseKinematics method which
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
    #	@param motiv_leg Motivation network of the leg
    #	@param extr_pos extreme positions of the stance movement (also encoding direction)
    def __init__(self, leg):
        self.leg_controller = leg
        self.init_stance_footpoint = False
        self.bodyModelStance = self.leg_controller.robot.body_model
        self.inverseKinematic_provider = self.leg_controller.leg

    def __del__(self):
        pass

    def reset_stance_trajectory(self):
        self.init_stance_footpoint = False

    ##	Function called by the ModulatingMotivationUnit when active.
    #	Invokes the execution of the routine which has to be defined by the derived
    #	classes.
    def modulated_routine_function_call(self):
        if not self.init_stance_footpoint:
            # stance_foot_pos = self.leg_controller.leg.ee_position()
            # if (stance_foot_pos[0] <= self.leg_controller.pep_shifted[0]):
            #     print("Stance correction: foot moved in Body Model in front of PEP ",
            #         self.leg_controller.wleg.leg.name)
            #     stance_foot_pos[0] = self.leg_controller.pep_shifted[0] + 0.02
            # self.bodyModelStance.put_leg_on_ground(self.leg_controller.name, self.leg_controller.leg.compute_forward_kinematics_c1())  #self.leg_controller.leg.ee_position())
            self.bodyModelStance.put_leg_on_ground(self.leg_controller.name,
                    self.leg_controller.leg.ee_position() - self.leg_controller.leg.apply_c1_static_transform())
            self.init_stance_footpoint = True
        target_vec = None
        try:
            target_vec = self.leg_controller.leg.apply_c1_static_transform() + self.bodyModelStance.get_leg_vector(
                    self.leg_controller.leg.name)
            next_angles = self.inverseKinematic_provider.compute_inverse_kinematics(target_vec)
            # rospy.loginfo(self.leg_controller.name + " target height = " + str(target_vec[2]))
            self.leg_controller.leg.set_command(next_angles)
        except ValueError as ve:
            rospy.logerr("modulated_routine_function_call: ValueError during inverse kinematics computation: " +
                         str(ve))
            # + "\n Tried to reach position " + str(target_vec) +
            # "\ncurrent position is: " + str(self.leg_controller.leg.ee_position()) +
            # "\ncurrent angles are: " + str(self.leg_controller.leg.get_current_angles()) +
            # "\nMaintaining current angles.")
            self.leg_controller.leg.set_command(self.leg_controller.leg.get_current_angles())

        # current_angles = numpy.array(self.leg_controller.leg.ee_position())
        # new_joint_velocities = (next_angles - current_angles.T) / (1 / RSTATIC.controller_frequency)
        # self.leg_controller.wleg.addControlVelocities(new_joint_velocities)
