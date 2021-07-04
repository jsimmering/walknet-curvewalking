#!/usr/bin/env python3
from math import fabs, exp, cos, sin, isnan, pi

import numpy
import rospy
from control_msgs.msg import JointControllerState
from walknet_curvewalking.msg import rules

import walknet_curvewalking_project.phantomx.RobotSettings as RSTATIC
from walknet_curvewalking_project.motion_primitives.stance_movement_body_model import StanceMovementBodyModel
from walknet_curvewalking_project.motion_primitives.swing_movement_bezier import SwingMovementBezier
from walknet_curvewalking_project.phantomx.SingleLeg import SingleLeg


class SingleLegController:
    def __init__(self, name, note_handle, swing, robot, step_length, shift_aep, shift_aep_x, decrease_inner_stance):
        self.robot = robot
        self.name = name
        self.nh = note_handle
        self.rate = rospy.Rate(RSTATIC.controller_frequency)
        if 'l' in self.name:
            rospy.loginfo("leg on left side movement_dir 1")
            self.movement_dir = 1
        else:
            rospy.loginfo("leg on right side movement_dir -1")
            self.movement_dir = -1

        self.step_length = step_length

        if isnan(shift_aep):
            self.shift_aep = False
        else:
            self.shift_aep = True
        self.aep_y_shift_value = shift_aep
        if isnan(shift_aep_x):
            self.shift_aep_x = False
        else:
            self.shift_aep_x = True
        self.aep_x_shift_value = shift_aep_x
        rospy.loginfo(self.name + ": shift aep y = {}, value = {}, shift aep x = {}, value = {}".format(self.shift_aep,
                self.aep_y_shift_value, self.shift_aep_x, self.aep_x_shift_value))

        self.stance_length_sum = []
        self.stance_count = 0
        self.current_stance_delayed = False
        self.current_stance_start = None
        self.current_stance_start_time = None
        self.current_swing_start_time = None
        self.stance_durations = []
        self.swing_durations = []

        self.decrease_inner_stance = bool(decrease_inner_stance)
        self.stance_diff = decrease_inner_stance

        self.leg = SingleLeg(name, self.movement_dir, self.step_length)

        self.swing_generator = SwingMovementBezier(self.leg)
        self.swing = swing
        self.swing_delays = 0
        self.init_pos = None
        self.last_stance_activation = None

        self.delay_1b = None
        self.threshold_rule3_ipsilateral = None
        self.threshold_rule3_contralateral = None

        self.displ_leg_ipsilateral_rule3_default = 0.4
        self.displ_leg_ipsilateral_rule2_default = 0.5375

        self.displ_leg_rule1 = 0.7
        self.displ_leg_rule1b = 0.2
        self.displ_leg_rule2 = 0.1375
        self.displ_leg_rule3 = 0.3125
        if self.name == "lm" or self.name == "rm":
            self.displ_leg_rule3 = 0.0
        elif self.name == "lr" or self.name == "rr":
            self.displ_leg_rule3 = 0.375

        self.target_pos = RSTATIC.initial_aep[RSTATIC.leg_names.index(self.name) // 2].copy()
        self.target_pos[1] = self.target_pos[1] * self.movement_dir

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

        self._rules_pub = rospy.Publisher('/walknet/' + self.name + '/rules', rules, queue_size=1)

        leg_behind_idx = RSTATIC.leg_names.index(self.name) + 2
        if 0 <= leg_behind_idx < 6:
            self._ipsilateral_rules_sub = rospy.Subscriber('/walknet/' + RSTATIC.leg_names[leg_behind_idx] +
                                                           '/rules', rules, self.ipsilateral_rules_callback)
        leg_in_front_idx = RSTATIC.leg_names.index(self.name) - 2
        if 0 <= leg_in_front_idx < 6:
            self._ipsilateral_rules_sub = rospy.Subscriber('/walknet/' + RSTATIC.leg_names[leg_in_front_idx] +
                                                           '/rules', rules, self.ipsilateral_rules_from_front_callback)
        neighbour_leg_idx = RSTATIC.leg_names.index(self.name) + (1 * self.movement_dir)
        if 0 <= neighbour_leg_idx < 6:
            self._contralateral_rules_sub = rospy.Subscriber('/walknet/' + RSTATIC.leg_names[neighbour_leg_idx] +
                                                             '/rules', rules, self.contralateral_rules_callback)

        self.rule1 = True
        self.rule2_contra = True
        self.rule2_ipsi = True
        self.rule3_contra = True
        self.rule3_ipsi = True

        self.default_step_length = RSTATIC.default_stance_distance

    def set_init_pos(self, p):
        self.init_pos = p

    def set_pull_dependent_parameter(self, velocity, angle):
        rospy.loginfo("##################################################################")

        # RULE 1
        v1 = 0.07
        v2 = 0.107
        v3 = 0.142
        self.delay_1b = 0.0
        if velocity <= v1:
            self.delay_1b = 0.27
        elif v1 < velocity <= v2:
            self.delay_1b = 0.27 + ((0.27 - 0.2) / (v1 - v2)) * (velocity - v1)
        elif v2 < velocity <= v3:
            self.delay_1b = 0.2 + (0.2 / (v2 - v3) * (velocity - v2))
        if self.delay_1b < 0.0:
            self.delay_1b = 0
        rospy.loginfo(self.name + ": self.delay_1b = " + str(self.delay_1b))

        # ADJUST DEFAULT STEP LENGTH
        self.stance_diff = -pow(0.28 * (angle - 0.9), 2) + 0.02
        if self.stance_diff < 0:
            self.stance_diff = 0

        if self.decrease_inner_stance and (
                angle < 0.0 and (self.name == "rf" or self.name == "rm" or self.name == "rr")) or (
                angle > 0.0 and (self.name == "lf" or self.name == "lm" or self.name == "lr")):
            rospy.loginfo(self.name + ": STANCE DIFF = -" + str(self.stance_diff))
            self.default_step_length -= self.stance_diff
            rospy.loginfo(self.name + ":  default_step_length = " + str(self.default_step_length))
            if self.step_length:
                self.leg.set_default_step_length(self.default_step_length)
            else:
                RSTATIC.initial_pep[RSTATIC.leg_names.index(self.name) // 2][0] += self.stance_diff
                rospy.loginfo(self.name + ": update RobotSettings new initial pep = " + str(
                        RSTATIC.initial_pep[RSTATIC.leg_names.index(self.name) // 2][0]))
        else:
            rospy.loginfo(self.name + ": maintain original default_step_length = " + str(self.default_step_length))

        # RULE 3
        self.threshold_rule3_ipsilateral = fabs(self.default_step_length) / (1.0 + exp(50 * (velocity - 0.085)))
        self.threshold_rule3_contralateral = fabs(self.default_step_length) * (0.5 - 1.5 * velocity)
        rospy.logerr(self.name + ": threshold_rule3_ipsilateral = " + str(self.threshold_rule3_ipsilateral) +
                     "; threshold_rule3_contralateral = " + str(self.threshold_rule3_contralateral) +
                     "; default_step_length = " + str(self.default_step_length) + "; velocity = " + str(velocity))

        if self.leg.viz and self.step_length:
            self.leg.pub_default_pep_threshold()

    def pub_rules(self, rules_msg):
        self._rules_pub.publish(rules_msg)

    def ipsilateral_rules_callback(self, data):
        shift_distance = 0
        if self.rule1:
            shift_distance += self.default_step_length * data.rule1
        if self.rule2_ipsi:
            shift_distance += self.default_step_length * data.rule2_ipsilateral
        self.leg.shift_pep_ipsilateral(shift_distance)

    def ipsilateral_rules_from_front_callback(self, data):
        shift_distance = 0
        if self.rule3_ipsi:
            shift_distance += self.default_step_length * data.rule3_ipsilateral
        self.leg.shift_pep_ipsilateral_from_front(shift_distance)

    def contralateral_rules_callback(self, data):
        shift_distance = 0
        if self.rule2_contra:
            shift_distance += self.default_step_length * data.rule2_contralateral
        if self.rule3_contra:
            shift_distance += self.default_step_length * data.rule3_contralateral
        if (self.name == "lr" or self.name == "rr") and self.rule1:
            shift_distance += self.default_step_length * data.rule1
        self.leg.shift_pep_contralateral(shift_distance)

    # function for executing a single step in either stance or swing movement depending on current phase.
    def manage_walk(self, legs_in_swing, swing):
        if self.leg.viz:
            self.leg.pub_pep_threshold()
        if self.swing:
            if swing:
                return self.execute_swing_step(legs_in_swing)
            else:
                self.robot.running = False
                return legs_in_swing
        else:
            return self.execute_stance_step(legs_in_swing)

    def execute_stance_step(self, legs_in_swing):
        # rospy.loginfo(self.name + ": execute stance step.")
        if self.last_stance_activation:
            stance_duration = rospy.Time.now() - self.last_stance_activation
        else:
            stance_duration = None
        rules_msg = rules(0.0, 0.0, 0.0, 0.0, 0.0)
        if stance_duration and rospy.Duration.from_sec(0) <= stance_duration <= rospy.Duration.from_sec(self.delay_1b):
            rules_msg.rule1 = -self.displ_leg_rule1b
        if stance_duration and rospy.Duration.from_sec(0.27) <= stance_duration <= rospy.Duration.from_sec(0.32):
            rules_msg.rule2_ipsilateral = self.displ_leg_ipsilateral_rule2_default
            rules_msg.rule2_contralateral = self.displ_leg_rule2
        if self.step_length and self.current_stance_start is not None:
            stance_progress = numpy.linalg.norm(self.current_stance_start - self.leg.ee_position())
        else:
            stance_progress = self.target_pos[0] - self.leg.ee_position()[0]
        if self.threshold_rule3_ipsilateral < stance_progress < self.threshold_rule3_ipsilateral + 0.008:
            rules_msg.rule3_ipsilateral = self.displ_leg_ipsilateral_rule3_default
        if self.threshold_rule3_contralateral < stance_progress < self.threshold_rule3_contralateral + 0.008:
            rules_msg.rule3_contralateral = self.displ_leg_rule3
        self.pub_rules(rules_msg)
        self.stance_net.modulated_routine_function_call()
        # rospy.loginfo(self.name + ': current pep_thresh = ' + str(self.leg.pep_thresh))
        # if self.leg.reached_pep() and legs_in_swing < 3:
        if (self.step_length and self.leg.reached_step_length() and legs_in_swing < 3) or (
                not self.step_length and self.leg.reached_pep() and legs_in_swing < 3):
            # if self.name == "rf":
            #     rospy.logerr(self.name + ": reached_pep. switch to swing mode.")
            self.stance_net.reset_stance_trajectory()
            if (self.shift_aep or self.shift_aep_x) and self.stance_count > 0:
                self.move_aep()
            self.swing = True
            self.current_swing_start_time = rospy.Time.now()
            if self.current_stance_start_time is not None:
                self.stance_durations.append((self.current_swing_start_time - self.current_stance_start_time).to_sec())
                # rospy.loginfo(self.name + ": append stance, stance_durations = " + str(self.stance_durations))
                self.current_stance_start_time = None
            if not self.current_stance_delayed:
                if self.current_stance_start is not None:
                    if len(self.stance_length_sum) > 10:
                        self.stance_length_sum.pop(0)
                    self.stance_length_sum.append(numpy.linalg.norm(self.leg.ee_position() - self.current_stance_start))
                    self.stance_count += 1
                    self.current_stance_start = None
                # else:
                #     rospy.logwarn(self.name + " no stance start position available, stance count = " +
                #     str(self.stance_count))
            self.current_stance_delayed = False
            legs_in_swing = legs_in_swing + 1
        # elif self.leg.reached_pep() and legs_in_swing >= 3:
        elif (self.step_length and self.leg.reached_step_length() and legs_in_swing >= 3) or (
                not self.step_length and self.leg.reached_pep() and legs_in_swing >= 3):
            rospy.logwarn(self.name + ": delayed swing start.")
            self.swing_delays += 1
            if not self.current_stance_delayed:
                if self.current_stance_start is not None:
                    if len(self.stance_length_sum) > 10:
                        self.stance_length_sum.pop(0)
                    self.stance_length_sum.append(numpy.linalg.norm(self.leg.ee_position() - self.current_stance_start))
                    self.stance_count += 1
                    # self.current_stance_start = None
                # else:
                #     rospy.logwarn(self.name + " no stance start position available, stance count = " +
                #     str(self.stance_count))
                self.current_stance_delayed = True
        return legs_in_swing

    def move_aep(self):
        shifted_y = False

        ee_pos = self.leg.ee_position()
        step_vector = ee_pos - self.target_pos
        average_step_length = numpy.median(self.stance_length_sum)

        if self.shift_aep:
            step_vector_default_length = (average_step_length / numpy.linalg.norm(
                    numpy.array(step_vector))) * step_vector

            ee_default_step = self.target_pos + step_vector_default_length
            # ydir
            offset_center_1 = self.target_pos[1] - RSTATIC.initial_aep[RSTATIC.leg_names.index(self.name) // 2].copy()[
                1] * self.movement_dir
            offset_center_2 = RSTATIC.initial_aep[RSTATIC.leg_names.index(self.name) // 2].copy()[
                                  1] * self.movement_dir - ee_default_step[1]
            new_aep_y = RSTATIC.initial_aep[RSTATIC.leg_names.index(self.name) // 2].copy()[
                            1] * self.movement_dir - (step_vector_default_length[1] / 2)
            if abs(new_aep_y - self.target_pos[1]) >= 0.01:
                rospy.loginfo(self.name + ": new_aep_y = {} previous aep = {} default aep y = {}".format(new_aep_y,
                        self.target_pos[1],
                        RSTATIC.initial_aep[RSTATIC.leg_names.index(self.name) // 2].copy()[1] * self.movement_dir))
                self.target_pos[1] = new_aep_y
                # self.leg.shift_default_aep(self.target_pos)
                shifted_y = True

        # xdir
        shifted_x = False
        if self.shift_aep_x:
            step_vector_default_length = (average_step_length / numpy.linalg.norm(
                    numpy.array(step_vector))) * step_vector

            x_center = RSTATIC.initial_aep[RSTATIC.leg_names.index(self.name) // 2].copy()[0] - (
                    (RSTATIC.default_stance_distance * 1) / 2)
            new_aep_x = x_center - ((step_vector_default_length[0] * 1) / 2)
            if abs(new_aep_x - self.target_pos[0]) >= 0.01:
                rospy.loginfo(self.name + ": new_aep_x = {} previous aep = {} default aep y = {}".format(new_aep_x,
                        self.target_pos[0],
                        RSTATIC.initial_aep[RSTATIC.leg_names.index(self.name) // 2].copy()[0]))
                self.target_pos[0] = new_aep_x
                shifted_x = True

        if shifted_y or shifted_x:
            rospy.loginfo(
                    self.name + ": average stance length = {}, default stance length = {}".format(average_step_length,
                            self.default_step_length))
            self.leg.shift_default_aep(self.target_pos)

    def execute_swing_step(self, legs_in_swing):
        # rospy.loginfo(self.name + ": execute swing step.")
        if self.swing_generator.swing_start_point is None:
            # rospy.loginfo(self.name + ": reset swing")
            self.swing_generator.swing_start_point = self.leg.ee_position()
            self.swing_generator.swing_target_point = self.target_pos
            self.swing_generator.reacht_peak = False
            # self.temp.trajectory_generator.bezier_points = self.temp.compute_bezier_points()
            self.swing_generator.trajectory_generator.bezier_points = self.swing_generator.compute_bezier_points_with_joint_angles()
        rules_msg = rules(-self.displ_leg_rule1, 0.0, 0.0, 0.0, 0.0)  # rules(-0.1, 0.0, 0.0, 0.0, 0.0)
        self.pub_rules(rules_msg)
        self.swing_generator.move_to_next_point(1)
        if self.swing_generator.reacht_peak and self.leg.predicted_ground_contact():
            self.swing_generator.move_to_next_point(0)
            self.swing_generator.swing_start_point = None
            self.swing = False
            self.current_stance_start_time = rospy.Time.now()
            if self.current_swing_start_time is not None:
                self.swing_durations.append((self.current_stance_start_time - self.current_swing_start_time).to_sec())
                # rospy.loginfo(self.name + ": append swing, swing_durations = " + str(self.swing_durations))
                self.current_swing_start_time = None

            legs_in_swing = legs_in_swing - 1
            self.last_stance_activation = rospy.Time.now()
            # if self.name == "rf":
            #     rospy.logerr(self.name + ': swing is finished switch to stance.')
            self.current_stance_start = self.leg.ee_position()
        return legs_in_swing

    # function for moving this leg into the position provided or the init position.
    def move_leg_to(self, p=None):
        if p is None and not rospy.is_shutdown():
            p = self.init_pos
        angles = self.leg.compute_inverse_kinematics(p)
        self.leg.set_joint_point_and_target(angles)
