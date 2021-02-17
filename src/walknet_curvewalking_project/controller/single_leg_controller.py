#!/usr/bin/env python3
import threading
from math import fabs, exp

import rospy
from control_msgs.msg import JointControllerState
from walknet_curvewalking.msg import rules

import walknet_curvewalking_project.phantomx.RobotSettings as RSTATIC
from walknet_curvewalking_project.motion_primitives.stance_movement_body_model import StanceMovementBodyModel
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
            rospy.loginfo("leg on right side movement_dir -1")
            self.movement_dir = -1
        self.leg = SingleLeg(name, self.movement_dir)
        self.temp = SwingMovementBezier(self.leg)
        self.swing = swing
        self.init_pos = None
        self.last_stance_activation = None
        if not self.swing:
            self.last_stance_activation = rospy.Time.now()
        self.delay_1b = None
        self.threshold_rule3_ipsilateral = None
        self.threshold_rule3_contralateral = None

        # self.target_pos = None
        # TODO possibly needs to be adjusted with velocity? 4cm steps 0.01 velocity 0.02; 0.075 - 0.1 velocity 0.033?
        self.displ_leg_ipsilateral = 0.05
        if self.name == "lf" or self.name == "rf":
            self.target_pos = RSTATIC.front_initial_aep.copy()
            self.displ_leg = 0.025
        elif self.name == "lm" or self.name == "rm":
            self.target_pos = RSTATIC.middle_initial_aep.copy()
            self.displ_leg = 0.0
        elif self.name == "lr" or self.name == "rr":
            self.target_pos = RSTATIC.hind_initial_aep.copy()
            self.displ_leg = 0.01
        self.target_pos[1] = self.target_pos[1] * self.movement_dir
        rospy.loginfo("leg " + str(self.name) + " target_pos = " + str(self.target_pos))
        self.aep_x = RSTATIC.initial_aep[RSTATIC.leg_names.index(self.name)][0].copy()

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
            # rospy.loginfo(self.name + ": leg_behind_idx = " + str(leg_behind_idx) + " leg_behind = " + str(
            #        RSTATIC.leg_names[leg_behind_idx]))
            self._ipsilateral_rules_sub = rospy.Subscriber('/walknet/' + RSTATIC.leg_names[leg_behind_idx] +
                                                           '/rules', rules, self.ipsilateral_rules_callback)
        leg_in_front_idx = RSTATIC.leg_names.index(self.name) - 2
        if 0 <= leg_in_front_idx < 6:
            # rospy.loginfo(self.name + ": leg_behind_idx = " + str(leg_behind_idx) + " leg_in_front_idx = " + str(
            #        RSTATIC.leg_names[leg_in_front_idx]))
            self._ipsilateral_rules_sub = rospy.Subscriber('/walknet/' + RSTATIC.leg_names[leg_in_front_idx] +
                                                           '/rules', rules, self.ipsilateral_rules_from_front_callback)
        neighbour_leg_idx = RSTATIC.leg_names.index(self.name) + (1 * self.movement_dir)
        if 0 <= neighbour_leg_idx < 6:
            # rospy.loginfo(self.name + ": neighbour_leg_idx = " + str(neighbour_leg_idx) + " neighbour_leg = " + str(
            #        RSTATIC.leg_names[neighbour_leg_idx]))
            self._contralateral_rules_sub = rospy.Subscriber('/walknet/' + RSTATIC.leg_names[neighbour_leg_idx] +
                                                             '/rules', rules, self.contralateral_rules_callback)
        # publish pep visualization
        th = threading.Thread(target=self.leg.pub_pep_threshold, daemon=True)
        th.start()

        self.rule1 = True
        self.rule2_ipsi = True
        self.rule2_contra = True
        self.rule3_ipsi = True
        self.rule3_contra = True

    def set_init_pos(self, p):
        self.init_pos = p
        rospy.loginfo(self.name + ": set init pos to P = " + str(p))
        rospy.loginfo(self.name + ": set init pos = " + str(self.init_pos))

    def set_delay_1b(self, velocity):
        if velocity >= 0.03:
            delay = 0.8 - 1.5 * velocity
        elif velocity <= 0.03:
            delay = 0.4 - 0.5 * velocity
        if delay > 0.27:
            self.delay_1b = 0.27
        elif delay < 0.0:
            self.delay_1b = 0
        else:
            self.delay_1b = delay
        pep_x = RSTATIC.initial_pep[RSTATIC.leg_names.index(self.name)][0].copy()
        self.threshold_rule3_ipsilateral = fabs(self.aep_x - pep_x) / (1.0 + exp(-15.0 * (velocity - 0.37)))
        self.threshold_rule3_contralateral = fabs(self.aep_x - pep_x) * (0.5 + 0.5 * velocity)
        rospy.loginfo(self.name + ": aep_x = " + str(self.aep_x) + "pep_x = " + str(pep_x) + " aep_x - pep_x = " +
                      str(fabs(self.aep_x - pep_x)))
        rospy.loginfo(self.name + ": threshold_rule3_ipsilateral = " + str(self.threshold_rule3_ipsilateral))
        rospy.loginfo(
                self.name + ": threshold_rule3_ipsilateral + 0.015 = " + str(self.threshold_rule3_ipsilateral + 0.015))
        rospy.loginfo(self.name + ": threshold_rule3_contralateral = " + str(self.threshold_rule3_contralateral))
        rospy.loginfo(self.name + ": threshold_rule3_contralateral + 0.015 = " +
                      str(self.threshold_rule3_contralateral + 0.015))

    def bezier_swing(self):
        while not self.leg.is_ready() and not rospy.is_shutdown():
            rospy.loginfo("leg not connected yet! wait...")
            self.rate.sleep()
        self.temp.swing_start_point = self.leg.ee_position()

        self.temp.swing_target_point = self.target_pos
        # the offset that is added to the middle point that was computed on the connecting line between start and
        # end point using the apex_point_ratio concept.
        # temp.apex_point_offset = numpy.array([0, 0, 0.4]) # constant is used
        # temp.collision_point = numpy.array([0.8, 0, 0.256])
        # bezier_points = temp.compute_bezier_points()
        # self.temp.trajectory_generator.bezier_points = self.temp.compute_bezier_points()
        self.temp.trajectory_generator.bezier_points = self.temp.compute_bezier_points_with_joint_angles()
        print(self.temp.trajectory_generator.bezier_points)
        while not rospy.is_shutdown() and not self.leg.predicted_ground_contact():
            self.temp.move_to_next_point(1)
            self.rate.sleep()
        self.temp.move_to_next_point(0)
        self.rate.sleep()
        self.swing = False

    def pub_rules(self, rules_msg):
        now = rospy.Time.now()
        # rospy.loginfo(self.name + ' start swing publish rule 1 at ' + str(now.secs) + ' sec and ' + str(now.nsecs) +
        #              'nsecs')
        self._rules_pub.publish(rules_msg)

    def ipsilateral_rules_callback(self, data):
        # rospy.loginfo(self.name + ' received rule 1 from leg behind ' +
        #               str(RSTATIC.leg_names[RSTATIC.leg_names.index(self.name) + 2]) + ' leg at ' + str(now.secs) +
        #               ' sec and ' + str(now.nsecs) + 'nsecs. shift target.')
        shift_distance = 0.0
        if self.rule1:
            shift_distance += data.rule1
        if self.rule2_ipsi:
            shift_distance += data.rule2_ipsilateral
        if shift_distance != 0.0:
            rospy.loginfo(self.name + ": from leg behind shift_distance (" + str(shift_distance) + ") = data.rule1 (" +
                          str(data.rule1) + ") + data.rule2_ipsilateral (" + str(data.rule2_ipsilateral) + ")")
        self.leg.shift_pep_ipsilateral(shift_distance)

    def ipsilateral_rules_from_front_callback(self, data):
        # rospy.loginfo(self.name + ' received rule 1 from leg behind ' +
        #               str(RSTATIC.leg_names[RSTATIC.leg_names.index(self.name) + 2]) + ' leg at ' + str(now.secs) +
        #               ' sec and ' + str(now.nsecs) + 'nsecs. shift target.')
        shift_distance = 0.0
        if self.rule3_ipsi:
            shift_distance += data.rule3_ipsilateral
        if shift_distance != 0.0:
            rospy.loginfo(self.name + ": from leg in front shift_distance (" + str(shift_distance) +
                          ") = data.rule3_ipsilateral (" + str(data.rule3_ipsilateral) + ")")
        self.leg.shift_pep_ipsilateral_from_front(shift_distance)

    def contralateral_rules_callback(self, data):
        # rospy.loginfo(self.name + ' received rule 1 from neighbouring leg ' +
        #               str(RSTATIC.leg_names.index(self.name) + (1 * self.movement_dir)) + ' leg at ' + str(now.secs) +
        #               ' sec and ' + str(now.nsecs) + 'nsecs. shift target.')
        shift_distance = 0.0
        if self.rule2_contra:
            shift_distance += data.rule2_contralateral
        if self.rule3_contra:
            shift_distance += data.rule3_contralateral
        if shift_distance != 0.0:
            rospy.loginfo(self.name + ": from neighbour leg shift_distance (" + str(shift_distance) +
                          ") = data.rule2_contralateral (" + str(data.rule2_contralateral) +
                          ") + data.rule3_contralateral (" + str(data.rule3_contralateral) + ")")
        self.leg.shift_pep_contralateral(shift_distance)

    # function for executing a single step in a stance movement.
    def manage_walk(self):
        # if not self.robot.walk_motivation or rospy.is_shutdown():
        #     rospy.loginfo("no moving motivation or shutdown...")
        #     return
        # else:
        if self.swing:
            self.execute_swing_step()
        else:
            self.execute_stance_step()

    def execute_stance_step(self):
        # rospy.loginfo(self.name + ": execute stance step.")
        if RSTATIC.DEBUG:
            rospy.loginfo("time since last_stance_activation = now (" + str(
                    rospy.Time.now()) + ") - last_activation (" + str(
                    self.last_stance_activation) + ") = " + str(
                    rospy.Time.now() - self.last_stance_activation))
            rospy.loginfo("must be <= delay_1b " + str(rospy.Duration.from_sec(self.delay_1b)))

        stance_duration = rospy.Time.now() - self.last_stance_activation
        rules_msg = rules(0.0, 0.0, 0.0, 0.0, 0.0)
        # rules_msg.rule1 = 0.0
        # rules_msg.rule2_ipsilateral = 0.0
        # rules_msg.rule2_contralateral = 0.0
        # rules_msg.rule3_ipsilateral = 0.0
        # rules_msg.rule3_contralateral = 0.0
        if rospy.Duration.from_sec(0) <= stance_duration <= rospy.Duration.from_sec(self.delay_1b):
            #rospy.logerr(self.name + " rule 1 -0.006")
            rules_msg.rule1 = -0.02
        if rospy.Duration.from_sec(0.25) <= stance_duration <= rospy.Duration.from_sec(0.37):
            #rospy.logerr(self.name + " rule 2 ipsi = 0.008 contra = 0.002")
            rules_msg.rule2_ipsilateral = 0.032
            rules_msg.rule2_contralateral = 0.01
        stance_progress = self.aep_x - self.leg.compute_forward_kinematics()[0]
        rospy.loginfo(self.name + ": stance_progress (" + str(stance_progress) +
                      ") = self.aep_x (" + str(self.aep_x) + ") - leg.compute_forward_kinematics()[0] (" +
                      str(self.leg.compute_forward_kinematics()[0]))
        # rospy.logerr(self.name + ": self.threshold_rule3_ipsilateral (" + str(self.threshold_rule3_ipsilateral) +
        #              ") < stance_progress (" + str(stance_progress) + ") < " +
        #              str(self.threshold_rule3_ipsilateral + 0.01))
        if self.threshold_rule3_ipsilateral < stance_progress < self.threshold_rule3_ipsilateral + 0.015:
            #rospy.logerr(self.name + " rule 3 " + str(self.displ_leg_ipsilateral))
            rules_msg.rule3_ipsilateral = self.displ_leg_ipsilateral
        if self.threshold_rule3_contralateral < stance_progress < self.threshold_rule3_contralateral + 0.015:
            #rospy.logerr(self.name + " rule 3 " + str(self.displ_leg))
            rules_msg.rule3_contralateral = self.displ_leg
        self.pub_rules(rules_msg)
        self.stance_net.modulated_routine_function_call()
        # rospy.loginfo(self.name + ': current pep_thresh = ' + str(self.leg.pep_thresh))
        if self.leg.reached_pep():
            rospy.loginfo(self.name + ": reached_pep. switch to swing mode.")
            self.stance_net.reset_stance_trajectory()
            # self.rate.sleep()
            self.swing = True

    def execute_swing_step(self):
        # rospy.loginfo(self.name + ": execute swing step.")
        if self.temp.swing_start_point is None:
            rospy.loginfo(self.name + ": reset swing")
            self.temp.swing_start_point = self.leg.ee_position()
            self.temp.swing_target_point = self.target_pos
            # self.temp.swing_target_point = self.leg.compute_forward_kinematics(
            #                                [self.movement_dir * 0.3, -0.5, -1.2])
            # self.temp.trajectory_generator.bezier_points = self.temp.compute_bezier_points()
            self.temp.trajectory_generator.bezier_points = self.temp.compute_bezier_points_with_joint_angles()
        # rospy.loginfo("####### pub rule 1 inhibit swing for leg in front")
        rules_msg = rules(-0.08, 0.0, 0.0, 0.0, 0.0)
        # rules_msg.rule1 = -0.024
        # rules_msg.rule2_ipsilateral = 0.0
        # rules_msg.rule2_contralateral = 0.0
        # rules_msg.rule3_ipsilateral = 0.0
        # rules_msg.rule3_contralateral = 0.0
        self.pub_rules(rules_msg)
        self.temp.move_to_next_point(1)
        self.rate.sleep()
        if self.leg.predicted_ground_contact():
            self.temp.move_to_next_point(0)
            self.temp.swing_start_point = None
            # self.rate.sleep()
            self.swing = False
            self.last_stance_activation = rospy.Time.now()
            rospy.loginfo(self.name + 'swing is finished switch to stance.')

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
        if rospy.is_shutdown():
            return
        else:
            if p is None:
                p = self.init_pos
                rospy.loginfo(self.name + ": move_leg_to p = " + str(p) + " = init_pos = " + str(self.init_pos))
            else:
                rospy.loginfo(self.name + ": move_leg_to p = " + str(p))
            if self.init_pos[0] != p[0] or self.init_pos[1] != p[1] or self.init_pos[2] != p[2]:
                rospy.logerr("move leg to " + str(p) + " but init pose is set to " + str(self.init_pos))
            angles = self.leg.compute_inverse_kinematics(p)
            rospy.loginfo(
                    self.name + ": move_leg_to inverse kinematic. for position " + str(p) + " angles = " + str(angles))
            self.leg.set_command(angles)


if __name__ == '__main__':
    nh = rospy.init_node('single_leg_controller', anonymous=True)
    legController = SingleLegController('lm', nh, True, None)
    try:
        # legController.manage_walk()
        legController.bezier_swing()
    except rospy.ROSInterruptException:
        pass
